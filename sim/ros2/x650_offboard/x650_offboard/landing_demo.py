#!/usr/bin/env python3
"""X650 定点着陆 demo：SkySail PointLandingController 外环 + PX4 OFFBOARD

流程：
  1. 起飞到 --alt（位置设定点，PX4 内部位置环）
  2. 切到 SkySail 的 LQR 外环：把 PX4 的位置/速度喂给控制器，输出加速度指令，
     以 TrajectorySetpoint.acceleration 发给 PX4（纯加速度跟踪）
  3. 满足触地条件后切 AUTO.LAND，落地后完成

用法：
  ros2 run x650_offboard landing_demo --ros-args -p alt:=2.5 -p landing_x:=8.0
"""

from __future__ import annotations

import os
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

CMD_ARM_DISARM = 400
CMD_DO_SET_MODE = 176
OFFBOARD = 6
AUTO_LAND_CUSTOM = (4 << 16) | 6     # PX4: main=AUTO(4), sub=LAND(6)


def qos_profile() -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


class LandingDemo(Node):
    def __init__(self) -> None:
        super().__init__("x650_landing")
        q = qos_profile()
        self.pub_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", q)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", q)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", q)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.on_pos, q)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.on_status, q)

        self.declare_parameter("alt", 2.5)
        self.declare_parameter("landing_x", 8.0)
        self.declare_parameter("landing_y", 0.0)
        self.declare_parameter("skysail_path", os.path.expanduser("~/planning_ws/src/SkySail"))
        self.declare_parameter("rate_hz", 20.0)

        self.alt = float(self.get_parameter("alt").value)
        self.pad = np.array([float(self.get_parameter("landing_x").value),
                             float(self.get_parameter("landing_y").value), 0.0])

        self.ctrl = self._build_controller()

        self.pos: np.ndarray | None = None
        self.vel: np.ndarray | None = None
        self.armed = False
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.state = "wait"          # wait → takeoff → approach → land → done
        self.arm_sent = 0.0
        self.offboard_req = False
        self.land_sent = False
        self.traj: list[tuple[float, float, float]] = []

        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self.loop)
        self.get_logger().info(f"目标：起飞 {self.alt:.1f} m → 落点 ({self.pad[0]:.1f}, {self.pad[1]:.1f})")

    def _build_controller(self):
        path = str(self.get_parameter("skysail_path").value)
        if path not in sys.path:
            sys.path.insert(0, path)
        from UAV_Control import AugmentedLQRController, PointLandingController  # noqa: PLC0415
        from UAV_Control.UAV_params import LQRParams, UAVParams                 # noqa: PLC0415

        # X650 参数（BOM：3.5767 kg，Ixx=Iyy=0.1240，Izz=0.2265）
        uav = UAVParams(
            model_name="Holybro X650 (BOM)",
            mass_kg=3.5767,
            inertia_kg_m2=(0.1240, 0.1240, 0.2265),
            gravity_m_s2=9.81,
            dt_s=0.05,
            max_acceleration_m_s2=(3.0, 3.0, 2.5),
            min_acceleration_m_s2=(-3.0, -3.0, -2.5),
            max_tilt_rad=float(np.deg2rad(25.0)),
            max_descent_speed_m_s=1.0,
        )
        lqr = LQRParams(position_weight=(16.0, 16.0, 25.0),
                        velocity_weight=(4.0, 4.0, 9.0),
                        control_weight=(1.0, 1.0, 1.5),
                        preview_horizon=10, curvature_weight=0.0,
                        tracking_velocity_scale=4.0)
        inner = AugmentedLQRController(uav=uav, lqr=lqr, reference_speed_m_s=1.0)
        self.get_logger().info("已载入 SkySail PointLandingController（X650 参数）")
        # 进近阶段保持高度：目标设为 (pad_x, pad_y, -alt)，到位后再垂直下降
        approach = (float(self.pad[0]), float(self.pad[1]), -self.alt)
        return PointLandingController(approach, controller=inner)

    def on_pos(self, m: VehicleLocalPosition) -> None:
        self.pos = np.array([m.x, m.y, m.z])
        self.vel = np.array([m.vx, m.vy, m.vz])
        if self.state in ("approach", "land") and len(self.traj) < 20000:
            self.traj.append((float(m.x), float(m.y), float(-m.z)))

    def on_status(self, m: VehicleStatus) -> None:
        self.armed = (m.arming_state == 2)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9 - self.t0

    def send_cmd(self, cmd: int, p1: float = 0.0, p2: float = 0.0) -> None:
        m = VehicleCommand()
        m.command = cmd
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(m)

    def publish_mode(self, use_position: bool) -> None:
        m = OffboardControlMode()
        m.position = use_position
        m.acceleration = not use_position
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_mode.publish(m)

    def publish_position(self, xyz: np.ndarray) -> None:
        sp = TrajectorySetpoint()
        sp.position = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
        sp.yaw = 0.0
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_sp.publish(sp)

    def publish_acceleration(self, acc: np.ndarray) -> None:
        sp = TrajectorySetpoint()
        sp.position = [float("nan")] * 3
        sp.velocity = [float("nan")] * 3
        sp.acceleration = [float(acc[0]), float(acc[1]), float(acc[2])]
        sp.yaw = 0.0
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_sp.publish(sp)

    def loop(self) -> None:
        t = self.now()

        # ---- 解锁 + OFFBOARD ----
        if not self.armed:
            self.publish_mode(True)
            self.publish_position(np.array([0.0, 0.0, -self.alt]))
            if t > 1.5 and (self.arm_sent == 0.0 or t - self.arm_sent > 5.0):
                self.arm_sent = t
                force = 21196.0 if t > 8.0 else 0.0
                self.send_cmd(CMD_ARM_DISARM, 1.0, force)
            return
        if not self.offboard_req and t > 2.0:
            self.send_cmd(CMD_DO_SET_MODE, 1.0, OFFBOARD)
            self.offboard_req = True
            self.get_logger().info("请求 OFFBOARD")
            return

        if self.pos is None:
            return

        alt = -self.pos[2]

        # ---- 起飞段 ----
        if self.state in ("wait", None) or self.state == "takeoff":
            if self.state == "wait":
                self.state = "takeoff"
            if alt > self.alt - 0.3:
                self.state = "approach"
                self.get_logger().info(f"到达 {alt:.2f} m，交给 SkySail 控制器")
            else:
                self.publish_mode(True)
                self.publish_position(np.array([0.0, 0.0, -self.alt]))
                return

        # ---- 进场：SkySail LQR 加速度控制（保持高度平飞）----
        if self.state == "approach":
            acc = np.asarray(self.ctrl.compute_control(self.pos, self.vel, "tracking"), dtype=float)
            self.publish_mode(False)
            self.publish_acceleration(acc)
            horiz = float(np.hypot(self.pos[0] - self.pad[0], self.pos[1] - self.pad[1]))
            speed = float(np.linalg.norm(self.vel))
            if horiz < 0.6 and speed < 0.5:
                self.state = "descend"
                self.get_logger().info(f"到达落点上方（水平 {horiz:.2f} m），开始垂直下降")
            return

        # ---- 垂直下降 → 触地 ----
        if self.state in ("descend", "land"):
            self.publish_mode(True)
            self.publish_position(np.array([self.pad[0], self.pad[1], -0.12]))
            if not self.land_sent:
                self.land_sent = True
                self.send_cmd(CMD_DO_SET_MODE, 1.0, AUTO_LAND_CUSTOM)
                self.get_logger().info("切 AUTO.LAND")
            alt = -self.pos[2]
            speed = float(np.linalg.norm(self.vel))
            if alt < 0.15 and speed < 0.25:
                self.state = "done"
                self._dump_summary()
                err = self.pos - self.pad
                self.get_logger().info(
                    f"着陆完成：落点误差 {np.linalg.norm(err[:2]):.3f} m，轨迹点 {len(self.traj)}")
            return


    def _dump_summary(self) -> None:
        """落地后把轨迹写成 CSV 并打印统计（给汇报用）。"""
        import csv
        out = os.path.expanduser("~/x650_ws/landing_traj.csv")
        try:
            with open(out, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["x_m", "y_m", "alt_m"])
                w.writerows(self.traj)
        except OSError as exc:
            self.get_logger().warn(f"轨迹写入失败：{exc}")
            return
        if self.traj:
            arr = np.array(self.traj)
            self.get_logger().info(
                f"轨迹：{len(arr)} 点，x∈[{arr[:,0].min():.2f},{arr[:,0].max():.2f}]，"
                f"y∈[{arr[:,1].min():.2f},{arr[:,1].max():.2f}]，"
                f"高度∈[{arr[:,2].min():.2f},{arr[:,2].max():.2f}] m")
        self.get_logger().info(f"轨迹已保存：{out}")


def main() -> None:
    rclpy.init()
    node = LandingDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
