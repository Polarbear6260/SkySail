#!/usr/bin/env python3
"""X650 移动平台着陆 demo

外环：SkySail LQR（参考速度非零）→ 加速度指令 → PX4 OFFBOARD
参考：/ship/pad_pose + /ship/pad_twist（船驱动脚本发布，ENU）

阶段：
  1. takeoff   起飞到 2.5 m（PX4 位置环）
  2. approach  LQR 追落点板（板面上方 1.0 m）
  3. descend   LQR 沿板面下降
  4. settle    贴在板面上 2.5 s（跟踪移动的板）
  5. 上锁收尾

用法：
  ros2 run x650_offboard moving_landing --ros-args -p alt:=2.5
"""

from __future__ import annotations

import os
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

CMD_ARM_DISARM = 400
CMD_DO_SET_MODE = 176
OFFBOARD = 6


def fmu_qos() -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


def enu_to_ned(v: np.ndarray) -> np.ndarray:
    """ENU(x东,y北,z上) → NED(x北,y东,z下)。"""
    return np.array([v[1], v[0], -v[2]])


class MovingLanding(Node):
    def __init__(self) -> None:
        super().__init__("x650_moving_landing")
        q = fmu_qos()
        self.pub_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", q)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", q)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", q)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.on_pos, q)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.on_status, q)
        self.create_subscription(PoseStamped, "/ship/pad_pose", self.on_pad_pose, 10)
        self.create_subscription(TwistStamped, "/ship/pad_twist", self.on_pad_twist, 10)

        self.declare_parameter("alt", 2.5)
        self.declare_parameter("approach_clearance", 1.0)   # 进场时板面上方高度
        self.declare_parameter("settle_s", 2.5)
        self.declare_parameter("skysail_path", os.path.expanduser("~/planning_ws/src/SkySail"))
        self.alt = float(self.get_parameter("alt").value)
        self.clearance = float(self.get_parameter("approach_clearance").value)
        self.settle_s = float(self.get_parameter("settle_s").value)

        self.ctrl = self._build_controller()

        self.pos_ned: np.ndarray | None = None
        self.vel_ned: np.ndarray | None = None
        self.pad_ned: np.ndarray | None = None
        self.pad_vel_ned = np.zeros(3)
        self.armed = False
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.state = "wait"
        self.arm_sent = 0.0
        self.offboard_req = False
        self.settle_start = 0.0
        self.land_err = float("nan")

        self.create_timer(0.05, self.loop)
        self.get_logger().info(f"目标：起飞 {self.alt:.1f} m → 追移动落点板（上方 {self.clearance:.1f} m 进场）")

    def _build_controller(self):
        path = str(self.get_parameter("skysail_path").value)
        if path not in sys.path:
            sys.path.insert(0, path)
        from UAV_Control import AugmentedLQRController          # noqa: PLC0415
        from UAV_Control.UAV_params import LQRParams, UAVParams  # noqa: PLC0415

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
        self.get_logger().info("已载入 SkySail LQR（X650 参数，参考速度非零）")
        return AugmentedLQRController(uav=uav, lqr=lqr, reference_speed_m_s=1.0)

    # ---------- 订阅 ----------
    def on_pos(self, m: VehicleLocalPosition) -> None:
        self.pos_ned = np.array([m.x, m.y, m.z])
        self.vel_ned = np.array([m.vx, m.vy, m.vz])

    def on_status(self, m: VehicleStatus) -> None:
        self.armed = (m.arming_state == 2)

    def on_pad_pose(self, m: PoseStamped) -> None:
        self.pad_ned = enu_to_ned(np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]))

    def on_pad_twist(self, m: TwistStamped) -> None:
        self.pad_vel_ned = enu_to_ned(np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]))

    # ---------- 发布 ----------
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

    def publish_position(self, xyz: np.ndarray) -> None:
        mode = OffboardControlMode()
        mode.position = True
        mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint()
        sp.position = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
        sp.yaw = 0.0
        sp.timestamp = mode.timestamp
        self.pub_sp.publish(sp)

    def publish_accel(self, acc: np.ndarray) -> None:
        mode = OffboardControlMode()
        mode.acceleration = True
        mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint()
        sp.position = [float("nan")] * 3
        sp.velocity = [float("nan")] * 3
        sp.acceleration = [float(acc[0]), float(acc[1]), float(acc[2])]
        sp.yaw = 0.0
        sp.timestamp = mode.timestamp
        self.pub_sp.publish(sp)

    # ---------- 主循环 ----------
    def loop(self) -> None:
        t = self.now()

        if not self.armed:
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

        # 每 2 s 打一次调试状态，便于定位“卡住”
        if int(t * 2) % 4 == 0:
            self.get_logger().info(
                f"state={self.state} alt={-self.pos_ned[2]:.2f} "
                f"pad={'ok' if self.pad_ned is not None else 'None'} "
                f"armed={self.armed}", throttle_duration_sec=2.0)

        if self.pos_ned is None or self.pad_ned is None:
            return

        alt = -self.pos_ned[2]
        pad_alt = -self.pad_ned[2]

        if self.state in ("wait", "takeoff"):
            if self.state == "wait":
                self.state = "takeoff"
            if alt > self.alt - 0.3:
                self.state = "approach"
                self.get_logger().info(f"到达 {alt:.2f} m，开始追落点板")
            else:
                self.publish_position(np.array([0.0, 0.0, -self.alt]))
                return

        if self.pad_ned is None:
            return

        rel = self.pos_ned - self.pad_ned
        rel_h = float(np.hypot(rel[0], rel[1]))
        rel_v = float(np.linalg.norm(self.vel_ned - self.pad_vel_ned))

        if self.state == "approach":
            target = self.pad_ned + np.array([0.0, 0.0, -self.clearance])
            acc = self.ctrl.compute_control(self.pos_ned, self.vel_ned, target,
                                            self.pad_vel_ned, "tracking",
                                            np.zeros(self.ctrl.lqr.preview_horizon))
            self.publish_accel(acc)
            if rel_h < 0.4 and abs(rel[2] + self.clearance) < 0.5:
                self.state = "descend"
                self.get_logger().info(f"到达板面上方（水平 {rel_h:.2f} m），开始下降")
            return

        if self.state == "descend":
            target = self.pad_ned + np.array([0.0, 0.0, 0.02])
            acc = self.ctrl.compute_control(self.pos_ned, self.vel_ned, target,
                                            self.pad_vel_ned, "tracking",
                                            np.zeros(self.ctrl.lqr.preview_horizon))
            self.publish_accel(acc)
            if (alt - pad_alt) < 0.18 and rel_v < 0.3:
                self.state = "settle"
                self.settle_start = t
                self.land_err = rel_h
                self.get_logger().info(
                    f"接触（相对高度 {alt - pad_alt:.2f} m，相对速度 {rel_v:.2f} m/s，"
                    f"水平误差 {rel_h:.2f} m），贴板 {self.settle_s:.1f} s")
            return

        if self.state == "settle":
            # 跟着移动的板走，让 PX4 位置环把它按在甲板上
            self.publish_position(self.pad_ned + np.array([0.0, 0.0, -0.06]))
            rel_h_now = alt - pad_alt
            if rel_h_now > 0.08 and t - self.settle_start < 6.0:
                return                      # 还没贴到板面，继续压
            if t - self.settle_start > self.settle_s and self.state != "hold":
                # 不立即上锁：甲板仍在移动，上锁后没有系留会滑落海里。
                # 改为继续用位置环把它按在落点板上（等价于“着陆后系留”）。
                self.state = "hold"
                self.get_logger().info(
                    f"已贴板并保持跟踪（水平误差 {self.land_err:.2f} m），demo 结束")
            return


def main() -> None:
    rclpy.init()
    node = MovingLanding()
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
