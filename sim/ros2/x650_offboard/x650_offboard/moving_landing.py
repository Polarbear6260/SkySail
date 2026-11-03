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

import json
import os
import sys

import math

import numpy as np
import rclpy

try:                                     # Gazebo 真值（用于着陆判定）
    import gz.msgs10.pose_v_pb2 as pose_v_pb2
    import gz.transport13 as gz_transport
    HAS_GZ = True
except Exception:                        # noqa: BLE001
    HAS_GZ = False
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32, String
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLandDetected,
                          VehicleLocalPosition, VehicleStatus)

CMD_ARM_DISARM = 400
CMD_DO_SET_MODE = 176
OFFBOARD = 6
AUTO_LAND_MODE = (4 << 16) | 6          # AUTO.LAND 的 custom_mode


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
        # 平台位姿话题（默认真值；接感知仿真器时指向 /ship/pad_pose_d）
        self.declare_parameter("pad_pose_topic", "/ship/pad_pose")
        self.declare_parameter("pad_twist_topic", "/ship/pad_twist")
        self.declare_parameter("gps_pose_topic", "")          # 空=不用 GPS 粗定位
        self.declare_parameter("handover_range", 10.0)        # 交接距离 m
        q = fmu_qos()
        self.pub_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", q)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", q)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", q)
        self.pub_dbg = self.create_publisher(String, "/x650/debug", 10)
        self.create_subscription(Float32, "/aruco/range", self.on_range, 10)
        self.vision_range = None
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.on_pos, q)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.on_status, q)
        for topic in ("/fmu/out/vehicle_land_detected", "/fmu/out/vehicle_land_detected_v1"):
            self.create_subscription(VehicleLandDetected, topic, self.on_land, q)
        pose_topic = str(self.get_parameter("pad_pose_topic").value)
        twist_topic = str(self.get_parameter("pad_twist_topic").value)
        self.create_subscription(PoseStamped, pose_topic, self.on_pad_pose, 10)
        gps_topic = str(self.get_parameter("gps_pose_topic").value)
        if gps_topic:
            self.create_subscription(PoseStamped, gps_topic, self.on_gps_pose, 10)
            self.get_logger().info(f"远距离段使用 GPS 级粗定位：{gps_topic}")
        self.create_subscription(TwistStamped, twist_topic, self.on_pad_twist, 10)

        self.declare_parameter("alt", 2.5)
        self.declare_parameter("approach_clearance", 0.5)   # 进场时板面上方高度
        self.declare_parameter("gz_world", "x650_sea")
        self.declare_parameter("origin_enu", "0,16,1.02")   # 起飞点世界坐标 = PX4 本地原点
        self.declare_parameter("cruise_alt", 10.0)        # 巡航高度（相对落点板）
        self.declare_parameter("local_z_offset", 0.0)   # PX4 本地原点=GPS 世界原点，无需偏移   # 出生点世界高度 = EKF 原点
        self.declare_parameter("cruise_hold_s", 1.5)   # 巡航高度保持多久再下降
        self.declare_parameter("lead_time", 0.5)         # 转场时朝船的“未来位置”飞
        self.declare_parameter("vel_gate", 0.40)        # 横向相对速度门限，满足才下降
        self.declare_parameter("descent_rate", 0.45)   # 落地下降率 m/s（果断）
        self.declare_parameter("settle_s", 2.5)
        self.declare_parameter("skysail_path", os.path.expanduser("~/planning_ws/src/SkySail"))
        self.alt = float(self.get_parameter("alt").value)
        self.clearance = float(self.get_parameter("approach_clearance").value)
        self.settle_s = float(self.get_parameter("settle_s").value)
        self.descent_rate = float(self.get_parameter("descent_rate").value)
        self.lead_time = float(self.get_parameter("lead_time").value)
        self.vel_gate = float(self.get_parameter("vel_gate").value)
        self.z_offset = float(self.get_parameter("local_z_offset").value)
        self.cruise_alt = float(self.get_parameter("cruise_alt").value)
        ox, oy, oz = (float(v) for v in str(self.get_parameter("origin_enu").value).split(","))
        self.origin_enu = np.array([ox, oy, oz])
        self.cruise_hold_s = float(self.get_parameter("cruise_hold_s").value)

        self.ctrl = self._build_controller()

        self.pos_ned: np.ndarray | None = None
        self.pos_valid = False
        self.sink_alt = None
        self.floor_since = 0.0
        self.aligned = False
        self.vel_ned: np.ndarray | None = None
        self.pad_ned: np.ndarray | None = None
        self.pad_ned_fine: np.ndarray | None = None
        self.pad_ned_gps: np.ndarray | None = None
        self._pad_msgs = 0
        self._pad_seen = -1
        self.pad_vel_ned = np.zeros(3)
        self.armed = False
        self.nav_state = 0
        self.landed = False
        self.finished = False
        self.alt_hist = []
        self.true_horiz = float("inf")
        self.true_above = float("inf")
        self.cruise_t = 0.0
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.state = "wait"
        self.arm_sent = 0.0
        self.offboard_req = False
        self.settle_start = 0.0
        self.land_retry = 0
        self.last_rebuild = 0.0
        self.last_acc = (0.0, 0.0, 0.0)
        self.land_err = float("nan")

        if HAS_GZ:
            world = str(self.get_parameter("gz_world").value)
            self._gz = gz_transport.Node()
            self._gz.subscribe(pose_v_pb2.Pose_V, f"/world/{world}/pose/info", self.on_gz_pose)
            self.get_logger().info("已接入 Gazebo 真值（着陆判定用真值，控制仍用 PX4）")
        else:
            self.get_logger().warn("没有 gz 绑定，着陆判定退回 PX4 估计（会有偏差）")

        self.create_timer(0.05, self.loop)
        self.get_logger().info(f"目标：起飞 {self.alt:.1f} m → 追移动落点板（上方 {self.clearance:.1f} m 进场）")

    def _build_controller(self):
        path = str(self.get_parameter("skysail_path").value)
        if path not in sys.path:
            sys.path.insert(0, path)
        from UAV_Control import AugmentedLQRController          # noqa: PLC0415
        from UAV_Control.mionet.reference_controller import MIONetLandingController  # noqa: PLC0415
        from UAV_Control.UAV_params import LQRParams, UAVParams  # noqa: PLC0415
        from x650_offboard.predictors import ConstantTurnPredictor, PadHistory  # noqa: PLC0415

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
        # 打开曲率预览前馈（上游默认 curvature_weight=1e-3）
        lqr = LQRParams(position_weight=(16.0, 16.0, 25.0),
                        velocity_weight=(4.0, 4.0, 9.0),
                        control_weight=(1.0, 1.0, 1.5),
                        preview_horizon=10, curvature_weight=1e-3,
                        tracking_velocity_scale=4.0)
        self.uav_params = uav
        self.lqr_params = lqr
        self.history = PadHistory()
        self.predictor = ConstantTurnPredictor(horizon_s=1.0, preview_horizon=lqr.preview_horizon)
        aug = AugmentedLQRController(uav=uav, lqr=lqr, reference_speed_m_s=1.0)
        self.get_logger().info(
            "已载入 SkySail LQR + MIONet 控制器结构（预测器=恒速恒转向，preview 前馈已启用）")
        return MIONetLandingController(predictor=self.predictor, controller=aug)

    def _update_gains(self) -> None:
        """平台速度/方向变化较大时，用新的参考速度与横向方向重建 LQR 增益。"""
        t = self.now()
        if t - self.last_rebuild < 1.0 or self.pad_vel_ned is None:
            return
        self.last_rebuild = t
        speed = float(np.hypot(self.pad_vel_ned[0], self.pad_vel_ned[1]))
        lateral = np.array([-self.pad_vel_ned[1], self.pad_vel_ned[0], 0.0])
        norm = np.linalg.norm(lateral)
        lateral = lateral / norm if norm > 1e-3 else np.array([0.0, 1.0, 0.0])
        try:
            from UAV_Control import AugmentedLQRController  # noqa: PLC0415
            from UAV_Control.mionet.reference_controller import MIONetLandingController  # noqa: PLC0415
            aug = AugmentedLQRController(uav=self.uav_params, lqr=self.lqr_params,
                                         reference_speed_m_s=max(speed, 0.3),
                                         lateral_direction=tuple(lateral))
            self.ctrl = MIONetLandingController(predictor=self.predictor, controller=aug)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"重建增益失败：{exc}")

    # ---------- 订阅 ----------
    def on_pos(self, m: VehicleLocalPosition) -> None:
        self.pos_ned = np.array([m.x, m.y, m.z])
        self.pos_valid = bool(m.xy_valid and m.z_valid)
        self.vel_ned = np.array([m.vx, m.vy, m.vz])

    def on_status(self, m: VehicleStatus) -> None:
        self.armed = (m.arming_state == 2)
        self.nav_state = int(m.nav_state)

    def on_gps_pose(self, m: PoseStamped) -> None:
        world_enu = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        self.pad_ned_gps = enu_to_ned(world_enu - self.origin_enu)
        self._pad_msgs += 1

    def _blend_pad(self) -> None:
        """按距离在"GPS 粗定位"和"精定位"之间平滑加权，避免交接跳变。"""
        fine, gps = self.pad_ned_fine, self.pad_ned_gps
        # 视觉更新可能断续：超过 0.4 s 没更新就当作不可用，避免追一个"冻结"的目标
        if fine is not None and (self.now() - getattr(self, "_fine_t", 0.0)) > 0.4:
            fine = None
        if fine is None and gps is None:
            return
        if fine is None:
            self.pad_ned = gps
        elif gps is None or self.pos_ned is None:
            self.pad_ned = fine
        else:
            d = float(np.hypot(self.pos_ned[0] - fine[0], self.pos_ned[1] - fine[1]))
            rng = float(self.get_parameter("handover_range").value)
            w = float(np.clip((rng - d) / (rng * 0.5), 0.0, 1.0))   # 近→1，远→0
            self.pad_ned = (1.0 - w) * gps + w * fine
        # 只在新测量到达时记录，避免用重复值污染速度估计（GPS 只有 1 Hz）
        # 注意：yaw 可能缺失（只有 GPS 时没有姿态），此时用 0 兜底——
        # 早期版本因为这里要求 yaw 存在，导致历史永远为空、预测器一直输出原点
        if self._pad_msgs != self._pad_seen:
            self._pad_seen = self._pad_msgs
            yaw_enu = getattr(self, "_yaw_enu", None)
            self.history.push(self.now(), self.pad_ned, -(yaw_enu if yaw_enu is not None else 0.0))

    def on_range(self, m: Float32) -> None:
        self.vision_range = float(m.data)

    def on_land(self, m: VehicleLandDetected) -> None:
        self.landed = bool(m.landed)

    def on_pad_pose(self, m: PoseStamped) -> None:
        # 世界 ENU − 起飞点（PX4 本地原点）→ NED，得到 PX4 本地系坐标
        world_enu = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        new_fine = enu_to_ned(world_enu - self.origin_enu)
        if self.pad_ned_fine is None:
            self.pad_ned_fine = new_fine
        else:
            self.pad_ned_fine = 0.6 * new_fine + 0.4 * self.pad_ned_fine   # EMA，抑制抖动
        self._fine_t = self.now()
        self._pad_msgs += 1
        self._yaw_enu = math.atan2(2.0 * (m.pose.orientation.w * m.pose.orientation.z
                                         + m.pose.orientation.x * m.pose.orientation.y),
                                   1.0 - 2.0 * (m.pose.orientation.y ** 2 + m.pose.orientation.z ** 2))

    def on_gz_pose(self, msg) -> None:
        """从 Gazebo 读真值，算机体相对落点板的真实水平距离与高度。"""
        drone = ship = None
        for p in msg.pose:
            if p.name == "x650_0":
                drone = p.position
            elif p.name == "ship":
                ship = p
        if drone is None or ship is None:
            return
        q = ship.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        px = ship.position.x + 0.30 * math.cos(yaw)
        py = ship.position.y + 0.30 * math.sin(yaw)
        pz = ship.position.z + 0.45
        self.true_horiz = math.hypot(drone.x - px, drone.y - py)
        self.true_above = drone.z - pz

    def on_pad_twist(self, m: TwistStamped) -> None:
        self.pad_vel_ned = enu_to_ned(np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]))

    # ---------- 发布 ----------
    def publish_debug(self) -> None:
        ref = getattr(self.predictor, "last_reference", None)
        dbg = {
            "t": self.now(),
            "state": self.state,
            "armed": bool(self.armed),
            "nav_state": int(self.nav_state),
            "horizon": float(getattr(self.predictor, "horizon_s", 0.0)),
            "descent_rate": float(getattr(self.predictor, "descent_rate", 0.0)),
            "true_horiz": None if self.true_horiz == float("inf") else float(self.true_horiz),
            "true_above": None if self.true_above == float("inf") else float(self.true_above),
            "acc": list(self.last_acc),
            "ref_pos": None if ref is None else list(ref.position),
            "ref_vel": None if ref is None else list(ref.velocity),
            "pad_ned": None if self.pad_ned is None else [float(v) for v in self.pad_ned],
            "pad_vel": [float(v) for v in self.pad_vel_ned],
            "pos_ned": None if self.pos_ned is None else [float(v) for v in self.pos_ned],
            # 直接给出世界 ENU 坐标，方便调试面板与船位姿画在同一坐标系
            "pos_enu": None if self.pos_ned is None else [
                float(self.origin_enu[0] + self.pos_ned[1]),
                float(self.origin_enu[1] + self.pos_ned[0]),
                float(self.origin_enu[2] - self.pos_ned[2])],
        }
        msg = String()
        msg.data = json.dumps(dbg)
        self.pub_dbg.publish(msg)

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
        self.last_acc = tuple(float(v) for v in acc)
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
        """每个周期都无条件发布设定点（否则 PX4 会判 offboard 丢失 → RTL）。"""
        t = self.now()

        if t - getattr(self, "_last_dbg", 0.0) > 0.1:
            self._last_dbg = t
            self.publish_debug()

        # 0) 已着陆上锁：彻底停机，不再发指令（否则会被重新解锁）
        if self.finished:
            return

        # 1) 未解锁：发位置设定点并请求解锁
        if not self.armed:
            self.publish_position(np.array([0.0, 0.0, -self.alt]))
            # EKF 未就绪时不要解锁：否则位置估计是垃圾（曾出现 alt=-500 m）
            if not self.pos_valid:
                if int(t) % 5 == 0:
                    self.get_logger().info("等待 EKF 位置有效（xy_valid && z_valid）…")
                return
            if t > 1.5 and (self.arm_sent == 0.0 or t - self.arm_sent > 5.0):
                self.arm_sent = t
                self.send_cmd(CMD_ARM_DISARM, 1.0, 21196.0 if t > 8.0 else 0.0)
            return

        # 2) 解锁后请求 OFFBOARD（本周期仍然发布，避免任意一次空档）
        if not self.offboard_req:
            self.publish_position(np.array([0.0, 0.0, -self.alt]))
            if t > 2.0:
                self.send_cmd(CMD_DO_SET_MODE, 1.0, OFFBOARD)
                self.offboard_req = True
                self.get_logger().info("请求 OFFBOARD")
            return

        self._blend_pad()
        if self.pos_ned is None or self.pad_ned is None:
            self.publish_position(np.array([0.0, 0.0, -self.alt]))
            return

        alt = -self.pos_ned[2]
        pad_alt = -self.pad_ned[2]
        rel = self.pos_ned - self.pad_ned
        rel_h = float(np.hypot(rel[0], rel[1]))
        rel_v = float(np.linalg.norm(self.vel_ned - self.pad_vel_ned))
        rel_v_h = float(np.hypot(self.vel_ned[0] - self.pad_vel_ned[0],
                                 self.vel_ned[1] - self.pad_vel_ned[1]))

        if int(t * 2) % 4 == 0:
            ref = getattr(self.predictor, "last_reference", None)
            if ref is not None:
                detail = (f"pos=({self.pos_ned[0]:+.1f},{self.pos_ned[1]:+.1f}) "
                          f"pad=({self.pad_ned[0]:+.1f},{self.pad_ned[1]:+.1f}) "
                          f"ref=({ref.position[0]:+.1f},{ref.position[1]:+.1f},{ref.position[2]:+.2f}) "
                          f"rvz={ref.velocity[2]:+.2f}")
            else:
                detail = "ref=n/a"
            self.get_logger().info(
                f"state={self.state} alt={alt:.2f} rel_h={rel_h:.2f} "
                f"真值(水平={self.true_horiz:.2f} 高于板面={self.true_above:+.2f}) "
                f"src(fine={'Y' if self.pad_ned_fine is not None else 'N'},"
                f"gps={'Y' if self.pad_ned_gps is not None else 'N'}) {detail} "
                f"acc=({self.last_acc[0]:+.2f},{self.last_acc[1]:+.2f},{self.last_acc[2]:+.2f}) "
                f"vrange={self.vision_range if self.vision_range is not None else -1:.2f} armed={self.armed}",
                throttle_duration_sec=2.0)

        # 3) 起飞段：先爬到巡航高度
        if self.state in ("wait", "takeoff"):
            if self.state == "wait":
                self.state = "takeoff"
            self.publish_position(np.array([0.0, 0.0, -self.cruise_alt]))
            if alt > self.cruise_alt - 0.4:
                self.state = "transit"
                self.get_logger().info(f"到达巡航高度 {alt:.2f} m，转场飞向落点板")
            return

        # 3b) 转场：在巡航高度飞向落点板正上方
        if self.state == "transit":
            self.predictor.horizon_s = 0.8
            self.predictor.z_offset = -self.cruise_alt        # 平台上空
            lead = self.pad_vel_ned * self.lead_time
            target = self.pad_ned + lead + np.array([0.0, 0.0, -self.cruise_alt])
            self._update_gains()
            acc = self.ctrl.compute_control(self.pos_ned, self.vel_ned, self.history, "tracking")
            self.publish_accel(acc)
            # 用“到提前量目标”的水平距离判定，而不是到板本身
            d_target = float(np.hypot(self.pos_ned[0] - target[0], self.pos_ned[1] - target[1]))
            if d_target < 2.5 and rel_h < 3.0:
                if self.cruise_t == 0.0:
                    self.cruise_t = t
                    self.get_logger().info(
                        f"已到板上方 {alt:.1f} m（水平 {rel_h:.2f} m），保持 {self.cruise_hold_s:.0f} s")
                if rel_h > 5.0:                      # 明显跑偏才重新计时
                    self.cruise_t = 0.0
                elif t - self.cruise_t >= self.cruise_hold_s:
                    self.state = "approach"
                    self.get_logger().info(f"开始下降（自 {alt:.1f} m）")
            return

        # 4) 进场：从巡航高度下降，目标 = 板面
        if self.state == "approach":
            self.predictor.horizon_s = 0.3
            self.predictor.descent_rate = 0.0
            # 对准后才下降，并带滞回：小于 0.8 m 置位，大于 3 m 复位，避免在阈值上反复升降
            if rel_h < 0.8:
                self.aligned = True
            elif rel_h > 3.0:
                self.aligned = False
            self.predictor.z_offset = -self.clearance if self.aligned else -self.cruise_alt
            target = self.pad_ned + np.array([0.0, 0.0, 0.02])
            self._update_gains()
            acc = self.ctrl.compute_control(self.pos_ned, self.vel_ned, self.history, "tracking")
            self.publish_accel(acc)
            if (rel_h < 1.0 and self.true_above < self.clearance and rel_v_h < self.vel_gate):
                self.state = "descend"
                self.get_logger().info(f"降到板面上方 {alt - pad_alt:.2f} m（水平 {rel_h:.2f} m），精降")
            return

        # 5) 下降：横向用视觉对准，纵向按固定下降率强制下沉
        #    飞控的 z 估计会漂（实测差 2 m），所以不拿它当绝对高度用；
        #    触地判定改用视觉给出的"相机到板距离"
        if self.state == "descend":
            if not hasattr(self, "sink_alt") or self.sink_alt is None:
                self.sink_alt = alt
            floor_alt = -float(self.pad_ned[2]) - 0.05    # 板面高度（PX4 本体系）再往下 5 cm
            if self.sink_alt > floor_alt:
                self.sink_alt = max(floor_alt, self.sink_alt - 0.25 * 0.05)
            self.publish_position(np.array([self.pad_ned[0], self.pad_ned[1], -self.sink_alt]))
            # 到达下沉下限即认为已在甲板上（近距离标记占满画面，视觉无法再用）
            at_floor = self.sink_alt <= floor_alt + 1e-3
            if at_floor and self.floor_since == 0.0:
                self.floor_since = t
            elif not at_floor:
                self.floor_since = 0.0
            # 只有当视觉距离可信且确实很近时才允许上锁（防止板估计偏差导致提前上锁）
            nearby = self.vision_range is not None and self.vision_range < 1.0
            if (self.floor_since and t - self.floor_since > 0.5 and nearby) or \
                    (self.vision_range is not None and self.vision_range < 0.42):
                self.state = "landed"
                self.finished = True
                self.send_cmd(CMD_ARM_DISARM, 0.0, 21196.0)
                self.get_logger().info(
                    f"视觉触地（相机到板 {self.vision_range:.2f} m，"
                    f"真值水平 {self.true_horiz:.2f} m）→ 上锁")
            return

        # 5b) 旧加速度下降（保留备用）
        if False:
            target = self.pad_ned + np.array([0.0, 0.0, 0.02])
            ref_vel = self.pad_vel_ned + np.array([0.0, 0.0, self.descent_rate])
            self._update_gains()
            acc = self.ctrl.compute_control(self.pos_ned, self.vel_ned, self.history, "touchdown")
            self.publish_accel(acc)
            if rel_h > 1.0:                     # 只有明显偏出才回到进场，避免抖动
                self.state = "approach"
                self.get_logger().info(f"下降中横向偏出 {rel_h:.2f} m，回到进场段重新对准")
                return
            if (alt - pad_alt) < 0.18 and rel_v < 0.3:
                self.state = "settle"
                self.settle_start = t
                self.land_err = rel_h
                self.get_logger().info(
                    f"接触（相对高度 {alt - pad_alt:.2f} m，相对速度 {rel_v:.2f} m/s，"
                    f"水平误差 {rel_h:.2f} m）")
            return

        # 5b) 贴板压紧：位置设定点压到板面以下，逼出真实接触
        if self.state == "settle":
            # 加速度控制 + 甲板速度前馈：位置环追移动目标有稳态滞后，会滑到板边
            target = self.pad_ned + np.array([0.0, 0.0, -0.02])   # 板面之上 2 cm
            ref_vel = self.pad_vel_ned                              # 不再压下去
            self.predictor.descent_rate = 0.15
            self.predictor.z_offset = -0.03
            self.predictor.horizon_s = 0.0
            self._update_gains()
            acc = self.ctrl.compute_control(self.pos_ned, self.vel_ned, self.history, "touchdown")
            self.publish_accel(acc)
            # 船在升沉，PX4 的落地检测永远不会判 True（世界系里一直在动），
            # 所以自己判：相对板面高度稳定 + 相对速度小 → 认为已坐实
            # 触地判据用“高度停滞”而不是绝对高度差：
            # PX4 的 alt 是相对出生点，落点板坐标是世界系，两者恒差一个量，
            # 用差值判断永远对不上；改用“命令持续下压但高度不再下降”来判定接触。
            # 简单果断：压紧后水平在 0.35 m 内持续 3 s，就认为已落稳 → 上锁
            rel_h_now = alt - pad_alt
            # 对准且贴到板面上方 12 cm 以内 → 交给 PX4 的 AUTO.LAND：
            # 飞控自己收油门、自己判落地、自己上锁（不是我们硬停）
            # 成功判据（按约定）：真值确认"落在甲板上"一次即可
            # 成功判据必须带上下界：只判"低于板面"会把"穿过甲板掉下去"也算成功
            if (self.true_horiz < 0.30 and -0.10 < self.true_above < 0.10
                    and rel_v < 0.40):
                if self.settle_start == 0.0:
                    self.settle_start = t
                elif t - self.settle_start > 0.3:
                    self.state = "landed"
                    self.finished = True
                    self.get_logger().info(
                        f"TOUCHDOWN 成功：高于板面 {self.true_above:+.2f} m，"
                        f"水平误差 {self.true_horiz:.2f} m，时刻 t={t:.1f}s")
                    rclpy.shutdown()          # 触板即结束，不再空跑
                    return
            else:
                self.settle_start = 0.0
            return

        # 5c) AUTO.LAND：不发设定点，等飞控自己落地并上锁
        if self.state == "auto_land":
            # 继续发设定点：AUTO.LAND 下 PX4 忽略它们，但万一切模式没生效，
            # 也不至于 offboard 断流触发 RTL（刚才就是这么飞回去的）
            self.publish_position(self.pad_ned + np.array([0.0, 0.0, -0.02]))
            if t - self.settle_start > 2.0 and self.land_retry < 3:
                self.land_retry += 1
                self.settle_start = t
                self.send_cmd(CMD_DO_SET_MODE, 1.0, AUTO_LAND_MODE)
                self.get_logger().info(f"重试 AUTO.LAND（当前 nav_state={self.nav_state}）")
            if not self.armed:
                self.state = "landed"
                self.finished = True
                self.get_logger().info("飞控已判定落地并自动上锁（桨停）")
            elif int(t) % 4 == 0:
                self.get_logger().info(f"AUTO.LAND 中：nav_state={self.nav_state} 高度 {alt:.2f} m")
            return

        # 5d) 已上锁：不再发送任何会触发失效保护的东西，等 demo 结束
        if self.state in ("touchdown", "landed"):
            self.publish_position(self.pad_ned + np.array([0.0, 0.0, 0.05]))
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
