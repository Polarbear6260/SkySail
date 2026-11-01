#!/usr/bin/env python3
"""X650 OFFBOARD 控制节点（uXRCE-DDS + px4_msgs）

流程：
  1. 20 Hz 持续发 OffboardControlMode + TrajectorySetpoint（PX4 要求先进流才允许 OFFBOARD）
  2. 收到本地位置后发解锁命令（普通失败则 force-arm 兜底，仅仿真）
  3. 解锁后请求 OFFBOARD（custom_mode=6）
  4. 起飞到 --alt，再依次飞航点，最后悬停
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

CMD_ARM_DISARM = 400
CMD_DO_SET_MODE = 176
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


class OffboardHover(Node):
    def __init__(self) -> None:
        super().__init__("x650_offboard")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.on_pos, qos)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.on_status, qos)

        self.declare_parameter("alt", 2.5)
        self.declare_parameter("waypoints", "")
        self.alt = float(self.get_parameter("alt").value)
        raw = str(self.get_parameter("waypoints").value)
        v = [float(x) for x in raw.replace("[", " ").replace("]", " ").replace(",", " ").split()]
        self.waypoints = [tuple(v[i:i + 3]) for i in range(0, len(v) - 2, 3)]
        self.pos = None
        self.armed = False
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.arm_at = 0.0
        self.offboard_req = False
        self.wp = -1
        self.create_timer(0.05, self.loop)
        self.get_logger().info(f"alt={self.alt} m, waypoints={len(self.waypoints)}")

    def on_pos(self, m): self.pos = m
    def on_status(self, m):
        self.armed = (m.arming_state == 2)
        if getattr(self, "_nav", None) != m.nav_state:
            self._nav = m.nav_state
            self.get_logger().info(f"nav_state={m.nav_state} (14=OFFBOARD) armed={self.armed}")

    def now(self): return self.get_clock().now().nanoseconds / 1e9 - self.t0

    def send_cmd(self, cmd, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command = cmd; m.param1 = float(p1); m.param2 = float(p2)
        m.target_system = 1; m.target_component = 1
        m.source_system = 1; m.source_component = 1
        m.from_external = True
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(m)

    def loop(self):
        t = self.now()
        x = float(self.pos.x) if self.pos else 0.0
        y = float(self.pos.y) if self.pos else 0.0
        z = -self.alt
        if self.pos and 0 <= self.wp < len(self.waypoints):
            wx, wy, wz = self.waypoints[self.wp]
            x, y, z = wx, wy, -wz
            if (self.pos.x - wx) ** 2 + (self.pos.y - wy) ** 2 < 0.25:
                self.get_logger().info(f"到达航点 {self.wp + 1}/{len(self.waypoints)}")
                self.wp += 1

        mode = OffboardControlMode(); mode.position = True
        mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint(); sp.position = [x, y, z]; sp.yaw = 0.0
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_sp.publish(sp)

        if not self.armed:
            if t > 1.5 and (self.arm_at == 0.0 or t - self.arm_at > 5.0):
                self.arm_at = t
                force = 21196.0 if t > 8.0 else 0.0
                self.send_cmd(CMD_ARM_DISARM, 1.0, force)
                self.get_logger().info("force-arm" if force else "发送解锁命令")
            return
        if int(t * 2) % 10 == 0 and self.pos is not None:
            self.get_logger().info(f"alt={-self.pos.z:.2f} m, wp={self.wp}", throttle_duration_sec=4.0)
        if not self.offboard_req and t > 2.0:
            self.send_cmd(CMD_DO_SET_MODE, 1.0, PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            self.offboard_req = True
            self.get_logger().info("请求 OFFBOARD")
        if self.wp == -1 and self.pos and -self.pos.z > self.alt - 0.3:
            self.wp = 0
            self.get_logger().info(f"到达 {-self.pos.z:.2f} m，开始航点")


def main():
    rclpy.init()
    node = OffboardHover()
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
