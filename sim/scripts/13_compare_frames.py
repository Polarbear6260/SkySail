#!/usr/bin/env python3
"""13 · 同一时刻并排打印 PX4 本地位置 与 Gazebo 真值

目的：钉死两套坐标系到底差在哪（原点？轴序？还是读错实体）。
输出每 0.5 s 一行：PX4 的 (x,y,z) 与 gz 里 x650_0 的 (x,y,z)，以及差值。

用法：python3 scripts/13_compare_frames.py --world x650_sea --duration 30
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import gz.msgs10.pose_v_pb2 as pose_v_pb2
import gz.transport13 as gz_transport
import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def fmu_qos() -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


class Comparator(Node):
    def __init__(self) -> None:
        super().__init__("frame_compare")
        self.px4 = None
        self.gz = None
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
                                 self.on_px4, fmu_qos())
        self.gz_node = gz_transport.Node()
        world = self.declare_parameter("world", "x650_sea").value
        self.gz_node.subscribe(pose_v_pb2.Pose_V, f"/world/{world}/pose/info", self.on_gz)

    def on_px4(self, m: VehicleLocalPosition) -> None:
        self.px4 = (m.x, m.y, m.z, m.heading, m.xy_valid, m.z_valid)

    def on_gz(self, msg) -> None:
        for p in msg.pose:
            if p.name == "x650_0":
                self.gz = (p.position.x, p.position.y, p.position.z)
                return


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="x650_sea")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--entity", default="x650_0")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Comparator()
    node.set_parameters([rclpy.parameter.Parameter("world", value=args.world)])
    t0 = time.time()
    print(" t(s) │      PX4 本地 (x, y, z)      │      Gazebo 真值 (x, y, z)    │   差值 (dx,dy,dz)")
    print("──────┼──────────────────────────────┼───────────────────────────────┼──────────────────────")
    last = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            t = time.time() - t0
            if t > args.duration:
                break
            if node.px4 and node.gz and t - last >= 0.5:
                last = t
                px, py, pz, heading, xy_ok, z_ok = node.px4
                gx, gy, gz = node.gz
                print(f"{t:6.1f} │ ({px:+7.2f}, {py:+7.2f}, {pz:+7.2f}) │ "
                      f"({gx:+7.2f}, {gy:+7.2f}, {gz:+7.2f}) │ "
                      f"({px-gx:+6.2f},{py-gy:+6.2f},{pz-gz:+6.2f})  "
                      f"xy_valid={xy_ok} z_valid={z_ok}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
