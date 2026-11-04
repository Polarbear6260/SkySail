#!/usr/bin/env python3
"""25 · 检查 PX4 的高度估计与 Gazebo 真值的偏差随时间变化

同时记录：
  · /fmu/out/vehicle_local_position_v1 的 z（飞控估计，相对起飞点）
  · Gazebo 真值里 x650_0 的 z 与落点板的 z

输出 CSV：t, px4_alt, true_alt_local, pad_alt_local, dz
用法：python3 scripts/25_z_drift_check.py --duration 90 --csv logs/z_drift.csv
"""
from __future__ import annotations
import argparse, csv, time
import gz.msgs10.pose_v_pb2 as pv
import gz.transport13 as gz
import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

ORIGIN_Z = 1.02     # 起飞点世界高度（dock 台面）
PAD_LOCAL = (0.30, 0.0, 0.45)


def fmu_qos():
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


class Drift(Node):
    def __init__(self, fh):
        super().__init__("z_drift")
        self.fh = fh
        self.w = csv.writer(fh)
        self.w.writerow(["t", "px4_alt", "true_alt", "pad_alt", "dz"])
        self.px4 = None
        self.drone = None
        self.ship = None
        self.t0 = time.time()
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
                                 self.on_px4, fmu_qos())
        self.gz = gz.Node()
        self.gz.subscribe(pv.Pose_V, "/world/x650_sea/pose/info", self.on_gz)
        self.create_timer(0.1, self.tick)
        self.n = 0

    def on_px4(self, m):
        self.px4 = -m.z

    def on_gz(self, msg):
        for p in msg.pose:
            if p.name == "x650_0":
                self.drone = p.position.z
            elif p.name == "ship":
                self.ship = p.position.z + PAD_LOCAL[2]

    def tick(self):
        if self.px4 is None or self.drone is None:
            return
        t = time.time() - self.t0
        true_alt = self.drone - ORIGIN_Z
        pad_alt = (self.ship - ORIGIN_Z) if self.ship is not None else float("nan")
        self.w.writerow([round(t, 2), round(self.px4, 3), round(true_alt, 3),
                         round(pad_alt, 3), round(self.px4 - true_alt, 3)])
        if self.n % 10 == 0:
            self.fh.flush()
        self.n += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=90.0)
    ap.add_argument("--csv", default="logs/z_drift.csv")
    args = ap.parse_args()
    rclpy.init()
    fh = open(args.csv, "w", newline="")
    node = Drift(fh)
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
    fh.close()
    node.destroy_node()
    rclpy.shutdown()
    print(f"采样 {node.n} 帧 → {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
