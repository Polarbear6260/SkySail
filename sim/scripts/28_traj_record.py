#!/usr/bin/env python3
"""28 · 轨迹记录：把控制器内部量（参考、指令、状态）逐拍写成 CSV

用途：给「原始算法复算 vs Gazebo 实测」的对比分析提供数据。

订阅：
  /x650/debug                        控制器内部（参考、指令、阶段）
  /fmu/out/vehicle_local_position_v1 飞控状态（位置、速度）

输出列：
  t, state, px,py,pz, vx,vy,vz, rpx,rpy,rpz, rvx,rvy,rvz, ax,ay,az,
  padx,pady,padz, true_horiz, true_above

用法：
  python3 scripts/28_traj_record.py --duration 120 --csv logs/traj.csv
"""
from __future__ import annotations

import argparse
import csv
import json
import time

import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def fmu_qos() -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            history=HistoryPolicy.KEEP_LAST, depth=1)


class TrajRecord(Node):
    def __init__(self, fh) -> None:
        super().__init__("traj_record")
        self.w = csv.writer(fh)
        self.w.writerow(["t", "state", "px", "py", "pz", "vx", "vy", "vz",
                         "rpx", "rpy", "rpz", "rvx", "rvy", "rvz",
                         "ax", "ay", "az", "padx", "pady", "padz",
                         "true_horiz", "true_above"])
        self.fh = fh
        self.pos = None
        self.vel = None
        self.dbg = None
        self.t0 = time.time()
        self.n = 0
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
                                 self.on_pos, fmu_qos())
        self.create_subscription(String, "/x650/debug", self.on_dbg, 10)
        self.create_timer(0.05, self.tick)      # 20 Hz

    def on_pos(self, m: VehicleLocalPosition) -> None:
        self.pos = (m.x, m.y, m.z)
        self.vel = (m.vx, m.vy, m.vz)

    def on_dbg(self, m: String) -> None:
        try:
            self.dbg = json.loads(m.data)
        except Exception:  # noqa: BLE001
            pass

    def tick(self) -> None:
        d = self.dbg
        if self.pos is None or self.vel is None or d is None:
            return
        if d.get("ref_pos") is None or d.get("acc") is None or d.get("pad_ned") is None:
            return
        t = time.time() - self.t0
        self.w.writerow([round(t, 3), d.get("state", ""),
                         *[round(v, 4) for v in self.pos],
                         *[round(v, 4) for v in self.vel],
                         *[round(v, 4) for v in d["ref_pos"]],
                         *[round(v, 4) for v in d["ref_vel"]],
                         *[round(v, 4) for v in d["acc"]],
                         *[round(v, 4) for v in d["pad_ned"]],
                         round(float(d.get("true_horiz") or 0.0), 4),
                         round(float(d.get("true_above") or 0.0), 4)])
        self.n += 1
        if self.n % 20 == 0:
            self.fh.flush()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--csv", default="logs/traj.csv")
    args = ap.parse_args()
    rclpy.init()
    fh = open(args.csv, "w", newline="")
    node = TrajRecord(fh)
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
    fh.close()
    node.destroy_node()
    rclpy.shutdown()
    print(f"记录 {node.n} 拍 → {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
