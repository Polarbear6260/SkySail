#!/usr/bin/env python3
"""19 · GPS 级粗定位源：把船的位置"劣化"成远距离 GPS 相对定位

远距离方案里，船的位置来自（船的 GPS 经数传广播 + 本机 GPS）算出的相对矢量，
特点是：**低频率（1–2 Hz）、米级噪声（1–3 m）、偶发跳变**。

这个节点把真值位姿劣化成这种"GPS 级"数据，供控制器在远距离段使用：

输入： /ship/pad_pose（真值，ENU）
输出： /ship/pad_pose_gps（ENU，1 Hz，σ=--noise 米的高斯噪声 + 偶发跳变）

用法：
  python3 scripts/19_gps_source.py --rate 1 --noise 1.5 --jump-p 0.02
"""

from __future__ import annotations

import argparse
import math
import random
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GpsSource(Node):
    def __init__(self, args) -> None:
        super().__init__("gps_source")
        self.noise = float(args.noise)
        self.jump_p = float(args.jump_p)
        self.jump_m = float(args.jump_m)
        self.rng = random.Random(args.seed)

        self.latest: PoseStamped | None = None
        self.create_subscription(PoseStamped, "/ship/pad_pose", self.on_pose, 10)
        self.pub = self.create_publisher(PoseStamped, "/ship/pad_pose_gps", 10)
        self.create_timer(1.0 / float(args.rate), self.publish)

        self.get_logger().info(
            f"GPS 级粗定位: {args.rate} Hz, 噪声 σ={self.noise:.1f} m, "
            f"跳变 {self.jump_p*100:.0f}% × {self.jump_m:.1f} m, seed={args.seed}")

    def on_pose(self, m: PoseStamped) -> None:
        self.latest = m

    def publish(self) -> None:
        if self.latest is None:
            return
        src = self.latest
        out = PoseStamped()
        out.header = src.header
        out.pose = src.pose
        jx = jy = 0.0
        if self.rng.random() < self.jump_p:          # 偶发多径/星历跳变
            ang = self.rng.uniform(0, 2 * math.pi)
            jx, jy = self.jump_m * math.cos(ang), self.jump_m * math.sin(ang)
        out.pose.position.x += self.rng.gauss(0.0, self.noise) + jx
        out.pose.position.y += self.rng.gauss(0.0, self.noise) + jy
        self.pub.publish(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--noise", type=float, default=1.5)
    ap.add_argument("--jump-p", type=float, default=0.02)
    ap.add_argument("--jump-m", type=float, default=3.0)
    ap.add_argument("--seed", type=int, default=None)
    args, _ = ap.parse_known_args()
    if args.seed is None:
        args.seed = random.randrange(1, 10**6)

    rclpy.init()
    node = GpsSource(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
