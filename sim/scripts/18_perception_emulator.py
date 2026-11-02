#!/usr/bin/env python3
"""18 · 感知仿真器：给"完美"的平台位姿加延迟 / 噪声 / 丢帧

真机上落点板位姿来自视觉检测，端到端有几十到上百毫秒延迟、厘米级噪声、
偶发丢帧。这个节点把仿真里的真值位姿劣化成"像视觉出来的数据"，
让控制器在带噪数据下工作，从而量出它对感知质量的敏感度。

输入： /ship/pad_pose, /ship/pad_twist        （真值，来自船驱动）
输出： /ship/pad_pose_d, /ship/pad_twist_d    （劣化后的"感知"数据）

参数（都是相对真值的偏差）：
  --delay      端到端延迟 s            （默认 0.0）
  --noise      位置噪声 σ m            （默认 0.0，三轴独立高斯）
  --vel-noise  速度噪声 σ m/s          （默认 0.0）
  --dropout    丢帧概率 0..1           （默认 0.0，丢了就保持上一帧）
  --seed       随机种子                （默认随机）

用法：
  python3 scripts/18_perception_emulator.py --delay 0.1 --noise 0.03 --dropout 0.05
"""

from __future__ import annotations

import argparse
import random
import sys
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node


class PerceptionEmulator(Node):
    def __init__(self, args) -> None:
        super().__init__("perception_emulator")
        self.delay = float(args.delay)
        self.noise = float(args.noise)
        self.vel_noise = float(args.vel_noise)
        self.dropout = float(args.dropout)
        self.rng = random.Random(args.seed)

        self.pub_pose = self.create_publisher(PoseStamped, "/ship/pad_pose_d", 10)
        self.pub_twist = self.create_publisher(TwistStamped, "/ship/pad_twist_d", 10)
        self.create_subscription(PoseStamped, "/ship/pad_pose", self.on_pose, 10)
        self.create_subscription(TwistStamped, "/ship/pad_twist", self.on_twist, 10)

        self.pose_q: deque = deque()      # (release_time, msg)
        self.twist_q: deque = deque()
        self.create_timer(0.01, self.flush)
        self.stat = {"in": 0, "dropped": 0, "out": 0}
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"感知仿真器: 延迟 {self.delay*1000:.0f} ms, 位置噪声 {self.noise*100:.1f} cm, "
            f"速度噪声 {self.vel_noise:.2f} m/s, 丢帧 {self.dropout*100:.0f}%, seed={args.seed}")

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def distort_pose(self, m: PoseStamped) -> PoseStamped:
        out = PoseStamped()
        out.header = m.header
        out.pose = m.pose
        out.pose.position.x += self.rng.gauss(0.0, self.noise)
        out.pose.position.y += self.rng.gauss(0.0, self.noise)
        out.pose.position.z += self.rng.gauss(0.0, self.noise)
        return out

    def distort_twist(self, m: TwistStamped) -> TwistStamped:
        out = TwistStamped()
        out.header = m.header
        out.twist = m.twist
        out.twist.linear.x += self.rng.gauss(0.0, self.vel_noise)
        out.twist.linear.y += self.rng.gauss(0.0, self.vel_noise)
        out.twist.linear.z += self.rng.gauss(0.0, self.vel_noise)
        return out

    def on_pose(self, m: PoseStamped) -> None:
        self.stat["in"] += 1
        if self.rng.random() < self.dropout:
            self.stat["dropped"] += 1
            return
        self.pose_q.append((self.now_s() + self.delay, self.distort_pose(m)))

    def on_twist(self, m: TwistStamped) -> None:
        if self.rng.random() < self.dropout:
            return
        self.twist_q.append((self.now_s() + self.delay, self.distort_twist(m)))

    def flush(self) -> None:
        now = self.now_s()
        while self.pose_q and self.pose_q[0][0] <= now:
            self.pub_pose.publish(self.pose_q.popleft()[1])
            self.stat["out"] += 1
        while self.twist_q and self.twist_q[0][0] <= now:
            self.pub_twist.publish(self.twist_q.popleft()[1])

    def report(self) -> None:
        if self.stat["in"]:
            self.get_logger().info(
                f"感知仿真: 收 {self.stat['in']} 帧, 丢 {self.stat['dropped']} 帧, "
                f"发 {self.stat['out']} 帧")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--delay", type=float, default=0.0)
    ap.add_argument("--noise", type=float, default=0.0)
    ap.add_argument("--vel-noise", type=float, default=0.0)
    ap.add_argument("--dropout", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=None)
    args, _ = ap.parse_known_args()
    if args.seed is None:
        args.seed = random.randrange(1, 10**6)

    rclpy.init()
    node = PerceptionEmulator(args)
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
