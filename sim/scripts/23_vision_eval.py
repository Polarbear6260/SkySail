#!/usr/bin/env python3
"""23 · 视觉估计精度评估：把 /ship/pad_pose_vis 与 Gazebo 真值逐帧对比

输出 CSV：时间、视觉估计、真值、误差分量、水平距离、机体倾角。
用来判断视觉误差是随距离增长还是随倾角增长（决定改进方向）。

用法：
  python3 scripts/23_vision_eval.py --duration 120 --csv logs/vision_eval.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time

import gz.msgs10.pose_v_pb2 as pose_v_pb2
import gz.transport13 as gz_transport
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

PAD_LOCAL = (0.30, 0.0, 0.45)


def quat_to_rot(q):
    x, y, z, w = q
    return (((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
             (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
             (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def tilt_deg(q) -> float:
    x, y, z, w = q
    # 机体系 z 轴与世界 z 轴夹角
    r20 = 2 * (x * z - y * w)
    r21 = 2 * (y * z + x * w)
    r22 = 1 - 2 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, r22))))


class Eval(Node):
    def __init__(self, fh) -> None:
        super().__init__("vision_eval")
        self.fh = fh
        self.w = csv.writer(fh)
        self.w.writerow(["t", "vis_x", "vis_y", "vis_z", "true_x", "true_y", "true_z",
                         "ex", "ey", "ez", "err", "dist", "tilt_deg"])
        self.vis = None
        self.ship = None
        self.drone = None
        self.t0 = time.time()
        self.create_subscription(PoseStamped, "/ship/pad_pose_vis", self.on_vis, 10)
        self.gz = gz_transport.Node()
        self.gz.subscribe(pose_v_pb2.Pose_V, "/world/x650_sea/pose/info", self.on_gz)
        self.create_timer(0.1, self.tick)
        self.n = 0

    def on_vis(self, m: PoseStamped) -> None:
        self.vis = (m.pose.position.x, m.pose.position.y, m.pose.position.z)

    def on_gz(self, msg) -> None:
        for p in msg.pose:
            if p.name == "ship":
                yaw = math.atan2(2 * (p.orientation.w * p.orientation.z
                                      + p.orientation.x * p.orientation.y),
                                 1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2))
                px = p.position.x + PAD_LOCAL[0] * math.cos(yaw)
                py = p.position.y + PAD_LOCAL[0] * math.sin(yaw)
                pz = p.position.z + PAD_LOCAL[2]
                self.ship = (px, py, pz)
            elif p.name == "x650_0":
                self.drone = p

    def tick(self) -> None:
        if self.vis is None or self.ship is None or self.drone is None:
            return
        t = time.time() - self.t0
        ex = self.vis[0] - self.ship[0]
        ey = self.vis[1] - self.ship[1]
        ez = self.vis[2] - self.ship[2]
        err = math.sqrt(ex * ex + ey * ey + ez * ez)
        dist = math.dist((self.drone.position.x, self.drone.position.y, self.drone.position.z),
                         self.ship)
        self.w.writerow([round(t, 2), round(self.vis[0], 3), round(self.vis[1], 3), round(self.vis[2], 3),
                         round(self.ship[0], 3), round(self.ship[1], 3), round(self.ship[2], 3),
                         round(ex, 3), round(ey, 3), round(ez, 3), round(err, 3),
                         round(dist, 2), round(tilt_deg((self.drone.orientation.x,
                                                         self.drone.orientation.y,
                                                         self.drone.orientation.z,
                                                         self.drone.orientation.w)), 1)])
        if self.n % 10 == 0:
            self.fh.flush()
        self.n += 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--csv", default="logs/vision_eval.csv")
    args, _ = ap.parse_known_args()
    rclpy.init()
    fh = open(args.csv, "w", newline="")
    node = Eval(fh)
    t0 = time.time()
    try:
        while rclpy.ok() and time.time() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        fh.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"采样 {node.n} 帧 → {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
