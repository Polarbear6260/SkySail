#!/usr/bin/env python3
"""21 · 跟随相机：把 camrig 模型贴到无人机位姿上（俯视），供视觉检测用

为什么需要它：机体模型由 PX4 通过 gz_bridge 持有，无法直接给它加相机传感器，
所以用一个"镜像模型"跟随无人机，相机装在它上面。

用法：python3 scripts/21_camrig.py --world x650_sea --drone x650_0 --rig camrig
"""
from __future__ import annotations
import argparse, sys, time
import gz.msgs10.boolean_pb2 as bool_pb2
import gz.msgs10.pose_pb2 as pose_pb2
import gz.msgs10.pose_v_pb2 as pose_v_pb2
import gz.transport13 as gz_transport


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="x650_sea")
    ap.add_argument("--drone", default="x650_0")
    ap.add_argument("--rig", default="camrig")
    ap.add_argument("--duration", type=float, default=240.0)
    args = ap.parse_args()

    state = {"pose": None}

    def on_pose(msg):
        for p in msg.pose:
            if p.name == args.drone:
                state["pose"] = p

    node = gz_transport.Node()
    node.subscribe(pose_v_pb2.Pose_V, f"/world/{args.world}/pose/info", on_pose)
    svc = f"/world/{args.world}/set_pose"

    t0 = time.time(); nxt = t0
    print(f"跟随相机：{args.rig} ← {args.drone}")
    while time.time() - t0 < args.duration:
        now = time.time()
        if now < nxt:
            time.sleep(0.005); continue
        nxt += 0.05
        src = state["pose"]
        if src is None:
            continue
        p = pose_pb2.Pose()
        p.name = args.rig
        p.position.x, p.position.y = src.position.x, src.position.y
        p.position.z = src.position.z - 0.10          # 略低于机体，避免被桨遮挡
        p.orientation.x, p.orientation.y = 0.0, 0.7071068   # 俯视：绕 y 转 +90°
        p.orientation.z, p.orientation.w = 0.0, 0.7071068
        node.request(svc, p, pose_pb2.Pose, bool_pb2.Boolean, 200)
    return 0


if __name__ == "__main__":
    sys.exit(main())
