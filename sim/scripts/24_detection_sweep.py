#!/usr/bin/env python3
"""24 · 检出率 vs 距离扫描：把相机固定在板正上方不同高度，统计 ArUco 检出率

用法：python3 scripts/24_detection_sweep.py --heights 1,2,3,5,8,10,12
输出：logs/detection_sweep.csv（高度、帧数、检出帧数、检出率、处理耗时）
"""
from __future__ import annotations
import argparse, csv, time
import numpy as np
import cv2
import gz.msgs10.boolean_pb2 as b
import gz.msgs10.image_pb2 as im
import gz.msgs10.pose_pb2 as pb
import gz.transport13 as t

PAD = (0.30, 0.0, 0.81)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="x650_sea")
    ap.add_argument("--heights", default="1,2,3,5,8,10,12")
    ap.add_argument("--dwell", type=float, default=4.0)
    ap.add_argument("--csv", default="logs/detection_sweep.csv")
    args = ap.parse_args()
    heights = [float(h) for h in args.heights.split(",")]

    ctl = t.Node()
    cam = t.Node()
    svc = f"/world/{args.world}/set_pose"
    topic = f"/world/{args.world}/model/x650_0/link/base_link/sensor/downcam/image"

    state = {"frames": 0, "det": 0, "sides": []}
    K = None

    def on_img(m):
        state["frames"] += 1
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
        gray = cv2.cvtColor(a[:, :, :3], cv2.COLOR_RGB2GRAY)
        h, w = gray.shape
        fx = (w / 2.0) / np.tan(1.4 / 2.0)
        kmat = np.array([[fx, 0, w / 2], [0, fx, h / 2], [0, 0, 1]], dtype=float)
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=params)
        if ids is not None and len(ids):
            state["det"] += 1
            for i in range(len(ids)):
                q = corners[i].reshape(4, 2)
                state["sides"].append(float(np.mean([np.linalg.norm(q[j] - q[(j + 1) % 4]) for j in range(4)])))

    cam.subscribe(im.Image, topic, on_img)

    rows = []
    for h in heights:
        # 相机装机体上（base_link 比模型原点高 0.28，传感器再 -0.05）
        model_z = PAD[2] + h - 0.23
        p = pb.Pose(); p.name = "x650_0"
        p.position.x, p.position.y, p.position.z = PAD[0], PAD[1], model_z
        p.orientation.w = 1.0
        t0 = time.time()
        while time.time() - t0 < args.dwell:
            ctl.request(svc, p, pb.Pose, b.Boolean, 100)
            time.sleep(0.05)
        time.sleep(0.3)
        rate = 100.0 * state["det"] / max(1, state["frames"])
        side = float(np.mean(state["sides"])) if state["sides"] else 0.0
        print(f"高度 {h:4.1f} m: {state['frames']:4d} 帧, 检出 {state['det']:4d}, "
              f"检出率 {rate:5.1f}%, 标记边长≈{side:4.1f} px")
        rows.append((h, state["frames"], state["det"], round(rate, 1), round(side, 1)))
        state["frames"] = state["det"] = 0
        state["sides"].clear()

    with open(args.csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["height_m", "frames", "detected", "rate_pct", "marker_px"])
        w.writerows(rows)
    print(f"CSV → {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
