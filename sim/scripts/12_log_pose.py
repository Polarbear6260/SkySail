#!/usr/bin/env python3
"""12 · 从 Gazebo 直接记录机体与落点板的真实相对位姿

不依赖 PX4 的坐标（那是相对出生点的，容易和世界系混淆），
直接读 /world/<world>/pose/info，自己算：

  · 落点板世界位置 = 船体位置 + 旋转后的船体系偏移 (0.3, 0, 0.45)
  · 水平距离 = 机体到板中心的水平距离
  · 相对高度 = 机体 z - 板面 z

输出：控制台每秒一行 + CSV（--csv 指定）

用法：
  python3 scripts/12_log_pose.py --world x650_sea --duration 200 --csv logs/pose.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time

import gz.msgs10.pose_v_pb2 as pose_v_pb2
import gz.transport13 as gz_transport

PAD_LOCAL = (0.30, 0.0, 0.45)


def quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="x650_sea")
    ap.add_argument("--drone", default="x650_0")
    ap.add_argument("--ship", default="ship")
    ap.add_argument("--duration", type=float, default=200.0)
    ap.add_argument("--csv", default="")
    args = ap.parse_args()

    state = {"drone": None, "ship": None}

    def on_pose(msg) -> None:
        for p in msg.pose:
            if p.name == args.drone:
                state["drone"] = (p.position.x, p.position.y, p.position.z)
            elif p.name == args.ship:
                state["ship"] = (p.position.x, p.position.y, p.position.z,
                                 quat_to_yaw(p.orientation))

    node = gz_transport.Node()
    node.subscribe(pose_v_pb2.Pose_V, f"/world/{args.world}/pose/info", on_pose)

    t0 = time.time()
    rows = []
    fh = open(args.csv, "w", newline="") if args.csv else None
    if fh:
        import csv as _csv
        _w = _csv.writer(fh)
        _w.writerow(["t", "dx", "dy", "dz", "px", "py", "pz", "horiz", "above"])
    last_print = 0.0
    print(f"记录 {args.drone} 相对落点板的真实位置（{args.duration:.0f} s）")
    try:
        while True:
            t = time.time() - t0
            if t > args.duration:
                break
            time.sleep(0.05)
            if state["drone"] is None or state["ship"] is None:
                continue
            dx, dy, dz = state["drone"]
            sx, sy, sz, yaw = state["ship"]
            px = sx + PAD_LOCAL[0] * math.cos(yaw) - PAD_LOCAL[1] * math.sin(yaw)
            py = sy + PAD_LOCAL[0] * math.sin(yaw) + PAD_LOCAL[1] * math.cos(yaw)
            pz = sz + PAD_LOCAL[2]
            horiz = math.hypot(dx - px, dy - py)
            above = dz - pz
            rows.append((t, dx, dy, dz, px, py, pz, horiz, above))
            if fh:
                _w.writerow([round(t, 3), round(dx, 3), round(dy, 3), round(dz, 3),
                             round(px, 3), round(py, 3), round(pz, 3),
                             round(horiz, 3), round(above, 3)])
                if len(rows) % 20 == 0:
                    fh.flush()
            if t - last_print >= 1.0:
                last_print = t
                print(f"  t={t:6.1f}s drone=({dx:+6.2f},{dy:+6.2f},{dz:6.2f}) "
                      f"pad=({px:+6.2f},{py:+6.2f},{pz:6.2f}) "
                      f"水平={horiz:.2f} 高于板面={above:+.2f}", flush=True)
    except KeyboardInterrupt:
        pass

    if rows:
        mins = min(r[7] for r in rows)
        print(f"\n全程最小水平距离: {mins:.2f} m")
        if args.csv:
            with open(args.csv, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["t", "dx", "dy", "dz", "px", "py", "pz", "horiz", "above"])
                w.writerows(rows)
            print(f"  CSV → {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
