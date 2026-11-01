#!/usr/bin/env python3
"""10 · 移动平台驱动（运动学船，供 X650 移动着陆实验用）

用 gz-transport 的 `/world/<world>/set_pose` 服务以 50 Hz 设置船体位姿：
  · 水平：绕半径 R 的圆弧（模拟船绕圈航行，速度可调）
  · 升沉：z = z0 + A_heave·sin(2π f_heave t)
  · 横摇 / 纵摇：小幅正弦（模拟涌浪）
  · 艏向：沿圆弧切线

输出：
  · 控制台每秒打印一次船体/落点板位姿
  · 退出时打印 summary（用于和飞行日志对齐）
  · 可选 --csv 把位姿时间序列写成 CSV（给控制器的参考轨迹做真值）

用法：
  python3 scripts/10_move_platform.py                       # 默认参数跑 120 s
  python3 scripts/10_move_platform.py --duration 60 --radius 5 --speed 0.4
  python3 scripts/10_move_platform.py --csv logs/ship.csv

依赖：gz.transport13 / gz.msgs10（随 Gazebo Harmonic 安装，系统 python 即可）
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time

import gz.msgs10.boolean_pb2 as boolean_pb2
import gz.msgs10.pose_pb2 as pose_pb2
import gz.transport13 as gz_transport

PAD_LOCAL = (0.30, 0.0, 0.45)      # 落点板中心（船体坐标系）
SHIP_Z = 0.36                      # 船体基准高度（吃水线以上）


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def build_pose(name: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> pose_pb2.Pose:
    # 小角度近似：roll/pitch 直接塞进四元数（横摇纵摇都很小，误差可忽略）
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    p = pose_pb2.Pose()
    p.name = name
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="x650_sea")
    ap.add_argument("--model", default="ship")
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--radius", type=float, default=6.0, help="圆弧半径 m")
    ap.add_argument("--speed", type=float, default=0.4, help="航速 m/s")
    ap.add_argument("--heave", type=float, default=0.10, help="升沉幅值 m")
    ap.add_argument("--heave-period", type=float, default=5.0, help="升沉周期 s")
    ap.add_argument("--roll-deg", type=float, default=3.0)
    ap.add_argument("--pitch-deg", type=float, default=2.0)
    ap.add_argument("--rate", type=float, default=50.0, help="set_pose 频率 Hz")
    ap.add_argument("--csv", default="")
    args = ap.parse_args()

    node = gz_transport.Node()
    svc = f"/world/{args.world}/set_pose"
    req_type = pose_pb2.Pose
    rep_type = boolean_pb2.Boolean

    def set_pose(p: pose_pb2.Pose) -> bool:
        ok, _ = node.request(svc, p, req_type, rep_type, 500)
        return bool(ok)

    period = 1.0 / args.rate
    omega_ship = args.speed / max(args.radius, 1e-3)      # 角速度 rad/s
    t0 = time.time()
    next_tick = t0
    rows: list[tuple[float, float, float, float, float, float, float]] = []
    ok_ct = 0
    last_print = 0.0

    print(f"驱动 {args.model}：R={args.radius} m  v={args.speed} m/s  "
          f"升沉 {args.heave} m/{args.heave_period}s  时长 {args.duration}s")
    try:
        while True:
            now = time.time()
            t = now - t0
            if t > args.duration:
                break
            if now < next_tick:
                time.sleep(min(next_tick - now, 0.002))
                continue
            next_tick += period

            yaw = omega_ship * t
            x = args.radius * math.sin(yaw)
            y = args.radius * math.cos(yaw) - args.radius
            z = SHIP_Z + args.heave * math.sin(2 * math.pi * t / args.heave_period)
            roll = math.radians(args.roll_deg) * math.sin(2 * math.pi * t / args.heave_period + 0.6)
            pitch = math.radians(args.pitch_deg) * math.sin(2 * math.pi * t / 7.0)

            p = build_pose(args.model, x, y, z, roll, pitch, yaw)
            if set_pose(p):
                ok_ct += 1

            # 落点板世界位姿（船体坐标系 → 世界）
            px = x + PAD_LOCAL[0] * math.cos(yaw) - PAD_LOCAL[1] * math.sin(yaw)
            py = y + PAD_LOCAL[0] * math.sin(yaw) + PAD_LOCAL[1] * math.cos(yaw)
            pz = z + PAD_LOCAL[2]
            rows.append((t, px, py, pz, roll, pitch, yaw))

            if t - last_print >= 2.0:
                last_print = t
                print(f"  t={t:6.2f}s  ship=({x:+.2f},{y:+.2f},{z:.2f}) "
                      f"yaw={math.degrees(yaw):+6.1f}°  pad=({px:+.2f},{py:+.2f},{pz:.2f})", flush=True)
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        pass  # gz.transport Node 没有 close()

    if rows:
        ts = [r[0] for r in rows]
        print(f"\n完成：{len(rows)} 帧，{ok_ct} 次 set_pose 成功，时长 {ts[-1]:.1f}s")
        print(f"  落点板 x∈[{min(r[1] for r in rows):+.2f},{max(r[1] for r in rows):+.2f}] "
              f"y∈[{min(r[2] for r in rows):+.2f},{max(r[2] for r in rows):+.2f}]")
    if args.csv and rows:
        with open(args.csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "pad_x", "pad_y", "pad_z", "roll", "pitch", "yaw"])
            w.writerows(rows)
        print(f"  CSV → {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
