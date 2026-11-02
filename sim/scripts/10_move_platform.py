#!/usr/bin/env python3
"""10 · 船运动驱动（随机起点/航向/航速/缓弯，可复现）

默认随机生成一次"航行任务"：

  · 起点：在停机坪（dock）周围 35–55 m 的圆上随机取一点
  · 航向：大致朝向 dock（±20°），保证会从附近经过
  · 航速：1.0–2.0 m/s（2–4 节）
  · 缓弯：±0.008 rad/s 的持续转向（真实船舶不会走绝对直线）
  · 海况：升沉/横摇/纵摇随机幅值与相位

会打印所用种子与参数，`--seed N` 可完全复现同一次任务。

用法：
  python3 scripts/10_move_platform.py                       # 随机
  python3 scripts/10_move_platform.py --seed 42             # 复现某次
  python3 scripts/10_move_platform.py --fixed --start-x -40 --start-y 0 --start-yaw 0 --speed 1.5
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import sys
import time

import gz.msgs10.boolean_pb2 as boolean_pb2
import gz.msgs10.pose_pb2 as pose_pb2
import gz.transport13 as gz_transport

PAD_LOCAL = (0.30, 0.0, 0.45)
SHIP_Z = 0.36
DOCK = (0.0, 16.0)          # 停机坪位置（世界 ENU），与 world 里的 dock 一致

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from rclpy.node import Node as RclNode
    HAS_ROS = True
except Exception:  # noqa: BLE001
    HAS_ROS = False


def build_pose(name, x, y, z, roll, pitch, yaw) -> pose_pb2.Pose:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    p = pose_pb2.Pose()
    p.name = name
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x = sr * cp * cy - cr * sp * sy
    p.orientation.y = cr * sp * cy + sr * cp * sy
    p.orientation.z = cr * cp * sy - sr * sp * cy
    p.orientation.w = cr * cp * cy + sr * sp * sy
    return p


def sample_mission(rng: random.Random) -> dict:
    """在停机坪周围随机生成一次航行任务（起点、航向、航速、缓弯、海况）。"""
    dist = rng.uniform(35.0, 55.0)
    ang = rng.uniform(-math.pi, math.pi)
    x0 = DOCK[0] + dist * math.cos(ang)
    y0 = DOCK[1] + dist * math.sin(ang)
    aim = math.atan2(DOCK[1] - y0, DOCK[0] - x0)
    yaw0 = aim + rng.uniform(-0.35, 0.35)      # 朝 dock ±20°，确保会经过附近
    return {
        "x0": x0, "y0": y0, "yaw0": yaw0,
        "speed": rng.uniform(1.0, 2.0),
        "turn_rate": rng.uniform(-0.008, 0.008),
        "heading_amp": math.radians(rng.uniform(1.0, 4.0)),
        "heading_period": rng.uniform(15.0, 35.0),
        "heave": rng.uniform(0.05, 0.12),
        "heave_period": rng.uniform(4.0, 8.0),
        "heave_phase": rng.uniform(0, 2 * math.pi),
        "roll": math.radians(rng.uniform(2.0, 5.0)),
        "roll_phase": rng.uniform(0, 2 * math.pi),
        "pitch": math.radians(rng.uniform(1.0, 3.0)),
        "pitch_period": rng.uniform(5.0, 9.0),
        "pitch_phase": rng.uniform(0, 2 * math.pi),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="x650_sea")
    ap.add_argument("--model", default="ship")
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--delay", type=float, default=0.0, help="起航前等待秒数")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--fixed", action="store_true", help="关闭随机，用下面给的值")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--start-x", type=float, default=-40.0)
    ap.add_argument("--start-y", type=float, default=0.0)
    ap.add_argument("--start-yaw", type=float, default=0.0)
    ap.add_argument("--speed", type=float, default=1.5)
    ap.add_argument("--csv", default="")
    ap.add_argument("--no-ros", action="store_true")
    args = ap.parse_args()

    seed = args.seed if args.seed is not None else random.randrange(1, 10**6)
    rng = random.Random(seed)
    if args.fixed:
        m = {"x0": args.start_x, "y0": args.start_y, "yaw0": args.start_yaw,
             "speed": args.speed, "turn_rate": 0.0,
             "heading_amp": math.radians(2.5), "heading_period": 25.0,
             "heave": 0.08, "heave_period": 6.0, "heave_phase": 0.0,
             "roll": math.radians(4.0), "roll_phase": 0.0,
             "pitch": math.radians(2.0), "pitch_period": 7.0, "pitch_phase": 0.0}
    else:
        m = sample_mission(rng)

    print(f"seed={seed}")
    print(f"起点 ({m['x0']:+.1f}, {m['y0']:+.1f})  航向 {math.degrees(m['yaw0']):+.1f}°  "
          f"航速 {m['speed']:.2f} m/s ({m['speed'] / 0.5144:.1f} kn)  "
          f"转向 {m['turn_rate']:+.4f} rad/s")
    print(f"海况: 升沉 {m['heave']:.2f} m/{m['heave_period']:.1f}s  "
          f"横摇 {math.degrees(m['roll']):.1f}°  纵摇 {math.degrees(m['pitch']):.1f}°")

    pub = None
    rnode = None
    if HAS_ROS and not args.no_ros:
        rclpy.init()
        rnode = RclNode("ship_driver")
        pub_pose = rnode.create_publisher(PoseStamped, "/ship/pad_pose", 10)
        pub_twist = rnode.create_publisher(TwistStamped, "/ship/pad_twist", 10)
        pub = True
        print("ROS 话题：/ship/pad_pose, /ship/pad_twist")

    node = gz_transport.Node()
    svc = f"/world/{args.world}/set_pose"

    def set_pose(p) -> bool:
        ok, _ = node.request(svc, p, pose_pb2.Pose, boolean_pb2.Boolean, 500)
        return bool(ok)

    x, y, yaw = m["x0"], m["y0"], m["yaw0"]
    v, omega = m["speed"], m["turn_rate"]
    t0 = time.time()
    next_tick = t0
    rows = []
    dt = 1.0 / args.rate

    set_pose(build_pose(args.model, x, y, SHIP_Z, 0.0, 0.0, yaw))
    print(f"驱动 {args.model}：时长 {args.duration}s", flush=True)

    try:
        while True:
            now = time.time()
            t = now - t0
            if t > args.duration:
                break
            if now < next_tick:
                time.sleep(min(next_tick - now, 0.002))
                continue
            next_tick += dt

            if t >= args.delay:
                yaw += omega * dt
                x += v * math.cos(yaw) * dt
                y += v * math.sin(yaw) * dt
            yaw_vis = yaw + m["heading_amp"] * math.sin(2 * math.pi * t / m["heading_period"])
            z = SHIP_Z + m["heave"] * math.sin(
                2 * math.pi * t / m["heave_period"] + m["heave_phase"])
            roll = m["roll"] * math.sin(2 * math.pi * t / m["heave_period"] + m["roll_phase"])
            pitch = m["pitch"] * math.sin(2 * math.pi * t / m["pitch_period"] + m["pitch_phase"])

            set_pose(build_pose(args.model, x, y, z, roll, pitch, yaw_vis))

            px = x + PAD_LOCAL[0] * math.cos(yaw_vis)
            py = y + PAD_LOCAL[0] * math.sin(yaw_vis)
            pz = z + PAD_LOCAL[2]
            moving = t >= args.delay
            vx = v * math.cos(yaw) if moving else 0.0
            vy = v * math.sin(yaw) if moving else 0.0
            vz = m["heave"] * 2 * math.pi / m["heave_period"] * math.cos(
                2 * math.pi * t / m["heave_period"] + m["heave_phase"])

            if pub is not None:
                stamp = rnode.get_clock().now().to_msg()
                ps = PoseStamped()
                ps.header.stamp = stamp
                ps.header.frame_id = "map"
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = px, py, pz
                ps.pose.orientation.z = math.sin(yaw_vis / 2)
                ps.pose.orientation.w = math.cos(yaw_vis / 2)
                ts = TwistStamped()
                ts.header.stamp = stamp
                ts.twist.linear.x, ts.twist.linear.y, ts.twist.linear.z = vx, vy, vz
                pub_pose.publish(ps)
                pub_twist.publish(ts)

            rows.append((t, px, py, pz, vx, vy, vz))
    except KeyboardInterrupt:
        pass
    finally:
        if rnode is not None:
            rnode.destroy_node()
            rclpy.shutdown()

    if rows:
        print(f"完成：{len(rows)} 帧，时长 {rows[-1][0]:.1f}s，"
              f"航程 {math.hypot(rows[-1][1] - rows[0][1], rows[-1][2] - rows[0][2]):.1f} m")
    if args.csv and rows:
        with open(args.csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "pad_x", "pad_y", "pad_z", "vx", "vy", "vz"])
            w.writerows(rows)
        print(f"CSV → {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
