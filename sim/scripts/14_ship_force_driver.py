#!/usr/bin/env python3
"""14 · 力驱动的船（动态刚体，接触真实）

为什么不用 set_pose：
  静态体每 20 ms 被瞬移时，物理引擎不会给它“拖住”落在上面的机体，
  飞机会被留在原地掉下去。改成动态体 + 每步施加合力，接触是真实接触，
  上锁停机后机体能被甲板带着走。

控制律（每 50 ms 施加一次，作用在 ship::hull 上）：
  F = m·(kp·(p_des − p) + kd·(v_des − v)) + m·g·ẑ
  τ_z = I_zz·(kp_yaw·Δyaw + kd_yaw·Δω)

同时把落点板的世界位姿（由船的实际位姿算出）发布到 ROS：/ship/pad_pose、/ship/pad_twist

用法：
  python3 scripts/14_ship_force_driver.py --world x650_sea --radius 6 --speed 0.4 --delay 12
"""

from __future__ import annotations

import argparse
import math
import time

import gz.msgs10.entity_wrench_pb2 as entity_wrench_pb2
import gz.msgs10.pose_v_pb2 as pose_v_pb2
import gz.transport13 as gz_transport

PAD_LOCAL = (0.30, 0.0, 0.45)
G = 9.81

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from rclpy.node import Node as RclNode
    HAS_ROS = True
except Exception:  # noqa: BLE001
    HAS_ROS = False


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="x650_sea")
    ap.add_argument("--model", default="ship")
    ap.add_argument("--link", default="hull")
    ap.add_argument("--mass", type=float, default=300.0)
    ap.add_argument("--izz", type=float, default=480.0)
    ap.add_argument("--radius", type=float, default=6.0)
    ap.add_argument("--speed", type=float, default=0.4)
    ap.add_argument("--heave", type=float, default=0.10)
    ap.add_argument("--heave-period", type=float, default=5.0)
    ap.add_argument("--delay", type=float, default=12.0)
    ap.add_argument("--duration", type=float, default=240.0)
    ap.add_argument("--kp", type=float, default=2.5)
    ap.add_argument("--kd", type=float, default=4.0)
    ap.add_argument("--csv", default="")
    ap.add_argument("--no-ros", action="store_true")
    args = ap.parse_args()

    state = {"pose": None, "prev": None}

    def on_pose(msg) -> None:
        for p in msg.pose:
            if p.name == args.model:
                now = time.time()
                pos = (p.position.x, p.position.y, p.position.z)
                if state["pose"] is not None and state["prev"] is not None:
                    dt = now - state["prev"]
                    if dt > 1e-4:
                        old = state["pose"]
                        state["vel"] = tuple((pos[i] - old[i]) / dt for i in range(3))
                state["prev"] = now
                state["pose"] = pos
                state["yaw"] = yaw_of(p.orientation)

    node = gz_transport.Node()
    node.subscribe(pose_v_pb2.Pose_V, f"/world/{args.world}/pose/info", on_pose)
    wrench_pub = node.advertise(f"/world/{args.world}/wrench", entity_wrench_pb2.EntityWrench)

    rnode = pub_pose = pub_twist = None
    if HAS_ROS and not args.no_ros:
        rclpy.init()
        rnode = RclNode("ship_force_driver")
        pub_pose = rnode.create_publisher(PoseStamped, "/ship/pad_pose", 10)
        pub_twist = rnode.create_publisher(TwistStamped, "/ship/pad_twist", 10)
        print("ROS 话题已启用：/ship/pad_pose, /ship/pad_twist")

    omega = args.speed / max(args.radius, 1e-3)
    t0 = time.time()
    rows: list[tuple[float, ...]] = []
    next_tick = t0
    last_print = 0.0
    print(f"力驱动 {args.model}：R={args.radius} m  v={args.speed} m/s  "
          f"升沉 {args.heave} m/{args.heave_period}s  kp={args.kp} kd={args.kd}")

    try:
        while True:
            now = time.time()
            t = now - t0
            if t > args.duration:
                break
            if now < next_tick:
                time.sleep(min(next_tick - now, 0.002))
                continue
            next_tick += 0.05

            if state["pose"] is None:
                continue
            t_move = max(0.0, t - args.delay)
            yaw_des = omega * t_move
            x_des = args.radius * math.sin(yaw_des)
            y_des = args.radius * math.cos(yaw_des) - args.radius
            z_des = 0.36 + args.heave * math.sin(2 * math.pi * t / args.heave_period)
            vx_des = args.speed * math.cos(yaw_des)
            vy_des = -args.speed * math.sin(yaw_des)
            vz_des = args.heave * 2 * math.pi / args.heave_period * math.cos(
                2 * math.pi * t / args.heave_period)

            px, py, pz = state["pose"]
            vx, vy, vz = state.get("vel", (0.0, 0.0, 0.0))
            yaw = state.get("yaw", 0.0)
            dyaw = (yaw_des - yaw + math.pi) % (2 * math.pi) - math.pi

            fx = args.mass * (args.kp * (x_des - px) + args.kd * (vx_des - vx))
            fy = args.mass * (args.kp * (y_des - py) + args.kd * (vy_des - vy))
            fz = args.mass * (args.kp * (z_des - pz) + args.kd * (vz_des - vz) + G)
            tz = args.izz * (2.0 * dyaw - 3.0 * 0.0)

            w = entity_wrench_pb2.EntityWrench()
            w.entity.name = f"{args.model}::{args.link}"
            w.entity.type = 2                     # gz.msgs.Entity: 2 = LINK
            w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = fx, fy, fz
            w.wrench.torque.z = tz
            wrench_pub.publish(w)

            if pub_pose is not None and rnode is not None:
                pad_x = px + PAD_LOCAL[0] * math.cos(yaw)
                pad_y = py + PAD_LOCAL[0] * math.sin(yaw)
                pad_z = pz + PAD_LOCAL[2]
                stamp = rnode.get_clock().now().to_msg()
                ps = PoseStamped()
                ps.header.stamp = stamp
                ps.header.frame_id = "map"
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pad_x, pad_y, pad_z
                q = ps.pose.orientation
                q.z, q.w = math.sin(yaw / 2), math.cos(yaw / 2)
                ts = TwistStamped()
                ts.header.stamp = stamp
                ts.twist.linear.x, ts.twist.linear.y, ts.twist.linear.z = vx, vy, vz
                pub_pose.publish(ps)
                pub_twist.publish(ts)
                rows.append((t, pad_x, pad_y, pad_z, vx, vy, vz))

            if t - last_print >= 2.0:
                last_print = t
                print(f"  t={t:6.1f}s ship=({px:+.2f},{py:+.2f},{pz:.2f}) "
                      f"des=({x_des:+.2f},{y_des:+.2f},{z_des:.2f}) "
                      f"F=({fx:+.0f},{fy:+.0f},{fz:+.0f}) N", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if rnode is not None:
            rnode.destroy_node()
            rclpy.shutdown()

    if rows and args.csv:
        import csv
        with open(args.csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "pad_x", "pad_y", "pad_z", "vx", "vy", "vz"])
            w.writerows(rows)
        print(f"  CSV → {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
