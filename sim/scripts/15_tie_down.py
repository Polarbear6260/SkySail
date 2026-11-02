#!/usr/bin/env python3
"""15 · 着陆系留：判定已落在落点板上后，把机体固定到甲板随船移动

背景：船是用 set_pose 每 20 ms 瞬移的运动学刚体，物理引擎不会给它"拖住"落在上面的机体；
机体一旦上锁（桨停）就会被留在原地掉下去。真实舰船作业里，降落后也是要系留固定的，
所以这里做一个"系留"：一旦判定已落在板上，就每 50 ms 把机体位姿设成落点板的位置。

触发条件（用 Gazebo 真值）：
  · 机体在落点板圆心 0.8 m 内
  · 机体低于板面 +0.35 m 以上（即已经贴/压到板上）

用法：
  python3 scripts/15_tie_down.py --world x650_sea --csv logs/tiedown.csv
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import gz.msgs10.boolean_pb2 as bool_pb2
import gz.msgs10.pose_pb2 as pose_pb2
import gz.msgs10.pose_v_pb2 as pose_v_pb2
import gz.transport13 as gz_transport

PAD_LOCAL = (0.30, 0.0, 0.45)
SIT_HEIGHT = 0.30          # 机体原点相对板面的高度（起落架高度）


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="x650_sea")
    ap.add_argument("--drone", default="x650_0")
    ap.add_argument("--ship", default="ship")
    ap.add_argument("--duration", type=float, default=240.0)
    ap.add_argument("--horiz", type=float, default=0.80)
    ap.add_argument("--above", type=float, default=0.35)
    args = ap.parse_args()

    state: dict[str, object] = {"ship": None, "drone": None, "tied": False}

    def on_pose(msg) -> None:
        for p in msg.pose:
            if p.name == args.ship:
                state["ship"] = p
            elif p.name == args.drone:
                state["drone"] = p

    node = gz_transport.Node()
    node.subscribe(pose_v_pb2.Pose_V, f"/world/{args.world}/pose/info", on_pose)
    set_pose = node.advertise(f"/world/{args.world}/set_pose", pose_pb2.Pose)

    t0 = time.time()
    next_tick = t0
    announced = False
    print("系留监视中（落在板上后自动固定）")

    while True:
        now = time.time()
        if now - t0 > args.duration:
            break
        if now < next_tick:
            time.sleep(0.005)
            continue
        next_tick += 0.05

        ship, drone = state["ship"], state["drone"]
        if ship is None or drone is None:
            continue

        q = ship.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        px = ship.position.x + PAD_LOCAL[0] * math.cos(yaw)
        py = ship.position.y + PAD_LOCAL[0] * math.sin(yaw)
        pz = ship.position.z + PAD_LOCAL[2]

        horiz = math.hypot(drone.position.x - px, drone.position.y - py)
        above = drone.position.z - pz

        if not state["tied"]:
            if horiz < args.horiz and above < args.above:
                state["tied"] = True
                print(f"判定已落在板上（水平 {horiz:.2f} m，高于板面 {above:+.2f} m）→ 开始系留")
            else:
                continue

        # 系留：把机体放在落点板中心上方 SIT_HEIGHT，朝向跟船一致
        target = pose_pb2.Pose()
        target.name = args.drone
        target.position.x = px
        target.position.y = py
        target.position.z = pz + SIT_HEIGHT
        target.orientation.z = math.sin(yaw / 2)
        target.orientation.w = math.cos(yaw / 2)
        set_pose.publish(target)

        if not announced:
            announced = True
            print(f"系留生效：机体跟随落点板移动（水平 {horiz:.2f} m）")

    return 0


if __name__ == "__main__":
    sys.exit(main())
