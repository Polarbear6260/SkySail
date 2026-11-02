#!/usr/bin/env python3
"""17 · 移动着陆调试面板（GUI，实时）

同时显示：
  · 俯视图：无人机轨迹 / 落点板轨迹 / 预测参考点（世界坐标）
  · 误差曲线：真值水平误差、相对板面高度、相对速度
  · 状态面板：状态机、预测视界、下降前馈、加速度指令、armed/nav_state

数据来源：
  /x650/debug                 控制器内部状态（JSON）
  /fmu/out/vehicle_local_position_v1   PX4 位置（本地 NED）
  /ship/pad_pose, /ship/pad_twist    平台位姿（ENU）

用法：
  python3 scripts/17_debug_dashboard.py
  python3 scripts/17_debug_dashboard.py --save logs/dashboard.png   # 跑完存图

依赖：rclpy、matplotlib（TkAgg）
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import deque

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from geometry_msgs.msg import PoseStamped, TwistStamped  # noqa: E402
from px4_msgs.msg import VehicleLocalPosition  # noqa: E402

WINDOW = 400          # 曲线保留点数


def fmu_qos() -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


class Dashboard(Node):
    def __init__(self) -> None:
        super().__init__("x650_dashboard")
        self.dbg = {}
        self.pad = None            # ENU
        self.pad_l = None
        self.trail_drone: deque = deque(maxlen=1200)
        self.trail_pad: deque = deque(maxlen=1200)
        self.t = deque(maxlen=WINDOW)
        self.horiz = deque(maxlen=WINDOW)
        self.above = deque(maxlen=WINDOW)
        self.relv = deque(maxlen=WINDOW)

        self.create_subscription(String, "/x650/debug", self.on_dbg, 10)
        self.create_subscription(PoseStamped, "/ship/pad_pose", self.on_pad, 10)
        self.create_subscription(TwistStamped, "/ship/pad_twist", self.on_pad_twist, 10)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
                                 self.on_pos, fmu_qos())

    def on_dbg(self, m: String) -> None:
        try:
            self.dbg = json.loads(m.data)
        except json.JSONDecodeError:
            return
        t = self.dbg.get("t", 0.0)
        h = self.dbg.get("true_horiz")
        a = self.dbg.get("true_above")
        pv = self.dbg.get("pad_vel") or [0, 0, 0]
        pos = self.dbg.get("pos_ned")
        self.t.append(t)
        self.horiz.append(h if h is not None else float("nan"))
        self.above.append(a if a is not None else float("nan"))
        pos_enu = self.dbg.get("pos_enu")
        if pos_enu is not None:
            self.trail_drone.append((pos_enu[0], pos_enu[1]))
        if pos is not None:
            self.relv.append(math.dist(pos, self.dbg.get("pad_ned") or pos))

    def on_pad(self, m: PoseStamped) -> None:
        self.pad = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        self.trail_pad.append((self.pad[0], self.pad[1]))

    def on_pad_twist(self, m: TwistStamped) -> None:
        self.pad_l = (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z)

    def on_pos(self, m: VehicleLocalPosition) -> None:
        pass            # 位置来自 debug 的 pos_ned；这里保留订阅以备扩展


def fmt(v) -> str:
    return "None" if v is None else f"{float(v):+.2f}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", default="", help="退出时把面板存成图片")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Dashboard()

    fig = plt.figure(figsize=(13, 7.5))
    fig.canvas.manager.set_window_title("X650 moving-landing debug dashboard")
    ax_xy = fig.add_subplot(1, 2, 1)
    ax_err = fig.add_subplot(2, 2, 2)
    ax_txt = fig.add_subplot(2, 2, 4)
    ax_txt.axis("off")

    def draw(_frame):
        node.dbg  # keep reference
        ax_xy.clear()
        ax_xy.set_title("Top view (world ENU)")
        ax_xy.set_xlabel("x [m]")
        ax_xy.set_ylabel("y [m]")
        ax_xy.grid(alpha=0.3)
        if node.trail_pad:
            px, py = zip(*node.trail_pad)
            ax_xy.plot(px, py, "r-", lw=1.2, label="pad track")
            ax_xy.plot(px[-1], py[-1], "ro", ms=8)
        if node.trail_drone:
            dx, dy = zip(*node.trail_drone)
            ax_xy.plot(dx, dy, "b-", lw=1.2, label="UAV")
            ax_xy.plot(dx[-1], dy[-1], "b^", ms=8)
        dbg = node.dbg
        if dbg.get("ref_pos") and dbg.get("pad_ned"):
            # 参考点是 NED（相对起飞点），这里叠加到最后一个板位置附近仅作提示
            ax_xy.plot([], [], "g+", label=f"pred horizon {dbg.get('horizon', 0):.2f}s")
        ax_xy.legend(loc="upper right", fontsize=8)
        ax_xy.set_aspect("equal", adjustable="datalim")

        ax_err.clear()
        ax_err.set_title("Errors (ground truth)")
        if node.t:
            tt = list(node.t)
            ax_err.plot(tt, list(node.horiz), "b-", label="horizontal error [m]")
            ax_err.plot(tt, list(node.above), "g-", label="height above pad [m]")
        ax_err.grid(alpha=0.3)
        ax_err.legend(fontsize=8, loc="upper right")

        ax_txt.clear()
        ax_txt.axis("off")
        state = dbg.get("state", "-")
        acc = dbg.get("acc") or [0, 0, 0]
        lines = [
            f"state      : {state}",
            f"armed      : {dbg.get('armed')}   nav_state={dbg.get('nav_state')}",
            f"horizon    : {dbg.get('horizon', 0):.2f} s",
            f"descent ff : {dbg.get('descent_rate', 0):.2f} m/s",
            f"acc cmd    : ({acc[0]:+.2f}, {acc[1]:+.2f}, {acc[2]:+.2f}) m/s2",
            f"horiz err  : {fmt(dbg.get('true_horiz'))} m",
            f"above pad  : {fmt(dbg.get('true_above'))} m",
            f"pad vel    : {tuple(round(v, 2) for v in (dbg.get('pad_vel') or [0, 0, 0]))} m/s",
            f"ref vel    : {tuple(round(v, 2) for v in (dbg.get('ref_vel') or [0, 0, 0]))} m/s",
        ]
        ax_txt.text(0.0, 0.95, "\n".join(lines), va="top", family="monospace", fontsize=10)

    _anim = FuncAnimation(fig, draw, interval=200, cache_frame_data=False)
    try:
        import threading

        def spin():
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)

        threading.Thread(target=spin, daemon=True).start()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        if args.save:
            fig.savefig(args.save, dpi=120)
            print(f"面板已保存：{args.save}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
