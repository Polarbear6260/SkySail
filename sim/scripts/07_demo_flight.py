#!/usr/bin/env python3
"""
07 · 自动演示飞行（无头 / 无需 QGC）

对已在运行的 SITL 做一套标准动作，用于产出汇报素材与 ulog：

    设 NAV_DLL_ACT=0 → 解锁 → AUTO.TAKEOFF 到 3 m → 悬停 20 s → AUTO.LAND

用法（先另开一个终端把仿真跑起来）：
    ./scripts/04_run_sitl.sh            # 终端 A
    ./scripts/07_demo_flight.py         # 终端 B

结束后用 ./scripts/06_analyze_ulog.py 出统计表。

依赖：pymavlink
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavutil

TAKEOFF_ALT = 3.0
HOVER_S = 20.0


def run_shell(command: str, timeout: float = 15.0) -> bool:
    """在 PX4 shell 里执行一条命令（无头环境下最可靠的指令通道）。"""
    px4_dir = Path(os.environ.get("PX4_DIR", Path.home() / "PX4-Autopilot"))
    py = px4_dir / ".venv-px4" / "bin" / "python"
    tool = px4_dir / "Tools" / "mavlink_shell.py"
    if not py.is_file() or not tool.is_file():
        print("    ! 找不到 venv 或 mavlink_shell.py")
        return False
    try:
        subprocess.run([str(py), str(tool), "udpin:127.0.0.1:14540"],
                       input=command + "\n", text=True, capture_output=True,
                       timeout=timeout)
        return True
    except Exception as exc:  # noqa: BLE001
        print(f"    ! shell 失败：{exc}")
        return False


def set_param_via_shell(name: str, value: float) -> bool:
    """用 PX4 自带的 mavlink_shell 设参数。

    实测：pymavlink 直连 udpin:14540 能收到心跳，但 param_set / command_long
    在无头 SITL 下不被 PX4 处理（实例只对已建立的链路回执），
    而 mavlink_shell 这条路是稳的（04 --smoke 也用它）。
    """
    px4_dir = Path(os.environ.get("PX4_DIR", Path.home() / "PX4-Autopilot"))
    py = px4_dir / ".venv-px4" / "bin" / "python"
    tool = px4_dir / "Tools" / "mavlink_shell.py"
    if not py.is_file() or not tool.is_file():
        return False
    cmd = f"param set {name} {value}\n"
    try:
        subprocess.run([str(py), str(tool), "udpin:127.0.0.1:14540"],
                       input=cmd, text=True, capture_output=True, timeout=15)
        return True
    except Exception as exc:  # noqa: BLE001
        print(f"    ! shell 设参失败：{exc}")
        return False


def set_param(mav, name: str, value: float) -> None:
    mav.mav.param_set_send(1, 1, name.encode(), float(value),
                           mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    t0 = time.time()
    while time.time() - t0 < 3:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id == name:
            print(f"    {name} = {msg.param_value}")
            return
    print(f"    {name} 设置无回执（继续）")


def set_mode(mav, mode: str, timeout: float = 5.0) -> bool:
    mapping = mav.mode_mapping() or {}
    if mode not in mapping:
        print(f"    ! 不支持模式 {mode}", file=sys.stderr)
        return False
    mav.set_mode(mapping[mode])
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg and msg.custom_mode == mapping[mode]:
            print(f"    模式 → {mode}")
            return True
    print(f"    ! 切模式 {mode} 超时", file=sys.stderr)
    return False


def wait_armed(mav, timeout: float = 10.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def altitude(mav) -> float:
    msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
    return -msg.z if msg else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="udpin:127.0.0.1:14550")
    ap.add_argument("--alt", type=float, default=TAKEOFF_ALT)
    ap.add_argument("--hover", type=float, default=HOVER_S)
    args = ap.parse_args()

    print("连接 SITL ...")
    mav = mavutil.mavlink_connection(args.conn, source_system=255)
    hb = mav.wait_heartbeat(timeout=30)
    # SITL 里可能先收到别的 sysid（比如 0），显式锁定到心跳来源，避免指令发给 0 号
    mav.target_system = hb.get_srcSystem() if hb else 1
    mav.target_component = 1
    print(f"  心跳 OK，sysid={mav.target_system}")

    print("1) 放开 GCS 依赖（无头测试专用）")
    if not set_param_via_shell("NAV_DLL_ACT", 0):
        set_param(mav, "NAV_DLL_ACT", 0)

    print("2) 解锁")
    mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                              0, 1, 0, 0, 0, 0, 0, 0)
    if not wait_armed(mav, timeout=6):
        # PX4 的 force-arm 魔法值：跳过预检（仅仿真演示用；真机绝不可用）
        print("    常规解锁被拒，改用 force-arm（仿真演示专用）")
        mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                  0, 1, 21196, 0, 0, 0, 0, 0)
        if not wait_armed(mav, timeout=6):
            print("  ✘ 解锁失败：先看 04 的日志", file=sys.stderr)
            return 1
    print("  ✔ 已解锁")

    print(f"3) commander takeoff（PX4 shell）")
    if not run_shell("commander takeoff", timeout=20):
        return 1

    peak = 0.0
    t0 = time.time()
    while time.time() - t0 < 40:
        z = altitude(mav)
        if z == z:
            peak = max(peak, z)
            if z > args.alt - 0.5:
                break
        time.sleep(0.2)
    print(f"  ✔ 起飞到 {peak:.2f} m（目标 ≥ {args.alt - 0.5:.1f} m）")

    print(f"4) 悬停 {args.hover:.0f} s")
    t0 = time.time()
    zs = []
    while time.time() - t0 < args.hover:
        z = altitude(mav)
        if z == z:
            zs.append(z)
        time.sleep(0.2)
    if zs:
        print(f"  高度均值 {sum(zs) / len(zs):.2f} m，波动 {max(zs) - min(zs):.3f} m")

    print("5) commander land（PX4 shell）")
    run_shell("commander land", timeout=20)
    t0 = time.time()
    while time.time() - t0 < 40:
        z = altitude(mav)
        if z == z and z < 0.3:
            print("  ✔ 已落地")
            break
        time.sleep(0.5)

    print("\n完成。用 ./scripts/06_analyze_ulog.py 看悬停油门与高度保持。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
