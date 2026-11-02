#!/usr/bin/env python3
"""
06 · ULog 成果分析（给汇报/验收用）

从 PX4 SITL 的 ulog 里提取可写进汇报的数字：
  · 仿真时长、EKF 收敛标志、预检是否通过
  · 磁强实测 vs WMM 参考（验证 world 的 spherical_coordinates 生效）
  · 悬停段：平均油门（对比 BOM 的 53%）、高度保持偏差
  · 电池电压

用法：
  ./scripts/06_analyze_ulog.py                    # 自动取最新 ulog
  ./scripts/06_analyze_ulog.py path/to/file.ulg

依赖：pyulog（pip install pyulog；或用 02_build_sitl.sh 建的 venv）
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
from pathlib import Path

import numpy as np

try:
    from pyulog import ULog
except ImportError:  # pragma: no cover
    print("缺 pyulog：source ~/PX4-Autopilot/.venv-px4/bin/activate && "
          "pip install pyulog -i https://pypi.tuna.tsinghua.edu.cn/simple", file=sys.stderr)
    raise SystemExit(2)

PX4_DIR = Path(os.environ.get("PX4_DIR", Path.home() / "PX4-Autopilot"))
LOG_GLOB = str(PX4_DIR / "build" / "px4_sitl_default" / "rootfs" / "log" / "*" / "*.ulg")


def latest_ulog() -> Path:
    files = sorted(glob.glob(LOG_GLOB), key=os.path.getmtime)
    if not files:
        raise SystemExit(f"找不到 ulog：{LOG_GLOB}")
    return Path(files[-1])


def hovering_mask(z: np.ndarray, vz: np.ndarray) -> np.ndarray:
    return (z > 0.5) & (np.abs(vz) < 0.3)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("ulog", nargs="?")
    ap.add_argument("--min-hover-s", type=float, default=2.0,
                    help="悬停段最短持续时间（秒），不足则跳过统计")
    args = ap.parse_args()

    path = Path(args.ulog) if args.ulog else latest_ulog()
    u = ULog(str(path))
    D = {d.name: d.data for d in u.data_list}

    print("=" * 68)
    print(f"ULog 分析：{path.name}")
    print("=" * 68)
    print(f"  时长        {u.last_timestamp / 1e6:6.1f} s")
    print(f"  消息类型    {len(D):6d}")

    # ---- 预检 / EKF ----
    vs = D.get("vehicle_status")
    if vs is not None:
        pf = vs["pre_flight_checks_pass"]
        print(f"  预检通过    {'是' if pf.any() else '否'}（占比 {pf.mean() * 100:.0f}%）")
        print(f"  解锁状态    末值 arming_state={int(vs['arming_state'][-1])}"
              f"（0=disarmed, 2=armed）")

    es = D.get("estimator_status")
    if es is not None:
        for k, label in (("mag_strength_gs", "磁强实测 (G)"),
                         ("mag_strength_ref_gs", "磁强参考 (G)"),
                         ("mag_inclination_deg", "磁倾角实测 (°)"),
                         ("mag_inclination_ref_deg", "磁倾角参考 (°)")):
            if k in es:
                print(f"  {label:14s}{es[k][-1]:8.4f}")
        if "mag_strength_gs" in es and "mag_strength_ref_gs" in es:
            ratio = es["mag_strength_gs"][-1] / es["mag_strength_ref_gs"][-1]
            ok = abs(ratio - 1.0) < 0.2
            print(f"  磁强偏差    {abs(ratio - 1) * 100:5.1f}%  "
                  f"{'✔ 通过 (<20%)' if ok else '✘ 超差，检查 world 的 spherical_coordinates'}")

    bs = D.get("battery_status")
    if bs is not None and "voltage_v" in bs:
        print(f"  电池电压    {bs['voltage_v'][-1]:6.2f} V")

    # ---- 飞行段 ----
    lp = D.get("vehicle_local_position")
    am = D.get("actuator_motors")
    if lp is None or am is None:
        print("  没有位置/电机数据，跳过飞行统计")
        return 0

    z = lp["z"]          # NED：向上为负
    vz = lp["vz"]
    t = lp["timestamp"] / 1e6
    mask = hovering_mask(-z, vz)
    if mask.sum() * 0.004 < args.min_hover_s:      # 位置按 ~250 Hz 记录
        print("  未检测到悬停段（样本不足）：本次可能没有起飞")
        if len(z):
            print(f"  最大离地高度 {(-z).max():5.2f} m")
        return 0

    idx = np.flatnonzero(mask)
    t_h = t[idx]
    print()
    print("  悬停段统计")
    print(f"    悬停时长    {t_h[-1] - t_h[0]:6.1f} s")
    print(f"    平均高度    {(-z[idx]).mean():6.2f} m（标准差 {(-z[idx]).std():.3f} m）")
    print(f"    高度漂移    {(-z[idx]).max() - (-z[idx]).min():6.3f} m")

    # 电机数据与位置数据采样率不同，按时间窗对齐（不能直接用同一个下标）
    t_am = am["timestamp"] / 1e6
    sel = (t_am >= t_h[0]) & (t_am <= t_h[-1])
    ctrl = [am[f"control[{i}]"][sel] for i in range(4) if f"control[{i}]" in am]
    if ctrl and sel.any():
        hover = float(np.mean([c.mean() for c in ctrl]))
        print(f"    悬停油门    {hover * 100:6.2f} %")
        print(f"    BOM 设计值   53.00 %   偏差 {(hover - 0.53) * 100:+.2f} 个百分点")
        verdict = "✔ 落在 50–56% 区间" if 0.50 <= hover <= 0.56 else "✘ 偏离，按 docs/01 §3.2 重标定"
        print(f"    判定          {verdict}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
