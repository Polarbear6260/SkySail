#!/usr/bin/env python3
"""09 · 物理参数估算与出处（BOM 可推导部分）

输出三样东西，用于替换仿真模型里的“拍脑袋”参数：
  1. 分部件转动惯量 + 质心（box/cylinder 平行轴定理加总）
  2. 电机推力曲线拟合（对 T-Motor 台架 5 点），对比三种约束
  3. 每个参数的来源与不确定度

用法：  ./scripts/09_estimate_params.py [--apply]
        --apply 时会把惯量与质心写回 gz_extras/models/x650_base/model.sdf
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
G = 9.81
M_TOTAL_G = 3576.7

# 部件：(名称, 质量 g, 位置 m, 尺寸 m 或 None=点质量)
# 质量拆分依据 BOM 分类：裸机 3115.5 g / 计算载荷 331.2 g / 装配余量 130 g
COMPONENTS = [
    ("机架结构(碳板+机臂+脚架)", 1100, (0.0, 0.0, -0.02), (0.65, 0.65, 0.28)),
    ("电机×4 @旋翼位",            520, None, None),
    ("桨×4 @旋翼位",              100, None, None),
    ("电调×4",                     40, None, None),
    ("PDB+线材",                  240, (0.0, 0.0, 0.00), (0.30, 0.30, 0.03)),
    ("飞控 V6X",                   99, (0.0, 0.0, 0.045), (0.09, 0.05, 0.03)),
    ("PMU",                        15, (0.05, 0.0, 0.02), (0.05, 0.03, 0.02)),
    ("GPS NEO4(含支架)",         36.5, (0.05, 0.0, 0.10), (0.06, 0.06, 0.02)),
    ("数传(空端)",                 30, (0.0, 0.06, 0.03), (0.06, 0.03, 0.02)),
    ("接收机",                      5, (0.0, 0.04, 0.05), None),
    ("电池 6S 6800",              930, (0.0, 0.0, -0.075), (0.155, 0.050, 0.045)),
    ("Orin 载板",                39.2, (0.05, 0.0, -0.03), (0.10, 0.07, 0.02)),
    ("电解电容",                    4, (0.09, 0.0, -0.03), None),
    ("Orin Nano 模组",             90, (0.05, 0.0, -0.02), (0.07, 0.05, 0.02)),
    ("Gemini 335L",               133, (0.16, 0.0, -0.04), (0.124, 0.030, 0.028)),
    ("散热模组",                   45, (0.05, 0.0, 0.00), (0.08, 0.06, 0.03)),
    ("WiFi 模块",                  12, (0.0, 0.03, -0.02), None),
    ("SSD",                         8, (0.05, 0.0, -0.02), None),
    ("装配余量(线材/螺丝)",        130, (0.0, 0.0, 0.0), (0.30, 0.30, 0.05)),
]
ROTORS = [(0.2298, -0.2298), (0.2298, 0.2298), (-0.2298, 0.2298), (-0.2298, -0.2298)]
ROTOR_Z = 0.06

BENCH = np.array([[0.50, 830], [0.65, 1150], [0.75, 1430], [0.85, 1690], [1.00, 1920]], float)


def mass_properties():
    items, m_sum, cg = [], 0.0, np.zeros(3)
    for name, m_g, pos, dims in COMPONENTS:
        if pos is None:                      # 旋翼位上的 4 个部件
            per = m_g / len(ROTORS)
            for x, y in ROTORS:
                p = np.array([x, y, ROTOR_Z])
                items.append((per / 1000, p, None))
                m_sum += per / 1000
                cg += per / 1000 * p
        else:
            p = np.array(pos)
            items.append((m_g / 1000, p, dims))
            m_sum += m_g / 1000
            cg += m_g / 1000 * p
    cg /= m_sum
    I = np.zeros((3, 3))
    for m, p, dims in items:
        r = p - cg
        I += m * ((r @ r) * np.eye(3) - np.outer(r, r))       # 平行轴
        if dims:
            dx, dy, dz = dims
            I += m / 12 * np.diag([dy**2 + dz**2, dx**2 + dz**2, dx**2 + dy**2])
    return m_sum, cg, I


def motor_fit(mode: str):
    """mode: 'endpoint' 满推力精确 | 'free3' 三参数无约束 | 'weighted' 悬停段加权"""
    w = np.array([3.0, 3.0, 1.0, 0.4, 1.0])
    best = None
    emin_grid = np.arange(0, 200, 2.0)
    emax_grid = np.arange(400, 560, 0.5)
    for emax in emax_grid:
        for emin in emin_grid:
            if emin >= emax:
                continue
            if mode == "free3":
                res = _opt_k(emin, emax, None)
                e, k = res
            elif mode == "weighted":
                res = _opt_k(emin, emax, 18.835 / emax**2, w)
                e, k = res
            else:
                res = _opt_k(emin, emax, 18.835 / emax**2)
                e, k = res
            if best is None or e < best[0]:
                best = (e, emin, emax, k)
    return best


def _opt_k(emin, emax, k_fixed, weights=None):
    if k_fixed is None:                      # k 也放开：解析最优
        num = den = 0.0
        for u, T in BENCH:
            om = emin + u * (emax - emin)
            target = G * T / 1000
            num += target / om**2
            den += 1.0
        k = num / den
    else:
        k = k_fixed
    e = 0.0
    for i, (u, T) in enumerate(BENCH):
        om = emin + u * (emax - emin)
        err = k * om * om / (G * T / 1000) - 1
        e += (weights[i] if weights is not None else 1.0) * err**2
    return e, k


def hover_throttle(k, emin, emax):
    need = M_TOTAL_G / 1000 * G / 4
    return (np.sqrt(need / k) - emin) / (emax - emin)


def apply_to_sdf(inertia, cg):
    p = ROOT / "gz_extras/models/x650_base/model.sdf"
    s = p.read_text()
    s = re.sub(r"<mass>[\d.]+</mass>", f"<mass>{M_TOTAL_G / 1000 - 4 * 0.09:.4f}</mass>", s, count=1)
    s = re.sub(r"<ixx>[\d.]+</ixx>", f"<ixx>{inertia[0, 0]:.4f}</ixx>", s, count=1)
    s = re.sub(r"<iyy>[\d.]+</iyy>", f"<iyy>{inertia[1, 1]:.4f}</iyy>", s, count=1)
    s = re.sub(r"<izz>[\d.]+</izz>", f"<izz>{inertia[2, 2]:.4f}</izz>", s, count=1)
    pose = f"<pose>{cg[0]:.4f} {cg[1]:.4f} {cg[2]:.4f} 0 0 0</pose>"
    if "<inertial>" in s and "<pose>" in s.split("<inertial>")[1].split("</inertial>")[0]:
        s = re.sub(r"(<inertial>\s*)<pose>[^<]*</pose>", rf"\1{pose}", s, count=1)
    else:
        s = s.replace("<inertial>", f"<inertial>\n        {pose}", 1)
    p.write_text(s)
    print(f"已写回 {p.relative_to(ROOT)}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true")
    args = ap.parse_args()

    m, cg, I = mass_properties()
    print("=" * 66)
    print("1) 质量 / 质心 / 惯量（分部件，平行轴定理）")
    print("=" * 66)
    print(f"  总质量 {m * 1000:.1f} g（BOM {M_TOTAL_G:.1f} g）")
    print(f"  质心  ({cg[0]*100:+.2f}, {cg[1]*100:+.2f}, {cg[2]*100:+.2f}) cm")
    print(f"  Ixx={I[0,0]:.4f}  Iyy={I[1,1]:.4f}  Izz={I[2,2]:.4f} kg·m²")
    print(f"  旧值（箱体） 0.1240 / 0.1240 / 0.2265 → 变化 "
          f"{100*(I[0,0]/0.1240-1):+.0f}% / {100*(I[1,1]/0.1240-1):+.0f}% / {100*(I[2,2]/0.2265-1):+.0f}%")

    print()
    print("=" * 66)
    print("2) 电机推力曲线拟合（T-Motor 台架 5 点）")
    print("=" * 66)
    rows = []
    for mode, label in (("endpoint", "满推力精确"), ("weighted", "悬停段加权"), ("free3", "三参数无约束")):
        e, emin, emax, k = motor_fit(mode)
        errs = []
        for u, T in BENCH:
            om = emin + u * (emax - emin)
            errs.append((k * om * om / (G * T / 1000) - 1) * 100)
        rows.append((label, emin, emax, k, np.sqrt(e) * 100, max(abs(x) for x in errs),
                     hover_throttle(k, emin, emax) * 100, errs))
    print(f"  {'方案':<12}{'EC_MIN':>7}{'EC_MAX':>8}{'k':>13}{'RMSE%':>7}{'最大偏差%':>10}{'悬停油门%':>10}")
    for label, emin, emax, k, rmse, mx, hv, _ in rows:
        print(f"  {label:<12}{emin:>7.0f}{emax:>8.0f}{k:>13.4e}{rmse:>7.2f}{mx:>10.2f}{hv:>10.2f}")
    for label, _, _, _, _, _, _, errs in rows:
        print(f"  {label:<12} 50/65/75/85/100% 偏差: " + " ".join(f"{x:+5.1f}" for x in errs))

    print()
    print("  结论：三种方案最大偏差都在 4–8%，因为 gz 的电机模型固定为 T=k·ω²（只有两个自由度），")
    print("  无法拟合台架曲线的真实形状。当前模型保留「满推力精确」方案，")
    print("  并把悬停段（我们实际飞行区间）的偏差作为主要口径。")

    if args.apply:
        apply_to_sdf(I, cg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
