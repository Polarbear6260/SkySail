#!/usr/bin/env python3
"""29 · 原始算法复算 vs Gazebo 实测

思路：
  1. 从 `28_traj_record.py` 的记录里取出每一拍的【状态】与【参考】
  2. 用**上游原始 LQR**（`UAV_Control`，不做任何外层补偿）重新算一遍加速度指令
  3. 两条轨迹对比：
     · 指令对比：`acc_alg` vs 记录的 `acc`  → 判断我们的外层是否忠实
     · 轨迹对比：用 `acc_alg` 做双积分器外推 vs Gazebo 实测轨迹 → 判断被控对象/内环的差距

用法：
  python3 scripts/29_replay_original.py --csv logs/traj.csv --outdir logs/replay
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

SKYSAIL = os.path.expanduser("~/planning_ws/src/SkySail")
if SKYSAIL not in sys.path:
    sys.path.insert(0, SKYSAIL)

from UAV_Control import AugmentedLQRController          # noqa: E402
from UAV_Control.UAV_params import LQRParams, UAVParams  # noqa: E402


def build_controller(phase: str, ref_speed: float, lateral: tuple[float, float, float]):
    uav = UAVParams(
        model_name="Holybro X650 (BOM)", mass_kg=3.5767,
        inertia_kg_m2=(0.1240, 0.1240, 0.2265), gravity_m_s2=9.81, dt_s=0.05,
        max_acceleration_m_s2=(3.0, 3.0, 2.5), min_acceleration_m_s2=(-3.0, -3.0, -2.5),
        max_tilt_rad=float(np.deg2rad(25.0)), max_descent_speed_m_s=1.0,
    )
    lqr = LQRParams(position_weight=(16.0, 16.0, 25.0),
                    velocity_weight=(4.0, 4.0, 9.0),
                    control_weight=(1.0, 1.0, 1.5),
                    preview_horizon=10, curvature_weight=1e-3,
                    tracking_velocity_scale=4.0)
    return AugmentedLQRController(uav=uav, lqr=lqr, reference_speed_m_s=max(ref_speed, 0.3),
                                  lateral_direction=lateral)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="logs/traj.csv")
    ap.add_argument("--outdir", default="logs/replay")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    rows = list(csv.DictReader(open(args.csv)))
    # 只比较 LQR 真正接管的阶段（起飞段用的是 PX4 位置环，不属于算法）
    lqr_states = {"transit", "approach", "descend", "settle"}
    rows = [r for r in rows if r["state"] in lqr_states]
    if len(rows) < 20:
        print(f"数据太少（{len(rows)} 行）")
        return 1

    t = np.array([float(r["t"]) for r in rows])
    pos = np.array([[float(r["px"]), float(r["py"]), float(r["pz"])] for r in rows])
    vel = np.array([[float(r["vx"]), float(r["vy"]), float(r["vz"])] for r in rows])
    rpos = np.array([[float(r["rpx"]), float(r["rpy"]), float(r["rpz"])] for r in rows])
    rvel = np.array([[float(r["rvx"]), float(r["rvy"]), float(r["rvz"])] for r in rows])
    acc_cmd = np.array([[float(r["ax"]), float(r["ay"]), float(r["az"])] for r in rows])
    phases = [r["state"] for r in rows]

    # 1) 用原始算法逐拍复算指令
    acc_alg = np.zeros_like(acc_cmd)
    ctrl = None
    last_phase = None
    for i in range(len(rows)):
        phase = "touchdown" if phases[i] in ("descend", "settle", "landed") else "tracking"
        ref_speed = float(np.linalg.norm(rvel[i][:2]))
        if ctrl is None or phase != last_phase:
            ctrl = build_controller(phase, ref_speed, (0.0, 1.0, 0.0))
            last_phase = phase
        err_p = pos[i] - rpos[i]
        err_v = vel[i] - rvel[i]
        state = np.concatenate([err_p, err_v])
        K, _ = ctrl.gains(phase)
        u = -K @ state
        lo = np.array([-3.0, -3.0, -2.5])
        hi = np.array([3.0, 3.0, 2.5])
        acc_alg[i] = np.clip(u, lo, hi)

    # 2) 闭环仿真：原始算法 + 理想双积分器，参考轨迹用记录下来的 pad 参考
    #    （开环积分一个反馈系统必然发散，所以必须闭环重放）
    dt = np.diff(t, prepend=t[0])
    dt[dt <= 0] = 1e-3
    sim_pos = np.zeros_like(pos)
    sim_vel = np.zeros_like(vel)
    sim_acc = np.zeros_like(acc_cmd)
    sim_pos[0], sim_vel[0] = pos[0], vel[0]
    ctrl = None
    last_phase = None
    for i in range(1, len(rows)):
        dt_i = float(min(dt[i], 0.2))
        phase = "touchdown" if phases[i] in ("descend", "settle", "landed") else "tracking"
        if ctrl is None or phase != last_phase:
            ctrl = build_controller(phase, float(np.linalg.norm(sim_vel[i - 1][:2])), (0.0, 1.0, 0.0))
            last_phase = phase
        err_p = sim_pos[i - 1] - rpos[i - 1]
        err_v = sim_vel[i - 1] - rvel[i - 1]
        K, _ = ctrl.gains(phase)
        u = -K @ np.concatenate([err_p, err_v])
        u = np.clip(u, np.array([-3.0, -3.0, -2.5]), np.array([3.0, 3.0, 2.5]))
        sim_acc[i - 1] = u
        sim_vel[i] = sim_vel[i - 1] + u * dt_i
        sim_pos[i] = sim_pos[i - 1] + sim_vel[i - 1] * dt_i + 0.5 * u * dt_i ** 2

    # 3) 指标
    e_cmd = np.linalg.norm(acc_alg - acc_cmd, axis=1)
    e_traj = np.linalg.norm(sim_pos - pos, axis=1)
    e_traj_h = np.linalg.norm((sim_pos - pos)[:, :2], axis=1)
    print(f"样本 {len(rows)} 拍，时长 {t[-1]:.1f} s")
    print(f"[指令] 原始算法 vs 我们发出的：RMS {np.sqrt(np.mean(e_cmd**2)):.2f} m/s², "
          f"最大 {e_cmd.max():.2f}")
    print(f"[轨迹] 原始算法闭环 vs Gazebo：末端 {e_traj[-1]:.2f} m, "
          f"最大 {e_traj.max():.2f} m, RMS {np.sqrt(np.mean(e_traj**2)):.2f} m")
    print(f"        水平分量：末端 {e_traj_h[-1]:.2f} m, 最大 {e_traj_h.max():.2f} m")

    # 4) 出图
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 2, figsize=(12, 7))
        ax[0, 0].plot(pos[:, 0], pos[:, 1], label="Gazebo")
        ax[0, 0].plot(sim_pos[:, 0], sim_pos[:, 1], "--", label="original algorithm")
        ax[0, 0].plot(rpos[:, 0], rpos[:, 1], ":", label="reference")
        ax[0, 0].set_xlabel("N [m]"); ax[0, 0].set_ylabel("E [m]")
        ax[0, 0].legend(); ax[0, 0].set_title("horizontal trajectory")
        ax[0, 1].plot(t, pos[:, 2], label="Gazebo z")
        ax[0, 1].plot(t, sim_pos[:, 2], "--", label="algorithm z")
        ax[0, 1].plot(t, rpos[:, 2], ":", label="ref z")
        ax[0, 1].set_xlabel("t [s]"); ax[0, 1].legend(); ax[0, 1].set_title("vertical")
        ax[1, 0].plot(t, e_cmd); ax[1, 0].set_xlabel("t [s]")
        ax[1, 0].set_ylabel("|Δacc| [m/s²]"); ax[1, 0].set_title("command difference")
        ax[1, 1].plot(t, e_traj); ax[1, 1].set_xlabel("t [s]")
        ax[1, 1].set_ylabel("[m]"); ax[1, 1].set_title("trajectory difference")
        plt.tight_layout()
        out = os.path.join(args.outdir, "replay_compare.png")
        plt.savefig(out, dpi=120)
        print(f"图 → {out}")
    except Exception as exc:  # noqa: BLE001
        print(f"绘图失败：{exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
