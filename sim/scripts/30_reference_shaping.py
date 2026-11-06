#!/usr/bin/env python3
"""30 · 用原始算法验证：阶跃参考 vs 限速参考

同一个原始 LQR、同一个初始状态、同一架"理想双积分器"被控对象，
只改参考高度是否限速，比较下降速度。

用法：python3 scripts/30_reference_shaping.py --csv logs/traj-<ts>.csv
"""
from __future__ import annotations
import argparse, csv, os, sys
import numpy as np

SKYSAIL = os.path.expanduser("~/planning_ws/src/SkySail")
sys.path.insert(0, SKYSAIL)
from UAV_Control import AugmentedLQRController           # noqa: E402
from UAV_Control.UAV_params import LQRParams, UAVParams   # noqa: E402


def build(phase="tracking"):
    uav = UAVParams(model_name="X650", mass_kg=3.5767, inertia_kg_m2=(0.124, 0.124, 0.2265),
                    gravity_m_s2=9.81, dt_s=0.05,
                    max_acceleration_m_s2=(3.0, 3.0, 2.5),
                    min_acceleration_m_s2=(-3.0, -3.0, -2.5),
                    max_tilt_rad=float(np.deg2rad(25.0)), max_descent_speed_m_s=1.0)
    lqr = LQRParams(position_weight=(16.0, 16.0, 25.0), velocity_weight=(4.0, 4.0, 9.0),
                    control_weight=(1.0, 1.0, 1.5), preview_horizon=10,
                    curvature_weight=1e-3, tracking_velocity_scale=4.0)
    return AugmentedLQRController(uav=uav, lqr=lqr, reference_speed_m_s=1.0,
                                  lateral_direction=(0.0, 1.0, 0.0))


def simulate(t, ref_p, ref_v, start_p, start_v, ctrl, phase="tracking"):
    n = len(t)
    p = np.zeros((n, 3)); v = np.zeros((n, 3))
    p[0], v[0] = start_p, start_v
    K, _ = ctrl.gains(phase)
    lo = np.array([-3.0, -3.0, -2.5]); hi = -lo
    for i in range(1, n):
        dt = float(min(t[i] - t[i - 1], 0.2))
        u = np.clip(-K @ np.concatenate([p[i - 1] - ref_p[i - 1], v[i - 1] - ref_v[i - 1]]), lo, hi)
        v[i] = v[i - 1] + u * dt
        p[i] = p[i - 1] + v[i - 1] * dt + 0.5 * u * dt ** 2
    return p, v


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="")
    ap.add_argument("--rate", type=float, default=1.0, help="限速剖面 m/s")
    args = ap.parse_args()
    path = args.csv or sorted(__import__("glob").glob("logs/traj-*.csv"))[-1]
    rows = [r for r in csv.DictReader(open(path)) if r["state"] in
            {"transit", "approach", "descend", "settle"}]
    t = np.array([float(r["t"]) for r in rows]) - float(rows[0]["t"])
    ref_p = np.array([[float(r["rpx"]), float(r["rpy"]), float(r["rpz"])] for r in rows])
    ref_v = np.array([[float(r["rvx"]), float(r["rvy"]), float(r["rvz"])] for r in rows])

    # 限速版本：只改参考 z，按 args.rate 收敛
    ref_p_slow = ref_p.copy()
    for i in range(1, len(t)):
        dt = float(min(t[i] - t[i - 1], 0.2))
        step = args.rate * dt
        ref_p_slow[i, 2] = ref_p_slow[i - 1, 2] + float(np.clip(ref_p[i, 2] - ref_p_slow[i - 1, 2], -step, step))

    ctrl = build()
    p_raw, v_raw = simulate(t, ref_p, ref_v, ref_p[0], ref_v[0], ctrl)
    p_slow, v_slow = simulate(t, ref_p_slow, ref_v, ref_p_slow[0], ref_v[0], ctrl)

    def report(name, v, p, rp):
        vz = np.abs(v[:, 2])
        i = int(np.argmax(vz))
        err = np.linalg.norm(p - rp, axis=1)
        print(f"{name}: 最大下降速度 {vz.max():4.2f} m/s (t={t[i]:.1f}s), "
              f"跟踪误差 平均 {err.mean():.2f} m / 最大 {err.max():.2f} m")

    print(f"参考文件 {path}，{len(rows)} 拍，{t[-1]:.1f} s")
    report("阶跃参考（现状）", v_raw, p_raw, ref_p)
    report(f"限速参考({args.rate} m/s)", v_slow, p_slow, ref_p_slow)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
