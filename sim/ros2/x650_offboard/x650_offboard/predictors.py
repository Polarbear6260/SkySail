#!/usr/bin/env python3
"""移动平台着陆的参考预测器（不需要真值，只用观测历史）

上游 `MIONetLandingController` 需要一个 `ReferencePredictor`：
    predictor(observation) -> LandingReference(position, velocity, curvature_preview)

这里实现一个**不含模型、只用观测历史**的预测器，作为 MIONet 的占位/基线：

  · 位置：用最近 N 帧（默认 1 s）做最小二乘，得到速度与加速度
  · 外推：在 `horizon` 秒后的落点位置/速度（恒速 + 半加速度项）
  · 曲率：由艏向角速度 ω 与速度 v 估计 κ = ω / v（平面缓弯）
  · preview：把 κ 复制成 `preview_horizon` 长度的序列（假设曲率不变）

后续把 MIONet 接进来时，只要替换这个类即可（接口一致）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np


@dataclass
class PadHistory:
    """控制器坐标系（PX4 本地 NED）下的平台观测历史。"""

    times: list[float] = field(default_factory=list)          # s
    positions: list[tuple[float, float, float]] = field(default_factory=list)
    yaws: list[float] = field(default_factory=list)           # rad

    def push(self, t: float, position: Sequence[float], yaw: float, window_s: float = 2.0) -> None:
        self.times.append(float(t))
        self.positions.append((float(position[0]), float(position[1]), float(position[2])))
        self.yaws.append(float(yaw))
        while len(self.times) > 3 and (self.times[-1] - self.times[0]) > window_s:
            self.times.pop(0)
            self.positions.pop(0)
            self.yaws.pop(0)

    def ready(self, min_samples: int = 6) -> bool:
        return len(self.times) >= min_samples


def _fit_velocity(times: list[float], values: list[float]) -> tuple[float, float]:
    """最小二乘拟合 value = a*t + b，返回 (a=速度, b=截距)；样本不足时返回 0。"""
    if len(times) < 3:
        return 0.0, values[-1] if values else 0.0
    t = np.asarray(times, dtype=float)
    v = np.asarray(values, dtype=float)
    A = np.vstack([t, np.ones_like(t)]).T
    slope, intercept = np.linalg.lstsq(A, v, rcond=None)[0]
    return float(slope), float(intercept)


class ConstantTurnPredictor:
    """恒速 + 恒转向外推的参考预测器（MIONet 的基线替代）。"""

    def __init__(self, horizon_s: float = 1.0, preview_horizon: int = 10) -> None:
        self.horizon_s = float(horizon_s)
        self.preview_horizon = int(preview_horizon)
        # 下降前馈（NED 下 z 正=向下）：由控制器按阶段设置
        self.descent_rate = 0.0
        # 参考点的垂直偏移（NED，正=向下）。阶段不同：巡航在平台上空、末端贴板
        self.z_offset = 0.0

    def __call__(self, observation: PadHistory):
        # 延迟导入，避免在没有 SkySail 时也能 import 本模块
        from UAV_Control.mionet.reference_controller import LandingReference  # noqa: PLC0415

        if not observation.ready():
            pos = observation.positions[-1] if observation.positions else (0.0, 0.0, 0.0)
            # 预热阶段也必须给 preview，否则 LQR 的曲率前馈项会报错
            ref = LandingReference(position=(pos[0], pos[1], pos[2] + self.z_offset),
                                   velocity=(0.0, 0.0, 0.0),
                                   curvature_preview=tuple([0.0] * self.preview_horizon))
            self.last_reference = ref
            return ref

        t = observation.times
        xs = [p[0] for p in observation.positions]
        ys = [p[1] for p in observation.positions]
        zs = [p[2] for p in observation.positions]

        vx, _ = _fit_velocity(t, xs)
        vy, _ = _fit_velocity(t, ys)
        vz, _ = _fit_velocity(t, zs)
        omega, _ = _fit_velocity(t, observation.yaws)

        h = self.horizon_s
        vz += self.descent_rate
        # 注意：vz 里已经加过 descent_rate，这里不能再加一次（此前是重复计入）
        pos = (xs[-1] + vx * h, ys[-1] + vy * h, zs[-1] + vz * h + self.z_offset)
        vel = (vx, vy, vz)

        speed = float(np.hypot(vx, vy))
        curvature = float(omega / speed) if speed > 0.2 else 0.0
        preview = tuple([curvature] * self.preview_horizon)

        ref = LandingReference(position=pos, velocity=vel, curvature_preview=preview)
        self.last_reference = ref          # 供调试面板读取
        return ref
