"""Vehicle and controller parameters for the first UAV control prototype.

The default vehicle is an engineering approximation of a DJI F450-class
quadcopter. It is not an official DJI flight-controller parameter set.
"""

from dataclasses import dataclass
from typing import Final

import numpy as np


@dataclass(frozen=True)
class UAVParams:
    """Physical and outer-loop limits used by the acceleration controller."""

    model_name: str
    mass_kg: float
    inertia_kg_m2: tuple[float, float, float]
    gravity_m_s2: float
    dt_s: float
    max_acceleration_m_s2: tuple[float, float, float]
    min_acceleration_m_s2: tuple[float, float, float]
    max_tilt_rad: float
    max_descent_speed_m_s: float

    def __post_init__(self) -> None:
        if self.mass_kg <= 0 or self.gravity_m_s2 <= 0 or self.dt_s <= 0:
            raise ValueError("mass, gravity, and dt must be positive")
        if any(value <= 0 for value in self.inertia_kg_m2):
            raise ValueError("all inertia values must be positive")
        if any(
            lower >= upper
            for lower, upper in zip(
                self.min_acceleration_m_s2, self.max_acceleration_m_s2
            )
        ):
            raise ValueError("each acceleration lower bound must be below its upper bound")
        if not 0 < self.max_tilt_rad < np.pi / 2:
            raise ValueError("max_tilt_rad must be between zero and pi/2")
        if self.max_descent_speed_m_s <= 0:
            raise ValueError("max_descent_speed_m_s must be positive")

    @property
    def max_acceleration(self) -> np.ndarray:
        return np.asarray(self.max_acceleration_m_s2, dtype=float)

    @property
    def min_acceleration(self) -> np.ndarray:
        return np.asarray(self.min_acceleration_m_s2, dtype=float)


@dataclass(frozen=True)
class LQRParams:
    """Weights and preview settings for the three control phases."""

    position_weight: tuple[float, float, float]
    velocity_weight: tuple[float, float, float]
    control_weight: tuple[float, float, float]
    preview_horizon: int
    # A small positive weight makes the unit-circle preview states detectable
    # by the discrete Riccati solver while keeping curvature cost negligible.
    curvature_weight: float = 1e-3
    # The ideal preview hold has an uncontrollable unit pole. A slight decay
    # is used for the first infinite-horizon implementation.
    preview_decay: float = 0.995
    touchdown_position_scale: float = 2.0
    touchdown_velocity_scale: float = 2.0
    tracking_position_scale: float = 1.0
    tracking_velocity_scale: float = 1.0

    def __post_init__(self) -> None:
        if self.preview_horizon < 0:
            raise ValueError("preview_horizon must be non-negative")
        if not 0 < self.preview_decay < 1:
            raise ValueError("preview_decay must be between zero and one")
        weights = (
            *self.position_weight,
            *self.velocity_weight,
            *self.control_weight,
            self.curvature_weight,
        )
        if any(value <= 0 for value in weights if value != 0):
            raise ValueError("non-zero LQR weights must be positive")


# Engineering approximation for a common F450-class research quadrotor.
F450_PARAMS: Final[UAVParams] = UAVParams(
    model_name="DJI F450-class quadrotor (engineering approximation)",
    mass_kg=1.5,
    inertia_kg_m2=(0.029, 0.029, 0.055),
    gravity_m_s2=9.81,
    dt_s=0.05,
    max_acceleration_m_s2=(4.0, 4.0, 3.0),
    min_acceleration_m_s2=(-4.0, -4.0, -3.0),
    max_tilt_rad=np.deg2rad(25.0),
    max_descent_speed_m_s=1.0,
)

LQR_PARAMS: Final[LQRParams] = LQRParams(
    position_weight=(16.0, 16.0, 25.0),
    velocity_weight=(4.0, 4.0, 9.0),
    control_weight=(1.0, 1.0, 1.5),
    preview_horizon=10,
    curvature_weight=0.0,
    tracking_velocity_scale=4.0,
)
