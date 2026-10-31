"""Landing controller for a directly supplied fixed landing point."""

from typing import Iterable

import numpy as np

from UAV_Control.aug_lqr import AugmentedLQRController, Phase, _as_vector


class PointLandingController:
    """Adapt a fixed landing point to the shared outer-loop LQR controller."""

    def __init__(
        self,
        landing_position: Iterable[float],
        controller: AugmentedLQRController | None = None,
    ) -> None:
        self.controller = controller or AugmentedLQRController()
        self.landing_position = _as_vector(
            landing_position, 3, "landing_position"
        )

    def compute_control(
        self,
        position: Iterable[float],
        velocity: Iterable[float],
        phase: Phase | str = Phase.TRACKING,
    ) -> np.ndarray:
        """Return acceleration toward the fixed point with zero target velocity."""
        horizon = self.controller.lqr.preview_horizon
        curvature_preview = np.zeros(horizon)
        return self.controller.compute_control(
            position=position,
            velocity=velocity,
            reference_position=self.landing_position,
            reference_velocity=(0.0, 0.0, 0.0),
            phase=phase,
            curvature_preview=curvature_preview,
        )
