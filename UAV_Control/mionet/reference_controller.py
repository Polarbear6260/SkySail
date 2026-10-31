"""Connect an external MIONet predictor to the shared LQR controller.

The adapter intentionally depends on a small predictor protocol instead of
importing PyTorch or a particular MIONet checkpoint format. A model-specific
adapter can convert its output into :class:`LandingReference`.
"""

from dataclasses import dataclass
from typing import Any, Iterable, Protocol

import numpy as np

from UAV_Control.aug_lqr import AugmentedLQRController, Phase, _as_vector


@dataclass(frozen=True)
class LandingReference:
    """One predicted landing reference in the controller's coordinate frame."""

    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    curvature_preview: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        _as_vector(self.position, 3, "reference.position")
        _as_vector(self.velocity, 3, "reference.velocity")
        preview = np.asarray(self.curvature_preview, dtype=float)
        if preview.ndim != 1 or not np.all(np.isfinite(preview)):
            raise ValueError("reference.curvature_preview must be finite and one-dimensional")


class ReferencePredictor(Protocol):
    """Protocol implemented by a MIONet-specific prediction adapter."""

    def __call__(self, observation: Any) -> LandingReference:
        ...


class MIONetLandingController:
    """Use MIONet-derived references with the shared outer-loop controller."""

    def __init__(
        self,
        predictor: ReferencePredictor,
        controller: AugmentedLQRController | None = None,
    ) -> None:
        self.predictor = predictor
        self.controller = controller or AugmentedLQRController()

    def compute_control(
        self,
        position: Iterable[float],
        velocity: Iterable[float],
        observation: Any,
        phase: Phase | str = Phase.TRACKING,
    ) -> np.ndarray:
        """Predict a reference, then compute the corresponding acceleration."""
        reference = self.predictor(observation)
        if not isinstance(reference, LandingReference):
            raise TypeError("predictor must return a LandingReference")
        return self.controller.compute_control(
            position=position,
            velocity=velocity,
            reference_position=reference.position,
            reference_velocity=reference.velocity,
            phase=phase,
            curvature_preview=reference.curvature_preview,
        )
