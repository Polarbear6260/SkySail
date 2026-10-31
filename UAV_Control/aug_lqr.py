"""Preview-augmented LQR outer-loop controller for shipboard landing.

This module controls commanded translational acceleration. It deliberately
does not contain a motor mixer, attitude controller, or flight-stack adapter.
"""

from enum import Enum
from typing import Iterable

import numpy as np
from scipy.linalg import solve_discrete_are

from .UAV_params import F450_PARAMS, LQR_PARAMS, LQRParams, UAVParams


class Phase(str, Enum):
    HOVER = "hover"
    TRACKING = "tracking"
    TOUCHDOWN = "touchdown"


def _as_vector(values: Iterable[float], size: int, name: str) -> np.ndarray:
    result = np.asarray(tuple(values), dtype=float)
    if result.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},)")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


class AugmentedLQRController:
    """Three-phase acceleration controller with curvature preview feedforward."""

    state_size = 6

    def __init__(
        self,
        uav: UAVParams = F450_PARAMS,
        lqr: LQRParams = LQR_PARAMS,
        reference_speed_m_s: float = 1.0,
        lateral_direction: Iterable[float] = (0.0, 1.0, 0.0),
    ) -> None:
        if reference_speed_m_s < 0:
            raise ValueError("reference_speed_m_s must be non-negative")
        direction = _as_vector(lateral_direction, 3, "lateral_direction")
        norm = np.linalg.norm(direction)
        if not np.isclose(norm, 1.0):
            raise ValueError("lateral_direction must be a unit vector")
        self.uav = uav
        self.lqr = lqr
        self.reference_speed_m_s = float(reference_speed_m_s)
        self.lateral_direction = direction
        self.A, self.B, self.D = self._build_system()
        self._gains = {
            phase: self._design_gains(phase) for phase in Phase
        }

    def _build_system(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        dt = self.uav.dt_s
        eye = np.eye(3)
        zero = np.zeros((3, 3))
        A = np.block([[eye, dt * eye], [zero, eye]])
        B = np.vstack((0.5 * dt**2 * eye, dt * eye))
        # Error is UAV minus reference: reference acceleration enters with
        # a negative sign, including its position contribution.
        D = -B @ (self.reference_speed_m_s**2 * self.lateral_direction[:, None])
        return A, B, D

    def _weights(self, phase: Phase) -> tuple[np.ndarray, np.ndarray]:
        position = np.asarray(self.lqr.position_weight, dtype=float)
        velocity = np.asarray(self.lqr.velocity_weight, dtype=float)
        if phase is Phase.TRACKING:
            position *= self.lqr.tracking_position_scale
            velocity *= self.lqr.tracking_velocity_scale
        elif phase is Phase.TOUCHDOWN:
            position *= self.lqr.touchdown_position_scale
            velocity *= self.lqr.touchdown_velocity_scale
        Q = np.diag(np.concatenate((position, velocity)))
        R = np.diag(np.asarray(self.lqr.control_weight, dtype=float))
        return Q, R

    def _design_gains(self, phase: Phase) -> tuple[np.ndarray, np.ndarray]:
        Q, R = self._weights(phase)
        if phase is Phase.HOVER or self.lqr.preview_horizon == 0:
            P = solve_discrete_are(self.A, self.B, Q, R)
            feedback = np.linalg.solve(R + self.B.T @ P @ self.B, self.B.T @ P @ self.A)
            return feedback, np.zeros((3, 0))

        horizon = self.lqr.preview_horizon
        shift = np.zeros((horizon, horizon))
        if horizon > 1:
            shift[:-1, 1:] = np.eye(horizon - 1)
        shift[-1, -1] = self.lqr.preview_decay
        # Only the first preview sample acts now; the shift advances future
        # samples. Summing the horizon here multiplies the disturbance.
        current_sample = np.zeros((1, horizon))
        current_sample[0, 0] = 1.0
        A_aug = np.block([
            [self.A, self.D @ current_sample],
            [np.zeros((horizon, self.state_size)), shift],
        ])
        B_aug = np.vstack((self.B, np.zeros((horizon, 3))))
        Q_aug = np.zeros((self.state_size + horizon, self.state_size + horizon))
        Q_aug[:self.state_size, :self.state_size] = Q
        Q_aug[self.state_size:, self.state_size:] = np.eye(horizon) * self.lqr.curvature_weight
        P = solve_discrete_are(A_aug, B_aug, Q_aug, R)
        gain = np.linalg.solve(
            R + B_aug.T @ P @ B_aug, B_aug.T @ P @ A_aug
        )
        return gain[:, :self.state_size], gain[:, self.state_size:]

    def gains(self, phase: Phase | str) -> tuple[np.ndarray, np.ndarray]:
        """Return feedback and preview gains for inspection and testing."""
        return self._gains[Phase(phase)]

    def _prepare_preview(self, preview: Iterable[float]) -> np.ndarray:
        raw_values = tuple(preview)
        values = _as_vector(raw_values, len(raw_values), "curvature_preview")
        horizon = self.lqr.preview_horizon
        if horizon == 0:
            return np.zeros(0)
        if values.size == 0:
            raise ValueError("curvature_preview cannot be empty")
        if values.size >= horizon:
            return values[:horizon]
        return np.pad(values, (0, horizon - values.size), mode="edge")

    def compute_control(
        self,
        position: Iterable[float],
        velocity: Iterable[float],
        reference_position: Iterable[float],
        reference_velocity: Iterable[float],
        phase: Phase | str,
        curvature_preview: Iterable[float] = (),
    ) -> np.ndarray:
        """Return a saturated [ax, ay, az] command in m/s²."""
        selected_phase = Phase(phase)
        error = np.concatenate((
            _as_vector(position, 3, "position")
            - _as_vector(reference_position, 3, "reference_position"),
            _as_vector(velocity, 3, "velocity")
            - _as_vector(reference_velocity, 3, "reference_velocity"),
        ))
        feedback, preview_gain = self.gains(selected_phase)
        command = -feedback @ error
        if preview_gain.shape[1]:
            command -= preview_gain @ self._prepare_preview(curvature_preview)
        return np.clip(command, self.uav.min_acceleration, self.uav.max_acceleration)
