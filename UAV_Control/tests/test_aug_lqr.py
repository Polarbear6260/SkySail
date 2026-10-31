from dataclasses import replace
import unittest

import numpy as np

from UAV_Control.UAV_params import F450_PARAMS, LQR_PARAMS
from UAV_Control.aug_lqr import AugmentedLQRController, Phase


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = AugmentedLQRController(
            F450_PARAMS, LQR_PARAMS, reference_speed_m_s=1.0
        )

    def test_system_dimensions_and_finite_gains(self):
        self.assertEqual(self.controller.A.shape, (6, 6))
        self.assertEqual(self.controller.B.shape, (6, 3))
        self.assertEqual(self.controller.D.shape, (6, 1))
        for phase in Phase:
            feedback, preview = self.controller.gains(phase)
            self.assertEqual(feedback.shape, (3, 6))
            self.assertEqual(preview.shape, (3, LQR_PARAMS.preview_horizon if phase is not Phase.HOVER else 0))
            self.assertTrue(np.all(np.isfinite(feedback)))
            self.assertTrue(np.all(np.isfinite(preview)))

    def test_hover_reduces_position_error(self):
        command = self.controller.compute_control(
            position=(1.0, 0.0, 0.0),
            velocity=(0.0, 0.0, 0.0),
            reference_position=(0.0, 0.0, 0.0),
            reference_velocity=(0.0, 0.0, 0.0),
            phase=Phase.HOVER,
        )
        self.assertLess(command[0], 0.0)

    def test_preview_padding_and_saturation(self):
        command = self.controller.compute_control(
            position=(0.0, 0.0, 0.0),
            velocity=(0.0, 0.0, 0.0),
            reference_position=(0.0, 0.0, 0.0),
            reference_velocity=(0.0, 0.0, 0.0),
            phase=Phase.TRACKING,
            curvature_preview=(10.0,),
        )
        self.assertTrue(np.all(command <= F450_PARAMS.max_acceleration))
        self.assertTrue(np.all(command >= F450_PARAMS.min_acceleration))

    def test_empty_preview_is_rejected_for_preview_phases(self):
        with self.assertRaises(ValueError):
            self.controller.compute_control(
                (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0),
                Phase.TRACKING,
            )


    def test_constant_curvature_feedforward_has_correct_sign_and_scale(self):
        # Curvature 0.2 at 1 m/s needs +0.2 m/s^2, regardless of horizon.
        for horizon in (1, 10, 20):
            with self.subTest(horizon=horizon):
                controller = AugmentedLQRController(
                    lqr=replace(LQR_PARAMS, preview_horizon=horizon)
                )
                command = controller.compute_control(
                    (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0),
                    Phase.TRACKING, (0.2,) * horizon,
                )
                self.assertAlmostEqual(command[1], 0.2, delta=0.005)
                np.testing.assert_allclose(command[[0, 2]], 0.0, atol=1e-12)
                np.testing.assert_allclose(controller.D[:, 0], -controller.B[:, 1])

    def test_tracking_brakes_before_crossing_stationary_target(self):
        state = np.array([1.0, 0, 0, 0, 0, 0])
        for _ in range(240):
            command = self.controller.compute_control(
                state[:3], state[3:], (0, 0, 0), (0, 0, 0),
                Phase.TRACKING, (0.0,) * LQR_PARAMS.preview_horizon,
            )
            state = self.controller.A @ state + self.controller.B @ command
            self.assertGreaterEqual(state[0], -1e-8)
        self.assertLess(abs(state[0]), 1e-4)
