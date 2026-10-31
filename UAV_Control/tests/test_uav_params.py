import unittest

import numpy as np

from UAV_Control.UAV_params import F450_PARAMS, LQR_PARAMS


class ParameterTests(unittest.TestCase):
    def test_f450_parameters_are_valid(self):
        self.assertEqual(F450_PARAMS.model_name.split()[0], "DJI")
        self.assertGreater(F450_PARAMS.mass_kg, 0)
        self.assertTrue(np.all(F450_PARAMS.max_acceleration > F450_PARAMS.min_acceleration))

    def test_lqr_weights_and_preview_are_valid(self):
        self.assertEqual(len(LQR_PARAMS.position_weight), 3)
        self.assertEqual(len(LQR_PARAMS.velocity_weight), 3)
        self.assertEqual(len(LQR_PARAMS.control_weight), 3)
        self.assertGreaterEqual(LQR_PARAMS.preview_horizon, 0)
