import unittest

import numpy as np

from UAV_Control import (
    F450_PARAMS,
    LandingReference,
    MIONetLandingController,
    Phase,
    PointLandingController,
)


class AdapterTests(unittest.TestCase):
    def test_direct_controller_tracks_fixed_landing_point(self):
        controller = PointLandingController((1.0, 0.0, 0.0))
        command = controller.compute_control(
            position=(0.0, 0.0, 0.0),
            velocity=(0.0, 0.0, 0.0),
            phase=Phase.TRACKING,
        )
        self.assertGreater(command[0], 0.0)
        self.assertTrue(np.all(command <= F450_PARAMS.max_acceleration))

    def test_mionet_adapter_uses_predicted_reference(self):
        reference = LandingReference(
            position=(1.0, 0.0, 0.0),
            velocity=(0.0, 0.0, 0.0),
            curvature_preview=(0.0,) * 10,
        )
        controller = MIONetLandingController(lambda observation: reference)
        command = controller.compute_control(
            position=(0.0, 0.0, 0.0),
            velocity=(0.0, 0.0, 0.0),
            observation={"history": "model-specific"},
        )
        self.assertGreater(command[0], 0.0)

    def test_mionet_adapter_rejects_wrong_predictor_result(self):
        controller = MIONetLandingController(lambda observation: None)
        with self.assertRaises(TypeError):
            controller.compute_control((0, 0, 0), (0, 0, 0), None)

    def test_reference_rejects_nonfinite_values(self):
        with self.assertRaises(ValueError):
            LandingReference((np.nan, 0.0, 0.0), (0.0, 0.0, 0.0))
