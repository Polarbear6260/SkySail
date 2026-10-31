"""Minimal smoke example for the preview-augmented LQR controller."""

from UAV_Control.aug_lqr import AugmentedLQRController, Phase
from UAV_Control.UAV_params import F450_PARAMS


def main() -> None:
    controller = AugmentedLQRController(F450_PARAMS, reference_speed_m_s=1.0)
    command = controller.compute_control(
        position=(1.0, 0.2, 0.5),
        velocity=(0.1, 0.0, 0.0),
        reference_position=(0.0, 0.0, 0.0),
        reference_velocity=(0.0, 0.0, -0.2),
        phase=Phase.TRACKING,
        curvature_preview=(0.02,) * 10,
    )
    print(f"Vehicle: {F450_PARAMS.model_name}")
    print(f"Tracking acceleration command [m/s^2]: {command}")


if __name__ == "__main__":
    main()
