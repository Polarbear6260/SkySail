# UAV Control

This folder contains the UAV outer-loop landing controller. It is intentionally
split into a shared LQR core and two reference-generation layers:

```text
UAV_Control/
├── aug_lqr.py                 # Shared preview-augmented LQR core
├── UAV_params.py              # Vehicle and LQR parameters
├── direct/
│   └── point_landing.py       # Fixed landing point; no MIONet
├── mionet/
│   └── reference_controller.py # Predictor-to-LQR adapter
├── examples/
└── tests/
```

## Two control modes

### 1. Direct landing point

Use `PointLandingController` when the landing point is supplied directly. It
uses the fixed point as the position reference and assumes zero reference
velocity. It does not import, call, or require MIONet.

```python
from UAV_Control.direct import PointLandingController

controller = PointLandingController(landing_position=(10.0, 2.0, 0.0))
command = controller.compute_control(
    position=(8.0, 2.5, 3.0),
    velocity=(0.2, 0.0, -0.5),
)
```

### 2. MIONet-derived reference

Use `MIONetLandingController` when an external predictor should provide the
moving landing reference. The controller does not depend on PyTorch or on a
specific checkpoint format. A model-specific adapter must convert its output
into `LandingReference`:

```python
from UAV_Control.mionet import LandingReference, MIONetLandingController


def predict_reference(observation):
    # Run MIONet and convert its output into the UAV controller frame.
    return LandingReference(
        position=(x, y, z),
        velocity=(vx, vy, vz),
        curvature_preview=curvature_values,
    )


controller = MIONetLandingController(predict_reference)
command = controller.compute_control(
    position=uav_position,
    velocity=uav_velocity,
    observation=mionet_input,
)
```

`LandingReference` must be expressed in the same coordinate frame and units as
the LQR inputs: position in metres, velocity in metres per second, and
curvature in inverse metres. The adapter is the boundary where MIONet output
frames, units, latency, and forecast horizon must be handled.

The current `MIONet_v1` models predict ACE observation channels. Those outputs
are not automatically a physical deck pose or a UAV landing reference.
Therefore this package does **not** silently connect an `ACEModel` checkpoint
directly to the controller. A validated conversion from observation channels to
landing position, velocity, and curvature must be implemented in a
model-specific adapter first.

## Shared controller scope

`AugmentedLQRController` outputs a saturated three-axis acceleration command
`[ax, ay, az]` in m/s². It does not implement attitude control, motor mixing,
ROS, PX4, or MAVLink.

The default vehicle is a DJI F450-class engineering approximation, not an
official DJI flight-controller parameter set. Replace `F450_PARAMS` in
`UAV_params.py` when a measured airframe model is available.

## Run

From the project root:

```powershell
python -m unittest discover -s UAV_Control -p "test_*.py" -v
python -m UAV_Control.examples.demo_aug_lqr
```

The LQR core remains usable directly when a caller already has a reference:

```python
from UAV_Control import AugmentedLQRController
```
