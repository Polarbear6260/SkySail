# UAV 控制（着陆外环）

本目录是无人机着陆的外环控制器。结构上刻意拆成**共享的 LQR 核心**和**两套参考生成层**：

```text
UAV_Control/
├── aug_lqr.py                 # 共享的 preview 增强型 LQR 核心
├── UAV_params.py              # 机体参数与 LQR 权重
├── direct/
│   └── point_landing.py       # 固定落点（不依赖 MIONet）
├── mionet/
│   └── reference_controller.py # 预测器 → LQR 的适配层
├── examples/
└── tests/
```

## 两种控制模式

### 1. 直接给定落点

落点已知时用 `PointLandingController`：把固定点当位置参考，参考速度视为零，
不导入、不调用、也不依赖 MIONet。

```python
from UAV_Control.direct import PointLandingController

controller = PointLandingController(landing_position=(10.0, 2.0, 0.0))
command = controller.compute_control(
    position=(8.0, 2.5, 3.0),
    velocity=(0.2, 0.0, -0.5),
)
```

### 2. 由 MIONet 预测参考

移动平台着陆时，用 `MIONetLandingController`：由外部预测器给出**移动的着陆参考**。
控制器本身不依赖 PyTorch，也不绑定特定的权重文件格式；模型专属的适配器负责把网络输出
转成 `LandingReference`：

```python
from UAV_Control.mionet import LandingReference, MIONetLandingController


def predict_reference(observation):
    # 跑 MIONet，并把输出转换到控制器坐标系
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

`LandingReference` 必须与 LQR 输入用**同一坐标系和单位**：位置用米、速度用米每秒、
曲率用米的倒数。适配器就是处理这些边界的唯一位置——MIONet 输出的坐标系、单位、延迟、
预测时域，都在这里对齐。

当前的 `MIONet_v1` 模型预测的是 **ACE 观测通道**，其输出并不是物理上的甲板位姿，
也不是无人机的着陆参考。因此本包**不会**把 `ACEModel` 权重直接接到控制器上；
必须先实现“观测通道 → 着陆位置/速度/曲率”的转换，并验证通过。

## 共享控制器的范围

`AugmentedLQRController` 只输出**饱和后的三轴加速度指令** `[ax, ay, az]`（m/s²），
不包含姿态控制、电机混控、ROS、PX4 或 MAVLink。

默认机体是 DJI F450 级别的工程近似，**不是** DJI 官方飞控参数。等有了实测机体模型，
替换 `UAV_params.py` 里的 `F450_PARAMS` 即可（本仓库的仿真是换成 X650 的 BOM 参数）。

## 运行

在项目根目录：

```bash
python -m unittest discover -s UAV_Control -p "test_*.py" -v
python -m UAV_Control.examples.demo_aug_lqr
```

调用方已有参考时，也可以直接用 LQR 核心：

```python
from UAV_Control import AugmentedLQRController
```
