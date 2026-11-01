# 移动平台（海上着陆场景）

最终算法目标是**落到移动平台上**，所以先把平台做出来。当前实现是
**运动学平台 + 视觉水面**：船按预设轨迹被 `set_pose` 移动，不做浮力/波浪耦合。

## 为什么先做运动学

| 方案 | 优点 | 缺点 | 选择 |
|---|---|---|---|
| 运动学船（当前） | 轨迹完全可控、可重复、易验证 | 无波浪-船体耦合 | ✅ 先把控制闭环打通 |
| 浮力 + 波浪（gz `buoyancy`/`hydrodynamics` 插件） | 接近真实 | 调参麻烦、结果不可重复、落地判定嘈杂 | ⏳ 后续再加 |

先把"能不能落上去"验证清楚，再谈"海浪下稳不稳"。

## 世界：`gz_extras/worlds/x650_sea.sdf`

- `sea_surface`：400×400 平面，**只有视觉无碰撞**（避免落水判定干扰）
- `ship`：4.0×1.6 m 船体 + 甲板 + 驾驶舱，**static**（由驱动脚本移动）
  - 着陆板：船体坐标系 `(0.30, 0, 0.45)`，半径 0.55 m，红色 + 白边
- 带 `spherical_coordinates`（磁力计需要，否则会报强磁干扰）

## 驱动：`scripts/10_move_platform.py`

50 Hz 调 `/world/<world>/set_pose`（用 `gz.transport13` Python 绑定，不是 CLI）。

默认轨迹：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--radius` | 6.0 m | 圆弧半径 |
| `--speed` | 0.4 m/s | 航速（≈0.77 kn，慢速靠泊） |
| `--heave` / `--heave-period` | 0.10 m / 5.0 s | 升沉 |
| `--roll-deg` / `--pitch-deg` | 3° / 2° | 横摇 / 纵摇 |
| `--rate` | 50 Hz | set_pose 频率 |

用法：

```bash
# 终端 1：起世界（独立测试时需带上 PX4 的 server.config 才有 set_pose 服务）
export GZ_SIM_SERVER_CONFIG_PATH=~/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config
export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds
gz sim -r -s gz_extras/worlds/x650_sea.sdf

# 终端 2：驱动船
python3 scripts/10_move_platform.py --csv logs/ship.csv
```

输出：控制台每 2 s 打印船体与落点板的世界位姿；`--csv` 记录时间序列（作为控制器的参考真值）。

## 已验证（2026-11-01）

- `set_pose` 服务存在并生效：驱动 15 s 后，服务端读到船体位姿 `(5.04, −2.75, 0.357)`，与驱动算出的轨迹一致
- GUI 渲染正常：海面、船体、甲板红色落点板都在（见 `results/2026-11-01-moving-platform-preview.png`）

## 下一步（控制器改造）

当前 `landing_demo` 只能落到**固定点**（`PointLandingController` 的参考速度是零）。要落到移动平台，需要：

1. **参考带速度**：`AugmentedLQRController.compute_control` 已支持 `reference_velocity`，
   但 `PointLandingController` 写死了 `(0,0,0)`，需要新增一个"移动参考"适配器
   （正好对应上游的 `MIONetLandingController` 接口形状）
2. **平台位姿订阅**：控制器要知道船和落点板在哪，以及它们的速度
   （可从 `10_move_platform.py` 的 CSV 真值、或 gz 的 `pose/info` 话题、或 ros_gz_bridge 取）
3. **触地判定改成相对量**：相对平台的水平距离 + 相对垂直速度 + 相对水平速度，
   而不是对地速度
