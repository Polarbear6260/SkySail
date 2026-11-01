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

## 移动着陆联调记录（2026-11-01）

**已打通**：

- 平台位姿 → ROS 2：驱动以 50 Hz 发布 `/ship/pad_pose`、`/ship/pad_twist`（ENU）
- 控制器：`moving_landing` 节点把 ENU→NED，用 SkySail LQR（参考速度 = 平台速度）追板
- 起飞：飞机由 `PX4_GZ_MODEL_POSE="0,0,0.80,0,0,0"` 生成在甲板上，从移动甲板起飞

**连续三次跑出的接触数据**（相对移动落点板）：

| 指标 | 第 1 次 | 第 2 次 | 第 3 次 |
|---|---|---|---|
| 相对高度 | 0.07 m | 0.11 m | 0.14 m |
| 相对速度 | 0.29 m/s | 0.30 m/s | 0.28 m/s |
| 水平误差 | 0.13 m | 0.15 m | 0.13 m |

**未解决**：接触后 PX4 触发 `Failsafe activated`，飞机随后穿过甲板下坠（不是落在船上）。
可能原因（待查）：OFFBOARD 控制模式在“加速度→位置”切换时被判定异常；或
`vehicle_control_mode` 掉出 OFFBOARD。下一步应打印 `vehicle_status.nav_state` 与
`failsafe_flags`，并考虑全程只用一种控制模式或在接触后直接切 `AUTO.LAND`。

## 暂停点（2026-11-01 19:20）

**下坠的根因找到了：不是控制模式，是电池失效保护。**

```
WARN  [failsafe] Failsafe activated
INFO  [tone_alarm] battery warning (fast)
INFO  [navigator] RTL: start return at 7 m
INFO  [navigator] RTL: land at destination
```

飞机被 RTL 带回起飞点降落，但船已经开走 → 落进海里（水面无碰撞）→ 一路下坠。

处置：把 SITL 电池相关参数改成“不触发失效保护”，已确认生效：

| 参数 | 值 | 作用 |
|---|---|---|
| `NAV_DLL_ACT` | 0 | 无地面站也能解锁 |
| `SIM_BAT_DRAIN` | 36000 | 10 小时才放完电 |
| `COM_LOW_BAT_ACT` | 0 | 低电量只告警、不 RTL |
| `SIM_BAT_MIN_PCT` | 50 | 保持默认 |

**注意**：最后一次运行时，参数是在 SITL 启动后、起飞前设的，但日志里仍有 `battery warning (fast)`，
需要下一轮**先设参数、再确认无告警**，然后才起飞验证。

**下次恢复的命令**：

```bash
cd /home/green/文档/deepseek-harness/default-workspace/px4-gz-sim
./logs/run_ml2.sh          # 清场 → agent → SITL → 设参数 → 船 → 移动着陆
```

当前最好成绩（相对移动落点板）：**水平误差 0.13–0.23 m，相对高度 0.07–0.14 m**。

## 移动平台着陆：已跑通（2026-11-02 01:44）

**关键修复**：PX4 的 `offboard_control_signal_lost` 是这一串失败的真正原因——
状态机一旦进入某个不发布设定点的分支（`hold`/`landed`），PX4 在 0.5 s 内判定 OFFBOARD 丢失，
触发 failsafe → RTL（日志里还会伴随 `battery warning (fast)` 的误导性提示）。
现在 `loop()` **每个周期都无条件发布设定点**，状态只决定“发什么”。

实测（第 9 轮）：

```
state=landed  alt=0.71~0.85 m   pad_alt=0.71~0.91 m   rel_v=0.04~0.12 m/s
```

高度与落点板同步起伏 → 机体随船升沉，确实“坐在甲板上”。

遗留：水平偏移约 0.41–0.47 m（在半径 0.55 m 板内但未居中），下一轮加横向收敛再收。

## 下一轮场景（按需求调整）

改为更接近真实作业的流程：

1. **固定平台起飞**：在离船较远的位置放一个 static 起飞台（dock），飞机生成在台上；
2. **巡航高度**：起飞后先爬升到 6 m 左右，在高度上平飞接近；
3. **下降着陆**：到落点板上方再下降、贴板。

需要改的地方：

| 文件 | 改动 |
|---|---|
| `gz_extras/worlds/x650_sea.sdf` | 加 static `dock` 模型（如 2×2 m，台面 1.0 m），放在离船 15–20 m |
| `11_run_moving_landing.sh` | `PX4_GZ_MODEL_POSE` 指向 dock 台面 |
| `moving_landing.py` | 加 `cruise_alt` 参数：起飞 → 巡航高度 → LQR 在该高度追板 → 到板正上方再下降 |
