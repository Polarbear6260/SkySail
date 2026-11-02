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

## 真正的降落（2026-11-02 01:52）

**上锁已经能成功**：不再等 PX4 的 `vehicle_land_detected`（船在升沉，世界系里永远在动，
它永远判 False），改为控制器自己判——相对板面高度稳定 1.5 s + 相对速度 <0.2 m/s，
然后发强制上锁（`MAV_CMD_COMPONENT_ARM_DISARM`, param2=21196）。日志确认 `Disarmed by`。

**遗留问题（下一轮要修）**：上锁瞬间水平偏移已经涨到 0.41–0.54 m，正好在落点板边缘，
停机后从边上滑落。原因是 `settle` 阶段用**位置设定点**跟踪移动的板，PX4 的位置环对
0.4 m/s 的移动目标有稳态滞后，偏移越积越大。

修法：`settle/压紧` 阶段也改用**加速度控制**（或位置 + 速度前馈），让跟踪不滞后；
同时下降前要求水平误差 <0.15 m（已改）。

## 下一轮场景（按需求，2026-11-02 确认）

1. **固定平台起飞**：在离船 15–20 m 处放 static `dock`（2×2 m，台面 1.0 m），飞机生成在台上
2. **先爬高**：起飞到 6 m 巡航高度
3. **高空接近**：在巡航高度平飞接近船
4. **下降着陆**：到板正上方后下降、压紧、上锁

改三处：`x650_sea.sdf` 加 dock；`11_run_moving_landing.sh` 改 `PX4_GZ_MODEL_POSE`；
`moving_landing.py` 加 `cruise_alt` 并加位置+速度前馈。

## 决定性下降 + 真实上锁（2026-11-02 02:07）

参考了同类系统（[deck-velocity feedforward + 低相对运动窗口](https://github.com/Manas-arumalla/drone-landing-moving-platform/blob/main/src/drone_landing/autopilot.py)、
[PX4 precision landing](https://docs.px4.io/v1.13/zh/advanced_features/precland)）后做了四点改动：

1. **进场高度 1.0 → 0.5 m**（少在头上磨蹭）
2. **下降段改用 SkySail 的 `touchdown` 阶段**（位置/速度权重加倍，动作更硬）
3. **恒定 0.6 m/s 下降前馈**（参考速度 = 甲板速度 + 下降率），不靠误差慢慢挤
4. **压紧段改加速度控制 + 甲板速度前馈**（位置环追移动目标有稳态滞后，会滑到板边）

**修掉的关键 bug：坐标系错位。** `alt` 是 PX4 本体系（原点 = 出生点，甲板高度），
`pad_alt` 是世界的绝对高度（0.85 m），两者差一个恒定 0.85 m —— 所以之前所谓的
"压到板面"其实一直在甲板上方 0.85 m，飞机当然只悬停。现在 `on_pad_pose` 里减掉这个偏移。

**实测（最后一轮）**：

```
到达板面上方（水平 0.13 m）→ 接触（相对高度 0.06 m）→ 坐实（+0.09 m）→ 强制上锁
Armed by external ×1   Disarmed by ×1   无 failsafe
```

**遗留**：上锁后机体从平坦的落点板上**滑出去掉进海里**（视频可见甲板空了）。
加的那圈"挡边"其实是实心圆盘（把红板盖住了），不是环，所以没有拦挡作用。
下一轮要做成真正的环形挡边 + 提高摩擦，或者加一个"系留"逻辑（上锁前先降到底）。

## 下一轮（已确认）

1. 固定 dock（离船 15–20 m）起飞 → 爬升 6 m 巡航 → 高空接近 → 下降着陆
2. 落点板改成真正的环形挡边（外径 0.60 / 内径 0.56），摩擦 1.5
3. 上锁判定加"已降到底"（相对高度 < 0.05）再停机，避免在半空停机后摔落

## dock 起飞 + 6 m 巡航场景（2026-11-02 02:28）

按需求改成"固定平台起飞 → 飞高 → 再降落"：

| 环境 | 内容 |
|---|---|
| `dock` | static 起飞台，2.4×2.4 m，台面 1.0 m，位于 `(0, 16)`，离船 ~16 m |
| 出生点 | `PX4_GZ_MODEL_POSE="0,16,1.02,0,0,0"` |
| 无人机流程 | 起飞到 **6 m 巡航** → 转场到落点板上方 → LQR 下降 → 压紧 → 上锁 |
| 落点板 | 24 段环形挡边（每段 0.16×0.02×0.08 m）+ 摩擦 1.5 |

**实测（02:28 那轮）**：

```
到达巡航高度 6 m → 已到板上方（水平 0.49 m）→ 下降 → 接触（水平 0.24 m）
→ 坐实（相对速度 0.04 m/s）→ 强制上锁
Armed by external ×1   Disarmed by ×1   无 failsafe
```

**本轮修掉的两个环境问题**：

1. 船驱动在 ROS 环境下 `import gz.msgs10` 会命中 `gz_msgs_vendor` 而缺 `google.protobuf`
   → 启动时加 `PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH`
2. OFFBOARD 断流会触发 RTL → airframe 里设 `COM_OF_LOSS_T 5.0`、`COM_OBL_RC_ACT 0`

**仍未解决**：上锁之后机体仍会从甲板上消失（视频末帧甲板是空的）。
下一步建议：上锁前用 gz 侧（机体 vs 船）相对位姿确认"确实在板内且贴板"，
或用 `vehicle_land_detected` 之外的独立判据；也可以让机体重心/摩擦更保守。

## 2026-11-02 13:40 暂停（未解决）

**新证据**：加了 `12_log_pose.py`（直接从 Gazebo 读真值）后发现，**PX4 的本地位置和 Gazebo 真值对不上**：

| 时刻 | PX4 认为 | Gazebo 真值 |
|---|---|---|
| 判"接触"时 | 高于板面 −0.45 m，水平 0.0x m | 水平最近 6.6–12.6 m，且在自由落体 |
| 结束时 | alt ≈ 0.8 m | z = 7.0 m |

也就是说：**PX4 报告的"贴合"是假的，飞机实际在半空，上锁后自由落体掉进海里**
（之前位姿日志里 z 掉到 −50000 m 就是自由落体 100 多秒的结果）。

已尝试：把 `local_z_offset` 从 1.02 改成 0、把着陆判定改成用 Gazebo 真值（`on_gz_pose`）。
但真值判定下 PX4 的本地坐标与真值仍差十几米，节点一直停在 `approach` 阶段。

**下一步必须先做的**（不要再调参）：
做一次静态 A/B 对比——仿真不动、PX4 解锁但不控，同一时刻同时打印
`/fmu/out/vehicle_local_position_v1` 和 `/world/<world>/pose/info` 里 `x650_0` 的位姿，
确认两者到底差多少、差在哪（原点？轴序？还是根本是另一个实体）。
先把坐标系钉死，后面的着陆逻辑才有意义。
