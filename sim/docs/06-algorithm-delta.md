# 原始算法 vs 当前实现：差异说明

> 上游理论代码位于 `UAV_Control/`，**一行未改**（`git status UAV_Control/` 为空）。
> 所有差异都发生在外层：`sim/ros2/x650_offboard/x650_offboard/` 里的节点与预测器。
> 本文逐项列出「理论是什么」「现在是什么」「为什么改」「是否属于权宜之计」。

## 1. 原始算法（上游 `UAV_Control/`）

### 1.1 结构

```text
aug_lqr.py                 预览增强型 LQR 核心（离散 DARE 求解）
UAV_params.py              机体参数 + LQR 权重（默认 DJI F450）
direct/point_landing.py    固定落点适配器（参考速度恒为 0）
mionet/reference_controller.py
                           LandingReference + ReferencePredictor 协议 + MIONetLandingController
```

### 1.2 数学模型

- 状态量：`[位置误差(3), 速度误差(3)]`，即 `x = [p - p_ref, v - v_ref]`
- 离散双积分模型（步长 `dt=0.05 s`）：
  `A = [[I, dt·I], [0, I]]`， `B = [[0.5·dt²·I], [dt·I]]`
- 增益：`K = (R + BᵀPB)⁻¹BᵀPA`，`P` 由离散 Riccati 方程（DARE）求得
- 控制律：`u = -K·x`（再按 `max_acceleration` 饱和）
- 三阶段权重：

| 阶段 | 位置权重 | 速度权重 | 说明 |
|---|---|---|---|
| `HOVER` | (16,16,25) | (4,4,9) | 基准 |
| `TRACKING` | 同上 × `tracking_*_scale` | | 跟踪移动目标 |
| `TOUCHDOWN` | 同上 × 2 | 同上 × 2 | 末端更硬 |

- **预览前馈**（可选）：把未来 `preview_horizon` 步的曲率序列并入增广状态，
  在 `curvature_weight > 0` 时生效，用于补偿参考轨迹的弯曲。
- 输出：饱和后的三轴加速度 `[ax, ay, az]`，**不含**姿态控制、电机混控、ROS/PX4。

### 1.3 两种参考生成方式

| 方式 | 参考位置 | 参考速度 |
|---|---|---|
| `PointLandingController` | 固定点 | **恒为 0** |
| `MIONetLandingController` | 由外部 `predictor(observation)` 给出 | 同左 |

`predictor` 需返回 `LandingReference(position, velocity, curvature_preview)`。
上游**不包含** MIONet 网络本身，只定义协议。

## 2. 当前实现

### 2.1 控制链路

```text
/ship/pad_pose(_gps / _vis) ──┐
/fmu/out/vehicle_local_position_v1 ──┴→ PadHistory（观测历史）
                                   → ConstantTurnPredictor（预测器）
                                   → LandingReference
                                   → MIONetLandingController.compute_control
                                   → AugmentedLQRController（未改）
                                   → 加速度指令 → PX4 OFFBOARD
```

### 2.2 任务状态机（外加上去的）

`wait → takeoff → transit → approach → descend → settle → landed`

| 状态 | 控制方式 | 目标 |
|---|---|---|
| `takeoff` | **位置设定点**（PX4 位置环） | 爬到 `cruise_alt`（默认 10 m） |
| `transit` | LQR 加速度 | 平台上空 `cruise_alt` |
| `approach` | LQR 加速度 | 对准后降到板面上方 `clearance`（0.5 m） |
| `descend` | LQR 加速度（touchdown 阶段 + 下降率前馈） | 板面 |
| `settle` | LQR 加速度 | 压紧，等待触地判据 |
| `landed` | — | 上锁停机 |

**起飞段用的是 PX4 位置环，不是 LQR**——上游理论不含"起飞"这种任务逻辑。

### 2.3 预测器（`predictors.py`，MIONet 的占位）

- 对最近 2 s 观测做最小二乘 → 线速度 `v`
- 对艏向最小二乘 → 角速度 `ω`
- 外推：`p_ref = p_last + v·h + z_offset`
- 曲率：`κ = ω / |v|`，复制 `preview_horizon`(=10) 份作为预览序列
- **不是** MIONet 网络，接口与 `ReferencePredictor` 一致，可替换

## 3. 差异清单

### 3.1 结构性差异（任务层，不算偏离理论）

| 项 | 理论 | 当前 | 性质 |
|---|---|---|---|
| 起飞 | 无 | 位置设定点 | 任务逻辑 |
| 状态机 | 无 | 7 状态 | 任务逻辑 |
| 触地判定 | 无 | 真值/视觉距离 + 上锁 | 任务逻辑 |
| 解锁门限 | 无 | 等 EKF `xy_valid && z_valid` | 安全逻辑 |

### 3.2 控制层差异（**属于偏离**，需逐项审视）

| # | 项 | 理论 | 当前 | 为什么加 | 是否权宜 |
|---|---|---|---|---|---|
| A | 预测视界调度 | 固定 `preview_horizon` | 按阶段 0.8 / 0.3 / 0.2 / 0 s | 固定 1 s 视界在 1.5 m/s 下等于瞄准船前方 1.5 m，末端永远偏 | 是 |
| B | 垂直偏移 `z_offset` | 参考 = 落点 | 巡航 −10 m / 进场 −0.5 m / 下降 −0.02 m | 预测器只输出平台位置，巡航目标应是"平台上空" | 部分是（巡航段合理，末端偏置可疑） |
| C | 下降率前馈 `descent_rate` | 无 | 0.45 m/s 叠加到参考速度 | 让末端下降更"果断" | **是**（用它补 LQR 收敛慢，理论上不该需要） |
| D | 双源融合 | 单一参考 | GPS 粗定位与视觉按距离线性加权，12→6 m 过渡 | 远距离视觉不可用 | 否（属感知架构） |
| E | 视觉时效 + EMA | 无 | 0.4 s 超时丢弃，EMA 0.6/0.4 | 视觉更新断续（30–35% 帧），参考会冻结跳变 | 是 |
| F | 横向速度门限 | 无 | `rel_v_h < 0.4` 才允许下降 | 横向没追上就下降会撞船体 | 是 |
| G | 对准滞回 | 无 | <0.8 m 置位、>3 m 复位 | 硬阈值会在 1 m 附近反复升降 | 是 |

### 3.3 已修复的实现缺陷（不是算法差异，但影响结果）

| 缺陷 | 影响 | 状态 |
|---|---|---|
| 观测历史被 yaw 判空挡住 | 预测器恒输出原点 | 已修 |
| 相机安装漏算 `base_link` 0.28 m | 视觉 z 偏 0.25–0.48 m | 已修 |
| 内参用 FOV 反推 | 距离尺度偏大 ~15% | 已改 `camera_info` |
| 远距离误检 | 视觉位置误差 60–80 m | 已加距离/像素/中值过滤 |
| **`descent_rate` 重复计入** | 参考位置多下降 `0.45·h`（≈0.09 m） | 已修（2026-11-04） |

## 4. 当前结论与未解决项

### 已验证

| 场景 | 结果 |
|---|---|
| 定点着陆（`PointLandingController`） | 2.6 cm |
| 移动平台着陆（真值参考，船 1–2 m/s） | 0.10–0.21 m |
| 视觉闭环（静止船，ArUco 精定位） | 0.11 m / 0.48 m（两次） |
| 感知延迟敏感度 | ≤150 ms 可落，≥300 ms 失效 |

### 未解决

1. **末端悬停在板上方 0.4–0.55 m 不下压**：垂直加速度指令只有 ±0.3 m/s²，说明垂直参考误差接近零。
   已排除"飞控高度漂移"（实测 ±5 cm）。嫌疑集中在 `z_offset` 与 `descent_rate` 的组合。
2. **移动船 + 视觉追不上**：横向滞后约 2 m。
3. **触地判定依赖真值**：真机上必须换纯机载判据。

## 5. 如果要"回到理论"

建议做一个对照实验：

1. 去掉 C（`descent_rate`）与 B 的末端偏差，参考只保留"落点位置 + 平台速度 + 平台曲率预览"
2. 用纯 LQR 输出，不加任何门限/滞回
3. 看能否落到板上，并记录误差

- 若能落 → 说明理论方案本身够用，现有补丁可以删
- 若不能落 → 说明需要对**参考生成**（预测器）或**内环延迟**做补偿，这才是真正该改的地方

这份对照实验是判断"我们的实现偏离理论多少"最直接的证据，建议作为下一步。
