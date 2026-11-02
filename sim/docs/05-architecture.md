# 移动平台着陆：程序结构图

> 代码位置：`sim/ros2/x650_offboard/x650_offboard/moving_landing.py`
> 船驱动：`sim/scripts/10_move_platform.py`　真值记录：`sim/scripts/12_log_pose.py`

## 1. 系统数据流

```mermaid
flowchart LR
    subgraph GZ["Gazebo Harmonic（物理真值）"]
        WORLD["world: x650_sea<br/>sea_surface / ship+pad / dock"]
        DRONE["model: x650_0（机体）"]
    end

    subgraph SITL["PX4 SITL"]
        BRIDGE["gz_bridge"]
        PX4["飞控<br/>EKF · 姿态环 · 位置环"]
    end

    subgraph ROS["ROS 2 节点"]
        SHIPD["10_move_platform.py<br/>船运动驱动"]
        NODE["moving_landing.py<br/>外环（LQR）+ 状态机"]
        MON["12_log_pose.py<br/>真值日志"]
    end

    SHIPD -- "50 Hz set_pose<br/>（gz.transport）" --> WORLD
    SHIPD -- "/ship/pad_pose<br/>/ship/pad_twist（ENU）" --> NODE
    WORLD -- "/world/x650_sea/pose/info" --> NODE
    WORLD -- "/world/x650_sea/pose/info" --> MON

    DRONE <-- "IMU / 电机转速" --> BRIDGE
    BRIDGE <-- "uORB ↔ gz 消息" --> PX4

    NODE -- "/fmu/in/offboard_control_mode<br/>/fmu/in/trajectory_setpoint<br/>/fmu/in/vehicle_command" --> PX4
    PX4 -- "/fmu/out/vehicle_local_position_v1<br/>/fmu/out/vehicle_status_v1" --> NODE
```

**要点**：控制器不直接驱动电机，它只发"设定点"给 PX4；PX4 的 `offboard` 模式要求
**至少 2 Hz 连续收到设定点**，断流 0.5 s 就判 `offboard_control_signal_lost` → RTL。
所以 `loop()` 里每个分支都必须发布，这是踩过三次的坑。

## 2. 状态机

```mermaid
stateDiagram-v2
    [*] --> wait
    wait --> takeoff: 定时器首次触发
    takeoff --> transit: alt > cruise_alt − 0.4
    transit --> approach: rel_h < 0.5 且保持 cruise_hold_s
    approach --> descend: true_above < clearance
    descend --> approach: rel_h > 0.35（横向偏出，重对准）
    descend --> settle: (alt − pad_alt) < 0.18 且 rel_v < 0.3
    settle --> touchdown: true_horiz < 0.35 且 true_above < 0.10，持续 3 s
    touchdown --> finished: 强制上锁（param2=21196）→ 桨停
    finished --> [*]
```

| 状态 | 控制方式 | 目标 |
|---|---|---|
| `takeoff` | 位置设定点 | 爬到 `cruise_alt`（默认 10 m） |
| `transit` | LQR 加速度（tracking 权重） | 板上方 `cruise_alt` 高度 |
| `approach` | LQR 加速度（tracking 权重） | 板面 + 2 cm |
| `descend` | LQR 加速度（**touchdown 权重**，加倍）+ `descent_rate` 前馈 | 板面 |
| `settle` | LQR 加速度（touchdown 权重）+ 0.2 m/s 下压 | 板面 |
| `touchdown` | — | 强制上锁 |

## 3. 每个周期（20 Hz）的决策顺序

```mermaid
flowchart TD
    A["loop() 每 50 ms"] --> B{finished?}
    B -- 是 --> Z["直接返回（不再发指令）"]
    B -- 否 --> C{已解锁?}
    C -- 否 --> C1["发位置设定点<br/>每 5 s 请求解锁<br/>（t>8 s 用 force 21196）"]
    C1 --> Z
    C -- 是 --> D{已请求 OFFBOARD?}
    D -- 否 --> D1["发位置设定点<br/>t>2 s 发 DO_SET_MODE(6)"]
    D1 --> Z
    D -- 是 --> E{"pos_ned / pad_ned 都有?"}
    E -- 否 --> E1["发位置设定点兜底"]
    E1 --> Z
    E -- 是 --> F["算 alt / pad_alt / rel_h / rel_v<br/>（PX4 估计）<br/>算 true_horiz / true_above<br/>（Gazebo 真值）"]
    F --> G{state}
    G -- takeoff --> G1["位置设定点到 cruise_alt"]
    G -- transit --> G2["LQR → 板上方"]
    G -- approach --> G3["LQR → 板面"]
    G -- descend --> G4["LQR(touchdown) + 下降前馈"]
    G -- settle --> G5["LQR(touchdown) + 下压 0.2 m/s"]
    G1 --> Z
    G2 --> Z
    G3 --> Z
    G4 --> Z
    G5 --> Z
```

## 4. 已知问题（图里两套坐标并存的原因）

| 来源 | 量 | 说明 |
|---|---|---|
| PX4 `/fmu/out/vehicle_local_position_v1` | `alt`、`rel_h`、`rel_v` | 相对 **EKF 原点**，与 Gazebo 世界系存在偏差 |
| Gazebo `/world/.../pose/info` | `true_horiz`、`true_above` | **物理真值** |

目前 `approach → descend` 和 `settle → touchdown` 用真值，`descend` 的横向回退用 PX4 估计。
两者实测能差到 **6–12 m**，所以"判到贴合"并不等于真的贴合 —— 这是当前最需要先解决的问题
（详见 `04-moving-platform.md` 的暂停记录）。
