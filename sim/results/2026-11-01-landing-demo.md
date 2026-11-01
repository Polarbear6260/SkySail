# 定点着陆 demo（SkySail LQR 外环 + PX4 OFFBOARD，2026-11-01）

把仓库里的理论控制器（`UAV_Control.direct.PointLandingController`）接进仿真，
完成「起飞 → 8 m 外进场 → 落点上方 → 垂直下降 → 着陆」。

## 控制结构

```
PX4 local_position (NED) ──▶ PointLandingController ──▶ 加速度指令 (m/s²)
                                    ▲                        │
                                    │                        ▼
                             UAV_params（X650）      TrajectorySetpoint.acceleration
                                                             │
                                                             ▼
                                              PX4 OFFBOARD（内部含重力补偿）
```

- 控制器参数换成 X650：`mass=3.5767 kg`、`I=(0.1240, 0.1240, 0.2265)`、
  `dt=0.05 s`、加速度限幅 `(3.0, 3.0, 2.5) m/s²`、最大倾角 25°
- LQR 权重沿用上游：位置 `(16,16,25)`、速度 `(4,4,9)`、控制 `(1,1,1.5)`、preview 10
- 进近阶段把控制器目标设在**进近高度**（先平飞），到位后再垂直下降，避免“一边平移一边撞地”

## 实测结果

| 项 | 结果 |
|---|---|
| 起飞高度 | 2.50 m（悬停 2.48–2.51 m） |
| 进近距离 | 8.0 m（x 从 0 到 7.88 m） |
| 进近耗时 | ≈ 5.7 s（平均 1.4 m/s） |
| 进近段高度保持 | 2.21 – 2.51 m |
| 水平横向偏差 | y ∈ [−0.40, 0.29] m |
| **落点误差** | **0.026 m** |
| 轨迹点数 | 285（20 Hz 采样） |

> 落点误差 = 落地瞬间 PX4 估计位置与目标落点 (8.0, 0.0) 的水平距离。

## 复现

```bash
# 终端 1：agent        终端 2：SITL（见 ../ros2/README.md）
ros2 run x650_offboard landing_demo --ros-args -p alt:=2.5 -p landing_x:=8.0
```

轨迹会写到 `~/x650_ws/landing_traj.csv`（`x_m,y_m,alt_m`）。

## 结论与局限

- 外环 LQR 与 PX4 内环级联是稳定的：8 m 进场、2.5 m 高度、落点误差 2.6 cm。
- 该 demo 只验证**定点**着陆；SkySail 的另一条 `MIONetLandingController` 分支（移动落点）尚未接入。
- 无风、无扰动、无气动干扰，真机上误差会更大；后续可在 world 里加风场（`windy.sdf`）做鲁棒性对比。
