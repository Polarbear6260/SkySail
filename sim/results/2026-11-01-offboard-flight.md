# OFFBOARD 外部控制飞行（2026-11-01）

首次用 ROS 2 + `px4_msgs` + uXRCE-DDS 从外部控制 X650 飞完全程。

## 链路

```
ros2 run x650_offboard  ──DDS──▶  MicroXRCEAgent:8888  ──UDP──▶  PX4 SITL (uxrce_dds_client)
        ▲                                                              │
        └──────────── /fmu/out/*（BEST_EFFORT + TRANSIENT_LOCAL）◀─────┘
```

## 飞行结果

| 项 | 目标 | 实测 |
|---|---|---|
| 模式 | OFFBOARD (`nav_state=14`) | ✅ 已进入 |
| 解锁 | 外部命令 | ✅ `Armed by external command` |
| 起飞 | `Takeoff detected` | ✅ |
| 定高 | 2.50 m | 2.48 – 2.51 m（波动 0.03 m） |
| 航点 1 | (2.0, 0.0, 2.5) | 8.7 s 到达 |
| 航点 2 | (2.0, 2.0, 2.5) | 10.4 s 到达 |
| 航点 3 | (0.0, 0.0, 2.5) | 回到原点上方 |
| 轨迹包络 | — | x ∈ [−0.55, 2.26] m，y ∈ [−0.31, 1.85] m |
| 到位精度 | 0.5 m 判定半径 | 末速度 vx/vy/vz ≈ 0.02/0.005/0.03 m/s |

## 复现命令

```bash
# 三个终端：agent / SITL / 节点（详见 ros2/README.md）
ros2 run x650_offboard offboard_hover --ros-args -p alt:=2.5 \
    -p 'waypoints:=2.0 0.0 2.5, 2.0 2.0 2.5, 0.0 0.0 2.5'
```

## 需要注意

- 该次飞行用 `force-arm` 兜底（因为 `NAV_DLL_ACT=2` 要求 GCS 在线）；接 QGC 后不需要。
- 轨迹来自 ulog 的 `vehicle_local_position`，可用 `scripts/06_analyze_ulog.py` 复核。
- 下一步：接 SkySail 的 `PointLandingController`（换 X650 参数）做定点着陆，再上 3DVFH*。
