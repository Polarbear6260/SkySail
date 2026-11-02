# SkySail · X650 移动平台着陆

科研用无人机（**Holybro X650 V2 + CUAV V6X + Jetson Orin Nano + Gemini 335L**）的
**自主着陆控制算法**与配套仿真环境。核心目标：让无人机落到**移动平台**（海上船只）上。

```text
UAV_Control/     着陆外环控制库（LQR + preview 前馈，可接 MIONet 预测器）
sim/             PX4 SITL + Gazebo Harmonic 仿真环境（按 BOM 参数建模）
```

## 快速开始

```bash
cd sim
./scripts/16_moving_landing_demo.sh              # 一键：船随机航行 → 起飞 10 m → 追船 → 触板
./scripts/16_moving_landing_demo.sh --dashboard  # 同上，另开实时调试面板
```

约 75 秒自动结束，输出控制日志、Gazebo 真值 CSV 和视频到 `sim/logs/`。

## 目前做到哪一步

| 能力 | 结果 |
|---|---|
| 悬停油门（与 BOM 对照） | 52.72% vs 53.00% |
| 固定点着陆 | 落点误差 **2.6 cm** |
| **移动平台着陆** | 船速 1–2 m/s 随机航行，落点误差 **8–21 cm**，多次验证 |
| 控制结构 | 观测历史 → 预测器 → `LandingReference` → LQR（含曲率 preview 前馈） |
| 可复现性 | 每次随机任务打印 `seed=`，`SHIP_SEED=N` 可完全复现 |

## 文档

| 文档 | 内容 |
|---|---|
| [`sim/README.md`](sim/README.md) | 仿真环境总览 |
| [`sim/docs/01-bom-to-sim.md`](sim/docs/01-bom-to-sim.md) | BOM → 仿真参数、惯量/质心、参数来源与不确定度 |
| [`sim/docs/02-acceptance.md`](sim/docs/02-acceptance.md) | 协议验收数据 |
| [`sim/docs/03-reproduce.md`](sim/docs/03-reproduce.md) | 从零复现指南（含一键 demo） |
| [`sim/docs/04-moving-platform.md`](sim/docs/04-moving-platform.md) | 移动平台着陆全过程与已知限制 |
| [`sim/docs/05-architecture.md`](sim/docs/05-architecture.md) | 程序结构图（Mermaid） |
| [`sim/ros2/README.md`](sim/ros2/README.md) | ROS 2 + uXRCE-DDS 控制链路 |
| [`UAV_Control/README.md`](UAV_Control/README.md) | **控制库 API 细节**（`PointLandingController` / `MIONetLandingController`） |

## 已知限制

- 仿真中着陆后机体仍会从运动学甲板上滑落（PX4 持有模型位姿 + 甲板是 `set_pose` 瞬移），
  属建模问题；按约定"触板一次"即判定算法成功。
- `MIONet` 目前是"恒速 + 恒转向"基线预测器，接口与上游 `ReferencePredictor` 一致，
  换成训练好的网络即可。
- 不模拟电池放电曲线、Jetson 算力、数传链路、双目视差；续航与链路结论以真机为准。
