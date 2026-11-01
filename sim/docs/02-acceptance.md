# 验收清单

按阶段推进，每一阶段给出**命令 + 期望输出**。能自动判定的都写了退出码。

## 阶段 0 · 离线自检（已完成，不需要 PX4 / Gazebo）

```bash
make check          # 等价于 python3 scripts/05_check_model_math.py
```

- [x] 9 个 XML 全部可解析；2 个 airframe 脚本 `sh -n` 通过且可执行
- [x] `model.config` 的 `name` 与 `model.sdf` 的 `<model name>` 一致
- [x] 起飞重量 3.5767 kg（BOM ±0.01 kg）
- [x] `EC_MAX` = `maxRotVelocity` = 460
- [x] 单电机满油门 18.835 N（1920 g）、推重比 2.147
- [x] 悬停油门 52.88%（BOM 53.0%，容差 ±3%）
- [x] 台架曲线最大偏差 8.2%（口径 ≤10%）
- [x] 惯量为正、Ixx≈Iyy、Izz>Ixx

退出码 `0` 才算过。改了任何模型参数都必须重跑这一条。

## 阶段 1 · 依赖 + 编译 + 冒烟（✅ 2026-10-30 已通过）

```bash
sudo ./scripts/00_install_host_deps.sh
./scripts/01_fetch_sources.sh
./scripts/02_build_sitl.sh
./scripts/03_install_gz_assets.sh
./scripts/02_build_sitl.sh          # airframe 变更后必须重编
./scripts/04_run_sitl.sh --smoke
```

检查点：

| # | 检查 | 期望 |
|---|---|---|
| 1.1 | `gz sim --versions` | 含 `8.x.y`（Harmonic） |
| 1.2 | `01` 结束 | `PX4 v1.17.0`，`Tools/simulation/gz/models` 有 ≥30 个型号 |
| 1.3 | `build/px4_sitl_default/bin/px4` | 存在且可执行 |
| 1.4 | `03` 结束 | 4 个模型 + 1 个世界 + 2 个 airframe 就位 |
| 1.5 | `--smoke` 退出码 | `0`；`✔ PX4 预检通过` `✔ Gazebo 世界加载并 spawn 模型` `✔ gz_bridge 已连上模型` |

冒烟日志留在 `logs/sitl-<时间戳>.log`，失败先看它。

## 阶段 2 · 物理标定（真跑起来之后，必须先做）

这几个数字对了，后面的避障结论才有意义。

```bash
./scripts/04_run_sitl.sh --model x650 --world default
# 另开终端，PX4 shell：
#   pxh> listener actuator_motors
#   pxh> listener vehicle_thrust_setpoint
```

| # | 检查 | 方法 | 期望 |
|---|---|---|---|
| 2.1 | 悬停油门 | 起飞后悬停，读 `actuator_motors.control[0..3]` 均值 | 0.50–0.56（模型算的是 0.529） |
| 2.2 | 高度保持 | 定点模式悬停 60 s，看 `vehicle_local_position.z` | 漂移 < 0.3 m，无持续下沉 |
| 2.3 | 爬升率 | 位置模式给 5 m 阶跃 | 1–2 s 内响应，无超调 >15% |
| 2.4 | 偏航响应 | 打满偏航 1 s | 转速平滑，`vehicle_angular_velocity` 无饱和振荡 |
| 2.5 | 满载悬停 | 把 `base_link/mass` 加到 4.17 kg（60% 推力点） | 仍能悬停，油门 ≈0.60 |
| 2.6 | 推力上限 | `pxh> test` 或直接满油门 | 起飞重量 >4.6 kg 时无法悬停（BOM 红线） |

如果 2.1 偏离超过 ±0.05，按 `docs/01-bom-to-sim.md` §3.2 重解 `EC_MAX`/`k`，两处一起改，
然后 `make check` + 重编。

## 阶段 2 前置：两个已修的环境问题

| 现象 | 根因 | 处理 |
|---|---|---|
| `Preflight Fail: Strong magnetic interference` | world 缺 `<spherical_coordinates>`：磁力计按特斯拉原值发布，WMM 参考点退化到 (0,0) | 已在 `x650_obstacles.sdf` 补苏黎世原点；实测 0.4807 G vs 参考 0.4824 G |
| `Preflight check: FAILED`（无其他报错） | `NAV_DLL_ACT 2` 要求必须有 GCS 数据链，无头跑没有 QGC | `04 --smoke` 用 MAVLink shell 置 `NAV_DLL_ACT=0`；真机保持 2，或用 QGC |

## 阶段 3 · 感知与避障闭环

```bash
./scripts/04_run_sitl.sh            # x650 + gemini_335l + x650_obstacles
gz topic -l | grep -E "depth|image" # 确认相机话题
```

| # | 检查 | 期望 |
|---|---|---|
| 3.1 | `gz topic -l | grep -i camera` | 有 `/depth_camera`、`/depth_camera/points`、`/camera_info` |
| 3.2 | ROS 2 `ros_gz_bridge` 起桥后 `ros2 topic hz` | 点云 ≥10 Hz（CPU 软渲染下） |
| 3.2b | 注意：PX4 的 gz_bridge 只把 **LaserScan** 映射成 `distance_sensor`，深度相机不直接进 PX4 | 避障要么走伴侣机→`OBSTACLE_DISTANCE`，要么加 2D 雷达 |
| 3.3 | 3DVFH* 沿 +X 飞 20 m 穿越门洞 | 不撞柱、不撞墙，最小间距 >0.5 m |
| 3.4 | 限速验证 | 巡航 ≤3 m/s，绝对上限 4 m/s（BOM 速度红线） |
| 3.5 | 失控保护 | 断掉 ROS 侧指令，PX4 应悬停/返航（`NAV_DLL_ACT=2`） |

## 已知风险

| 风险 | 影响 | 处置 |
|---|---|---|
| 无 GPU 时软件渲染帧率低 | 相机 30 Hz 掉帧，点云稀疏 | 先降分辨率（640×480 → 320×240），或只在有 GPU 的机器跑阶段 3 |
| `EC_MIN/EC_MAX` 与 `maxRotVelocity` 不一致 | 推力整体缩放，悬停油门偏移 | `make check` 的 `[2]` 项会直接失败 |
| 惯量按箱体估算，比真机偏小 | 角速度响应偏快 | 2.4 若过冲，惯量 ×1.15 |
| 上游 x500 的 `CA_ROTOR*` 与 SDF 位姿本身不一致 | 偏航/滚转混控略有偏差 | 与上游同口径；阶段 2.4 实测后再决定是否统一 |
| PX4 主分支改动快 | 换版本后 `4001_gz_x500` 结构可能变 | 锁定 `v1.17.0`；升级时重跑阶段 0 + 1.5 |

## 回滚

```bash
# 只删本套资产，不动 PX4 主体
rm -rf $PX4_DIR/Tools/simulation/gz/models/{x650,x650_base,x650_depth,gemini_335l}
rm -f  $PX4_DIR/Tools/simulation/gz/worlds/x650_obstacles.sdf
rm -f  $PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/{22000_gz_x650,22001_gz_x650_depth}
./scripts/02_build_sitl.sh
```
