# X650 · PX4 SITL + Gazebo Harmonic 仿真环境

按 BOM（Holybro X650 V2 + CUAV V6X + Jetson Orin Nano + Gemini 335L）搭一套**与你实机同参数的**仿真环境，用于在装机/试飞之前验证：

- 悬停油门、推重比、爬升率是否与台架推算一致（BOM 表「重量与续航评估」）
- 3DVFH\* 等避障算法在 6 m 感知距离、≤3 m/s 巡航下的表现
- Jetson 侧 ROS 2 节点与 PX4 的 uXRCE-DDS / MAVLink 链路
- 上真机前的参数（`MPC_THR_HOVER`、`CA_ROTOR*`、失控保护）预填

> 目标版本：**PX4 v1.17.0**（2026-05 稳定版）+ **Gazebo Harmonic（gz-sim8）** + Ubuntu 24.04。
> 上游 `Tools/simulation/gz` 已改为 `PX4/PX4-gazebo-models` 子模块，模型/世界从那里取。

## 这套东西包含什么

```
px4-gz-sim/
├── Makefile                      # 一站式入口
├── scripts/
│   ├── 00_install_host_deps.sh   # ① sudo：apt + OSRF 源 + gz-harmonic（+可选 ROS 2 Jazzy）
│   ├── 01_fetch_sources.sh       # ② 拉 PX4 v1.17.0 + gz 模型库
│   ├── 02_build_sitl.sh          # ③ venv + make px4_sitl
│   ├── 03_install_gz_assets.sh   # ④ 装入 X650 模型/世界/airframe
│   ├── 04_run_sitl.sh            # ⑤ 运行 / --smoke 冒烟 / --gui
│   └── 05_check_model_math.py    # 离线自检（现在就能跑，不需要 PX4/Gazebo）
├── gz_extras/
│   ├── models/x650_base/         # 3.2167 kg 机体 + 4×0.09 kg 旋翼 + IMU/磁/气压/GPS
│   ├── models/x650/              # x650_base + 4 路 MulticopterMotorModel（台架标定）
│   ├── models/gemini_335l/       # 深度相机代理（0.17–20 m、90°×65°）
│   ├── models/x650_depth/        # x650 + 机头下挂相机
│   ├── worlds/x650_obstacles.sdf # 绕桩 → 门洞 → 走廊 → 建筑，对应 3DVFH* 场景
│   └── airframes/                # 22000/22001（自定义号段，03 会登记进 CMakeLists.txt）
├── docs/
│   ├── 03-reproduce.md           # ★ 复现指南（给别人照着做）
│   ├── 01-bom-to-sim.md          # BOM → 仿真参数的每一项推导
│   └── 02-acceptance.md          # 验收清单与阶段 2 待办
└── upstream-ref/                 # 上游 x500/x500_depth/OakD/ubuntu.sh 原件，供对照
```

> **要复现/交接给其他人，直接看 [docs/03-reproduce.md](docs/03-reproduce.md)**，
> 里面有完整的五步命令、实测速度、七个已知坑和汇报素材建议。

## 快速开始

```bash
# 0) 离线自检（无需任何依赖，现在就应通过）
make check

# 1) 宿主机依赖 —— 必须你本人在终端执行（DSH 沙箱无 root）
sudo ./scripts/00_install_host_deps.sh            # 加 --with-ros 可一并装 ROS 2 Jazzy

# 2) 源码 / 编译 / 装资产 / 冒烟
export PX4_DIR=$HOME/PX4-Autopilot
./scripts/01_fetch_sources.sh
./scripts/02_build_sitl.sh
./scripts/03_install_gz_assets.sh
./scripts/02_build_sitl.sh                        # airframe 变更后必须重编
./scripts/04_run_sitl.sh --smoke                   # 120 s 无头，自动判定通过与否

# 3) 日常使用
./scripts/04_run_sitl.sh                           # 无头，x650 + 相机 + 避障世界
./scripts/04_run_sitl.sh --gui                     # 有桌面/GPU 时开 Gazebo GUI
./scripts/04_run_sitl.sh --model x650 --world default   # 裸机，上游默认世界
```

QGroundControl：SITL 起来后会自动连 `udp://:14550`，宿主机直接开 QGC 即可（无需 GUI 转发）。

## 要下多少东西

| 项 | 下载 | 占用 | 能否省 |
|---|---:|---:|---|
| `gz-harmonic` + dev 包（apt，估算） | ~350–500 MB | ~1.5 GB | 省不了，物理引擎本体 |
| PX4 源码 v1.17.0 浅克隆（实测 14k 对象） | ~65 MB | ~300 MB | 省不了 |
| 必要子模块（mavlink / gps / cyphal） | ~20 MB | ~60 MB | 省不了 |
| NuttX + NuttX-apps 子模块 | ~300 MB | ~700 MB | ✅ `01` 默认跳过（编 SITL 不需要） |
| Gazebo 模型库 PX4-gazebo-models | ~60 MB | ~100 MB | 省不了（复用 x500_base 等） |
| 编译产物 `build/` | — | ~1.5 GB | 可随时删掉重建 |
| ROS 2 Jazzy | ~2 GB | ~5 GB | ✅ 默认不装，`00 --with-ros` 才装 |
| **默认合计** | **~0.5–0.65 GB** | **~3.5 GB** | 磁盘可用 147 GB |

一句话：**默认路径要下 0.5–0.65 GB，装完占 3.5 GB 左右**；比"全都要"省 2.5 GB 以上。
真正的大头是 apt 的 Gazebo 依赖（一次装好长期复用）和 PX4 源码，躲不开。

## 网络慢怎么办（本机实测，2026-10-30）

git 拉 GitHub 时如果卡在 `接收对象中: 38%` 然后 `unexpected disconnect`，是链路被限速/掐断，
不是脚本问题。本机同一时段实测：

| 线路 | 速度 | 克隆 PX4 浅克隆（65 MB）耗时 |
|---|---:|---:|
| github.com 直连 | ~17 KB/s | ~1 小时 |
| `socks5://127.0.0.1:7897`（Clash） | ~68 KB/s | ~16 分钟 |
| `--mirror`（gh-proxy.com） | ~140–170 KB/s | 6–8 分钟 |

所以这台机器上直接用镜像模式：

```bash
./scripts/01_fetch_sources.sh --mirror
```

`--mirror` 会把 PX4 主仓、子模块（用 `insteadOf` 重写）和 gz 模型库都切到镜像。
镜像仍失败时再试：`export https_proxy=http://127.0.0.1:7897`（apt 用，git 需 socks5 变量
`ALL_PROXY=socks5h://127.0.0.1:7897`）。

## 已核对 / 未核对

| 项目 | 状态 |
|---|---|
| 文件结构、型号名、插件名、airframe 编号（22000/22001，上游为自定义机型预留 22000–22999） | ✅ 对照 PX4 v1.17.0 + PX4-gazebo-models 上游原文 |
| 质量 3.5767 kg、推重比 2.147、悬停油门 52.9%、台架曲线偏差 ≤8.2% | ✅ `make check` 实测通过 |
| XML 可解析、airframe shell 语法、`model.config` 与 `model.sdf` 名称一致 | ✅ 同上 |
| include 目标存在、4 个电机插件引用的 joint/link 真实存在、相机关节挂接正确 | ✅ 同上 |
| 端到端冒烟：SITL + Gazebo + X650 spawn + 预检通过（`Ready for takeoff`） | ✅ 2026-10-30 实跑通过 |
| 深度相机话题 `/depth_camera`、`/depth_camera/points` 发布 | ✅ 实跑确认（软件渲染，EGL 警告不影响） |
| 悬停油门实测、角速度响应、载荷余量 | ⏳ 阶段 2 |
| 点云 → ROS 2 → 3DVFH* 闭环 | ⏳ 阶段 3 |
| 续航（电池放电曲线） | ❌ 未建模：Gazebo 里没有 6S 6800mAh 的电池模型，续航仍以 BOM 表为准 |

## 实跑踩到的两个坑（已修，改动都留在文件里）

1. **world 必须有 `<spherical_coordinates>`**。
   缺了它，Gazebo 的磁力计按特斯拉原值发布（4.8e-5），而 WMM 参考点退化到 (0,0)（0.319 G），
   PX4 判 `Strong magnetic interference`，预检直接不过。补上苏黎世原点后实测
   `mag_strength_gs=0.4807` vs 参考 `0.4824`，完全对得上。

2. **`NAV_DLL_ACT ≥ 1` 会让无头仿真无法解锁**。
   airframe 沿用上游的 `NAV_DLL_ACT 2`，PX4 的 `rcAndDataLinkCheck` 因此要求必须有一条 GCS 数据链；
   不连 QGC 就永远 `Preflight check: FAILED`。两种解法：连 QGC，或在测试里把该参数置 0
   （`04 --smoke` 已自动用 MAVLink shell 做这一步，不影响 airframe 里的真机配置）。

## 两个环境限制（重要）

1. **DSH 沙箱里没有 root**：`sudo` 被 `NoNewPrivs` 拦掉，`/usr`、`/opt`、`$HOME` 都不可写。
   所以 `apt install gz-harmonic`、克隆到 `~/PX4-Autopilot` 这类操作必须在**你自己的终端**执行。
2. **沙箱里没有 GPU、也没有 X**：`/dev/dri`、`/dev/nvidia*`、`/tmp/.X11-unix` 都不存在。
   沙箱内只能跑无头 `gz sim -s -r`（软件渲染）；Gazebo GUI 和 QGC 请在桌面会话里开。

另有一个踩过的坑：`raw.githubusercontent.com` 在你这台机器上只解析到 IPv6 且超时。
`git clone github.com` 正常，但如果哪一步卡在 raw 下载，设置
`export https_proxy=http://127.0.0.1:7890` 或用 `PX4_GIT_URL=https://gitee.com/mirrors/PX4-Autopilot.git`。

## 参数一览（详见 `docs/01-bom-to-sim.md`）

| 量 | BOM | 仿真 |
|---|---|---|
| 起飞重量 | 3.5767 kg | 3.2167 + 4×0.09 = 3.5767 kg |
| 轴距 / 桨 | 650 mm / 15×5 | rotor 位姿 ±0.2298 m / 桨 0.381 m |
| 单电机满油门 | 1920 g | k·ω²，ω_max=460 rad/s，k=8.9012e-5 |
| 悬停油门 | 53.0% | 52.88%（EC_MIN=150、EC_MAX=460） |
| 推重比 | 2.147 | 2.147 |
| 惯量 | —（未给） | Ixx=Iyy=0.1240、Izz=0.2265 kg·m²（箱体估算） |

## 排错

| 现象 | 处理 |
|---|---|
| `make px4_sitl` 报找不到 gz-transport / gz-sim | 依赖没装全，重跑 `00_install_host_deps.sh`，确认 `gz sim --versions` 输出 Harmonic |
| 起来后一直 `Waiting for simulator` | `Tools/simulation/gz/models/x650/model.sdf` 没装进去，或模型名与 `PX4_SIM_MODEL` 不一致 |
| 日志有 `ERROR [gz_bridge]` | airframe 的 `EC_MAX` 与 `maxRotVelocity` 不一致，跑 `make check` 会直接指出 |
| 无 GPU 时 `gz sim` 卡在渲染 | 加 `--headless` 即 `./scripts/04_run_sitl.sh`（默认无头）；GUI 只在有 `DISPLAY` 时用 |
| `PX4 server already running for instance 0` | 上次仿真残留（进程或 `/tmp/px4_lock-0`），`04` 启动前会自动清理；手工清理：`pkill -f 'px4_sitl_default/bin/px4'; rm -f /tmp/px4_lock-0` |
| 深度相机没有话题 | 先确认仿真真的起来了（stale 锁会让新实例直接退出）；正常运行时可看到 `/depth_camera`、`/depth_camera/points` |
| 模型装好但 `make px4_sitl gz_x650` 提示 airframe 不存在 | ① 装完资产必须**重编**；② airframe 必须登记在 `init.d-posix/airframes/CMakeLists.txt`（`03` 脚本会自动追加，重跑它即可） |

## ROS 2 外部控制

`ros2/x650_offboard`：用 `px4_msgs` + uXRCE-DDS 做 OFFBOARD 控制（仿真/真机同一条链路）。

- 接入步骤与四个坑：[`ros2/README.md`](ros2/README.md)
- 首飞结果：[`results/2026-11-01-offboard-flight.md`](results/2026-11-01-offboard-flight.md)
