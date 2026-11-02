# X650 仿真环境复现指南

> 目标读者：拿到这台机器（或另一台 Ubuntu 24.04）的实验室同学。
> 照着本文从零走一遍，约 **1.5–2 小时**（其中 1 小时是国内网络等待），得到一套能跑 X650 的
> PX4 SITL + Gazebo Harmonic 环境，并产出可直接写进汇报的数据。
>
> 环境已验证：**PX4 v1.17.0 + Gazebo Harmonic 8.15.0 + Ubuntu 24.04.5**，2026-10-30 实跑通过。

---

## 0. 这套环境能给你什么

| 能力 | 状态 |
|---|---|
| 按 BOM 参数（3.5767 kg / 650 mm 轴距 / 15×5 桨）仿真的 X650 四旋翼 | ✅ |
| 起飞 → 悬停 → 着陆全流程，无需真机 | ✅ |
| 悬停油门复算：**52.72% vs BOM 53.00%**（偏差 −0.28 个百分点） | ✅ |
| 悬停高度保持：2.50 m 目标，20 s 内波动 **0.017 m**（flight 全程 0.598 m） | ✅ |
| 磁强一致性：实测 0.4807 G vs WMM 参考 0.4824 G（偏差 0.3%） | ✅ |
| 深度相机（Gemini 335L 代理）话题 `/depth_camera`、`/depth_camera/points` | ✅（软件渲染） |
| 避障世界：绕桩 → 门洞 → 走廊 → 建筑 | ✅ |
| 3DVFH* / ROS 2 闭环 | ⏳ 待做（见 `docs/02-acceptance.md` 阶段 3） |

> 一句话结论：**仿真复现了 BOM 表推算的悬停油门，误差 0.3 个百分点**。这条可以直接作为
> “仿真与设计一致”的证据。

---

## 1. 版本矩阵（务必对齐，版本错一个就编译不过）

| 组件 | 版本 | 说明 |
|---|---|---|
| Ubuntu | 24.04.x (noble) | jammy 需要另换 gz 版本，不建议 |
| PX4-Autopilot | **v1.17.0**（tag） | 稳定版，2026-05 发布 |
| Gazebo | **Harmonic / gz-sim 8.15.0** | PX4 v1.17 配套；上游 `ubuntu.sh` 装的就是 `gz-harmonic` |
| gz 模型库 | `PX4/PX4-gazebo-models` @ 跟随 PX4 子模块 | v1.17 起 `Tools/simulation/gz` 是它的子模块 |
| Python | 3.12（venv） | 编译用 `empy==3.3.4` 等，见 `Tools/setup/requirements.txt` |
| 编译器 | gcc/g++ 13.3 | noble 自带 |

---

## 2. 前置条件

| 项 | 要求 | 检查命令 |
|---|---|---|
| 系统 | Ubuntu 24.04 x86_64 | `lsb_release -a` |
| 磁盘 | ≥ 15 GB 可用（源码 2 GB + 构建 2 GB + apt 1.5 GB） | `df -h /` |
| 内存 | ≥ 8 GB（构建期） | `free -h` |
| 网络 | 能访问 github.com、packages.osrfoundation.org、pypi | 见 §5 |
| 权限 | 有一个 sudo 账号（装 apt 包） | `sudo -v` |
| GPU / 显示 | **不要求**。无 GPU 也能跑（软件渲染，会有 `libEGL warning`，不影响使用） | — |

本文所有命令默认工作目录为 `px4-gz-sim/`，源码目录为 `~/PX4-Autopilot`（可用 `PX4_DIR` 覆盖）。

---

## 3. 五步复现

### 步骤 1：装系统依赖（唯一需要 sudo 的一步）

```bash
cd px4-gz-sim
sudo ./scripts/00_install_host_deps.sh
```

做的事：apt 更新 → 编译工具链（cmake/ninja/g++/python3-venv）→ 加 OSRF 源 → 装
`gz-harmonic libunwind-dev cppzmq-dev` + gstreamer/opencv 运行依赖 → 把当前用户加入 `dialout`。
（可选 `--with-ros` 一并装 ROS 2 Jazzy + `ros_gz_bridge`，约多 2 GB。）

**预期**：约 350–500 MB 下载，结束打印

```
== [5/5] 验证 ==
  ✔ gz     /usr/bin/gz
  gz 版本：8.15.0
  ✔ cmake ...
```

### 步骤 2：拉源码（约 400 MB）

```bash
./scripts/01_fetch_sources.sh --mirror
```

- `--mirror` 走 gh-proxy（本机实测 140–170 KB/s；直连只有 17 KB/s，要 1 小时）
- 只拉 SITL 需要的 14 个子模块，**跳过** NuttX / tflite / zenoh 等约 300 MB
- 自动 `--recursive`：`mavlink` 的 `pymavlink`、`uxrce` 的 `Micro-CDR` 是嵌套子模块，漏了会编译失败
- 弱网自动重试 5 次（低速 45 s 主动失败，clone 不能断点续传）

**预期**：

```
  浅克隆第 1/5 次（约 65–300 MB）...
  ✔ 模型库已就位
  PX4      v1.17.0
  gz 模型  36 个型号
```

### 步骤 3：编译 `px4_sitl`（约 3–8 分钟，20 核）

```bash
./scripts/02_build_sitl.sh
```

脚本会：建 `.venv-px4`（避免 PEP 668）→ pip 装依赖（默认清华源）→ `make px4_sitl`。
其中还特意做了两件事，**不要删**：

- `unset VSCODE_PID CI CLION_IDE` + `GIT_SUBMODULES_ARE_EVIL=1`
  —— 否则 PX4 的 `check_submodules.sh` 会把 NuttX/tflite 全递归拉一遍（多 600 MB）
- pip 走 `PIP_INDEX_URL`（默认清华）

**预期**：`✔ 编译完成：/home/green/PX4-Autopilot/build/px4_sitl_default/bin/px4`（51 MB）

### 步骤 4：装 X650 资产（模型 / 世界 / airframe）

```bash
./scripts/03_install_gz_assets.sh
./scripts/02_build_sitl.sh          # airframe 变更必须重编
```

`03` 会做三件事，其中第 3 件容易漏：

1. 拷 4 个模型：`x650_base` / `x650` / `x650_depth` / `gemini_335l` → `Tools/simulation/gz/models/`
2. 拷世界 `x650_obstacles.sdf` → `Tools/simulation/gz/worlds/`
3. 拷 airframe 并把 **`22000_gz_x650` / `22001_gz_x650_depth` 登记进
   `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`**
   —— 不登记 ROMFS 里就没有这个机型，`make px4_sitl gz_x650` 直接找不到目标。
   上游把 **22000–22999 预留给自定义机型**，本套用 22000/22001。

**预期**：

```
== 校验 ==
  ✔ 模型自检通过
  ✔ model://x650 就位
  ✔ airframe 22000_gz_x650 已登记
```

### 步骤 5：验证（三个层次，全绿才算复现成功）

```bash
# ① 离线自检：不需要 PX4/Gazebo，秒级
make check

# ② 端到端冒烟：约 60 秒，自动判定
./scripts/04_run_sitl.sh --smoke

# ③ 演示飞行 + 数据提取（可选，产出汇报数字）
./scripts/04_run_sitl.sh            # 终端 A：把仿真跑起来
./scripts/07_demo_flight.py         # 终端 B：解锁→起飞 2.5 m→悬停→着陆
./scripts/06_analyze_ulog.py        # 终端 B：出统计表
```

**期望输出（实测）**：

```
① make check                     结果：全部通过
② --smoke                        ✔ PX4 预检通过（Ready for takeoff）
                                 ✔ Gazebo 世界加载并 spawn 模型
                                 ✔ gz_bridge 已连上模型
                                 结果：通过
③ 06_analyze_ulog.py             悬停油门 52.72 %  (BOM 53.00 %，偏差 -0.28 pp)
                                 平均高度 2.47 m，20 s 波动 0.017 m
                                 磁强 0.4807 G vs 0.4824 G，偏差 0.3%
```

---

## 4. 目录结构

```
px4-gz-sim/
├── README.md                 # 总览
├── Makefile                  # check / fetch / build / install / smoke / run
├── scripts/
│   ├── 00_install_host_deps.sh   # ① sudo：apt + OSRF + gz-harmonic
│   ├── 01_fetch_sources.sh       # ② PX4 + 子模块 + gz 模型库（--mirror 走镜像）
│   ├── 02_build_sitl.sh          # ③ venv + make px4_sitl（含两个子模块坑的规避）
│   ├── 03_install_gz_assets.sh   # ④ 装资产 + 登记 CMakeLists
│   ├── 04_run_sitl.sh            # ⑤ 运行 / --smoke / --gui
│   ├── 05_check_model_math.py    # 离线自检（质量/推力/悬停点）
│   ├── 06_analyze_ulog.py        # ulog 成果统计
│   └── 07_demo_flight.py         # 自动演示飞行（无 QGC）
├── gz_extras/
│   ├── models/x650_base/     # 3.2167 kg 机体 + 4×0.09 kg 旋翼 + IMU/磁/气压/GPS
│   ├── models/x650/          # + 4 路 MulticopterMotorModel（按台架标定）
│   ├── models/gemini_335l/   # 深度相机代理（0.17–20 m，90°×65°）
│   ├── models/x650_depth/    # x650 + 机头下挂相机
│   ├── worlds/x650_obstacles.sdf  # 绕桩→门洞→走廊→建筑 + 地理原点
│   └── airframes/            # 22000_gz_x650 / 22001_gz_x650_depth
└── docs/
    ├── 01-bom-to-sim.md      # BOM → 参数的每一项推导
    ├── 02-acceptance.md      # 分阶段验收清单
    └── 03-reproduce.md       # 本文
```

---

## 5. 网络：本机实测与开关

同一时段实测（2026-10-30，国内家宽）：

| 线路 | 速度 | 65 MB 耗时 |
|---|---:|---:|
| github.com 直连 | ~17 KB/s | ~1 小时 |
| `socks5://127.0.0.1:7897`（Clash） | ~68 KB/s | ~16 分钟 |
| `--mirror`（gh-proxy.com） | 140–170 KB/s | 6–8 分钟 |
| PyPI 直连 | 几乎不可用 | — |
| PyPI 清华源 | 正常 | 秒级 |

对应开关：

```bash
./scripts/01_fetch_sources.sh --mirror        # git 走 gh-proxy
export PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple   # 02 默认就是这个
# apt 卡住时（连接挂死等 120 s 超时）：
printf 'Acquire::http::Proxy "http://127.0.0.1:7897";\nAcquire::https::Proxy "http://127.0.0.1:7897";\n' \
  | sudo tee /etc/apt/apt.conf.d/99proxy
```

> **已知网络坑**：`raw.githubusercontent.com` 在这台机器上只解析到 IPv6 且超时。
> 用 GitHub API 取文件可以正常工作；`git clone github.com` 是 IPv4，正常。

---

## 6. 七个已知坑（都已在脚本里规避，换机器复现时会再遇到）

| # | 现象 | 根因 | 规避 |
|---|---|---|---|
| 1 | `make px4_sitl` 在 configure 阶段卡住拉 NuttX | shell 里带 `VSCODE_PID`/`CI` 时，`check_submodules.sh` 递归更新所有子模块 | `02` 里 `unset` 这些变量 + `GIT_SUBMODULES_ARE_EVIL=1` |
| 2 | 编译报 `mavgen.py` 缺失 | `pymavlink` 是 mavlink 的**嵌套**子模块 | `01` 的 `--recursive` |
| 3 | 编译报 OpenCV 找不到 | `gz_plugins/optical_flow` 硬性 `find_package(OpenCV)` | `00` 装 `libopencv-dev` |
| 4 | `Preflight Fail: Strong magnetic interference` | world 缺 `<spherical_coordinates>`：磁强按特斯拉原值发布、WMM 参考点退化到 (0,0) | 世界文件里补苏黎世原点（已补） |
| 5 | `Preflight check: FAILED` 却无其他报错 | `NAV_DLL_ACT=2` 要求必须有 GCS 数据链才能解锁 | 连 QGC；或测试时置 0（`04 --smoke` 自动做） |
| 6 | `PX4 server already running for instance 0` | 上次仿真残留进程 / `/tmp/px4_lock-0` | `pkill -f 'px4_sitl_default/bin/px4'; rm -f /tmp/px4_lock-0`（`04` 自动清） |
| 7 | 日志暴涨到 1 GB+ | stdin 接 `/dev/null` 时 PX4 shell 读到 EOF，疯狂刷 `pxh>` | `04` 用长期打开的管道喂 stdin（日志降到 ~8 KB） |

---

## 7. 排错速查

| 现象 | 处理 |
|---|---|
| `gz sim --versions` 报 command not found | `00` 没跑完；重跑（幂等，只补缺的包） |
| apt 卡在某包 0 KB/s 超过 2 分钟 | Ctrl-C，设 §5 的 apt 代理后重跑 |
| `Waiting for simulator` 一直不动 | 模型没装或名字不对：`ls ~/PX4-Autopilot/Tools/simulation/gz/models | grep x650` |
| `make px4_sitl gz_x650` 报 No rule to make target | airframe 没登记或没重编：跑 `03` 再 `02` |
| 预检失败但看不出原因 | `./scripts/04_run_sitl.sh --smoke`，日志里 grep `Preflight Fail`；或 `07` 连上后用 `commander check` |
| 深度相机没话题 | 先确认仿真真的起来了（残留锁会让新实例秒退） |
| 有 CUDA/GPU 的机器想开 GUI | `./scripts/04_run_sitl.sh --gui`（需要 `DISPLAY`） |

---

## 8. 资源与时间

| 阶段 | 下载 | 磁盘 | 时间（本机 20 核 / 家宽） |
|---|---:|---:|---|
| apt 依赖 | 350–500 MB | ~1.5 GB | 10–40 分钟（取决于网络） |
| 源码 + 子模块 + 模型库 | ~400 MB | ~1.5 GB | 10–20 分钟（镜像） |
| 编译 | — | ~2 GB | 3–8 分钟 |
| 冒烟 + 演示飞行 | — | 日志 <1 MB | 各 1–2 分钟 |
| **合计** | **~0.8 GB** | **~5 GB** | **1.5–2 小时** |

---

## 9. 参数与 BOM 的对应（简表，推导见 `docs/01-bom-to-sim.md`）

| 量 | BOM | 仿真 |
|---|---|---|
| 起飞重量 | 3.5767 kg | 3.2167 + 4×0.09 = 3.5767 kg |
| 轴距 / 桨 | 650 mm / 15×5 | rotor 位姿 ±0.2298 m / 桨 0.381 m |
| 单电机满油门 | 1920 g | k·ω²，ω_max=460 rad/s，k=8.9012e-5 |
| 悬停油门 | 53.0% | **实测 52.72%**（模型算 52.88%） |
| 推重比 | 2.147 | 2.147 |
| 惯量 | 未给 | Ixx=Iyy=0.1240、Izz=0.2265（箱体估算） |

---


## 11. 移动平台着陆 demo（一键）

全套环境装好后，验证"算法层面的移动平台着陆"只需一条命令：

```bash
cd sim
./scripts/16_moving_landing_demo.sh            # 带 GUI + 录制（约 75 秒）
./scripts/16_moving_landing_demo.sh --no-gui   # 无头服务器
```

它会：清场 → 起 DDS agent → 起 SITL（机体生成在固定的 `dock` 上）→ 设仿真参数 →
起 GUI/录制 → 让船以 1.5 m/s 直线航行 → 无人机爬升 10 m → 追船 → 下降 →
**触板判定成功即停**。

预期输出（2026-11-02 实测）：

```
接触（相对高度 -0.10 m，相对速度 0.27 m/s，水平误差 0.09 m）
TOUCHDOWN 成功：高于板面 -0.00 m，水平误差 0.24 m，时刻 t=28.5s
```

产物：`logs/run-*.log`、`logs/pose-*.csv`（Gazebo 真值）、`logs/x650-moving-*.mp4`。

依赖可用环境变量覆盖：`PX4_DIR`、`AGENT_BIN`、`SKYSAIL`、`WX650`。
细节与已知限制见 `docs/04-moving-platform.md`。

## 10. 给汇报用的材料建议

1. **一致性证据**（最有力）：`06_analyze_ulog.py` 输出 → 悬停油门 52.72% vs BOM 53.00%，
   磁强偏差 0.3%，高度保持 0.017 m 波动。
2. **视觉材料**：有 GPU 的机器上 `04_run_sitl.sh --gui` 录一段绕桩飞行；或在 QGC 里截
   `x650_obstacles` 世界 + 深度相机画面。
3. **算法对比**：同一世界跑 3DVFH* 与直接飞直线的轨迹，比最小间距与耗时（阶段 3 产出）。
4. **ulog**：`~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/` 下的 `.ulg` 可用
   [Flight Review](https://review.px4.io/) 网页版可视化，直接截图。

> 注意：本环境**不模拟**电池放电曲线、Jetson 算力、SiK/ELRS 链路、双目视差。
> 续航与链路的结论仍以 BOM 表与真机实测为准，汇报时不要混用。
