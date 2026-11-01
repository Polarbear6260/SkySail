# ROS 2 侧：OFFBOARD 控制链路

把 PX4 SITL 接到 ROS 2 Jazzy，用 `px4_msgs` + `uXRCE-DDS` 做外部控制。
**仿真与真机同一条链路**（真机上是 Pixhawk V6X 串口跑 XRCE 客户端），代码不用改。

## 版本对齐（错一个就通不了）

| 组件 | 版本 | 说明 |
|---|---|---|
| ROS 2 | Jazzy | `sudo ../scripts/00_install_host_deps.sh --with-ros` |
| `px4_msgs` | **tag v1.17.0** | 与 PX4 v1.17.0 消息定义严格一致；Jazzy 没有 apt 包，必须源码编 |
| `MicroXRCEAgent` | v2.4.3 | PX4 的 `uxrce_dds_client` 连 `udp 8888` |
| Fast-DDS / Fast-CDR | 用 ROS 2 自带的 | `/opt/ros/jazzy` |
| spdlog | **1.9.2 + 自带 fmt + PIC** | 系统 1.12 会与 fmt 9 冲突，见下 |

## 一、编 px4_msgs

```bash
mkdir -p ~/x650_ws/src && cd ~/x650_ws/src
git clone --depth 1 --branch v1.17.0 https://gh-proxy.com/https://github.com/PX4/px4_msgs.git
cd ~/x650_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release
```

约 11 分钟，产出 705 个消息接口。

## 二、编 MicroXRCEAgent（有坑，按这个顺序）

**不要用默认 superbuild**：它会从 github 直接克隆 Fast-DDS/spdlog，国内网络会
`fatal: 过早的文件结束符` 反复失败。改用 ROS 2 自带的依赖：

```bash
cd ~/tools
git clone --depth 1 --branch v2.4.3 https://gh-proxy.com/https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
# ① spdlog 1.9.2，必须「自带 fmt」且「-fPIC」
git clone --depth 1 --branch v1.9.2 https://gh-proxy.com/https://github.com/gabime/spdlog.git
cd spdlog && mkdir b && cd b
cmake .. -DCMAKE_BUILD_TYPE=Release -DSPDLOG_FMT_EXTERNAL=OFF -DSPDLOG_BUILD_SHARED=OFF \
         -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
         -DCMAKE_INSTALL_PREFIX=$HOME/tools/Micro-XRCE-DDS-Agent/build/temp_install/spdlog-1.9.2
cmake --build . -j20 --target install

# ② agent 本体：system Fast-DDS/Fast-CDR，logger 用刚编的 spdlog
cd ~/tools/Micro-XRCE-DDS-Agent
mkdir -p build2/temp_install && ln -sfn $PWD/build/temp_install/* build2/temp_install/ 2>/dev/null || true
cd build2 && source /opt/ros/jazzy/setup.bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DUAGENT_SUPERBUILD=OFF \
         -DUAGENT_USE_SYSTEM_FASTDDS=ON -DUAGENT_USE_SYSTEM_FASTCDR=ON \
         -DUAGENT_USE_SYSTEM_LOGGER=OFF -DCMAKE_PREFIX_PATH=/opt/ros/jazzy
cmake --build . -j20
# 产物：~/tools/Micro-XRCE-DDS-Agent/build2/MicroXRCEAgent
```

> `microxrcedds_client` 这个依赖在 superbuild 中断前已经装到 `build/temp_install/`，
> 所以 `build2` 要软链或复制它；否则 configure 会报 `microxrcedds_clientConfig.cmake` 找不到。

## 三、跑起来

```bash
# 终端 1：DDS agent
cd ~/tools/Micro-XRCE-DDS-Agent/build2
LD_LIBRARY_PATH=$PWD ./MicroXRCEAgent udp4 -p 8888

# 终端 2：SITL（不要用 timeout，否则会被中途杀掉）
cd ~/PX4-Autopilot && HEADLESS=1 PX4_GZ_WORLD=default make px4_sitl gz_x650

# 终端 3：验证链路（应看到 65 个 /fmu/* 话题）
source /opt/ros/jazzy/setup.bash && source ~/x650_ws/install/setup.bash
ros2 topic list | grep -c /fmu/

# 终端 3：飞
colcon build --packages-select x650_offboard
ros2 run x650_offboard offboard_hover --ros-args -p alt:=2.5 \
    -p 'waypoints:=2.0 0.0 2.5, 2.0 2.0 2.5, 0.0 0.0 2.5'
```

## 四个必踩的坑（都已修在代码里）

| # | 现象 | 根因 | 处理 |
|---|---|---|---|
| 1 | 节点收不到 `/fmu/out/*` 数据 | **话题名带版本后缀**：`/fmu/out/vehicle_status_v1`、`vehicle_local_position_v1` | 用 `ros2 topic list` 的真实名字 |
| 2 | 发布 QoS 不匹配 | PX4 发布端是 `BEST_EFFORT + TRANSIENT_LOCAL` | 订阅用同样组合 |
| 3 | 切不进 OFFBOARD | `MAV_CMD_DO_SET_MODE` 参数顺序错：`param1` 是 base mode（要 `1`=CUSTOM_MODE_ENABLED），`param2` 才是 `6`=OFFBOARD | 已改 |
| 4 | `waypoints` 参数报类型错误 | ROS 2 CLI 把 `[1,2,3]` 解析成 YAML 数组 | 传纯字符串：`'2.0 0.0 2.5, 2.0 2.0 2.5'` |

## 已验证结果（2026-11-01）

| 项 | 结果 |
|---|---|
| DDS 链路 | 65 个 `/fmu/*` 话题，`fmu/out` 数据正常 |
| 模式切换 | `nav_state=14`（OFFBOARD），`Armed by external command`，`Takeoff detected` |
| 定高悬停 | 目标 2.5 m，实测 2.48–2.51 m（波动 0.03 m） |
| 航点飞行 | (0,0) → (2,0) 8.7 s → (2,2) 10.4 s，x 最大 2.26 m、y 最大 1.85 m |
| 到位精度 | 末速度 < 0.03 m/s，3 个航点全部命中（0.5 m 判定半径） |

详见 [`../results/2026-11-01-offboard-flight.md`](../results/2026-11-01-offboard-flight.md)。
