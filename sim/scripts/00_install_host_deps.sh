#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# 00 · 宿主机依赖安装（需要 root）
#
#   !!! 这一步无法在 DSH 沙箱里跑（沙箱 NoNewPrivs=1、无 root），
#       请在你自己的终端执行：
#
#           sudo ./scripts/00_install_host_deps.sh
#           sudo ./scripts/00_install_host_deps.sh --with-ros   # 顺带装 ROS 2 Jazzy
#
# 目标环境：Ubuntu 24.04 (noble) + Gazebo Harmonic (gz-sim8) + PX4 v1.17
# 包清单对齐上游 PX4 Tools/setup/ubuntu.sh（v1.17.0）：
#   OSRF 源 → gz-harmonic libunwind-dev cppzmq-dev
# ---------------------------------------------------------------------------
set -euo pipefail

WITH_ROS=0
for arg in "$@"; do
  case "$arg" in
    --with-ros) WITH_ROS=1 ;;
    -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
    *) echo "未知参数：$arg"; exit 2 ;;
  esac
done

if [ "$(id -u)" -ne 0 ]; then
  echo "错误：需要 root。请执行 sudo $0 $*" >&2
  exit 1
fi

. /etc/os-release
echo "== 系统：$PRETTY_NAME =="
if [ "${VERSION_ID:-}" != "24.04" ]; then
  echo "警告：本脚本按 Ubuntu 24.04 编写，当前 ${VERSION_ID:-未知}。按回车继续，Ctrl-C 退出。"
  read -r _
fi

export DEBIAN_FRONTEND=noninteractive

echo "== [1/5] 基础编译与 Python 工具链 =="
apt-get update -q
apt-get install -y -q --no-install-recommends \
  build-essential cmake ninja-build git curl wget ca-certificates gnupg \
  lsb-release pkg-config unzip \
  python3-dev python3-venv python3-pip python3-setuptools \
  libeigen3-dev protobuf-compiler libxml2-utils

echo "== [2/5] OSRF 源与 Gazebo Harmonic =="
install -d -m 0755 /usr/share/keyrings
if [ ! -s /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
  wget -q https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  chmod 0644 /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/gazebo-stable.list
apt-get update -q

# cppzmq-dev 是 24.04 上 gz-harmonic 的已知依赖缺口，上游 ubuntu.sh 也显式装它
apt-get install -y -q --no-install-recommends \
  gz-harmonic libunwind-dev cppzmq-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav libgstreamer-plugins-base1.0-dev \
  libopencv-dev libimage-exiftool-perl

if [ "$WITH_ROS" -eq 1 ]; then
  echo "== [3/5] ROS 2 Jazzy（可选，约 2 GB）=="
  install -d -m 0755 /usr/share/keyrings
  # 本机 raw.githubusercontent.com 只解析到 IPv6 且超时，key 走 gh-proxy 兜底
  if [ ! -s /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    curl -sSL "https://gh-proxy.com/https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" \
      -o /usr/share/keyrings/ros-archive-keyring.gpg || \
    curl -sSL "http://packages.ros.org/ros.key" \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    chmod 0644 /usr/share/keyrings/ros-archive-keyring.gpg
  fi
  # 走清华镜像，packages.ros.org 在国内经常卡死在最后几个包
  ROS2_MIRROR="${ROS2_APT_MIRROR:-http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu}"
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] $ROS2_MIRROR $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list
  apt-get update -q
  # ros_gz_bridge：Gazebo ↔ ROS 2 桥；ros-dev-tools 提供 colcon
  # 注意：px4_msgs 没有 Jazzy 的 apt 包，必须从源码编（见 docs/03 §ROS 2）
  #       ros-jazzy-desktop 已含 rviz2，便于有 GPU 的机器看轨迹
  apt-get install -y -q --no-install-recommends \
    ros-jazzy-ros-gz ros-jazzy-rmw-cyclonedds-cpp ros-dev-tools \
    ros-jazzy-desktop
else
  echo "== [3/5] 跳过 ROS 2（需要时加 --with-ros）=="
fi

echo "== [4/5] 用户组（串口访问飞控）=="
if id -nG "${SUDO_USER:-}" 2>/dev/null | grep -qw dialout; then
  echo "  ${SUDO_USER} 已在 dialout 组"
else
  [ -n "${SUDO_USER:-}" ] && usermod -aG dialout "${SUDO_USER}" && \
    echo "  已把 ${SUDO_USER} 加入 dialout（重新登录生效）"
fi

echo "== [5/5] 验证 =="
fail=0
for c in gz cmake ninja g++ git; do
  if command -v "$c" >/dev/null 2>&1; then
    printf '  ✔ %-6s %s\n' "$c" "$(command -v "$c")"
  else
    printf '  ✘ %-6s 未安装\n' "$c"; fail=1
  fi
done
if command -v gz >/dev/null 2>&1; then
  echo -n "  gz 版本："; gz sim --versions 2>/dev/null | head -1 || echo "(无法查询)"
fi
if [ "$WITH_ROS" -eq 1 ] && [ -f /opt/ros/jazzy/setup.bash ]; then
  echo "  ✔ ROS 2 Jazzy：/opt/ros/jazzy/setup.bash"
fi

if [ "$fail" -ne 0 ]; then
  echo; echo "有依赖未装好，见上面的 ✘。" >&2; exit 1
fi

cat <<'EOF'

完成。下一步（普通用户，不要 sudo）：
  ./scripts/01_fetch_sources.sh     # 拉 PX4 v1.17.0 + gz 模型
  ./scripts/02_build_sitl.sh        # 编译 px4_sitl
  ./scripts/03_install_gz_assets.sh # 安装 X650 模型 / 世界 / airframe
  ./scripts/04_run_sitl.sh --smoke  # 无头冒烟测试
EOF
