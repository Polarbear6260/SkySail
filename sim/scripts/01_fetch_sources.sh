#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# 01 · 拉取 PX4 源码与 Gazebo 模型库
#
#   ./scripts/01_fetch_sources.sh                    # 默认 ~/PX4-Autopilot, v1.17.0，精简子模块
#   ./scripts/01_fetch_sources.sh --mirror           # 走 gh-proxy 镜像（本机实测快 5–10 倍）
#   ./scripts/01_fetch_sources.sh --full             # 连 NuttX 一起拉（要编飞控固件时才需要）
#   PX4_DIR=/opt/px4 ./scripts/01_fetch_sources.sh   # 自定义目录
#
# 本机实测速度（2026-10-30，同一时段）：
#   github.com 直连      ~17 KB/s   浅克隆要 1 小时
#   socks5 代理 7897     ~68 KB/s   约 16 分钟
#   gh-proxy.com         ~170 KB/s  约 6–8 分钟   ← 推荐
#
# 下载量（实测）：
#   PX4 浅克隆 ≈ 250–300 MB｜必要子模块 ≈ 160 MB（含 gz 模型库 60 MB）
#   精简模式合计 ≈ 420–460 MB；--full 再加 NuttX/tflite/zenoh 等约 600 MB
#
# 说明：
#  · v1.17.0 起 Tools/simulation/gz 指向 PX4/PX4-gazebo-models，模型/世界都在那边。
#  · 弱网下 git 会挂在半包上（"unexpected disconnect"），clone 不能续传，
#    所以这里用 lowSpeedLimit 主动失败 + 最多 5 次重试。
# ---------------------------------------------------------------------------
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_REF="${PX4_REF:-v1.17.0}"
MIRROR=0
MODE="minimal"

while [ $# -gt 0 ]; do
  case "$1" in
    --full)   MODE="full"; shift ;;
    --mirror) MIRROR=1; shift ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    *) echo "未知参数：$1" >&2; exit 2 ;;
  esac
done

GHPROXY="https://gh-proxy.com"
if [ "$MIRROR" -eq 1 ]; then
  PX4_GIT_URL="${PX4_GIT_URL:-$GHPROXY/https://github.com/PX4/PX4-Autopilot.git}"
  GZ_MODELS_URL="${GZ_MODELS_URL:-$GHPROXY/https://github.com/PX4/PX4-gazebo-models.git}"
  # 子模块 URL 是 github.com 绝对地址，用 insteadOf 一并改写到镜像
  REWRITE=(-c "url.$GHPROXY/https://github.com/.insteadOf=https://github.com/")
else
  PX4_GIT_URL="${PX4_GIT_URL:-https://github.com/PX4/PX4-Autopilot.git}"
  GZ_MODELS_URL="${GZ_MODELS_URL:-https://github.com/PX4/PX4-gazebo-models.git}"
  REWRITE=()
fi
GIT_OPTS=(-c http.lowSpeedLimit=20000 -c http.lowSpeedTime=45)

echo "== 目标：$PX4_DIR（$PX4_REF）｜镜像=$MIRROR｜子模块=$MODE =="

if [ -d "$PX4_DIR/.git" ]; then
  echo "  已存在仓库，切到 $PX4_REF"
  git -C "$PX4_DIR" "${GIT_OPTS[@]}" "${REWRITE[@]}" fetch --depth 1 origin \
    "refs/tags/$PX4_REF:refs/tags/$PX4_REF" || \
    git -C "$PX4_DIR" "${GIT_OPTS[@]}" "${REWRITE[@]}" fetch --depth 1 origin "$PX4_REF"
  git -C "$PX4_DIR" checkout -q "$PX4_REF"
else
  ok=0
  for attempt in 1 2 3 4 5; do
    echo "  浅克隆第 $attempt/5 次（约 65 MB）..."
    rm -rf "$PX4_DIR"
    if git "${GIT_OPTS[@]}" "${REWRITE[@]}" clone --depth 1 --single-branch \
         --branch "$PX4_REF" "$PX4_GIT_URL" "$PX4_DIR"; then ok=1; break; fi
    echo "  第 $attempt 次失败（弱网/超时），5 秒后重试" >&2
    sleep 5
  done
  [ "$ok" -eq 1 ] || { echo "克隆失败：加 --mirror 或用代理后重试" >&2; exit 1; }
fi

echo "== 子模块（模式：$MODE）=="
if [ "$MODE" = "full" ]; then
  for i in 1 2 3; do
    git -C "$PX4_DIR" "${GIT_OPTS[@]}" "${REWRITE[@]}" submodule update --init --recursive --depth 1 && break
    echo "  第 $i 次失败，重试..." >&2
    sleep 3
  done
else
  # 只拉 px4_sitl 编译真正需要的子模块（约 160 MB）。
  # 上游 .gitmodules 里共有 23 个，其中 tensorflow_lite_micro / zenoh-pico /
  # uavcan / 板卡 SDK 等在 SITL 默认配置下不参与编译，全拉要多下 300 MB+。
  SUBS=(
    src/modules/mavlink/mavlink
    src/drivers/gps/devices
    src/lib/events/libevents
    src/lib/heatshrink/heatshrink
    src/lib/crypto/monocypher
    src/lib/crypto/libtomcrypt
    src/lib/crypto/libtommath
    src/drivers/cyphal/libcanard
    src/drivers/cyphal/public_regulated_data_types
    src/drivers/cyphal/legacy_data_types
    src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client
    src/lib/cdrstream/cyclonedds
    src/lib/cdrstream/rosidl
    Tools/simulation/gz
  )
  echo "  拉取 ${#SUBS[@]} 个子模块（按 SITL 需要挑选，跳过 NuttX/tflite/zenoh 等）"
  for i in 1 2 3; do
    # 必须 --recursive：mavlink 的 pymavlink、uxrce 的 Micro-CDR 等都是嵌套子模块，
    # 少了它们编译会在 mavgen.py / Micro-CDR 头文件处失败
    git -C "$PX4_DIR" "${GIT_OPTS[@]}" "${REWRITE[@]}" submodule update --init --recursive --depth 1 "${SUBS[@]}" && break
    echo "  第 $i 次失败，重试..." >&2
    sleep 3
  done
fi

echo "== 检查 Gazebo 模型库（Tools/simulation/gz）=="
GZ_DIR="$PX4_DIR/Tools/simulation/gz"
if [ -f "$GZ_DIR/models/x500_base/model.sdf" ]; then
  echo "  ✔ 模型库已就位：$GZ_DIR"
else
  echo "  模型库为空，克隆 PX4-gazebo-models（约 60 MB）..."
  if [ -d "$GZ_DIR" ] && [ -n "$(ls -A "$GZ_DIR" 2>/dev/null)" ]; then
    echo "  错误：$GZ_DIR 非空但不是模型库，请检查后删除该目录重试" >&2
    exit 1
  fi
  ok=0
  for attempt in 1 2 3; do
    rm -rf "$GZ_DIR"
    if git "${GIT_OPTS[@]}" "${REWRITE[@]}" clone --depth 1 "$GZ_MODELS_URL" "$GZ_DIR"; then
      ok=1; break
    fi
    echo "  第 $attempt 次失败，重试..." >&2; sleep 5
  done
  [ "$ok" -eq 1 ] || { echo "模型库克隆失败" >&2; exit 1; }
  echo "  ✔ 已克隆到 $GZ_DIR"
fi

echo
echo "== 结果 =="
printf '  PX4      %s\n' "$(git -C "$PX4_DIR" describe --tags --always 2>/dev/null)"
printf '  体积     %s\n' "$(du -sh "$PX4_DIR" 2>/dev/null | cut -f1)"
printf '  gz 模型  %s 个\n' "$(ls "$GZ_DIR/models" | wc -l)"
printf '  世界     %s 个\n' "$(ls "$GZ_DIR/worlds"/*.sdf 2>/dev/null | wc -l)"
echo
echo "下一步：./scripts/02_build_sitl.sh"
