#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# 04 · 运行 PX4 SITL + Gazebo（X650）
#
#   ./scripts/04_run_sitl.sh                    # 无头 + 避障世界 + 相机
#   ./scripts/04_run_sitl.sh --model x650       # 不带相机的裸机
#   ./scripts/04_run_sitl.sh --world default    # 换回上游默认世界
#   ./scripts/04_run_sitl.sh --gui              # 开 Gazebo GUI（需要桌面/GPU）
#   ./scripts/04_run_sitl.sh --smoke            # 跑 120 s 后自动退出并判定
#
# 无头模式下 QGroundControl 仍可在宿主机直连（UDP 14550），无需 GUI 转发。
# ---------------------------------------------------------------------------
set -euo pipefail

kill_gz_leftovers() {
  # gz 的 CLI 是 ruby 进程，pkill -x gz 抓不到；必须按 cmdline 匹配
  for p in $(pgrep -x ruby 2>/dev/null); do
    if tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q "gz sim"; then
      kill -9 "$p" 2>/dev/null || true
    fi
  done
  pkill -x px4 2>/dev/null || true
}

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

MODEL="x650_depth"
WORLD="x650_obstacles"
HEADLESS=1
SMOKE=0
TIMEOUT=120

while [ $# -gt 0 ]; do
  case "$1" in
    --model)   MODEL="$2"; shift 2 ;;
    --world)   WORLD="$2"; shift 2 ;;
    --gui)     HEADLESS=0; shift ;;
    --smoke)   SMOKE=1; shift ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) echo "未知参数：$1" >&2; exit 2 ;;
  esac
done

[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] || {
  echo "px4_sitl 未编译，先跑 ./scripts/02_build_sitl.sh" >&2; exit 1; }
[ -f "$PX4_DIR/Tools/simulation/gz/models/$MODEL/model.sdf" ] || {
  echo "模型 $MODEL 未安装，先跑 ./scripts/03_install_gz_assets.sh" >&2; exit 1; }
AF_FILE="$(ls "$PX4_DIR"/ROMFS/px4fmu_common/init.d-posix/airframes/*_gz_"$MODEL" 2>/dev/null | head -1 || true)"
[ -n "$AF_FILE" ] || {
  echo "airframe gz_$MODEL 不存在，先跑 ./scripts/03_install_gz_assets.sh 并重编" >&2; exit 1; }

mkdir -p "$ROOT/logs"
LOG="$ROOT/logs/sitl-$(date +%Y%m%d-%H%M%S).log"

# 清掉上一次残留的实例（PX4 用 instance 锁，残留会让新实例直接退出）
kill_gz_leftovers && sleep 1

echo "模型 $MODEL ｜ 世界 $WORLD ｜ 无头=$HEADLESS ｜ 日志 $LOG"
cd "$PX4_DIR"

export PX4_GZ_WORLD="$WORLD"
if [ "$HEADLESS" -eq 1 ]; then export HEADLESS=1; else unset HEADLESS; fi

if [ "$SMOKE" -eq 1 ]; then
  # 无头冒烟：后台起仿真，等 Gazebo 就绪后用 MAVLink shell 把 NAV_DLL_ACT 置 0，
  # 再等 PX4 报 "Ready for takeoff"。
  #
  # 为什么需要这一步：airframe 沿用上游的 NAV_DLL_ACT=2，PX4 的
  # rcAndDataLinkCheck 会因此要求“必须有一条 GCS 数据链”才允许解锁；
  # 无头跑没有 QGC，预检会一直 FAILED（真机/带 QGC 时不需要改）。
  # stdin 用一个长期打开的管道：直接接 /dev/null 会让 PX4 shell 读到 EOF，
  # 每毫秒刷一次 "pxh>" 提示符，日志能涨到 1 GB+
  setsid nohup timeout "$TIMEOUT" make px4_sitl gz_"$MODEL" > "$LOG" 2>&1 \
    < <(sleep "$TIMEOUT") &
  SIM_PID=$!

  for _ in $(seq 1 40); do
    grep -qa "Gazebo world is ready" "$LOG" 2>/dev/null && break
    sleep 1
  done

  VENV_PY="$PX4_DIR/.venv-px4/bin/python"
  SHELL_TOOL="$PX4_DIR/Tools/mavlink_shell.py"
  if [ -x "$VENV_PY" ] && [ -f "$SHELL_TOOL" ]; then
    for _ in $(seq 1 15); do
      grep -qa "mavlink.*14580\|Startup script returned" "$LOG" 2>/dev/null && break
      sleep 1
    done
    printf 'param set NAV_DLL_ACT 0\n' | timeout 15 "$VENV_PY" "$SHELL_TOOL" \
      udpin:127.0.0.1:14540 >/dev/null 2>&1 || echo "  ! MAVLink shell 设参失败（可手动连 QGC）"
  else
    echo "  ! 缺 pymavlink 或 mavlink_shell.py，跳过 NAV_DLL_ACT 覆盖"
  fi

  for _ in $(seq 1 30); do
    grep -qa "Ready for takeoff" "$LOG" 2>/dev/null && break
    sleep 1
  done

  # setsid 起的进程组，负号 PID 杀整组，避免 px4 / gz 残留
  kill -INT -"$SIM_PID" 2>/dev/null || kill -INT "$SIM_PID" 2>/dev/null || true
  sleep 2
  kill -TERM -"$SIM_PID" 2>/dev/null || true
  wait "$SIM_PID" 2>/dev/null || true
  pkill -f "px4_sitl_default/bin/p[x]4" 2>/dev/null || true

  echo
  echo "== 冒烟判定 =="
  ok=1
  if grep -qa "Ready for takeoff" "$LOG"; then
    echo "  ✔ PX4 预检通过（Ready for takeoff）"
  else
    echo "  ✘ 未见 'Ready for takeoff'，见 $LOG"; ok=0
  fi
  if grep -qa "Gazebo world is ready" "$LOG"; then
    echo "  ✔ Gazebo 世界加载并 spawn 模型"
  else
    echo "  ✘ Gazebo 未就绪"; ok=0
  fi
  if grep -qa "gz_bridge.*model:" "$LOG"; then
    echo "  ✔ gz_bridge 已连上模型"
  else
    echo "  ✘ gz_bridge 未连上"; ok=0
  fi
  if grep -qa "Strong magnetic interference" "$LOG"; then
    echo "  ✘ 磁力计单位/地理原点有问题（检查 world 的 spherical_coordinates）"; ok=0
  fi
  [ "$ok" -eq 1 ] && { echo "结果：通过"; exit 0; } || { echo "结果：未通过"; exit 1; }
else
  make px4_sitl gz_"$MODEL" 2>&1 | tee "$LOG"
fi
