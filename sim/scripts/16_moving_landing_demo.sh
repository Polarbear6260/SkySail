#!/usr/bin/env bash
# 16 · 移动平台着陆 demo（一键）
#
# 流程：清场 → DDS agent → SITL（飞机生成在固定 dock 上）→ 设参数 → 起 GUI（可选）
#      → 录制（可选）→ 船直线航行 → 无人机起飞 10 m → 追船 → 下降 → 触板 → 立即收尾
#
# 用法：
#   ./16_moving_landing_demo.sh                 # 带 GUI + 录制（默认）
#   ./16_moving_landing_demo.sh --no-gui        # 无头（服务器上跑）
#   ./16_moving_landing_demo.sh --no-record     # 只看不录
#   ./16_moving_landing_demo.sh --dashboard     # 额外开实时调试面板
#   ./16_moving_landing_demo.sh --ship-speed 1.0
#
# 环境变量（都有默认值）：
#   PX4_DIR     PX4-Autopilot 目录            （默认 ~/PX4-Autopilot）
#   AGENT_BIN   MicroXRCEAgent 可执行文件     （默认 ~/tools/.../build2/MicroXRCEAgent）
#   SKYSAIL     SkySail 仓库（提供 LQR 代码）（默认 ~/planning_ws/src/SkySail）
#
# 成功判据：真值（Gazebo）确认机体落在移动甲板上一次即可 —— 输出 "TOUCHDOWN 成功"。
set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
AGENT_BIN="${AGENT_BIN:-$HOME/tools/Micro-XRCE-DDS-Agent/build2/MicroXRCEAgent}"
SKYSAIL="${SKYSAIL:-$HOME/planning_ws/src/SkySail}"
WX650="${WX650:-$HOME/x650_ws}"

GUI=1; RECORD=1; DASHBOARD=0; SHIP_SPEED=1.5; SHIP_START_X=-40; SHIP_DELAY=12; TIMEOUT=150
DOCK_X=0; DOCK_Y=16; DOCK_Z=1.02          # 起飞台台面（与 world 中的 dock 一致）
CRUISE_ALT=10

while [ $# -gt 0 ]; do
  case "$1" in
    --no-gui) GUI=0 ;;
    --no-record) RECORD=0 ;;
    --dashboard) DASHBOARD=1 ;;
    --ship-speed) SHIP_SPEED="$2"; shift ;;
    --ship-start-x) SHIP_START_X="$2"; shift ;;
    --timeout) TIMEOUT="$2"; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
  shift
done

TS=$(date +%Y%m%d-%H%M%S)
LOG="$WS/logs/run-$TS.log"
MP4="$WS/logs/x650-moving-$TS.mp4"
mkdir -p "$WS/logs"
say() { echo "[$(date +%H:%M:%S)] $*"; }

say "== 0/7 检查环境 =="
[ -d "$PX4_DIR" ] || { echo "找不到 PX4: $PX4_DIR" >&2; exit 1; }
[ -x "$AGENT_BIN" ] || { echo "找不到 MicroXRCEAgent: $AGENT_BIN" >&2; exit 1; }
[ -f "$SKYSAIL/UAV_Control/__init__.py" ] || { echo "找不到 SkySail 代码: $SKYSAIL" >&2; exit 1; }
command -v gz >/dev/null || { echo "找不到 gz（Gazebo Harmonic）" >&2; exit 1; }
say "   OK（PX4=$PX4_DIR）"

say "== 1/7 清理残留 =="
PIDS=$(ps -eo pid,args | awk -v me=$$ -v dad=$PPID \
  '/10_move_platform|12_log_pose|MicroXRCEAgent|bin\/px4|x650_offboard\/lib/ && !/awk/ && !/grep/ {print $1}' \
  | grep -vxE "($$|$PPID)" || true)
[ -n "$PIDS" ] && kill -9 $PIDS 2>/dev/null
for p in $(pgrep -x ruby 2>/dev/null); do
  tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "gz sim" && kill -9 "$p"
done
sleep 2; rm -f /tmp/px4_lock-0

say "== 2/7 启动 DDS agent =="
( cd "$(dirname "$AGENT_BIN")" && setsid nohup env LD_LIBRARY_PATH="$PWD" \
    "$AGENT_BIN" udp4 -p 8888 > "$WS/logs/agent-$TS.log" 2>&1 & )
sleep 2
pgrep -x MicroXRCEAgent >/dev/null || { echo "agent 启动失败" >&2; exit 1; }

say "== 3/7 启动 SITL（飞机在 dock 上，$DOCK_X,$DOCK_Y,$DOCK_Z）=="
cp "$WS/gz_extras/worlds/x650_sea.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/" 2>/dev/null
cp "$WS/gz_extras/airframes/22000_gz_x650" "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/" 2>/dev/null
( cd "$PX4_DIR" && HEADLESS=1 PX4_GZ_WORLD=x650_sea \
    PX4_GZ_MODEL_POSE="$DOCK_X,$DOCK_Y,$DOCK_Z,0,0,0" \
    setsid nohup timeout $((TIMEOUT + 120)) make px4_sitl gz_x650 \
    > "$WS/logs/sitl-$TS.log" 2>&1 < <(sleep $((TIMEOUT + 120))) & )
for i in $(seq 1 90); do
  grep -qa "gz_bridge.*model: x650" "$WS/logs/sitl-$TS.log" 2>/dev/null && break
  sleep 1
done
grep -qa "gz_bridge.*model: x650" "$WS/logs/sitl-$TS.log" || { echo "SITL 未就绪" >&2; exit 1; }
say "   SITL 就绪"

say "== 4/7 设置仿真参数 =="
( cd "$PX4_DIR" && [ -d .venv-px4 ] && source .venv-px4/bin/activate
  for kv in "NAV_DLL_ACT 0" "SIM_BAT_ENABLE 0" "COM_LOW_BAT_ACT 0" \
            "COM_OF_LOSS_T 5.0" "COM_OBL_RC_ACT 0"; do
    printf 'param set %s\n' "$kv" | timeout 15 python Tools/mavlink_shell.py udpin:127.0.0.1:14540 >/dev/null 2>&1
  done )

if [ "$GUI" = "1" ]; then
  say "== 5/7 起 GUI =="
  ( cd "$WS" && setsid nohup gz sim -g --gui-config "$HOME/.gz/sim/8/gui-demo.config" \
      > "$WS/logs/gui-$TS.log" 2>&1 & )
  for i in $(seq 1 60); do
    xwininfo -root -tree 2>/dev/null | grep -q '"Gazebo Sim".*gz-sim-gui' && break
    sleep 0.5
  done
  if [ "$RECORD" = "1" ]; then
    FF=$(/home/green/planning_ws/.venv/bin/python -c "import imageio_ffmpeg;print(imageio_ffmpeg.get_ffmpeg_exe())" 2>/dev/null)
    GEO=$(xwininfo -root -tree 2>/dev/null | grep '"Gazebo Sim"' | grep -v mutter | head -1)
    SZ=$(echo "$GEO" | grep -oE '[0-9]+x[0-9]+\+[0-9]+\+[0-9]+' | head -1)
    AB=$(echo "$GEO" | grep -oE '\+[0-9]+\+[0-9]+' | tail -1 | sed 's/^+//;s/+/ /')
    GW=$(echo "$SZ" | cut -dx -f1); GH=$(echo "$SZ" | cut -dx -f2 | cut -d+ -f1)
    GX=$(echo "$AB" | cut -d' ' -f1); GY=$(echo "$AB" | cut -d' ' -f2)
    "$FF" -y -loglevel error -f x11grab -video_size "${GW}x${GH}" -framerate 15 \
      -i ":1+${GX},${GY}" -t $((TIMEOUT + 30)) -vf scale=1280:-2 -pix_fmt yuv420p "$MP4" &
    REC_PID=$!
    say "   录制中 → $MP4"
  fi
fi

say "== 6/7 起船 + 真值日志 =="
set +u; source /opt/ros/jazzy/setup.bash; source "$WX650/install/setup.bash"; set -u
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
setsid nohup python3 "$WS/scripts/12_log_pose.py" --world x650_sea --duration $((TIMEOUT + 20)) \
  --csv "$WS/logs/pose-$TS.csv" > "$WS/logs/pose-$TS.log" 2>&1 &
SHIP_ARGS=()
[ -n "${SHIP_SEED:-}" ] && SHIP_ARGS+=(--seed "$SHIP_SEED")
[ "${SHIP_FIXED:-0}" = "1" ] && SHIP_ARGS+=(--fixed --start-x "$SHIP_START_X" --speed "$SHIP_SPEED")
setsid nohup python3 "$WS/scripts/10_move_platform.py" --world x650_sea --delay "$SHIP_DELAY" \
  "${SHIP_ARGS[@]}" --duration $((TIMEOUT + 40)) \
  --csv "$WS/logs/ship-$TS.csv" > "$WS/logs/ship-$TS.log" 2>&1 &
sleep 3

if [ "$DASHBOARD" = "1" ]; then
  say "   起调试面板"
  setsid nohup python3 "$WS/scripts/17_debug_dashboard.py" > "$WS/logs/dashboard-$TS.log" 2>&1 &
  DASH_PID=$!
  sleep 2
fi

say "== 7/7 起飞并追船（触板即停）=="
timeout "$TIMEOUT" ros2 run x650_offboard moving_landing --ros-args \
  -p gz_world:=x650_sea \
  -p origin_enu:="$DOCK_X,$DOCK_Y,$DOCK_Z" \
  -p cruise_alt:="${CRUISE_ALT}.0" \
  -p skysail_path:="$SKYSAIL" > "$LOG" 2>&1

# —— 收尾：立刻停掉一切 ——
PIDS=$(ps -eo pid,args | awk \
  '/10_move_platform|12_log_pose|MicroXRCEAgent|bin\/px4|x650_offboard\/lib/ && !/awk/ && !/grep/ {print $1}' \
  | grep -vxE "($$|$PPID)" || true)
[ -n "$PIDS" ] && kill -9 $PIDS 2>/dev/null
for p in $(pgrep -x ruby 2>/dev/null); do
  tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "gz sim" && kill -9 "$p"
done
[ -n "${DASH_PID:-}" ] && kill -9 $DASH_PID 2>/dev/null
if [ -n "${REC_PID:-}" ]; then
  kill -TERM "$REC_PID" 2>/dev/null
  for i in $(seq 1 20); do kill -0 "$REC_PID" 2>/dev/null || break; sleep 1; done
  kill -9 "$REC_PID" 2>/dev/null
fi

echo
echo "════════════════════ 结果 ════════════════════"
grep -aE "TOUCHDOWN|接触|贴板" "$LOG" | tail -3
echo "  日志: $LOG"
echo "  真值: $WS/logs/pose-$TS.csv"
[ -n "${REC_PID:-}" ] && echo "  视频: $MP4 ($(du -h "$MP4" 2>/dev/null | cut -f1))"
echo "══════════════════════════════════════════════"
