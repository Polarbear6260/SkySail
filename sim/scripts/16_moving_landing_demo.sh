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

GUI=1; RECORD=1; DASHBOARD=0; PERCEPTION=""; PERCEPTION_SEED=""; GPS_COARSE=""; VISION=0; RENDER_ENGINE="${RENDER_ENGINE:-}" SHIP_SPEED="${SHIP_SPEED:-1.5}"; SHIP_START_X="${SHIP_START_X:--40}"; SHIP_DELAY="${SHIP_DELAY:-12}"; TIMEOUT=150
DOCK_X=0; DOCK_Y=16; DOCK_Z=1.02          # 起飞台台面（与 world 中的 dock 一致）
CRUISE_ALT=10

while [ $# -gt 0 ]; do
  case "$1" in
    --no-gui) GUI=0 ;;
    --no-record) RECORD=0 ;;
    --dashboard) DASHBOARD=1 ;;
    --perception) PERCEPTION="$2"; shift ;;   # "延迟s,位置噪声m,丢帧概率"
    --perception-seed) PERCEPTION_SEED="$2"; shift ;;
    --gps-coarse) GPS_COARSE="$2"; shift ;;   # "噪声m,跳变概率"
    --vision) VISION=1; RENDER_ENGINE="ogre" ;;  # 用 ArUco 视觉当精定位
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
  '/10_move_platform|12_log_pose|18_perception|19_gps_source|21_camrig|22_aruco_detector|23_vision_eval|MicroXRCEAgent|bin\/px4|x650_offboard\/lib/ && !/awk/ && !/grep/ {print $1}' \
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
( cd "$PX4_DIR" && HEADLESS=1 PX4_GZ_WORLD=x650_sea PX4_GZ_SIM_RENDER_ENGINE="$RENDER_ENGINE" \
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
  GUI_CFG="$WS/gz_extras/gui-min.config"
  [ -f "$GUI_CFG" ] || GUI_CFG="$HOME/.gz/sim/8/gui-demo.config"
  cp "$GUI_CFG" "$HOME/.gz/sim/8/gui-active.config"
  ( cd "$WS" && setsid nohup gz sim -g --gui-config "$HOME/.gz/sim/8/gui-active.config" \
      > "$WS/logs/gui-$TS.log" 2>&1 & )
  for i in $(seq 1 60); do
    xwininfo -root -tree 2>/dev/null | grep -q '"Gazebo Sim".*gz-sim-gui' && break
    sleep 0.5
  done
  /home/green/planning_ws/.venv/bin/python - <<'PYX' 2>/dev/null
from Xlib import display
d = display.Display(); root = d.screen().root; tgt=[None]
def walk(w):
    try:
        for c in w.query_tree().children:
            try:
                if (c.get_wm_name() or "").strip() == "Gazebo Sim": tgt[0]=c
            except Exception: pass
            walk(c)
    except Exception: pass
walk(root)
if tgt[0]:
    tgt[0].configure(width=1280, height=720); d.sync()
PYX
  sleep 1
  if [ "$RECORD" = "1" ]; then
    FF=$(/home/green/planning_ws/.venv/bin/python -c "import imageio_ffmpeg;print(imageio_ffmpeg.get_ffmpeg_exe())" 2>/dev/null)
    GEO=$(xwininfo -root -tree 2>/dev/null | grep '"Gazebo Sim"' | grep -v mutter | head -1)
    SZ=$(echo "$GEO" | grep -oE '[0-9]+x[0-9]+\+[0-9]+\+[0-9]+' | head -1)
    AB=$(echo "$GEO" | grep -oE '\+[0-9]+\+[0-9]+' | tail -1 | sed 's/^+//;s/+/ /')
    GW=$(echo "$SZ" | cut -dx -f1); GH=$(echo "$SZ" | cut -dx -f2 | cut -d+ -f1)
    GX=$(echo "$AB" | cut -d' ' -f1); GY=$(echo "$AB" | cut -d' ' -f2)
    "$FF" -y -loglevel error -f x11grab -video_size "${GW}x${GH}" -framerate 25 \
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

PAD_POSE_TOPIC="/ship/pad_pose"; PAD_TWIST_TOPIC="/ship/pad_twist"; GPS_TOPIC=""
if [ "$VISION" = "1" ]; then
  say "   启用 ArUco 视觉（相机跟随 + 检测）"
  setsid nohup python3 "$WS/scripts/22_aruco_detector.py" --world x650_sea \
    > "$WS/logs/aruco-$TS.log" 2>&1 &
  ARUCO_PID=$!
  setsid nohup python3 "$WS/scripts/23_vision_eval.py" --duration $((TIMEOUT + 20)) \
    --csv "$WS/logs/vision_eval-$TS.csv" > "$WS/logs/viseval-$TS.log" 2>&1 &
  VISEVAL_PID=$!
  PAD_POSE_TOPIC="/ship/pad_pose_vis"; PAD_TWIST_TOPIC="/ship/pad_twist_vis"
  sleep 2
fi
if [ -n "$GPS_COARSE" ]; then
  IFS=',' read -r G_NOISE G_JUMP G_RATE <<< "$GPS_COARSE"
  say "   GPS 级粗定位：σ=${G_NOISE:-1.5}m 跳变 ${G_JUMP:-0.02}"
  setsid nohup python3 "$WS/scripts/19_gps_source.py" --rate "${G_RATE:-1}" \
    --noise "${G_NOISE:-1.5}" --jump-p "${G_JUMP:-0.02}" \
    > "$WS/logs/gps-$TS.log" 2>&1 &
  GPS_PID=$!
  GPS_TOPIC="/ship/pad_pose_gps"
  sleep 1
fi
if [ -n "$PERCEPTION" ]; then
  IFS=',' read -r P_DELAY P_NOISE P_DROP <<< "$PERCEPTION"
  say "   感知劣化：延迟 ${P_DELAY}s 噪声 ${P_NOISE}m 丢帧 ${P_DROP}"
  setsid nohup python3 "$WS/scripts/18_perception_emulator.py" \
    --delay "${P_DELAY:-0}" --noise "${P_NOISE:-0}" --dropout "${P_DROP:-0}" \
    ${PERCEPTION_SEED:+--seed "$PERCEPTION_SEED"} \
    > "$WS/logs/perception-$TS.log" 2>&1 &
  PERC_PID=$!
  PAD_POSE_TOPIC="/ship/pad_pose_d"; PAD_TWIST_TOPIC="/ship/pad_twist_d"
  sleep 1
fi

say "== 7/7 起飞并追船（触板即停）=="
timeout "$TIMEOUT" ros2 run x650_offboard moving_landing --ros-args \
  -p gz_world:=x650_sea \
  -p origin_enu:="$DOCK_X,$DOCK_Y,$DOCK_Z" \
  -p cruise_alt:="${CRUISE_ALT}.0" \
  -p skysail_path:="$SKYSAIL" \
  -p pad_pose_topic:="$PAD_POSE_TOPIC" \
  -p pad_twist_topic:="$PAD_TWIST_TOPIC" \
  ${GPS_TOPIC:+-p gps_pose_topic:="$GPS_TOPIC"} > "$LOG" 2>&1

# —— 收尾：立刻停掉一切 ——
PIDS=$(ps -eo pid,args | awk \
  '/10_move_platform|12_log_pose|18_perception|19_gps_source|21_camrig|22_aruco_detector|23_vision_eval|MicroXRCEAgent|bin\/px4|x650_offboard\/lib/ && !/awk/ && !/grep/ {print $1}' \
  | grep -vxE "($$|$PPID)" || true)
[ -n "$PIDS" ] && kill -9 $PIDS 2>/dev/null
for p in $(pgrep -x ruby 2>/dev/null); do
  tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "gz sim" && kill -9 "$p"
done
[ -n "${DASH_PID:-}" ] && kill -9 $DASH_PID 2>/dev/null
[ -n "${PERC_PID:-}" ] && kill -9 $PERC_PID 2>/dev/null
[ -n "${GPS_PID:-}" ] && kill -9 $GPS_PID 2>/dev/null
[ -n "${CAMRIG_PID:-}" ] && kill -9 $CAMRIG_PID 2>/dev/null
[ -n "${ARUCO_PID:-}" ] && kill -9 $ARUCO_PID 2>/dev/null
[ -n "${VISEVAL_PID:-}" ] && kill -9 $VISEVAL_PID 2>/dev/null
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
