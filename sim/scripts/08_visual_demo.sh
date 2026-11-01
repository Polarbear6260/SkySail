#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# 08 · 一条命令出可视化演示（Gazebo GUI + 自动录屏）
#
#   ./scripts/08_visual_demo.sh                  # 落点 demo（8 m 进场 → 着陆），录 MP4
#   ./scripts/08_visual_demo.sh --hover          # 悬停 + 航点
#   LAND_X=4 ALT=2.5 ./scripts/08_visual_demo.sh # 换落点/高度
#   REC=0 ./scripts/08_visual_demo.sh            # 只看不录
#
# 流程：
#   1. 清残留（gz 的 CLI 是 ruby 进程，pkill -x gz 抓不到，必须按 cmdline 匹配）
#   2. 起 MicroXRCEAgent（uXRCE-DDS）
#   3. 起 SITL（server only）→ 等模型 spawn
#   4. 起 GUI 客户端附着到同一个 server，配固定机位；提到最前
#   5. 只录 Gazebo 窗口那块区域（不录整个桌面）
#   6. 跑 x650_offboard 的 demo，落点误差/轨迹打印出来
# ---------------------------------------------------------------------------
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WS="${WS:-$HOME/x650_ws}"
PY="${PY:-$HOME/planning_ws/.venv/bin/python}"
AGENT_DIR="${AGENT_DIR:-$HOME/tools/Micro-XRCE-DDS-Agent/build2}"
MODEL="${MODEL:-x650}"
WORLD="${WORLD:-default}"
ALT="${ALT:-2.5}"
LAND_X="${LAND_X:-8.0}"
REC="${REC:-1}"
MODE="landing"
[ "${1:-}" = "--hover" ] && MODE="hover"

mkdir -p "$ROOT/logs"
TS=$(date +%Y%m%d-%H%M%S)
LOG="$ROOT/logs/visual-$TS.log"
MP4="$ROOT/logs/x650-${MODE}-$TS.mp4"
GUI_CFG="$HOME/.gz/sim/8/gui-demo.config"

die() { echo "错误：$*" >&2; exit 1; }
step() { echo "[$(date +%H:%M:%S)] $*"; }

kill_leftovers() {
  pgrep -x px4 2>/dev/null | xargs -r kill -9 2>/dev/null || true
  pgrep -x MicroXRCEAgent 2>/dev/null | xargs -r kill -9 2>/dev/null || true
  for p in $(pgrep -x ruby 2>/dev/null || true); do
    if tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q "gz sim"; then
      kill -9 "$p" 2>/dev/null || true
    fi
  done
  sleep 2
  rm -f /tmp/px4_lock-0 /tmp/px4-sock-0
}

make_gui_config() {
  local base="$HOME/.gz/sim/8/gui.config"
  [ -f "$base" ] || return 0                      # 没装过 GUI 就用默认配置
  cp -f "$base" "$GUI_CFG"
  "$PY" - "$GUI_CFG" <<'PY' 2>/dev/null || true
import re, sys
p = sys.argv[1]
s = open(p).read()
pose = "<camera_pose>2 -6 2.6 0 0.35 1.5708</camera_pose>"
if "<camera_pose>" in s:
    s = re.sub(r"<camera_pose>[^<]*</camera_pose>", pose, s, count=1)
else:
    s = s.replace('<plugin filename="MinimalScene" name="3D View">',
                  '<plugin filename="MinimalScene" name="3D View">\n      ' + pose, 1)
open(p, "w").write(s)
PY
}

raise_gui() {
  "$PY" - <<'PY' 2>/dev/null || true
from Xlib import display, X
from Xlib.protocol import event as xevent
d = display.Display()
root = d.screen().root
target = None
def walk(w):
    global target
    try:
        for c in w.query_tree().children:
            try:
                if (c.get_wm_name() or "").strip() == "Gazebo Sim":
                    target = c
            except Exception:
                pass
            walk(c)
    except Exception:
        pass
walk(root)
if target:
    atom = d.intern_atom("_NET_ACTIVE_WINDOW")
    e = xevent.ClientMessage(window=target, client_type=atom, data=(32, [2, X.CurrentTime, 0, 0, 0]))
    root.send_event(e, event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask)
    d.sync()
    target.configure(stack_mode=X.Above)
    d.sync()
    print("window raised")
PY
}

step '== 0/6 清理残留 =='
kill_leftovers
[ -x "$AGENT_DIR/MicroXRCEAgent" ] || die "找不到 $AGENT_DIR/MicroXRCEAgent（先按 sim/ros2/README.md 编译）"
[ -d "$WS/install" ] || die "找不到 $WS/install（先 colcon build）"

step '== 1/6 启动 DDS agent =='
( cd "$AGENT_DIR" && setsid nohup env LD_LIBRARY_PATH="$PWD" ./MicroXRCEAgent udp4 -p 8888 \
    > "$ROOT/logs/agent-visual-$TS.log" 2>&1 & ) 
sleep 2

step '== 2/6 启动 SITL =='
( cd "$PX4_DIR" && HEADLESS=1 PX4_GZ_WORLD="$WORLD" setsid nohup timeout 300 \
    make px4_sitl gz_"$MODEL" > "$LOG" 2>&1 < <(sleep 300) & )
disown -a 2>/dev/null || true
for _ in $(seq 1 60); do
  grep -qa "gz_bridge.*model: ${MODEL}" "$LOG" 2>/dev/null && break
  sleep 1
done
grep -qa "gz_bridge.*model: ${MODEL}" "$LOG" || { tail -5 "$LOG" >&2; die "SITL 未就绪"; }
echo "   模型已 spawn"

step '== 3/6 附着 Gazebo GUI =='
make_gui_config
GUI_ARGS=(-g)
[ -f "$GUI_CFG" ] && GUI_ARGS+=(--gui-config "$GUI_CFG")
( setsid nohup gz sim "${GUI_ARGS[@]}" > "$ROOT/logs/gz-gui-$TS.log" 2>&1 & )
disown -a 2>/dev/null || true
sleep 12
raise_gui

step '== 4/6 定位窗口并开始录制 =='
W=1920; H=1080; X=0; Y=0
GEO=$(xwininfo -root -tree 2>/dev/null | grep '"Gazebo Sim"' | grep -v mutter | head -1)
if [ -n "$GEO" ]; then
  SZ=$(echo "$GEO" | grep -oE '[0-9]+x[0-9]+\+[0-9]+\+[0-9]+' | head -1)
  AB=$(echo "$GEO" | grep -oE '\+[0-9]+\+[0-9]+' | tail -1 | sed 's/^+//; s/+/ /')
  W=$(echo "$SZ" | cut -dx -f1); H=$(echo "$SZ" | cut -dx -f2 | cut -d+ -f1)
  X=$(echo "$AB" | cut -d' ' -f1); Y=$(echo "$AB" | cut -d' ' -f2)
  echo "   窗口 ${W}x${H}+${X}+${Y}"
else
  echo "   ! 未找到 Gazebo 窗口，退回全屏 1920x1080"
fi

FFMPEG=""
command -v ffmpeg >/dev/null 2>&1 && FFMPEG=$(command -v ffmpeg)
[ -z "$FFMPEG" ] && FFMPEG=$("$PY" -c "import imageio_ffmpeg;print(imageio_ffmpeg.get_ffmpeg_exe())" 2>/dev/null || true)

if [ "$REC" = "1" ] && [ -n "$FFMPEG" ] && [ -n "${DISPLAY:-}" ]; then
  "$FFMPEG" -y -loglevel error -f x11grab -video_size "${W}x${H}" -framerate 15 \
    -i ":${DISPLAY#:}+${X},${Y}" -t 120 -vf scale=1280:-2 -pix_fmt yuv420p "$MP4" &
  REC_PID=$!
  echo "   录制中 → $MP4"
else
  echo "   ! 跳过录制（REC=$REC, ffmpeg=${FFMPEG:-无}, DISPLAY=${DISPLAY:-无}）"
fi

step '== 5/6 跑 demo =='
# shellcheck disable=SC1091
# ROS 2 的 setup.bash 引用未定义变量，必须临时关掉 set -u
set +u
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u
command -v ros2 >/dev/null || die "source 之后仍然找不到 ros2"
sleep 3                                  # 等 DDS 话题稳定
if [ "$MODE" = "landing" ]; then
  timeout 90 ros2 run x650_offboard landing_demo --ros-args \
    -p alt:="$ALT" -p landing_x:="$LAND_X" 2>&1 | tee -a "$LOG"
else
  timeout 90 ros2 run x650_offboard offboard_hover --ros-args -p alt:="$ALT" 2>&1 | tee -a "$LOG"
fi

step '== 6/6 收尾 =='
sleep 2
if [ -n "${REC_PID:-}" ]; then
  # 必须用 SIGTERM：SIGKILL 会让 mp4 缺少 moov atom，文件只有几 KB 且无法播放
  kill -TERM "$REC_PID" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "$REC_PID" 2>/dev/null || break
    sleep 1
  done
  kill -9 "$REC_PID" 2>/dev/null || true
  if [ -f "$MP4" ]; then
    SZ=$(du -h "$MP4" | cut -f1)
    step "视频：$MP4（$SZ）"
    [ "$(stat -c%s "$MP4")" -lt 50000 ] && echo "   ! 视频异常小，可能是录制被强杀"
  fi
fi
kill_leftovers
step "日志：$LOG"
step "DONE"
