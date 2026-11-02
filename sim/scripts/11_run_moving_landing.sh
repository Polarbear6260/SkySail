#!/bin/bash
# 从 Gazebo 窗口出现的那一刻开始录，直到 demo 结束
WS=/home/green/文档/deepseek-harness/default-workspace/px4-gz-sim
cd "$WS"
TS=$(date +%H%M%S); LOG="$WS/logs/live_$TS.log"; MP4="$WS/logs/x650-moving-$TS.mp4"
PY=/home/green/planning_ws/.venv/bin/python
FF=$($PY -c "import imageio_ffmpeg;print(imageio_ffmpeg.get_ffmpeg_exe())")

PIDS=$(ps -eo pid,args | awk '/move_platf|moving_landing|MicroXRCEAgent|bin\/px4/ && !/awk/ && !/grep/ {print $1}')
[ -n "$PIDS" ] && kill -9 $PIDS 2>/dev/null
for p in $(pgrep -x ruby 2>/dev/null); do tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "gz sim" && kill -9 $p; done
sleep 3; rm -f /tmp/px4_lock-0 /tmp/px4-sock-0
echo "[$(date +%H:%M:%S)] 清场" > "$WS/logs/live_chain3.log"

(cd ~/tools/Micro-XRCE-DDS-Agent/build2 && setsid nohup env LD_LIBRARY_PATH=$PWD ./MicroXRCEAgent udp4 -p 8888 > "$WS/logs/agent_live3.log" 2>&1 &)
sleep 3
(cd ~/PX4-Autopilot && HEADLESS=1 PX4_GZ_WORLD=x650_sea PX4_GZ_MODEL_POSE="0,16,1.02,0,0,0" \
   setsid nohup timeout 400 make px4_sitl gz_x650 > "$WS/logs/sitl_live3.log" 2>&1 < <(sleep 400) &)
for i in $(seq 1 90); do grep -qa "gz_bridge.*model: x650" "$WS/logs/sitl_live3.log" 2>/dev/null && break; sleep 1; done
echo "[$(date +%H:%M:%S)] SITL 就绪" >> "$WS/logs/live_chain3.log"

# 参数：地面站/电池失效保护全部关掉（仿真专用）
(cd ~/PX4-Autopilot && source .venv-px4/bin/activate
 for kv in "NAV_DLL_ACT 0" "SIM_BAT_ENABLE 0" "COM_LOW_BAT_ACT 0" "BAT1_V_EMPTY 2.0" "BAT1_R_INTERNAL 0" "SIM_BAT_MIN_PCT 100" "SIM_BAT_DRAIN 100000" "COM_OF_LOSS_T 5.0" "COM_OBL_RC_ACT 0"; do
   printf "param set %s\n" "$kv" | timeout 15 python Tools/mavlink_shell.py udpin:127.0.0.1:14540 >/dev/null 2>&1
 done)
echo "[$(date +%H:%M:%S)] 参数已设（含电池阈值）" >> "$WS/logs/live_chain3.log"

# GUI：先起窗口
python3 - <<'PY' 2>/dev/null
import re, os
p = os.path.expanduser("~/.gz/sim/8/gui-demo.config"); s = open(p).read()
s = re.sub(r"<camera_pose>[^<]*</camera_pose>", "<camera_pose>0 -15 9 0 0.42 1.5708</camera_pose>", s, count=1)
open(p,"w").write(s)
PY
setsid nohup gz sim -g --gui-config ~/.gz/sim/8/gui-demo.config > "$WS/logs/gui_live3.log" 2>&1 &
# 等窗口出现（最多 30 s），立刻开录
for i in $(seq 1 60); do
  xwininfo -root -tree 2>/dev/null | grep -q '"Gazebo Sim".*gz-sim-gui' && break
  sleep 0.5
done
GEO=$(xwininfo -root -tree 2>/dev/null | grep '"Gazebo Sim"' | grep -v mutter | head -1)
SZ=$(echo "$GEO" | grep -oE '[0-9]+x[0-9]+\+[0-9]+\+[0-9]+' | head -1)
AB=$(echo "$GEO" | grep -oE '\+[0-9]+\+[0-9]+' | tail -1 | sed 's/^+//;s/+/ /')
GW=$(echo "$SZ" | cut -dx -f1); GH=$(echo "$SZ" | cut -dx -f2 | cut -d+ -f1)
GX=$(echo "$AB" | cut -d' ' -f1); GY=$(echo "$AB" | cut -d' ' -f2)
"$FF" -y -loglevel error -f x11grab -video_size "${GW}x${GH}" -framerate 15 \
   -i ":1+${GX},${GY}" -t 300 -vf scale=1280:-2 -pix_fmt yuv420p "$MP4" &
REC_PID=$!
echo "[$(date +%H:%M:%S)] 录制开始（窗口刚出现）$MP4" >> "$WS/logs/live_chain3.log"

sleep 2
$PY - <<'PY' 2>/dev/null
from Xlib import display, X
from Xlib.protocol import event as xevent
d = display.Display(); root = d.screen().root; target=[None]
def walk(w):
    try:
        for c in w.query_tree().children:
            try:
                if (c.get_wm_name() or "").strip() == "Gazebo Sim": target[0]=c
            except Exception: pass
            walk(c)
    except Exception: pass
walk(root)
if target[0]:
    a = d.intern_atom("_NET_ACTIVE_WINDOW")
    e = xevent.ClientMessage(window=target[0], client_type=a, data=(32,[2,X.CurrentTime,0,0,0]))
    root.send_event(e, event_mask=X.SubstructureRedirectMask|X.SubstructureNotifyMask); d.sync()
    target[0].configure(stack_mode=X.Above); d.sync()
PY

set +u; source /opt/ros/jazzy/setup.bash; source ~/x650_ws/install/setup.bash
setsid nohup env PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH python3 scripts/15_tie_down.py --duration 230 > "$WS/logs/tiedown.txt" 2>&1 &
setsid nohup env PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH python3 scripts/12_log_pose.py --world x650_sea --duration 220 --csv "$WS/logs/pose_$(date +%H%M%S).csv" > "$WS/logs/pose_log.txt" 2>&1 &
setsid nohup env PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH python3 scripts/10_move_platform.py --world x650_sea --delay 12 --start-yaw 0 --start-x -40 --speed 1.5 \
   --duration 240 --csv "$WS/logs/ship_live3.csv" > "$WS/logs/ship_live3.log" 2>&1 &
sleep 3
timeout 120 ros2 run x650_offboard moving_landing --ros-args -p alt:=2.5 > "$LOG" 2>&1
echo "[$(date +%H:%M:%S)] 节点结束" >> "$WS/logs/live_chain3.log"
# 触板即停：立刻收掉仿真与录制
PIDS=$(ps -eo pid,args | awk '/move_platf|moving_landing|MicroXRCEAgent|bin\/px4|12_log_pose|15_tie|gz sim/ && !/awk/ && !/grep/ {print $1}')
[ -n "$PIDS" ] && kill -9 $PIDS 2>/dev/null
for _p in $(pgrep -x ruby 2>/dev/null); do tr '\0' ' ' < /proc/$_p/cmdline 2>/dev/null | grep -q "gz sim" && kill -9 $_p; done
RESULT=$(grep -a "TOUCHDOWN" "$LOG" | tail -1)
[ -n "$RESULT" ] && echo "[$(date +%H:%M:%S)] $RESULT" >> "$WS/logs/live_chain3.log"
if [ -n "${REC_PID:-}" ]; then
  kill -TERM "$REC_PID" 2>/dev/null; for i in $(seq 1 25); do kill -0 "$REC_PID" 2>/dev/null || break; sleep 1; done
  kill -9 "$REC_PID" 2>/dev/null
  echo "[$(date +%H:%M:%S)] 视频 $MP4 ($(du -h "$MP4" 2>/dev/null | cut -f1))" >> "$WS/logs/live_chain3.log"
fi
