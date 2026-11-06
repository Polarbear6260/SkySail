#!/usr/bin/env bash
# 27 · 成功率统计：连续跑 N 次真值着陆，汇总结果
# 用法：./scripts/27_success_sweep.sh [次数]
set -uo pipefail
cd "$(dirname "$0")/.."
N="${1:-6}"
OUT="logs/success-sweep-$(date +%Y%m%d-%H%M%S).csv"
echo "run,result,above_pad_m,horiz_m,time_s,log" > "$OUT"

for i in $(seq 1 "$N"); do
  echo "=== 第 $i/$N 次 ==="
  PIDS=$(ps -eo pid,args | awk '/16_moving_landing|10_move_platform|12_log_pose|19_gps|22_aruco|25_z_drift|17_debug|MicroXRCEAgent|bin\/px4|gz sim/ && !/awk/ && !/grep/ {print $1}')
  [ -n "$PIDS" ] && kill -9 $PIDS 2>/dev/null
  for p in $(pgrep -x ruby 2>/dev/null); do tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "gz sim" && kill -9 $p; done
  sleep 3; rm -f /tmp/px4_lock-0

  LOG="logs/sweep-$i.log"
  ./scripts/16_moving_landing_demo.sh --no-gui --no-record --timeout 100 > "$LOG" 2>&1
  sleep 3

  LINE=$(grep -a "TOUCHDOWN 成功" "$LOG" | tail -1)
  if [ -n "$LINE" ]; then
    ABOVE=$(echo "$LINE" | grep -oE "高于板面 [-+0-9.]+" | grep -oE "[-+0-9.]+")
    HORIZ=$(echo "$LINE" | grep -oE "水平误差 [0-9.]+" | grep -oE "[0-9.]+")
    TS=$(echo "$LINE" | grep -oE "t=[0-9.]+" | grep -oE "[0-9.]+")
    echo "$i,成功,$ABOVE,$HORIZ,$TS,$LOG" | tee -a "$OUT"
  else
    echo "$i,失败,,,," | tee -a "$OUT" | sed 's/^/  /'
  fi
done
echo; echo "汇总 → $OUT"
awk -F, 'NR>1{n++; if($2=="成功"){s++; a+=$4; if($4>m)m=$4}} END{printf "成功 %d/%d = %.0f%%", s, n, 100*s/n; if(s)printf "，水平误差 平均 %.2f m，最大 %.2f m\n", a/s, m; else print ""}' "$OUT"
