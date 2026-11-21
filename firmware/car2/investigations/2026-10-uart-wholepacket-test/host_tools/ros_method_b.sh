#!/bin/bash
# 方法 B（Orin 端）：啟動 uart_bridge_node（選擇性加上真正的感知節點），跑一段受控速度曲線，記錄 link status。
# 用法：ros_method_b.sh <label> <bridge_rate_hz> <minutes> <perception 0|1> <wheel_max_pct(<=60)>
# 需要輪子離地。BMduino 端的接收統計由電腦端 usb_method_b.py 從 USB 讀取。
LABEL="$1"; RATE="$2"; MINUTES="$3"; PERCEPTION="$4"; WHEEL="$5"
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source ~/car2_ws/install/setup.bash >/dev/null 2>&1
D=~/uart_wholepacket_test_20261008
ST=$D/ros_link_status_${LABEL}.txt
rm -f "$ST"
if [ "$WHEEL" -gt 60 ]; then echo "ABORT: wheel_max 超過 60"; exit 1; fi

OTHERS=$(pgrep -af "uart_bridge_node|line_follow_control|drive_dashboard|cmd_tcp_bridge|raw_serial_test|wp_test.py|ob_camera|depth_bev" | grep -v pgrep)
if [ -n "$OTHERS" ]; then echo "ABORT: 已有相關程式在跑:"; echo "$OTHERS"; exit 1; fi

snap() { grep -E "^(bm_to_orin|orin_to_bm|cmd_seq_sent|last_ackseq|sta_ok_count|sta_checksum_fail_count|fault_gate|motor_fault_flags)" "$ST" | tr '\n' ' '; }

PL=""; PGID=""
if [ "$PERCEPTION" = "1" ]; then
  setsid nohup ros2 launch car2_bringup car2_orbbec_bringup.launch.py > "$D/ros_${LABEL}_perception.log" 2>&1 &
  PL=$!; sleep 1; PGID=$(ps -o pgid= -p $PL | tr -d ' ')
  echo "[perception] launch pgid=$PGID，等 22 秒讓相機與感知起來"
  sleep 21
  echo "[perception] loadavg: $(cut -d' ' -f1-3 /proc/loadavg)"
fi

nohup ros2 run car2_bringup uart_bridge_node --ros-args -p max_wheel_speed:=${WHEEL}.0 -p control_rate_hz:=${RATE}.0 \
  -p link_status_file:="$ST" > "$D/ros_${LABEL}_bridge.log" 2>&1 &
BR=$!
sleep 3
echo "[bridge] $(grep -m1 '啟動' "$D/ros_${LABEL}_bridge.log" | sed 's/.*uart_bridge_node 啟動/uart_bridge_node 啟動/')"
echo "[start ] $(snap)"

python3 "$D/ros_cmd_vel_profile.py" --pct-max "$WHEEL" --seconds $((MINUTES*60)) > "$D/ros_${LABEL}_profile.log" 2>&1 &
PP=$!
for i in $(seq 1 $MINUTES); do
  sleep 60
  echo "[t=${i}min] loadavg $(cut -d' ' -f1 /proc/loadavg) | $(snap)"
done
wait $PP
sleep 1.5
echo "[end   ] $(snap)"
echo "--- 最終 link status"
cat "$ST"
echo "--- profile: $(tail -1 "$D/ros_${LABEL}_profile.log")"

kill -INT $BR 2>/dev/null; sleep 2
if pgrep -f "uart_bridge_node" >/dev/null; then pkill -TERM -f uart_bridge_node; sleep 1; fi
if [ -n "$PGID" ]; then
  kill -INT -$PGID 2>/dev/null; sleep 8
  if ps -eo pgid | awk '{print $1}' | grep -qx "$PGID"; then kill -TERM -$PGID 2>/dev/null; sleep 3; fi
  if ps -eo pgid | awk '{print $1}' | grep -qx "$PGID"; then kill -KILL -$PGID 2>/dev/null; sleep 1; fi
fi
echo "--- 結束後殘留"
pgrep -af "uart_bridge_node|line_follow_control|depth_bev|ob_camera|method_b_profile|ros_cmd_vel_profile" | grep -v pgrep || echo "無殘留"
