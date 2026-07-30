#!/usr/bin/env bash
# Verify GarudaNEX is actually ready before commanding motion.
# Run this BEFORE every flight. A dead PX4 makes cmd_vel a no-op that
# looks like success.

ok()   { printf "  \033[32m%-28s %s\033[0m\n" "$1" "$2"; }
bad()  { printf "  \033[31m%-28s %s\033[0m\n" "$1" "$2"; }

echo "=== processes ==="
pgrep -f "bin/px4"      >/dev/null && ok "px4 SITL" "running"  || bad "px4 SITL" "DEAD - restart"
pgrep -f "gz sim"       >/dev/null && ok "gazebo" "running"     || bad "gazebo" "DEAD"
pgrep -f MicroXRCEAgent >/dev/null && ok "uxrce-dds agent" "running" || bad "uxrce-dds agent" "DEAD"

echo "=== ros 2 nodes ==="
NODES="$(ros2 node list 2>/dev/null)"
for n in garudanex_odom_bridge garudanex_cmd_vel_bridge \
         garudanex_scan_frame_relay slam_toolbox robot_state_publisher; do
  grep -q "$n" <<<"$NODES" && ok "$n" "up" || bad "$n" "MISSING"
done

echo "=== topics ==="
TOPICS="$(ros2 topic list 2>/dev/null)"
for t in /clock /scan /odom /map /tf /tf_static; do
  grep -qx "$t" <<<"$TOPICS" && ok "$t" "present" || bad "$t" "MISSING"
done

echo "=== px4 state ==="
ST="$(timeout 3 ros2 topic echo /fmu/out/vehicle_status_v1 --once \
        --qos-reliability best_effort --qos-durability transient_local 2>/dev/null)"
ARM="$(grep -m1 'arming_state:' <<<"$ST" | awk '{print $2}')"
NAV="$(grep -m1 'nav_state:'    <<<"$ST" | awk '{print $2}')"
case "$ARM" in
  2) ok  "arming_state" "2 = ARMED" ;;
  1) bad "arming_state" "1 = DISARMED" ;;
  *) bad "arming_state" "no data - is PX4 alive?" ;;
esac
case "$NAV" in
  14) ok  "nav_state" "14 = OFFBOARD  <-- ready to fly" ;;
  4)  bad "nav_state" "4 = AUTO_LOITER - run: commander mode offboard" ;;
  17) bad "nav_state" "17 = AUTO_TAKEOFF - still climbing, wait" ;;
  *)  bad "nav_state" "${NAV:-none} - not offboard" ;;
esac

echo "=== tf: map -> base_link ==="
timeout 3 ros2 run tf2_ros tf2_echo map base_link \
  --ros-args -p use_sim_time:=true 2>&1 \
  | grep -m1 -E "Translation|does not exist" \
  | sed 's/^/  /' || echo "  no data"
