#!/usr/bin/env bash
# Kill every GarudaNEX process, including Gazebo.
#
# A leftover `gz sim` server keeps ownership of the world. The next
# `make px4_sitl` then spawns its model into that stale world instead of a
# fresh one - the GUI never appears and PX4 reports
# "Preflight Fail: Attitude failure (roll)" because the model tips over on
# top of the previous wreck. Killing tmux alone does NOT clean this up.
echo "--- stopping GarudaNEX ---"

tmux kill-server 2>/dev/null || true

for p in \
  "rviz2" \
  "odometry_bridge" "cmd_vel_bridge" "scan_frame_relay" \
  "robot_state_publisher" "parameter_bridge" \
  "MicroXRCEAgent" \
  "px4_sitl" "bin/px4" \
  "gz sim" "gz-sim" "gzserver" "gzclient" "simulation-gazebo"
do
  pkill -9 -f "$p" 2>/dev/null && echo "  killed: $p" || true
done

sleep 2

echo "--- survivors (should be empty) ---"
pgrep -af "px4|gz sim|MicroXRCEAgent|parameter_bridge|rviz2|garudanex" || echo "  none. clean."
