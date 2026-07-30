#!/usr/bin/env bash
# GarudaNEX full SITL stack in one tmux session.
#
#   ./start.sh [world]        default world: walls
#
# Windows:  0:px4  1:ros  2:rviz  3:shell
# Detach Ctrl-b d | switch Ctrl-b <n> | kill: ./stop.sh
set -euo pipefail

SESSION="garudanex"
WS="${HOME}/GarudaNEX/ros2_ws"
PX4_DIR="${HOME}/PX4-Autopilot"
WORLD="${1:-walls}"
AIRFRAME="gz_x500_lidar_2d"
RVIZ_CFG="${WS}/src/garudanex_description/rviz/garudanex.rviz"
SRC="source /opt/ros/jazzy/setup.bash && source ${WS}/install/setup.bash"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Full clean first. Zombie Gazebo servers cause the model to spawn into a
# stale world and immediately trip the roll failure detector.
"${HERE}/stop.sh"

tmux new-session -d -s "$SESSION" -n px4
tmux send-keys -t "$SESSION:px4" \
  "cd ${PX4_DIR} && PX4_GZ_WORLD=${WORLD} make px4_sitl ${AIRFRAME}" C-m

# Wait for Gazebo to actually be serving /clock, rather than guessing a sleep.
tmux new-window -t "$SESSION" -n ros
tmux send-keys -t "$SESSION:ros" \
  "${SRC} && \
   printf 'waiting for Gazebo /clock'; \
   for i in \$(seq 1 60); do gz topic -l 2>/dev/null | grep -qx '/clock' && break; printf '.'; sleep 1; done; \
   echo ' up'; sleep 3; \
   ros2 launch garudanex_bringup garudanex_sitl.launch.py world:=${WORLD}" C-m

# RViz waits for /scan, which means the whole bridge chain is alive.
tmux new-window -t "$SESSION" -n rviz
tmux send-keys -t "$SESSION:rviz" \
  "${SRC} && \
   printf 'waiting for /scan'; \
   for i in \$(seq 1 90); do ros2 topic list 2>/dev/null | grep -qx '/scan' && break; printf '.'; sleep 1; done; \
   echo ' up'; sleep 2; \
   rviz2 -d ${RVIZ_CFG} --ros-args -p use_sim_time:=true" C-m

tmux new-window -t "$SESSION" -n shell
tmux send-keys -t "$SESSION:shell" "${SRC} && cd ${WS}" C-m

tmux select-window -t "$SESSION:px4"
echo "GarudaNEX starting in world '${WORLD}'..."
sleep 2
tmux attach -t "$SESSION"
