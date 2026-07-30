#!/usr/bin/env bash
# GarudaNEX full SITL stack in one tmux session.
#
#   ./start.sh [world]        default world: walls
#
# Windows:
#   px4    PX4 SITL + Gazebo (interactive pxh> prompt)
#   ros    uXRCE-DDS agent + gz bridge + relay + description + odom bridge
#   rviz   RViz2 with the GarudaNEX config
#   shell  free shell, workspace already sourced
#
# Detach: Ctrl-b d      Switch window: Ctrl-b <number>
# Kill everything: tmux kill-session -t garudanex
set -euo pipefail

SESSION="garudanex"
WS="${HOME}/GarudaNEX/ros2_ws"
PX4_DIR="${HOME}/PX4-Autopilot"
WORLD="${1:-walls}"
AIRFRAME="gz_x500_lidar_2d"
RVIZ_CFG="${WS}/src/garudanex_description/rviz/garudanex.rviz"
SRC="source /opt/ros/jazzy/setup.bash && source ${WS}/install/setup.bash"

# Clean slate: stale processes are the #1 cause of duplicate-publisher bugs.
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"
pkill -f MicroXRCEAgent    2>/dev/null || true
pkill -f parameter_bridge  2>/dev/null || true
pkill -f odometry_bridge   2>/dev/null || true
pkill -f scan_frame_relay  2>/dev/null || true
pkill -f robot_state_publisher 2>/dev/null || true
sleep 1

tmux new-session  -d -s "$SESSION" -n px4
tmux send-keys -t "$SESSION:px4" \
  "cd ${PX4_DIR} && PX4_GZ_WORLD=${WORLD} make px4_sitl ${AIRFRAME}" C-m

# T+18s: PX4 and Gazebo need to be up before the DDS agent and /clock bridge.
tmux new-window -t "$SESSION" -n ros
tmux send-keys -t "$SESSION:ros" \
  "${SRC} && sleep 18 && ros2 launch garudanex_bringup garudanex_sitl.launch.py world:=${WORLD}" C-m

# T+30s: RViz last, so /clock and TF already exist.
tmux new-window -t "$SESSION" -n rviz
tmux send-keys -t "$SESSION:rviz" \
  "${SRC} && sleep 30 && rviz2 -d ${RVIZ_CFG} --ros-args -p use_sim_time:=true" C-m

tmux new-window -t "$SESSION" -n shell
tmux send-keys -t "$SESSION:shell" "${SRC} && cd ${WS}" C-m

tmux select-window -t "$SESSION:px4"
echo "GarudaNEX starting in world '${WORLD}'. Attaching in 2s..."
sleep 2
tmux attach -t "$SESSION"
