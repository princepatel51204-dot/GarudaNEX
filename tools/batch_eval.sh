#!/usr/bin/env bash
# Usage: bash tools/batch_eval.sh [n_runs] [timeout_s]
set -u
N=${1:-10}
TMO=${2:-2400}
WS=~/GarudaNEX/ros2_ws
BATCH=~/GarudaNEX/results/batch_$(date +%m%d_%H%M)
mkdir -p "$BATCH"
echo "batch -> $BATCH   runs=$N  timeout=${TMO}s"

cleanup() {
  pkill -f smart_explorer; pkill -f run_recorder
  pkill -f component_container; pkill -f navigation_launch; pkill -f lifecycle_manager
  for n in controller_server planner_server bt_navigator behavior_server \
           smoother_server velocity_smoother waypoint_follower collision_monitor \
           docking_server route_server; do pkill -9 -f $n; done
  tmux kill-server 2>/dev/null
  pkill -f px4; pkill -f 'gz sim'; pkill -f ruby; pkill -f MicroXRCEAgent
  pkill -f rviz2; pkill -f slam_toolbox; pkill -f octomap; pkill -f parameter_bridge
  sleep 6
}

for i in $(seq 1 "$N"); do
  RUN="$BATCH/run$(printf '%02d' "$i")"
  mkdir -p "$RUN"
  echo "=== RUN $i/$N -> $RUN ==="
  cleanup
  bash "$WS/src/garudanex_bringup/scripts/start.sh" garudanex_facility > "$RUN/sim.log" 2>&1
  sleep 60
  source "$WS/install/setup.bash"
  ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
    use_composition:=False \
    params_file:="$WS/src/garudanex_navigation/config/nav2_uav.yaml" \
    > "$RUN/nav2.log" 2>&1 &
  sleep 35
  ros2 run garudanex_explore run_recorder --ros-args \
    -p use_sim_time:=true -p results_dir:="$RUN" > "$RUN/recorder.log" 2>&1 &
  REC=$!
  sleep 3
  timeout "$TMO" ros2 run garudanex_explore smart_explorer --ros-args \
    -p use_sim_time:=true -p results_dir:="$RUN" > "$RUN/explorer.log" 2>&1
  echo "  explorer exit=$?"
  kill -INT $REC 2>/dev/null; sleep 6; kill -9 $REC 2>/dev/null
  echo "  done: $(ls "$RUN" | tr '\n' ' ')"
done
cleanup
echo "BATCH COMPLETE -> $BATCH"
