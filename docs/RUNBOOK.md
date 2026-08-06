# GarudaNEX Operating Runbook

Four terminals, in order. Every gate must pass before continuing.

## Reference run

| Metric | Value |
|---|---|
| Environment | 60 x 40 m GPS-denied facility, 76 obstacle links |
| Map produced | 64.9 x 46.8 m, 2131 m2 free, 74.5% known |
| Distance flown | 671 m, fully autonomous |
| Duration | 33.9 min |
| Goals reached | 58 / 78 (74.4%) |
| Zones retired by coverage memory | 51 |
| Closest obstacle approach | 0.55 m (LiDAR range_min) |
| Collisions | 0 |
| Human input after launch | none |

## Terminal 1 - simulation, PX4 arm and take-off

```bash
# clean
pkill -f smart_explorer; pkill -f run_recorder; pkill -f component_container
pkill -f navigation_launch; pkill -f lifecycle_manager; tmux kill-server 2>/dev/null
for n in controller_server planner_server bt_navigator behavior_server smoother_server \
  velocity_smoother waypoint_follower collision_monitor docking_server route_server; do
  pkill -9 -f $n; done
pkill -9 -f px4; pkill -9 -f 'gz sim'; pkill -9 -f ruby; pkill -f MicroXRCEAgent
pkill -f rviz2; pkill -f slam_toolbox; pkill -f octomap; pkill -f parameter_bridge
sleep 8

# simulation (world argument is mandatory)
cd ~/GarudaNEX/ros2_ws && bash src/garudanex_bringup/scripts/start.sh garudanex_facility_pro
sleep 60

# PX4: arm, take off, offboard
tmux send-keys -t garudanex:0 'commander takeoff' C-m;      sleep 12
tmux send-keys -t garudanex:0 'commander mode offboard' C-m; sleep 5
tmux capture-pane -pt garudanex:0 | tail -8
```

Expect `Armed by internal command` and `Takeoff detected`.

### Pre-flight gate

```bash
source install/setup.bash
ros2 topic list | grep -o '/world/[^/]*' | sort -u   # /world/garudanex_facility_pro
ros2 lifecycle get /slam_toolbox                     # active [3]
ros2 topic info /scan    | grep Publisher            # >= 1
ros2 topic info /cmd_vel | grep Subscription         # >= 1
```

If slam_toolbox is not active: `ros2 lifecycle set /slam_toolbox configure` then `activate`.

## Terminal 2 - Nav2

```bash
cd ~/GarudaNEX/ros2_ws && source install/setup.bash
ros2 node list | grep -cE 'bt_navigator$|controller_server$'   # MUST be 0

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true use_composition:=False \
  params_file:=$PWD/src/garudanex_navigation/config/nav2_uav.yaml \
  > ~/GarudaNEX/logs/nav2.log 2>&1 &
sleep 35

for n in bt_navigator controller_server planner_server behavior_server \
         smoother_server collision_monitor; do
  echo -n "$n: "; ros2 node list | grep -cx "/$n"; done   # each must be 1
ros2 param get /controller_server FollowPath.vx_max        # 1.3
ros2 param get /planner_server GridBased.allow_unknown     # False

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, \
orientation: {w: 1.0}}}}"                                  # must SUCCEED
```

## Terminal 3 - metrics recorder

```bash
cd ~/GarudaNEX/ros2_ws && source install/setup.bash
RUN=~/GarudaNEX/results/run_$(date +%m%d_%H%M); mkdir -p $RUN
echo $RUN > ~/GarudaNEX/results/.current
ros2 run garudanex_explore run_recorder --ros-args -p use_sim_time:=true -p results_dir:=$RUN
```

## Terminal 4 - autonomous exploration

```bash
cd ~/GarudaNEX/ros2_ws && source install/setup.bash
RUN=$(cat ~/GarudaNEX/results/.current)
ros2 run garudanex_explore smart_explorer --ros-args \
  -p use_sim_time:=true -p results_dir:=$RUN 2>&1 | tee $RUN/explorer.log
```

In RViz add a MarkerArray on `/garudanex/frontiers` - cyan candidates, orange committed target.

## After the run

Ctrl-C Terminal 3 first, then:

```bash
RUN=$(cat ~/GarudaNEX/results/.current)
cat $RUN/explorer_summary.json; cat $RUN/recorder_summary.json
python3 src/garudanex_explore/tools/plot_run.py $RUN
```

## Three rules

1. **Never launch Nav2 twice.** Duplicate nodes make every goal abort with
   `BtActionNode::Tick: invalid status value` while all lifecycle checks still read active.
2. **Always restart the simulation between runs.** A persisted SLAM map makes the next
   run finish in 13 s with meaningless numbers.
3. **The drone must be airborne at ~1.5 m before the explorer starts.** On the ground the
   2D scan slice sits at floor level and maps almost nothing.

See [debugging.md](debugging.md) for sixteen documented silent-failure classes.
