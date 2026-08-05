# Silent failure classes

Sixteen failures encountered building GarudaNEX where every health check
reported green and nothing worked. Each entry is symptom, cause, fix.

### 1. slam_toolbox launched as a plain Node
- **Symptom** `/map` never published. Node alive, no errors, no warnings.
- **Cause** `async_slam_toolbox_node` is a lifecycle node. Launched as a plain
  `Node` it sits in `unconfigured`: parameters undeclared, no `/scan`
  subscription, and no complaint.
- **Fix** Launch as `LifecycleNode` with explicit configure/activate events, and
  register the `OnStateTransition` handler *before* emitting configure.

### 2. gz sdf -k reports errors on valid worlds
- **Symptom** World validation fails on every Fuel `<uri>`.
- **Cause** The standalone SDF parser has no Fuel resolver.
- **Fix** Validate by loading the world headless instead of parsing it.

### 3. Nav2 TF_ERROR 102 on every goal
- **Symptom** "Unable to transform goal pose into costmap frame".
- **Cause** `transform_tolerance` 0.3 s against a 4.6 Hz control loop.
- **Fix** Raise to 1.0 s and reduce MPPI load so the loop keeps up.

### 4. Voxel local costmap on a flying vehicle
- **Symptom** "sensor origin out of map bounds (0.00 to 0.78)".
- **Cause** The drone at 1.54 m was above its own 3D costmap column.
- **Fix** Use a 2D `obstacle_layer` with an 8x8 m window; the flattened scan
  already encodes the relevant geometry.

### 5. collision_monitor throttling to 0.1 m/s
- **Symptom** Goals fail with `105 FAILED_TO_MAKE_PROGRESS`; the drone crawls.
- **Cause** The approach polygon scaled velocity toward zero because the LiDAR
  was reporting obstacles at `range_min` (see #8).
- **Fix** Fix the self-return first; the monitor is then safe to leave enabled.

### 6. Nav2 returns error_code 0 on ABORTED goals
- **Symptom** Every goal reported success; `visited` climbed to 42 with the
  drone stationary.
- **Cause** `error_code` is only populated on some failure paths. On abort it
  stays 0, so `if error_code == 0` treats every failure as success.
- **Fix** Only `GoalStatus.STATUS_SUCCEEDED` is a reliable success signal.

### 7. PX4 stack smashing on a multi-ring LiDAR
- **Symptom** `*** stack smashing detected ***`, PX4 SITL dies at spawn.
- **Cause** PX4's `gz_bridge` auto-binds any airframe LiDAR and copies it into
  the fixed-size `obstacle_distance` uORB message; 16 rings overrun the buffer.
- **Fix** Rename the sensor so PX4's binding does not match it.

### 8. LiDAR detecting its own airframe
- **Symptom** Every goal fails `208 NO_VALID_PATH`, even 2 m away.
- **Cause** `range_min` 0.15 m let prop tips and legs return at exactly 0.150 m
  on every beam, forming a closed obstacle ring around the robot.
- **Fix** `range_min` 0.55 m. Min observed range went 0.150 -> 2.45 m.

### 9. Frontier clearance window has two hard bounds
- **Symptom** Either frontiers land in lethal cells, or none are found at all.
- **Cause** Clearance must be >= robot radius (0.38 m) and < half the narrowest
  passage (0.80 m for a 1.6 m door). Outside that window it always fails.
- **Fix** 0.50 m.

### 10. Frontier livelock at zero distance
- **Symptom** Same frontier selected 26 times in 8 s; map never changed.
- **Cause** Goals 0.14 m away return SUCCEEDED without any motion.
- **Fix** `min_goal_distance` 1.5 m.

### 11. Symmetric LiDAR rings have no zero-degree beam
- **Symptom** Flattened scan truncated to ~8.6 m despite a 30 m sensor.
- **Cause** 16 symmetric rings sit at +/-1, +/-3 degrees. With no beam at
  exactly 0, the +/-0.15 m height filter rejected all long returns.
- **Fix** 15 samples over +/-14 degrees (2 degree spacing, includes 0), height
  band widened to +/-0.25 m.

### 12. SLAM produces 0.0% occupied cells
- **Symptom** Map exists but contains no obstacles.
- **Cause** Not perception. `minimum_travel_distance: 0.3` means a hovering
  drone generates no pose-graph nodes at all.
- **Fix** Seed SLAM with a short commanded motion before exploring.

### 13. QoS mismatch silently drops all sensor data
- **Symptom** `closest_obstacle_m` recorded as `inf` for an entire run.
- **Cause** Subscriber used default RELIABLE; the LiDAR publishes BEST_EFFORT.
  The only sign is one easily-missed warning line.
- **Fix** `qos_profile_sensor_data` on every sensor subscription.

### 14. Bringup script silently defaults to a throwaway world
- **Symptom** Map plateaus at 132 m2 with 1-3 frontiers; exploration ends in 60 s.
- **Cause** `WORLD="${1:-walls}"` - running the script bare loaded a 4-box test
  world instead of the facility. Every node reported healthy.
- **Fix** Fail loudly on a missing world argument rather than defaulting.
  This one cost four full debugging cycles.

### 15. Duplicate composed Nav2 nodes
- **Symptom** `bt_navigator`: "unknown goal response, ignoring" then
  `BtActionNode::Tick: invalid status value`. Every goal aborts in 460 ms.
- **Cause** Two complete Nav2 stacks running. Action replies came from the
  wrong instance. `pkill -f controller_server` never matched because composed
  nodes live inside `component_container_isolated` processes.
- **Fix** Kill `component_container`, relaunch with `use_composition:=False`.
  Note that every lifecycle check reported `active` throughout.

### 16. Landing at the end broke every subsequent run
- **Symptom** After adding auto-land, later runs mapped almost nothing.
- **Cause** Each new run started with the drone on the ground, so the 2D scan
  slice sat at floor level and saw nothing.
- **Fix** Explorer climbs or descends to cruise altitude before exploring, and
  holds altitude on completion by default.
