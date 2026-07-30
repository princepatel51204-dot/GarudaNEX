#!/usr/bin/env python3
"""Append the drone's current map pose to the mission waypoint YAML.

    python3 capture_waypoint.py aisle_east_1

Waits up to 30 s for map -> base_footprint, because slam_toolbox's transform
is intermittently stale while it catches up on scan processing.
"""
import os, sys, math, time, yaml, rclpy
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener

name = sys.argv[1] if len(sys.argv) > 1 else 'wp'
path = os.path.expanduser(
    '~/GarudaNEX/ros2_ws/src/garudanex_mission/config/warehouse_inspection.yaml')

rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
node = rclpy.create_node('capture_waypoint',
                         automatically_declare_parameters_from_overrides=True)
buf = Buffer()
TransformListener(buf, node, spin_thread=True)

tf, last_err, deadline = None, '', time.time() + 30.0
while time.time() < deadline and tf is None:
    try:
        tf = buf.lookup_transform('map', 'base_footprint',
                                  rclpy.time.Time(), timeout=Duration(seconds=2.0))
    except Exception as e:
        last_err = str(e)[:90]
        time.sleep(0.3)

if tf is None:
    print('FAILED after 30 s: %s' % last_err)
    print('check with: ros2 run tf2_ros tf2_echo map odom --ros-args -p use_sim_time:=true')
    rclpy.shutdown()
    sys.exit(1)

t, q = tf.transform.translation, tf.transform.rotation
yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

data = {'waypoints': []}
if os.path.exists(path):
    data = yaml.safe_load(open(path)) or {'waypoints': []}
data.setdefault('waypoints', []).append(
    {'name': name, 'x': round(t.x, 3), 'y': round(t.y, 3), 'yaw': round(yaw, 3)})

os.makedirs(os.path.dirname(path), exist_ok=True)
yaml.safe_dump(data, open(path, 'w'), sort_keys=False)
print('captured %-14s x=%7.3f  y=%7.3f  yaw=%6.3f   (total %d)'
      % (name, t.x, t.y, yaw, len(data['waypoints'])))
rclpy.shutdown()
