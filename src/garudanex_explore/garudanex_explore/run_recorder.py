#!/usr/bin/env python3
"""GarudaNEX run recorder: 1 Hz metrics CSV + summary JSON.

Runs alongside the explorer. Ctrl-C writes the summary.
Tracks path length, speed, closest obstacle approach, coverage growth.
"""
import os, csv, json, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import tf2_ros


class Recorder(Node):
    def __init__(self):
        super().__init__('run_recorder')
        self.declare_parameter('results_dir', '~/GarudaNEX/results/latest')
        self.declare_parameter('collision_threshold', 0.35)
        self.out = os.path.expanduser(self.get_parameter('results_dir').value)
        self.cth = self.get_parameter('collision_threshold').value
        os.makedirs(self.out, exist_ok=True)

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.grid = None
        self.scan_min = float('inf')
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)

        self.t0 = time.time()
        self.prev = None
        self.dist = 0.0
        self.speeds = []
        self.closest = float('inf')
        self.contacts = 0
        self.rows = 0
        self.f = open(os.path.join(self.out, 'metrics.csv'), 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t_s', 'x', 'y', 'yaw_deg', 'dist_m', 'speed_mps',
                         'min_scan_m', 'known_pct', 'free_m2', 'occ_m2'])
        self.f.flush()
        self.create_timer(1.0, self.tick)
        self.get_logger().info('recording -> %s' % self.out)

    def on_map(self, m):
        h, w = m.info.height, m.info.width
        self.grid = (np.asarray(m.data, dtype=np.int8).reshape(h, w),
                     m.info.resolution)

    def on_scan(self, s):
        r = np.asarray(s.ranges, dtype=np.float32)
        r = r[np.isfinite(r) & (r > s.range_min)]
        if r.size:
            self.scan_min = float(r.min())

    def pose(self):
        for fr in ('base_footprint', 'base_link'):
            try:
                t = self.tfbuf.lookup_transform('map', fr, rclpy.time.Time())
            except Exception:
                continue
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            return t.transform.translation.x, t.transform.translation.y, yaw
        return None

    def tick(self):
        p = self.pose()
        if p is None:
            return
        x, y, yaw = p
        sp = 0.0
        if self.prev is not None:
            d = math.hypot(x - self.prev[0], y - self.prev[1])
            if d < 5.0:
                self.dist += d
                sp = d
        self.prev = (x, y)
        self.speeds.append(sp)
        if self.scan_min < self.closest:
            self.closest = self.scan_min
        if self.scan_min < self.cth:
            self.contacts += 1
        kp = fa = oa = 0.0
        if self.grid is not None:
            g, res = self.grid
            tot = g.size
            a = res * res
            kp = round(100.0 * (tot - int((g < 0).sum())) / tot, 2)
            fa = round(int(((g >= 0) & (g <= 25)).sum()) * a, 1)
            oa = round(int((g > 65).sum()) * a, 1)
        self.w.writerow([round(time.time() - self.t0, 1), round(x, 3), round(y, 3),
                         round(math.degrees(yaw), 1), round(self.dist, 2),
                         round(sp, 2), round(self.scan_min, 2), kp, fa, oa])
        self.f.flush()
        self.rows += 1
        if self.rows % 10 == 0:
            try:
                self.summarize(verbose=False)
            except Exception:
                pass

    def summarize(self, verbose=True):
        sp = [s for s in self.speeds if s > 0.01]
        kp = fa = oa = 0.0
        if self.grid is not None:
            g, res = self.grid
            tot = g.size; a = res * res
            kp = round(100.0 * (tot - int((g < 0).sum())) / tot, 2)
            fa = round(int(((g >= 0) & (g <= 25)).sum()) * a, 1)
            oa = round(int((g > 65).sum()) * a, 1)
        s = {'duration_s': round(time.time() - self.t0, 1),
             'path_length_m': round(self.dist, 2),
             'mean_speed_mps': round(float(np.mean(sp)), 2) if sp else 0.0,
             'max_speed_mps': round(float(np.max(sp)), 2) if sp else 0.0,
             'closest_obstacle_m': round(self.closest, 2),
             'contact_samples': self.contacts,
             'collision_threshold_m': self.cth,
             'final_known_pct': kp, 'free_area_m2': fa, 'occupied_area_m2': oa,
             'samples': self.rows}
        with open(os.path.join(self.out, 'recorder_summary.json'), 'w') as f:
            json.dump(s, f, indent=2)
        if verbose:
            print('\n=== RUN SUMMARY ===')
            for k, v in s.items():
                print('%-24s %s' % (k, v))
        return s


def main():
    rclpy.init()
    n = Recorder()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.summarize(); n.f.close()
        except Exception as e:
            print('summary failed:', e)
        try:
            n.destroy_node(); rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
