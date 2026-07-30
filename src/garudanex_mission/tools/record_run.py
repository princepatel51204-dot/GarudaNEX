#!/usr/bin/env python3
"""Log Gazebo ground truth against the SLAM estimate during a mission.

    python3 record_run.py runs/mission_01.csv

Ground truth comes from /gz/ground_truth (gz Pose_V bridged to TFMessage).
The gz->ROS conversion drops entity names, so transforms[0] is used: it is
base_link of the only dynamic model in the world. The count is asserted so
this fails loudly if another dynamic body is ever added.
"""
import os, sys, math, csv, rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

OUT = sys.argv[1] if len(sys.argv) > 1 else 'runs/mission.csv'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, path):
        super().__init__('garudanex_recorder')
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self.fh = open(path, 'w', newline='')
        self.w = csv.writer(self.fh)
        self.w.writerow(['t', 'gt_x', 'gt_y', 'gt_z', 'gt_yaw',
                         'est_x', 'est_y', 'est_yaw'])
        self.gt = None
        self.n = 0
        self.buf = Buffer()
        TransformListener(self.buf, self, spin_thread=True)
        self.create_subscription(TFMessage, '/gz/ground_truth', self._on_gt, 10)
        self.create_timer(0.1, self._sample)
        self.get_logger().info('recording to %s (Ctrl-C to stop)' % path)

    def _on_gt(self, msg):
        if msg.transforms:
            self.gt = msg.transforms[0].transform

    def _sample(self):
        if self.gt is None:
            return
        try:
            tf = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return
        t = self.get_clock().now().nanoseconds / 1e9
        g, e = self.gt, tf.transform
        self.w.writerow(['%.4f' % t,
                         '%.4f' % g.translation.x, '%.4f' % g.translation.y,
                         '%.4f' % g.translation.z, '%.5f' % yaw_of(g.rotation),
                         '%.4f' % e.translation.x, '%.4f' % e.translation.y,
                         '%.5f' % yaw_of(e.rotation)])
        self.fh.flush()
        self.n += 1
        if self.n % 100 == 0:
            self.get_logger().info('%d samples' % self.n)

    def close(self):
        self.fh.close()
        self.get_logger().info('wrote %d samples' % self.n)


def main():
    rclpy.init()
    node = Recorder(OUT)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()


if __name__ == '__main__':
    main()
