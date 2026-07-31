#!/usr/bin/env python3
"""GarudaNEX waypoint inspection mission with failsafes.

Sequences a YAML route through Nav2's navigate_to_pose action, and aborts the
mission when the stack stops being trustworthy rather than flying blind:
  - SLAM transform staleness (map -> base_footprint older than a limit)
  - loss of PX4 offboard / disarm
  - per-waypoint goal failure, with retries
"""
import math, time, threading, yaml

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from px4_msgs.msg import VehicleStatus

ARMED, OFFBOARD = 2, 14


def px4_qos():
    q = QoSProfile(depth=1)
    q.reliability = QoSReliabilityPolicy.BEST_EFFORT
    q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    q.history = QoSHistoryPolicy.KEEP_LAST
    return q


class MissionExecutor(Node):
    def __init__(self):
        super().__init__('garudanex_mission_executor')
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('goal_timeout', 120.0)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('hold_seconds', 3.0)
        self.declare_parameter('slam_stale_limit', 3.0)
        self.declare_parameter('require_offboard', True)
        self.declare_parameter('return_home', True)

        self.wp_file = self.get_parameter('waypoint_file').value
        self.timeout = self.get_parameter('goal_timeout').value
        self.retries = self.get_parameter('max_retries').value
        self.hold = self.get_parameter('hold_seconds').value
        self.stale_limit = self.get_parameter('slam_stale_limit').value
        self.need_ob = self.get_parameter('require_offboard').value
        self.rth = self.get_parameter('return_home').value

        self.nav_state = None
        self.arm_state = None
        self.abort_flag = False
        self.state = 'IDLE'

        self.buf = Buffer()
        TransformListener(self.buf, self, spin_thread=True)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/garudanex/mission_status', 10)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self._on_status, px4_qos())
        self.create_service(Trigger, '/garudanex/abort_mission', self._on_abort)
        self.create_timer(1.0, self._tick)

    def _on_status(self, msg):
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state

    def _on_abort(self, req, resp):
        self.abort_flag = True
        resp.success = True
        resp.message = 'abort requested'
        self.get_logger().warn('ABORT requested by operator')
        return resp

    def _tick(self):
        m = String()
        m.data = self.state
        self.status_pub.publish(m)

    def slam_age(self):
        try:
            tf = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            stamp = rclpy.time.Time.from_msg(tf.header.stamp)
            return (self.get_clock().now() - stamp).nanoseconds / 1e9
        except Exception:
            return float('inf')

    def healthy(self):
        if self.abort_flag:
            return False, 'operator abort'
        age = self.slam_age()
        if age > self.stale_limit:
            return False, 'SLAM transform stale (%.1f s)' % age
        if self.need_ob:
            if self.arm_state != ARMED:
                return False, 'not armed (arming_state=%s)' % self.arm_state
            if self.nav_state != OFFBOARD:
                return False, 'not offboard (nav_state=%s)' % self.nav_state
        return True, ''

    def goto(self, wp):
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = rclpy.time.Time().to_msg()  # 0 = use latest TF, not an exact instant
        p.pose.position.x = float(wp['x'])
        p.pose.position.y = float(wp['y'])
        yaw = float(wp.get('yaw', 0.0))
        p.pose.orientation.z = math.sin(yaw * 0.5)
        p.pose.orientation.w = math.cos(yaw * 0.5)
        goal.pose = p

        t0 = time.time()
        fut = self.nav.send_goal_async(goal)
        while not fut.done() and time.time() - t0 < 10.0:
            time.sleep(0.05)
        if not fut.done():
            return False, 'goal send timed out'
        handle = fut.result()
        if not handle.accepted:
            return False, 'goal rejected by bt_navigator'

        res = handle.get_result_async()
        while not res.done():
            if time.time() - t0 > self.timeout:
                handle.cancel_goal_async()
                return False, 'goal timeout after %.0f s' % self.timeout
            ok, why = self.healthy()
            if not ok:
                handle.cancel_goal_async()
                return False, why
            time.sleep(0.2)
        r = res.result()
        # Nav2 returns error_code 0 even on ABORTED - the action status
        # is the only reliable success signal.
        ok = (r.status == GoalStatus.STATUS_SUCCEEDED)
        return ok, 'status=%d error_code=%d' % (r.status, r.result.error_code)

    def run(self):
        wps = (yaml.safe_load(open(self.wp_file)) or {}).get('waypoints', [])
        self.get_logger().info('loaded %d waypoints from %s' % (len(wps), self.wp_file))
        if not wps:
            self.state = 'FAILED no waypoints'
            return

        self.state = 'WAIT_STACK'
        if not self.nav.wait_for_server(timeout_sec=60.0):
            self.get_logger().error('navigate_to_pose server unavailable - is Nav2 up?')
            self.state = 'FAILED no nav2'
            return
        for _ in range(120):
            ok, why = self.healthy()
            if ok:
                break
            self.get_logger().warn('waiting for healthy stack: %s' % why)
            time.sleep(1.0)

        done = 0
        for i, wp in enumerate(wps):
            ok = False
            for attempt in range(self.retries + 1):
                self.state = 'NAV %d/%d %s (try %d)' % (i + 1, len(wps), wp['name'], attempt + 1)
                self.get_logger().info(self.state)
                ok, why = self.goto(wp)
                if ok:
                    break
                self.get_logger().warn('%s failed: %s' % (wp['name'], why))
                if self.abort_flag:
                    self.state = 'ABORTED'
                    return
            if ok:
                done += 1
                self.state = 'HOLD %s' % wp['name']
                self.get_logger().info('reached %s, holding %.1f s' % (wp['name'], self.hold))
                time.sleep(self.hold)
            else:
                self.get_logger().error('skipping %s' % wp['name'])

        if self.rth:
            self.state = 'RETURN_HOME'
            self.get_logger().info(self.state)
            self.goto(wps[0])
        self.state = 'COMPLETE %d/%d' % (done, len(wps))
        self.get_logger().info(self.state)


def main():
    rclpy.init()
    node = MissionExecutor()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(2.0)
    try:
        node.run()
        time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
