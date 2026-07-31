#!/usr/bin/env python3
"""Frontier-based autonomous exploration for GarudaNEX.

A frontier is a free cell adjacent to unknown space: the boundary of what the
robot knows. Clustering those cells and driving to the best cluster is what
turns "follows waypoints" into "explores an unknown building".

Selection is utility = area - lambda * travel_distance, so large nearby
unknown regions win over small distant ones. Clusters that fail twice are
blacklisted, which is what stops the classic frontier-exploration livelock
where an unreachable gap is re-selected forever.

Stack health is a precondition for every goal, re-checked mid-flight:
SLAM transform staleness and PX4 offboard loss both abort the goal.
"""
import math, time, threading
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
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


def map_qos():
    q = QoSProfile(depth=1)
    q.reliability = QoSReliabilityPolicy.RELIABLE
    q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    q.history = QoSHistoryPolicy.KEEP_LAST
    return q


def dilate(a):
    o = a.copy()
    o[1:, :] |= a[:-1, :]
    o[:-1, :] |= a[1:, :]
    o[:, 1:] |= a[:, :-1]
    o[:, :-1] |= a[:, 1:]
    return o


def dilate_n(a, n):
    for _ in range(int(n)):
        a = dilate(a)
    return a


class Explorer(Node):
    def __init__(self):
        super().__init__('garudanex_explorer')
        self.declare_parameter('min_frontier_cells', 25)
        self.declare_parameter('clearance_m', 0.65)
        self.declare_parameter('distance_weight', 0.5)
        self.declare_parameter('area_weight', 0.010)
        self.declare_parameter('goal_timeout', 90.0)
        self.declare_parameter('max_failures', 2)
        self.declare_parameter('blacklist_radius', 1.5)
        self.declare_parameter('slam_stale_limit', 3.0)
        self.declare_parameter('dry_runs_to_finish', 3)
        self.declare_parameter('return_home', True)

        g = lambda k: self.get_parameter(k).value
        self.min_cells = g('min_frontier_cells')
        self.clearance = g('clearance_m')
        self.wdist = g('distance_weight')
        self.warea = g('area_weight')
        self.timeout = g('goal_timeout')
        self.max_fail = g('max_failures')
        self.bl_radius = g('blacklist_radius')
        self.stale_limit = g('slam_stale_limit')
        self.dry_target = g('dry_runs_to_finish')
        self.rth = g('return_home')

        self.grid = None
        self.nav_state = None
        self.arm_state = None
        self.abort = False
        self.state = 'IDLE'
        self.blacklist = []
        self.visited = 0
        self.home = None

        self.buf = Buffer()
        TransformListener(self.buf, self, spin_thread=True)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos())
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self._on_status, px4_qos())
        self.status_pub = self.create_publisher(String, '/garudanex/exploration_status', 10)
        self.mark_pub = self.create_publisher(MarkerArray, '/garudanex/frontiers', 10)
        self.create_service(Trigger, '/garudanex/abort_exploration', self._on_abort)
        self.create_timer(1.0, lambda: self.status_pub.publish(String(data=self.state)))

    def _on_map(self, msg):
        self.grid = msg

    def _on_status(self, msg):
        self.nav_state, self.arm_state = msg.nav_state, msg.arming_state

    def _on_abort(self, req, resp):
        self.abort = True
        resp.success, resp.message = True, 'abort requested'
        return resp

    def pose(self):
        try:
            tf = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            t = tf.transform.translation
            return np.array([t.x, t.y]), tf
        except Exception:
            return None, None

    def healthy(self):
        if self.abort:
            return False, 'operator abort'
        p, tf = self.pose()
        if p is None:
            return False, 'no map -> base_footprint'
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
        if age > self.stale_limit:
            return False, 'SLAM stale (%.1f s)' % age
        if self.arm_state != ARMED:
            return False, 'not armed (%s)' % self.arm_state
        if self.nav_state != OFFBOARD:
            return False, 'not offboard (%s)' % self.nav_state
        return True, ''

    def frontiers(self):
        """Cluster frontier cells and return [(world_xy, n_cells), ...]."""
        m = self.grid
        if m is None:
            return []
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        g = np.asarray(m.data, dtype=np.int16).reshape(h, w)

        free = (g >= 0) & (g < 25)
        unknown = g < 0
        blocked = dilate_n(g > 65, max(1, round(self.clearance / res)))
        front = free & dilate(unknown) & ~blocked

        idx = np.argwhere(front)
        if idx.size == 0:
            return []
        seen = np.zeros_like(front)
        out = []
        for r0, c0 in idx:
            if seen[r0, c0]:
                continue
            q, cells = deque([(r0, c0)]), []
            seen[r0, c0] = True
            while q:
                r, c = q.popleft()
                cells.append((r, c))
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1),
                               (1, 1), (1, -1), (-1, 1), (-1, -1)):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and front[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        q.append((rr, cc))
            if len(cells) < self.min_cells:
                continue
            arr = np.array(cells)
            cen = arr.mean(0)
            k = int(np.argmin(np.linalg.norm(arr - cen, axis=1)))
            r, c = arr[k]
            out.append((np.array([ox + (c + 0.5) * res, oy + (r + 0.5) * res]), len(cells)))
        return out

    def publish_markers(self, cands, chosen):
        ma = MarkerArray()
        d = Marker(); d.action = Marker.DELETEALL
        ma.markers.append(d)
        for i, (p, n) in enumerate(cands):
            mk = Marker()
            mk.header.frame_id = 'map'
            mk.ns, mk.id, mk.type, mk.action = 'frontiers', i, Marker.SPHERE, Marker.ADD
            mk.pose.position = Point(x=float(p[0]), y=float(p[1]), z=0.3)
            mk.pose.orientation.w = 1.0
            s = 0.35 + min(n, 400) / 400.0 * 0.65
            mk.scale.x = mk.scale.y = mk.scale.z = s
            hit = chosen is not None and np.allclose(p, chosen)
            mk.color = ColorRGBA(r=0.22, g=0.53, b=0.90, a=0.9) if hit else \
                       ColorRGBA(r=0.85, g=0.35, b=0.15, a=0.55)
            ma.markers.append(mk)
        self.mark_pub.publish(ma)

    def blacklisted(self, p):
        return any(np.linalg.norm(p - b) < self.bl_radius for b in self.blacklist)

    def goto(self, p):
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = rclpy.time.Time().to_msg()   # 0 = latest TF
        ps.pose.position.x, ps.pose.position.y = float(p[0]), float(p[1])
        ps.pose.orientation.w = 1.0
        goal.pose = ps

        t0 = time.time()
        fut = self.nav.send_goal_async(goal)
        while not fut.done() and time.time() - t0 < 10.0:
            time.sleep(0.05)
        if not fut.done():
            return False, 'send timeout'
        h = fut.result()
        if not h.accepted:
            return False, 'rejected'
        res = h.get_result_async()
        while not res.done():
            if time.time() - t0 > self.timeout:
                h.cancel_goal_async()
                return False, 'timeout'
            ok, why = self.healthy()
            if not ok:
                h.cancel_goal_async()
                return False, why
            time.sleep(0.2)
        code = res.result().result.error_code
        return code == 0, 'error_code=%d' % code

    def run(self):
        self.state = 'WAIT_STACK'
        if not self.nav.wait_for_server(timeout_sec=60.0):
            self.state = 'FAILED no nav2'
            self.get_logger().error('navigate_to_pose unavailable')
            return
        for _ in range(120):
            ok, why = self.healthy()
            if ok and self.grid is not None:
                break
            self.get_logger().warn('waiting: %s' % (why or 'no /map yet'))
            time.sleep(1.0)
        self.home, _ = self.pose()
        self.get_logger().info('home = (%.2f, %.2f)' % (self.home[0], self.home[1]))

        fails, dry = {}, 0
        while dry < self.dry_target and not self.abort:
            cands = [(p, n) for p, n in self.frontiers() if not self.blacklisted(p)]
            if not cands:
                dry += 1
                self.state = 'NO_FRONTIERS (%d/%d)' % (dry, self.dry_target)
                self.get_logger().info(self.state)
                self.publish_markers([], None)
                time.sleep(2.0)
                continue
            dry = 0
            here, _ = self.pose()
            best = max(cands, key=lambda c: c[1] * self.warea - self.wdist * np.linalg.norm(c[0] - here))
            self.publish_markers(cands, best[0])
            self.state = 'GOTO (%.1f, %.1f) | %d cells | %d frontiers | visited %d' % (
                best[0][0], best[0][1], best[1], len(cands), self.visited)
            self.get_logger().info(self.state)

            ok, why = self.goto(best[0])
            key = (round(best[0][0], 1), round(best[0][1], 1))
            if ok:
                self.visited += 1
                fails.pop(key, None)
            else:
                fails[key] = fails.get(key, 0) + 1
                self.get_logger().warn('frontier failed (%s), strike %d' % (why, fails[key]))
                if fails[key] >= self.max_fail:
                    self.blacklist.append(best[0])
                    self.get_logger().warn('blacklisted frontier %s (%d total)'
                                           % (key, len(self.blacklist)))

        if self.rth and self.home is not None and not self.abort:
            self.state = 'RETURN_HOME'
            self.get_logger().info(self.state)
            self.goto(self.home)
        self.state = 'EXPLORATION COMPLETE | %d frontiers visited | %d blacklisted' % (
            self.visited, len(self.blacklist))
        self.get_logger().info(self.state)


def main():
    rclpy.init()
    node = Explorer()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(2.0)
    try:
        node.run()
        time.sleep(3.0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
