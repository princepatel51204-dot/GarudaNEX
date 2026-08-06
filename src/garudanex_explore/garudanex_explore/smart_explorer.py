#!/usr/bin/env python3
"""GarudaNEX information-gain frontier explorer.

Over greedy nearest-frontier this adds:
  * expected information gain via an integral image over the unknown mask
  * true traversal cost from Nav2 ComputePathToPose (not Euclidean)
  * heading-continuity penalty  -> smooth sweeps instead of zig-zag
  * zone locking                -> finish a region before leaving it
  * goal commitment hysteresis  -> no target thrashing
  * early re-plan when the target's gain collapses (area already seen)
"""
import math, time, threading, os, json, subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose, ComputePathToPose
import tf2_ros


def integral(mask):
    return np.pad(mask.astype(np.int32), ((1, 0), (1, 0))).cumsum(0).cumsum(1)


def boxsum(I, r0, c0, r1, c1):
    return I[r1 + 1, c1 + 1] - I[r0, c1 + 1] - I[r1 + 1, c0] + I[r0, c0]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class SmartExplorer(Node):
    def __init__(self):
        super().__init__('smart_explorer')
        d = self.declare_parameter
        d('sensor_range', 8.0);        d('w_gain', 1.0)
        d('w_path', 0.55);             d('w_turn', 0.35)
        d('zone_size', 12.0);          d('zone_bonus', 0.30)
        d('hysteresis', 1.20);         d('top_k', 8)
        d('clearance_m', 0.50);        d('min_frontier_cells', 12)
        d('min_goal_distance', 1.2);   d('goal_timeout', 75.0)
        d('max_failures', 2);          d('blacklist_radius', 1.5)
        d('dry_runs_to_finish', 5);    d('gain_collapse', 0.35)
        d('bootstrap_secs', 4.0);      d('plan_timeout', 4.0)
        d('results_dir', '~/GarudaNEX/results/latest')
        d('unresolved_ratio', 0.75)
        d('min_new_cells', 1000)
        d('memory_zone_m', 4.0)
        d('zone_done_unknown', 0.12)
        d('max_goal_distance', 25.0)
        d('stuck_timeout', 20.0)
        d('stuck_move_m', 0.30)
        d('cruise_alt', 1.5)
        d('climb_speed', 0.6)
        d('land_on_finish', False)
        g = lambda k: self.get_parameter(k).value
        self.R = g('sensor_range');    self.wg = g('w_gain')
        self.wp = g('w_path');         self.wt = g('w_turn')
        self.zs = g('zone_size');      self.zb = g('zone_bonus')
        self.hyst = g('hysteresis');   self.topk = int(g('top_k'))
        self.clear = g('clearance_m'); self.minc = int(g('min_frontier_cells'))
        self.mind = g('min_goal_distance'); self.tmo = g('goal_timeout')
        self.maxf = int(g('max_failures')); self.bl_r = g('blacklist_radius')
        self.dry_n = int(g('dry_runs_to_finish')); self.collapse = g('gain_collapse')
        self.boot = g('bootstrap_secs'); self.ptmo = g('plan_timeout')

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.grid = None; self.lock = threading.Lock(); self.map_t = 0.0
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.viz = self.create_publisher(MarkerArray, '/garudanex/frontiers', 1)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.plan = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        self.black = []; self.visited = 0; self.zone = None
        self.fails = 0; self.replans = 0; self.home = None; self.t0 = time.time()
        self.stuck = 0; self.recent = []
        self.ur = self.get_parameter('unresolved_ratio').value
        self.minnew = int(self.get_parameter('min_new_cells').value)
        self.mz = self.get_parameter('memory_zone_m').value
        self.zdone = self.get_parameter('zone_done_unknown').value
        self.maxd = self.get_parameter('max_goal_distance').value
        self.stuck_t = self.get_parameter('stuck_timeout').value
        self.stuck_m = self.get_parameter('stuck_move_m').value
        self.stucks = 0
        self.done_zones = set(); self.visit_count = {}
        self.alt_ref = self.get_parameter('cruise_alt').value
        self.climb = self.get_parameter('climb_speed').value
        self.do_land = self.get_parameter('land_on_finish').value
        self.last_heading = 0.0

    # ---------------- map ----------------
    def on_map(self, m):
        h, w = m.info.height, m.info.width
        a = np.asarray(m.data, dtype=np.int8).reshape(h, w)
        with self.lock:
            self.grid = a
            self.res = m.info.resolution
            self.ox = m.info.origin.position.x
            self.oy = m.info.origin.position.y
            self.map_t = time.time()

    def snap(self):
        with self.lock:
            if self.grid is None:
                return None
            return self.grid.copy(), self.res, self.ox, self.oy

    def w2c(self, x, y):
        return int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def c2w(self, r, c):
        return self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res

    # ---------------- pose ----------------
    def pose(self):
        for frame in ('base_footprint', 'base_link'):
            try:
                t = self.tfbuf.lookup_transform('map', frame, rclpy.time.Time())
            except Exception:
                continue
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            return np.array([t.transform.translation.x,
                             t.transform.translation.y]), yaw
        return None, None

    # ---------------- frontiers ----------------
    def frontiers(self):
        s = self.snap()
        if s is None:
            return [], None
        g, res, _, _ = s
        free = (g >= 0) & (g <= 25)
        occ = g > 65
        unk = g < 0
        nb = np.zeros_like(unk)
        for ax, sh in ((0, 1), (0, -1), (1, 1), (1, -1)):
            nb |= np.roll(unk, sh, axis=ax)
        cand = free & nb
        if not cand.any():
            return [], (g, res)
        # clearance test: no occupied cell inside a box of clearance_m
        Iocc = integral(occ)
        H, W = g.shape
        k = max(1, int(self.clear / res))
        rr, cc = np.nonzero(cand)
        r0 = np.clip(rr - k, 0, H - 1); r1 = np.clip(rr + k, 0, H - 1)
        c0 = np.clip(cc - k, 0, W - 1); c1 = np.clip(cc + k, 0, W - 1)
        keep = boxsum(Iocc, r0, c0, r1, c1) == 0
        rr, cc = rr[keep], cc[keep]
        if rr.size == 0:
            return [], (g, res)
        # 8-connected clustering
        alive = set(zip(rr.tolist(), cc.tolist()))
        clusters = []
        while alive:
            seed = alive.pop()
            stack = [seed]; comp = [seed]
            while stack:
                r, c = stack.pop()
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        n = (r + dr, c + dc)
                        if n in alive:
                            alive.discard(n); stack.append(n); comp.append(n)
            if len(comp) >= self.minc:
                clusters.append(comp)
        # info gain per cluster centroid, via integral image on unknown
        Iunk = integral(unk)
        kr = max(1, int(self.R / res))
        out = []
        for comp in clusters:
            arr = np.array(comp)
            r, c = int(arr[:, 0].mean()), int(arr[:, 1].mean())
            gr0, gr1 = max(0, r - kr), min(H - 1, r + kr)
            gc0, gc1 = max(0, c - kr), min(W - 1, c + kr)
            gain = float(boxsum(Iunk, gr0, gc0, gr1, gc1))
            x, y = self.c2w(r, c)
            out.append({'p': np.array([x, y]), 'n': len(comp), 'gain': gain})
        return out, (g, res)

    def alt(self):
        try:
            t = self.tfbuf.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform.translation.z
        except Exception:
            return None

    def takeoff(self):
        a = self.alt()
        if a is not None and abs(a - self.alt_ref) <= 0.25:
            self.get_logger().info('at cruise altitude %.2f m' % a); return
        self.get_logger().info('takeoff -> %.2f m (now %s)'
                               % (self.alt_ref, 'unknown' if a is None else '%.2f' % a))
        t = Twist(); t.linear.z = self.climb
        t0 = time.time()
        while time.time() - t0 < 30.0:
            self.cmd.publish(t); time.sleep(0.05)
            a = self.alt()
            if a is not None and abs(a - self.alt_ref) <= 0.1:
                break
        self.cmd.publish(Twist()); time.sleep(1.5)
        self.get_logger().info('takeoff done, alt %s'
                               % ('unknown' if self.alt() is None else '%.2f' % self.alt()))

    def mzone(self, p):
        return (int(math.floor(p[0] / self.mz)), int(math.floor(p[1] / self.mz)))

    def zone_unknown_frac(self, key):
        st = self.snap()
        if st is None:
            return 1.0
        g, res, ox, oy = st
        H, W = g.shape
        x0, y0 = key[0] * self.mz, key[1] * self.mz
        c0 = max(0, int((x0 - ox) / res)); c1 = min(W, int((x0 + self.mz - ox) / res))
        r0 = max(0, int((y0 - oy) / res)); r1 = min(H, int((y0 + self.mz - oy) / res))
        if c1 <= c0 or r1 <= r0:
            return 1.0
        sub_g = g[r0:r1, c0:c1]
        return float((sub_g < 0).sum()) / sub_g.size

    def update_memory(self, here):
        k = self.mzone(here)
        self.visit_count[k] = self.visit_count.get(k, 0) + 1
        for kk in list(self.visit_count.keys()):
            if kk in self.done_zones:
                continue
            f = self.zone_unknown_frac(kk)
            if f < self.zdone:
                self.done_zones.add(kk)
                self.get_logger().info('  zone %s complete (%.0f%% unknown) - retired'
                                       % (str(kk), 100.0 * f))

    def known_cells(self):
        s = self.snap()
        return 0 if s is None else int((s[0] >= 0).sum())

    def gain_at(self, p):
        s = self.snap()
        if s is None:
            return 0.0
        g, res, _, _ = s
        H, W = g.shape
        I = integral(g < 0)
        r, c = self.w2c(p[0], p[1])
        k = max(1, int(self.R / res))
        return float(boxsum(I, max(0, r - k), max(0, c - k),
                            min(H - 1, r + k), min(W - 1, c + k)))

    # ---------------- action helpers ----------------
    def wait(self, fut, tmo):
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > tmo:
                return False
            time.sleep(0.05)
        return True

    def stamped(self, p, yaw=0.0):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = rclpy.time.Time().to_msg()
        ps.pose.position.x = float(p[0]); ps.pose.position.y = float(p[1])
        ps.pose.orientation.z = math.sin(yaw / 2)
        ps.pose.orientation.w = math.cos(yaw / 2)
        return ps

    def path_cost(self, p):
        g = ComputePathToPose.Goal()
        g.goal = self.stamped(p); g.use_start = False
        try:
            g.planner_id = 'GridBased'
        except Exception:
            pass
        f = self.plan.send_goal_async(g)
        if not self.wait(f, self.ptmo):
            return None
        gh = f.result()
        if gh is None or not gh.accepted:
            return None
        rf = gh.get_result_async()
        if not self.wait(rf, self.ptmo + 2.0):
            return None
        r = rf.result()
        if r is None or r.status != GoalStatus.STATUS_SUCCEEDED:
            return None
        pts = r.result.path.poses
        if len(pts) < 2:
            return None
        a = np.array([[q.pose.position.x, q.pose.position.y] for q in pts])
        return float(np.linalg.norm(np.diff(a, axis=0), axis=1).sum())

    def blacklisted(self, p):
        return any(np.linalg.norm(p - b) < self.bl_r for b in self.black)

    # ---------------- scoring ----------------
    def choose(self, here, yaw, current=None):
        fr, _ = self.frontiers()
        base = [f for f in fr
                if not self.blacklisted(f['p'])
                and np.linalg.norm(f['p'] - here) > self.mind
                and self.mzone(f['p']) not in self.done_zones]
        fr = [f for f in base if np.linalg.norm(f['p'] - here) < self.maxd]
        if not fr and base:
            hard = [f for f in base
                    if np.linalg.norm(f['p'] - here) < self.maxd * 1.6]
            if hard:
                self.get_logger().info('  no near frontier - relaxing cap to %.0fm'
                                       % (self.maxd * 1.6))
                fr = hard
        if not fr:
            return None, 0
        n_raw = len(fr)
        for f in fr:
            f['d'] = float(np.linalg.norm(f['p'] - here))
            f['pre'] = f['gain'] / (1.0 + f['d'])
        fr.sort(key=lambda f: -f['pre'])
        short = fr[:self.topk]
        for f in short:
            c = self.path_cost(f['p'])
            f['cost'] = c if c is not None else None
        short = [f for f in short if f['cost'] is not None]
        if not short:
            for f in fr[:2]:
                self.black.append(f['p'])
            self.get_logger().warn('  no plannable candidate - retired 2')
            return None, n_raw
        gmax = max(f['gain'] for f in short) or 1.0
        cmax = max(f['cost'] for f in short) or 1.0
        zr = self.zone_of(here)
        for f in short:
            v = f['p'] - here
            turn = abs(wrap(math.atan2(v[1], v[0]) - yaw)) / math.pi
            u = (self.wg * f['gain'] / gmax
                 - self.wp * f['cost'] / cmax
                 - self.wt * turn)
            if self.zone_of(f['p']) == zr:
                u += self.zb
            f['u'] = u
        short.sort(key=lambda f: -f['u'])
        best = short[0]
        if current is not None:
            for f in short:
                if np.linalg.norm(f['p'] - current) < self.bl_r:
                    if best['u'] < f['u'] * self.hyst:
                        best = f
                    break
        self.publish_viz(short, best['p'])
        return best, n_raw

    def zone_of(self, p):
        return (int(math.floor(p[0] / self.zs)), int(math.floor(p[1] / self.zs)))

    def publish_viz(self, cands, goal):
        ma = MarkerArray()
        m = Marker(); m.header.frame_id = 'map'; m.ns = 'frontiers'; m.id = 0
        m.type = Marker.SPHERE_LIST; m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.35
        m.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=0.9)
        for f in cands:
            m.points.append(Point(x=float(f['p'][0]), y=float(f['p'][1]), z=0.2))
        ma.markers.append(m)
        t = Marker(); t.header.frame_id = 'map'; t.ns = 'target'; t.id = 1
        t.type = Marker.SPHERE; t.action = Marker.ADD
        t.scale.x = t.scale.y = t.scale.z = 0.7
        t.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.95)
        t.pose.position.x = float(goal[0]); t.pose.position.y = float(goal[1])
        t.pose.position.z = 0.3; t.pose.orientation.w = 1.0
        ma.markers.append(t)
        self.viz.publish(ma)

    # ---------------- navigation ----------------
    def drive(self, target):
        g0 = max(1.0, target['gain'])
        yaw = math.atan2(target['p'][1], target['p'][0])
        goal = NavigateToPose.Goal()
        goal.pose = self.stamped(target['p'], yaw)
        f = self.nav.send_goal_async(goal)
        if not self.wait(f, 8.0):
            return 'timeout'
        gh = f.result()
        if gh is None or not gh.accepted:
            return 'rejected'
        rf = gh.get_result_async()
        t0 = time.time()
        anchor, _ = self.pose()
        anchor_t = t0
        while not rf.done():
            if time.time() - t0 > self.tmo:
                gh.cancel_goal_async(); time.sleep(1.0); return 'timeout'
            pn, _ = self.pose()
            if pn is not None:
                if anchor is None or np.linalg.norm(pn - anchor) > self.stuck_m:
                    anchor = pn; anchor_t = time.time()
                elif time.time() - anchor_t > self.stuck_t:
                    gh.cancel_goal_async(); time.sleep(0.5)
                    self.escape()
                    return 'stuck'
            if time.time() - t0 > 4.0 and self.gain_at(target['p']) < self.collapse * g0:
                gh.cancel_goal_async(); time.sleep(0.6)
                self.get_logger().info('  gain collapsed -> early replan')
                return 'seen'
            time.sleep(0.4)
        r = rf.result()
        if r is None:
            return 'lost'
        return 'ok' if r.status == GoalStatus.STATUS_SUCCEEDED else 'fail:%d' % r.status

    # ---------------- main ----------------
    def run(self):
        self.get_logger().info('waiting for /map, TF and Nav2 ...')
        self.nav.wait_for_server()
        self.plan.wait_for_server()
        while rclpy.ok() and self.snap() is None:
            time.sleep(0.5)
        self.takeoff()
        t = Twist(); t.linear.x = 0.5
        self.get_logger().info('bootstrap: seeding SLAM with %.0f s of motion' % self.boot)
        t0 = time.time()
        while time.time() - t0 < self.boot:
            self.cmd.publish(t); time.sleep(0.05)
        self.cmd.publish(Twist()); time.sleep(2.0)
        self.t0 = time.time()
        self.home, _ = self.pose()

        dry = 0; strikes = 0; current = None
        while rclpy.ok():
            here, yaw = self.pose()
            if here is None:
                time.sleep(0.5); continue
            self.update_memory(here)
            best, nraw = self.choose(here, yaw, current)
            if best is None:
                dry += 1
                self.get_logger().warn('no reachable frontier (dry %d/%d)' % (dry, self.dry_n))
                if dry >= self.dry_n:
                    break
                time.sleep(3.0); continue
            dry = 0
            self.get_logger().info(
                'GOTO (%.1f, %.1f) | gain %d | path %.1fm | u %.2f | %d frontiers | visited %d'
                % (best['p'][0], best['p'][1], int(best['gain']),
                   best['cost'], best['u'], nraw, self.visited))
            current = best['p']
            k0 = self.known_cells()
            res = self.drive(best)
            gained = self.known_cells() - k0
            if res == 'ok':
                self.visited += 1; strikes = 0; current = None
                area = gained * (self.res ** 2)
                self.get_logger().info('  arrived | +%d cells (%.1f m2)' % (gained, area))
                if gained < self.minnew:
                    self.black.append(best['p']); self.stuck += 1
                    self.get_logger().warn('  no new information -> blacklisted (%d)'
                                           % len(self.black))
                self.recent.append(tuple(np.round(best['p'], 0)))
                self.recent = self.recent[-6:]
                for t in set(self.recent):
                    if self.recent.count(t) >= 3:
                        self.black.append(np.array(t))
                        self.get_logger().warn('  oscillation on %s -> blacklisted' % (t,))
                        self.recent = []
                        break
            elif res == 'seen':
                self.replans += 1
                strikes = 0; current = None
            else:
                if res == 'stuck':
                    self.black.append(best['p']); strikes = 0; current = None
                    self.get_logger().warn('  stuck point blacklisted (%d)'
                                           % len(self.black))
                    continue
                strikes += 1; self.fails += 1
                self.get_logger().warn('  goal failed (%s), strike %d' % (res, strikes))
                if strikes >= self.maxf:
                    self.black.append(best['p']); strikes = 0; current = None
                    self.get_logger().warn('  blacklisted (%d total)' % len(self.black))
        self.get_logger().info('EXPLORATION COMPLETE | visited %d | blacklisted %d'
                               % (self.visited, len(self.black)))
        self.finish()

    def finish(self):
        el = time.time() - self.t0
        out = os.path.expanduser(self.get_parameter('results_dir').value)
        os.makedirs(out, exist_ok=True)
        cov = {}
        s = self.snap()
        if s is not None:
            g, res, _, _ = s
            tot = int(g.size)
            unk = int((g < 0).sum())
            free = int(((g >= 0) & (g <= 25)).sum())
            occ = int((g > 65).sum())
            a = res * res
            cov = {'known_pct': round(100.0 * (tot - unk) / tot, 2),
                   'free_area_m2': round(free * a, 1),
                   'occupied_area_m2': round(occ * a, 1),
                   'map_w_m': round(g.shape[1] * res, 1),
                   'map_h_m': round(g.shape[0] * res, 1)}
        try:
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                            '-f', os.path.join(out, 'map'),
                            '--ros-args', '-p', 'save_map_timeout:=20.0'], timeout=60)
        except Exception as e:
            self.get_logger().warn('map save failed: %s' % e)
        summ = {'goals_reached': self.visited, 'goals_failed': self.fails,
                'early_replans': self.replans, 'blacklisted': len(self.black),
                'duration_s': round(el, 1), 'unresolved_frontiers': self.stuck,
                'stuck_recoveries': self.stucks,
                'zones_completed': len(self.done_zones),
                'coverage': cov}
        with open(os.path.join(out, 'explorer_summary.json'), 'w') as f:
            json.dump(summ, f, indent=2)
        self.get_logger().info('SUMMARY ' + json.dumps(summ))
        if self.home is not None:
            self.get_logger().info('returning home (%.1f, %.1f)'
                                   % (self.home[0], self.home[1]))
            self.drive({'p': self.home, 'gain': 1e9})
        if self.do_land:
            self.land()
        else:
            self.get_logger().info('holding altitude (land_on_finish=false)')

    def escape(self):
        self.get_logger().warn('  STUCK - escape maneuver')
        self.stucks += 1
        t = Twist(); t.linear.x = -0.5; t.linear.z = 0.25
        t0 = time.time()
        while time.time() - t0 < 2.0:
            self.cmd.publish(t); time.sleep(0.05)
        t2 = Twist(); t2.angular.z = 0.9
        t0 = time.time()
        while time.time() - t0 < 1.5:
            self.cmd.publish(t2); time.sleep(0.05)
        self.cmd.publish(Twist()); time.sleep(1.0)

    def land(self):
        self.get_logger().info('landing')
        t = Twist(); t.linear.z = -0.4
        t0 = time.time()
        while time.time() - t0 < 6.0:
            self.cmd.publish(t); time.sleep(0.05)
        self.cmd.publish(Twist())


def main():
    rclpy.init()
    n = SmartExplorer()
    th = threading.Thread(target=rclpy.spin, args=(n,), daemon=True)
    th.start()
    try:
        n.run()
    except KeyboardInterrupt:
        n.get_logger().info('interrupted - writing results')
        try:
            n.finish()
        except Exception as e:
            print('finish failed:', e)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
        th.join(timeout=2.0)
        try:
            n.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
