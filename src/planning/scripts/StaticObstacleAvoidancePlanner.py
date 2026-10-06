#!/usr/bin/env python3
"""LiDAR local planning using the existing /global_path and /local_path interfaces."""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import math
import threading
from collections import deque
import time
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int16
from visualization_msgs.msg import Marker, MarkerArray
from lidar_object_detection.msg import ObjectInfo
from simul_msgs.msg import LocalPlan
from local_planning_core import FrenetLocalPlanner, in_zones, validate_zones
from local_planning_state import ObstacleMemory, input_fresh, return_progress, approach_zone_distance
from utils import make_pose_stamped


class StaticObstacleAvoidancePlanner:
    def __init__(self):
        self.p = rospy.get_param('~')
        for key in ('static_zones', 'dynamic_zones'):
            try:
                validate_zones(self.p[key])
            except ValueError as exc:
                rospy.logerr('[LOCAL CONFIG] %s: %s', key, str(exc))
                raise
        self.core = FrenetLocalPlanner(self.p)
        if self.p['dynamic_policy'] not in ('stop_on_obstacle', 'observe_only'):
            raise ValueError('unknown dynamic_policy')
        for key in ('update_hz', 'input_timeout', 'static_speed_kmh', 'dynamic_speed_kmh',
                    'max_lateral_accel', 'obstacle_memory_s', 'return_clear_time',
                    'obstacle_merge_distance', 'global_path_timeout', 'approach_speed_kmh',
                    'return_d_tolerance', 'return_heading_tolerance'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(key + ' must be positive and finite')
        if not math.isfinite(self.p['approach_distance']) or self.p['approach_distance'] < 0:
            raise ValueError('approach_distance must be finite and nonnegative')
        rospy.loginfo('[LOCAL CONFIG] width=%.2f front=%.2f rear=%.2f wheelbase=%.2f m; road_half_width=%.2f; approach=%.1f m',
                      self.p['vehicle_width'], self.p['vehicle_front'], self.p['vehicle_rear'],
                      self.p['wheelbase'], self.p['road_half_width'], self.p['approach_distance'])
        if not self.p['static_zones'] and not self.p['dynamic_zones']:
            rospy.logwarn('[LOCAL CONFIG] mission zones empty: global tracking preserved; no obstacle mission enabled')
        self.lock = threading.RLock()
        self.listener = tf.TransformListener()
        self.global_path = None
        self.points = None
        self.path_received = None
        self.path_error = 'global_path unavailable'
        self.pose = None
        self.pose_received = None
        self.curr_idx = None
        self.index_received = None
        self.pending_scans = deque(maxlen=30)
        self.obstacle_msg = None
        self.obstacle_received = None
        self.obstacle_stamp = None
        self.memory = ObstacleMemory(self.p['obstacle_memory_s'], self.p['obstacle_merge_distance'])
        self.avoidance_active = False
        self.clear_since = None
        self.last_state = None
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.local_path_done_pub = rospy.Publisher('/local_path_done', Bool, queue_size=1)
        self.plan_pub = rospy.Publisher('/local_plan', LocalPlan, queue_size=1)
        self.candidate_pub = rospy.Publisher('~candidates', MarkerArray, queue_size=1)
        self.obstacle_pub = rospy.Publisher('~obstacles', MarkerArray, queue_size=1)
        self.state_pub = rospy.Publisher('~state_marker', Marker, queue_size=1)
        rospy.Subscriber('/global_path', Path, self.global_path_callback, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        rospy.Subscriber('/curr_idx', Int16, self.curr_idx_callback, queue_size=1)
        rospy.Subscriber('/obstacle_info_static', ObjectInfo, self.obstacle_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.p['update_hz']), self.tick)

    def global_path_callback(self, msg):
        with self.lock:
            self.path_received = time.monotonic()
            xy = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
            if msg.header.frame_id != 'map' or len(xy) < 4 or not np.isfinite(xy).all():
                self.points = None
                self.path_error = 'invalid global_path frame/points'
                return
            if any(z['end'] >= len(xy) for z in self.p['static_zones'] + self.p['dynamic_zones'] if 'end' in z):
                self.points = None
                self.path_error = 'zone waypoint range exceeds global_path'
                rospy.logwarn_throttle(2.0, '[LOCAL CONFIG] %s', self.path_error)
                return
            if self.points is not None and np.array_equal(xy, self.points):
                return
            self.global_path, self.points = msg, xy
            self.core.reset()
            self.clear_since = None

    def current_pose_callback(self, msg):
        with self.lock:
            self.pose, self.pose_received = msg, time.monotonic()

    def curr_idx_callback(self, msg):
        with self.lock:
            self.curr_idx, self.index_received = msg.data, time.monotonic()

    def obstacle_callback(self, msg):
        with self.lock:
            self.obstacle_msg, self.obstacle_received = msg, time.monotonic()
            self.pending_scans.append((msg, self.obstacle_received))

    def fresh(self, received, stamp=None):
        return input_fresh(received, time.monotonic(), self.p['input_timeout'],
                           stamp.to_sec() if stamp is not None else None, rospy.Time.now().to_sec())

    def transform_obstacles_to_global(self):
        msg = self.obstacle_msg
        if msg is None or not self.fresh(self.obstacle_received, msg.header.stamp):
            raise ValueError('LiDAR missing/stale')
        arrays = ('centerX', 'centerY', 'centerZ', 'lengthX', 'lengthY', 'lengthZ')
        if not msg.header.frame_id or not 0 <= msg.objectCounts <= min(len(getattr(msg, name)) for name in arrays):
            raise ValueError('invalid LiDAR frame/count (or cluster overflow)')
        latest_values = np.array([getattr(msg, name)[:msg.objectCounts] for name in arrays])
        if not np.isfinite(latest_values).all() or np.any(latest_values[3:] < 0):
            raise ValueError('invalid obstacle geometry')
        now = time.monotonic()
        self.memory.circles(now)
        # LiDAR can run faster than localization. Keep a small queue so an older,
        # still-fresh scan can be transformed after the next pose brackets its stamp.
        # Retrying only the newest scan would starve forever on future-TF errors.
        for candidate, received in reversed(self.pending_scans):
            if not self.fresh(received, candidate.header.stamp):
                continue
            if not candidate.header.frame_id or not 0 <= candidate.objectCounts <= min(len(getattr(candidate, name)) for name in arrays):
                continue
            key = (candidate.header.stamp.to_nsec(), candidate.header.frame_id)
            if self.obstacle_stamp == key:
                return self.memory.circles(now)
            try:
                translation, rotation = self.listener.lookupTransform(
                    'map', candidate.header.frame_id, candidate.header.stamp)
            except tf.Exception:
                continue
            msg = candidate
            observed_at = received
            break
        else:
            raise ValueError('LiDAR TF map <- %s unavailable at scan time' % self.obstacle_msg.header.frame_id)
        # Transform at scan acquisition time, never mix velodyne coordinates with map.
        if not np.isfinite(list(translation) + list(rotation)).all() or abs(np.linalg.norm(rotation) - 1) > .01:
            raise ValueError('invalid LiDAR TF translation/quaternion')
        matrix = tf.transformations.quaternion_matrix(rotation)
        matrix[:3, 3] = translation
        circles = []
        for i in range(msg.objectCounts):
            xyz = np.array([msg.centerX[i], msg.centerY[i], msg.centerZ[i], 1.0])
            dims = np.array([msg.lengthX[i], msg.lengthY[i], msg.lengthZ[i]])
            if not np.isfinite(xyz).all() or not np.isfinite(dims).all() or np.any(dims < 0):
                raise ValueError('invalid obstacle geometry')
            # Project all 8 transformed box corners to a conservative map-plane circle.
            corners = np.array([[sx, sy, sz] for sx in (-.5, .5)
                                for sy in (-.5, .5) for sz in (-.5, .5)]) * dims
            center = matrix.dot(xyz)
            offsets = corners.dot(matrix[:3, :3].T)
            radius = max(0.05, float(np.max(np.linalg.norm(offsets[:, :2], axis=1))))
            circles.append((float(center[0]), float(center[1]), radius))
        # Apply only after the entire scan validates; a bad box cannot partially update memory.
        result = self.memory.observe(circles, observed_at, now)
        self.obstacle_stamp = key
        return result

    def make_path(self, candidate):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        if candidate is not None:
            # Existing controller uses position.z as curvature, not altitude.
            path.poses = [make_pose_stamped(path.header.stamp, 'map', [x, y, abs(k)],
                          tf.transformations.quaternion_from_euler(0, 0, yaw))
                          for (x, y), yaw, k in zip(candidate.xy, candidate.yaw, candidate.curvature)]
        return path

    def publish(self, state, active=False, stop=False, candidate=None, candidates=(), obstacles=(), speed=0):
        path = self.make_path(candidate)
        plan = LocalPlan()
        plan.header = path.header
        plan.active, plan.stop, plan.state = active, stop, state
        plan.speed_limit_kmh, plan.path = speed, path
        self.plan_pub.publish(plan)
        self.local_path_pub.publish(path)
        self.local_path_done_pub.publish(Bool(data=not active))
        self.visualize(state, candidates, candidate, obstacles)
        detail = '%s active=%s stop=%s speed_limit=%.1f offset=%s cost=%s' % (
            state, active, stop, speed, 'none' if candidate is None else '%+.2f' % candidate.offset,
            'none' if candidate is None else '%.3f' % candidate.cost)
        phase = state.split(':', 1)[0]
        if phase != self.last_state:
            rospy.loginfo('[LOCAL PLANNER] %s', detail)
            self.last_state = phase
        else:
            rospy.loginfo_throttle(2.0, '[LOCAL PLANNER] %s', detail)

    def tick(self, _event):
        with self.lock:
            try:
                self.process()
            except (ValueError, IndexError, np.linalg.LinAlgError, tf.Exception) as exc:
                self.clear_since = None  # HOLD cannot count toward a continuous safe return.
                rospy.logwarn_throttle(1.0, '[LOCAL HOLD] reason=%s reference=%s', str(exc), str(self.core.reference_diagnostic))
                self.publish('HOLD: ' + str(exc), active=True, stop=True)

    def process(self):
        # Unconfigured mission zones leave the existing global controller untouched.
        if not self.p['static_zones'] and not self.p['dynamic_zones']:
            self.publish('NORMAL: zones not configured')
            return
        if self.pose is None or not self.fresh(self.pose_received, self.pose.header.stamp):
            raise ValueError('current_pose missing/stale')
        if self.pose.header.frame_id != 'map':
            raise ValueError('current_pose must be in map')
        pos, ori = self.pose.pose.position, self.pose.pose.orientation
        q = [ori.x, ori.y, ori.z, ori.w]
        if not np.isfinite([pos.x, pos.y] + q).all() or abs(np.linalg.norm(q) - 1) > .01:
            raise ValueError('invalid current pose')
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        if any('start' in z for z in self.p['static_zones'] + self.p['dynamic_zones']):
            if not self.fresh(self.index_received) or self.curr_idx is None or self.curr_idx < 0:
                raise ValueError('waypoint index missing/stale')
        if self.points is None:
            raise ValueError(self.path_error)
        if not input_fresh(self.path_received, time.monotonic(), self.p['global_path_timeout']):
            raise ValueError('global_path publisher stale')
        # Map rectangles also work before the index arrives; projection is then geometric.
        idx = self.curr_idx if self.fresh(self.index_received) else int(np.argmin(np.linalg.norm(self.points - [pos.x, pos.y], axis=1)))
        if idx is None or not 0 <= idx < len(self.points):
            raise ValueError('waypoint outside global path')
        static = in_zones(self.p['static_zones'], self.curr_idx, pos.x, pos.y)
        dynamic = in_zones(self.p['dynamic_zones'], self.curr_idx, pos.x, pos.y)
        if not static and not dynamic and not self.avoidance_active:
            self.core.reset()
            approaches = []
            for zones, limit, name in ((self.p['static_zones'], self.p['static_speed_kmh'], 'STATIC_OBSTACLE'),
                                       (self.p['dynamic_zones'], self.p['dynamic_speed_kmh'], 'DYNAMIC_OBSTACLE')):
                distance = approach_zone_distance(zones, idx, pos.x, pos.y, self.points, self.p['approach_distance'])
                if distance is not None:
                    approaches.append((min(self.p['approach_speed_kmh'], limit), distance, name))
            if approaches:
                speed, distance, name = min(approaches)
                self.publish('NORMAL: approach %s in %.1fm' % (name, distance), speed=speed)
            else:
                self.publish('NORMAL')
            return
        csp = self.core.reference(self.points, idx)
        diagnostic = self.core.reference_diagnostic
        rospy.loginfo_throttle(5.0, '[LOCAL REFERENCE] input=%d removed=%d peak_curvature=%.3f',
                              diagnostic['input_points'], diagnostic['removed_points'], diagnostic['peak_curvature'])
        obstacles = self.transform_obstacles_to_global()
        # Dynamic policy is independent of static lateral avoidance.
        if dynamic:
            self.process_dynamic(csp, (pos.x, pos.y, yaw), obstacles)
            return
        returning = not static
        best, candidates, (_, d, heading) = self.core.plan(csp, (pos.x, pos.y, yaw), obstacles, returning)
        if best is None:
            self.avoidance_active = True
            self.clear_since = None
            reasons = sorted({c.rejection for c in candidates})
            self.publish('STATIC_OBSTACLE: no valid candidate (' + ', '.join(reasons) + ')', True, True,
                         candidates=candidates, obstacles=obstacles)
            return
        # Prefer a newly validated centre return after avoidance. Hysteresis must
        # not keep a free-road lateral offset indefinitely after passing an object.
        if self.avoidance_active and not returning:
            centre = next((c for c in candidates if c.offset == 0 and not c.rejection), None)
            if centre is not None:
                best = centre
        self.core.previous = best
        peak_k = max(1e-4, float(np.max(np.abs(best.curvature))))
        speed = min(self.p['static_speed_kmh'], 3.6 * math.sqrt(self.p['max_lateral_accel'] / peak_k))
        # Once the centre candidate is selected, finish the return inside the zone too.
        returning = returning or (best.offset == 0 and (self.avoidance_active or
                    abs(d) > self.p['return_d_tolerance'] or abs(heading) > self.p['return_heading_tolerance']))
        if returning:
            complete, self.clear_since = return_progress(d, heading, time.monotonic(), self.clear_since, self.p)
            if complete:
                self.avoidance_active = False
                self.core.reset()
                self.clear_since = None
                self.publish('NORMAL: return complete', candidates=candidates, obstacles=obstacles,
                             speed=speed if static else 0)
                return
        else:
            self.clear_since = None
            if best.offset == 0 and not self.avoidance_active:
                self.publish('NORMAL: static corridor clear', candidates=candidates, obstacles=obstacles,
                             speed=speed)
                return
        self.avoidance_active = True
        self.publish('RETURN_TO_GLOBAL' if returning else 'STATIC_OBSTACLE', True, False,
                     best, candidates, obstacles, speed)

    def process_dynamic(self, csp, pose, obstacles):
        # No motion classification/tracking or time-based obstacle ignoring is implied.
        best, candidates, _ = self.core.plan(csp, pose, obstacles, returning=True)
        if self.p['dynamic_policy'] == 'observe_only' and not self.avoidance_active:
            self.publish('DYNAMIC_OBSTACLE: observe_only', candidates=candidates, obstacles=obstacles)
        elif best is None:
            self.publish('DYNAMIC_OBSTACLE: stop', True, True, candidates=candidates, obstacles=obstacles)
        else:
            self.core.previous = best
            self.avoidance_active = True
            self.publish('DYNAMIC_OBSTACLE: corridor_clear', True, False, best, candidates,
                         obstacles, self.p['dynamic_speed_kmh'])

    def marker(self, ns, ident, kind):
        m = Marker()
        m.header.frame_id, m.header.stamp = 'map', rospy.Time.now()
        m.ns, m.id, m.type, m.action = ns, ident, kind, Marker.ADD
        m.pose.orientation.w = 1
        m.lifetime = rospy.Duration(0.5)
        return m

    def visualize(self, state, candidates, selected, obstacles):
        markers = MarkerArray()
        clear = self.marker('clear', 0, Marker.LINE_STRIP)
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, c in enumerate(candidates):
            m = self.marker('candidates', i, Marker.LINE_STRIP)
            m.scale.x = .14 if c is selected else .055
            color = (1., .15, .15) if c.rejection else ((0., 1., .2) if c is selected else (.2, .6, 1.))
            m.color.r, m.color.g, m.color.b, m.color.a = (*color, .9)
            m.points = [Point(x=float(x), y=float(y), z=.1) for x, y in c.xy]
            markers.markers.append(m)
            label = self.marker('costs', i, Marker.TEXT_VIEW_FACING)
            label.pose.position.x, label.pose.position.y = c.xy[len(c.xy) // 2]
            label.pose.position.z = 1 + .15 * i
            label.scale.z, label.color.a = .35, 1
            label.color.r = label.color.g = label.color.b = 1
            label.text = '%s d=%+.1f %s cost=%.2f clearance=%.2f' % (
                'SELECTED' if c is selected else '', c.offset, c.rejection or 'OK', c.cost, c.clearance)
            markers.markers.append(label)
        self.candidate_pub.publish(markers)
        detected = MarkerArray()
        detected.markers.append(clear)
        for i, (x, y, radius) in enumerate(obstacles):
            m = self.marker('obstacles', i, Marker.CYLINDER)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, .5
            m.scale.x = m.scale.y = 2 * radius
            m.scale.z = 1
            m.color.r, m.color.g, m.color.a = 1, .5, .5
            detected.markers.append(m)
        self.obstacle_pub.publish(detected)
        if self.pose is not None and np.isfinite([self.pose.pose.position.x, self.pose.pose.position.y]).all():
            m = self.marker('state', 0, Marker.TEXT_VIEW_FACING)
            m.pose.position.x, m.pose.position.y = self.pose.pose.position.x, self.pose.pose.position.y
            m.pose.position.z, m.scale.z, m.color.a = 3., .5, 1.
            m.color.r = m.color.g = m.color.b = 1.
            m.text = state + ('\noffset=%+.2f cost=%.3f' % (selected.offset, selected.cost)
                             if selected is not None else '\nselected=none')
            self.state_pub.publish(m)


if __name__ == '__main__':
    rospy.init_node('static_obstacle_avoidance_planner')
    try:
        planner = StaticObstacleAvoidancePlanner()
    except (ValueError, TypeError, KeyError) as exc:
        rospy.logfatal('[LOCAL CONFIG] startup rejected: %s', str(exc))
        raise
    rospy.spin()
