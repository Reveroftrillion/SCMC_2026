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
from utils import make_pose_stamped


class StaticObstacleAvoidancePlanner:
    def __init__(self):
        self.p = rospy.get_param('~')
        self.core = FrenetLocalPlanner(self.p)
        for key in ('static_zones', 'dynamic_zones'):
            validate_zones(self.p[key])
        if self.p['dynamic_policy'] not in ('stop_on_obstacle', 'observe_only'):
            raise ValueError('unknown dynamic_policy')
        for key in ('update_hz', 'input_timeout', 'static_speed_kmh', 'dynamic_speed_kmh',
                    'max_lateral_accel', 'obstacle_memory_s', 'return_clear_time'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(key + ' must be positive and finite')
        self.lock = threading.RLock()
        self.listener = tf.TransformListener()
        self.global_path = None
        self.points = None
        self.pose = None
        self.pose_received = None
        self.curr_idx = None
        self.index_received = None
        self.pending_scans = deque(maxlen=30)
        self.obstacle_msg = None
        self.obstacle_received = None
        self.obstacle_stamp = None
        self.obstacles = []  # world circles x/y/radius + last observation monotonic time
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
            xy = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
            if msg.header.frame_id != 'map' or len(xy) < 4 or not np.isfinite(xy).all():
                self.points = None
                return
            if self.points is not None and np.array_equal(xy, self.points):
                return
            self.global_path, self.points = msg, xy
            self.core.reset()

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
        if received is None or time.monotonic() - received > self.p['input_timeout']:
            return False
        if stamp is not None:
            age = (rospy.Time.now() - stamp).to_sec()
            return stamp != rospy.Time() and -0.1 <= age <= self.p['input_timeout']
        return True

    def transform_obstacles_to_global(self):
        msg = self.obstacle_msg
        if msg is None or not self.fresh(self.obstacle_received, msg.header.stamp):
            raise ValueError('LiDAR missing/stale')
        if not msg.header.frame_id or not 0 <= msg.objectCounts <= len(msg.centerX):
            raise ValueError('invalid LiDAR frame/count (or cluster overflow)')
        now = time.monotonic()
        self.obstacles = [o for o in self.obstacles if now - o[3] <= self.p['obstacle_memory_s']]
        # LiDAR can run faster than localization. Keep a small queue so an older,
        # still-fresh scan can be transformed after the next pose brackets its stamp.
        # Retrying only the newest scan would starve forever on future-TF errors.
        for candidate, received in reversed(self.pending_scans):
            if not self.fresh(received, candidate.header.stamp):
                continue
            if not candidate.header.frame_id or not 0 <= candidate.objectCounts <= len(candidate.centerX):
                continue
            if self.obstacle_stamp == candidate.header.stamp:
                return [o[:3] for o in self.obstacles]
            try:
                translation, rotation = self.listener.lookupTransform(
                    'map', candidate.header.frame_id, candidate.header.stamp)
            except tf.Exception:
                continue
            msg = candidate
            break
        else:
            raise ValueError('LiDAR TF unavailable at scan time')
        # Transform at scan acquisition time, never mix velodyne coordinates with map.
        matrix = tf.transformations.quaternion_matrix(rotation)
        matrix[:3, 3] = translation
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
            # Keep short-lived old detections to avoid cutting back through an occluded object.
            near = [j for j, o in enumerate(self.obstacles)
                    if np.hypot(o[0] - center[0], o[1] - center[1]) < self.p['obstacle_merge_distance']]
            item = (float(center[0]), float(center[1]), radius, now)
            if near:
                self.obstacles[near[0]] = item
            else:
                self.obstacles.append(item)
        self.obstacle_stamp = msg.header.stamp
        return [o[:3] for o in self.obstacles]

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
        if state != self.last_state:
            rospy.loginfo('[LOCAL PLANNER] %s', state)
            self.last_state = state

    def tick(self, _event):
        with self.lock:
            try:
                self.process()
            except (ValueError, IndexError, tf.Exception) as exc:
                rospy.logwarn_throttle(1.0, '[LOCAL PLANNER] %s', str(exc))
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
            if not self.fresh(self.index_received) or self.curr_idx < 0:
                raise ValueError('waypoint index missing/stale')
        static = in_zones(self.p['static_zones'], self.curr_idx, pos.x, pos.y)
        dynamic = in_zones(self.p['dynamic_zones'], self.curr_idx, pos.x, pos.y)
        if not static and not dynamic and not self.avoidance_active:
            self.core.reset()
            self.publish('NORMAL')
            return
        if self.points is None:
            raise ValueError('global_path unavailable')
        # Map rectangles also work before the index arrives; projection is then geometric.
        idx = self.curr_idx if self.fresh(self.index_received) else int(np.argmin(np.linalg.norm(self.points - [pos.x, pos.y], axis=1)))
        if not 0 <= idx < len(self.points):
            raise ValueError('waypoint outside global path')
        csp = self.core.reference(self.points, idx)
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
            self.publish('STATIC_OBSTACLE: blocked', True, True, candidates=candidates, obstacles=obstacles)
            return
        self.core.previous = best
        if returning and abs(d) <= self.p['return_d_tolerance'] and abs(heading) <= self.p['return_heading_tolerance']:
            if self.clear_since is None:
                self.clear_since = time.monotonic()
            if time.monotonic() - self.clear_since >= self.p['return_clear_time']:
                self.avoidance_active = False
                self.core.reset()
                self.publish('NORMAL', candidates=candidates, obstacles=obstacles)
                return
        else:
            self.clear_since = None
        self.avoidance_active = True
        peak_k = max(1e-4, float(np.max(np.abs(best.curvature))))
        speed = min(self.p['static_speed_kmh'], 3.6 * math.sqrt(self.p['max_lateral_accel'] / peak_k))
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
            label.text = 'd=%+.1f %s cost=%.2f' % (c.offset, c.rejection or 'OK', c.cost)
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
        if self.pose is not None:
            m = self.marker('state', 0, Marker.TEXT_VIEW_FACING)
            m.pose.position.x, m.pose.position.y = self.pose.pose.position.x, self.pose.pose.position.y
            m.pose.position.z, m.scale.z, m.color.a = 3., .5, 1.
            m.color.r = m.color.g = m.color.b = 1.
            m.text = state
            self.state_pub.publish(m)


if __name__ == '__main__':
    rospy.init_node('static_obstacle_avoidance_planner')
    planner = StaticObstacleAvoidancePlanner()
    rospy.spin()
