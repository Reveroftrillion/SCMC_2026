"""Execute the real ROS adapter with import/message/clock/TF fixtures, without ROS.

These tests validate node decisions, not rospy transport, TF interpolation or RViz.
"""
import copy
import importlib.util
import math
import sys
import types
import unittest
from collections import deque
from pathlib import Path
from unittest.mock import Mock, patch
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
NS = types.SimpleNamespace


class Stamp:
    now_value = 10.
    def __init__(self, seconds=0):
        self.seconds = seconds
    @classmethod
    def now(cls):
        return cls(cls.now_value)
    def to_sec(self):
        return self.seconds
    def to_nsec(self):
        return round(self.seconds * 1e9)
    def __eq__(self, other):
        return isinstance(other, Stamp) and self.seconds == other.seconds


def header():
    return NS(frame_id='', stamp=Stamp())


def pose_message():
    return NS(header=header(), pose=NS(position=NS(x=0., y=0., z=0.),
              orientation=NS(x=0., y=0., z=0., w=1.)))


def path_message():
    return NS(header=header(), poses=[])


def plan_message():
    return NS(header=header(), active=False, stop=False, speed_limit_kmh=0., state='', path=path_message())


def load_adapter():
    rospy = types.ModuleType('rospy')
    rospy.Time = Stamp
    for name in ('loginfo', 'logwarn', 'logerr', 'loginfo_throttle', 'logwarn_throttle'):
        setattr(rospy, name, Mock())
    rospy.Publisher = lambda *a, **kw: Mock()
    rospy.Subscriber = Mock()
    rospy.Timer = Mock()
    rospy.Duration = lambda x: x
    tf = types.ModuleType('tf')
    tf.Exception = type('TransformError', (Exception,), {})
    tf.TransformListener = Mock
    def matrix(q):
        result = np.eye(4)
        result[:3, :3] = Rotation.from_quat(q).as_matrix()
        return result
    tf.transformations = NS(quaternion_matrix=matrix,
        quaternion_from_euler=lambda r, p, y: Rotation.from_euler('xyz', [r, p, y]).as_quat(),
        euler_from_quaternion=lambda q: Rotation.from_quat(q).as_euler('xyz'))
    modules = {'rospy': rospy, 'tf': tf}
    messages = {
        'geometry_msgs': {'PoseStamped': pose_message, 'Point': lambda **kw: NS(**kw)},
        'nav_msgs': {'Path': path_message},
        'std_msgs': {'Bool': lambda **kw: NS(**kw), 'Int16': lambda **kw: NS(**kw)},
        'visualization_msgs': {'Marker': Mock, 'MarkerArray': Mock},
        'lidar_object_detection': {'ObjectInfo': Mock},
        'simul_msgs': {'LocalPlan': plan_message},
    }
    for package, names in messages.items():
        parent, msg = types.ModuleType(package), types.ModuleType(package + '.msg')
        for name, constructor in names.items():
            setattr(msg, name, constructor)
        parent.msg = msg
        modules[package], modules[package + '.msg'] = parent, msg
    # Load utils under an isolated name so mocks never replace its production import.
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location('_fixture_utils', SCRIPTS / 'utils.py')
        utils = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(utils)
        with patch.dict(sys.modules, {'utils': utils}):
            spec = importlib.util.spec_from_file_location('_fixture_local_node', SCRIPTS / 'StaticObstacleAvoidancePlanner.py')
            adapter = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(adapter)
    return adapter


ADAPTER = load_adapter()


class LocalNodeTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        Stamp.now_value = self.now
        self.config = yaml.safe_load((SCRIPTS.parent / 'config/local_planner.yaml').read_text())
        self.p = self.config['static_obstacle_avoidance_planner']
        self.p['static_zones'] = [dict(xmin=-1., xmax=30., ymin=-4., ymax=4.)]
        ADAPTER.rospy.get_param = lambda _: self.p
        self.clock = patch.object(ADAPTER.time, 'monotonic', lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.node = ADAPTER.StaticObstacleAvoidancePlanner()
        self.node.visualize = Mock()
        self.node.listener.lookupTransform.return_value = ((0, 0, 0), (0, 0, 0, 1))
        self.path = path_message()
        self.path.header.frame_id = 'map'
        self.path.poses = [pose_message() for _ in range(401)]
        for i, pose in enumerate(self.path.poses):
            pose.pose.position.x = i * .5
        self.refresh()

    def refresh(self, x=0., y=0., objects=()):
        Stamp.now_value = self.now
        self.node.global_path_callback(self.path)
        pose = pose_message()
        pose.header.frame_id, pose.header.stamp = 'map', Stamp(self.now)
        pose.pose.position.x, pose.pose.position.y = x, y
        self.node.current_pose_callback(pose)
        self.node.curr_idx_callback(NS(data=int(x / .5)))
        msg = NS(header=NS(frame_id='velodyne', stamp=Stamp(self.now)), objectCounts=len(objects))
        for name in ('centerX', 'centerY', 'centerZ', 'lengthX', 'lengthY', 'lengthZ'):
            setattr(msg, name, [0.] * 100)
        for i, (ox, oy, size) in enumerate(objects):
            msg.centerX[i], msg.centerY[i] = ox - x, oy - y
            msg.lengthX[i] = msg.lengthY[i] = size
            msg.lengthZ[i] = .5
        # Pose/TF fixture translates to the same map position as the synthetic car.
        self.node.listener.lookupTransform.return_value = ((x, y, 0), (0, 0, 0, 1))
        self.node.obstacle_callback(msg)

    def tick(self):
        self.node.tick(None)
        return self.node.plan_pub.publish.call_args[0][0]

    def test_empty_zones_preserve_normal_without_inputs(self):
        self.p['static_zones'] = []
        self.node.pose, self.node.points = None, None
        plan = self.tick()
        self.assertFalse(plan.active or plan.stop)
        self.assertEqual(plan.speed_limit_kmh, 0)
        self.assertEqual(plan.path.poses, [])

    def test_no_obstacles_keeps_global_with_zone_speed_cap(self):
        plan = self.tick()
        self.assertTrue(plan.state.startswith('NORMAL'))
        self.assertFalse(plan.active or plan.stop)
        self.assertEqual(plan.speed_limit_kmh, 20.)

    def test_static_centre_left_right_and_complete_blockage(self):
        for y in (0., -1., 1.):
            self.node.core.reset()
            self.node.avoidance_active = False
            self.node.memory.items = []
            self.now += .1
            self.refresh(objects=[(12, y, 1)])
            plan = self.tick()
            self.assertTrue(plan.active)
            self.assertFalse(plan.stop)
            self.assertTrue(plan.state.startswith('STATIC_OBSTACLE'))
            self.assertGreater(len(plan.path.poses), 5)
        self.now += .1
        self.refresh(objects=[(12, 0, 12)])
        plan = self.tick()
        self.assertTrue(plan.active and plan.stop)
        self.assertIn('no valid candidate', plan.state)
        self.assertEqual(plan.path.poses, [])

    def test_pass_obstacle_return_and_release_inside_zone(self):
        self.refresh(objects=[(12, 0, 1)])
        self.assertTrue(self.tick().active)
        side = self.node.core.previous.offset
        self.now = 13.
        self.refresh(x=19., y=side)
        plan = self.tick()
        self.assertEqual(plan.state, 'RETURN_TO_GLOBAL')
        self.assertTrue(plan.active)
        self.assertEqual(self.node.core.previous.offset, 0)
        self.now = 13.1
        self.refresh(x=20.)
        self.assertEqual(self.tick().state, 'RETURN_TO_GLOBAL')
        self.now = 14.
        self.refresh(x=20.)
        plan = self.tick()
        self.assertTrue(plan.state.startswith('NORMAL'))
        self.assertFalse(plan.active or plan.stop)

    def test_zone_exit_retains_local_until_aligned(self):
        self.node.avoidance_active = True
        self.refresh(x=32., y=1.)
        self.assertEqual(self.tick().state, 'RETURN_TO_GLOBAL')
        self.now += .1
        self.refresh(x=34.)
        self.assertTrue(self.tick().active)
        self.now += .9
        self.refresh(x=34.)
        self.assertFalse(self.tick().active)

    def test_hold_interrupts_return_completion_timer(self):
        self.node.avoidance_active = True
        self.refresh(x=32.)
        self.assertTrue(self.tick().active)
        self.now += .6
        plan = self.tick()
        self.assertTrue(plan.stop)
        self.assertIsNone(self.node.clear_since)
        self.now += .3
        self.refresh(x=32.)
        self.assertTrue(self.tick().active)

    def test_invalid_pose_and_stale_pose_hold(self):
        self.node.pose.pose.orientation.w = 0
        self.assertIn('invalid current pose', self.tick().state)
        self.refresh()
        self.now += .6
        self.assertIn('current_pose missing/stale', self.tick().state)

    def test_stale_obstacle_empty_scan_and_missing_tf_hold(self):
        self.node.obstacle_received -= 1.
        self.assertIn('LiDAR missing/stale', self.tick().state)
        self.refresh()
        self.node.listener.lookupTransform.side_effect = ADAPTER.tf.Exception('fixture')
        plan = self.tick()
        self.assertTrue(plan.stop)
        self.assertIn('map <- velodyne', plan.state)

    def test_invalid_tf_or_box_hold_without_partial_memory(self):
        self.node.listener.lookupTransform.return_value = ((float('nan'), 0, 0), (0, 0, 0, 1))
        self.assertIn('invalid LiDAR TF', self.tick().state)
        self.now += .1
        self.refresh(objects=[(12, 0, 1), (14, 0, 1)])
        self.node.obstacle_msg.lengthX[1] = float('nan')
        self.assertIn('invalid obstacle geometry', self.tick().state)
        self.assertEqual(self.node.memory.items, [])

    def test_invalid_and_stale_global_path_hold(self):
        self.path.header.frame_id = 'other'
        self.node.global_path_callback(self.path)
        self.assertIn('invalid global_path', self.tick().state)
        self.path.header.frame_id = 'map'
        self.refresh()
        self.node.path_received -= 3
        self.assertIn('global_path publisher stale', self.tick().state)

    def test_reference_end_and_discontinuity_hold(self):
        self.p['static_zones'] = [dict(xmin=-1., xmax=210., ymin=-4., ymax=4.)]
        self.refresh(x=199.)
        self.assertTrue(self.tick().stop)
        self.refresh()
        self.node.points[20:40] = self.node.points[19::-1]
        self.assertIn('reference yaw discontinuity', self.tick().state)

    def test_index_and_rectangle_approach_cap_without_local_path(self):
        for zones in ([{'start': 40, 'end': 60}], [dict(xmin=20.2, xmax=30., ymin=-1., ymax=1.)]):
            self.p['static_zones'] = zones
            self.refresh(x=10.)
            plan = self.tick()
            self.assertIn('NORMAL: approach STATIC_OBSTACLE', plan.state)
            self.assertFalse(plan.active or plan.stop)
            self.assertEqual(plan.speed_limit_kmh, 20.)
            self.assertFalse(plan.path.poses)

    def test_out_of_range_zone_reports_reason(self):
        self.p['static_zones'] = [{'start': 300, 'end': 500}]
        self.refresh()
        self.assertIn('zone waypoint range exceeds', self.tick().state)

    def test_dynamic_policy_still_stops_without_lateral_avoidance(self):
        self.p['static_zones'] = []
        self.p['dynamic_zones'] = [dict(xmin=-1., xmax=30., ymin=-4., ymax=4.)]
        self.refresh(objects=[(12, 0, 1)])
        self.assertEqual(self.tick().state, 'DYNAMIC_OBSTACLE: stop')
        self.now += 2.1
        self.refresh()
        plan = self.tick()
        self.assertEqual(plan.state, 'DYNAMIC_OBSTACLE: corridor_clear')
        self.assertEqual(plan.speed_limit_kmh, 10.)
