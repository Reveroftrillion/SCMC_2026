"""Exercise the real LiDAR mission decision callback without ROS installed."""
import importlib.util
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))


def module(name, **items):
    result = ModuleType(name)
    for key, value in items.items():
        setattr(result, key, value)
    return result


def obj(s, d, ego=30):
    return dict(s=s, d=d, hs=.35, hd=.35, bx=s-ego, by=d, radius=.35,
                wx=s, wy=d, vx=0., vy=0., observed=2, id=int(s*10+d),
                size=.7, height=1.5, clear=0)


class LidarMissionIntegration(unittest.TestCase):
    def setUp(self):
        dummy = type('Dummy', (), {})
        modules = {
            'rospy': module('rospy', loginfo_throttle=lambda *a: None),
            'geometry_msgs': module('geometry_msgs'),
            'geometry_msgs.msg': module('geometry_msgs.msg', PoseStamped=dummy),
            'sensor_msgs': module('sensor_msgs', point_cloud2=module('point_cloud2')),
            'sensor_msgs.msg': module('sensor_msgs.msg', PointCloud2=dummy),
            'std_msgs': module('std_msgs'),
            'std_msgs.msg': module('std_msgs.msg', Bool=dummy, Float64=dummy),
        }
        self.patcher = patch.dict(sys.modules, modules)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        spec = importlib.util.spec_from_file_location('lidar_node_test', ROOT/'scripts/lidar_missions.py')
        self.mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.mod)
        n = self.mod.LidarSafety.__new__(self.mod.LidarSafety)
        n.path = np.array([[float(x), 0.] for x in range(101)])
        n.cum = np.arange(101, dtype=float)
        n.total = 100.
        n.zones = {'highway': [0, 90], 'tunnel': [0, 90]}
        n.allow_avoidance, n.road_half_width = True, 6.
        n.highway_return_s, n.offset, n.lookahead = 90., 3.2, 5.
        n.plan = n.plan_mode = n.ped_lock = None
        n.lateral = 0.
        n.lock = threading.Lock()
        self.n = n
        self.pose = (30., 0., 0.)
        self.ground = np.array([[x-30, y] for x in np.arange(10., 80., .5)
                                for y in np.arange(-6., 6.1, .5)])

    def test_highway_pass_keeps_plan_and_stops_for_new_hazard(self):
        self.n.zones.pop('tunnel')
        lead = obj(48, 0)
        self.n.decide([lead], 30, [lead], self.ground, self.pose)
        self.assertEqual(self.n.plan_mode, 'highway')
        self.assertLessEqual(self.n.plan[3]+self.n.plan[4], self.n.highway_return_s-15.)
        plan = self.n.plan
        blocker = obj(55, plan[1])
        self.n.decide([lead, blocker], 40, [lead, blocker], self.ground, self.pose)
        self.assertTrue(self.n.stop)
        self.assertEqual(self.n.plan, plan)
        self.n.decide([lead], 40, [lead], None, self.pose)
        self.assertTrue(self.n.stop)
        self.assertEqual(self.n.plan, plan)

    def test_tunnel_unknown_space_stops_and_observed_space_generates_plan(self):
        self.n.zones.pop('highway')
        lead = obj(48, 0)
        self.n.decide([lead], 30, [lead], None, self.pose)
        self.assertIsNone(self.n.plan)
        self.n.decide([lead], 30, [lead], self.ground, self.pose)
        self.assertEqual(self.n.plan_mode, 'tunnel')
        self.assertTrue(self.n.plan)

    def test_unverified_width_never_enables_avoidance(self):
        self.n.zones.pop('tunnel')
        self.n.road_half_width = 0.
        lead = obj(48, 0)
        self.n.decide([lead], 30, [lead], self.ground, self.pose)
        self.assertIsNone(self.n.plan)

    def test_pedestrian_zone_limits_approach_speed(self):
        self.n.zones = {'pedestrian': [50., 90.]}
        self.n.decide([], 30., [])
        self.assertLessEqual(self.n.speed_limit, 20. / 3.6)

    def test_old_lane_object_does_not_block_current_lane(self):
        self.n.zones.pop('tunnel')
        self.n.plan = (20., 3.2, 10., 60., 12.)
        self.n.plan_mode = 'highway'
        lead = obj(36., 0.)
        lead.update(bx=6.5, by=-4.5)
        ground = self.ground.copy()
        ground[:, 1] -= 3.2
        self.n.decide([lead], 30., [lead], ground, (30.,3.2,0.))
        self.assertFalse(self.n.stop)


if __name__ == '__main__':
    unittest.main()
