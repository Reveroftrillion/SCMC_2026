"""Elevated roundabout asphalt must not become ring-car occupancy."""
import json
import unittest

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header, String

from ctrl_lidar.ctrl_obstacle_node import ObstacleNode


def elevated_scene(intensity=True, support=True):
    x, y = np.meshgrid(np.linspace(3.1, 27.9, 80), np.linspace(-4.9, 4.9, 25))
    floor = np.c_[x.ravel(), y.ravel(), np.full(x.size, .53), np.full(x.size, 10.)]
    x, y = np.meshgrid(np.linspace(10., 10.2, 8), np.linspace(-1.5, -1.3, 8))
    paint = np.c_[x.ravel(), y.ravel(), np.full(x.size, .53), np.full(x.size, 109.)]
    x, y = np.meshgrid(np.linspace(15., 15.7, 12), np.linspace(-.4, .4, 12))
    car = np.c_[x.ravel(), y.ravel(), np.full(x.size, 1.23), np.zeros(x.size)]
    raw = np.vstack(([floor] if support else []) + [paint, car]).astype(np.float32)
    raw[:, :3] -= [2., 0., 2.11]
    names = ('x', 'y', 'z', 'intensity') if intensity else ('x', 'y', 'z')
    fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
              for i, k in enumerate(names)]
    raw = raw[:, :len(names)]
    return pc2.create_cloud(Header(frame_id='lidar'), fields, raw), raw


class RoundaboutMaterialGroundTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = ObstacleNode(parameter_overrides=[Parameter('use_tf', value=False),
            Parameter('use_route', value=False), Parameter('publish_markers', value=False)])
        self.addCleanup(self.node.destroy_node)

    def mission(self, name):
        self.node.on_mission(String(data=json.dumps(dict(mission=name))))

    def test_roundabout_road_paint_removed_and_zero_return_car_preserved(self):
        msg, raw = elevated_scene()
        self.mission('ROUNDABOUT')
        dets = self.node.detect(msg, 1.)
        self.assertFalse(any(abs(d['x']-10.1) < .2 for d in dets))
        self.assertTrue(any(abs(d['x']-15.35) < .1 and abs(d['z_max']-.7) < .01 for d in dets))
        np.testing.assert_array_equal(pc2.read_points_numpy(msg), raw)

    def test_missing_material_support_or_intensity_keeps_fallback(self):
        for intensity, support in ((False, True), (True, False)):
            with self.subTest(intensity=intensity, support=support):
                msg, raw = elevated_scene(intensity, support)
                self.mission('ROUNDABOUT')
                self.node.detect(msg, 1.)
                self.assertGreater(float(np.median(self.node.raw_clearance_points[:, 2])), .5)
                np.testing.assert_array_equal(pc2.read_points_numpy(msg), raw)

    def test_stale_mission_and_other_missions_keep_fallback(self):
        msg, _ = elevated_scene()
        for mission in ('CRUISE', 'HIGHWAY'):
            self.mission(mission)
            self.node.detect(msg, 1.)
            self.assertGreater(float(np.median(self.node.raw_clearance_points[:, 2])), .5)
        self.mission('ROUNDABOUT')
        self.node.mission_received_at -= 2.
        self.node.detect(msg, 1.)
        self.assertGreater(float(np.median(self.node.raw_clearance_points[:, 2])), .5)
