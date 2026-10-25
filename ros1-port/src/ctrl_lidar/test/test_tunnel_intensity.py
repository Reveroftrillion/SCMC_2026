import json
import unittest
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Header, String
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2 as pc2
from ctrl_lidar.ctrl_obstacle_node import ObstacleNode


def cloud(with_intensity=True):
    # Two otherwise identical objects; only their return intensity differs.
    rows=[]
    for y,intensity in ((-1.,0.),(1.,42.)):
        for x in np.linspace(5.,5.8,8):
            for dy in np.linspace(0.,.6,7):
                rows.append([x-2.,y+dy,1.-2.11,intensity])
    fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1)
            for i,k in enumerate(('x','y','z','intensity') if with_intensity else ('x','y','z'))]
    return pc2.create_cloud(Header(frame_id='lidar'),fields,np.array(rows,dtype=np.float32)[:,:len(fields)])


class TunnelIntensityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.node=ObstacleNode(parameter_overrides=[Parameter('use_tf',value=False),Parameter('use_route',value=False),Parameter('publish_markers',value=False)])
    def tearDown(self):self.node.destroy_node()
    def mission(self,name):self.node.on_mission(String(data=json.dumps(dict(mission=name))))

    def test_only_tunnel_drops_zero_returns_and_exit_restores_them(self):
        msg=cloud()
        # The highway keeps the zero returns (road users) and drops the paint-bright one (test_highway_intensity).
        for mission,count in [('HIGHWAY',1),('TUNNEL',1),('CRUISE',2),('ROUNDABOUT',2)]:
            self.mission(mission)
            dets=self.node.detect(msg,1.)
            self.assertEqual(len(dets),count)
            if count==1:self.assertEqual(dets[0]['y']>0.,mission=='TUNNEL')
        # Input cloud and clearance evidence are not rewritten as free space.
        self.assertEqual(len(self.node.raw_clearance_points),112)
        self.assertEqual(len(pc2.read_points_numpy(msg)),112)

    def test_missing_intensity_or_missing_expired_mission_does_not_filter(self):
        self.assertEqual(len(self.node.detect(cloud(),1.)),2)
        self.mission('TUNNEL')
        self.assertEqual(len(self.node.detect(cloud(False),1.)),2)
        self.node.mission_received_at-=2.
        self.assertEqual(len(self.node.detect(cloud(),1.)),2)

    def test_elevated_asphalt_reference_removes_paint_but_keeps_low_box(self):
        x, y = np.meshgrid(np.linspace(3.1, 27.9, 80), np.linspace(-4.9, 4.9, 25))
        floor = np.c_[x.ravel(), y.ravel(), np.full(x.size, .53), np.full(x.size, 10.)]
        x, y = np.meshgrid(np.linspace(10., 10.2, 8), np.linspace(-1.5, -1.3, 8))
        paint = np.c_[x.ravel(), y.ravel(), np.full(x.size, .53), np.full(x.size, 54.)]
        x, y = np.meshgrid(np.linspace(15., 15.7, 12), np.linspace(-.4, .4, 12))
        box = np.c_[x.ravel(), y.ravel(), np.full(x.size, 1.23), np.full(x.size, 127.)]
        raw = np.vstack((floor, paint, box)).astype(np.float32)
        raw[:, :3] -= [2., 0., 2.11]
        fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, k in enumerate(('x', 'y', 'z', 'intensity'))]
        msg = pc2.create_cloud(Header(frame_id='lidar'), fields, raw)
        self.mission('TUNNEL')
        dets = self.node.detect(msg, 1.)
        self.assertTrue(any(abs(d['x']-15.35) < .1 and abs(d['z_max']-.7) < .01 for d in dets))
        self.assertFalse(any(abs(d['x']-10.1) < .2 and abs(d['y']+1.4) < .2 for d in dets))
        # Source cloud bytes are preserved; other mission policies and stale
        # tunnel ownership must retain their existing fixed-height behavior.
        np.testing.assert_array_equal(pc2.read_points_numpy(msg), raw)
        for mission in ('CRUISE', 'HIGHWAY', 'ROUNDABOUT'):
            self.mission(mission)
            self.node.detect(msg, 1.)
            self.assertGreater(float(np.median(self.node.raw_clearance_points[:, 2])), .5)
        self.mission('TUNNEL')
        self.node.mission_received_at -= 2.
        self.node.detect(msg, 1.)
        self.assertGreater(float(np.median(self.node.raw_clearance_points[:, 2])), .5)

    def test_static_ground_support_removes_paint_preserves_zero_return_box_and_expires(self):
        x, y = np.meshgrid(np.linspace(3.1, 27.9, 80), np.linspace(-4.9, 4.9, 25))
        floor = np.c_[x.ravel(), y.ravel(), np.full(x.size, .53), np.full(x.size, 10.)]
        x, y = np.meshgrid(np.linspace(10., 10.2, 8), np.linspace(-1.5, -1.3, 8))
        paint = np.c_[x.ravel(), y.ravel(), np.full(x.size, .53), np.full(x.size, 54.)]
        x, y = np.meshgrid(np.linspace(15., 15.7, 12), np.linspace(-.4, .4, 12))
        box = np.c_[x.ravel(), y.ravel(), np.full(x.size, 1.23), np.zeros(x.size)]
        raw = np.vstack((floor, paint, box)).astype(np.float32)
        raw[:, :3] -= [2., 0., 2.11]
        fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, k in enumerate(('x', 'y', 'z', 'intensity'))]
        msg = pc2.create_cloud(Header(frame_id='lidar'), fields, raw)
        self.mission('STATIC_OBSTACLE')
        dets = self.node.detect(msg, 1.)
        self.assertTrue(any(abs(d['x']-15.35) < .1 and abs(d['z_max']-.7) < .01 for d in dets))
        self.assertFalse(any(abs(d['x']-10.1) < .2 for d in dets))
        np.testing.assert_array_equal(pc2.read_points_numpy(msg), raw)
        self.node.mission_received_at -= 2.
        self.node.detect(msg, 1.)
        self.assertGreater(float(np.median(self.node.raw_clearance_points[:, 2])), .5)

    def test_far_material_support_removes_road_strip_but_preserves_low_box(self):
        x, y = np.meshgrid(np.linspace(4., 25., 40), np.linspace(-4., 4., 20))
        near = np.c_[x.ravel(), y.ravel(), np.full(x.size, .53), np.full(x.size, 10.)]
        y = np.linspace(-4., 4., 61)
        far = np.c_[np.full(len(y), 33.), y, np.full(len(y), .53), np.full(len(y), 10.)]
        x, y = np.meshgrid(np.linspace(30., 30.2, 8), np.linspace(-1.5, -1.3, 8))
        strip = np.c_[x.ravel(), y.ravel(), np.full(x.size, .53), np.full(x.size, 20.)]
        x, y = np.meshgrid(np.linspace(30., 30.7, 12), np.linspace(-.4, .4, 12))
        box = np.c_[x.ravel(), y.ravel(), np.full(x.size, 1.23), np.full(x.size, 127.)]
        raw = np.vstack((near, far, strip, box)).astype(np.float32)
        raw[:, :3] -= [2., 0., 2.11]
        fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, k in enumerate(('x', 'y', 'z', 'intensity'))]
        self.mission('TUNNEL')
        msg = pc2.create_cloud(Header(frame_id='lidar'), fields, raw)
        dets = self.node.detect(msg, 1.)
        self.assertTrue(any(abs(d['x']-30.35) < .1 and abs(d['z_max']-.7) < .01 for d in dets))
        self.assertFalse(any(abs(d['x']-30.1) < .2 and abs(d['y']+1.4) < .2 for d in dets))
        np.testing.assert_array_equal(pc2.read_points_numpy(msg), raw)

    def test_curved_road_material_support_preserves_box_outside_fixed_ground_rectangle(self):
        x, y = np.meshgrid(np.linspace(4., 25., 40), np.linspace(-4., 4., 20))
        near = np.c_[x.ravel(), y.ravel(), np.full(x.size, .53), np.full(x.size, 10.)]
        y = np.linspace(-8., 2., 61)
        far = np.c_[np.full(len(y), 31.), y, np.full(len(y), .53), np.full(len(y), 10.)]
        x, y = np.meshgrid(np.linspace(29.5, 29.7, 8), np.linspace(-5.8, -5.6, 8))
        strip = np.c_[x.ravel(), y.ravel(), np.full(x.size, .53), np.full(x.size, 110.)]
        x, y = np.meshgrid(np.linspace(29.5, 30.2, 12), np.linspace(-7.3, -6.5, 12))
        box = np.c_[x.ravel(), y.ravel(), np.full(x.size, 1.23), np.full(x.size, 127.)]
        raw = np.vstack((near, far, strip, box)).astype(np.float32)
        raw[:, :3] -= [2., 0., 2.11]
        fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, k in enumerate(('x', 'y', 'z', 'intensity'))]
        self.mission('TUNNEL')
        msg = pc2.create_cloud(Header(frame_id='lidar'), fields, raw)
        dets = self.node.detect(msg, 1.)
        self.assertTrue(any(abs(d['x']-29.85) < .1 and abs(d['y']+6.9) < .1
                            and abs(d['z_max']-.7) < .01 for d in dets))
        self.assertFalse(any(abs(d['x']-29.6) < .2 and abs(d['y']+5.7) < .2 for d in dets))
