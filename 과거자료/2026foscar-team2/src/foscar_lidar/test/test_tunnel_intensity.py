import json
import unittest
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Header, String
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2 as pc2
from foscar_lidar.obstacle_node import ObstacleNode


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
