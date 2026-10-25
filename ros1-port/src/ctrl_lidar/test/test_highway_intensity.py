import json
import unittest
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Header, String
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2 as pc2
from ctrl_lidar.ctrl_obstacle_node import ObstacleNode


def cloud(intensities, with_intensity=True):
    # One object per entry, 3 m apart sideways; each entry gives the intensity of its points (cycled).
    rows=[]
    for k,values in enumerate(intensities):
        i=0
        for x in np.linspace(5.,5.8,8):
            for dy in np.linspace(0.,.6,7):
                rows.append([x-2.,-4.5+3.*k+dy,1.-2.11,values[i%len(values)]]);i+=1
    fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1)
            for i,k in enumerate(('x','y','z','intensity') if with_intensity else ('x','y','z'))]
    return pc2.create_cloud(Header(frame_id='lidar'),fields,np.array(rows,dtype=np.float32)[:,:len(fields)])


class HighwayIntensityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.node=ObstacleNode(parameter_overrides=[Parameter('use_tf',value=False),Parameter('use_route',value=False),Parameter('publish_markers',value=False)])
    def tearDown(self):self.node.destroy_node()
    def mission(self,name):self.node.on_mission(String(data=json.dumps(dict(mission=name))))

    def test_highway_drops_asphalt_points_and_paint_clusters_only(self):
        # asphalt, paint, a road user (0 / 190), a road user one ring of which hits paint-bright parts (half)
        msg=cloud([[10.,11.],[49.,91.,110.],[0.,190.],[0.,66.]])
        self.mission('HIGHWAY')
        ys=sorted(round(d['y']) for d in self.node.detect(msg,1.))
        self.assertEqual(ys,[2,5])
        # Clearance evidence keeps every return.
        self.assertEqual(len(self.node.raw_clearance_points),224)

    def test_other_missions_and_missing_intensity_keep_everything(self):
        msg=cloud([[10.,11.],[49.,91.,110.],[0.,190.],[0.,66.]])
        self.mission('CRUISE')
        self.assertEqual(len(self.node.detect(msg,1.)),4)
        # the roundabout drops asphalt returns too (its road surface came up as ring cars), nothing else
        self.mission('ROUNDABOUT')
        self.assertEqual(len(self.node.detect(msg,1.)),3)
        self.mission('HIGHWAY')
        self.assertEqual(len(self.node.detect(cloud([[10.],[49.],[0.],[0.]],False),1.)),4)
        self.node.mission_received_at-=2.
        self.assertEqual(len(self.node.detect(msg,1.)),4)
