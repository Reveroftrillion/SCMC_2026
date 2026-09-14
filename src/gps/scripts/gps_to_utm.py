#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import GPSMessage
from tf.transformations import quaternion_from_euler
from pyproj import Transformer, CRS
import numpy as np
class TransformCoordinate:
    def __init__(self):
        self.curr_pose_pub=rospy.Publisher('/current_pose',PoseStamped,queue_size=10)
        self.transformer=Transformer.from_crs(CRS('EPSG:4326'),CRS('EPSG:32652'),always_xy=True)
        self.prev_x,self.prev_y,self.prev_z=0.,0.,0.; self.curr_pose=PoseStamped(); self.curr_pose.header.frame_id='map'
        rospy.Subscriber('/gps',GPSMessage,self.gps_callback)
    def gps_callback(self,msg):
        x,y=self.transformer.transform(msg.longitude,msg.latitude); self.curr_pose.header.stamp=rospy.Time.now(); self.curr_pose.pose.position.x=x; self.curr_pose.pose.position.y=y; self.curr_pose.pose.position.z=msg.altitude
        dist=np.hypot(x-self.prev_x,y-self.prev_y)
        if dist>0.2: self.quat=quaternion_from_euler(0.,0.,np.arctan2(y-self.prev_y,x-self.prev_x)); self.prev_x,self.prev_y,self.prev_z=x,y,msg.altitude
        self.curr_pose.pose.orientation.x,self.curr_pose.pose.orientation.y,self.curr_pose.pose.orientation.z,self.curr_pose.pose.orientation.w=self.quat
        self.curr_pose_pub.publish(self.curr_pose)
if __name__=='__main__': rospy.init_node('wgs84_to_utm52n'); TransformCoordinate(); rospy.spin()
