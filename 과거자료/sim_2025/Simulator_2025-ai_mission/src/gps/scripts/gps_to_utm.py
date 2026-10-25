#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import Pose, PoseStamped
from morai_msgs.msg import GPSMessage

from tf.transformations import quaternion_from_euler

from pyproj import Transformer, CRS

import numpy as np

class TransformCoordinate:
    def __init__(self):
        self.curr_pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=10)

        proj_WGS84 = CRS('EPSG:4326')
        proj_UTM52N = CRS('EPSG:32652')

        self.prev_x, self.prev_y, self.prev_z = 0., 0., 0.
        
        self.curr_pose = PoseStamped()
        self.curr_pose.header.frame_id = 'map'
        
        self.offset = Pose()

        self.transformer = Transformer.from_crs(proj_WGS84, proj_UTM52N, always_xy=True)
        rospy.Subscriber('/gps', GPSMessage, self.gps_callback)

    def gps_callback(self, msg: GPSMessage):
        # rospy.loginfo('hihihi')  # Commented out debug log

        x, y = self.transformer.transform(msg.longitude, msg.latitude)
        
        self.curr_pose.header.stamp = rospy.Time.now()
        
        self.curr_pose.pose.position.x = x
        self.curr_pose.pose.position.y = y
        self.curr_pose.pose.position.z = msg.altitude

        dist = np.hypot(x - self.prev_x, y - self.prev_y)
        if dist > 0.2:
            yaw = np.arctan2(y - self.prev_y, x - self.prev_x)
            self.quat = quaternion_from_euler(0., 0., yaw)

            self.prev_x = self.curr_pose.pose.position.x
            self.prev_y = self.curr_pose.pose.position.y
            self.prev_z = self.curr_pose.pose.position.z
        
        self.curr_pose.pose.orientation.x = self.quat[0]
        self.curr_pose.pose.orientation.y = self.quat[1]
        self.curr_pose.pose.orientation.z = self.quat[2]
        self.curr_pose.pose.orientation.w = self.quat[3]
        
        self.curr_pose_pub.publish(self.curr_pose)
        
if __name__ == "__main__":
    
    rospy.init_node("wgs84_to_utm52n")
    pub = TransformCoordinate()
    rospy.spin()
