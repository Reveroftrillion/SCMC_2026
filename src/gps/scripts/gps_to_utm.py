#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from simul_msgs.msg import VehicleStatus

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
        self.quat = None
        self.last_status = None

        self.curr_pose = PoseStamped()
        self.curr_pose.header.frame_id = 'map'

        self.offset = Pose()

        self.transformer = Transformer.from_crs(proj_WGS84, proj_UTM52N, always_xy=True)
        rospy.Subscriber('/gps', GPSMessage, self.gps_callback)
        rospy.Subscriber('/vehicle_status', VehicleStatus, self.status_callback)

    def status_callback(self, msg):
        if np.isfinite(msg.yaw):
            self.quat = quaternion_from_euler(0., 0., np.deg2rad(msg.yaw))
            self.last_status = time.monotonic()

    def gps_callback(self, msg: GPSMessage):
        # rospy.loginfo('hihihi')  # Commented out debug log

        if (not np.all(np.isfinite([msg.latitude, msg.longitude, msg.altitude]))
                or (msg.latitude == 0.0 and msg.longitude == 0.0)
                or abs(msg.latitude) > 90 or abs(msg.longitude) > 180):
            rospy.logwarn_throttle(2.0, 'Invalid GPS fix; retaining last pose')
            return
        if self.quat is None or self.last_status is None or time.monotonic() - self.last_status > 0.5:
            rospy.logwarn_throttle(2.0, 'Waiting for fresh vehicle heading')
            return
        x, y = self.transformer.transform(msg.longitude, msg.latitude)
        if not np.all(np.isfinite([x, y])):
            return

        self.curr_pose.header.stamp = rospy.Time.now()

        self.curr_pose.pose.position.x = x
        self.curr_pose.pose.position.y = y
        self.curr_pose.pose.position.z = msg.altitude

        # Heading comes from the permitted vehicle status, not the UTM origin
        # or a small GPS displacement. Blackout localization is a separate task.

        self.curr_pose.pose.orientation.x = self.quat[0]
        self.curr_pose.pose.orientation.y = self.quat[1]
        self.curr_pose.pose.orientation.z = self.quat[2]
        self.curr_pose.pose.orientation.w = self.quat[3]

        self.curr_pose_pub.publish(self.curr_pose)

if __name__ == "__main__":

    rospy.init_node("wgs84_to_utm52n")
    pub = TransformCoordinate()
    rospy.spin()
