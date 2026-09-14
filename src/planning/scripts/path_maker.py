#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import rospkg
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from datetime import datetime
from tf.transformations import euler_from_quaternion

class PathMaker:
    def __init__(self):
        try:
            rospy.init_node('path_maker', anonymous=True)
        except rospy.exceptions.ROSException:
            pass

        self.curr_pose = PoseStamped()
        rospy.Subscriber('/current_pose', PoseStamped, self.curr_pose_callback)
        self.rviz_global_path_pub = rospy.Publisher('/rviz_global_path', Path, queue_size=1)
        self.rviz_global_path = Path()
        self.rviz_global_path.header.frame_id = 'map'
        self.rviz_global_path.header.stamp = rospy.Time.now()

        self.is_status = False
        self.idx = 0
        self.prev_x = 0
        self.prev_y = 0
        self.start_x = 0
        self.start_y = 0
        self.f = None

        rospack = rospkg.RosPack()
        ROS_HOME = rospack.get_path('planning')

        now = datetime.now()
        self.f = open(f'{ROS_HOME}/paths/{now.year}-{now.month}-{now.day}_{now.hour}-{now.minute}.txt', 'w')
        
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.is_status:
                self.path_make()
            rate.sleep()

        self.f.close()

    def curr_pose_callback(self, msg: PoseStamped):
        self.curr_pose = msg
        self.is_status = True

    def path_make(self):
        x = self.curr_pose.pose.position.x
        y = self.curr_pose.pose.position.y

        if self.start_x == 0 and self.start_y == 0:
            self.start_x = x
            self.start_y = y

        quat = self.curr_pose.pose.orientation 
        orientation_list = [quat.x, quat.y, quat.z, quat.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        mode = 9
        
        distance = np.hypot(x - self.prev_x, y - self.prev_y)
        
        self.rviz_global_path.header.stamp = rospy.Time.now()

        if distance > 0.2:
            self.prev_x = x
            self.prev_y = y

            rviz_pose = PoseStamped()
            rviz_pose.header.frame_id = 'map'
            rviz_pose.header.stamp = rospy.Time.now()
            rviz_pose.header.seq = self.idx
            rviz_pose.pose.position.x = x - self.start_x
            rviz_pose.pose.position.y = y - self.start_y
            rviz_pose.pose.position.z = 0.

            rviz_pose.pose.orientation.w = quat.w
            rviz_pose.pose.orientation.x = quat.x
            rviz_pose.pose.orientation.y = quat.y
            rviz_pose.pose.orientation.z = quat.z

            self.rviz_global_path.poses.append(rviz_pose)            

            data = '{0} {1} {2} {3} \n'.format(x, y, yaw, mode)
            self.f.write(data)
            
            self.idx += 1
            rospy.loginfo(f'{self.idx}, {x}, {y}, {yaw}')

        self.rviz_global_path_pub.publish(self.rviz_global_path)

if __name__ == "__main__":
    try:
        PathMaker()
    except rospy.ROSInterruptException:
        pass