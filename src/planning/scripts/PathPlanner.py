#!/usr/bin/env python3

import sys
import os

# Add the scripts directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from collections import deque

import rospy
import rospkg

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from simul_msgs.msg import TrafficSign

from GlobalPathPlanner import GlobalPathPlanner

from std_msgs.msg import Float64

from utils import CubicSpline2D, catesian_to_frenet, make_empty_path

from tf.transformations import quaternion_matrix

PATH_STEP = 0.5             # (m)
LOOKAHEAD_DISTANCE = 20    # (m)
EMPTY_MARKER_ARRAY = MarkerArray()

class PathPlanner:
    def __init__(self):
        rospy.init_node('planner')
        self.pub_control_path = rospy.Publisher('/control_path', Path, queue_size=10)

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('planning')

        self.gpp = GlobalPathPlanner(PATH_STEP, LOOKAHEAD_DISTANCE, pkg_path)

        self.current_pose = None
        self.traffic_sign = TrafficSign()
        self.traffic_sign.traffic_sign = 'None'
        self.empty_path = make_empty_path(rospy.Time.now(), 'map')

        rospy.Subscriber('/current_pose', PoseStamped, callback=self.current_pose_callback)

        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=10)
        rospy.Timer(rospy.Duration(1), self.timer_callback)        

    def process(self):
        self.gpp.make_g_path_subset(self.curr_pose)
        self.pub_control_path.publish(self.gpp.g_path_subset)

    def current_pose_callback(self, current_pose: PoseStamped):
        self.curr_pose = current_pose
        self.process()
        
    def timer_callback(self, event):
        self.global_path_pub.publish(self.gpp.g_path_msg)
        
if __name__ == '__main__':
    path_planner = PathPlanner()
    rospy.spin()
