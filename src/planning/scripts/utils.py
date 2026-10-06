#!/usr/bin/env python3

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def make_pose_stamped(time, frame_id, pos=[0, 0, 0], ori=[0, 0, 0, 1]):
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = time
    pose_stamped.header.frame_id = frame_id

    pose_stamped.pose.position.x = pos[0]
    pose_stamped.pose.position.y = pos[1]
    pose_stamped.pose.position.z = pos[2]

    pose_stamped.pose.orientation.x = ori[0]
    pose_stamped.pose.orientation.y = ori[1]
    pose_stamped.pose.orientation.z = ori[2]
    pose_stamped.pose.orientation.w = ori[3]

    return pose_stamped

def make_empty_path(time, frame_id):
    empty_path = Path()
    empty_path.header.stamp = time
    empty_path.header.frame_id = frame_id

    return empty_path

def arr2path(time, frame_id, poss):
    path = Path()
    path.header.stamp = time
    path.header.frame_id = frame_id

    path.poses = [make_pose_stamped(time, frame_id, pos, ori) for pos, ori in poss]

    return path

from planning_geometry import CubicSpline2D, catesian_to_frenet
