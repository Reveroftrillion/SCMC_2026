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

import numpy as np
from scipy.interpolate import CubicSpline as CubicSpline1D
import rospy

class CubicSpline2D:
    def __init__(self, x, y, interval=0.2):
        self.interval = interval
        
        dx = np.diff(x)
        dy = np.diff(y)

        self.rs = np.concatenate([[0], np.cumsum(np.hypot(dx, dy))])
        
        sx = CubicSpline1D(self.rs, x)
        sy = CubicSpline1D(self.rs, y)
        
        ss = np.arange(0, self.rs[-1], interval)
        
        self.rx = sx(ss)
        self.ry = sy(ss)
        
        dx = sx(ss, 1)
        dy = sy(ss, 1)
        self.ryaw = np.arctan2(dy, dx)
        
        ddx = sx(ss, 2)
        ddy = sy(ss, 2)
        
        denom = (dx ** 2 + dy ** 2) ** 1.5
        with np.errstate(divide='ignore', invalid='ignore'):
            kappa = np.abs(dx * ddy - dy * ddx) / denom
        
        self.rkappa = np.nan_to_num(kappa, nan=0.0, posinf=0.0, neginf=0.0)

def catesian_to_frenet(x: float, y: float, csp: CubicSpline2D):
    dist = np.hypot(csp.rx - x, csp.ry - y)
    
    idx = np.argmin(dist)

    dx = x - csp.rx[idx]
    dy = y - csp.ry[idx]

    __cos = np.cos(csp.ryaw[idx])
    __sin = np.sin(csp.ryaw[idx])

    s = idx * csp.interval
    d = np.copysign(dist[idx], __cos * dy - __sin * dx)

    return s, d