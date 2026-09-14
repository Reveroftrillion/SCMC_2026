#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np

import rospy

from geometry_msgs.msg import PoseStamped

from tf.transformations import quaternion_from_euler

from utils import make_pose_stamped, make_empty_path

class GlobalPathPlanner:
    def __init__(self, path_step, lookahead_distance, pkg_path):
        assert(path_step != 0)
        
        ##### 변수 초기화 #####
        self.curr_idx = 0
        self.g_path = []
        self.g_path_msg = make_empty_path(rospy.Time.now(), 'map')
        self.g_path_subset = make_empty_path(rospy.Time.now(), 'map')
        self.g_path_subset_array = []
        self.publish_path_size = int(lookahead_distance / path_step)
        #######################
        
        ##### 전역 경로 로딩 #####
        self.gps_offset_x, self.gps_offset_y = 302595.0, 4124145.0 
        with open(os.path.join(pkg_path, 'paths', 'zzinmak.txt'), 'r') as f:
            while True:
                line = f.readline()
                if not line: break
                
                parts = line.split()
                if len(parts) < 2:
                    continue
                x, y = float(parts[0]), float(parts[1])
                x += self.gps_offset_x
                y += self.gps_offset_y
                
                self.g_path_msg.poses.append(
                    make_pose_stamped(
                        self.g_path_msg.header.stamp,
                        'map',
                        [x, y, 0])
                )
                
                self.g_path.append((x, y, 0., 0.))
        #########################

        ##### 곡률 계산 #####
        self.g_path = np.array(self.g_path)

        dx, dy = np.gradient(self.g_path[:, 0]), np.gradient(self.g_path[:, 1])
        ddx, ddy = np.gradient(dx), np.gradient(dy)
        denom = (dx ** 2 + dy ** 2) ** 1.5
        with np.errstate(divide='ignore', invalid='ignore'):
            kappa = np.abs(dx * ddy - dy * ddx) / denom
        
        self.g_path[:, 2] = np.nan_to_num(kappa, nan=0.0, posinf=0.0, neginf=0.0)
        #######################

        ##### yaw 계산 #####
        diffs = np.diff(self.g_path[:, :2], axis=0)
        yaws = np.arctan2(diffs[:, 1], diffs[:, 0])
        self.g_path[:, 3] = np.concatenate([yaws, [yaws[-1]]])
        ####################

    def make_g_path_subset(self, current_pose: PoseStamped):
        time = current_pose.header.stamp
        self.g_path_subset.header.stamp = time
        
        self.g_path_subset.poses = []
        self.g_path_subset_array = []
        self.curr_idx = self.__current_waypoint(current_pose)
        search_range_end = min(self.curr_idx + self.publish_path_size, self.g_path.shape[0])
        for i in range(self.curr_idx, search_range_end):
            quat = quaternion_from_euler(0., 0., self.g_path[i, 3])
            pose_stamped = make_pose_stamped(time, 'map', self.g_path[i, :3], quat)

            self.g_path_subset.poses.append(pose_stamped)
            self.g_path_subset_array.append((self.g_path[i][0], self.g_path[i][1]))

        self.g_path_subset_array = np.array(self.g_path_subset_array)

    def __current_waypoint(self, current_pose: PoseStamped):
        # Check if current index is far from current position (initial startup or teleport)
        if self.curr_idx < self.g_path.shape[0]:
            curr_x = self.g_path[self.curr_idx, 0] - current_pose.pose.position.x
            curr_y = self.g_path[self.curr_idx, 1] - current_pose.pose.position.y
            curr_dist = np.hypot(curr_x, curr_y)

            # If current index is far (> 50m), search entire path
            if curr_dist > 50.0:
                x = self.g_path[:, 0] - current_pose.pose.position.x
                y = self.g_path[:, 1] - current_pose.pose.position.y
                dist = np.hypot(x, y)
                return np.argmin(dist)

        # Normal case: search near current index
        start = max(self.curr_idx - int(self.publish_path_size * 0.25), 0)
        end = min(self.curr_idx + self.publish_path_size, self.g_path.shape[0])

        x = self.g_path[start:end, 0] - current_pose.pose.position.x
        y = self.g_path[start:end, 1] - current_pose.pose.position.y

        dist = np.hypot(x, y)
        return np.argmin(dist) + start
