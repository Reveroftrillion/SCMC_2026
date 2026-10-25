#!/usr/bin/env python3

import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[2]))

import numpy as np
from tf.transformations import quaternion_from_euler

import rospy
from lib.network.UDP import Receiver
from lib.define.EgoVehicleStatus import EgoVehicleStatus
from simul_msgs.msg import VehicleStatus
from geometry_msgs.msg import PoseStamped

IP = '192.168.0.33'
PORT = 1234

def main():
    rospy.init_node('vehicle_status_publisher', anonymous=True)
    
    current_pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=10)

    egovehiclestatus = Receiver(IP, PORT, EgoVehicleStatus())
    
    rate = rospy.Rate(10)  # 10 Hz
    
    curr_pose = PoseStamped()
    curr_pose.header.frame_id = 'map'
    curr_pose.pose.orientation.x = 0.0
    curr_pose.pose.orientation.y = 0.0
    curr_pose.pose.orientation.z = 0.0
    curr_pose.pose.orientation.w = 1.0
    prev_x, prev_y = 0.0, 0.0
    while not rospy.is_shutdown():
        try:
            status = egovehiclestatus.get_data()
            
            if status:
                curr_pose.header.stamp = rospy.Time.now()
                curr_pose.pose.position.x = status.pos_x
                curr_pose.pose.position.y = status.pos_y
                curr_pose.pose.position.z = status.vel_x

                dist = np.hypot(status.pos_x - prev_x, status.pos_y - prev_y)
                if dist > 0.2:
                    yaw = np.arctan2(status.pos_y - prev_y, status.pos_x - prev_x)
                    quat = quaternion_from_euler(0., 0., yaw)

                    prev_x = status.pos_x
                    prev_y = status.pos_y

                    curr_pose.pose.orientation.x = quat[0]
                    curr_pose.pose.orientation.y = quat[1]
                    curr_pose.pose.orientation.z = quat[2]
                    curr_pose.pose.orientation.w = quat[3]

                current_pose_pub.publish(curr_pose)
        
        except Exception as e:
            rospy.logerr(f"Error getting vehicle status: {e}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass