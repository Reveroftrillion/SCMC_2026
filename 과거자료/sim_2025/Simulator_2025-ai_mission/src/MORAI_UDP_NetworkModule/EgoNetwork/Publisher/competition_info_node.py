#!/usr/bin/env python3

import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[2]))

import rospy
from lib.network.UDP import Receiver
from lib.define.EgoVehicleStatus import EgoVehicleStatus
from simul_msgs.msg import VehicleStatus

IP = '127.0.0.1'
PORT = 9102

def main():
    rospy.init_node('vehicle_status_publisher', anonymous=True)
    
    vehicle_status_pub = rospy.Publisher('/vehicle_status', VehicleStatus, queue_size=10)
    
    egovehiclestatus = Receiver(IP, PORT, EgoVehicleStatus())
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        try:
            status = egovehiclestatus.get_data()
            
            if status:
                vehicle_msg = VehicleStatus()
                vehicle_msg.vel_x = status.vel_x
                vehicle_msg.yaw = status.yaw
                
                vehicle_status_pub.publish(vehicle_msg)
                # rospy.loginfo(f"Published: vel_x={status.vel_x:.2f}, yaw={status.yaw:.2f}")
        
        except Exception as e:
            rospy.logerr(f"Error getting vehicle status: {e}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass