#!/usr/bin/env python3
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[2]))
import rospy
from lib.network.UDP import Receiver
from lib.define.EgoVehicleStatus import EgoVehicleStatus
from simul_msgs.msg import VehicleStatus
IP = '127.0.0.1'
PORT = 9082
def main():
    rospy.init_node('vehicle_status_publisher', anonymous=True)
    pub = rospy.Publisher('/vehicle_status', VehicleStatus, queue_size=10)
    receiver = Receiver(IP, PORT, EgoVehicleStatus())
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            status = receiver.get_data()
            if status:
                pub.publish(VehicleStatus(vel_x=status.vel_x, yaw=status.yaw))
        except Exception as e: rospy.logerr(f'Error getting vehicle status: {e}')
        rate.sleep()
if __name__ == '__main__':
    try: main()
    except rospy.ROSInterruptException: pass
