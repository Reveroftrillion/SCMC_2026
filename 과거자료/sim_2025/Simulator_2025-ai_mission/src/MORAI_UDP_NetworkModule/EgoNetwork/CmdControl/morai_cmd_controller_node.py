#!/usr/bin/env python3

import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[2]))

import rospy
from lib.network.UDP import Sender
from lib.define.EgoCtrlCmd import EgoCtrlCmd
from simul_msgs.msg import ControlCmd

IP = '192.168.0.33' 
PORT = 4400

class MoraiCmdController:
    def __init__(self):
        rospy.init_node('morai_cmd_controller', anonymous=True)
        
        # UDP sender for MORAI
        self.ego_ctrl = Sender(IP, PORT)
        
        # ROS subscriber for control commands
        self.control_sub = rospy.Subscriber('/control_cmd', ControlCmd, self.controlCmdCallback)
        
        # Initialize EgoCtrlCmd with default values
        self.data = EgoCtrlCmd()
        self.data.ctrl_mode = 2  # 1 : Keyboard   2 : AutoMode
        self.data.gear = 4       # D (Drive)
        self.data.cmd_type = 1   # Throttle(accel,brake,steer)
        self.data.steer = 0.0    # Straight ahead
        
        # Default values
        self.data.accel = 0.0
        self.data.brake = 0.0
        
        rospy.loginfo("MORAI Cmd Controller initialized")
        rospy.loginfo("Listening to /control_cmd topic")
        rospy.loginfo(f"Sending UDP commands to {IP}:{PORT}")
        
    def controlCmdCallback(self, msg):
        """
        Callback function for control command messages
        """
        # Update accel and brake based on received message

        self.data.accel = msg.accel
        self.data.brake = msg.brake
        self.data.steer = msg.steering

        # Send UDP command to MORAI
        try:
            self.ego_ctrl.send(self.data)
            # rospy.loginfo(f"Sent to MORAI: accel={self.data.accel:.2f}, brake={self.data.brake:.2f}, steer={self.data.steer:.2f}")
        except Exception as e:
            rospy.logerr(f"Failed to send UDP command: {e}")

def main():
    try:
        controller = MoraiCmdController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MORAI Cmd Controller shutting down")

if __name__ == '__main__':
    main()