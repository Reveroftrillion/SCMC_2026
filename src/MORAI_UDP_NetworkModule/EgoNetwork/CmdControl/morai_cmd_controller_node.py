#!/usr/bin/env python3
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[2]))
import rospy
from lib.network.UDP import Sender
from lib.define.EgoCtrlCmd import EgoCtrlCmd
from simul_msgs.msg import ControlCmd
IP = '127.0.0.1'
PORT = 9093
class MoraiCmdController:
    def __init__(self):
        rospy.init_node('morai_cmd_controller', anonymous=True)
        self.ego_ctrl = Sender(IP, PORT)
        self.control_sub = rospy.Subscriber('/control_cmd', ControlCmd, self.controlCmdCallback)
        self.data = EgoCtrlCmd()
        self.data.ctrl_mode, self.data.gear, self.data.cmd_type = 2, 4, 1
        self.data.steer = self.data.accel = self.data.brake = 0.0
        rospy.loginfo('MORAI Cmd Controller initialized')
    def controlCmdCallback(self, msg):
        self.data.accel, self.data.brake, self.data.steer = msg.accel, msg.brake, msg.steering
        try: self.ego_ctrl.send(self.data)
        except Exception as e: rospy.logerr(f'Failed to send UDP command: {e}')
if __name__ == '__main__':
    try:
        MoraiCmdController(); rospy.spin()
    except rospy.ROSInterruptException: pass
