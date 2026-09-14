#!/usr/bin/env python3
import sys
from pathlib import Path
import rospkg
sys.path.insert(0, rospkg.RosPack().get_path('morai_udp'))

import ctypes
import time
import rospy
from lib.network.UDP import Receiver
from lib.network.parsers import status_parser
from lib.define.EgoVehicleStatus import EgoVehicleStatus
from simul_msgs.msg import VehicleStatus


def main():
    rospy.init_node('vehicle_status_publisher')
    confirmed = rospy.get_param('~layout_confirmed', False)
    header = rospy.get_param('~header', '#MoraiInfo$').encode('ascii')
    if len(header) != 11:
        raise ValueError('status header must be exactly 11 bytes for this layout')
    parser = status_parser(EgoVehicleStatus, header)
    rospy.logwarn('Status candidate layout: %d bytes, header=%r; confirmed=%s',
                  ctypes.sizeof(EgoVehicleStatus), header, confirmed)

    def parse(raw):
        if not confirmed:
            rospy.logwarn_throttle(2.0, 'Status diagnostic only: length=%d header=%r. '
                                   'Confirm competition field offsets before setting layout_confirmed.',
                                   len(raw), raw[:11])
            return None
        return parser(raw)

    receiver = Receiver(rospy.get_param('~bind_ip', '0.0.0.0'),
                        rospy.get_param('~port', 9082), parser=parse,
                        source_ip=rospy.get_param('~source_ip', ''), on_error=rospy.logwarn)
    rospy.on_shutdown(receiver.close)
    publisher = rospy.Publisher('/vehicle_status', VehicleStatus, queue_size=1)
    timeout = float(rospy.get_param('~timeout', 0.5))
    sequence = 0
    while not rospy.is_shutdown():
        value, current = receiver.snapshot(timeout)
        if value is None:
            rospy.logwarn_throttle(2.0, 'No fresh validated vehicle status')
        elif current != sequence:
            sequence = current
            publisher.publish(VehicleStatus(vel_x=value.vel_x, yaw=value.yaw))
        time.sleep(0.01)


if __name__ == '__main__':
    main()
