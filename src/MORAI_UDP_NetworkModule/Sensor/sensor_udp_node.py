#!/usr/bin/env python3
import sys
from pathlib import Path
import rospkg
sys.path.insert(0, rospkg.RosPack().get_path('morai_udp'))

import time
import rospy
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import CompressedImage, Imu
from lib.network.UDP import Receiver
from lib.network.parsers import GPSParser, CameraParser, imu_parser


def main():
    rospy.init_node('sensor_udp')
    kind = rospy.get_param('~kind')
    if kind == 'gps':
        parser, msg_type = GPSParser(), GPSMessage
    elif kind == 'imu':
        parser, msg_type = imu_parser(rospy.get_param('~layout', 'timestamped_115')), Imu
    elif kind == 'camera':
        parser, msg_type = CameraParser(), CompressedImage
    else:
        raise ValueError('unknown sensor kind: ' + kind)
    timeout = float(rospy.get_param('~timeout', 0.5))
    topic = rospy.get_param('~topic')
    publisher = rospy.Publisher(topic, msg_type, queue_size=1)
    frame = rospy.get_param('~frame_id', kind)
    receiver = Receiver(rospy.get_param('~bind_ip', '0.0.0.0'),
                        rospy.get_param('~port'), parser=parser,
                        source_ip=rospy.get_param('~source_ip', ''), on_error=rospy.logwarn)
    rospy.on_shutdown(receiver.close)
    rospy.loginfo('%s UDP port %s -> %s', kind, rospy.get_param('~port'), topic)
    sequence = 0
    while not rospy.is_shutdown():
        value, current = receiver.snapshot(timeout)
        if value is None:
            rospy.logwarn_throttle(2.0, 'No fresh %s UDP data on %s', kind, topic)
        elif current != sequence:
            sequence = current
            msg = msg_type()
            # Arrival time in the local ROS clock; no assumption about simulator epoch.
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frame
            if kind == 'gps':
                msg.latitude, msg.longitude, msg.altitude, msg.status = value
                msg.eastOffset = rospy.get_param('~east_offset', 302595.0)
                msg.northOffset = rospy.get_param('~north_offset', 4124145.0)
            elif kind == 'imu':
                msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = value[:4]
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = value[4:7]
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = value[7:]
            else:
                msg.format, msg.data = 'jpeg', value
            publisher.publish(msg)
        time.sleep(0.005)


if __name__ == '__main__':
    main()
