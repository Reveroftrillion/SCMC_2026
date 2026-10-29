#!/usr/bin/env python3
"""Read-only ROS stream summary; run alongside the receive-only launch."""
import argparse
import math
import threading
import time


def summarize(count, first, last, now, timeout=1.0):
    if count <= 0 or last is None:
        return 'MISSING', 0.0, None
    age = now-last
    hz = (count-1)/(last-first) if count > 1 and last > first else 0.0
    return ('FRESH' if 0 <= age <= timeout else 'STALE'), hz, age


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from morai_msgs.msg import GPSMessage
    from sensor_msgs.msg import CompressedImage, Imu, PointCloud2
    from simul_msgs.msg import ControlCmd, VehicleStatus
    from std_msgs.msg import Bool, Float64

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10.)
    options = parser.parse_args(rospy.myargv()[1:])
    if options.seconds <= 0: raise SystemExit('--seconds must be positive')
    rospy.init_node('competition_live_status', anonymous=True, disable_signals=True)
    topics = [('/gps', GPSMessage), ('/imu', Imu), ('/vehicle_status', VehicleStatus),
              ('/camera/traffic/image/compressed', CompressedImage), ('/velodyne_points', PointCloud2),
              ('/competition/pose', PoseStamped), ('/competition/progress', Float64),
              ('/competition/localization_ok', Bool), ('/competition/dr_active', Bool),
              ('/competition/traffic_stop', Bool), ('/competition/traffic_speed_limit', Float64),
              ('/competition/lidar_stop', Bool), ('/competition/lidar_speed_limit', Float64),
              ('/competition/lateral_offset', Float64), ('/competition/control_cmd', ControlCmd)]
    state, lock = {}, threading.Lock()
    def receive(msg, topic):
        now = time.monotonic()
        with lock:
            entry = state.setdefault(topic, [0, now, now, None])
            entry[0] += 1; entry[2] = now; entry[3] = msg
    subscribers = [rospy.Subscriber(name, typ, receive, callback_args=name, queue_size=1,
                                    buff_size=2**24) for name, typ in topics]
    deadline = time.monotonic()+options.seconds
    while time.monotonic() < deadline and not rospy.is_shutdown(): time.sleep(.05)
    now = time.monotonic()
    print('ROS topic                     state     Hz   age(s)  latest')
    with lock:
        for name, _ in topics:
            count, first, last, msg = state.get(name, (0, None, None, None))
            status, hz, age = summarize(count, first, last, now)
            detail = ''
            if msg is not None:
                if name == '/gps': detail = 'fix=%s lat=%.7f lon=%.7f' % (msg.status, msg.latitude, msg.longitude)
                elif name == '/imu':
                    q = msg.orientation
                    yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
                    detail = 'heading=%.1f deg' % yaw
                elif name == '/vehicle_status': detail = 'speed=%.1f km/h yaw=%.1f deg' % (msg.vel_x,msg.yaw)
                elif name == '/competition/pose': detail = 'x=%.2f y=%.2f' % (msg.pose.position.x,msg.pose.position.y)
                elif name == '/competition/control_cmd': detail = 'accel=%.2f brake=%.2f steer=%.2f' % (msg.accel,msg.brake,msg.steering)
                elif hasattr(msg, 'data') and name != '/camera/traffic/image/compressed': detail = str(msg.data)
                elif name == '/camera/traffic/image/compressed':
                    import cv2
                    import numpy as np
                    frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                    detail = 'JPEG decode failed' if frame is None else '%dx%d' % (frame.shape[1], frame.shape[0])
                elif name == '/velodyne_points': detail = 'frame=%s points=%d' % (msg.header.frame_id, msg.width*msg.height)
            print('%-29s %-7s %5.1f %7s  %s' % (name, status, hz, '-' if age is None else '%.2f'%age, detail))
    enabled = rospy.get_param('/competition_udp_bridge/enable_control', None)
    confirmed = rospy.get_param('/competition_udp_bridge/status_layout_confirmed', None)
    print('UDP control sender configured: %s; status layout confirmed: %s' % (enabled, confirmed))
    print('When sender is disabled no control datagrams should be emitted; confirm on the wire if needed.')
    print('For 181-byte status mode/gear and YOLO labels check competition_udp_bridge and competition_traffic logs.')
    print('Freshness does not establish sensor calibration, signal correctness, Auto/D or driving safety.')
    for sub in subscribers: sub.unregister()


if __name__ == '__main__': main()
