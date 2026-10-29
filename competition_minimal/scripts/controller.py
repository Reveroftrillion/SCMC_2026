#!/usr/bin/env python3
"""Final arbitration: any stale input or stop request overrides propulsion."""
import math
import threading
import time
import rospy
from geometry_msgs.msg import PoseStamped
from simul_msgs.msg import ControlCmd, VehicleStatus
from std_msgs.msg import Bool, Float64
from control_core import Driver, valid_zones, uncertainty_limit
from localization import read_path, quaternion_yaw


class ControlNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.values = {}
        self.driver = Driver(read_path(rospy.get_param('~path_file')),
                             max_speed=float(rospy.get_param('~max_speed_kmh', 40.0))/3.6)
        self.timeout = float(rospy.get_param('~input_timeout', .7))
        self.sensor_reference_verified = bool(rospy.get_param('~sensor_reference_verified', False))
        self.mission_zones_verified = (bool(rospy.get_param('~mission_zones_verified', False)) and
                                       valid_zones(rospy.get_param('~zones', {}), self.driver.s[-1]))
        self.unverified_sections = rospy.get_param('~unverified_sections', None)
        if rospy.get_param('~unverified_sections_required', True) and not self.unverified_sections:
            self.mission_zones_verified = False
        self.pub = rospy.Publisher('/competition/control_cmd', ControlCmd, queue_size=1)
        self.finished_pub = rospy.Publisher('/competition/course_finished', Bool, queue_size=1)
        definitions = (('pose', '/competition/pose', PoseStamped),
                       ('progress', '/competition/progress', Float64),
                       ('localization_ok', '/competition/localization_ok', Bool),
                       ('speed', '/vehicle_status', VehicleStatus),
                       ('traffic_stop', '/competition/traffic_stop', Bool),
                       ('traffic_limit', '/competition/traffic_speed_limit', Float64),
                       ('lidar_stop', '/competition/lidar_stop', Bool),
                       ('lidar_limit', '/competition/lidar_speed_limit', Float64),
                       ('offset', '/competition/lateral_offset', Float64),
                       ('dr', '/competition/dr_active', Bool))
        self.subscribers = [rospy.Subscriber(topic, typ, self.receive, callback_args=name, queue_size=1)
                            for name, topic, typ in definitions]
        self.last_tick = time.monotonic()
        rospy.on_shutdown(self.shutdown)
        rospy.Timer(rospy.Duration(.05), self.tick)

    def receive(self, msg, name):
        with self.lock: self.values[name] = (msg, time.monotonic())

    def publish(self, a, b, s):
        self.pub.publish(ControlCmd(accel=a, brake=b, steering=s))
        self.finished_pub.publish(Bool(data=self.driver.finished))

    def shutdown(self):
        self.publish(0., 1., 0.)

    def tick(self, _event):
        now = time.monotonic()
        with self.lock:
            dt, self.last_tick = now-self.last_tick, now
            required = ('pose', 'progress', 'localization_ok', 'speed', 'traffic_stop',
                        'traffic_limit', 'lidar_stop', 'lidar_limit', 'offset', 'dr')
            missing = [key for key in required if key not in self.values or
                       not 0 <= now-self.values[key][1] <= self.timeout]
            if not self.sensor_reference_verified or not self.mission_zones_verified:
                self.driver.reset(); self.publish(0., 1., 0.)
                rospy.logerr_throttle(5., 'Stop: sensor reference or mission zones are not verified')
                return
            if missing:
                self.driver.reset(); self.publish(0., 1., 0.)
                rospy.logwarn_throttle(2., 'Stop: missing or stale %s', ','.join(missing))
                return
            v = {key: self.values[key][0] for key in required}
            p = v['pose'].pose
            yaw = quaternion_yaw(p.orientation)
            traffic_limit, lidar_limit = v['traffic_limit'].data, v['lidar_limit'].data
            limits_valid = (not math.isnan(traffic_limit) and traffic_limit >= 0 and
                            not math.isnan(lidar_limit) and lidar_limit >= 0)
            if yaw is None or not limits_valid or v['pose'].header.frame_id != 'map':
                self.driver.reset(); self.publish(0., 1., 0.); return
            speed_limit = min(traffic_limit, lidar_limit, 15/3.6 if v['dr'].data else float('inf'))
            speed_limit = min(speed_limit, uncertainty_limit(self.unverified_sections, v['progress'].data,
                                                              self.driver.s[-1]))
            stop = not v['localization_ok'].data or v['traffic_stop'].data or v['lidar_stop'].data
            a, b, steer = self.driver.update(p.position.x, p.position.y, yaw, v['progress'].data,
                                              v['speed'].vel_x/3.6, v['offset'].data,
                                              speed_limit, stop, dt)
            self.publish(a, b, steer)
            rospy.loginfo_throttle(1., 'Control s=%.1f speed=%.1fkm/h limit=%.1fkm/h offset=%.2f accel=%.2f brake=%.2f steer=%.2f DR=%s',
                                   v['progress'].data, v['speed'].vel_x, speed_limit*3.6,
                                   v['offset'].data, a, b, steer, v['dr'].data)
            if self.driver.finished:
                rospy.logwarn_throttle(5., 'Course finished: brake held. Transfer to manual control in MORAI before terminating.')
            elif stop:
                rospy.logwarn_throttle(2., 'Stop: localization=%s traffic=%s lidar=%s',
                                       v['localization_ok'].data, v['traffic_stop'].data, v['lidar_stop'].data)


if __name__ == '__main__':
    rospy.init_node('competition_controller')
    ControlNode()
    rospy.spin()
