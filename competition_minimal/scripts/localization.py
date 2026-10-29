#!/usr/bin/env python3
"""GPS/IMU/speed localization. All arrival freshness uses a monotonic clock."""
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu
from simul_msgs.msg import VehicleStatus
from std_msgs.msg import Bool, Float64
from pyproj import Transformer

from localization_core import CourseProgress, Estimator


def read_path(path):
    points = []
    with open(path, encoding='utf-8-sig') as source:
        for line in source:
            row = line.split()
            if row and not row[0].startswith('#'):
                points.append((float(row[0]), float(row[1])))
    return points


def quaternion_yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        return None
    norm = math.sqrt(sum(v*v for v in values))
    if norm < 0.5 or norm > 1.5:
        return None
    x, y, z, w = (v/norm for v in values)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class LocalizationNode:
    def __init__(self):
        self.lock = threading.Lock()
        self.transformer = Transformer.from_crs('EPSG:4326', 'EPSG:32652', always_xy=True)
        self.east = float(rospy.get_param('~east_offset', 302595.0))
        self.north = float(rospy.get_param('~north_offset', 4124145.0))
        self.yaw_offset = math.radians(float(rospy.get_param('~imu_yaw_offset_deg', 0.0)) +
                                       float(rospy.get_param('~imu_grid_convergence_deg', -1.35)))
        self.sensor_timeout = float(rospy.get_param('~sensor_timeout', 0.5))
        self.course = CourseProgress(read_path(rospy.get_param('~path_file')),
                                     float(rospy.get_param('~max_path_error', 8.0)))
        self.est = Estimator(max_outage=float(rospy.get_param('~max_dr_outage', 90.0)),
                             blend_time=float(rospy.get_param('~gps_blend_time_s', 2.0)))
        self.gps = self.imu = self.status = None
        self.gps_seq = self.used_gps_seq = 0
        self.pose_pub = rospy.Publisher('/competition/pose', PoseStamped, queue_size=1)
        self.progress_pub = rospy.Publisher('/competition/progress', Float64, queue_size=1)
        self.health_pub = rospy.Publisher('/competition/localization_ok', Bool, queue_size=1)
        self.dr_pub = rospy.Publisher('/competition/dr_active', Bool, queue_size=1)
        rospy.Subscriber('/gps', GPSMessage, self.on_gps, queue_size=1)
        rospy.Subscriber('/imu', Imu, self.on_imu, queue_size=1)
        rospy.Subscriber('/vehicle_status', VehicleStatus, self.on_status, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.tick)
        rospy.logwarn('IMU combined yaw correction %.2f degrees; verify IMU reference, convergence sign and GPS antenna against simulator', math.degrees(self.yaw_offset))

    def on_gps(self, msg):
        if (msg.status <= 0 or not all(math.isfinite(v) for v in (msg.latitude, msg.longitude))
                or not (35.0 <= msg.latitude <= 39.0 and 125.0 <= msg.longitude <= 130.0)):
            return
        x, y = self.transformer.transform(msg.longitude, msg.latitude)
        if not math.isfinite(x) or not math.isfinite(y):
            return
        with self.lock:
            self.gps = (x-self.east, y-self.north, time.monotonic())
            self.gps_seq += 1

    def on_imu(self, msg):
        yaw = quaternion_yaw(msg.orientation)
        if yaw is not None:
            with self.lock:
                self.imu = (yaw+self.yaw_offset, time.monotonic())

    def on_status(self, msg):
        if math.isfinite(msg.vel_x) and abs(msg.vel_x) <= 250:
            with self.lock:
                self.status = (msg.vel_x/3.6, time.monotonic())

    def tick(self, _event):
        now = time.monotonic()
        with self.lock:
            if (self.imu is None or self.status is None
                    or now-self.imu[1] > self.sensor_timeout
                    or now-self.status[1] > self.sensor_timeout):
                self.health_pub.publish(Bool(data=False))
                rospy.logwarn_throttle(2.0, 'Waiting for fresh IMU and vehicle speed')
                return
            yaw, speed = self.imu[0], self.status[0]
            if self.est.x is not None and not self.est.advance(now, speed, yaw):
                # A fresh close fix can restart integration without a position jump.
                fresh = (self.gps_seq != self.used_gps_seq and self.gps is not None
                         and now-self.gps[2] <= self.sensor_timeout)
                if fresh:
                    self.used_gps_seq = self.gps_seq
                if not fresh or not self.est.recover(self.gps[0], self.gps[1], now):
                    self.health_pub.publish(Bool(data=False))
                    rospy.logerr_throttle(2.0, 'DR timeout: waiting for a new GPS fix close to last pose')
                    return
                rospy.logwarn('Localization resumed from a nearby fresh GPS fix')
            if self.gps_seq != self.used_gps_seq:
                self.used_gps_seq = self.gps_seq
                if now-self.gps[2] <= self.sensor_timeout:
                    if not self.est.fix(self.gps[0], self.gps[1], now):
                        rospy.logerr_throttle(2.0, 'GPS fix rejected: too far from DR estimate')
            if self.est.x is None or not self.course.update(self.est.x, self.est.y):
                self.health_pub.publish(Bool(data=False))
                rospy.logwarn_throttle(2.0, 'No initial GPS fix or pose too far from expected path')
                return
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = self.est.x, self.est.y
            pose.pose.orientation.z = math.sin(yaw/2.0)
            pose.pose.orientation.w = math.cos(yaw/2.0)
            self.pose_pub.publish(pose)
            self.progress_pub.publish(Float64(data=self.course.progress))
            self.dr_pub.publish(Bool(data=now-self.est.last_fix > self.sensor_timeout))
            self.health_pub.publish(Bool(data=True))
            rospy.loginfo_throttle(2., 'Localization x=%.2f y=%.2f yaw=%.1f s=%.1f DR=%s',
                                   self.est.x, self.est.y, math.degrees(yaw), self.course.progress,
                                   now-self.est.last_fix > self.sensor_timeout)


def main():
    rospy.init_node('competition_localization')
    LocalizationNode()
    rospy.spin()


if __name__ == '__main__':
    main()
