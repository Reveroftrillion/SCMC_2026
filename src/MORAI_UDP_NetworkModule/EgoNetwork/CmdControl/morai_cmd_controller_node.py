#!/usr/bin/env python3
import sys
from pathlib import Path
import rospkg
sys.path.insert(0, rospkg.RosPack().get_path('morai_udp'))

import math
import threading
import time
import rospy
from lib.network.UDP import Sender
from lib.network.parsers import command_values, watchdog_values
from lib.define.EgoCtrlCmd import EgoCtrlCmd
from simul_msgs.msg import ControlCmd, VehicleStatus
from morai_msgs.msg import GPSMessage


class MoraiCmdController:
    def __init__(self):
        rospy.init_node('morai_cmd_controller')
        self.sender = Sender(
            rospy.get_param('~morai_ip'),
            rospy.get_param('~port', 9093),
            source_ip=rospy.get_param('~bind_ip', '0.0.0.0'),
            source_port=rospy.get_param('~source_port', 9094),
        )
        self.enabled = rospy.get_param('~enabled', False)
        self.timeout = float(rospy.get_param('~command_timeout', 0.3))
        self.status_timeout = float(rospy.get_param('~status_timeout', 0.5))
        hz = float(rospy.get_param('~send_hz', 50.0))
        if not all(math.isfinite(x) and x > 0 for x in (hz, self.timeout, self.status_timeout)):
            raise ValueError('send rate and timeouts must be positive and finite')
        self.period = 1.0 / hz
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.values, self.last_command, self.last_status = None, None, None
        self.last_gps = None
        self.data = EgoCtrlCmd()
        self.data.ctrl_mode, self.data.cmd_type = 2, 1
        self.data.gear = 4
        self.control_sub = rospy.Subscriber('/control_cmd', ControlCmd, self.controlCmdCallback, queue_size=1)
        self.status_sub = rospy.Subscriber('/vehicle_status', VehicleStatus, self.status_callback, queue_size=1)
        self.gps_sub = rospy.Subscriber('/gps', GPSMessage, self.gps_callback, queue_size=1)
        self.worker = threading.Thread(target=self.send_loop, daemon=True)
        rospy.on_shutdown(self.close)
        self.worker.start()
        rospy.loginfo('UDP control enabled=%s, destination=%s:%s',
                      self.enabled, self.sender.ip, self.sender.port)

    def controlCmdCallback(self, msg):
        try:
            values = command_values(msg.accel, msg.brake, msg.steering)
        except ValueError as exc:
            with self.lock:
                self.values, self.last_command = None, None
            rospy.logwarn_throttle(2.0, 'Rejected control command: %s', exc)
            return
        with self.lock:
            self.values, self.last_command = values, time.monotonic()

    def status_callback(self, msg):
        if math.isfinite(msg.vel_x) and math.isfinite(msg.yaw):
            with self.lock:
                self.last_status = time.monotonic()

    def send_loop(self):
        # Wall-clock watchdog continues even if /clock stalls.
        while not self.stop.is_set():
            if self.enabled:
                with self.lock:
                    # Blackout reports are still live GPS messages. Missing packets are not.
                    state_time = (min(self.last_status, self.last_gps)
                                  if self.last_status is not None and self.last_gps is not None else None)
                    values = watchdog_values(self.values, self.last_command, state_time,
                                             time.monotonic(), self.timeout, self.status_timeout)
                self.data.accel, self.data.brake, self.data.steer = values
                try:
                    self.sender.send(self.data)
                except OSError as exc:
                    rospy.logwarn_throttle(2.0, 'UDP command send failed: %s', exc)
            self.stop.wait(self.period)

    def gps_callback(self, msg):
        if all(math.isfinite(x) for x in (msg.latitude, msg.longitude, msg.altitude)):
            with self.lock:
                self.last_gps = time.monotonic()

    def close(self):
        self.stop.set()
        if self.worker.is_alive():
            self.worker.join(timeout=1.0)
        if self.enabled:
            self.data.accel, self.data.brake, self.data.steer = 0.0, 1.0, 0.0
            try:
                self.sender.send(self.data)
            except OSError:
                pass
        self.sender.close()


if __name__ == '__main__':
    controller = MoraiCmdController()
    rospy.spin()
