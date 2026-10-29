#!/usr/bin/env python3
"""ROS bridge for the documented sensor and command UDP packet formats."""
import math
import select
import socket
import threading
import time

import rospy
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu, CompressedImage
from simul_msgs.msg import VehicleStatus, ControlCmd
from std_msgs.msg import Bool
from udp_protocol import CameraAssembler, ControlGate, StatusSequence, command_packet, parse_gps, parse_imu, parse_status_packet, status_diagnostic, validate_ports


class Bridge:
    def __init__(self):
        self.ip = rospy.get_param('~morai_ip')
        self.bind_ip = rospy.get_param('~bind_ip', '0.0.0.0')
        self.enabled = bool(rospy.get_param('~enable_control', False))
        self.confirmed = bool(rospy.get_param('~status_layout_confirmed', False))
        self.imu_layout = rospy.get_param('~imu_layout', 'timestamped_115')
        self.lock = threading.Lock()
        self.command = self.command_time = self.status_time = None
        self.gate = ControlGate()
        self.status_sequence = StatusSequence()
        self.finished = False
        self.camera = CameraAssembler()
        self.pubs = {'gps': rospy.Publisher('/gps', GPSMessage, queue_size=1),
                     'imu': rospy.Publisher('/imu', Imu, queue_size=1),
                     'status': rospy.Publisher('/vehicle_status', VehicleStatus, queue_size=1),
                     'camera': rospy.Publisher('/camera/traffic/image/compressed', CompressedImage, queue_size=1)}
        self.sockets = {}
        ports = {kind: int(rospy.get_param('~'+kind+'_port', default)) for kind, default in
                 (('gps', 1111), ('imu', 1112), ('status', 909), ('camera', 9297),
                  ('command_source', 9094), ('lidar', 2368))}
        validate_ports(ports)
        for kind, default in (('gps', 1111), ('imu', 1112), ('status', 909), ('camera', 9297)):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2**20)
            sock.bind((self.bind_ip, ports[kind]))
            self.sockets[sock] = kind
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx.bind((self.bind_ip, ports['command_source']))
        self.destination = (self.ip, int(rospy.get_param('~command_port', 9093)))
        rospy.Subscriber('/competition/control_cmd', ControlCmd, self.on_command, queue_size=1)
        rospy.Subscriber('/competition/course_finished', Bool, self.on_finished, queue_size=1)
        self.stopping = threading.Event()
        self.sender = threading.Thread(target=self.send_loop, daemon=True)
        self.sender.start()
        rospy.on_shutdown(self.close)
        if not self.confirmed:
            rospy.logwarn('Competition status may be observed, but control remains disabled until status_layout_confirmed=true')
        rospy.logwarn('UDP sender enabled=%s; confirm 181-byte Competition status against live MORAI before control', self.enabled)
        rospy.loginfo('UDP local ports %s, command destination=%s', ports, self.destination)

    def on_finished(self, msg):
        with self.lock:
            self.finished = self.finished or bool(msg.data)

    def on_command(self, msg):
        values = (msg.accel, msg.brake, msg.steering)
        with self.lock:
            self.command = values if all(math.isfinite(v) for v in values) else None
            self.command_time = time.monotonic() if self.command is not None else None

    def send_loop(self):
        while not self.stopping.is_set():
            with self.lock:
                values = self.gate.output(self.command, self.command_time, time.monotonic(), self.finished)
            if self.enabled and self.confirmed and values is not None:
                try: self.tx.sendto(command_packet(*values), self.destination)
                except OSError as exc: rospy.logerr_throttle(2., 'UDP send failure: %s', exc)
            self.stopping.wait(.02)

    def receive(self, kind, raw):
        if kind == 'status':
            try:
                status, stamp, kind_name = parse_status_packet(raw)
                if kind_name != 'competition':
                    raise ValueError('229-byte Ego status is diagnostic only, not competition status')
                self.status_sequence.accept(stamp)
            except ValueError as exc:
                with self.lock:
                    self.status_time = None
                    self.gate.invalidate()
                rospy.logwarn_throttle(2., 'Rejected status packet: %s (%s)', status_diagnostic(raw), exc)
                raise
            # VehicleStatus is the existing ROS contract: km/h and degrees.
            self.pubs[kind].publish(VehicleStatus(vel_x=status.speed * 3.6, yaw=math.degrees(status.yaw)))
            with self.lock:
                self.status_time = time.monotonic()
                self.gate.observe(status, self.status_time)
            rospy.loginfo_throttle(5., 'Competition status: speed=%.2f m/s yaw=%.1f deg steer=%.3f deg mode=%s gear=%s timestamp=%s',
                                   status.speed, math.degrees(status.yaw), status.steer, status.mode, status.gear, stamp)
            if status.mode != 2 or status.gear != 4:
                rospy.logwarn_throttle(3., 'Vehicle is not Auto/D (mode=%s gear=%s); propulsion is disabled', status.mode, status.gear)
        elif kind == 'gps':
            fix = parse_gps(raw)
            if fix is None: return
            msg = GPSMessage(); msg.header.stamp = rospy.Time.now(); msg.header.frame_id = 'gps'
            msg.latitude, msg.longitude, msg.altitude, msg.status = fix
            msg.eastOffset, msg.northOffset = 302595., 4124145.
            self.pubs[kind].publish(msg)
        elif kind == 'imu':
            v = parse_imu(raw, self.imu_layout)
            msg = Imu(); msg.header.stamp = rospy.Time.now(); msg.header.frame_id = 'imu'
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = v[:4]
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = v[4:7]
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = v[7:10]
            self.pubs[kind].publish(msg)
        else:
            jpeg = self.camera.feed(raw)
            if jpeg is None: return
            msg = CompressedImage(); msg.header.stamp = rospy.Time.now(); msg.header.frame_id = 'camera_traffic'
            msg.format, msg.data = 'jpeg', jpeg
            self.pubs[kind].publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            try: ready, _, _ = select.select(list(self.sockets), [], [], .1)
            except (OSError, ValueError): break
            for sock in ready:
                try:
                    raw, peer = sock.recvfrom(65535)
                    if peer[0] == self.ip: self.receive(self.sockets[sock], raw)
                except (OSError, ValueError, UnicodeError, KeyError) as exc:
                    rospy.logwarn_throttle(2., 'Invalid UDP packet: %s', exc)

    def close(self):
        self.stopping.set()
        if self.sender.is_alive(): self.sender.join(timeout=.5)
        # Only brake an active Auto/D vehicle; do not seize manual control on exit.
        with self.lock:
            last = self.gate.last_status
            can_brake = (self.gate.active and last and last.mode == 2 and last.gear == 4
                         and not self.gate.complete and self.gate.handover_request is None)
        if self.enabled and self.confirmed and can_brake:
            try: self.tx.sendto(command_packet(0., 1., 0.), self.destination)
            except OSError: pass
        for sock in list(self.sockets): sock.close()
        self.tx.close()


if __name__ == '__main__':
    rospy.init_node('competition_udp_bridge')
    Bridge().run()
