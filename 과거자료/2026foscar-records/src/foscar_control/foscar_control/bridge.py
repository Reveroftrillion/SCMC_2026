"""Receive only the three allowed channels; sole 50Hz UDP command sender."""
import math
import select
import signal
import socket
import struct
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from foscar_interfaces.msg import ControlCommand, VehicleStatus

from .core import SteeringMap, ctrl_packet, handoff_packet, parse_gps, parse_imu, parse_status


class Bridge(Node):
    def __init__(self):
        super().__init__('sensor_bridge')
        defaults = dict(bind_ip='0.0.0.0', sim_ip='192.168.10.1', gps_port=9102,
                        imu_port=9104, status_port=9099, ctrl_port=9093,
                        drive=False, command_timeout=0.25, sensor_timeout=1.5, accel_limit=0.4,
                        steering_angles_deg=[0., 15.2, 34.9], steering_commands=[0., 0.5, 1.])
        self.p = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        self.calibration = SteeringMap(self.p['steering_angles_deg'], self.p['steering_commands'])
        self.gps_pub = self.create_publisher(NavSatFix, '/sensors/gps', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/sensors/imu', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(VehicleStatus, '/vehicle/status', qos_profile_sensor_data)
        self.create_subscription(ControlCommand, '/control/request', self.command, 1)
        self.lock = threading.Lock()
        self.latest_command = None
        self.last_valid = {name: 0. for name in ('gps', 'imu', 'status')}
        self.source_stamps = {}
        self.errors = {name: 0 for name in self.last_valid}
        self.received = {name: 0 for name in self.last_valid}
        self.sockets = {}
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destination = (self.p['sim_ip'], self.p['ctrl_port'])
        self.stop_event = threading.Event()
        self.threads = []
        self.sent_control = False
        try:
            for name in self.last_valid:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sockets[sock] = name
                sock.bind((self.p['bind_ip'], self.p[name+'_port']))
                sock.setblocking(False)
            for callback in (self.receive, self.transmit):
                thread = threading.Thread(target=callback, daemon=True)
                self.threads.append(thread)
                thread.start()
        except Exception as exc:
            self.get_logger().error(f'sensor bridge initialization failed: {type(exc).__name__}: {exc}')
            self.close()
            raise
        self.create_timer(3., self.report)
        self.get_logger().info(f'GPS :{self.p["gps_port"]}, IMU :{self.p["imu_port"]}, status :{self.p["status_port"]}; drive={self.p["drive"]}')

    def command(self, msg):
        age = (self.get_clock().now().nanoseconds - (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        valid = (-0.05 <= age <= self.p['command_timeout']
                 and all(math.isfinite(v) for v in (msg.accel, msg.brake, msg.steering_rad))
                 and 0 <= msg.accel <= self.p['accel_limit'] and 0 <= msg.brake <= 1
                 and abs(msg.steering_rad) <= math.radians(35)
                 and not (msg.accel > 0 and msg.brake > 0))
        with self.lock:
            self.latest_command = (time.monotonic(), msg) if valid else None

    def fresh_source(self, name, stamp):
        # A repeated simulator timestamp must not keep stale measurements alive.
        if self.source_stamps.get(name) == stamp:
            return False
        self.source_stamps[name] = stamp
        return True

    def receive(self):
        while not self.stop_event.is_set():
            try:
                ready = select.select(list(self.sockets), [], [], 0.05)[0]
                for sock in ready:
                    raw, source = sock.recvfrom(65535)
                    name = self.sockets[sock]
                    if source[0] != self.p['sim_ip']:
                        continue
                    self.received[name] += 1
                    try:
                        valid = self.publish_packet(name, raw)
                        if valid:
                            # An invalid fix (tunnel) leaves the last valid time alone so the report
                            # shows how long GPS has been gone instead of resetting to 'never'.
                            with self.lock:
                                self.last_valid[name] = time.monotonic()
                    except (ValueError, IndexError, UnicodeError, OverflowError, struct.error) as exc:
                        self.errors[name] += 1
                        if self.errors[name] == 1:
                            self.get_logger().error(f'{name}: {exc}')
            except (OSError, ValueError):
                if not self.stop_event.is_set():
                    self.get_logger().error('UDP receive failed; watchdog will brake')
                break
            except Exception as exc:
                # ROS context can close between a packet arrival and publish on Ctrl+C.
                if rclpy.ok() and not self.stop_event.is_set():
                    self.get_logger().error(f'UDP publishing stopped: {exc}')
                break

    def publish_packet(self, name, raw):
        stamp = self.get_clock().now().to_msg()
        if name == 'gps':
            result = None
            for fix in parse_gps(raw):
                if not self.fresh_source('gps_'+fix['kind'], fix['utc']):
                    continue
                msg = NavSatFix()
                msg.header.stamp, msg.header.frame_id = stamp, 'gps_link'
                msg.status.service = NavSatStatus.SERVICE_GPS
                msg.status.status = NavSatStatus.STATUS_FIX if fix['valid'] else NavSatStatus.STATUS_NO_FIX
                if fix['valid']:
                    msg.latitude, msg.longitude, msg.altitude = fix['lat'], fix['lon'], fix['alt']
                else:
                    msg.latitude = msg.longitude = msg.altitude = float('nan')
                self.gps_pub.publish(msg)
                result = fix['valid']
            return result
        if name == 'imu':
            q, angular, acceleration, sim_time = parse_imu(raw)
            if not self.fresh_source(name, sim_time):
                return None
            msg = Imu()
            msg.header.stamp, msg.header.frame_id = stamp, 'imu_link'
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = map(float, q)
            # Only orientation is consumed in this first version. Raw vector axis
            # conventions still need verification, so do not publish them as ENU.
            msg.angular_velocity_covariance[0] = -1.
            msg.linear_acceleration_covariance[0] = -1.
            self.imu_pub.publish(msg)
            return True
        data = parse_status(raw)
        if not self.fresh_source(name, raw[27:35]):
            return None
        msg = VehicleStatus()
        msg.header.stamp, msg.header.frame_id = stamp, 'base_link'
        msg.speed_mps, msg.yaw_rad = data['speed'], data['yaw']
        msg.ctrl_mode, msg.gear = data['mode'], data['gear']
        msg.steer_feedback_deg, msg.link_id = data['steer_deg'], data['link']
        self.status_pub.publish(msg)
        return True

    def transmit(self):
        previous_steer = 0.
        deadline = time.monotonic()
        while not self.stop_event.is_set():
            if self.p['drive']:
                now = time.monotonic()
                with self.lock:
                    latest = self.latest_command
                    # GPS is deliberately NOT in this watchdog: the tunnel (s=1881~1998) blacks it out
                    # and localization dead-reckons through. Braking here would override the tracker.
                    healthy = all(now-self.last_valid[k] < self.p['sensor_timeout'] for k in ('imu', 'status'))
                accel, brake, steer = 0., 1., 0.
                if latest is not None and healthy and now-latest[0] < self.p['command_timeout']:
                    msg = latest[1]
                    accel, brake = msg.accel, msg.brake
                    steer = self.calibration.command(msg.steering_rad)
                # Maximum normalized steering slew 1.0/s at 50Hz.
                steer = max(previous_steer-0.02, min(previous_steer+0.02, steer))
                try:
                    self.tx.sendto(ctrl_packet(accel, brake, steer), self.destination)
                    self.sent_control = True
                except OSError as exc:
                    self.get_logger().error(f'UDP send failed: {exc}')
                previous_steer = steer
            deadline += 0.02
            if deadline < time.monotonic():
                deadline = time.monotonic()
            self.stop_event.wait(max(0., deadline-time.monotonic()))

    def report(self):
        now = time.monotonic()
        with self.lock:
            ages = {k: round(now-v, 2) if v else None for k, v in self.last_valid.items()}
        self.get_logger().info(f'RX={self.received} parse_errors={self.errors} age_s={ages}')

    def handoff(self):
        # Receiver thread is joined; consume fresh status directly during shutdown.
        rx = next((sock for sock, name in self.sockets.items() if name == 'status'), None)
        status, arrival = None, 0.
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            if rx is not None:
                while True:
                    try:
                        raw, source = rx.recvfrom(65535)
                    except (BlockingIOError, OSError):
                        break
                    if source[0] != self.p['sim_ip']:
                        continue
                    try:
                        parsed = parse_status(raw)
                    except (ValueError, struct.error, UnicodeError):
                        continue
                    if self.fresh_source('status', raw[27:35]):
                        status, arrival = parsed, time.monotonic()
            fresh = status if time.monotonic()-arrival < .25 else None
            try:
                self.tx.sendto(handoff_packet(fresh), self.destination)
            except OSError as exc:
                self.get_logger().error(f'shutdown handoff send failed: {exc}')
                return
            if fresh and fresh['mode'] == 1 and fresh['gear'] == 1 and abs(fresh['speed']) < .1:
                message = 'shutdown handoff confirmed: Manual/P, brake released'
                if rclpy.ok():
                    self.get_logger().info(message)
                else:
                    print(message, flush=True)
                return
            time.sleep(.02)
        # Do not leave a release command latched if acknowledgement never arrived.
        self.tx.sendto(ctrl_packet(0., 1., 0.), self.destination)
        self.get_logger().warn('shutdown handoff not confirmed within 3s; brake held')

    def close(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=1.)
        try:
            # A second launch that failed to bind must not interfere with the owner.
            if self.sent_control:
                self.handoff()
        finally:
            for sock in self.sockets:
                sock.close()
            self.tx.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Bridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch/group shutdown can deliver SIGINT twice; finish the bounded handoff.
        previous_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if node is not None:
                node.close()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous_sigint)
