#!/usr/bin/env python3
"""Publish a foscar_interfaces/Event on /events at 10 Hz — stand-in for a perception node so the FSM
(behavior_node) can be exercised without lidar/camera.

  python3 tools/fake_event.py obstacle 329            # static 2x3 m box on the route at s=329 (map frame)
  python3 tools/fake_event.py obstacle 329 d 0.8      # ... shifted 0.8 m left of the route
  python3 tools/fake_event.py obstacle 329 size 1 7   # a wall across the road -> AVOID must stop (blocked)
  python3 tools/fake_event.py pedestrian 460 d 4 vy -1.4
  python3 tools/fake_event.py light 315 RED           # TRAFFIC_LIGHT event (only matters with a stop_line state)
The object is placed with /map/route + /localization/s, so path_follower and localization must be running.
Ctrl+C stops publishing; behavior_node forgets the event 0.5 s later.
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

from foscar_control.core import Route
from foscar_planning.frenet import s_offset_from
from foscar_interfaces.msg import Event
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64

TYPES = {'obstacle': 'OBSTACLE', 'pedestrian': 'PEDESTRIAN', 'vehicle': 'VEHICLE', 'light': 'TRAFFIC_LIGHT'}


def parse(args):
    if len(args) < 2 or args[0] not in TYPES:
        sys.exit(__doc__)
    spec = dict(type=TYPES[args[0]], s=float(args[1]), d=0., size=(3., 2.), v=(0., 0.), value='')
    if spec['type'] == 'TRAFFIC_LIGHT':
        spec['value'] = args[2] if len(args) > 2 else 'RED'
        return spec
    i = 2
    while i < len(args):
        key = args[i]
        if key == 'd':
            spec['d'], i = float(args[i+1]), i+2
        elif key == 'size':
            spec['size'], i = (float(args[i+1]), float(args[i+2])), i+3
        elif key == 'vy':
            spec['v'], i = (0., float(args[i+1])), i+2
        else:
            sys.exit(__doc__)
    return spec


class FakeEvent(Node):
    def __init__(self, spec):
        super().__init__('fake_event')
        self.spec = spec
        self.route = self.s_offset = self.s_global = self.odom = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/map/route', self.on_route, latched)
        self.create_subscription(Float64, '/localization/s', self.on_s, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/localization/odometry', self.on_odom, qos_profile_sensor_data)
        self.pub = self.create_publisher(Event, '/events', 10)
        self.create_timer(0.1, self.tick)
        self.t = 0.

    def on_route(self, msg):
        self.route = Route([(q.pose.position.x, q.pose.position.y) for q in msg.poses])

    def on_s(self, msg):
        self.s_global = msg.data

    def on_odom(self, msg):
        self.odom = msg

    def tick(self):
        msg = Event()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type, msg.s, msg.value, msg.confidence = self.spec['type'], self.spec['s'], self.spec['value'], 1.
        if msg.type != 'TRAFFIC_LIGHT':
            if self.route is None or self.s_global is None or self.odom is None:
                self.get_logger().info('waiting for /map/route, /localization/s, /localization/odometry', throttle_duration_sec=2.)
                return
            if self.s_offset is None:
                o = self.odom.pose.pose.position
                self.s_offset = s_offset_from(self.route, o.x, o.y, self.s_global)
            self.t += 0.1
            s_local = self.spec['s'] + self.s_offset
            d = self.spec['d'] + self.spec['v'][1] * self.t
            x, y = self.route.at(s_local)
            h = self.route.heading_at(s_local)
            msg.header.frame_id = 'map'
            msg.x, msg.y = x - math.sin(h) * d, y + math.cos(h) * d
            msg.size_x, msg.size_y = self.spec['size']
            msg.vx, msg.vy = -math.sin(h) * self.spec['v'][1], math.cos(h) * self.spec['v'][1]
        self.pub.publish(msg)


def main():
    spec = parse(sys.argv[1:])
    rclpy.init()
    node = FakeEvent(spec)
    node.get_logger().info(f'publishing {spec}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
