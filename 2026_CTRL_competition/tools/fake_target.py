#!/usr/bin/env python3
"""Publish a DrivingTarget on /behavior/target at 20 Hz — stand-in for the FSM so the tracker's
mission hooks can be exercised without any perception code.

  python3 tools/fake_target.py stop 208          # red light: stop with front wheels at s=208
  python3 tools/fake_target.py vmax 5            # cap speed at 5 m/s (18 km/h)
  python3 tools/fake_target.py offset 1.0        # drive 1 m left of the route (obstacle avoidance)
  python3 tools/fake_target.py stop 208 vmax 5 offset -0.5   # combinations
  python3 tools/fake_target.py clear             # plain cruise (same as no target)
Ctrl+C stops publishing; with require_target=false the tracker just keeps cruising.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from foscar_interfaces.msg import DrivingTarget


def parse(args):
    target = dict(v_max=math.inf, stop_s=math.nan, lat_offset=0., state='FAKE')
    i = 0
    while i < len(args):
        key = args[i]
        if key == 'clear':
            i += 1
            continue
        if key not in ('stop', 'vmax', 'offset') or i+1 >= len(args):
            sys.exit(__doc__)
        value = float(args[i+1])
        target[{'stop': 'stop_s', 'vmax': 'v_max', 'offset': 'lat_offset'}[key]] = value
        target['state'] = 'FAKE_'+'_'.join(k.upper() for k in args[0::2] if k != 'clear') or 'FAKE'
        i += 2
    return target


def main():
    target = parse(sys.argv[1:])
    rclpy.init()
    node = Node('fake_target')
    pub = node.create_publisher(DrivingTarget, '/behavior/target', 1)

    def tick():
        msg = DrivingTarget()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.state, msg.v_max, msg.stop_s, msg.lat_offset = (target['state'], float(target['v_max']),
                                                           float(target['stop_s']), float(target['lat_offset']))
        pub.publish(msg)

    node.create_timer(0.05, tick)
    node.get_logger().info(f'publishing {target} on /behavior/target (Ctrl+C to stop)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
