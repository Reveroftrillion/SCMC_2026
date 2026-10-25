"""Collect one signal cycle's worth of images at each mapped stop line (fixed time).

Shares foscar_camera.stop_lines.StopLineTable and stop_line_node's DrivingTarget
contract. This collection-only node does not use signal colors to release a stop.
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from foscar_interfaces.msg import DrivingTarget, VehicleStatus
from foscar_camera.stop_lines import StopLineTable, default_file, default_routes
from foscar_control.core import FRONT_AXLE

from .dwell import CollectionStops, Decision


class CollectionStopNode(Node):
    def __init__(self):
        super().__init__('collection_stop_node')
        p = self.declare_parameter
        path, spawn = default_routes()
        self.duration = float(p('stop_seconds', 60.).value)
        self.offset = float(p('stop_before_m', 0.).value)
        self.reach = float(p('reach_m', .5).value)
        self.stopped_speed = float(p('stopped_speed_mps', .1).value)
        self.timeout = float(p('sensor_timeout_s', .5).value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('sensor_timeout_s must be positive and finite')
        # Validate settings before starting, even when the route is not ready yet.
        CollectionStops([0.], self.duration, FRONT_AXLE, self.offset,
                        self.reach, self.stopped_speed, self.timeout)
        self.table = StopLineTable(
            p('stop_lines_file', default_file()).value,
            p('path_file', path).value, p('spawn_file', spawn).value,
            route=p('route', 'auto').value,
            max_lateral=float(p('max_lateral_m', 1.).value), logger=self.get_logger())
        self.fsm = None
        self.s = self.speed = math.nan
        self.s_at = self.speed_at = -math.inf
        self.last_report = -math.inf
        self.last_state = None
        self.create_subscription(Odometry, '/localization/odometry', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Float64, '/localization/s', self.on_s, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, '/vehicle/status', self.on_status, qos_profile_sensor_data)
        self.target_pub = self.create_publisher(DrivingTarget, '/behavior/target', 1)
        self.state_pub = self.create_publisher(String, '/behavior/state', 1)
        self.create_timer(.05, self.tick)
        self.get_logger().info(f'수집 정차: 정지선 {self.offset:g} m 앞, 정지 확인 후 '
                               f'{self.duration:g}초 대기 → 자동 출발 (신호색과 무관)')

    def on_odom(self, msg):
        if not self.table.ready:
            self.table.on_pose(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_s(self, msg):
        self.s, self.s_at = float(msg.data), time.monotonic()

    def on_status(self, msg):
        self.speed, self.speed_at = float(msg.speed_mps), time.monotonic()

    def tick(self):
        now = time.monotonic()
        if self.table.ready and self.fsm is None:
            self.fsm = CollectionStops([line.s for line in self.table.lines], self.duration,
                                       FRONT_AXLE, self.offset, self.reach,
                                       self.stopped_speed, self.timeout)
        fresh = now - self.s_at <= self.timeout and now - self.speed_at <= self.timeout
        decision = (self.fsm.step(now, self.s, self.speed, healthy=fresh) if self.fsm
                    else Decision('COLLECT_WAITING', v_max=0., remaining_s=self.duration))
        msg = DrivingTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.state, msg.stop_s, msg.v_max = decision.state, decision.stop_s, decision.v_max
        msg.lat_offset = 0.
        self.target_pub.publish(msg)
        self.state_pub.publish(String(data=decision.state))
        if decision.state != self.last_state or now - self.last_report >= 3.:
            self.get_logger().info(f'{decision.state} | s={self.s:.1f} 정지선={decision.line_s:.1f} '
                                   f'v={self.speed * 3.6:.2f} km/h 대기 남음={decision.remaining_s:.1f}초')
            self.last_state, self.last_report = decision.state, now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CollectionStopNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
