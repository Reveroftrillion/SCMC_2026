#!/usr/bin/env python3
"""bev_node — RViz 위에서 내려다본(BEV) 정적 배경을 그린다: 차선, s 눈금, 자차 박스.

    ros2 run foscar_planning bev_node
    rviz2 -d $(ros2 pkg prefix foscar_planning)/share/foscar_planning/rviz/bev.rviz

RViz 가 스스로 못 그리는 것만 여기서 만든다. 나머지는 RViz 기본 디스플레이가 직접 그린다:
    /sensors/lidar/points        PointCloud2, Color Transformer = Intensity (빛반사 — 차선 페인트가 밝다)
    /perception/obstacle_markers obstacle_node 의 박스
    /behavior/markers            behavior_node 의 후보 오프셋·장애물 Frenet 박스·선택 경로·상태판
    /map/route, /control/target  경로와 추종점

여기서 내는 것 (/bev/map 은 latched 라 한 번, /bev/ego 는 5 Hz)
    링크 중심선(회색) + 좌우 경계선(폭/2, 흰색; 반대차로 링크는 노란색)   ← link_set.json
        lane_boundary_set.json 이 없어서 진짜 차선 도형은 못 그린다. 링크 폭(전부 3.5 m)으로 근사.
    자차 박스 4.635 × 1.892 m (후륜축 원점, 앞바퀴축 선 포함)              ← base_link 프레임
    s 눈금: 뒤 20 m ~ 앞 60 m, 10 m 마다 가로선 + global s 숫자          ← /map/route + /localization/s
"""

import json
import math
from pathlib import Path as FilePath
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

from geometry_msgs.msg import Point
from foscar_control.core import Route
from foscar_control.mission_config import geometry_path
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import ColorRGBA, Float64
from visualization_msgs.msg import Marker, MarkerArray

from .frenet import s_offset_from, to_xy

DEFAULT_LINK_SET = 'package://foscar_bringup/maps/link_set.json'
CAR_LENGTH, CAR_WIDTH, WHEELBASE, REAR_OVERHANG = 4.635, 1.892, 3.0, 0.8   # Ioniq5. 후륜 오버행은 추정치


def rgba(r, g, b, a=1.):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class BevNode(Node):
    def __init__(self):
        super().__init__('bev_node')
        self.link_set = self.declare_parameter('link_set', DEFAULT_LINK_SET).value
        self.station_ticks = self.declare_parameter('station_ticks', False).value
        self.tick_m = self.declare_parameter('tick_m', 10.).value
        self.ahead_m = self.declare_parameter('ahead_m', 60.).value
        self.behind_m = self.declare_parameter('behind_m', 20.).value
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(MarkerArray, '/bev/map', latched)
        self.ego_pub = self.create_publisher(MarkerArray, '/bev/ego', 1)
        self.route: Optional[Route] = None
        self.s_offset: Optional[float] = None
        self.s_global: Optional[float] = None
        self.odom: Optional[Odometry] = None
        self.create_subscription(Path, '/map/route', self.on_route, latched)
        self.create_subscription(Float64, '/localization/s', self.on_s, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/localization/odometry', self.on_odom, qos_profile_sensor_data)
        self.publish_map()
        self.create_timer(0.2, self.publish_ego)

    # ------------------------------------------------------------------ 맵 (한 번)
    def publish_map(self):
        arr = MarkerArray()
        path = geometry_path(self.link_set, FilePath.cwd())
        if not path.is_file():
            self.get_logger().warn(f'link_set 없음: {path} — 차선 배경 없이 갑니다 (-p link_set:=경로)')
            return
        links = json.loads(path.read_text())
        centre = self.line_marker('links', 0, rgba(0.45, 0.45, 0.45, 0.6), 0.05, Marker.LINE_LIST)
        white = self.line_marker('lanes', 1, rgba(0.95, 0.95, 0.95, 0.8), 0.08, Marker.LINE_LIST)
        yellow = self.line_marker('lanes', 2, rgba(0.95, 0.8, 0.1, 0.8), 0.08, Marker.LINE_LIST)
        # Marker.points 에 += 로 하나씩 붙이면 매번 전체를 검사해 O(n²) — 리스트를 다 만들고 한 번에 넣는다
        c_pts, w_pts, y_pts = [], [], []
        for link in links:
            pts = [(float(p[0]), float(p[1])) for p in link.get('points', [])]
            if len(pts) < 2:
                continue
            half = float(link.get('width_start') or 3.5) / 2.
            edge = y_pts if link.get('opp_traffic') else w_pts
            for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
                dx, dy = x1 - x0, y1 - y0
                n = math.hypot(dx, dy)
                if n < 1e-6:
                    continue
                nx, ny = -dy / n * half, dx / n * half          # 왼쪽 법선
                c_pts += [Point(x=x0, y=y0, z=0.), Point(x=x1, y=y1, z=0.)]
                for sign in (1., -1.):
                    edge += [Point(x=x0 + sign * nx, y=y0 + sign * ny, z=0.02),
                             Point(x=x1 + sign * nx, y=y1 + sign * ny, z=0.02)]
        centre.points, white.points, yellow.points = c_pts, w_pts, y_pts
        arr.markers += [centre, white, yellow]
        self.map_pub.publish(arr)
        self.get_logger().info(f'차선 배경: 링크 {len(links)}개 ({path.name})')

    def line_marker(self, ns, mid, color, width, kind=Marker.LINE_STRIP, frame='map'):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, kind, Marker.ADD
        m.scale.x = width
        m.color = color
        m.pose.orientation.w = 1.
        return m

    # ------------------------------------------------------------------ 자차 + 눈금 (5 Hz)
    def on_route(self, msg: Path):
        try:
            self.route = Route([(q.pose.position.x, q.pose.position.y) for q in msg.poses])
            self.s_offset = None
        except ValueError as e:
            self.get_logger().error(f'/map/route: {e}')

    def on_s(self, msg: Float64):
        self.s_global = float(msg.data)

    def on_odom(self, msg: Odometry):
        self.odom = msg

    def publish_ego(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        # 자차 박스 (base_link: 원점 = 후륜축)
        box = Marker()
        box.header.frame_id, box.header.stamp = 'base_link', stamp
        box.ns, box.id, box.type, box.action = 'ego', 0, Marker.CUBE, Marker.ADD
        box.pose.position.x = CAR_LENGTH / 2. - REAR_OVERHANG
        box.pose.position.z = 0.4
        box.pose.orientation.w = 1.
        box.scale.x, box.scale.y, box.scale.z = CAR_LENGTH, CAR_WIDTH, 0.8
        box.color = rgba(0.2, 0.6, 1., 0.35)
        arr.markers.append(box)
        axle = self.line_marker('ego', 1, rgba(0.2, 0.6, 1., 0.9), 0.06, Marker.LINE_LIST, frame='base_link')
        axle.header.stamp = stamp
        axle.points = [Point(x=x, y=y, z=0.05) for x in (0., WHEELBASE)      # 후륜축 / 앞바퀴축 (stop_s 기준)
                       for y in (-CAR_WIDTH / 2, CAR_WIDTH / 2)]
        arr.markers.append(axle)
        # s 눈금
        if self.station_ticks and self.route is not None and self.s_global is not None and self.odom is not None:
            if self.s_offset is None:
                o = self.odom.pose.pose.position
                self.s_offset = s_offset_from(self.route, o.x, o.y, self.s_global)
            ticks = self.line_marker('ticks', 2, rgba(0.6, 0.8, 1., 0.5), 0.04, Marker.LINE_LIST)
            ticks.header.stamp = stamp
            tick_pts = []
            first = math.floor((self.s_global - self.behind_m) / self.tick_m) * self.tick_m
            k = 3
            s = first
            while s <= self.s_global + self.ahead_m:
                sl = s + self.s_offset
                if 0. <= sl <= self.route.s[-1]:
                    a, b = to_xy(self.route, sl, -3.), to_xy(self.route, sl, 3.)
                    tick_pts += [Point(x=a[0], y=a[1], z=0.03), Point(x=b[0], y=b[1], z=0.03)]
                    t = Marker()
                    t.header.frame_id, t.header.stamp = 'map', stamp
                    t.ns, t.id, t.type, t.action = 'ticks', k, Marker.TEXT_VIEW_FACING, Marker.ADD
                    k += 1
                    tx, ty = to_xy(self.route, sl, 3.8)
                    t.pose.position.x, t.pose.position.y, t.pose.position.z = tx, ty, 0.1
                    t.pose.orientation.w = 1.
                    t.scale.z = 0.7
                    t.color = rgba(0.6, 0.8, 1., 0.9)
                    t.text = f'{s:.0f}'
                    arr.markers.append(t)
                s += self.tick_m
            ticks.points = tick_pts
            arr.markers.append(ticks)
        self.ego_pub.publish(arr)


def main():
    rclpy.init()
    node = BevNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
