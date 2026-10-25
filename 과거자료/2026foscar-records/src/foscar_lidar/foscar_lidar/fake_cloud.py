#!/usr/bin/env python3
"""가짜 라이다 발행기 — 시뮬레이터 없이 obstacle_node 를 확인하기 위한 테스트 도구.

`/sensors/lidar/points` 는 아직 브리지(팀 3)에 붙지 않아서, 이 노드가 없으면
obstacle_node 가 도는지 눈으로 확인할 방법이 없다. 여기서는 map 위에 물체를 놓고
자차를 직진시키면서, 라이다가 봤을 법한 포인트를 합성해 10 Hz 로 쏜다.

장면 (map 기준)
    - 지면        : 평평한 바닥 (지면 제거가 되는지 확인)
    - 드럼통      : x=25 에 고정            → OBSTACLE 로 나와야 함
    - 보행자      : x=38 에서 좌→우 횡단    → PEDESTRIAN 로 나와야 함
    - NPC 차량    : x=55 에서 같은 방향 주행 → VEHICLE 로 나와야 함

자차가 움직이는데도 드럼통이 OBSTACLE 로 유지되면 ego-motion 보상이 제대로 되는 것이다.
`/localization/odometry` 도 같이 발행해서 그 경로까지 함께 검증한다.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def box_points(center, size, count, rng):
    """박스 안에 랜덤 포인트를 뿌려 '덩어리' 를 만든다 (map 기준, z 는 바닥부터)."""
    cx, cy = center
    sx, sy, sz = size
    return np.stack([
        rng.uniform(cx - sx / 2, cx + sx / 2, count),
        rng.uniform(cy - sy / 2, cy + sy / 2, count),
        rng.uniform(0.05, sz, count),
    ], axis=1)


class FakeCloud(Node):

    def __init__(self):
        super().__init__('fake_cloud')

        p = self.declare_parameter
        self.rate = p('rate_hz', 10.0).value
        self.ego_speed = p('ego_speed_mps', 3.0).value     # 0 이면 정차 상태로 테스트
        self.lidar_x = p('lidar_x', 2.0).value
        self.lidar_y = p('lidar_y', 0.0).value
        self.lidar_z = p('lidar_z', 2.11).value
        self.frame_id = p('frame_id', 'lidar').value
        self.noise = p('noise_m', 0.02).value
        self.ground_points = p('ground_points', 3000).value

        self.rng = np.random.default_rng(7)
        self.t0 = None
        self.ego_x = 0.0

        self.cloud_pub = self.create_publisher(PointCloud2, '/sensors/lidar/points',
                                               qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/localization/odometry',
                                              qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f'가짜 장면 발행 시작 ({self.rate:.0f} Hz, 자차 {self.ego_speed:.1f} m/s): '
            '드럼통(OBSTACLE) / 보행자(PEDESTRIAN) / NPC 차량(VEHICLE)')

    def scene(self, t: float) -> np.ndarray:
        """t 초 시점의 물체 포인트들을 map 좌표로 만든다."""
        rng = self.rng
        drum = box_points((25.0, 0.3), (0.6, 0.6, 1.0), 150, rng)
        # 보행자: y = -5 에서 1.4 m/s 로 좌 → 우 횡단
        ped = box_points((38.0, -5.0 + 1.4 * t), (0.5, 0.5, 1.7), 120, rng)
        # NPC 차량: 옆 차선에서 5 m/s 로 같은 방향 주행 (자차보다 빨라 서서히 멀어진다)
        npc = box_points((25.0 + 5.0 * t, 3.5), (4.5, 1.9, 1.5), 400, rng)
        return np.concatenate([drum, ped, npc], axis=0)

    def ground(self, ego_x: float) -> np.ndarray:
        """자차 주변 바닥. ROI 안을 고르게 덮어서 지면 제거를 시험한다."""
        n = int(self.ground_points)
        return np.stack([
            self.rng.uniform(ego_x - 2.0, ego_x + 45.0, n),
            self.rng.uniform(-8.0, 8.0, n),
            np.zeros(n),
        ], axis=1)

    def tick(self) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()
        t_now = now.nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t_now
        t = t_now - self.t0
        self.ego_x = self.ego_speed * t

        pts = np.concatenate([self.scene(t), self.ground(self.ego_x)], axis=0)

        # map → base_link (자차는 +x 로 직진하므로 회전 없음) → lidar
        pts[:, 0] -= self.ego_x
        pts -= np.array([self.lidar_x, self.lidar_y, self.lidar_z])
        pts += self.rng.normal(0.0, self.noise, pts.shape)

        # 라이다가 실제로 볼 수 있는 범위만 남긴다 (뒤쪽·너무 먼 점 제거)
        base_x = pts[:, 0] + self.lidar_x
        pts = pts[(base_x > 0.0) & (base_x < 45.0)]

        header_cloud = PointCloud2().header
        header_cloud.stamp = stamp
        header_cloud.frame_id = self.frame_id
        self.cloud_pub.publish(pc2.create_cloud_xyz32(header_cloud,
                                                      pts.astype(np.float32)))

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.ego_x)
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = float(self.ego_speed)
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FakeCloud()
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
