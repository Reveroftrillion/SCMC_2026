#!/usr/bin/env python3
"""터미널 라이다 모니터 — RViz 없이 라이다가 뭘 보고 있는지 확인한다.

    ros2 run foscar_lidar lidar_monitor

`/sensors/lidar/points` 와 `/events` 를 같이 구독해서, 터미널을 갱신하며

    * 수신 주기와 포인트 수 (ROI · 지면제거 단계별로 몇 개가 남는지)
    * **지면 높이 역산 → 라이다 장착 높이 추정** (현장에서 z 값 확인용)
    * 위에서 내려다본 ASCII 지도 (+ `/events` 로 확정된 객체 위치)

를 보여준다. 장착 높이가 틀리면 지면이 통째로 장애물로 잡히거나 낮은 장애물이
지면과 함께 잘려나가므로, 현장에서 제일 먼저 확인해야 하는 값이다.
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from foscar_interfaces.msg import Event
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

MARK = {'OBSTACLE': 'O', 'PEDESTRIAN': 'P', 'VEHICLE': 'V', 'DYNAMIC_UNKNOWN': 'D'}
SHADE = ' .:*#'          # 셀 안 포인트 밀도


class LidarMonitor(Node):

    def __init__(self):
        super().__init__('lidar_monitor')

        p = self.declare_parameter
        self.lidar_x = p('lidar_x', 2.0).value
        self.lidar_y = p('lidar_y', 0.0).value
        self.lidar_z = p('lidar_z', 2.11).value   # 지면 기준 실측값
        self.roi_x_max = p('roi_x_max', 40.0).value
        self.roi_y_abs = p('roi_y_abs', 6.0).value
        self.roi_z_min = p('roi_z_min', -0.3).value
        self.roi_z_max = p('roi_z_max', 2.5).value
        self.ground_z = p('ground_z', 0.15).value
        self.ground_slope_tol = p('ground_slope_tol', 0.01).value
        self.event_hold_s = p('event_hold_s', 0.5).value   # 이보다 오래된 이벤트는 화면에서 지운다
        self.rows = p('map_rows', 20).value
        self.cols = p('map_cols', 49).value
        self.plain = p('plain', False).value      # true 면 화면을 지우지 않고 계속 쌓는다

        self.cloud = None
        self.stamps = deque(maxlen=20)
        self.events = {}
        self.total = 0

        self.create_subscription(PointCloud2, '/sensors/lidar/points',
                                 self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Event, '/events', self.on_event, 10)
        self.create_timer(0.5, self.render)

    def on_cloud(self, msg: PointCloud2) -> None:
        raw = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        self.cloud = raw.reshape(-1, 3) if raw.size else np.empty((0, 3), np.float32)
        self.stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.total += 1

    def on_event(self, msg: Event) -> None:
        # 같은 프레임의 객체들이 연달아 오므로 stamp 별로 모았다가 최신 것만 그린다
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.events.setdefault(key, []).append(msg)
        for old in sorted(self.events)[:-1]:
            del self.events[old]

    # ------------------------------------------------------------------

    def hz(self) -> float:
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / span if span > 1e-6 else 0.0

    def render(self) -> None:
        out = [] if self.plain else ['\033[2J\033[H']
        out.append('─── LiDAR 모니터 ' + '─' * 46)

        if self.cloud is None:
            out.append('')
            out.append('  /sensors/lidar/points 수신 없음.')
            out.append('  → lidar_bridge 가 떠 있는지, MORAI 센서가 Connect 상태인지 확인하세요.')
            out.append('     ros2 run foscar_lidar lidar_bridge')
            print('\n'.join(out), flush=True)
            return

        # 라이다 → base_link (지면 = 0)
        pts = self.cloud + np.array([self.lidar_x, self.lidar_y, self.lidar_z], np.float32)
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        in_roi = ((x > 0) & (x < self.roi_x_max) & (np.abs(y) < self.roi_y_abs)
                  & (z > self.roi_z_min) & (z < self.roi_z_max))
        ground = np.maximum(self.roi_z_min,
                            self.ground_z + self.ground_slope_tol * np.maximum(x, 0.0))
        above = in_roi & (z > ground)

        out.append(f'수신    {self.hz():5.1f} Hz (총 {self.total} 회전)')
        out.append(f'포인트  전체 {len(pts):6d} → ROI {int(in_roi.sum()):6d} '
                   f'→ 지면제거 {int(above.sum()):6d}')
        out.append(self.ground_line())

        if above.sum():
            fx, fy = x[above], y[above]
            near = int(np.argmin(np.hypot(fx, fy)))
            out.append(f'최근접  전방 {fx[near]:5.1f} m, 좌우 {fy[near]:+5.1f} m '
                       f'(거리 {np.hypot(fx[near], fy[near]):.1f} m)')
        else:
            out.append('최근접  ROI 안에 지면 위 포인트 없음')

        out.append('')
        out.extend(self.ascii_map(x[above], y[above]))
        out.append('')
        out.extend(self.event_lines())
        print('\n'.join(out), flush=True)

    def ground_line(self) -> str:
        """전방 노면의 z 를 재서 장착 높이가 맞는지 알려준다."""
        x, y, z = self.cloud[:, 0], self.cloud[:, 1], self.cloud[:, 2]   # 라이다 기준
        road = (x > 8.0) & (x < 25.0) & (np.abs(y) < 4.0)
        if road.sum() < 50:
            return '지면    전방 노면 샘플 부족 (차가 벽 앞에 있거나 데이터가 적음)'
        ground_z = float(np.median(z[road]))
        est = -ground_z
        flag = '' if abs(est - self.lidar_z) < 0.10 else \
            f'  ← 설정값 {self.lidar_z:.2f} 와 {abs(est-self.lidar_z):.2f} m 차이. lidar_z 확인!'
        return (f'지면    라이다 기준 z = {ground_z:+.2f} m '
                f'→ 장착 높이 추정 {est:.2f} m{flag}')

    def ascii_map(self, fx: np.ndarray, fy: np.ndarray) -> list:
        rows, cols = int(self.rows), int(self.cols)
        grid = np.zeros((rows, cols), dtype=np.int32)
        if fx.size:
            r = ((self.roi_x_max - fx) / self.roi_x_max * rows).astype(int)
            c = ((self.roi_y_abs - fy) / (2 * self.roi_y_abs) * cols).astype(int)
            ok = (r >= 0) & (r < rows) & (c >= 0) & (c < cols)
            np.add.at(grid, (r[ok], c[ok]), 1)

        chars = [[SHADE[min(int(n), len(SHADE) - 1)] for n in row] for row in grid]
        for ev in self.latest_events():
            r = int((self.roi_x_max - ev.x) / self.roi_x_max * rows)
            c = int((self.roi_y_abs - ev.y) / (2 * self.roi_y_abs) * cols)
            if 0 <= r < rows and 0 <= c < cols:
                chars[r][c] = MARK.get(ev.type, '?')

        lines = [f'  위에서 본 ROI   좌 +{self.roi_y_abs:.0f} m ' +
                 '─' * (cols - 24) + f' 우 -{self.roi_y_abs:.0f} m']
        for i, row in enumerate(chars):
            x_lab = self.roi_x_max - (i + 0.5) / rows * self.roi_x_max
            lines.append(f'{x_lab:5.1f}m |' + ''.join(row) + '|')
        lines.append('       ' + ' ' * (cols // 2) + '▲ 자차')
        lines.append(f'       밀도 "{SHADE[1:]}"  객체 O=정지 P=보행자 V=차량')
        return lines

    def latest_events(self) -> list:
        """마지막으로 받은 이벤트 묶음. 단, 너무 오래됐으면 빈 목록을 준다.

        새 이벤트가 올 때만 정리하면, 물체를 지나친 뒤 탐지가 끊겼을 때
        마지막 박스가 화면에 영원히 남는다 (실측 최장 43초).
        """
        if not self.events:
            return []
        newest = max(self.events)
        age = (self.stamps[-1] if self.stamps else 0.0) - (newest[0] + newest[1]*1e-9)
        if age > self.event_hold_s:
            self.events.clear()
            return []
        return self.events[newest]

    def event_lines(self) -> list:
        evs = self.latest_events()
        if not evs:
            return ['/events   없음 (3프레임 연속 확인된 객체가 아직 없습니다)']
        lines = [f'/events   {len(evs)} 건']
        for ev in sorted(evs, key=lambda e: e.x):
            lines.append(f'  {ev.type:<10s} x={ev.x:5.1f} y={ev.y:+5.1f}  '
                         f'크기 {ev.size_x:.1f}×{ev.size_y:.1f} m  '
                         f'속도 {np.hypot(ev.vx, ev.vy):4.1f} m/s  '
                         f'신뢰도 {ev.confidence:.2f}')
        return lines


def main(args=None):
    rclpy.init(args=args)
    node = LidarMonitor()
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
