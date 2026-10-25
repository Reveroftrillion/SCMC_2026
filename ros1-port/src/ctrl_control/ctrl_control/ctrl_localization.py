"""GPS + IMU + 속도 → map 좌표 (x, y, yaw). GPS 가 없어도 odom 은 멈추지 않는다.

    평상시   GPS 위치 + IMU 절대 yaw. fix 사이는 속도×yaw 로 이어 붙인다 (계단 없음).
    GPS 끊김 gps_timeout 초 넘게 유효 fix 가 없으면 추측항법(DR): 마지막 위치에서 speed×yaw 적분.
             터널 118 m (s=1881~1998, GPS 차단) 이 이 코드의 존재 이유다. 무노이즈라 오차는 작다.
    GPS 복귀 DR 오차를 gps_blend_s 초에 걸쳐 없앤다 (점프 금지 — tracker 가 'position jump' 로 본다).
    IMU/status 끊김 stale_hold_s 초까지는 마지막 속도로 적분, 그 뒤엔 제자리 유지(속도 0 으로 적분),
             sensor_timeout 초 넘으면 발행을 멈춘다 → tracker 가 HOLD.
             시뮬(Unity)이 멈추면 물리도 최대 ~0.33 s 만 따라잡고 나머지 시간은 버려진다 — 즉 차는
             거의 그 자리다. 계속 적분하면 2 s 프리징에 8 m 앞으로 튄다 (합성 테스트 실측).

/localization/mode 에 'GPS' | 'DR' 을 발행한다 (바뀔 때 + 1 Hz, transient local).
FSM 이 TUNNEL 진입/탈출 판단에, tracker 가 DR 중 속도 상한에 쓴다.

GPS 가 "멈춘 값" 을 계속 보내는 경우(시뮬 구현에 따라 다름)도 잡는다: 차는 움직이는데(속도 > 1 m/s)
위경도가 gps_frozen_s 초 동안 그대로면 무효로 본다. NMEA 양자화 0.19 m 라 1 m/s 에서 0.5 s 면
반드시 값이 바뀌어야 정상이다.
"""
import math
import time

from pyproj import Proj, Transformer
import ctrl_ros1 as rclpy
from ctrl_ros1.ctrl_executors import ExternalShutdownException
from ctrl_ros1.ctrl_node import Node
from ctrl_ros1.ctrl_qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from ctrl_interfaces.msg import VehicleStatus

from .ctrl_core import PoseEstimator, quaternion_yaw, select_route, wrap
from .ctrl_route_start import start_station_hint
from pathlib import Path as FilePath


class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        defaults = dict(origin_east=302595., origin_north=4124145., gps_offset_x=0.,
                        gps_offset_y=0., imu_yaw_offset_deg=0., sensor_timeout=1.5,
                        gps_timeout=0.3, gps_blend_s=2.0, gps_frozen_s=0.5, stale_hold_s=0.3,
                        path_file='', spawn_file='', start_at_beginning=False)
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.transformer = Transformer.from_crs(4326, 32652, always_xy=True)
        # MORAI 의 IMU yaw 는 진북 기준, 우리 map(x, y) 는 UTM52 격자북 기준이라 자오선 수차만큼 어긋난다.
        # K-City 에서 γ = -1.35°: GPS 모드에선 조향 적분기가 흡수했지만 추측항법에선 86 m 에 2.5 m 횡오차로
        # 그대로 쌓였다 (2026-09-19 실측, 20개 주행 로그에서 이동방향-yaw = -1.40° ± 0.05 로 일정).
        lon0, lat0 = Transformer.from_crs(32652, 4326, always_xy=True).transform(self.p['origin_east'], self.p['origin_north'])
        self.grid_convergence = math.radians(Proj('epsg:32652').get_factors(lon0, lat0).meridian_convergence)
        self.imu = self.status = None
        self.fix = None                 # 아직 tick 이 안 쓴 최신 유효 GPS (east, north) - origin
        self.last_latlon = None
        self.frozen_since = None
        self.arrivals = dict(gps=0., imu=0., status=0.)
        self.estimator = PoseEstimator(self.p['gps_timeout'], self.p['gps_blend_s'])
        self.create_subscription(NavSatFix, '/sensors/gps', self.on_gps, qos_profile_sensor_data)
        self.create_subscription(Imu, '/sensors/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, '/vehicle/status', self.on_status, qos_profile_sensor_data)
        self.pub = self.create_publisher(Odometry, '/localization/odometry', qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, '/localization/mode', latched)
        # /localization/s: GLOBAL route progress [m] so perception can say "traffic light at s=208" (Event.s).
        # Picked on the first pose like the tracker does (negative while still on the respawn connector).
        self.global_points = np.loadtxt(self.p['path_file']) if self.p['path_file'] else None
        self.spawn_points = np.loadtxt(self.p['spawn_file']) if self.p['spawn_file'] else self.global_points
        self.route, self.s_offset = None, 0.
        self.s_pub = self.create_publisher(Float64, '/localization/s', qos_profile_sensor_data)
        self.previous_s = None
        self.tf = TransformBroadcaster()
        self.last_tick = time.monotonic()
        self.last_mode_pub = 0.
        self.last_dr_report = 0.
        self.blind_warned = False
        self.create_timer(0.02, self.tick)
        self.get_logger().info(f'IMU yaw(진북) → UTM 격자북 보정 γ = {math.degrees(self.grid_convergence):+.3f}°'
                               f' + 장착 오프셋 {self.p["imu_yaw_offset_deg"]:+.2f}°')

    def fresh(self, msg):
        age = (self.get_clock().now().nanoseconds - (msg.header.stamp.secs*10**9+msg.header.stamp.nsecs))/1e9
        return -0.05 <= age <= self.p['sensor_timeout']

    def on_gps(self, msg):
        # 무효 fix(터널) 는 그냥 무시한다: arrivals['gps'] 가 안 갱신되면 gps_timeout 뒤 DR 로 넘어간다.
        if not self.fresh(msg) or msg.status.status < 0 or not all(math.isfinite(v) for v in (msg.latitude, msg.longitude)):
            return
        latlon = (msg.latitude, msg.longitude)
        now = time.monotonic()
        if latlon == self.last_latlon:
            moving = self.status is not None and abs(self.status.speed_mps) > 1.0
            if self.frozen_since is None:
                self.frozen_since = now
            elif moving and now-self.frozen_since > self.p['gps_frozen_s']:
                return                  # 차는 가는데 GPS 가 멈춰 있다 = 사실상 끊김
        else:
            self.frozen_since = None
        self.last_latlon = latlon
        east, north = self.transformer.transform(msg.longitude, msg.latitude)
        self.fix = (east-self.p['origin_east'], north-self.p['origin_north'])
        self.arrivals['gps'] = now

    def on_imu(self, msg):
        if self.fresh(msg):
            self.imu = msg
            self.arrivals['imu'] = time.monotonic()

    def on_status(self, msg):
        if self.fresh(msg):
            self.status = msg
            self.arrivals['status'] = time.monotonic()

    def tick(self):
        now = time.monotonic()
        dt = min(max(now-self.last_tick, 0.), 0.2)
        self.last_tick = now
        if self.imu is None or self.status is None:
            return
        if max(now-self.arrivals['imu'], now-self.arrivals['status']) > self.p['sensor_timeout']:
            # 자세도 속도도 모르면 적분할 수 없다. odom 을 끊어 tracker 가 HOLD 로 버티게 한다.
            if not self.blind_warned:
                self.get_logger().warn('IMU/status 끊김 — odom 발행 중단 (돌아오면 자동 재개)')
                self.blind_warned = True
            return
        self.blind_warned = False
        q = self.imu.orientation
        yaw = wrap(quaternion_yaw(q.w, q.x, q.y, q.z)+self.grid_convergence-math.radians(self.p['imu_yaw_offset_deg']))
        speed = self.status.speed_mps
        stale = max(now-self.arrivals['imu'], now-self.arrivals['status'])
        if stale > self.p['stale_hold_s']:
            speed = 0.                  # 데이터가 멈췄으면 차도 멈춘 것으로 본다 (위 docstring)
        fix = None
        if self.fix is not None:
            # GPS offset is expressed relative to rear axle in vehicle FLU coordinates.
            dx, dy = self.p['gps_offset_x'], self.p['gps_offset_y']
            fix = (self.fix[0]-math.cos(yaw)*dx+math.sin(yaw)*dy, self.fix[1]-math.sin(yaw)*dx-math.cos(yaw)*dy)
            self.fix = None
        est = self.estimator
        mode = est.update(dt, yaw, speed, fix)
        if est.event == 'FIRST':
            self.get_logger().info(f'첫 GPS fix: x={est.x:.1f} y={est.y:.1f}')
        elif est.event == 'DR':
            self.get_logger().warn(f'GPS {now-self.arrivals["gps"]:.1f}s 무수신 → 추측항법 (IMU yaw + 속도 적분)')
            self.last_dr_report = now
        elif est.event == 'REACQUIRED':
            self.get_logger().info(f'GPS 복귀: 추측항법 {est.dr_distance:.0f} m, 오차 {est.last_error:.2f} m → '
                                   f'{est.blend_total:.1f}s 에 걸쳐 수렴')
        if mode == 'DR' and now-self.last_dr_report > 5.:
            self.get_logger().warn(f'추측항법 {est.gps_age:.0f}s / {est.dr_distance:.0f} m 째 (GPS 아직 없음)')
            self.last_dr_report = now
        if est.x is None:
            return
        x, y = est.x, est.y
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'map', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        # covariance[0] 로도 DR 여부를 알 수 있게 (0 = GPS, 1 = DR). /localization/mode 가 정식 경로.
        msg.pose.covariance[0] = 1. if mode == 'DR' else 0.
        msg.twist.twist.linear.x = speed
        self.pub.publish(msg)
        if est.event in ('FIRST', 'DR', 'REACQUIRED') or now-self.last_mode_pub > 1.:
            self.mode_pub.publish(String(data=mode))
            self.last_mode_pub = now
        if self.route is None and self.global_points is not None:
            try:
                route, offset = select_route(x, y, self.global_points, self.spawn_points)
                hint = start_station_hint(route, x, y, self.p['start_at_beginning'],
                                          FilePath(self.p['path_file']).with_name('checkpoints.txt'))
                self.route, self.s_offset, self.previous_s = route, offset, hint
            except ValueError:
                pass
        if self.route is not None:
            self.previous_s, _, _ = self.route.project(x, y, self.previous_s)
            self.s_pub.publish(Float64(data=self.previous_s-self.s_offset))
        tf = TransformStamped()
        tf.header, tf.child_frame_id = msg.header, msg.child_frame_id
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
