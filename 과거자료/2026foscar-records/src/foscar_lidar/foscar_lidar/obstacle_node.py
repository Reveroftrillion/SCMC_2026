#!/usr/bin/env python3
"""장애물 인지 노드 (FOSCAR 2026 / 팀 2 라이다).

`/sensors/lidar/points` (VLP16, 10 Hz, frame `lidar`) 를 받아서

    좌표 변환 → ROI 자르기 → 지면 제거 → DBSCAN 클러스터링
    → 다중 프레임 추적 → 속도 추정 → 정지/이동 분류 → N프레임 안정화

를 수행하고, 확정된 객체마다 `/events` 로 `foscar_interfaces/msg/Event` 를 발행한다.

좌표계 규약
-----------
입력 포인트는 **라이다 원점 기준**이다. 맨 먼저 장착 오프셋(기본 x=2.0, y=0.0, z=2.11)
을 더해 **base_link 기준(지면 z=0, 전방 +x, 좌측 +y)** 으로 옮긴 뒤 모든 처리를 한다.
그래야 ROI 높이(-0.3~2.5 m)나 지면 임계값을 "지면 = 0" 이라는 직관적인 값으로 다룰 수 있다.
(라이다가 2.11 m 높이에 있으므로, 라이다 기준 z = -2.11 인 점이 곧 지면이다.)
MORAI 센서 패널에는 z=1.75 로 적혀 있지만 그건 **차량 원점 기준**이고, 차량 원점이
지면에서 0.36 m 위에 있다. 우리에게 필요한 건 지면 기준 높이라 2.11 이 맞다 —
K-City 정차 상태에서 전방 8~30 m 노면에 평면을 맞춰 실측했다 (구간별 -2.10 ~ -2.12 m,
잔차 0.135 m). 1.75 로 두면 지면이 ROI 아래로 밀려나면서 장애물 아랫부분 0.36 m 가
같이 잘려 ROI 안 포인트가 1889 → 175 개로 줄어든다.

`base_link → lidar` TF 가 발행되고 있으면 그쪽을 우선 사용하고(회전까지 반영),
없으면 파라미터 오프셋으로 폴백한다. 현장점검에서 장착 위치가 바뀌면 런치 인자만 고치면 된다.

속도 추정
---------
`vx, vy` 는 `/localization/odometry` 로 자차 움직임을 보상한 **map 프레임 절대속도**다
(`Event.msg` 규약). 보상을 안 하면 자차가 30 km/h 로 달릴 때 가만히 서 있는 전봇대가
8.3 m/s 로 다가오는 것처럼 보여서 정적 장애물이 전부 VEHICLE 로 오분류된다.
odom 이 끊기면 상대속도 모드로 자동 강등하고 경고를 남긴다.

경로 투영
---------
`/map/route` (전역경로) 와 `/localization/s` 를 받아 덩어리를 경로에 투영한다. 그러면
`Event.s` 를 채울 수 있고, **경로에서 옆으로 얼마나 떨어졌는지** 로 걸러낼 수 있다.
이게 없으면 곡선 구간에서 길가 가드레일이 "내 차선 안" 으로 보인다 — `y` 는 차가 똑바로
앞을 볼 때의 좌우 거리라서, 도로가 휘면 바깥쪽 구조물이 직진축 기준으로 안쪽에 들어온다.
실측에서 전체 이벤트의 96.5% 가 길가 구조물이었다.

움직이는 물체는 **지금 위치와 2초 뒤 예상 위치 중 가까운 쪽** 으로 판정한다. 인도에서
횡단을 시작한 보행자를 놓치지 않기 위해서다 (개발계획서 4-3 YIELD 진입조건).

시간
----
dt 는 **항상 메시지의 header.stamp** 로만 계산한다. `time.time()` 은 시뮬 재생/리플레이에서
벽시계와 어긋나 속도 추정이 통째로 망가지므로 이 파일 어디에서도 쓰지 않는다.
(`perf_counter` 는 "한 프레임 처리에 몇 ms 걸렸나" 를 재는 프로파일링 전용으로만 쓰고,
 인지 결과에는 절대 관여하지 않는다.)
"""

import math
import os
import uuid
from collections import deque
from time import perf_counter

from .ground import estimate_ground_plane, relative_height
from .sight import blind_bands, pitch_deg

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from foscar_control.mission_config import geometry_path
from foscar_interfaces.msg import Event
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64, String
import json
from .observed_footprint import observed_hull, oriented_dimensions
from geometry_msgs.msg import Point
from .clearance import ClearanceVerifier
from .fragment_merge import merge_vehicle_fragments, overlaps
from .highway_visibility import HighwaySensor
from .tunnel_walls import wall_faces
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray

from .road_mask import RoadMask

try:
    from sklearn.cluster import DBSCAN
except ImportError:  # pragma: no cover - 설치 안내용
    raise ImportError(
        'scikit-learn 이 없습니다. `sudo apt install python3-sklearn` 또는 '
        '`pip3 install scikit-learn` 으로 설치하세요.')


# ---------------------------------------------------------------------------
# 순수 유틸 (ROS 의존 없음)
# ---------------------------------------------------------------------------

def stamp_to_sec(stamp) -> float:
    """builtin_interfaces/Time → float 초. 시간 계산의 유일한 출처."""
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quat(q) -> float:
    """쿼터니언 → yaw (rad). 차량은 평면 주행이라 yaw 만 쓴다."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(np.arctan2(siny, cosy))


def quat_to_matrix(q) -> np.ndarray:
    """쿼터니언 → 3x3 회전행렬. TF 로 받은 라이다 자세(피치/롤 포함)를 반영할 때 쓴다."""
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3, dtype=np.float32)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ], dtype=np.float32)


def fresh_mission(node, name):
    return (getattr(node, 'mission_name', '') == name
            and 0 <= perf_counter()-getattr(node, 'mission_received_at', -math.inf) <= 1.)


def track_box(tr):
    """Observed AABB, independent of the return centroid used for tracking."""
    # Held tunnel outlines have already been transformed into the current frame.
    if getattr(tr, 'hold_fp', None) is None and len(tr.footprint):
        points = np.asarray(tr.footprint)
        lo, hi = points.min(axis=0), points.max(axis=0)
        center, size = (lo + hi) / 2., hi - lo
        return float(center[0]), float(center[1]), float(size[0]), float(size[1])
    return tr.x, tr.y, tr.size_x, tr.size_y


class Track:
    """한 덩어리(클러스터)를 프레임 간에 이어 붙인 추적 객체.

    위치 이력은 **map 프레임**으로 쌓는다. base_link 기준으로 쌓으면 자차가 움직일 때
    정지 물체도 움직이는 것으로 보이기 때문이다. 발행할 x, y 는 다시 base_link 기준
    최신 관측값을 그대로 쓴다 (Event.msg 의 frame_id = base_link).
    """

    def __init__(self, track_id: int, history_len: int):
        self.id = track_id
        self.history = deque(maxlen=history_len)   # (t, map_x, map_y)
        self.hits = 0          # 연속 탐지 점수. 놓치면 1씩 깎는다
        self.misses = 0        # 연속으로 못 본 프레임 수
        self.vx = 0.0
        self.vy = 0.0
        self.published = False
        self.motion_in_map = False
        self.moving = False    # 히스테리시스 상태. 경계값에서 깜빡이지 않게 한다
        self.straightness = 0.0  # 이동 궤적이 얼마나 곧은가 (0~1). 흔들림과 진짜 이동을 가른다
        self.kind = 'OBSTACLE'   # 이번 프레임 분류 결과 (classify 는 프레임당 한 번만 부른다)
        self.ped = None          # 이번 관측의 사람 모양 특징 (ObstacleNode.pedestrian_features)
        self.ped_votes = deque(maxlen=3)   # 최근 3 프레임 사람 모양 여부
        self.ped_confirmed = False         # 한 번 사람으로 확정되면 이 트랙이 끝날 때까지 사람 (사람일 수 없는 크기면 해제)
        self.ped_too_long = deque(maxlen=3)   # 최근 3 프레임 긴 변이 사람 상한의 2배를 넘었는지
        self.s = -1.0            # 경로 진행거리. 경로를 못 받으면 -1
        self.lat = None          # 경로에서 옆으로 떨어진 거리 (좌 +)
        self.structure = False   # 줄지어 선 구조물(가드레일 등) 의 일부인가
        # 최신 관측 (base_link 기준)
        self.x = 0.0
        self.y = 0.0
        self.size_x = 0.0
        self.size_y = 0.0
        self.size_z = 0.0
        self.z_max = 0.0
        self.n_points = 0
        self.last_t = 0.0
        self.footprint = []
        self.footprint_map = []

    def consistent_motion(self):
        """Require displacement across multiple steps, not a box centroid jump."""
        if not self.motion_in_map or len(self.history) < 3:
            return False
        h = np.asarray(self.history, dtype=float)
        dt = np.diff(h[:,0])
        delta = np.diff(h[:,1:],axis=0)
        distance = np.linalg.norm(delta,axis=1)
        net = h[-1,1:]-h[0,1:]
        displacement = np.linalg.norm(net)
        if (not np.all(np.isfinite(h)) or np.any(dt <= 0) or np.any(dt > .25)
                or h[-1,0]-h[0,0] < .19 or displacement < .6):
            return False
        forward = delta @ (net/displacement)
        return (displacement/max(float(distance.sum()),1e-6) >= .95
                and np.all(forward/dt >= .8))

    def predict(self, t: float) -> np.ndarray:
        """등속 가정으로 t 시점의 map 위치를 예측. 매칭 게이트의 중심이 된다."""
        if not self.history:
            return np.zeros(2)
        t0, mx, my = self.history[-1]
        dt = max(0.0, t - t0)
        return np.array([mx + self.vx * dt, my + self.vy * dt])

    def update(self, t: float, det: dict, cap: int) -> None:
        """이번 프레임에서 매칭된 관측으로 갱신."""
        self.history.append((t, det['mx'], det['my']))
        self.x, self.y = det['x'], det['y']
        self.size_x, self.size_y, self.size_z = det['size_x'], det['size_y'], det['size_z']
        self.z_max = det['z_max']
        self.structure = det.get('structure', False)
        self.n_points = det['n']
        self.last_t = t
        self.footprint = det.get('footprint', [])
        self.footprint_map = det.get('footprint_map', [])
        self.ped = det.get('ped')
        self.hits = min(self.hits + 1, cap)
        self.misses = 0
        self._fit_velocity()

    def miss(self) -> None:
        """이번 프레임에 못 봤을 때. 바로 지우지 않고 점수만 깎는다(깜빡임 방지)."""
        self.misses += 1
        self.hits = max(0, self.hits - 1)

    def _fit_velocity(self) -> None:
        """이력 전체에 직선을 최소자승으로 맞춰 기울기를 속도로 쓴다.

        연속 두 프레임 차분보다 노이즈에 훨씬 강하다. 이력이 1개거나 시간 폭이
        너무 좁으면(중복 stamp 등) 0 으로 둔다.
        """
        if len(self.history) < 2:
            self.vx = self.vy = self.straightness = 0.0
            return
        h = np.asarray(self.history, dtype=np.float64)
        ts = h[:, 0] - h[0, 0]
        if ts[-1] < 1e-3:
            self.vx = self.vy = self.straightness = 0.0
            return
        design = np.stack([ts, np.ones_like(ts)], axis=1)   # (n, 2)
        slope, _ = np.linalg.lstsq(design, h[:, 1:3], rcond=None)[0]
        self.vx, self.vy = float(slope[0]), float(slope[1])

        # 직진성 = 처음~끝 직선거리 / 실제 걸어온 거리.
        # 진짜 움직이는 물체는 1 에 가깝고, 클러스터 중심이 떨리는 정지물은 0 에 가깝다.
        steps = np.linalg.norm(np.diff(h[:, 1:3], axis=0), axis=1)
        path = float(steps.sum())
        net = float(np.linalg.norm(h[-1, 1:3] - h[0, 1:3]))
        self.straightness = net / path if path > 1e-3 else 0.0


# ---------------------------------------------------------------------------
# 노드
# ---------------------------------------------------------------------------

class ObstacleNode(Node):

    def __init__(self, **kwargs):
        # kwargs 는 rclpy Node 로 그대로 넘긴다 (테스트에서 parameter_overrides 주입용)
        super().__init__('obstacle_node', **kwargs)

        p = self.declare_parameter
        # --- 토픽 ---
        self.track_namespace = uuid.uuid4().hex
        self.input_topic = p('input_topic', '/sensors/lidar/points').value
        self.odom_topic = p('odom_topic', '/localization/odometry').value
        self.event_topic = p('event_topic', '/events').value
        self.observation_topic = p('observation_topic', '/perception/road_users').value
        self.highway_sensor = HighwaySensor.from_files(p('highway.config','').value, p('highway.mission_config','').value)
        self.highway_scan_valid = False
        self.marker_topic = p('marker_topic', '/perception/obstacle_markers').value
        # --- 라이다 장착 위치 (차 기준, 현장점검에서 값만 바꾸면 됨) ---
        self.lidar_x = p('lidar_x', 2.0).value
        self.lidar_y = p('lidar_y', 0.0).value
        self.lidar_z = p('lidar_z', 2.11).value   # 지면 기준 실측값 (패널의 1.75 는 차량 원점 기준)
        self.use_tf = p('use_tf', True).value
        self.base_frame = p('base_frame', 'base_link').value
        # --- ROI (전부 base_link 기준, 지면 = 0) ---
        self.roi_x_min = p('roi_x_min', 0.0).value
        self.roi_x_max = p('roi_x_max', 40.0).value
        self.roi_y_abs = p('roi_y_abs', 20.0).value
        self.roundabout_roi_y_abs = p('roundabout_roi_y_abs', 35.0).value
        # the ring seen from the mission start on (user 2026-10-03: perceive early, decide as before): 82-115 m ahead
        # and up to 34 m aside at s 801; points past roi_x_max are kept on mapped road only
        self.roundabout_roi_x_max = p('roundabout_roi_x_max', 100.0).value
        self.roundabout_roi_x_min = p('roundabout_roi_x_min', -15.0).value
        self.roundabout_observation_begin = p('roundabout_observation_begin_s', 800.92).value   # 860 saw the ring too late at speed (2026-09-27); the mission start since 2026-10-03
        self.roundabout_observation_end = p('roundabout_observation_end_s', 910.41).value
        # The ring lane (lane map, as the planner's CircularReference reads it): a cluster centred on it is a road user,
        # never part of a guardrail chain -- a crossing car seen as 3 aligned fragments was dropped as one (2026-10-04)
        self.ring_center = [float(v) for v in p('roundabout_ring_center', [-102.0177, 343.3816]).value]
        self.ring_radius = p('roundabout_ring_radius_m', 17.80).value
        self.ring_lane_half = p('roundabout_ring_lane_half_m', 1.75).value
        # Footprints of new tracks from their first scan in the roundabout window (the 3-scan confirmation is kept for
        # events): a car whose outline changed at our entry came back as a new track and left the ring 2-3 scans;
        # missing scans 4 -> 1 of ~130 over 3 runs (2026-10-04). Overlap matching (tried too) changed nothing.
        self.ring_fp_first = p('roundabout_footprints_first_scan', True).value
        self.roi_z_min = p('roi_z_min', -0.3).value
        self.roi_z_max = p('roi_z_max', 2.5).value
        # 지면 제거: 이 높이 이하는 바닥으로 본다. 라이다 기준으로는 z > (ground_z - lidar_z)
        self.ground_z = p('ground_z', 0.15).value
        # 거리에 비례해 지면 임계값을 올린다. 먼 곳일수록 지면 추정이 부정확해지기 때문:
        #   · -3도 빔은 40 m 지점에서 노면을 거의 수평으로 스친다 → 노면이 몇 cm만 울퉁불퉁해도
        #     점이 크게 튄다. 실측에서 30~40 m 구간 점의 50% 가 0.15~0.50 m 높이대였다.
        #   · 가감속 시 차가 1도만 끄덕여도 40 m 에서 지면이 0.7 m 어긋난다.
        # 0.01 = 1 m 당 1 cm. 40 m 에서 임계값 0.55 m. 어차피 40 m 에서는 빔이 지면(0.01 m)과
        # 1.41 m 두 군데밖에 안 닿아서 그 사이 높이의 물체는 원래 못 본다 — 손해가 거의 없다.
        self.ground_slope_tol = p('ground_slope_tol', 0.01).value
        # --- 클러스터링 ---
        self.max_points = p('max_points', 12000).value
        self.eps = p('dbscan_eps', 0.5).value
        self.min_samples = p('dbscan_min_samples', 5).value
        # Threads for DBSCAN and the road-mask query. -1 (all cores) spiked to 11-12 cores for
        # seconds in the tunnel while behavior_node/YOLO needed them (2026-09-28 full laps); one scan
        # takes ~20 ms, so 4 threads leave plenty of the 110 ms scan period.
        self.workers = int(p('workers', 4).value)
        # 가장 긴 변이 이보다 길면 "물체" 가 아니라 벽·레일 조각이다. 대회 최대 물체는 차량(4.6 m).
        # 10 m 였을 때 3.4×6.3, 3.5×9.9 짜리 벽 조각이 통로 안으로 투영돼 발행됐다 (2026-09-20).
        self.max_object_size = p('max_object_size_m', 5.0).value
        self.max_vehicle_length = p('roundabout_max_vehicle_length_m', 14.0).value
        # Widest truck hull read across its heading near the roundabout: p50 2.58, p90 3.13, p99 4.76, max 5.04 m (74 scans,
        # 16 CP08 runs 2026-10-02; truth width 3.15 m). At 3.2 a dump truck dropped out right at the entry and the entry
        # went ahead into it (settle-18 run).
        self.max_vehicle_width = p('roundabout_max_vehicle_width_m', 5.1).value
        # 도로를 가로지르는 폭이 이보다 넓으면 물체가 아니라 벽·레일이 곡선에서 잘린 것이다.
        # 차량 폭 1.9 m + 여유. 차량 좌표(size_x/size_y)가 아니라 **경로 좌표(d)** 로 재야 한다 —
        # 멀리서 앞면만 보이는 박스는 차량 좌표로는 0.3×1.4 라 '얇고 길다' 에 걸려 같이 죽는다.
        # 실측 2026-09-20: 박스 d폭 중앙 1.47 m / 오탐 d폭 중앙 3.48 m.
        self.max_block_width = p('max_block_width_m', 2.0).value
        # 점 몇 개짜리 퇴화 클러스터만 버린다. 값을 크게 잡으면 안 된다 —
        # K-City 중앙분리대의 델리네이터 봉이 두께 4~10 cm 라 0.15 로 두면 통째로 놓친다.
        # 얇은 것이 보행자·차량으로 잘못 올라가는 건 크기가 아니라 아래 모양 게이트
        # (min_road_user_width) 가 막는다. 여기서는 "있다" 는 사실만 남긴다.
        self.min_object_size = p('min_object_size_m', 0.05).value
        # 노면이 완전 평면이 아니라 지면 임계값을 살짝 넘는 점이 덩어리로 잡힌다.
        # 실측에서 ROI 지면위 점의 36% 가 0.15~0.30 m 구간(= 바닥)이었다.
        # 꼭대기가 이 높이도 안 되면 물체가 아니라 노면으로 본다.
        self.min_obstacle_top = p('min_obstacle_top_m', 0.30).value
        # 작은 덩어리는 더 높아야 인정한다. 실측에서 경로 안 424건 중 257건이 크기 0.5 m
        # 미만이었고, 카메라로 보니 그 자리에 아무것도 없었다 (노면 허상).
        # 라바콘 0.7 m·델리네이터 봉 0.77 m 는 이 기준을 통과한다.
        self.small_size = p('small_size_m', 0.5).value
        self.small_min_top = p('small_min_top_m', 0.5).value
        # 일직선으로 늘어선 덩어리 = 가드레일·방호벽. 도로 이용자로 승격시키지 않는다.
        # 실측: 가드레일 조각과 진짜 보행자는 속도(0.4~2.9)도 크기(0.33~1.17)도 완전히
        # 겹쳐 구분이 안 됐다. 남은 단서는 '줄지어 서 있다' 는 기하 구조뿐이다.
        self.chain_min = p('chain_min_count', 3).value
        self.chain_tol = p('chain_tol_m', 0.5).value
        self.chain_span = p('chain_span_m', 25.0).value
        # 줄의 구성원끼리 이만큼 넘게 벌어져 있으면 '이어진 구조물' 이 아니라
        # 각각 따로 있는 물체로 본다 (앞차 여러 대가 줄지어 선 경우).
        self.chain_gap = p('chain_gap_m', 1.5).value
        # --- 추적 / 분류 / 안정화 ---
        self.history_len = p('history_len', 5).value          # 5프레임 추적
        self.match_gate = p('match_gate_m', 2.0).value
        # 이동/정지 판정은 히스테리시스로 한다. 실측에서 25 m 쯤의 **정지한** 기둥도
        # 클러스터 중심이 떨려 0.43 m/s 까지 나왔다. 단일 임계값 0.5 를 쓰면 이런 게
        # 보행자로 튄다. 그래서 들어갈 때는 높게, 나올 때는 낮게 잡는다.
        self.moving_enter = p('moving_enter_mps', 0.8).value    # 이 이상이어야 '이동' 시작
        self.moving_exit = p('moving_exit_mps', 0.4).value      # 이 아래로 떨어져야 '정지' 복귀
        self.min_straightness = p('min_straightness', 0.6).value
        # 라바콘·연석처럼 낮은 것은 보행자·차량일 수 없다. 단 멀어지면 VLP16 수직 분해능이
        # 모자라 높이를 못 믿으므로(30 m 에서 빔 간격 1 m) 가까울 때만 적용한다.
        self.height_gate_m = p('height_gate_m', 20.0).value
        self.min_road_user_height = p('min_road_user_height_m', 0.6).value
        # 보행자·차량이 되려면 만족해야 하는 모양 조건. 벽을 링이 훑으면 교점이 벽을 따라
        # '곧고 빠르게' 미끄러져서 속도·직진성 검사를 통과해버린다. 그래서 모양으로 막는다.
        self.min_road_user_width = p('min_road_user_width_m', 0.25).value
        self.max_road_user_width = p('max_road_user_width_m', 2.6).value
        self.road_user_aspect_max = p('road_user_aspect_max', 4.0).value
        # 크기와 속도가 앞뒤가 맞아야 한다. 5분 실주행에서 35 m 앞 벽 조각(0.39×1.01 m)이
        # 9.4 m/s 로 미끄러지며 PEDESTRIAN 으로 216건 나왔다 — 사람은 그 속도로 못 뛰고,
        # 차는 1 m 일 수 없다. 둘 다 아니면 정적 장애물로 남긴다.
        self.ped_max_speed = p('ped_max_speed_mps', 3.0).value      # 대회 보행자는 10 km/h = 2.8
        self.vehicle_min_size = p('vehicle_min_size_m', 2.0).value  # 실제 차 4.5, 오토바이 2.0
        self.ped_max_size = p('ped_max_size_m', 1.2).value     # 이하면 보행자
        self.confirm_frames = p('confirm_frames', 3).value     # 3프레임 연속이면 발행
        # Roundabout handling (large vehicles kept, fragments joined) from the roundabout mission start, as the planner:
        # left at 850.93 when the mission moved to 800.92, trucks went unseen or in pieces up to s 851 while we already
        # decided there and two rolled into them (2026-10-03)
        self.fragment_merge_begin=p('fragment_merge_begin_s',800.92).value
        self.fragment_merge_end=p('fragment_merge_end_s',910.41).value
        self.max_misses = p('max_misses', 15).value            # 15프레임 놓치면 삭제
        # Tunnel (user 2026-10-04): a standing obstacle keeps the largest outline seen of it, and is kept on while it
        # drops out of sight close ahead. The 3 m low barricade read 3.0 m from 18 to 8 m, then 1.0 -> 0.2 m and nothing
        # from 4-5 m (Tunnel_LongBar_CP14): seen partly from close by, or not at all, it still stands there whole.
        self.tunnel_hold_speed = p('tunnel_hold_max_speed_mps', 0.5).value
        self.tunnel_hold_near = p('tunnel_hold_near_m', 10.0).value
        self.tunnel_hold_join = p('tunnel_hold_join_m', 1.5).value
        # kept unseen at most this long (passing a barricade from 5 m takes < 1 s at 20 km/h); a held outline that a
        # track seen now overlaps is that track's -- the held one is let go (a stale duplicate held on unseen beside
        # the barricade blocked the slalom and, standing, we never passed it: 04:35 run)
        self.tunnel_hold_unseen_frames = p('tunnel_hold_unseen_frames', 18).value
        self.odom_timeout = p('odom_timeout_s', 0.5).value
        self.publish_markers = p('publish_markers', True).value
        # --- 경로 투영 ---
        self.use_route = p('use_route', True).value
        self.route_topic = p('route_topic', '/map/route').value
        self.s_topic = p('s_topic', '/localization/s').value
        # **발행 기준은 하나다: "내가 갈 길을 막고 있는가".** 보행자인지 차량인지는 따지지 않는다.
        # 예전에는 정지물 2 m / 이동물 5 m 로 기준이 분류에 따라 달랐고, 오분류가 곧 필터 구멍이었다.
        # 실측 2026-09-20 (장애물이 하나도 스폰되지 않은 시나리오 = 발행된 189건 전부 오탐):
        #   OBSTACLE 170건 경로거리 중앙 1.98 m (임계 2.0 바로 아래) — 연석/가드레일 밑동
        #   PEDESTRIAN 11건 경로거리 2.06 m, VEHICLE 8건 경로거리 7.8 m / 도로밖 1.2 m
        # → 통로를 좁히고(1.5 m), 도로 밖은 움직이든 말든 무조건 버린다.
        self.corridor_half = p('corridor_half_m', 1.5).value
        # Tunnel: the slalom plans a side for every barricade on the pavement at once. A barricade in
        # the next lane was dropped (edge 2.9 m > 1.5 m) until the car had swerved next to it, 14 m
        # short of it (user 2026-09-29, newtunnelavoid).
        self.tunnel_corridor_half = p('tunnel_corridor_half_m', 6.0).value
        self.road_intensity_min = p('tunnel_road_intensity_min', 9.5).value
        self.road_intensity_max = p('tunnel_road_intensity_max', 11.5).value
        self.paint_intensity_min = p('tunnel_paint_intensity_min', 45.).value   # lane paint, measured
        self.paint_intensity_max = p('tunnel_paint_intensity_max', 112.).value  # floor paint p99 (36k tunnel floor points)
        self.paint_max_z = p('tunnel_paint_max_z', .4).value                     # above ground, measured
        # Highway (2026-10-01, 100 raw scans of HW980, height over the fitted road plane): asphalt 10-11 (90 % of
        # road returns, 0.3 % of on-road returns 0.5-3.5 m up), lane paint 30-118 (p99 118). Road users return
        # mostly 0 (68 %) and 149/154/190/255. Braking pitch lifts the asphalt over the height gate: dropped per
        # point. Paint also hits 8 % of road-user returns (a far car is often one ring), so only a cluster made
        # almost entirely of paint is dropped, never a point.
        self.highway_paint_intensity_min = p('highway_paint_intensity_min', 29.5).value
        self.highway_paint_intensity_max = p('highway_paint_intensity_max', 118.5).value
        self.highway_paint_cluster_fraction = p('highway_paint_cluster_fraction', .8).value
        # A car 30-45 m ahead gets one ring (-3 deg) on its lower body, 0.2-0.45 m up, 3-7 points (the -1 deg ring passes
        # over a 1.5 m roof): the range-rising ground gate (0.50 m at 35 m) and 5-point clusters dropped it, and a
        # 40 km/h car was first seen 18-25 m ahead at 107-110 km/h (contacts, 2026-10-01). On the highway the road is
        # removed by reflectivity (asphalt points, paint clusters) and the road gate, so: flat gate, 3-point clusters.
        # Replay, 3 laps, cars 30-45 m ahead: seen 73 -> 84 of 96; in-lane clusters with no car 12 in 420 scans.
        self.highway_min_samples = p('highway_dbscan_min_samples', 3).value
        # sight.py: the lowest highway car's roof (same-model lap 2026-10-02: 1.44-1.47 m) and how far the rings are traced
        self.highway_car_top_m = p('highway_car_top_m', 1.4).value
        self.highway_sight_far_m = p('highway_sight_far_m', 100.).value
        self.path_predict_s = p('path_predict_s', 2.0).value
        # 자차가 경로에서 이만큼 벗어나면 경로 필터를 끈다. 코스를 이탈했는데 인지까지
        # 먹통이 되면 안 되기 때문 — 경로를 모를 땐 버리지 말고 다 넘긴다.
        self.route_valid_dist = p('route_valid_dist_m', 10.0).value
        # --- 지도 기반 주행 가능 영역 (link_set.json 링크 합집합) ---
        # "도로 위인가" 는 분류와 무관한 기하 질문이다.
        self.link_set = p('link_set', str(geometry_path('package://foscar_bringup/maps/link_set.json', None))).value
        self.use_road_mask = p('use_road_mask', True).value
        # 도로 밖이면 **무엇이든** 버린다. 0.3 m 여유를 줬더니 연석이 통째로 들어왔다 (실측 위 참조).
        self.road_margin = p('road_margin_m', 0.0).value
        # 점 단계 컷: 이만큼 도로 밖인 점은 클러스터링 전에 버린다 (건물·수목·먼 구조물).
        self.road_point_margin = p('road_point_margin_m', 2.5).value
        self.stats_period = p('stats_period_s', 5.0).value   # 처리시간 요약 로그 주기 (0 이면 끔)

        self.hits_cap = self.confirm_frames + 2   # 신뢰 점수 상한 (과적립 방지)

        # --- 상태 ---
        self.tracks = []
        self.next_id = 0
        self.last_cloud_t = None
        self.odom_buf = deque(maxlen=100)     # (t, x, y, yaw)
        self.warned_odom_at = None
        self.warned_route_at = None
        self.route_xy = None                  # (N,2) 전역경로 map 좌표
        self.route_s = None                   # (N,) 경로 시작부터 누적거리
        self.ego_s = None                     # /localization/s (전역 s)
        self.tf_matrix = None                 # TF 로 얻은 (R, t). 없으면 None
        self.tf_last_try = None
        self.stats = {'frames': 0, 'ms': 0.0, 'peak_ms': 0.0, 'points': 0, 'since': None}
        self.dropped_off_route = 0            # 경로에서 멀어 버린 건수 (통계용)
        self.dropped_off_road = 0             # 도로 밖이라 버린 건수 (통계용)
        self.dropped_wide = 0                 # 도로를 가로질러 너무 넓어 버린 건수 (벽·레일)
        self.ego = None                       # 이번 프레임 자차 자세 (x, y, yaw)
        self.culled_points = 0                # 점 단계에서 도로 밖이라 버린 점 수 (통계용)

        # --- 도로 마스크 ---
        self.road = None
        if self.use_road_mask:
            try:
                self.road = RoadMask.from_file(self.link_set)
                if self.road is not None:
                    self.road.workers = self.workers
            except Exception as exc:                                   # 지도가 깨져도 노드는 살아야 한다
                self.get_logger().warn(f'link_set 읽기 실패 ({exc}) — 도로 마스크 없이 갑니다')
            if self.road is None:
                self.get_logger().warn(f'link_set 없음: {self.link_set} — 도로 마스크 없이 갑니다 '
                                       '(-p link_set:=경로 로 지정)')
            else:
                self.get_logger().info(f'도로 마스크: 링크 점 {self.road.half.size}개 '
                                       f'(도로밖 여유 {self.road_margin} m, 통로 반폭 {self.corridor_half} m)')

        # --- TF (있으면 쓰고 없으면 파라미터 폴백) ---
        if self.use_tf:
            import tf2_ros
            self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None

        # --- 통신 ---
        self.event_pub = self.create_publisher(Event, self.event_topic, 10)
        self.observation_pub = self.create_publisher(Event, self.observation_topic, 10)
        self.footprint_pub = self.create_publisher(String, '/perception/footprints', 10)
        self.clearance=ClearanceVerifier()
        self.raw_clearance_points=np.empty((0,3));self.raw_clearance_origin=np.zeros(3)
        self.mission_name=''
        self.mission_received_at=-math.inf
        self.create_subscription(String,'/behavior/mission',self.on_mission,1)
        self.create_subscription(String,'/perception/clearance_queries',self.on_clearance_queries,1)
        self.marker_pub = (self.create_publisher(MarkerArray, self.marker_topic, 1)
                           if self.publish_markers else None)
        self.create_subscription(PointCloud2, self.input_topic, self.on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom,
                                 qos_profile_sensor_data)
        if self.use_route:
            # 경로는 latched(TRANSIENT_LOCAL) 로 한 번만 나가므로 QoS 를 맞춰야 받는다
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Path, self.route_topic, self.on_route, latched)
            self.create_subscription(Float64, self.s_topic,
                                     lambda m: setattr(self, 'ego_s', float(m.data)),
                                     qos_profile_sensor_data)

        self.get_logger().info(
            f'obstacle_node 시작: {self.input_topic} → {self.event_topic} | '
            f'라이다 장착 x={self.lidar_x} y={self.lidar_y} z={self.lidar_z} '
            f'(use_tf={self.use_tf}) | ROI x<{self.roi_x_max} |y|<{self.roi_y_abs} '
            f'z {self.roi_z_min}~{self.roi_z_max}, 지면 {self.ground_z}')

    # ------------------------------------------------------------------
    # 자차 위치 (ego-motion 보상용)
    # ------------------------------------------------------------------

    def on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.odom_buf.append((stamp_to_sec(msg.header.stamp), pose.position.x,
                              pose.position.y, yaw_from_quat(pose.orientation)))

    def on_route(self, msg: Path) -> None:
        """전역경로를 받아 누적거리까지 미리 계산해 둔다 (latched 라 한 번만 온다)."""
        pts = np.array([[q.pose.position.x, q.pose.position.y] for q in msg.poses],
                       dtype=np.float64)
        if pts.shape[0] < 2:
            return
        step = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self.route_xy = pts
        self.route_s = np.concatenate([[0.0], np.cumsum(step)])
        self.get_logger().info(
            f'경로 수신: 점 {len(pts)}개, 길이 {self.route_s[-1]:.0f} m → Event.s 채웁니다')

    def nearest_index(self, mx: float, my: float, lo: int = 0, hi: int = None) -> int:
        """map 좌표에서 가장 가까운 경로 점의 인덱스."""
        seg = self.route_xy[lo:hi]
        return int(np.argmin((seg[:, 0] - mx) ** 2 + (seg[:, 1] - my) ** 2)) + lo

    def project_to_route(self, mx: float, my: float, near: int):
        """경로에 투영해서 (경로상 거리, 부호있는 횡거리) 를 준다.

        한 바퀴 도는 경로라 같은 지점을 두 번 지난다. 자차 근처 구간에서만 찾아야
        엉뚱한 곳에 투영되지 않는다. 점 간격이 0.5 m 라 점에만 붙이면 0.25 m 오차가
        생기므로 이웃한 두 선분에 각각 내려서 가까운 쪽을 쓴다.
        """
        pts, acc = self.route_xy, self.route_s
        lo = max(0, near - 60)                       # 뒤로 30 m
        hi = min(len(pts), near + 200)               # 앞으로 100 m
        k = self.nearest_index(mx, my, lo, hi)
        best = None
        for a in (k - 1, k):
            if a < 0 or a + 1 >= len(pts):
                continue
            p0, p1 = pts[a], pts[a + 1]
            d = p1 - p0
            length2 = float(d @ d)
            if length2 < 1e-9:
                continue
            length = math.sqrt(length2)
            u = float(np.clip(((mx - p0[0]) * d[0] + (my - p0[1]) * d[1]) / length2, 0.0, 1.0))
            foot = p0 + u * d
            # 외적 부호로 좌(+)/우(-) 를 가른다
            lat = ((mx - foot[0]) * d[1] - (my - foot[1]) * d[0]) / length
            dist = math.hypot(mx - foot[0], my - foot[1])
            if best is None or dist < best[0]:
                best = (dist, float(acc[a] + u * length), float(-lat))
        return None if best is None else (best[1], best[2])

    def ego_pose_at(self, t: float):
        """클라우드 stamp 에 가장 가까운 자차 자세. 너무 오래됐으면 None."""
        if not self.odom_buf:
            return None
        best = min(self.odom_buf, key=lambda e: abs(e[0] - t))
        if abs(best[0] - t) > self.odom_timeout:
            return None
        return best[1], best[2], best[3]

    def warn_no_odom(self, t: float) -> None:
        """odom 없을 때 5초에 한 번만 경고 (로그 폭주 방지)."""
        if self.warned_odom_at is None or t - self.warned_odom_at > 5.0:
            self.warned_odom_at = t
            self.get_logger().warn(
                f'{self.odom_topic} 가 없어 상대속도 모드로 동작합니다. '
                '주행 중에는 정지 장애물이 VEHICLE 로 오분류될 수 있습니다.')

    # ------------------------------------------------------------------
    # 1단계: 라이다 → base_link 좌표 변환
    # ------------------------------------------------------------------

    def lidar_to_base(self, pts: np.ndarray, frame_id: str, now: Time) -> np.ndarray:
        """포인트를 base_link 기준으로 옮긴다. TF 가 잡히면 회전까지 반영."""
        if self.tf_buffer is not None and frame_id and frame_id != self.base_frame:
            # 매 프레임 조회하면 비싸니 성공 시 캐시, 실패 시 1초에 한 번만 재시도
            if self.tf_matrix is None and (
                    self.tf_last_try is None or (now - self.tf_last_try).nanoseconds > 1e9):
                self.tf_last_try = now
                try:
                    tf = self.tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
                    trans = tf.transform.translation
                    self.tf_matrix = (quat_to_matrix(tf.transform.rotation),
                                      np.array([trans.x, trans.y, trans.z], dtype=np.float32))
                    self.get_logger().info(
                        f'{self.base_frame}→{frame_id} TF 사용: '
                        f'({trans.x:.2f}, {trans.y:.2f}, {trans.z:.2f})')
                except Exception:
                    pass   # 아직 TF 발행 전. 아래 파라미터 오프셋으로 진행
            if self.tf_matrix is not None:
                rot, trans = self.tf_matrix
                return pts @ rot.T + trans
        # 폴백: 파라미터 오프셋 (평면 장착 가정, 회전 없음)
        return pts + np.array([self.lidar_x, self.lidar_y, self.lidar_z], dtype=np.float32)

    # ------------------------------------------------------------------
    # 메인 콜백
    # ------------------------------------------------------------------

    def on_clearance_queries(self,msg):
        try:self.clearance.accept(json.loads(msg.data))
        except (ValueError,TypeError):return

    def on_cloud(self, msg: PointCloud2) -> None:
        t = stamp_to_sec(msg.header.stamp)

        # 같은 stamp 재수신은 무시, 시간이 뒤로 가면(bag 되감기) 추적 상태를 통째로 리셋
        if self.last_cloud_t is not None:
            if t <= self.last_cloud_t - 1e-3:
                self.get_logger().warn('stamp 가 뒤로 갔습니다. 추적 상태를 초기화합니다.')
                self.tracks.clear()
                self.odom_buf.clear()
            elif abs(t - self.last_cloud_t) < 1e-6:
                return
        self.last_cloud_t = t

        started = perf_counter()
        self.raw_clearance_points=np.empty((0,3))
        dets = self.detect(msg, t)
        self.track_and_publish(dets, msg.header.stamp, t)
        self.update_stats(t, (perf_counter() - started) * 1e3, len(dets))

    def update_stats(self, t: float, elapsed_ms: float, n_dets: int) -> None:
        """10 Hz 를 지키고 있는지 주기적으로 요약한다 (완료 기준 항목).

        여유가 없으면 max_points 를 줄이거나 ROI 를 좁히면 된다.
        """
        if self.stats_period <= 0.0:
            return
        st = self.stats
        if st['since'] is None:
            st['since'] = t
        st['frames'] += 1
        st['ms'] += elapsed_ms
        st['peak_ms'] = max(st['peak_ms'], elapsed_ms)
        st['points'] = n_dets
        if t - st['since'] >= self.stats_period:
            self.get_logger().info(
                f"{st['frames'] / (t - st['since']):.1f} Hz | 처리 "
                f"{st['ms'] / st['frames']:.1f} ms (최대 {st['peak_ms']:.1f}) | "
                f'덩어리 {n_dets}, 추적 {len(self.tracks)}'
                + (f', 경로밖 제외 {self.dropped_off_route}' if self.dropped_off_route else '')
                + (f', 도로밖 제외 {self.dropped_off_road}' if self.dropped_off_road else '')
                + (f', 벽·레일 제외 {self.dropped_wide}' if self.dropped_wide else '')
                + (f', 도로밖 점 {self.culled_points}' if self.culled_points else ''))
            self.dropped_off_route = self.dropped_off_road = self.culled_points = self.dropped_wide = 0
            self.stats = {'frames': 0, 'ms': 0.0, 'peak_ms': 0.0, 'points': 0, 'since': t}

    # ------------------------------------------------------------------
    # 2~4단계: ROI → 지면 제거 → 클러스터링
    # ------------------------------------------------------------------

    def to_map(self, pts: np.ndarray, t: float):
        """base_link 점들을 map 좌표로. odom 이 없으면 None (마스크를 쓸 수 없다)."""
        ego = self.ego_pose_at(t)
        if ego is None:
            return None
        ex, ey, eyaw = ego
        c, sn = math.cos(eyaw), math.sin(eyaw)
        return np.column_stack((ex + c * pts[:, 0] - sn * pts[:, 1],
                                ey + sn * pts[:, 0] + c * pts[:, 1]))

    def cull_off_road(self, pts: np.ndarray, t: float, with_mask: bool = False):
        """지도상 도로에서 road_point_margin 넘게 벗어난 점을 버린다. with_mask: (점, 남긴 마스크 또는 None)."""
        if self.road is None or pts.shape[0] == 0:
            return (pts, None) if with_mask else pts
        xy = self.to_map(pts, t)
        if xy is None:                       # 자차 위치를 모르면 지도를 못 쓴다 → 거르지 않는다
            return (pts, None) if with_mask else pts
        keep = self.road.outside(xy) <= self.road_point_margin
        self.culled_points += int(pts.shape[0] - keep.sum())
        return (pts[keep], keep) if with_mask else pts[keep]

    def highway_active(self):
        sensor=getattr(self,'highway_sensor',None)
        return sensor is not None and sensor.active(self.ego_s)

    def longitudinal_roi_mask(self, pts: np.ndarray, t: float) -> np.ndarray:
        sensor=getattr(self,'highway_sensor',None)
        if sensor is not None and sensor.active(self.ego_s) and self.road is not None:
            keep=(pts[:,0]>-sensor.rear_m)&(pts[:,0]<sensor.front_m)
            xy=self.to_map(pts,t)
            return keep & (self.road.outside(xy)<=self.road_margin) if xy is not None else np.zeros(len(pts),dtype=bool)
        keep=(pts[:,0]>self.roi_x_min)&(pts[:,0]<self.roi_x_max)
        if (self.ego_s is None or self.road is None or
            not self.roundabout_observation_begin<=self.ego_s<=self.roundabout_observation_end):return keep
        extra=np.flatnonzero(((pts[:,0]>self.roundabout_roi_x_min)&(pts[:,0]<=self.roi_x_min))
                             |((pts[:,0]>=self.roi_x_max)&(pts[:,0]<self.roundabout_roi_x_max)))
        if len(extra):
            xy=self.to_map(pts[extra],t)
            if xy is not None:keep[extra]=self.road.outside(xy)<=self.road_margin
        return keep

    def lateral_roi_mask(self, pts: np.ndarray, t: float) -> np.ndarray:
        """Observe the next circulating car before the current car has passed.

        Extra lateral returns are admitted only on mapped roads in the existing
        roundabout mission area. Original near-road pedestrian coverage stays
        unchanged; missing pose/map cannot enlarge the ROI.
        """
        sensor=getattr(self,'highway_sensor',None)
        if sensor is not None and sensor.active(self.ego_s):return np.abs(pts[:,1])<sensor.lateral_m
        lateral=np.abs(pts[:,1])
        keep=lateral<self.roi_y_abs
        if (self.ego_s is None or self.road is None or
            not self.roundabout_observation_begin<=self.ego_s<=self.roundabout_observation_end):
            return keep
        extra=np.flatnonzero((~keep)&(lateral<self.roundabout_roi_y_abs))
        if len(extra):
            xy=self.to_map(pts[extra],t)
            if xy is not None:keep[extra]=self.road.outside(xy)<=self.road_margin
        return keep

    def on_mission(self, msg):
        try:
            data=json.loads(msg.data)
            name=data.get('mission') if isinstance(data,dict) else None
            if not isinstance(name,str):return
        except (ValueError,TypeError):
            return
        self.mission_name=name
        self.mission_received_at=perf_counter()

    def detect(self, msg: PointCloud2, t: float) -> list:
        self.highway_scan_valid=False
        self.raw_clearance_points=np.empty((0,3))
        self.tunnel_walls=[]
        # Mission ownership, not GPS validity or tunnel-looking geometry, gates
        # this simulator-specific material filter. Stale ownership expires.
        owned=0<=perf_counter()-self.mission_received_at<=1. and any(f.name=='intensity' for f in msg.fields)
        filter_zero=owned and self.mission_name=='TUNNEL'
        material=owned and self.mission_name=='HIGHWAY'
        # practice diagnostic (2026-10-03): a static ghost on the ring lane near map (-106.75, 326.22) seen from s 863-867
        ghost_diag=(owned and self.mission_name=='ROUNDABOUT' and self.ego_s is not None and 850.<=self.ego_s<=872.)
        # Roundabout: asphalt returns (intensity 9.5-11.5, as on the highway) are dropped -- the road surface came up as
        # flat 0.3-0.5 m ghosts on the ring lane while we braked for the entry, intensity 10-11 (user 2026-10-03)
        ring_road=owned and self.mission_name=='ROUNDABOUT'
        # practice diagnostic (RB_LIDAR_DIAG): every cluster and track near us at the roundabout entry, and the gate that
        # dropped it -- a car crossing our entry left the ring runs for ~0.8 s (03:13 run 1)
        self.ring_diag=(self.ego is not None and ((owned and self.mission_name=='ROUNDABOUT' and self.ego_s is not None
                        and 850.<=self.ego_s<=885.) or os.environ.get('FOSCAR_LIDAR_DIAG','')=='1'))
        self.ring_diag_cl=[]
        self.diag_r=float(os.environ.get('FOSCAR_LIDAR_DIAG_R','20') or 20.)
        # intensity is read everywhere: a person's returns sit at 17-20 (2026-10-04, pedestrian mission), used with
        # the shape to tell road users apart
        has_int=any(f.name=='intensity' for f in msg.fields)
        fields=('x','y','z','intensity') if has_int else ('x','y','z')
        raw = pc2.read_points_numpy(msg, field_names=fields, skip_nans=False)
        if raw.size == 0:
            return []
        raw=raw.reshape(-1,len(fields))
        raw=raw[np.isfinite(raw[:,:3]).all(axis=1)]
        pts = raw[:,:3].astype(np.float32, copy=False)

        pts = self.lidar_to_base(pts, msg.header.frame_id, self.get_clock().now())
        if ghost_diag and t-getattr(self,'ghost_logged_at',-1.)>=.2:
            xy=self.to_map(pts,t)
            if xy is not None:
                near=np.hypot(xy[:,0]+106.75,xy[:,1]-326.22)<4.
                if near.any():
                    self.ghost_logged_at=t;z=pts[near,2];it=raw[near,3]
                    self.get_logger().info('RB_GHOST_PTS '+json.dumps(dict(t=round(t,3),s=round(float(self.ego_s),2),n=int(near.sum()),
                        z=[round(float(z.min()),2),round(float(np.median(z)),2),round(float(z.max()),2)],
                        i=[round(float(np.percentile(it,q)),1) for q in (10,50,90,100)],
                        xy=[round(float(xy[near,0].mean()),2),round(float(xy[near,1].mean()),2)])))
        # Braking tilts the observed road above the fixed z gate. Estimate a
        # broad near-ground plane before ROI filtering; sparse/occluded scenes
        # fall back to the existing height reference. XY and tracking stay in base_link.
        plane = estimate_ground_plane(pts)
        # Highway: how far ahead a car body is still met by a ring at this scan's pitch (sight.py). A blind stretch
        # reaching the end of the ROI means a car can come up unseen: the sight ends where it starts.
        self.sight=None
        if material:
            far=self.highway_sight_far_m
            bands=blind_bands(plane,self.lidar_x,self.lidar_z,body_high=self.highway_car_top_m,far=far)
            reach=min([lo for lo,hi in bands if hi>=far-.5],default=far)
            self.sight=dict(pitch_deg=round(pitch_deg(plane),3),range_m=round(reach,1),plane=plane is not None)
        pts[:, 2] = relative_height(pts, plane)
        self.raw_clearance_points=pts.copy()
        origin=self.lidar_to_base(np.zeros((1,3)),msg.header.frame_id,self.get_clock().now())
        origin[:,2]=relative_height(origin,plane)
        self.raw_clearance_origin=origin[0]
        self.highway_scan_valid=bool(np.isfinite(pts).all() and np.isfinite(origin).all())

        # Keep the unfiltered returns for clearance evidence and leave the
        # source point-cloud topic untouched. Exactly zero is excluded from
        # obstacle clustering; missing/unknown intensity is not assumed zero.
        if filter_zero:
            faces=wall_faces(pts,raw[:,3])
            xy=self.to_map(np.array([(fx,fy) for fx,fy,_ in faces]),t) if faces else None
            if xy is not None:
                self.tunnel_walls=[[float(px),float(py),side] for (px,py),(_,_,side) in zip(xy,faces)]
            # Asphalt returns at intensity 10-11 (58 % of tunnel floor points, 0.7 % of barricade points,
            # 2026-09-29 practice/sim/lidar_intensity.py): braking pitch lifts the floor above the height
            # gate and it became phantom obstacles next to the barricades (user: filter by reflectivity).
            road=(raw[:,3]>=self.road_intensity_min)&(raw[:,3]<=self.road_intensity_max)
            # Lane paint (solid/dashed lines) returns at intensity >= 45 (4 % of tunnel floor points; asphalt
            # 92.5 % at 10-12). Lifted above the height gate by braking pitch (z 0.26-0.32) it widened the
            # second barricade's box by 0.3 m toward the gap and the car stopped (2026-09-29, raw points of
            # newtunnelavoid). Low paint-bright points are floor: removes 57 of 68 such points there and
            # 0.4 % of the barricade's own points.
            # Only up to the paint's own brightness: a 0.85 m barricade board's low points (median 127) were all
            # taken for paint 11.8 m before it (2026-09-29 CP14, raw points); floor paint p99 is 112.
            paint=((raw[:,3]>=self.paint_intensity_min)&(raw[:,3]<=self.paint_intensity_max)
                   &(pts[:,2]<self.paint_max_z))
            zero_keep=(raw[:,3]!=0.)&~road&~paint
            pts=pts[zero_keep]
        intensity=raw[zero_keep,3].copy() if filter_zero and has_int else None
        if material:
            road=(raw[:,3]>=self.road_intensity_min)&(raw[:,3]<=self.road_intensity_max)
            pts=pts[~road];intensity=raw[~road,3]
        elif not ring_road and not filter_zero and has_int:
            intensity=raw[:,3].copy()
        elif ring_road:
            road=(raw[:,3]>=self.road_intensity_min)&(raw[:,3]<=self.road_intensity_max)
            pts=pts[~road]
            intensity=raw[~road,3].copy()
            if ghost_diag:raw=raw[~road]

        # ROI + 지면 제거를 불리언 마스크 하나로. 파이썬 루프 없이 벡터 연산만 쓴다.
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        # 지면 임계값은 점마다 다르다 (거리에 비례해서 올라간다)
        # In the tunnel the floor is flat and its asphalt/paint returns are already dropped by reflectivity above,
        # so the gate does not rise with range: rising 0.01 m/m it hid the 0.7 m tunnel boxes from 22 to 28 m (every
        # ring passed over them) and the first box was confirmed at 28-30 m, in 2 of 8 runs only at 11-12 m. Flat, the
        # same recorded scans confirm it at 33-36 m (2026-09-30).
        slope = 0. if filter_zero or material else self.ground_slope_tol
        ground = np.maximum(self.roi_z_min, self.ground_z + slope * np.maximum(x, 0.0))
        keep = (self.longitudinal_roi_mask(pts,t)
                & self.lateral_roi_mask(pts,t)
                & (z > ground) & (z < self.roi_z_max))
        pts = pts[keep]
        if intensity is not None:intensity=intensity[keep]
        if pts.shape[0] < self.min_samples:
            return []

        # 지도상 도로에서 한참 벗어난 점은 여기서 버린다. 클러스터링 **전에** 버려야
        # 가드레일·벽 조각이 진짜 물체와 한 덩어리로 묶이는 일도 같이 막힌다.
        # 여유(road_point_margin)를 넉넉히 둬서 인도의 보행자는 살려 둔다 — 도로 경계에
        # 딱 붙은 가드레일은 뒤의 객체 단계(on_route_path)에서 거른다.
        pts, kept = self.cull_off_road(pts, t, with_mask=True)
        if intensity is not None and kept is not None:intensity=intensity[kept]
        if pts.shape[0] < self.min_samples:
            return []

        # 10 Hz 를 지키기 위한 방어: 점이 너무 많으면 균일하게 솎아낸다
        if pts.shape[0] > self.max_points:
            idx = np.random.default_rng(0).choice(pts.shape[0], self.max_points, replace=False)
            pts = pts[idx]
            if intensity is not None:intensity=intensity[idx]

        # DBSCAN 은 x, y 평면에서만. 높이는 물체 구분에 도움이 안 되고
        # (사람 머리와 발이 갈라짐) 차원이 늘면 느려진다.
        # kd_tree + 멀티코어: 결과는 기본값과 완전히 동일한데 30% 빠르다
        # (실측 3962점 기준 37.5 → 26.3 ms). 복셀 다운샘플이 더 빠르지만(9.7 ms)
        # 점이 적은 먼 거리 물체를 통째로 잃어서 쓰지 않는다.
        labels = DBSCAN(eps=self.eps, min_samples=self.highway_min_samples if material else self.min_samples,
                        algorithm='kd_tree', n_jobs=self.workers).fit_predict(pts[:, :2])

        dets = []
        for label in np.unique(labels):
            if label < 0:        # -1 = 노이즈
                continue
            cluster = pts[labels == label]
            if intensity is not None and material:
                ci=intensity[labels==label]
                paint=(ci>=self.highway_paint_intensity_min)&(ci<=self.highway_paint_intensity_max)
                if paint.mean()>=self.highway_paint_cluster_fraction:
                    if self.ring_diag:
                        cxy=self.to_map(cluster[:,:3].mean(axis=0,keepdims=True),t)
                        if cxy is not None and math.hypot(cxy[0,0]-self.ego[0],cxy[0,1]-self.ego[1])<self.diag_r:
                            lo_,hi_=cluster.min(axis=0),cluster.max(axis=0)
                            self.ring_diag_cl.append([round(float(cxy[0,0]),1),round(float(cxy[0,1]),1),
                                round(float(hi_[0]-lo_[0]),1),round(float(hi_[1]-lo_[1]),1),round(float(hi_[2]-lo_[2]),1),
                                int(len(cluster)),'paint',[round(float(np.percentile(ci,q)),1) for q in (10,50,90)]
                                +[round(float(paint.mean()),2)],dict(rng=round(float(np.hypot(*cluster[:,:2].mean(axis=0))),1),
                                zmax=round(float(hi_[2]),2))])
                    continue
            lo, hi = cluster.min(axis=0), cluster.max(axis=0)
            if ghost_diag and intensity is not None:
                cxy=self.to_map(cluster[:,:3].mean(axis=0,keepdims=True),t)
                if cxy is not None and math.hypot(cxy[0,0]+106.75,cxy[0,1]-326.22)<6.:
                    ci=intensity[labels==label]
                    self.get_logger().info('RB_GHOST_CL '+json.dumps(dict(t=round(t,3),s=round(float(self.ego_s),2),
                        c=[round(float(cxy[0,0]),2),round(float(cxy[0,1]),2)],n=int(len(cluster)),
                        size=[round(float(hi[0]-lo[0]),2),round(float(hi[1]-lo[1]),2)],z=[round(float(lo[2]),2),round(float(hi[2]),2)],
                        i=[round(float(np.percentile(ci,q)),1) for q in (10,50,90)],hi_i=round(float((ci>=200).mean()),2))))
            size_x, size_y = float(hi[0] - lo[0]), float(hi[1] - lo[1])
            footprint = observed_hull(cluster[:,:2])
            if self.ring_diag:
                cxy=self.to_map(cluster[:,:3].mean(axis=0,keepdims=True),t)
                if cxy is not None and math.hypot(cxy[0,0]-self.ego[0],cxy[0,1]-self.ego[1])<self.diag_r:
                    big=max(size_x,size_y);top_=float(hi[2])
                    why=('size' if not self.accept_cluster_size(footprint,size_x,size_y,float(hi[2]-lo[2]),t) else
                         'small' if big<self.min_object_size else 'low' if top_<self.min_obstacle_top else
                         'smalllow' if big<self.small_size and top_<self.small_min_top else 'kept')
                    ci=intensity[labels==label] if intensity is not None else None
                    iq=[round(float(np.percentile(ci,q)),1) for q in (10,50,90)]+[round(float((ci>=100).mean()),2)] if ci is not None and len(ci) else None
                    # shape features (practice: what tells road users apart): oriented length/width, heights, spread,
                    # range, lidar layers crossed (elevation angles from the measured mount 2.0 m ahead, 2.11 m up)
                    ln,wd=oriented_dimensions(footprint);z=cluster[:,2]
                    rng=float(np.hypot(cluster[:,0].mean(),cluster[:,1].mean()))
                    el=np.degrees(np.arctan2(z-2.11,np.hypot(cluster[:,0]-2.,cluster[:,1])))
                    zb=np.histogram(z,bins=[-1.,.3,.6,.9,1.2,1.5,1.8,3.])[0]
                    feat=dict(len=round(ln,2),wid=round(wd,2),zmin=round(float(z.min()),2),zmax=round(float(z.max()),2),
                              zmed=round(float(np.median(z)),2),zstd=round(float(z.std()),2),rng=round(rng,1),
                              layers=int(len(np.unique(np.round(el/1.)))),zbins=[int(v) for v in zb],
                              imax=None if ci is None or not len(ci) else round(float(ci.max()),1))
                    self.ring_diag_cl.append([round(float(cxy[0,0]),1),round(float(cxy[0,1]),1),round(size_x,1),round(size_y,1),
                                              round(float(hi[2]-lo[2]),1),int(len(cluster)),why,iq,feat])
            if not self.accept_cluster_size(footprint,size_x,size_y,float(hi[2]-lo[2]),t):
                continue
            # 너무 작은 것 = 링이 벽을 스친 조각
            if max(size_x, size_y) < self.min_object_size:
                continue
            # 꼭대기가 낮은 것 = 지면 임계값을 살짝 넘긴 노면
            top = float(hi[2])
            if top < self.min_obstacle_top:
                continue
            # 작으면서 낮기까지 하면 물체가 아니라 노면 허상이다
            if max(size_x, size_y) < self.small_size and top < self.small_min_top:
                continue
            center = cluster.mean(axis=0)
            ped = self.pedestrian_features(cluster, footprint,
                                           intensity[labels == label] if intensity is not None else None)
            dets.append({
                'x': float(center[0]), 'y': float(center[1]),
                'size_x': size_x, 'size_y': size_y, 'size_z': float(hi[2] - lo[2]),
                'z_max': top,
                'n': int(cluster.shape[0]),
                'footprint': footprint,
                'ped': ped,
            })
        # Preserve the original chain evidence before reducing cluster count.
        self.mark_chains(dets)
        if self.ring_diag:
            ex,ey,eyaw=self.ego;c_,s_=math.cos(eyaw),math.sin(eyaw)
            self.ring_diag_det=[[round(ex+c_*d['x']-s_*d['y'],1),round(ey+s_*d['x']+c_*d['y'],1),round(d['size_x'],1),
                                 round(d['size_y'],1),int(d['structure'])] for d in dets if math.hypot(d['x'],d['y'])<20.]
        # ...and on the highway too (user 2026-10-05, as on the ring): a 4-point piece 1.2 m off a lane-4 car's side, alone,
        # had its centre in lane 3 and was taken as our lead there; the pass past lane 2 was dropped (04:25 run, s1297).
        highway=getattr(self,'highway_sensor',None)
        if self.ego_s is not None and (self.fragment_merge_begin<=self.ego_s<=self.fragment_merge_end
                                       or (highway is not None and highway.active(self.ego_s))):
            dets=merge_vehicle_fragments(dets)
        return dets

    def accept_cluster_size(self, footprint, size_x, size_y, height, t=None):
        if max(size_x,size_y)<=self.max_object_size:return True
        highway=getattr(self,'highway_sensor',None)
        if (not fresh_mission(self, 'STATIC_OBSTACLE')
                and not (highway is not None and highway.active(self.ego_s))
                and (self.ego_s is None or not self.fragment_merge_begin<=self.ego_s<=self.fragment_merge_end)):
            return False
        length,width=oriented_dimensions(footprint)
        # Large bodies still pass the mapped-road and structure gates. The static
        # obstacle mission can contain the same truck-sized geometry as traffic.
        # Thin rails, low curbs and broad connected walls are not rescued.
        if length>self.max_vehicle_length or width>self.max_vehicle_width or height<.8:return False
        if width>=.6:return True
        # A truck side can be a thin vertical sheet. Rescue it only with tall
        # returns centered inside mapped pavement, not at a roadside boundary.
        if t is None or height<1.2 or self.road is None:return False
        xy=self.to_map(np.asarray(footprint),t)
        return xy is not None and self.road.outside_one(*xy.mean(axis=0))<-.3

    def tunnel_hold_update(self, tr, ego) -> None:
        """Tunnel: keep a standing obstacle's outline as the union of what was seen of it (map frame) and give its box
        out from that, in base_link as the scan's own boxes are."""
        # Not for a person or animal: a deer standing at first was held, then ran across in steps each within the
        # join distance of the held outline, and its events kept an outline from mid-road -- 2 m onto the road from
        # where it stood (its footprint was right), so YIELD never saw it off our road (2026-10-10).
        if not self.tunnel_owned() or ego is None or tr.ped_confirmed or tr.kind == 'PEDESTRIAN':
            tr.hold_fp = None
            return
        # standing: its position over the track's history (>= 0.4 s of it), not the speed fitted to its centre -- a
        # barricade seen ever less of as we close in moved its centre, read as moving and was given up (04:15 runs)
        h = list(tr.history)
        old = h[0] if h and h[-1][0]-h[0][0] >= .4 else None
        static = (old is not None and math.hypot(h[-1][1]-old[1], h[-1][2]-old[2])/(h[-1][0]-old[0]) < self.tunnel_hold_speed)
        hold = getattr(tr, 'hold_fp', None)
        if tr.misses == 0 and len(tr.footprint_map) >= 2:
            new = np.asarray(tr.footprint_map, dtype=float).reshape(-1, 2)
            if hold is not None:
                # held until what is seen leaves the held outline (the object moved); less of it seen is no motion. The
                # longest single outline seen is kept, not a union: merged scan after scan the 3.0 m barricade grew to
                # 3.5 m and the slalom stopped beside it (04:18 run)
                gap = float(np.min(np.linalg.norm(new[:, None, :]-hold[None, :, :], axis=2)))
                inside = gap <= self.tunnel_hold_join or bool(overlaps(hold, new))
                if not inside:
                    tr.hold_fp = None
                elif oriented_dimensions(new)[0] >= oriented_dimensions(hold)[0]-.1:
                    tr.hold_fp = new
            elif static:
                tr.hold_fp = new
        hold = getattr(tr, 'hold_fp', None)
        if hold is None or len(hold) < 2:
            return
        ex, ey, eyaw = ego; c, sn = math.cos(eyaw), math.sin(eyaw)
        bx = c*(hold[:, 0]-ex)+sn*(hold[:, 1]-ey); by = -sn*(hold[:, 0]-ex)+c*(hold[:, 1]-ey)
        tr.x, tr.y = float((bx.min()+bx.max())/2), float((by.min()+by.max())/2)
        tr.size_x, tr.size_y = float(max(np.ptp(bx), tr.size_x if tr.misses == 0 else 0.)), float(max(np.ptp(by), tr.size_y if tr.misses == 0 else 0.))
        tr.hold_near = (bx.max() > -1. and float(np.min(np.hypot(bx, by))) < self.tunnel_hold_near)

    def tunnel_held_near(self, tr) -> bool:
        """A tunnel obstacle given out before, standing close ahead (not yet passed): kept and given out even unseen."""
        return (self.tunnel_owned() and getattr(tr, 'held_published', False) and getattr(tr, 'hold_fp', None) is not None
                and getattr(tr, 'hold_near', False) and tr.misses <= self.tunnel_hold_unseen_frames)

    def ring_window(self) -> bool:
        return (getattr(self,'mission_name',None)=='ROUNDABOUT' and self.ego_s is not None
                and self.roundabout_observation_begin<=self.ego_s<=self.roundabout_observation_end)

    def on_ring_lane(self, d) -> bool:
        """A detection (base_link x, y) centred on the ring lane."""
        if self.ego is None:
            return False
        ex, ey, eyaw = self.ego; c, s = math.cos(eyaw), math.sin(eyaw)
        mx, my = ex+c*d['x']-s*d['y'], ey+s*d['x']+c*d['y']
        return abs(math.hypot(mx-self.ring_center[0], my-self.ring_center[1])-self.ring_radius) <= self.ring_lane_half

    def mark_chains(self, dets: list) -> None:
        """일직선으로 늘어선 덩어리들을 구조물로 표시한다 (가드레일·방호벽·라바콘 줄).

        실제 차량(2 m 이상)은 후보에서 빼서, 앞차 여러 대가 한 줄로 선 상황을
        구조물로 오인하지 않게 한다.
        """
        for d in dets:
            d['structure'] = False
        ring = self.ring_window()
        cand = [i for i in range(len(dets)) if not (ring and self.on_ring_lane(dets[i]))]
        if len(cand) < self.chain_min:
            return
        pos = np.array([[dets[i]['x'], dets[i]['y']] for i in cand])
        for a in range(len(cand)):
            for b in range(a + 1, len(cand)):
                v = pos[b] - pos[a]
                length = float(np.hypot(v[0], v[1]))
                if not (1.0 < length < self.chain_span):
                    continue
                nx, ny = -v[1] / length, v[0] / length
                rel = pos - pos[a]
                perp = np.abs(rel[:, 0] * nx + rel[:, 1] * ny)
                along = (rel[:, 0] * v[0] + rel[:, 1] * v[1]) / (length * length)
                on_line = (perp < self.chain_tol) & (along > -0.05) & (along < 1.05)
                if int(on_line.sum()) < self.chain_min:
                    continue
                # 줄지어 선 앞차들과 구분한다. 가드레일 조각은 서로 맞닿아 있고
                # (사이가 거의 안 벌어짐), 차들은 앞뒤로 확실히 떨어져 있다.
                members = np.where(on_line)[0]
                order = members[np.argsort(along[members])]
                centers = along[order] * length
                halves = np.array([max(dets[cand[k]]['size_x'],
                                       dets[cand[k]]['size_y']) / 2.0 for k in order])
                gaps = np.diff(centers) - (halves[:-1] + halves[1:])
                if gaps.size and float(np.max(gaps)) > self.chain_gap:
                    continue       # 사이가 벌어져 있다 = 각각 따로 있는 물체
                for k in order:
                    dets[cand[k]]['structure'] = True

    # ------------------------------------------------------------------
    # 5~6단계: 추적 → 분류 → 안정화 → 발행
    # ------------------------------------------------------------------

    def track_and_publish(self, dets: list, stamp, t: float) -> None:
        # 관측을 map 좌표로 올린다. odom 이 없으면 base_link 를 그대로 쓰는
        # 상대속도 모드로 강등 (정확도는 떨어지지만 노드는 계속 돈다).
        ego = self.ego_pose_at(t)
        if ego is None:
            self.warn_no_odom(t)
            for d in dets:
                d['mx'], d['my'] = d['x'], d['y']
        else:
            ex, ey, eyaw = ego
            c, s = np.cos(eyaw), np.sin(eyaw)
            for d in dets:
                d['mx'] = ex + c * d['x'] - s * d['y']
                d['my'] = ey + s * d['x'] + c * d['y']
                footprint=self.to_map(np.asarray(d['footprint']),t)
                d['footprint_map']=footprint.tolist() if footprint is not None else []

        self.ego = ego
        matched = self.match(dets, t)

        # 매칭 안 된 관측은 새 트랙으로
        for i, d in enumerate(dets):
            if i in matched:
                continue
            track = Track(self.next_id, self.history_len)
            self.next_id += 1
            track.update(t, d, self.hits_cap)
            self.tracks.append(track)

        for tr in self.tracks:
            if tr.misses == 0:
                tr.motion_in_map = ego is not None
            self.tunnel_hold_update(tr, ego)
        if self.tunnel_owned():
            seen = [np.asarray(tr.footprint_map, dtype=float).reshape(-1, 2) for tr in self.tracks
                    if tr.misses == 0 and len(tr.footprint_map) >= 3]
            for tr in self.tracks:
                hold = getattr(tr, 'hold_fp', None)
                if tr.misses > 0 and hold is not None and len(hold) >= 3 and any(overlaps(hold, f) for f in seen):
                    tr.hold_fp = None

        # 오래 못 본 트랙 정리 (터널에서 가까이 앞에 서 있던 장애물은 지나칠 때까지 둔다)
        self.tracks = [tr for tr in self.tracks if tr.misses < self.max_misses or self.tunnel_held_near(tr)]

        # 이번 프레임에 실제로 본 + 3프레임 이상 연속 확인된 트랙만 발행한다.
        # (Event.msg 규약: 보이는 동안 10 Hz 이상 재발행)
        # 경로가 있으면 자차를 먼저 투영해 둔다 (물체 투영의 검색 기준점 + 전역 s 보정값)
        ego_idx, s_shift = None, None
        if self.route_xy is not None and ego is not None:
            near = self.nearest_index(ego[0], ego[1])
            ego_proj = self.project_to_route(ego[0], ego[1], near)
            if ego_proj is not None and abs(ego_proj[1]) <= self.route_valid_dist:
                ego_idx = near
                if self.ego_s is not None:
                    s_shift = self.ego_s - ego_proj[0]   # 지역 누적거리 → 전역 s
            elif ego_proj is not None:
                self.warn_off_route(t, abs(ego_proj[1]))

        published = []
        footprints = []
        diag_tr = []
        for tr in self.tracks:
            if tr.misses == 0:
                tr.kind = self.classify(tr)          # 프레임당 한 번 (히스테리시스 상태 갱신)
            held = self.tunnel_held_near(tr)
            if (tr.misses == 0 or held) and (tr.hits >= self.confirm_frames or self.highway_active() or held
                                   or (self.ring_fp_first and self.ring_window())):
                relevant, observe = (self.output_gates(tr, ego_idx, s_shift) if tr.hits >= self.confirm_frames or held
                                     else (False,False))
                on_road=(self.road is not None and tr.history and
                         self.observation_on_road(tr.history[-1][1],tr.history[-1][2]))
                if ego is not None and tr.footprint and not tr.structure and on_road:
                    xy=self.to_map(np.asarray(tr.footprint),t)
                    if xy is not None:
                        footprints.append(dict(id=f'lidar:{self.track_namespace}:{tr.id}',xy=xy.tolist(),
                                               center=list(tr.history[-1][1:]),vx=tr.vx,vy=tr.vy,
                                               kind=tr.kind,on_road=True,height=float(tr.z_max)))
                if getattr(self,'ring_diag',False) and tr.history:
                    hx,hy=tr.history[-1][1],tr.history[-1][2]
                    if math.hypot(hx-self.ego[0],hy-self.ego[1])<20.:
                        diag_tr.append([tr.id,round(hx,1),round(hy,1),tr.hits,int(tr.structure),int(bool(on_road)),
                                        int(bool(tr.footprint)),round(max(tr.size_x,tr.size_y),1),
                                        getattr(tr,'gate_why',None),int(bool(relevant)),tr.kind])
                if relevant:
                    tr.published = True
                    tr.held_published = getattr(tr, 'hold_fp', None) is not None
                    self.event_pub.publish(self.make_event(tr, stamp))
                    published.append(tr)
                elif observe:
                    self.observation_pub.publish(self.make_event(tr, stamp))
            elif getattr(self,'ring_diag',False) and tr.history:
                hx,hy=tr.history[-1][1],tr.history[-1][2]
                if math.hypot(hx-self.ego[0],hy-self.ego[1])<20.:
                    diag_tr.append([tr.id,round(hx,1),round(hy,1),tr.hits,-tr.misses,0,0,round(max(tr.size_x,tr.size_y),1)])
        if getattr(self,'ring_diag',False):
            # clusters: [x, y, size_x, size_y, height, points, gate]; tracks: [id, x, y, hits, structure (or -misses when
            # not given out this scan), on_road, has_footprint, size]
            self.get_logger().info('RB_LIDAR_DIAG '+json.dumps(dict(t=round(t,3),s=round(float(self.ego_s),2),
                ego=[round(float(self.ego[0]),2),round(float(self.ego[1]),2)],cl=self.ring_diag_cl,tr=diag_tr,
                det=getattr(self,'ring_diag_det',[]),
                fp=[f['id'].rsplit(':',1)[1] for f in footprints])))
        cleared=self.clearance.check(t,self.raw_clearance_points,self.raw_clearance_origin,ego,footprints)
        observed_ids={f'lidar:{self.track_namespace}:{tr.id}' for tr in self.tracks if tr.misses==0}
        cleared=[q for q in cleared if q['id'] not in observed_ids]
        if getattr(self,'mission_name',None)=='ROUNDABOUT' and self.ego_s is not None and 850.<=self.ego_s<=872.:
            for fp in footprints:
                if math.hypot(fp['center'][0]+106.75,fp['center'][1]-326.22)<5.:
                    self.get_logger().info('RB_GHOST_OBJ '+json.dumps(dict(t=round(t,3),s=round(float(self.ego_s),2),id=fp['id'],
                        kind=fp['kind'],h=round(fp['height'],2),c=[round(float(v),2) for v in fp['center']],
                        v=[round(float(fp['vx'] or 0.),2),round(float(fp['vy'] or 0.),2)],xy=[[round(a,2),round(b,2)] for a,b in fp['xy']])))
        self.footprint_pub.publish(String(data=json.dumps(dict(stamp=t,frame='map',objects=footprints,cleared=cleared,
            scan_valid=bool(self.highway_scan_valid and ego is not None and self.road is not None),
            sight=getattr(self,'sight',None),
            boundary_evidence=self.clearance.boundaries,tunnel_walls=getattr(self,'tunnel_walls',[])))))
        if self.marker_pub is not None:
            self.marker_pub.publish(self.make_markers(published, stamp))

    def output_gates(self, tr, ego_idx, s_shift):
        """Only the road/path test may admit a driving event.

        Adjacent and departing road users remain available for prediction and
        departure confirmation, never as a bypass around the geometry gate.
        """
        relevant = self.on_route_path(tr, ego_idx, s_shift)
        observe = (not tr.structure and (self.vehicle_observation(tr, ego_idx, s_shift)
                   or self.departure_observation(tr, ego_idx, s_shift))) if not relevant else False
        return relevant, observe

    def warn_off_route(self, t: float, dist: float) -> None:
        """코스 이탈 경고. 10초에 한 번만 (로그 폭주 방지)."""
        if self.warned_route_at is None or t - self.warned_route_at > 10.0:
            self.warned_route_at = t
            self.get_logger().warn(
                f'자차가 전역경로에서 {dist:.0f} m 벗어났습니다 → 경로 필터를 끕니다 '
                '(모든 덩어리를 발행하고 Event.s 는 -1)')

    def frenet_box(self, tr: Track, ego_idx, dx: float = 0., dy: float = 0.):
        """물체 바운딩박스 네 모서리를 경로에 투영해 (s 범위, d 범위) 를 준다.

        차량 좌표의 size_x/size_y 로는 모양을 판단할 수 없다. 곡선에서는 도로가 휘어서
        레일 조각이 차량 좌표로는 '앞뒤로 짧고 옆으로 긴' 모양이 되는데, 멀리서 앞면만
        보이는 박스도 똑같이 생겼다. **도로 방향 기준으로 재야** 둘이 갈린다
        (박스는 d폭 1.5 m, 레일 조각은 3~6 m — 실측 2026-09-20).
        """
        if self.ego is None:
            return None
        ex, ey, eyaw = self.ego
        c, sn = math.cos(eyaw), math.sin(eyaw)
        ss, dd = [], []
        x, y, size_x, size_y = track_box(tr)
        for ox in (-size_x / 2., size_x / 2.):
            for oy in (-size_y / 2., size_y / 2.):
                bx, by = x + ox + dx, y + oy + dy      # base_link
                px, py = ex + c * bx - sn * by, ey + sn * bx + c * by
                proj = self.project_to_route(px, py, ego_idx)
                if proj is None:
                    return None
                ss.append(proj[0])
                dd.append(proj[1])
        return min(ss), max(ss), min(dd), max(dd)

    def tunnel_owned(self) -> bool:
        """Fresh TUNNEL mission ownership (same rule as the zero-intensity filter)."""
        return fresh_mission(self, 'TUNNEL')

    def observation_on_road(self, mx: float, my: float) -> bool:
        """Keep tunnel shoulders in perception; route conflict is decided downstream.

        Measured walls bound the observed area. Where a wall is not observed,
        retain the existing point-cloud road band without extrapolating a wall.
        """
        if self.road is None:
            return False
        if not self.tunnel_owned():
            return self.road.outside_one(mx, my) <= self.road_margin
        inside = self.road.outside_one(mx, my) <= self.road_point_margin
        if self.ego is None:
            return inside
        ex, ey, yaw = self.ego
        c, sn = math.cos(yaw), math.sin(yaw)
        x, y = c*(mx-ex)+sn*(my-ey), -sn*(mx-ex)+c*(my-ey)
        measured = 0
        for side in ('R', 'L'):
            face = sorted((c*(px-ex)+sn*(py-ey), -sn*(px-ex)+c*(py-ey))
                          for px, py, label in getattr(self, 'tunnel_walls', []) if label == side)
            if len(face) < 2 or not face[0][0] <= x <= face[-1][0]:
                continue
            boundary = float(np.interp(x, [p[0] for p in face], [p[1] for p in face]))
            if (side == 'R' and y <= boundary) or (side == 'L' and y >= boundary):
                return False
            measured += 1
        return measured == 2 or inside

    def box_on_road(self, tr: Track, margin=None) -> bool:
        """All four bounding-box corners lie within the kept-point band of the mapped road.

        The tunnel is ~2 m wider than the mapped pavement on each side; a barricade
        reaching into that shoulder is still a barricade (tools/tunnel_sweep.py). Wall
        returns are already removed by the zero-intensity filter.
        """
        if self.road is None or self.ego is None:
            return False
        ex, ey, eyaw = self.ego
        c, sn = math.cos(eyaw), math.sin(eyaw)
        x, y, size_x, size_y = track_box(tr)
        corners = np.array([(ex + c*(x+ox) - sn*(y+oy), ey + sn*(x+ox) + c*(y+oy))
                            for ox in (-size_x/2., size_x/2.) for oy in (-size_y/2., size_y/2.)])
        margin = self.road_point_margin if margin is None else margin
        return bool((self.road.outside(corners) <= margin).all())

    def on_route_path(self, tr: Track, ego_idx, s_shift) -> bool:
        """경로에 투영해 tr.s / tr.lat 을 채우고, 발행할 가치가 있는지 판단한다.

        기준은 **종류와 무관하게 하나**다: 지도상 도로 안에 있고, 내 주행 통로를 막고 있는가.
        보행자인지 차량인지 가리지 않는다 — 도로 안에 뛰어든 무언가면 그 자체로 장애물이다.
        (분류는 그 뒤 FSM 이 '멈출까 비켜갈까' 를 정할 때만 쓴다.)

        경로를 아직 못 받았으면 아무것도 거르지 않는다 (예전 동작 그대로).
        """
        tr.s, tr.lat = -1.0, None
        tr.gate_why = 'nopath'
        if ego_idx is None or not tr.history:
            return True
        _, mx, my = tr.history[-1]
        now = self.project_to_route(mx, my, ego_idx)
        if now is None:
            return True
        s_local, lat = now
        # 1) 지도상 도로 밖이면 끝. 움직여 보이든 말든 상관없다 —
        #    라이다 링이 가드레일을 훑으면 1~9 m/s 의 겉보기 속도가 그냥 나온다.
        if self.road is not None and not self.observation_on_road(mx, my):
            self.dropped_off_road += 1
            tr.gate_why = 'offroad'
            return False
        box = self.frenet_box(tr, ego_idx)
        if box is None:                                   # 자차 자세를 모르면 거르지 않는다
            tr.lat, tr.s = lat, (-1.0 if s_shift is None else s_local + s_shift)
            return True
        _, _, d_min, d_max = box
        # 2) 도로를 가로지르는 폭이 차 한 대보다 넓으면 물체가 아니라 벽·레일이다.
        #    단 터널에서 박스 네 모서리가 모두 지도상 도로 안이면 벽이 아니라 도로를 막은
        #    바리케이드다 (2026-09-26 d폭 2.6~3.1 m 바리케이드 두 개를 벽으로 버리고 충돌).
        #    The static-obstacle mission also admits large mapped bodies; ordinary
        #    roadside objects retain the original width rejection.
        static_owned = fresh_mission(self, 'STATIC_OBSTACLE')
        mapped_obstacle = self.tunnel_owned() or static_owned
        # Static-mission bodies must fit mapped pavement; only the tunnel uses
        # the wider kept-point band for its shoulders beyond mapped lane edges.
        margin = self.road_margin if static_owned else None
        if d_max - d_min > self.max_block_width and not (mapped_obstacle and self.box_on_road(tr, margin)):
            self.dropped_wide += 1
            tr.gate_why = 'wide'
            return False
        corridor_half = self.tunnel_corridor_half if self.tunnel_owned() else self.corridor_half
        # 3) 내 통로를 막고 있는가. 중심이 아니라 **가장 가까운 모서리** 로 본다 —
        #    차 한 대가 차선을 반쯤 물고 서 있으면 중심은 통로 밖이어도 막고 있는 것이다.
        closest = 0. if d_min < 0. < d_max else min(abs(d_min), abs(d_max))
        if closest > corridor_half and (tr.vx or tr.vy):
            # 움직이는 물체는 predict_s 뒤 위치도 본다 (차선으로 걸어 들어오는 보행자).
            # 단 그 예상 위치도 **도로 안** 이어야 한다.
            fx = mx + tr.vx * self.path_predict_s
            fy = my + tr.vy * self.path_predict_s
            if self.road is None or self.observation_on_road(fx, fy):
                # Velocity is map-frame; frenet_box offsets are base_link.
                c, sn = math.cos(self.ego[2]), math.sin(self.ego[2])
                dx, dy = fx-mx, fy-my
                ahead = self.frenet_box(tr, ego_idx, c*dx+sn*dy, -sn*dx+c*dy)
                if ahead is not None:
                    closest = min(closest, 0. if ahead[2] < 0. < ahead[3] else min(abs(ahead[2]), abs(ahead[3])))
        if closest > corridor_half:
            self.dropped_off_route += 1
            tr.gate_why = 'offroute'
            return False
        tr.gate_why = 'ok'
        tr.lat = lat
        tr.s = -1.0 if s_shift is None else s_local + s_shift
        return True

    def vehicle_observation(self, tr: Track, ego_idx, s_shift) -> bool:
        """Observe confirmed vehicles on adjacent roads before they reach our corridor.

        Static roadside objects still use the original road/path gates. These
        observations provide prediction input, not an instruction to yield.
        """
        if tr.kind not in ('VEHICLE', 'DYNAMIC_UNKNOWN') or ego_idx is None or s_shift is None or not tr.history:
            return False
        _, mx, my = tr.history[-1]
        if self.road is None or self.road.outside_one(mx, my) > self.road_margin:
            return False
        projected = self.project_to_route(mx, my, ego_idx)
        if projected is None:
            return False
        tr.s, tr.lat = projected[0] + s_shift, projected[1]
        return True

    def departure_observation(self, tr: Track, ego_idx, s_shift) -> bool:
        """Keep observing an already relevant road user leaving the corridor.

        This does not admit new off-road objects or extrapolate missing detections.
        The planner decides whether the latest observed position still conflicts.
        """
        if not tr.published or tr.kind not in ('PEDESTRIAN', 'VEHICLE', 'DYNAMIC_UNKNOWN'):
            return False
        if ego_idx is None or s_shift is None or not tr.history:
            return False
        _, mx, my = tr.history[-1]
        projected = self.project_to_route(mx, my, ego_idx)
        if projected is None or abs(projected[1]) > self.roi_y_abs:
            return False
        tr.s, tr.lat = projected[0] + s_shift, projected[1]
        return True

    def match(self, dets: list, t: float) -> set:
        """트랙 예측 위치와 관측을 가까운 쌍부터 그리디로 짝짓는다.

        한 프레임의 객체 수는 많아야 수십 개라 헝가리안까지 갈 필요가 없다.
        """
        matched = set()
        if not self.tracks:
            return matched
        if not dets:
            for tr in self.tracks:
                tr.miss()
            return matched

        pred = np.array([tr.predict(t) for tr in self.tracks])           # (T, 2)
        obs = np.array([[d['mx'], d['my']] for d in dets])               # (D, 2)
        cost = np.linalg.norm(pred[:, None, :] - obs[None, :, :], axis=2)

        if self.ego_s is not None and self.fragment_merge_begin<=self.ego_s<=self.fragment_merge_end:
            for ti,tr in enumerate(self.tracks):
                elapsed=t-tr.last_t
                if not 0<elapsed<=.35 or np.hypot(tr.vx,tr.vy)<.8 or len(tr.footprint_map)<3:continue
                previous=np.asarray(tr.footprint_map)+np.array([tr.vx,tr.vy])*elapsed
                for di,det in enumerate(dets):
                    if not self.match_gate<cost[ti,di]<=6.5:continue
                    current=np.asarray(det.get('footprint_map',[]))
                    if len(current)>=3 and overlaps(previous,current):cost[ti,di]=self.match_gate-1e-6
        used_tracks = set()
        while True:
            ti, di = np.unravel_index(np.argmin(cost), cost.shape)
            if not np.isfinite(cost[ti, di]) or cost[ti, di] > self.match_gate:
                break
            self.tracks[ti].update(t, dets[di], self.hits_cap)
            used_tracks.add(ti)
            matched.add(di)
            cost[ti, :] = np.inf     # 한 트랙은 한 관측만
            cost[:, di] = np.inf
            if len(used_tracks) == len(self.tracks) or len(matched) == len(dets):
                break

        for i, tr in enumerate(self.tracks):
            if i not in used_tracks:
                tr.miss()
        return matched

    # A person (2026-10-04, pedestrian mission, 75 clusters of the competition's pedestrian against 2691 others and
    # 150 cars / boxes, 2 runs): top 1.51-1.86 m, footprint 0.47-0.90 x 0.16-0.54 m, points spread 0.3-1.5 m up
    # (sigma 0.34-0.46 m), returns at intensity 17-20 (poles / signs 0 or 44-45, cars / boxes 52-59). Shape alone
    # found 96 % with 4 false of 2691, shape and intensity 96 % with none.
    PED_TOP = (1.3, 2.0)        # m
    PED_LONG_MAX = 1.2          # m
    PED_SHORT_MAX = .8          # m
    PED_SPREAD_MIN = .25        # m, height sigma
    PED_MID_SHARE = .5          # points between 0.3 and 1.5 m up
    PED_INTENSITY = (12., 25.)  # cluster median
    PED_WALK_MPS = .5           # moved this fast over the track's history
    ANIMAL_RUN_MPS = 2.         # a tall mover not person-shaped (deer 3-4.6 m/s, 2026-10-10)

    def pedestrian_features(self, cluster, footprint, ci):
        z = cluster[:, 2]
        if len(z) < 3:
            return None
        ln, wd = oriented_dimensions(footprint) if len(footprint) >= 2 else (0., 0.)
        return dict(top=float(z.max()), long=float(ln), short=float(wd), spread=float(z.std()),
                    mid=float(((z >= .3) & (z <= 1.5)).mean()),
                    imed=float(np.median(ci)) if ci is not None and len(ci) else None)

    def pedestrian_vote(self, tr: Track) -> None:
        """Shape first (required); intensity or walking confirm it at once, else two of the last three scans do.
        Confirmed, the track stays a person (seen in part or joined with something for a scan it still is)."""
        f = tr.ped if tr.misses == 0 else None
        # A person is never 2.4 m long (twice the measured 1.2 m): a parked car first seen in part (0.8 x 1.7 m) was
        # confirmed a person and stayed one at 4.2 x 1.8 m, YIELD waited for it for good (2026-10-10 tunnel run).
        # Two of the last three scans, so one scan joined with something does not undo a person.
        if f is not None:
            tr.ped_too_long.append(f['long'] > 2.*self.PED_LONG_MAX)
        if tr.ped_confirmed and sum(tr.ped_too_long) >= 2:
            tr.ped_confirmed = False
            tr.ped_votes.clear()
            if tr.kind in ('PEDESTRIAN', 'DYNAMIC_UNKNOWN'):
                tr.kind = 'OBSTACLE'
        shape = (f is not None and self.PED_TOP[0] <= f['top'] <= self.PED_TOP[1] and f['long'] <= self.PED_LONG_MAX
                 and f['short'] <= self.PED_SHORT_MAX and f['spread'] >= self.PED_SPREAD_MIN
                 and f['mid'] >= self.PED_MID_SHARE)
        tr.ped_votes.append(shape)
        if tr.ped_confirmed or not shape:
            return
        refl = f['imed'] is not None and self.PED_INTENSITY[0] <= f['imed'] <= self.PED_INTENSITY[1]
        h = list(tr.history)
        walking = (len(h) >= 2 and h[-1][0]-h[0][0] >= .4
                   and math.hypot(h[-1][1]-h[0][1], h[-1][2]-h[0][2])/(h[-1][0]-h[0][0]) >= self.PED_WALK_MPS)
        # Shape with intensity or walking only (2026-10-04 measurement: shape alone 4 false of 2691, with intensity
        # none). Two shape votes without either confirmed poles off the road and a parked car's corner piece, which
        # YIELD then waited for in the middle of the road (2026-10-10 tunnel runs, intensity 0 / car 52-59).
        if refl or walking:
            tr.ped_confirmed = True

    def classify(self, tr: Track) -> str:
        """속도로 정지/이동을 먼저 가르고, 이동물만 크기로 사람/차를 나눈다.

        `tr.moving` 을 갱신하므로 **한 프레임에 한 번만** 불러야 한다.
        """
        speed = float(np.hypot(tr.vx, tr.vy))
        if tr.moving:
            tr.moving = speed >= self.moving_exit
        else:
            tr.moving = speed >= self.moving_enter and tr.straightness >= self.min_straightness
        # Motion state is not semantic identity. A confirmed person/car remains
        # a road user while stopped or during a noisy velocity/partial-size sample.
        # Identity expires with the track (max_misses), never from speed alone.
        # A person by shape (pedestrian_vote), never by size and speed alone: a barricade seen in part read as a
        # moving person-sized box and stayed a pedestrian for good (2026-10-04 tunnel runs)
        self.pedestrian_vote(tr)
        if tr.ped_confirmed:
            return 'PEDESTRIAN'
        if tr.structure:
            return 'OBSTACLE'
        # Tunnel has parked vehicles and moving people/animals: only two categories.
        # PEDESTRIAN includes children and animals; confirmed movers keep identity at rest.
        tunnel = self.tunnel_owned()
        person_sized = sum(tr.ped_too_long) < 2
        if tunnel and tr.kind in ('PEDESTRIAN', 'DYNAMIC_UNKNOWN') and person_sized:
            return 'PEDESTRIAN'
        if tunnel and tr.kind == 'VEHICLE':
            tr.kind = 'OBSTACLE'
        if tr.kind == 'VEHICLE':
            return tr.kind
        fallback = 'DYNAMIC_UNKNOWN' if tr.kind == 'DYNAMIC_UNKNOWN' else 'OBSTACLE'
        if not tr.moving:
            return fallback
        if tr.x < self.height_gate_m and tr.size_z < self.min_road_user_height:
            return 'OBSTACLE'      # 너무 낮다 — 라바콘·연석이지 도로 이용자가 아니다
        longest, shortest = max(tr.size_x, tr.size_y), min(tr.size_x, tr.size_y)
        roundabout=self.ego_s is not None and self.fragment_merge_begin<=self.ego_s<=self.fragment_merge_end
        if roundabout and len(tr.footprint)>=3:
            longest,shortest=oriented_dimensions(tr.footprint)
        if shortest < self.min_road_user_width:
            return 'OBSTACLE'      # 종잇장처럼 얇다 — 벽·가드레일 조각
        max_width=self.max_vehicle_width if roundabout else self.max_road_user_width
        if shortest > max_width:
            return 'OBSTACLE'      # 도로 차량은 법규상 2.5 m 를 못 넘는다 (곡선 가드레일의
                                   # 바운딩박스는 휘어서 폭이 3~5 m 까지 부푼다)
        large_vehicle=roundabout and 1.2<=shortest<=max_width and longest<=self.max_vehicle_length
        if not large_vehicle and longest > self.road_user_aspect_max * max(shortest, 1e-3):
            return 'OBSTACLE'      # 지나치게 길쭉하다 (차는 4.5×1.9 = 2.4 라 안 걸린다)
        if tr.consistent_motion():
            # person-shaped or person-tall this scan: a 3 m barricade gliding in the scan (ego motion, 0.8 m tall) read
            # as a person and left the BEV avoidance; a running deer (1.9 m long, top 1.2-2.0 m) is no person by shape
            # and, an OBSTACLE, was chased by the slalom until it blocked 4 m ahead (2026-10-10). Tall from the
            # measured person top (PED_TOP), above the 0.8 m barricades.
            # tall alone only when it runs (map frame, >= 0.4 s): a parked car (1.5 m) and a standing deer glided in
            # the scan as we came and read as running people (2026-10-10); the deer ran at 3-4.6 m/s
            f = tr.ped if tr.misses == 0 else None
            h = list(tr.history)
            running = (len(h) >= 2 and h[-1][0]-h[0][0] >= .4 and math.hypot(h[-1][1]-h[0][1], h[-1][2]-h[0][2])
                       / (h[-1][0]-h[0][0]) >= self.ANIMAL_RUN_MPS)
            tall = f is not None and f['top'] >= self.PED_TOP[0] and running
            shaped = person_sized and (tall or (bool(tr.ped_votes) and tr.ped_votes[-1]))
            fallback = 'PEDESTRIAN' if tunnel and shaped else 'DYNAMIC_UNKNOWN' if not tunnel else fallback
        if longest <= self.ped_max_size:
            # 사람 크기인데 사람 속도가 아니면 사람이 아니다
            return fallback        # person-sized: a person only by shape (pedestrian_vote)
        if longest >= self.vehicle_min_size:
            return fallback if tunnel else 'VEHICLE'
        # A partial view cannot decide person/car; proven motion still isn't static.
        return fallback

    def confidence(self, tr: Track) -> float:
        """연속 확인 횟수와 포인트 수를 섞은 0~1 점수.

        멀거나 가려진 물체는 점이 적어 자연히 낮게 나온다.
        """
        hit_term = min(tr.hits / float(self.hits_cap), 1.0)
        pts_term = min(tr.n_points / 40.0, 1.0)
        return round(float(np.clip(0.5 * hit_term + 0.5 * pts_term, 0.0, 1.0)), 3)

    def make_event(self, tr: Track, stamp) -> Event:
        ev = Event()
        ev.header.stamp = stamp                 # 센서 시각 그대로 (관측 시점 보존)
        ev.header.frame_id = self.base_frame    # x, y 의 기준 프레임
        ev.type = tr.kind
        ev.s = tr.s                             # 경로 진행거리 (경로 없으면 -1)
        ev.x, ev.y, ev.size_x, ev.size_y = track_box(tr)
        ev.vx, ev.vy = tr.vx, tr.vy             # map 프레임 절대속도
        ev.value = f'lidar:{self.track_namespace}:{tr.id}'  # stable identity, unique across node restarts
        ev.confidence = self.confidence(tr)
        return ev

    def make_markers(self, tracks: list, stamp) -> MarkerArray:
        """RViz 확인용 박스. 판단에는 쓰이지 않는다."""
        colors = {'OBSTACLE': (1.0, 0.8, 0.0), 'PEDESTRIAN': (1.0, 0.2, 0.2),
                  'VEHICLE': (0.2, 0.6, 1.0), 'DYNAMIC_UNKNOWN': (0.8, 0.2, 1.0)}
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for tr in tracks:
            kind = tr.kind
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.base_frame
            m.ns = 'obstacle'
            m.id = tr.id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y = tr.x, tr.y
            m.pose.position.z = tr.size_z * 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = max(tr.size_x, 0.2)
            m.scale.y = max(tr.size_y, 0.2)
            m.scale.z = max(tr.size_z, 0.2)
            if ((fresh_mission(self, 'STATIC_OBSTACLE') or
                 (self.ego_s is not None and self.fragment_merge_begin<=self.ego_s<=self.fragment_merge_end))
                    and len(tr.footprint)>=2):
                m.type=Marker.LINE_STRIP
                m.pose.position.x=m.pose.position.y=0.
                m.pose.position.z=.25
                m.scale.x=.08;m.scale.y=m.scale.z=0.
                m.points=[Point(x=float(x),y=float(y),z=0.) for x,y in tr.footprint+[tr.footprint[0]]]
            r, g, b = colors[kind]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.5
            m.lifetime = Duration(seconds=0.3).to_msg()
            arr.markers.append(m)

            if self.ego_s is not None and self.fragment_merge_begin<=self.ego_s<=self.fragment_merge_end:
                continue  # Roundabout GUI uses geometry, not per-object text.
            label = Marker()
            label.header = m.header
            label.ns = 'obstacle_label'
            label.id = tr.id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y = tr.x, tr.y
            label.pose.position.z = tr.size_z + 0.4
            label.pose.orientation.w = 1.0
            label.scale.z = 0.5
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'{kind} {np.hypot(tr.vx, tr.vy):.1f}m/s'
            label.lifetime = Duration(seconds=0.3).to_msg()
            arr.markers.append(label)
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
