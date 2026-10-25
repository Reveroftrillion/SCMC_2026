#!/usr/bin/env python3

import sys
import os

from polars import Int32
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from simul_msgs.msg import ObjectInfo
from std_msgs.msg import Bool, Int32

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from utils import CubicSpline2D, catesian_to_frenet


class StaticObstacleAvoidancePlanner:
    """
    정적 장애물 회피를 위한 로컬 경로 플래너
    /obstacle_info_static 토픽에서 장애물 정보를 받아
    global path 기준 왼쪽/오른쪽 판단 후 회피 경로 생성
    """

    def __init__(self):
        """
        정적 장애물 회피 경로 플래너
        """
        # Global path 관련
        self.global_path = None
        self.g_path = None  # numpy array (N, 4) [x, y, kappa, yaw]
        self.csp = None  # CubicSpline2D
        self.is_global_path_set = False

        # 현재 차량 상태
        self.curr_pose = Pose()
        self.curr_idx = 0
        self.is_pose_set = False  # 차량 위치 정보 수신 여부

        # 장애물 정보
        self.obstacle_detected = False
        self.obstacle_global_x = []
        self.obstacle_global_y = []
        self.obstacle_count = 0

        # 회피 경로 관련
        self.avoidance_active = False
        self.target_x = None
        self.target_y = None

        self.curr_idx = 0

        # 회피 경로 저장
        self.local_path = Path()
        self.local_path.header.frame_id = 'map'

        # Publisher
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.local_path_done_pub = rospy.Publisher('/local_path_done', Bool, queue_size=1)

        # Subscriber
        self.current_index_sub = rospy.Subscriber('/current_index', Int32, self.current_index_callback)
        self.global_path_sub = rospy.Subscriber('/global_path', Path, self.global_path_callback)
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_callback)
        self.obstacle_sub = rospy.Subscriber('/obstacle_info_static', ObjectInfo, self.obstacle_callback)

        rospy.loginfo("[StaticObstacleAvoidancePlanner] 초기화 완료")

    def global_path_callback(self, msg: Path):
        """
        /global_path 콜백 함수
        전역 경로를 받아서 CubicSpline2D 생성
        """
        self.global_path = msg

        # Path에서 x, y 추출
        if len(msg.poses) == 0:
            rospy.logwarn("[StaticObstacleAvoidancePlanner] Received empty global path")
            return

        # numpy array로 변환 (x, y, z는 curvature로 사용)
        path_x = [pose.pose.position.x for pose in msg.poses]
        path_y = [pose.pose.position.y for pose in msg.poses]
        path_z = [pose.pose.position.z for pose in msg.poses]  # curvature

        self.g_path = np.array([[x, y, z, 0.0] for x, y, z in zip(path_x, path_y, path_z)])

        # CubicSpline2D 생성
        try:
            self.csp = CubicSpline2D(path_x, path_y, interval=0.2)
            self.is_global_path_set = True
        except Exception as e:
            rospy.logerr(f"[StaticObstacleAvoidancePlanner] CubicSpline2D 생성 실패: {e}")
            self.is_global_path_set = False

    def current_index_callback(self, msg: Int32):
        """
        /current_index 콜백 함수
        현재 전역 경로 상의 인덱스 수신
        """
        self.curr_idx = msg.data

    def current_pose_callback(self, msg: PoseStamped):
        """
        /current_pose 콜백 함수
        GPS로부터 변환된 차량의 현재 위치 정보 수신
        """
        self.curr_pose = msg.pose
        if not self.is_pose_set:
            self.is_pose_set = True

    def obstacle_callback(self, msg: ObjectInfo):
        """
        /obstacle_info_static 콜백 함수
        장애물 정보를 받아 global 좌표계로 변환하고 시각화
        """
        # 필수 정보 확인
        if not self.is_pose_set:
            rospy.logwarn_throttle(1.0, "[StaticObstacleAvoidancePlanner] Waiting for current_pose...")
            return

        if not self.is_global_path_set:
            rospy.logwarn_throttle(1.0, "[StaticObstacleAvoidancePlanner] Waiting for global_path...")
            return

        # 회피 중이면 항상 완료 여부 확인 (장애물 유무와 무관)
        if self.avoidance_active:
            print('hmm......!!!!')
            
            if self.check_target_reached():
                self.send_path_done_signal()
                self.avoidance_active = False
            return

        if (self.curr_idx < 150 or self.curr_idx > 300) and (self.curr_idx < 610 or self.curr_idx > 780):
            return

        # 여기부터는 회피 중이 아닐 때만 실행
        self.obstacle_count = msg.objectCounts

        if self.obstacle_count == 0:
            self.obstacle_detected = False
            return

        # 차량 좌표계 -> 전역 좌표계 변환
        self.transform_obstacles_to_global(msg)

        # 장애물이 경로상에 있는지 확인하고 회피 경로 생성
        self.obstacle_detected = True
        self.plan_avoidance_path()

    def transform_obstacles_to_global(self, msg: ObjectInfo):
        """
        차량 좌표계의 장애물을 전역 좌표계로 변환
        GPS 기반의 정확한 차량 위치와 방향 정보 활용
        """
        # 차량의 현재 orientation에서 yaw 추출
        orientation_list = [
            self.curr_pose.orientation.x,
            self.curr_pose.orientation.y,
            self.curr_pose.orientation.z,
            self.curr_pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(orientation_list)

        # 회전 행렬
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        self.obstacle_global_x = []
        self.obstacle_global_y = []

        for i in range(self.obstacle_count):
            # 차량 좌표계 장애물 위치 (LiDAR 기준)
            rel_x = msg.centerX[i]
            rel_y = msg.centerY[i]

            # 전역 좌표계로 변환 (GPS 기반 차량 위치 + 회전 변환)
            global_x = self.curr_pose.position.x + cos_yaw * rel_x - sin_yaw * rel_y
            global_y = self.curr_pose.position.y + sin_yaw * rel_x + cos_yaw * rel_y

            self.obstacle_global_x.append(global_x)
            self.obstacle_global_y.append(global_y)

    def plan_avoidance_path(self):
        """
        Frenet 좌표계를 활용한 회피 경로 생성
        """
        if not self.is_global_path_set or self.csp is None:
            rospy.logwarn("[회피 경로 생성 실패] Global path 또는 CubicSpline2D가 초기화되지 않음")
            return

        # 가장 가까운 장애물 찾기 (GPS 기반 정확한 차량 위치 활용)
        min_dist = float('inf')
        nearest_obs_x = None
        nearest_obs_y = None

        for i in range(self.obstacle_count):
            obs_x = self.obstacle_global_x[i]
            obs_y = self.obstacle_global_y[i]

            # 차량의 GPS 기반 정확한 위치와 장애물 간의 거리 계산
            dist = np.hypot(obs_x - self.curr_pose.position.x,
                           obs_y - self.curr_pose.position.y)

            if dist < min_dist:
                min_dist = dist
                nearest_obs_x = obs_x
                nearest_obs_y = obs_y

        if nearest_obs_x is None:
            return

        # 너무 멀면 무시
        if min_dist > 9.0:
            return

        # 장애물을 Frenet 좌표계로 변환
        s_obs, d_obs = catesian_to_frenet(nearest_obs_x, nearest_obs_y, self.csp)

        # d 값으로 왼쪽/오른쪽 판단
        # d > 0: 경로 왼쪽에 장애물 → 오른쪽으로 회피
        # d < 0: 경로 오른쪽에 장애물 → 왼쪽으로 회피
        # d ≈ 0: 경로 중앙에 장애물 → 오른쪽으로 크게 회피

        lateral_offset = 1.8  # 기본 회피 거리 (m)

        if abs(d_obs) < 0.5:  # 중앙
            target_d = -lateral_offset * 2  # 오른쪽으로 크게
        elif d_obs > 0:  # 왼쪽
            target_d = -lateral_offset  # 오른쪽으로
        else:  # 오른쪽
            target_d = lateral_offset  # 왼쪽으로

        # 목표 지점 계산 (장애물보다 앞쪽)
        target_s = s_obs + 5  # 장애물보다 5m 앞

        # 경로의 총 길이를 넘어가지 않도록 target_s 보정
        if self.csp and hasattr(self.csp, 's') and target_s >= self.csp.s[-1]:
            target_s = self.csp.s[-1] - 0.1  # 마지막 지점 바로 앞으로 설정

        # 현재 차량 위치의 Frenet 좌표 (GPS 기반 정확한 위치)
        curr_s, curr_d = catesian_to_frenet(
            self.curr_pose.position.x,
            self.curr_pose.position.y,
            self.csp
        )

        # 회피 경로 생성 (3차 다항식 사용)
        path_length = target_s - curr_s
        if path_length < 2.0:
            rospy.logwarn("[회피 경로 생성 실패] 경로 길이가 너무 짧음")
            return

        # d(s) = a0 + a1*s + a2*s^2 + a3*s^3
        # 경계 조건: d(0) = curr_d, d'(0) = 0, d(L) = target_d, d'(L) = 0
        L = path_length
        a0 = curr_d
        a1 = 0
        a2 = (3 * (target_d - curr_d)) / (L ** 2)
        a3 = (-2 * (target_d - curr_d)) / (L ** 3)

        # 경로 포인트 생성
        num_points = max(int(path_length / 0.2), 10)
        s_points = np.linspace(curr_s, target_s, num_points)

        self.local_path.poses = []
        self.local_path.header.stamp = rospy.Time.now()

        for s in s_points:
            ds = s - curr_s
            d = a0 + a1 * ds + a2 * (ds ** 2) + a3 * (ds ** 3)

            # Frenet -> Cartesian 변환
            x, y, yaw = self.frenet_to_cartesian(s, d)

            # PoseStamped 생성
            pose_stamped = PoseStamped()
            pose_stamped.header = self.local_path.header
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0

            quat = quaternion_from_euler(0, 0, yaw)
            pose_stamped.pose.orientation.x = quat[0]
            pose_stamped.pose.orientation.y = quat[1]
            pose_stamped.pose.orientation.z = quat[2]
            pose_stamped.pose.orientation.w = quat[3]

            self.local_path.poses.append(pose_stamped)

        # 목표 지점 저장
        if len(self.local_path.poses) > 0:
            last_pose = self.local_path.poses[-1]
            self.target_x = last_pose.pose.position.x
            self.target_y = last_pose.pose.position.y

            self.avoidance_active = True

            # 경로 발행
            self.local_path_pub.publish(self.local_path)

    def frenet_to_cartesian(self, s: float, d: float):
        """
        Frenet 좌표를 Cartesian 좌표로 변환

        Args:
            s: 경로를 따라가는 거리
            d: 경로에 수직인 거리

        Returns:
            x, y, yaw
        """
        # s에 해당하는 인덱스 찾기
        idx = int(s / self.csp.interval)
        idx = min(max(idx, 0), len(self.csp.rx) - 1)

        # 경로상의 점
        rx = self.csp.rx[idx]
        ry = self.csp.ry[idx]
        ryaw = self.csp.ryaw[idx]

        # 수직 방향으로 d만큼 오프셋
        x = rx - d * np.sin(ryaw)
        y = ry + d * np.cos(ryaw)

        return x, y, ryaw

    def check_target_reached(self):
        """
        목표 지점 근처 도달 여부 확인
        목표 지점으로부터 3m 이내에 들어오면 True 반환
        (차량의 정확한 GPS 기반 위치 정보 활용)
        """
        if self.target_x is None or self.target_y is None:
            return False

        if not self.is_pose_set:
            return False

        # 차량의 실제 GPS 기반 위치와 목표점 간의 거리 계산
        dist = np.hypot(
            self.curr_pose.position.x - self.target_x,
            self.curr_pose.position.y - self.target_y
        )

        # 목표 지점에 1m 이내 도달하면 완료 신호
        COMPLETION_DISTANCE = 2.5

        if dist < COMPLETION_DISTANCE:
            rospy.loginfo(f"[목표 근접] 목표까지 거리: {dist:.2f}m (임계값: {COMPLETION_DISTANCE}m)")
            return True

        return False

    def send_path_done_signal(self):
        """
        Local path 완료 신호 발행
        Controller에게 안전하게 control_path로 복귀하도록 알림
        """
        done_msg = Bool()
        done_msg.data = True
        self.local_path_done_pub.publish(done_msg)

        # 목표 지점 초기화
        self.target_x = None
        self.target_y = None


if __name__ == '__main__':
    rospy.init_node('static_obstacle_avoidance_planner')
    planner = StaticObstacleAvoidancePlanner()
    rospy.spin()
