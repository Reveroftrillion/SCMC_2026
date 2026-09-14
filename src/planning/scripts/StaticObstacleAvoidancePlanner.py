#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from lidar_object_detection.msg import ObjectInfo
from std_msgs.msg import Bool, Int16

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
        # 센서 오프셋 (라이다가 GPS 앞 3.9m)
        self.lidar_offset_x = 1.0  # 라이다가 차량(GPS) 기준 전방 3.9m
        self.lidar_offset_y = 0.0   # 좌우 오프셋 없음

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

        # 회피 경로 저장
        self.local_path = Path()
        self.local_path.header.frame_id = 'map'

        # Publisher
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.local_path_done_pub = rospy.Publisher('/local_path_done', Bool, queue_size=1)

        # Subscriber
        self.global_path_sub = rospy.Subscriber('/global_path', Path, self.global_path_callback)
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_callback)
        self.curr_idx_sub = rospy.Subscriber('/curr_idx', Int16, self.curr_idx_callback)
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
            # rospy.logwarn("[StaticObstacleAvoidancePlanner] Received empty global path")
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

    def current_pose_callback(self, msg: PoseStamped):
        """
        /current_pose 콜백 함수
        GPS로부터 변환된 차량의 현재 위치 정보 수신
        """
        self.curr_pose = msg.pose
        if not self.is_pose_set:
            self.is_pose_set = True

    def curr_idx_callback(self, msg: Int16):
        """
        /curr_idx 콜백 함수
        현재 global path 상의 인덱스 정보 수신
        """
        self.curr_idx = msg.data

    # def in_static_obstacle_zone(self):
    #     """
    #     현재 위치가 정적 장애물 구간인지 확인
    #     controller.cpp의 in_static_obstacle_zone()과 동일한 로직
    #     """
    #     return (self.curr_idx >= 1930 and self.curr_idx <= 2200) or \
    #            (self.curr_idx >= 5420 and self.curr_idx <= 6000)
    def in_static_obstacle_zone(self):
        return False

    def obstacle_callback(self, msg: ObjectInfo):
        """
        /obstacle_info_static 콜백 함수
        장애물 정보를 받아 global 좌표계로 변환하고 시각화
        """
        # 정적 장애물 구간인지 먼저 확인
        if not self.in_static_obstacle_zone():
            rospy.loginfo_throttle(5.0, "[StaticObstacleAvoidancePlanner] 정적 장애물 구간이 아님 (idx: %d)", self.curr_idx)
            return

        # 필수 정보 확인
        if not self.is_pose_set:
            # rospy.logwarn_throttle(1.0, "[StaticObstacleAvoidancePlanner] Waiting for current_pose...")
            return

        if not self.is_global_path_set:
            # rospy.logwarn_throttle(1.0, "[StaticObstacleAvoidancePlanner] Waiting for global_path...")
            return

        # 회피 중이면 무조건 완료할 때까지 새 장애물 무시
        if self.avoidance_active:
            if self.check_target_reached():
                self.send_path_done_signal()
                self.avoidance_active = False
            return  # 회피 완료 전까지는 새 경로 생성 안함

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
        라이다-GPS 간 3.9m 오프셋 보정 포함
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
            # 라이다 좌표계 장애물 위치
            lidar_rel_x = msg.centerX[i]
            lidar_rel_y = msg.centerY[i]

            # Step 1: 라이다 좌표계 -> 차량(GPS) 좌표계 변환
            # 라이다가 GPS 앞 3.9m에 있으므로, GPS 기준으로는 3.9m 더해야 함
            vehicle_rel_x = lidar_rel_x + self.lidar_offset_x
            vehicle_rel_y = lidar_rel_y + self.lidar_offset_y

            # Step 2: 차량(GPS) 좌표계 -> 전역 좌표계 변환
            global_x = self.curr_pose.position.x + cos_yaw * vehicle_rel_x - sin_yaw * vehicle_rel_y
            global_y = self.curr_pose.position.y + sin_yaw * vehicle_rel_x + cos_yaw * vehicle_rel_y

            self.obstacle_global_x.append(global_x)
            self.obstacle_global_y.append(global_y)

    def plan_avoidance_path(self):
        """
        Frenet 좌표계를 활용한 회피 경로 생성
        """
        if not self.is_global_path_set or self.csp is None:
            rospy.logwarn("[회피 경로 생성 실패] Global path 또는 CubicSpline2D가 초기화되지 않음")
            return

        # 라이다 위치 계산 (GPS 앞 3.9m)
        orientation_list = [
            self.curr_pose.orientation.x,
            self.curr_pose.orientation.y,
            self.curr_pose.orientation.z,
            self.curr_pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(orientation_list)

        lidar_x = self.curr_pose.position.x + self.lidar_offset_x * np.cos(yaw)
        lidar_y = self.curr_pose.position.y + self.lidar_offset_x * np.sin(yaw)

        # 가장 가까운 장애물 찾기 (라이다 기준)
        min_dist = float('inf')
        nearest_obs_x = None
        nearest_obs_y = None

        for i in range(self.obstacle_count):
            obs_x = self.obstacle_global_x[i]
            obs_y = self.obstacle_global_y[i]

            # 라이다 기준 거리 계산
            dist = np.hypot(obs_x - lidar_x, obs_y - lidar_y)

            if dist < min_dist:
                min_dist = dist
                nearest_obs_x = obs_x
                nearest_obs_y = obs_y

        if nearest_obs_x is None:
            return

        # 라이다 기준 감지 거리 (더 일찍 감지)
        if min_dist > 10.0:
            return

        # 장애물을 Frenet 좌표계로 변환
        s_obs, d_obs = catesian_to_frenet(nearest_obs_x, nearest_obs_y, self.csp)

        # d 값으로 왼쪽/오른쪽 판단
        # d > 0: 경로 왼쪽에 장애물 → 오른쪽으로 회피
        # d < 0: 경로 오른쪽에 장애물 → 왼쪽으로 회피
        # d ≈ 0: 경로 중앙에 장애물 → 오른쪽으로 크게 회피

        # 현재 차량 위치의 Frenet 좌표 (GPS 기준)
        curr_s, curr_d = catesian_to_frenet(
            self.curr_pose.position.x,
            self.curr_pose.position.y,
            self.csp
        )

        lateral_offset = 0.9  # 기본 회피 거리 (m)

        if abs(d_obs) < 0.3:  # 중앙
            target_d = -lateral_offset * 2.4  # 오른쪽으로 크게 (1.28m)
            rospy.loginfo("[회피 경로 생성] 장애물이 경로 중앙에 위치 - 오른쪽으로 크게 회피")
        elif d_obs > 0:  # 왼쪽
            target_d = -lateral_offset # 오른쪽으로 (0.85m)
            rospy.loginfo("[회피 경로 생성] 장애물이 경로 왼쪽에 위치 - 오른쪽으로 회피")
        else:  # 오른쪽
            target_d = lateral_offset  # 왼쪽으로 (0.85m)
            rospy.loginfo("[회피 경로 생성] 장애물이 경로 오른쪽에 위치 - 왼쪽으로 회피")

        # 타이트한 회피 경로 (장애물 간격 20m 고려)
        # 현재 위치에서 장애물까지 거리 체크
        dist_to_obs = s_obs - curr_s

        # Step 1: 장애물을 지나는 지점 (장애물 + 1m) - 빨리 회피
        avoidance_s = s_obs + 1.0

        # Step 2: 빠른 복귀 경로 (장애물 + 8m, d=0으로 복귀) - 8m → 5m로 단축
        return_s = s_obs + 6.0
        target_s = return_s

        # 회피 경로 생성 (2단계: 회피 + 복귀)
        path_length = target_s - curr_s
        if path_length < 2.0:
            rospy.logwarn(f"[회피 경로 생성 실패] 경로 길이가 너무 짧음: {path_length:.2f}m")
            return

        self.local_path.poses = []
        self.local_path.header.stamp = rospy.Time.now()

        # Phase 1: 현재 위치 → 회피 지점 (target_d로 이동)
        avoidance_length = avoidance_s - curr_s
        waypoints_phase1 = []
        if avoidance_length > 0:
            L1 = avoidance_length
            a0_1 = curr_d
            a1_1 = 0
            a2_1 = (3 * (target_d - curr_d)) / (L1 ** 2)
            a3_1 = (-2 * (target_d - curr_d)) / (L1 ** 3)

            num_points_1 = max(int(avoidance_length / 0.2), 5)
            s_points_1 = np.linspace(curr_s, avoidance_s, num_points_1)

            for s in s_points_1:
                ds = s - curr_s
                d = a0_1 + a1_1 * ds + a2_1 * (ds ** 2) + a3_1 * (ds ** 3)
                x, y, _ = self.frenet_to_cartesian(s, d)
                waypoints_phase1.append((x, y))

        # Phase 2: 회피 지점 → 복귀 지점 (d=0으로 복귀)
        return_length = return_s - avoidance_s
        waypoints_phase2 = []
        if return_length > 0:
            L2 = return_length
            a0_2 = target_d
            a1_2 = 0
            a2_2 = (3 * (0 - target_d)) / (L2 ** 2)  # d=0으로 복귀
            a3_2 = (-2 * (0 - target_d)) / (L2 ** 3)

            num_points_2 = max(int(return_length / 0.2), 5)
            s_points_2 = np.linspace(avoidance_s, return_s, num_points_2)

            for s in s_points_2:
                ds = s - avoidance_s
                d = a0_2 + a1_2 * ds + a2_2 * (ds ** 2) + a3_2 * (ds ** 3)
                x, y, _ = self.frenet_to_cartesian(s, d)
                waypoints_phase2.append((x, y))

        # 전체 waypoints 합치기
        all_waypoints = waypoints_phase1 + waypoints_phase2

        # 실제 경로 진행 방향으로 yaw 계산하여 poses 생성
        for i, (x, y) in enumerate(all_waypoints):
            pose_stamped = PoseStamped()
            pose_stamped.header = self.local_path.header
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0

            # 다음 waypoint를 향하는 방향으로 yaw 계산
            if i < len(all_waypoints) - 1:
                dx = all_waypoints[i + 1][0] - x
                dy = all_waypoints[i + 1][1] - y
                yaw = np.arctan2(dy, dx)
            else:
                # 마지막 점은 이전 점의 방향 유지
                if i > 0:
                    dx = x - all_waypoints[i - 1][0]
                    dy = y - all_waypoints[i - 1][1]
                    yaw = np.arctan2(dy, dx)
                else:
                    yaw = 0.0

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
        라이다 기준으로 목표점까지 거리 계산
        """
        if self.target_x is None or self.target_y is None:
            return False

        if not self.is_pose_set:
            return False

        # 라이다 위치 계산 (GPS 앞 3.9m)
        orientation_list = [
            self.curr_pose.orientation.x,
            self.curr_pose.orientation.y,
            self.curr_pose.orientation.z,
            self.curr_pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(orientation_list)

        lidar_x = self.curr_pose.position.x + self.lidar_offset_x * np.cos(yaw)
        lidar_y = self.curr_pose.position.y + self.lidar_offset_x * np.sin(yaw)

        # 라이다 기준 목표점까지 거리
        dist = np.hypot(lidar_x - self.target_x, lidar_y - self.target_y)

        # 목표 지점에 3m 이내 도달하면 완료 신호
        COMPLETION_DISTANCE = 9.83

        if dist < COMPLETION_DISTANCE:
            rospy.loginfo(f"[회피 완료] 목표까지 거리: {dist:.2f}m (임계값: {COMPLETION_DISTANCE}m)")
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
