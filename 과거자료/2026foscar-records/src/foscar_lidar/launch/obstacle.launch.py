"""장애물 인지 노드 실행.

    ros2 launch foscar_lidar obstacle.launch.py                 # MORAI 실제 라이다
    ros2 launch foscar_lidar obstacle.launch.py sim:=true rviz:=true   # 시뮬 없이 확인

기본값은 `lidar_bridge`(MORAI VLP-16 UDP 수신) + `obstacle_node` + `lidar_view`(뷰어 창)
를 함께 띄운다. 창이 필요 없으면 `view:=false`.
`sim:=true` 면 브리지 대신 가짜 라이다·오도메트리를 띄워서, 시뮬레이터가 없어도
3프레임 필터 → 속도추정 → 분류까지 도는지 눈으로 볼 수 있다.

터미널로 값을 확인하려면 다른 터미널에서 (화면을 갱신하므로 따로 띄우는 게 좋다):

    ros2 run foscar_lidar lidar_monitor
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


LINK_SET = Path(get_package_share_directory('foscar_bringup')) / 'maps/link_set.json'


def generate_launch_description():
    share = Path(get_package_share_directory('foscar_lidar'))
    sim, rviz = LaunchConfiguration('sim'), LaunchConfiguration('rviz')
    lidar_x, lidar_y, lidar_z = (LaunchConfiguration('lidar_x'),
                                 LaunchConfiguration('lidar_y'),
                                 LaunchConfiguration('lidar_z'))

    def num(name):
        """런치 인자는 문자열로 오므로 float 로 못박아 노드에 넘긴다."""
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    arguments = [
        DeclareLaunchArgument('sim', default_value='false',
                              description='가짜 라이다 + 오도메트리로 시뮬 없이 확인'),
        DeclareLaunchArgument('bridge', default_value='true',
                              description='MORAI VLP-16 UDP 수신 노드도 함께 실행 (sim 이면 무시)'),
        DeclareLaunchArgument('lidar_port', default_value='9302',
                              description='MORAI 센서 Network Setting 의 Destination Port'),
        DeclareLaunchArgument('view', default_value='true',
                              description='라이다 뷰어 창 (위/옆에서 본 모습 + 수치)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        # 라이다 장착 위치 (차 기준). 현장점검에서 실측값이 나오면 여기만 고치면 된다.
        DeclareLaunchArgument('lidar_x', default_value='2.0'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        # z 는 **지면 기준** 높이. MORAI 패널의 1.75 는 차량 원점 기준이라 0.36 m 낮다.
        # 2.11 은 K-City 전방 노면 평면 실측값. 현장에서 바뀌면 이 인자로 넘기면 된다.
        DeclareLaunchArgument('lidar_z', default_value='2.11'),
        DeclareLaunchArgument('use_tf', default_value='true',
                              description='base_link→lidar TF 가 있으면 파라미터 대신 사용'),
        # 팀 3 브리지가 TF 를 발행하기 전까지의 임시 static TF. 기본값은 sim 과 동일.
        DeclareLaunchArgument('static_tf', default_value='true',
                              description='base_link→lidar static TF 를 직접 발행. '
                                          '팀 3 브리지가 이 TF 를 넣으면 false 로 끄세요'),
        # ROI / 지면 (전부 base_link 기준, 지면 = 0)
        DeclareLaunchArgument('highway_config', default_value=''),
        DeclareLaunchArgument('highway_mission_config', default_value=''),
        DeclareLaunchArgument('roi_x_max', default_value='40.0'),
        DeclareLaunchArgument('roi_y_abs', default_value='20.0'),
        DeclareLaunchArgument('roi_z_min', default_value='-0.3'),
        DeclareLaunchArgument('roi_z_max', default_value='2.5'),
        DeclareLaunchArgument('ground_z', default_value='0.15',
                              description='이 높이 이하는 지면으로 보고 버림'),
        DeclareLaunchArgument('ground_slope_tol', default_value='0.01',
                              description='지면 임계값을 1 m 당 이만큼 올림 (먼 거리 노면 튐 방지)'),
        # 클러스터링 / 분류 / 안정화
        DeclareLaunchArgument('dbscan_eps', default_value='0.5'),
        DeclareLaunchArgument('dbscan_min_samples', default_value='5'),
        DeclareLaunchArgument('max_object_size_m', default_value='5.0',
                              description='가장 긴 변이 이보다 길면 벽·레일 조각 (대회 최대 물체는 차량 4.6 m)'),
        DeclareLaunchArgument('min_object_size_m', default_value='0.05',
                              description='이보다 작은 덩어리는 버림. 크게 잡으면 델리네이터 봉(4~10cm)을 놓침'),
        DeclareLaunchArgument('min_obstacle_top_m', default_value='0.30',
                              description='꼭대기가 이보다 낮으면 노면으로 보고 버림'),
        DeclareLaunchArgument('small_size_m', default_value='0.5',
                              description='이보다 작은 덩어리는 아래 높이 조건을 추가로 만족해야 함'),
        DeclareLaunchArgument('small_min_top_m', default_value='0.5',
                              description='작은 덩어리의 최소 꼭대기 높이 (노면 허상 제거)'),
        DeclareLaunchArgument('chain_min_count', default_value='3',
                              description='일직선에 이 개수 이상 늘어서면 구조물로 판정'),
        DeclareLaunchArgument('chain_tol_m', default_value='0.5',
                              description='직선에서 벗어난 허용치'),
        DeclareLaunchArgument('chain_span_m', default_value='25.0',
                              description='한 줄로 묶어볼 최대 길이'),
        DeclareLaunchArgument('min_road_user_width_m', default_value='0.25',
                              description='이보다 얇으면 보행자·차량이 될 수 없음'),
        DeclareLaunchArgument('road_user_aspect_max', default_value='4.0',
                              description='가로세로 비가 이보다 크면 구조물로 봄'),
        DeclareLaunchArgument('moving_enter_mps', default_value='0.8',
                              description='이 속도 이상이어야 이동물로 판정 시작'),
        DeclareLaunchArgument('moving_exit_mps', default_value='0.4',
                              description='이 아래로 떨어져야 정지물로 복귀 (히스테리시스)'),
        DeclareLaunchArgument('min_straightness', default_value='0.6',
                              description='궤적 직진성. 낮추면 흔들리는 정지물도 이동물로 샘'),
        DeclareLaunchArgument('min_road_user_height_m', default_value='0.6',
                              description='이보다 낮으면 보행자·차량일 수 없음 (라바콘·연석)'),
        DeclareLaunchArgument('height_gate_m', default_value='20.0',
                              description='높이 판정을 믿을 수 있는 거리. 이보다 멀면 적용 안 함'),
        DeclareLaunchArgument('ped_max_size_m', default_value='1.2'),
        DeclareLaunchArgument('ped_max_speed_mps', default_value='3.0',
                              description='이보다 빠르면 보행자일 수 없음 (대회 보행자 2.8)'),
        DeclareLaunchArgument('vehicle_min_size_m', default_value='2.0',
                              description='이보다 작으면 차량일 수 없음'),
        DeclareLaunchArgument('confirm_frames', default_value='3'),
        DeclareLaunchArgument('max_misses', default_value='15'),
        DeclareLaunchArgument('markers', default_value='true'),
        DeclareLaunchArgument('use_route', default_value='true',
                              description='/map/route 로 경로 투영 (Event.s 채우고 경로 밖 제외)'),
        DeclareLaunchArgument('max_block_width_m', default_value='2.0',
                              description='경로 가로 방향 폭이 이보다 넓으면 벽·레일 (차량 폭 1.9 + 여유)'),
        DeclareLaunchArgument('corridor_half_m', default_value='1.5',
                              description='물체의 가장 가까운 모서리가 경로에서 이 안이어야 발행 (종류 무관)'),
        DeclareLaunchArgument('path_predict_s', default_value='2.0',
                              description='이동물은 이 시간 뒤 예상 위치로도 판정 (횡단 보행자)'),
        # --- 지도 기반 도로 마스크 (link_set.json 링크 합집합) ---
        DeclareLaunchArgument('use_road_mask', default_value='true',
                              description='지도상 도로 밖 물체를 분류와 무관하게 제외'),
        DeclareLaunchArgument('link_set', default_value=str(LINK_SET),
                              description='K-City link_set.json 경로 (없으면 마스크 끔)'),
        DeclareLaunchArgument('road_margin_m', default_value='0.0',
                              description='도로 경계에서 이 이상 밖이면 무엇이든 제외'),
        DeclareLaunchArgument('ego_speed', default_value='3.0',
                              description='sim 전용: 가짜 자차 속도 (0 이면 정차)'),
    ]

    obstacle = Node(
        package='foscar_lidar', executable='obstacle_node', output='screen',
        parameters=[{
            'lidar_x': num('lidar_x'), 'lidar_y': num('lidar_y'), 'lidar_z': num('lidar_z'),
            'use_tf': ParameterValue(LaunchConfiguration('use_tf'), value_type=bool),
            'roi_x_max': num('roi_x_max'), 'roi_y_abs': num('roi_y_abs'),
            'roi_z_min': num('roi_z_min'), 'roi_z_max': num('roi_z_max'),
            'ground_z': num('ground_z'),
            'ground_slope_tol': num('ground_slope_tol'),
            'dbscan_eps': num('dbscan_eps'),
            'dbscan_min_samples': ParameterValue(
                LaunchConfiguration('dbscan_min_samples'), value_type=int),
            'max_object_size_m': num('max_object_size_m'),
            'min_object_size_m': num('min_object_size_m'),
            'min_obstacle_top_m': num('min_obstacle_top_m'),
            'small_size_m': num('small_size_m'),
            'small_min_top_m': num('small_min_top_m'),
            'chain_min_count': ParameterValue(
                LaunchConfiguration('chain_min_count'), value_type=int),
            'chain_tol_m': num('chain_tol_m'),
            'chain_span_m': num('chain_span_m'),
            'min_road_user_width_m': num('min_road_user_width_m'),
            'road_user_aspect_max': num('road_user_aspect_max'),
            'moving_enter_mps': num('moving_enter_mps'),
            'moving_exit_mps': num('moving_exit_mps'),
            'min_straightness': num('min_straightness'),
            'min_road_user_height_m': num('min_road_user_height_m'),
            'height_gate_m': num('height_gate_m'),
            'ped_max_size_m': num('ped_max_size_m'),
            'ped_max_speed_mps': num('ped_max_speed_mps'),
            'vehicle_min_size_m': num('vehicle_min_size_m'),
            'confirm_frames': ParameterValue(
                LaunchConfiguration('confirm_frames'), value_type=int),
            'max_misses': ParameterValue(LaunchConfiguration('max_misses'), value_type=int),
            'publish_markers': ParameterValue(LaunchConfiguration('markers'), value_type=bool),
            'use_route': ParameterValue(LaunchConfiguration('use_route'), value_type=bool),
            'max_block_width_m': num('max_block_width_m'),
            'corridor_half_m': num('corridor_half_m'),
            'path_predict_s': num('path_predict_s'),
            'use_road_mask': ParameterValue(LaunchConfiguration('use_road_mask'), value_type=bool),
            'link_set': LaunchConfiguration('link_set'),
            'highway.config':LaunchConfiguration('highway_config'),
            'highway.mission_config':LaunchConfiguration('highway_mission_config'),
            'road_margin_m': num('road_margin_m'),
        }])

    lidar_bridge = Node(
        package='foscar_lidar', executable='lidar_bridge', output='screen',
        condition=IfCondition(AndSubstitution(NotSubstitution(sim), LaunchConfiguration('bridge'))),
        parameters=[{'port': ParameterValue(LaunchConfiguration('lidar_port'), value_type=int)}])

    fake = Node(
        package='foscar_lidar', executable='fake_cloud', output='screen',
        condition=IfCondition(sim),
        parameters=[{'lidar_x': num('lidar_x'), 'lidar_y': num('lidar_y'),
                     'lidar_z': num('lidar_z'), 'ego_speed_mps': num('ego_speed')}])

    # static_transform_publisher 인자는 문자열이어야 해서 LaunchConfiguration 을 그대로 넘긴다.
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', output='log',
        condition=IfCondition(LaunchConfiguration('static_tf')),
        arguments=['--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
                   '--frame-id', 'base_link', '--child-frame-id', 'lidar'])

    # 뷰어는 obstacle_node 와 같은 파라미터를 봐야 화면의 ROI·지면선이 실제 처리와 일치한다.
    view = Node(
        package='foscar_lidar', executable='lidar_view', output='log',
        condition=IfCondition(LaunchConfiguration('view')),
        parameters=[{
            'lidar_x': num('lidar_x'), 'lidar_y': num('lidar_y'), 'lidar_z': num('lidar_z'),
            'roi_x_max': num('roi_x_max'), 'roi_y_abs': num('roi_y_abs'),
            'roi_z_min': num('roi_z_min'), 'roi_z_max': num('roi_z_max'),
            'ground_z': num('ground_z'),
            'ground_slope_tol': num('ground_slope_tol')}])

    rviz_node = Node(
        package='rviz2', executable='rviz2', output='log',
        condition=IfCondition(rviz),
        arguments=['-d', str(share / 'rviz/obstacle.rviz')])

    return LaunchDescription(arguments + [static_tf, lidar_bridge, fake, obstacle, view, rviz_node])
