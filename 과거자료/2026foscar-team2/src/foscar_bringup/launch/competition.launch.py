"""대회용 통합 런치 — 한 터미널에서 전부 뜬다.

    bash run_competition.sh                       # 주행 + 라이다 인지 + FSM(회피·터널·속도구간) + 카메라 7번 신호 정지
    bash run_competition.sh rviz:=true            # + BEV RViz
    bash run_competition.sh drive:=false          # 차는 안 움직이고 인지·판단만 (직접 몰 때)
    bash run_competition.sh camera:=false         # 카메라 없이 (신호에서 서지 않는다)

구성
    foscar_control    sensor_bridge · localization · path_follower   ← require_target=true: FSM 이 조용하면 정지
    foscar_lidar      lidar_bridge · obstacle_node                   → /events    (obstacle.launch.py include)
    foscar_planning   behavior_node                                   → /behavior/target  ★ 유일한 발행자
                     bev_node (rviz:=true 일 때)
    foscar_camera     camera:=true (기본). yolo_signal.launch.py include (카메라 7번 YOLO 신호 인식 → behavior_node 신호 정지)
    rosbag           bag:=true (기본) 이면 ~/foscar_logs/bags/ 에 판단·제어 토픽 기록 (포인트클라우드 제외)

`states` 에 없는 상태는 존재하지 않는다. 신호 정지는 상태가 아니라 behavior_node 안에서 처리한다:
camera:=true 면 `traffic.enabled` 가 켜지고, `/perception/traffic_signal`(카메라 7번 YOLO)과
config/yolo_stoplines.json 의 정지 위치로 정지 목표를 합친다 (README §9).

기본 drive:=true 라 뜨는 순간 차가 출발한다 (시뮬 준비: 시나리오 로드 → P → F4 Load → START).
"""

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def code_version():
    """branch@commit(+dirty) of the source tree behind the (symlinked) install, for the records table."""
    import subprocess
    src = Path(__file__).resolve().parent
    try:
        git = lambda *a: subprocess.run(['git', '-C', str(src), *a], capture_output=True, text=True, timeout=2).stdout.strip()
        branch, commit = git('rev-parse', '--abbrev-ref', 'HEAD'), git('rev-parse', '--short', 'HEAD')
        return f"{branch}@{commit}{'+dirty' if git('status', '--porcelain', '--untracked-files=no') else ''}" if commit else ''
    except (OSError, subprocess.SubprocessError):
        return ''


def generate_launch_description():
    bringup = Path(get_package_share_directory('foscar_bringup'))
    lidar = Path(get_package_share_directory('foscar_lidar'))
    planning = Path(get_package_share_directory('foscar_planning'))
    try:
        camera = Path(get_package_share_directory('foscar_camera'))
    except PackageNotFoundError:
        camera = None

    def cfg(name):
        return LaunchConfiguration(name)

    arguments = [
        # --- 주행 ---
        DeclareLaunchArgument('drive', default_value='true', description='제어 송신. 직접 몰 때는 false'),
        DeclareLaunchArgument('route', default_value='auto', choices=['auto', 'spawn', 'global']),
        DeclareLaunchArgument('config', default_value=str(bringup / 'config/low_speed.yaml'),
                              description='주행 파라미터 (sim_ip 포함)'),
        # tracker 의 직선 상한. 실제 상한은 behavior.yaml 의 cruise.* 가 정한다 — 여기는 안전망.
        DeclareLaunchArgument('speed_kmh', default_value='60.0', description='일반 구간 상한; 고속도로는 고정 상한 해제'),
        # --- 판단 ---
        DeclareLaunchArgument('roundabout_control', default_value='true',
                              description='Experimental prediction entry veto, calibrated at global s880'),
        DeclareLaunchArgument('roundabout_local_entry', default_value='true',
                              description='Rolling gap admission, observed empty corridor and local following'),
        DeclareLaunchArgument('pedestrian_yield', default_value='true',
                              description='보행자 양보. 회전교차로 분리 시험에서만 false'),
        DeclareLaunchArgument('visual_details', default_value='false', description='상세 진단 마커 표시'),
        DeclareLaunchArgument('mission_config', default_value=str(planning / 'config/missions.yaml')),
        DeclareLaunchArgument('link_set', default_value=str(bringup / 'maps/link_set.json')),
        DeclareLaunchArgument('highway_config', default_value=str(planning / 'config/highway.yaml')),
        DeclareLaunchArgument('highway_checkpoints', default_value=str(bringup / 'routes/checkpoints.txt')),
        DeclareLaunchArgument('behavior_config', default_value=str(planning / 'config/behavior.yaml')),
        DeclareLaunchArgument('states', default_value='cruise,avoid,yield,tunnel',
                              description='behavior_node 가 돌릴 상태 목록. 신호 정지는 camera:=true로 활성화; stop_line 상태 추가 불필요'),
        # --- 라이다 ---
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('bag_points', default_value='false', description='also record raw lidar points (large; for perception diagnosis)'),
        DeclareLaunchArgument('lidar_port', default_value='9302'),
        DeclareLaunchArgument('lidar_sim', default_value='false', description='fake_cloud 로 대체 (시뮬 없이)'),
        # --- 카메라7 ONNX 인식 및 behavior_node 내부 신호 정지 ---
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('camera_port', default_value='9208'),
        # --- 관측 ---
        DeclareLaunchArgument('rviz', default_value='false', description='BEV RViz + bev_node'),
        DeclareLaunchArgument('view', default_value='false', description='라이다 matplotlib 뷰어'),
        DeclareLaunchArgument('bag', default_value='true'),
        DeclareLaunchArgument('bag_dir', default_value=str(Path.home() / 'foscar_logs' / 'bags')),
    ]

    config = cfg('config')
    path_file, spawn_file = str(bringup / 'routes/global_path.txt'), str(bringup / 'routes/spawn_path.txt')
    base = [
        Node(package='foscar_control', executable='sensor_bridge', output='screen', parameters=[config, {
            'drive': ParameterValue(cfg('drive'), value_type=bool)}]),
        Node(package='foscar_control', executable='localization', output='screen', parameters=[config, {
            'path_file': path_file, 'spawn_file': spawn_file}]),
        Node(package='foscar_control', executable='path_follower', output='screen', parameters=[config, {
            'path_file': path_file, 'spawn_file': spawn_file, 'route': cfg('route'),
            'speed_kmh': ParameterValue(cfg('speed_kmh'), value_type=float),
            'speed_policy_file': cfg('mission_config'),
            'continuous': True, 'require_target': True}]),
        Node(package='foscar_planning', executable='behavior_node', output='screen',
             parameters=[cfg('behavior_config'), {'prediction.control_config': config,
                'prediction.link_set':cfg('link_set'),
                'highway.config':cfg('highway_config'),'highway.path_file':path_file,
                'highway.checkpoint_file':cfg('highway_checkpoints'),
                'traffic.enabled': ParameterValue(cfg('camera'), value_type=bool),
                'traffic.config': str(camera/'config/yolo_stoplines.json') if camera else '',
                'traffic.path_file': path_file,
                'prediction.control_speed_kmh': ParameterValue(cfg('speed_kmh'), value_type=float), 'states': ParameterValue(cfg('states'), value_type=str),
                'mission.config': cfg('mission_config'),
                'visualization.details': ParameterValue(cfg('visual_details'), value_type=bool),
                'roundabout.enabled': ParameterValue(cfg('roundabout_control'), value_type=bool),
                'roundabout.local_entry': ParameterValue(cfg('roundabout_local_entry'), value_type=bool),
                'yield.pedestrian_enabled': ParameterValue(cfg('pedestrian_yield'), value_type=bool)}]),
    ]
    # 필수 노드가 죽으면 런치 전체 종료 → 브리지가 브레이크 걸고 나간다 (low_speed.launch.py 와 동일)
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason='required node exited'))])) for node in base]

    # include 에 넘긴 launch_arguments 는 바깥 스코프까지 덮어쓴다 (rviz:=false 가 우리 rviz 인자를 지웠음, 2026-09-19).
    # GroupAction(scoped=True) 로 가둬야 한다.
    lidar_launch = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(lidar / 'launch/obstacle.launch.py')),
        launch_arguments={'sim': cfg('lidar_sim'), 'lidar_port': cfg('lidar_port'),
                          'view': cfg('view'), 'rviz': 'false','link_set':cfg('link_set'),
                          'highway_config':cfg('highway_config'),'highway_mission_config':cfg('mission_config')}.items())],
        scoped=True, forwarding=True, condition=IfCondition(cfg('lidar')))

    extras = []
    if camera is not None:
        extras.append(TimerAction(period=5.0, actions=[GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(camera / 'launch/yolo_signal.launch.py')),
            launch_arguments={'port': cfg('camera_port')}.items())],
            scoped=True, forwarding=True, condition=IfCondition(cfg('camera')))]))

    # 기록은 rviz 와 무관하게 항상 남긴다 (~/foscar_logs/records.json)
    hud = Node(package='foscar_planning', executable='race_hud_node', output='log',
               parameters=[{'mission_config': cfg('mission_config'), 'code_version': code_version()}])
    bev = Node(package='foscar_planning', executable='bev_node', output='log',
               parameters=[{'station_ticks': ParameterValue(cfg('visual_details'), value_type=bool)}],
               condition=IfCondition(cfg('rviz')))
    rviz = Node(package='rviz2', executable='rviz2', output='log', arguments=['-d', str(planning / 'rviz/bev.rviz')],
                condition=IfCondition(cfg('rviz')))

    bag_name = 'run_' + datetime.now().strftime('%Y%m%d_%H%M%S')
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', [cfg('bag_dir'), '/' + bag_name],
             '/events', '/perception/road_users', '/perception/footprints', '/perception/clearance_queries', '/behavior/target', '/behavior/state', '/behavior/mission', '/behavior/markers', '/behavior/prediction', '/behavior/prediction_markers',
             '/localization/odometry', '/localization/s', '/localization/mode',
             '/behavior/highway', '/behavior/highway_markers', '/behavior/signal_stop', '/behavior/signal_camera', '/perception/traffic_signal',
             '/vehicle/status', '/control/request', '/control/state', '/control/telemetry', '/perception/obstacle_markers', '/map/route'],
        output='log', condition=IfCondition(cfg('bag')))

    points = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', [cfg('bag_dir'), '/' + bag_name + '_points'], '/sensors/lidar/points', '/localization/odometry'],
        output='log', condition=IfCondition(cfg('bag_points')))

    return LaunchDescription(arguments + handlers + base + [lidar_launch, hud, bev, rviz, bag, points] + extras)
