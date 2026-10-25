"""주행 + 라이다 인지를 **한 터미널에서** 같이 띄운다.

    ros2 launch ctrl_lidar ctrl_drive.launch.py                  # 자율주행 (상한 100 km/h, 경로 끝까지)
    ros2 launch ctrl_lidar ctrl_drive.launch.py speed_kmh:=30     # 상한 낮춰서
    ros2 launch ctrl_lidar ctrl_drive.launch.py drive:=false      # 직접 몰 때 (제어 송신 끔)

안에서 두 런치를 그대로 include 한다 — 다른 패키지 파일은 손대지 않는다.

    ctrl_bringup/ctrl_low_speed.launch.py   sensor_bridge + localization + path_follower
    ctrl_lidar/ctrl_obstacle.launch.py      lidar_bridge + obstacle_node + lidar_view
    ctrl_camera/ctrl_yolo_signal.launch.py  카메라 7번 브리지 + YOLO 신호 인식 (+ 뷰어)  cameras:=false 로 끔

기본값이 `drive:=true` 라 띄우는 순간 차가 출발한다. **직접 몰거나 차를 세워두고 싶으면
반드시 `drive:=false`** 를 붙여야 한다 — 베이스가 제어를 잡고 있으면 수동 조작과 싸우고,
제어 명령이 없는 동안에는 `ctrl_bridge.py` 가 기본값으로 풀 브레이크를 50 Hz 로 보낸다.

`drive:=false` 여도 sensor_bridge·localization 은 뜨므로 **/localization/odometry 는 나온다.**
즉 차를 안 움직여도 ego 보상은 켜진 채로 인지만 볼 수 있다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = Path(get_package_share_directory('ctrl_bringup'))
    lidar = Path(get_package_share_directory('ctrl_lidar'))
    camera = Path(get_package_share_directory('ctrl_camera'))

    def cfg(name):
        return LaunchConfiguration(name)

    arguments = [
        # --- 주행 (ctrl_bringup 쪽) ---
        DeclareLaunchArgument('drive', default_value='true',
                              description='차에 제어를 보낸다. 직접 몰 때는 drive:=false 로 끄세요'),
        # 순항 상한. 베이스가 받는 최대치(100)로 열어둠 — 실제 속도는 곡률 프로파일이 깎는다.
        # 규정 60 km/h, 초과 허용은 예외구간(s=1119~1878)만. 60 초과 주행은 미검증 (가이드 §6).
        DeclareLaunchArgument('speed_kmh', default_value='100.0'),
        DeclareLaunchArgument('continuous', default_value='true',
                              description='시험거리 무시하고 경로 끝까지'),
        DeclareLaunchArgument('distance_m', default_value='20.0'),
        DeclareLaunchArgument('route', default_value='auto', choices=['auto', 'spawn', 'global']),
        DeclareLaunchArgument('rviz', default_value='false', description='주행용 RViz'),
        DeclareLaunchArgument('config', default_value=str(bringup / 'config/low_speed.yaml'),
                              description='주행 파라미터 yaml (ctrl_bringup 기본값)'),
        # --- 라이다 (ctrl_lidar 쪽) ---
        DeclareLaunchArgument('view', default_value='true', description='라이다 뷰어 창'),
        # --- 카메라 (ctrl_camera 쪽) ---
        DeclareLaunchArgument('cameras', default_value='true',
                              description='카메라 7번 수신 + YOLO 신호 인식 + 뷰어 창. CPU 빠듯하면 false'),
        DeclareLaunchArgument('camera_port', default_value='9208',
                              description='카메라 7번 Destination Port'),
        DeclareLaunchArgument('camera_scale', default_value='0.5',
                              description='카메라 창 크기 배율 (전방 1280×720 이 크면 0.5)'),
        DeclareLaunchArgument('lidar_x', default_value='2.0'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='2.11'),
        DeclareLaunchArgument('lidar_port', default_value='9302'),
        DeclareLaunchArgument('roi_y_abs', default_value='20.0'),
        DeclareLaunchArgument('ground_z', default_value='0.15'),
        DeclareLaunchArgument('ground_slope_tol', default_value='0.01'),
        DeclareLaunchArgument('moving_enter_mps', default_value='0.8'),
        DeclareLaunchArgument('ped_max_size_m', default_value='1.2'),
        DeclareLaunchArgument('corridor_half_m', default_value='1.5'),
        DeclareLaunchArgument('use_route', default_value='true'),
    ]

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / 'launch/ctrl_low_speed.launch.py')),
        launch_arguments={'config': cfg('config'),
                          'drive': cfg('drive'), 'speed_kmh': cfg('speed_kmh'),
                          'continuous': cfg('continuous'), 'distance_m': cfg('distance_m'),
                          'route': cfg('route'), 'rviz': cfg('rviz')}.items())

    # 주행용 RViz 와 겹치지 않게 라이다 쪽 rviz 는 끄고 뷰어 창만 쓴다.
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(lidar / 'launch/ctrl_obstacle.launch.py')),
        launch_arguments={'view': cfg('view'), 'rviz': 'false',
                          'lidar_x': cfg('lidar_x'), 'lidar_y': cfg('lidar_y'),
                          'lidar_z': cfg('lidar_z'), 'lidar_port': cfg('lidar_port'),
                          'roi_y_abs': cfg('roi_y_abs'), 'ground_z': cfg('ground_z'),
                          'ground_slope_tol': cfg('ground_slope_tol'),
                          'moving_enter_mps': cfg('moving_enter_mps'),
                          'ped_max_size_m': cfg('ped_max_size_m'),
                          'corridor_half_m': cfg('corridor_half_m'),
                          'use_route': cfg('use_route')}.items())

    # 카메라: 7번 하나 (YOLO 신호 인식) + 인식 오버레이 뷰어.
    cams_on = IfCondition(cfg('cameras'))
    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(camera / 'launch/ctrl_yolo_signal.launch.py')),
        condition=cams_on, launch_arguments={'port': cfg('camera_port')}.items())
    view = Node(package='ctrl_camera', executable='ctrl_camera_view', name='camera_view',
                output='log', condition=cams_on,
                parameters=[{'scale': ParameterValue(cfg('camera_scale'), value_type=float)}])
    # 노드가 한꺼번에 뜨면 path_follower 첫 제어 루프가 밀린다. 카메라 그룹은 5초 뒤에 띄운다.
    cameras = TimerAction(period=5.0, actions=[yolo, view])

    return LaunchDescription(arguments + [base, perception, cameras])
