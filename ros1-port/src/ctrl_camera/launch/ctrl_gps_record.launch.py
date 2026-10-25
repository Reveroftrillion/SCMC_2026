"""주행 스택을 띄운 채 **터미널에서 키를 누르면 현재 GPS 위경도를 CSV 로 저장**한다.

    ros2 launch ctrl_camera ctrl_gps_record.launch.py                   # 자율주행 (20 km/h) 하며 기록
    ros2 launch ctrl_camera ctrl_gps_record.launch.py drive:=false      # 직접 몰면서 기록

    (launch 를 띄운 터미널에 포커스를 두고)  아무 키 = 저장,  q = 전체 종료

저장 위치: `~/foscar_logs/gps_<시각>.csv`, 열은 `time,lat,lon,alt,x,y`. 첫 저장 때 파일이 생긴다.
x, y 는 map 절대좌표 [m] (WGS84 → UTM 52N − 원점, `global_path.txt` 와 같은 좌표계).

안에서 `ctrl_bringup/ctrl_low_speed.launch.py` (sensor_bridge + localization + path_follower) 를
그대로 include 하고 `gps_logger` 만 얹는다. GPS 는 sensor_bridge 가 `/sensors/gps` 로 낸다.

`ctrl_drive.launch.py` 와 같은 규약: 기본 `drive:=true` 라 띄우는 순간 차가 출발한다.
**직접 몰 때는 반드시 `drive:=false`** — 제어 명령이 없는 동안 `ctrl_bridge.py` 가 풀 브레이크를
50 Hz 로 보내기 때문이다.

`gps_logger` 가 끝나면(q) launch 전체를 내려서 bridge 의 종료 제동이 걸리게 한다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = Path(get_package_share_directory('ctrl_bringup'))
    arguments = [
        DeclareLaunchArgument('drive', default_value='true', description='제어 송신. 직접 몰 때는 false'),
        DeclareLaunchArgument('speed_kmh', default_value='20.0'),
        DeclareLaunchArgument('continuous', default_value='true'),
        DeclareLaunchArgument('route', default_value='auto'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('save_dir', default_value=str(Path.home() / 'foscar_logs'),
                              description='CSV 저장 폴더 (gps_<시각>.csv)'),
    ]
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / 'launch/ctrl_low_speed.launch.py')),
        launch_arguments={k: LaunchConfiguration(k)
                          for k in ('drive', 'speed_kmh', 'continuous', 'route', 'rviz')}.items())
    logger = Node(package='ctrl_camera', executable='ctrl_gps_logger', output='screen',
                  parameters=[{'save_dir': LaunchConfiguration('save_dir')}])
    on_quit = RegisterEventHandler(OnProcessExit(
        target_action=logger, on_exit=[EmitEvent(event=Shutdown(reason='gps_logger exited'))]))
    return LaunchDescription(arguments + [base, logger, on_quit])
