"""신호등 데이터 수집 실행.

    # 1. 진단 — 본 수집 전 반드시 먼저. 저장하지 않는다 (명세서 §7)
    ros2 launch ctrl_camera_train ctrl_record.launch.py check:=true drive:=false

    # 2. 본 수집 — 조건마다 한 번씩 (Sunny/Foggy × 11/13/15시 = 6회)
    ros2 launch ctrl_camera_train ctrl_record.launch.py tag:=sunny_11h
    ros2 launch ctrl_camera_train ctrl_record.launch.py tag:=foggy_15h

`ctrl_bringup/ctrl_low_speed.launch.py`(sensor_bridge + localization + path_follower) 를 include 하고
카메라 브리지 둘(자유 9208 / semantic 9210)과 수집기를 띄운다. `continuous:=true` 가 기본이라
시험거리에서 멈추지 않고 랩을 끝까지 돈다 — 수집은 한 바퀴를 다 봐야 한다.

`traffic_light.launch.py` 는 include 하지 않는다 — 수집에 `traffic_light_node` 는 필요 없고,
브리지가 중복으로 떠서 포트가 충돌한다.

⚠️ 이 런치는 **semantic 카메라를 쓴다. 대회 주행 중에는 절대 띄우지 않는다.**
⚠️ MORAI 시나리오를 Load 하면 네트워크가 끊기고 내장 자율주행이 출발한다. P키로 세우고
   F5 Load 후 네트워크 설정을 다시 잡은 다음 이 런치를 띄울 것.
"""

from pathlib import Path
import math
import signal
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def record_process_exit(event, context):
    """The one-shot helper needs node failures even if ros2 launch exits zero."""
    failure_file = LaunchConfiguration('failure_file').perform(context)
    if not failure_file:
        return []
    if context.is_shutdown and event.returncode in (0, -signal.SIGINT, 130):
        return []
    reason = f'{event.process_name}: unexpected exit {event.returncode}'
    with Path(failure_file).open('a') as stream:
        stream.write(reason + '\n')
    if not context.is_shutdown:
        return [EmitEvent(event=Shutdown(reason=reason))]
    return []


def collection_drive_stack(context, bringup, record_config):
    seconds = float(LaunchConfiguration('stop_seconds').perform(context))
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError('stop_seconds must be finite and >= 0')
    enabled = (seconds > 0 and IfCondition(LaunchConfiguration('drive')).evaluate(context)
               and not IfCondition(LaunchConfiguration('check')).evaluate(context))
    drive_config = bringup / 'config/low_speed.yaml'
    cleanup = []
    stop_actions = []
    if enabled:
        # Retain the current driving configuration, overriding only its target
        # watchdog for collection. Never modify another team's source YAML.
        params = yaml.safe_load(drive_config.read_text())
        params['path_follower']['ros__parameters']['require_target'] = True
        with tempfile.NamedTemporaryFile(mode='w', prefix='launch_params_collection_',
                                         suffix='.yaml', delete=False) as stream:
            yaml.safe_dump(params, stream)
            drive_config = Path(stream.name)
        cleanup = [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _context: drive_config.unlink(missing_ok=True))]))]
        stop = Node(package='ctrl_camera_train', executable='ctrl_collection_stop_node',
                    name='collection_stop_node', output='screen',
                    parameters=[record_config.perform(context), {
                        'stop_seconds': seconds,
                        'route': ParameterValue(LaunchConfiguration('route'), value_type=str),
                    }])
        guard = RegisterEventHandler(OnProcessExit(target_action=stop, on_exit=[
            EmitEvent(event=Shutdown(reason='collection stop node exited'))]))
        stop_actions = [guard, stop]
    drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / 'launch/ctrl_low_speed.launch.py')),
        launch_arguments={
            'config': str(drive_config),
            'drive': LaunchConfiguration('drive'),
            'speed_kmh': LaunchConfiguration('speed_kmh'),
            'continuous': LaunchConfiguration('continuous'),
            'route': LaunchConfiguration('route'),
        }.items())
    return cleanup + [GroupAction(stop_actions + [drive], scoped=True)]


def generate_launch_description():
    share = Path(get_package_share_directory('ctrl_camera_train'))
    bringup = Path(get_package_share_directory('ctrl_bringup'))
    config = LaunchConfiguration('config')
    check = LaunchConfiguration('check')

    arguments = [
        DeclareLaunchArgument('config', default_value=str(share / 'config/record.yaml')),
        DeclareLaunchArgument('tag', default_value='',
                              description='조건 라벨 (sunny_11h 등). 본 수집에선 반드시 준다 — '
                                          '조건 단위 train/val 분할이 이 값을 쓴다'),
        DeclareLaunchArgument('check', default_value='false',
                              description='저장 없이 진단만 (팔레트·정렬·Δstamp). 본 수집 전 필수'),
        DeclareLaunchArgument('view', default_value='true',
                              description='check 모드의 오버레이 창'),
        DeclareLaunchArgument('drive', default_value='true',
                              description='자율주행으로 수집. check 때는 false 로 세워두고 봐도 된다'),
        DeclareLaunchArgument('speed_kmh', default_value='20.0'),
        DeclareLaunchArgument('stop_seconds', default_value='60.0',
                              description='각 정지선에서 실제 정지 후 대기할 시간. 0이면 비활성'),
        DeclareLaunchArgument('continuous', default_value='true',
                              description='시험거리에서 멈추지 않고 랩 끝까지'),
        DeclareLaunchArgument('route', default_value='auto', choices=['auto', 'global', 'spawn']),
        DeclareLaunchArgument('out_dir', default_value='~/foscar_logs/dataset'),
        DeclareLaunchArgument('failure_file', default_value='',
                              description='1회 수집 도우미가 읽는 노드 비정상 종료 기록 (선택)'),
    ]

    drive_stack = OpaqueFunction(function=collection_drive_stack,
                                 kwargs={'bringup': bringup, 'record_config': config})

    # 자유 카메라(RGB) 와 semantic. 이름이 record.yaml 의 블록 이름과 같아야 파라미터가 붙는다.
    bridges = [
        Node(package='ctrl_camera', executable='ctrl_camera_bridge', name=name,
             output='screen', parameters=[config])
        for name in ('camera_bridge', 'camera_bridge_semantic')]

    recorder = Node(
        package='ctrl_camera_train', executable='ctrl_recorder_node', name='recorder_node',
        output='screen',
        parameters=[config, {
            'tag': ParameterValue(LaunchConfiguration('tag'), value_type=str),
            'out_dir': ParameterValue(LaunchConfiguration('out_dir'), value_type=str),
            'check': ParameterValue(check, value_type=bool),
            'check_view': ParameterValue(LaunchConfiguration('view'), value_type=bool),
        }])

    # Both launches use `config`. Give the drive stack its own file and scope so
    # it neither inherits record.yaml nor replaces the camera configuration.
    # Also observe exits from the included drive stack. Its handlers queue a
    # Shutdown event, so this ProcessExited is still reported before shutdown.
    failure_guard = RegisterEventHandler(OnProcessExit(on_exit=record_process_exit))
    return LaunchDescription(arguments + [failure_guard, drive_stack]
                             + bridges + [recorder])
