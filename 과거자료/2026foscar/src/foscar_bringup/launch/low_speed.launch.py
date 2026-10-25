from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('foscar_bringup'))
    config = LaunchConfiguration('config')
    arguments = [
        DeclareLaunchArgument('config', default_value=str(share/'config/low_speed.yaml')),
        DeclareLaunchArgument('drive', default_value='false', description='Enable UDP control output'),
        DeclareLaunchArgument('speed_kmh', default_value='5.0'),
        DeclareLaunchArgument('distance_m', default_value='20.0'),
        DeclareLaunchArgument('continuous', default_value='false',
                              description='Ignore test distance and follow to route end; fault stops remain enabled'),
        DeclareLaunchArgument('route', default_value='auto', choices=['auto', 'spawn', 'global'],
                              description='auto: pick from where the car is; spawn: respawn connector + lap; '
                                          'global: official lap only. s is always global-lap s'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]
    path_file, spawn_file = str(share/'routes/global_path.txt'), str(share/'routes/spawn_path.txt')
    nodes = [
        Node(package='foscar_control', executable='sensor_bridge', output='screen', parameters=[config, {
            'drive': ParameterValue(LaunchConfiguration('drive'), value_type=bool)}]),
        Node(package='foscar_control', executable='localization', output='screen', parameters=[config, {
            'path_file': path_file, 'spawn_file': spawn_file}]),
        Node(package='foscar_control', executable='path_follower', output='screen', parameters=[config, {
            'path_file': path_file, 'spawn_file': spawn_file, 'route': LaunchConfiguration('route'),
            'speed_kmh': ParameterValue(LaunchConfiguration('speed_kmh'), value_type=float),
            'continuous': ParameterValue(LaunchConfiguration('continuous'), value_type=bool),
            'test_distance_m': ParameterValue(LaunchConfiguration('distance_m'), value_type=float)}]),
    ]
    # Losing a required node terminates the launch, allowing bridge shutdown braking.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason='required node exited'))])) for node in nodes]
    rviz = Node(package='rviz2', executable='rviz2', arguments=['-d', str(share/'rviz/low_speed.rviz')],
                condition=IfCondition(LaunchConfiguration('rviz')), output='screen')
    return LaunchDescription(arguments+handlers+nodes+[rviz])
