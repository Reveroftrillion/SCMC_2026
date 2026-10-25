"""Camera 7 RGB only, no legacy stop_line_node or control publisher.

Camera 1 was tried and dropped (2026-09-29): at the intersection-02 stop line its lamp leaves the frame,
so it never saw the green and the car stayed. Every processed frame is logged to ~/foscar_logs/yolo/.
"""
from datetime import datetime
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('ctrl_camera'))
    logs = Path.home() / 'foscar_logs' / 'yolo'
    logs.mkdir(parents=True, exist_ok=True)
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9208'),
        DeclareLaunchArgument('model', default_value=str(share / 'models/best.onnx')),
        Node(package='ctrl_camera', executable='ctrl_camera_bridge', name='camera_bridge', output='screen', parameters=[{
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'topic': '/sensors/camera/traffic/compressed', 'frame_id': 'camera_7', 'stamp_source': 'recv'}]),
        Node(package='ctrl_camera', executable='ctrl_yolo_signal_node', output='screen', parameters=[{
            'model_file': LaunchConfiguration('model'), 'stoplines_file': str(share / 'config/yolo_stoplines.json'),
            'detections_log': str(logs / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_cam7.jsonl')),
            'confidence': .4, 'confirm_frames': 2, 'max_frame_age_s': 1., 'threads': 4}])])
