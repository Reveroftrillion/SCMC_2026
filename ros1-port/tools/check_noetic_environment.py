#!/usr/bin/env python3
"""Read-only deployment checks; never initialize nodes or connect to MORAI.

Run with the selected CPython 3.10 interpreter after sourcing the Noetic and
catkin setup files. Import checks are real, without verification ROS stubs.
"""
import sys
sys.dont_write_bytecode = True

import argparse
import hashlib
import importlib
from importlib import metadata
import io
import json
import math
import os
from pathlib import Path
import platform
import shlex
import shutil
sys.path.insert(0,str(Path(__file__).resolve().parent))
from ctrl_names import executable_name, module_name


WORKSPACE = Path(__file__).resolve().parents[1]
PINNED = {'numpy': '1.24.4', 'onnxruntime-gpu': '1.23.2'}
ENTRYPOINTS = {
    'ctrl_control': {'sensor_bridge': 'bridge', 'localization': 'localization', 'path_follower': 'follower'},
    'ctrl_lidar': {'obstacle_node': 'obstacle_node', 'lidar_bridge': 'lidar_bridge',
                     'lidar_view': 'lidar_view', 'fake_cloud': 'fake_cloud', 'lidar_monitor': 'lidar_monitor'},
    'ctrl_camera': {'camera_bridge': 'camera_bridge', 'yolo_signal_node': 'yolo_signal_node',
                      'camera_view': 'camera_view', 'gps_logger': 'gps_logger'},
    'ctrl_planning': {'behavior_node': 'behavior_node', 'bev_node': 'bev_node', 'race_hud_node': 'race_hud_node'},
}
INTERFACE_FIELDS = {
    'ControlCommand': [('header', 'std_msgs/Header'), ('accel', 'float64'), ('brake', 'float64'), ('steering_rad', 'float64'), ('state', 'string')],
    'VehicleStatus': [('header', 'std_msgs/Header'), ('speed_mps', 'float64'), ('yaw_rad', 'float64'), ('ctrl_mode', 'int8'), ('gear', 'int8'), ('steer_feedback_deg', 'float64'), ('link_id', 'string')],
    'Event': [('header', 'std_msgs/Header'), ('type', 'string'), ('s', 'float64'), ('x', 'float64'), ('y', 'float64'), ('size_x', 'float64'), ('size_y', 'float64'), ('vx', 'float64'), ('vy', 'float64'), ('value', 'string'), ('confidence', 'float64')],
    'DrivingTarget': [('header', 'std_msgs/Header'), ('state', 'string'), ('v_max', 'float64'), ('stop_s', 'float64'), ('lat_offset', 'float64'), ('return_s', 'float64'), ('return_length_m', 'float64'), ('entry_s', 'float64'), ('entry_length_m', 'float64'), ('chain_offset', 'float64[]'), ('chain_start_s', 'float64[]'), ('chain_length_m', 'float64[]'), ('reference_id', 'string'), ('reference_global_s', 'float64[]'), ('reference_x', 'float64[]'), ('reference_y', 'float64[]'), ('reference_release', 'bool')],
}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def check_python():
    require(platform.python_implementation() == 'CPython', 'Selected runtime must be CPython')
    require(sys.version_info[:2] == (3, 10), 'Selected deployment requires CPython 3.10; running ' + platform.python_version())
    require(sys.platform.startswith('linux'), 'Noetic deployment must be checked on Linux, not ' + sys.platform)
    return sys.executable + ' (' + platform.python_version() + ')'


def check_ros_environment():
    expected = {'ROS_VERSION': '1', 'ROS_DISTRO': 'noetic', 'ROS_PYTHON_VERSION': '3'}
    mismatches = [key + '=' + repr(os.environ.get(key)) for key, value in expected.items() if os.environ.get(key) != value]
    require(not mismatches, 'Source the selected Noetic setup; wrong/missing ' + ', '.join(mismatches))
    for executable in ('roslaunch', 'rosbag', 'catkin_make', 'rospack'):
        require(shutil.which(executable), executable + ' is not on PATH')
    return 'ROS1 Noetic tools are on PATH'


def check_pin(distribution):
    found = metadata.version(distribution)
    require(found == PINNED[distribution], '{} must be {}, found {}'.format(distribution, PINNED[distribution], found))
    module = importlib.import_module('onnxruntime' if distribution == 'onnxruntime-gpu' else distribution)
    require(module.__version__ == PINNED[distribution], distribution + ' metadata/module versions disagree')
    detail = '{} {}; {}'.format(distribution, found, module.__file__)
    if distribution == 'onnxruntime-gpu':
        providers = module.get_available_providers()
        require('CUDAExecutionProvider' in providers, 'GPU distribution is shadowed or lacks CUDA provider: ' + repr(providers))
        detail += '; compiled providers=' + repr(providers)
    return detail


def check_native_modules():
    import numpy as np
    import cv2
    import scipy.linalg
    import sklearn.cluster
    import pyproj
    import matplotlib
    require(cv2.cvtColor(np.zeros((1, 1, 3), dtype=np.uint8), cv2.COLOR_BGR2RGB).shape == (1, 1, 3), 'OpenCV native conversion failed')
    require(np.array_equal(scipy.linalg.solve(np.eye(2), np.ones(2)), np.ones(2)), 'SciPy native solve failed')
    labels = sklearn.cluster.DBSCAN(eps=.5, min_samples=2).fit_predict(np.array([[0., 0.], [.1, .1]]))
    require(np.array_equal(labels, [0, 0]), 'scikit-learn native clustering failed')
    xy = pyproj.Transformer.from_crs(4326, 32652, always_xy=True).transform(127., 37.)
    require(all(math.isfinite(number) for number in xy), 'pyproj native transform failed')
    return '; '.join(name + '=' + importlib.import_module(name).__version__ for name in ('cv2', 'scipy', 'sklearn', 'pyproj', 'matplotlib'))


def check_ros_imports():
    for module in ('rospy', 'rosgraph', 'roslib.packages', 'catkin_pkg', 'rospkg', 'std_msgs.msg',
                   'sensor_msgs.msg', 'geometry_msgs.msg', 'nav_msgs.msg',
                   'visualization_msgs.msg', 'rosgraph_msgs.msg', 'tf2_py', 'tf2_ros', 'ctrl_ros1'):
        importlib.import_module(module)
    return 'rospy, generated messages, catkin modules and native tf2 import under ' + sys.executable


def check_script_interpreter(path):
    with Path(path).open(encoding='utf-8') as stream:
        first_line = stream.readline().strip()
    require(first_line.startswith('#!'), 'Python executable has no shebang: ' + str(path))
    command = shlex.split(first_line[2:])
    require(command, 'Empty executable shebang: ' + str(path))
    if Path(command[0]).name == 'env':
        require(len(command) == 2, 'Unsupported env shebang: ' + first_line)
        interpreter = shutil.which(command[1])
    else:
        interpreter = command[0]
    require(interpreter and Path(interpreter).is_file(), 'Executable Python interpreter missing: ' + first_line)
    require(Path(interpreter).resolve() == Path(sys.executable).resolve(),
            str(path) + ' would run a different Python interpreter: ' + str(interpreter)
            + '; selected ' + sys.executable)


def check_entrypoints():
    import roslib.packages
    count = 0
    for package, executables in ENTRYPOINTS.items():
        for executable, short_module in executables.items():
            executable=executable_name(executable)
            original_package=package.replace('ctrl_','foscar_',1)
            module = importlib.import_module(module_name(original_package + '.' + short_module))
            require(callable(getattr(module, 'main', None)), package + '/' + executable + ' has no main')
            matches = roslib.packages.find_node(package, executable)
            require(matches, 'catkin executable not found: ' + package + '/' + executable)
            executable_paths = [path for path in matches if os.access(path, os.X_OK)]
            require(executable_paths, 'catkin executable is not executable: ' + package + '/' + executable)
            # roslaunch selects the executable path; verify that every matching
            # copy uses the selected Python, including devel/install wrappers.
            for path in executable_paths:
                check_script_interpreter(path)
            count += 1
    for executable in ('delay_wall', 'launch_info', 'record_bag'):
        executable=executable_name(executable)
        paths = roslib.packages.find_node('ctrl_bringup', executable)
        require(paths, 'bringup helper missing: ' + executable)
        for path in paths:
            check_script_interpreter(path)
    require(roslib.packages.find_node('tf2_ros', 'static_transform_publisher'), 'static TF executable missing')
    require(roslib.packages.find_node('rviz', 'rviz'), 'optional RViz executable missing')
    return '{} production/optional node modules and catkin executable paths/interpreters; no constructors invoked'.format(count)


def check_resources(package_lookup=None):
    if package_lookup is None:
        import rospkg
        package_lookup = rospkg.RosPack().get_path
    with (Path(__file__).with_name('ctrl_resource_hashes.json')).open(encoding='utf-8') as stream:
        expected = json.load(stream)['files']
    failures = []
    for relative, entry in expected.items():
        package, filename = relative.split('/', 1)
        try:
            path = Path(package_lookup(package)) / filename
            data = path.read_bytes()
            require(len(data) == entry['size'], 'size differs')
            require(hashlib.sha256(data).hexdigest() == entry['sha256'], 'SHA256 differs')
        except Exception as exc:
            failures.append(relative + ': ' + str(exc))
    require(not failures, '\n'.join(failures))
    return '{} installed/source-share resources match the reviewed CTRL deployment snapshot'.format(len(expected))


def check_messages():
    import rospy
    from ctrl_interfaces import msg as interfaces
    from sensor_msgs.msg import NavSatStatus, PointField
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Pose
    from std_msgs.msg import Header
    from ctrl_ros1 import ctrl_point_cloud2 as point_cloud2
    from ctrl_ros1.ctrl_messages import Marker as HumbleMarker
    for name, fields in INTERFACE_FIELDS.items():
        message = getattr(interfaces, name)
        require(list(zip(message.__slots__, message._slot_types)) == fields, 'stale or different generated interface: ' + name)
    require((NavSatStatus.STATUS_NO_FIX, NavSatStatus.STATUS_FIX, NavSatStatus.SERVICE_GPS) == (-1, 0, 1), 'NavSatStatus constants differ')
    require((PointField.INT8, PointField.UINT8, PointField.INT16, PointField.UINT16, PointField.INT32, PointField.UINT32, PointField.FLOAT32, PointField.FLOAT64) == tuple(range(1, 9)), 'PointField constants differ')
    require((Marker.ADD, Marker.DELETE, Marker.DELETEALL) == (0, 2, 3), 'Marker constants differ')
    marker = HumbleMarker()
    require(isinstance(marker, Marker) and marker.pose.orientation.w == 1., 'Humble Marker quaternion default was not preserved')
    explicit_pose = Pose()
    explicit_pose.orientation.w = 0.
    require(HumbleMarker(pose=explicit_pose).pose.orientation.w == 0., 'Marker adapter changed an explicit pose')
    stream = io.BytesIO()
    marker.serialize(stream)
    require(Marker().deserialize(stream.getvalue()).pose.orientation.w == 1., 'adapted Marker failed native inherited serialization')
    header = Header(stamp=rospy.Time(123, 456789012), frame_id='lidar')
    cloud = point_cloud2.create_cloud_xyz32(header, [(1., 2., 3.)])
    stream = io.BytesIO()
    cloud.serialize(stream)  # Real genpy uint8[] serialization, without a publisher.
    recovered = type(cloud)().deserialize(stream.getvalue())
    require(recovered.header.stamp == header.stamp and recovered.header.frame_id == 'lidar', 'PointCloud2 header changed on wire')
    require(bytes(recovered.data) == bytes(cloud.data), 'PointCloud2 point bytes changed on wire')
    command = interfaces.ControlCommand(header=header, accel=.25, brake=0., steering_rad=.1, state='TRACKING')
    stream = io.BytesIO()
    command.serialize(stream)
    recovered = type(command)().deserialize(stream.getvalue())
    require((recovered.accel, recovered.brake, recovered.steering_rad, recovered.state) == (.25, 0., .1, 'TRACKING'), 'ControlCommand fields changed on wire')
    return '4 generated interface contracts/constants, preserved Marker default and native PointCloud2/ControlCommand/Marker wire roundtrips'


def check_topology():
    from catkin_pkg.topological_order import topological_order
    source = WORKSPACE / 'src'
    if not source.is_dir():
        return 'source workspace unavailable in this install-only copy; run catkin build checks on build host'
    ordered = topological_order(str(source))
    require(ordered and all(path is not None for path, _package in ordered), 'catkin dependency cycle: ' + repr(ordered))
    return '{} catkin package manifests parse and topologically order'.format(len(ordered))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json', action='store_true', help='print results as JSON to stdout')
    args = parser.parse_args()
    results = []
    checks = [('python', check_python), ('ros_environment', check_ros_environment),
              ('numpy_pin', lambda: check_pin('numpy')), ('onnxruntime_pin', lambda: check_pin('onnxruntime-gpu')),
              ('native_libraries', check_native_modules), ('ros_imports', check_ros_imports),
              ('entrypoints', check_entrypoints), ('resources', check_resources),
              ('messages', check_messages), ('catkin_topology', check_topology)]
    for name, function in checks:
        try:
            results.append({'check': name, 'status': 'PASS', 'detail': function()})
        except Exception as exc:
            results.append({'check': name, 'status': 'FAIL', 'detail': type(exc).__name__ + ': ' + str(exc)})
    # The source artifact and its model README disagree. Preserve both and
    # report the discrepancy; resource integrity is enforced against the
    # source bytes above, without replacing the original runtime checks.
    results.append({'check': 'source_model_readme', 'status': 'KNOWN_SOURCE_ISSUE',
                    'detail': 'best.onnx actual SHA256 d3c3939f51cc8cfa19d0eefc815c3bb9eaae6533e795f8ff87f347e9ed048b98; preserved README claims 04ab171d8a46b5576891a23034000fa34cc63bf63336c8edb67b9a3fee551b8b'})
    if args.json:
        print(json.dumps({'checks': results, 'driving_started': False, 'ros_nodes_initialized': False}, indent=2))
    else:
        for result in results:
            print('{} {}: {}'.format(result['status'], result['check'], result['detail']))
        print('No ROS nodes, publishers, UDP sockets or simulator commands were started. CUDA session loading, model inference, ROS-master transport, live timers, build and driving remain separate checks.')
    return 1 if any(result['status'] == 'FAIL' for result in results) else 0


if __name__ == '__main__':
    raise SystemExit(main())
