import sys
import threading

import rospy
import yaml

_initialized = False
_argv = None
_overrides = {}
_parameter_files = []
_init_lock = threading.Lock()
_default_executor = None


def init(args=None):
    """Keep ROS1 remappings; parse source test/CLI parameter override syntax.

    rospy requires a node name, so transport initialization occurs at Node().
    ROS1 intentionally supports one node/context lifetime per process.
    """
    global _initialized, _argv, _overrides, _parameter_files
    if rospy.is_shutdown():
        raise RuntimeError('rospy cannot restart after shutdown; start a new process')
    values = list(sys.argv[1:] if args is None else args)
    _argv = [sys.argv[0]]
    _overrides = {}
    _parameter_files = []
    in_ros_args = False
    index = 0
    while index < len(values):
        value = values[index]
        if value == '--ros-args':
            in_ros_args = True
        elif value == '--':
            in_ros_args = False
        elif in_ros_args and value in ('-p', '--param'):
            index += 1
            name, raw = values[index].split(':=', 1)
            _overrides[name] = yaml.safe_load(raw)
        elif in_ros_args and value == '--params-file':
            index += 1
            _parameter_files.append(values[index])
        elif in_ros_args:
            raise ValueError('Unsupported ROS2 CLI argument in ROS1 port: ' + value)
        else:
            _argv.append(value)
        index += 1
    _initialized = True


def ensure_node(name):
    if not _initialized:
        raise RuntimeError('Call ctrl_ros1.init() before creating a Node')
    with _init_lock:
        if not rospy.core.is_initialized():
            rospy.init_node(name, argv=_argv)


def ok():
    return _initialized and not rospy.is_shutdown()


def shutdown():
    global _initialized
    rospy.signal_shutdown('ctrl_ros1 shutdown')
    _initialized = False


def try_shutdown():
    if ok():
        shutdown()


def get_global_executor():
    global _default_executor
    if _default_executor is None:
        from .ctrl_executors import SingleThreadedExecutor
        _default_executor = SingleThreadedExecutor()
    return _default_executor
