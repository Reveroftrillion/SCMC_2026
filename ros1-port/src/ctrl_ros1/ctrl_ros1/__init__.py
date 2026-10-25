"""Explicit ROS1 boundary; this package never installs an rclpy alias."""
from .ctrl_context import init, ok, shutdown, try_shutdown, get_global_executor
from . import ctrl_parameter as parameter


def spin(node, executor=None):
    executor = executor or get_global_executor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)


def spin_once(node, executor=None, timeout_sec=None):
    executor = executor or get_global_executor()
    executor.add_node(node)
    try:
        executor.spin_once(timeout_sec=timeout_sec)
    finally:
        executor.remove_node(node)
