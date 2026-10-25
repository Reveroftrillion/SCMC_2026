"""ROS1 node boundary for the exact APIs used by the source nodes."""
import inspect
from pathlib import Path
import threading
import time

import rospy
import yaml

from . import ctrl_context as _context
from .ctrl_entities import Publisher, Subscription, Timer
from .ctrl_callback_groups import MutuallyExclusiveCallbackGroup
from .ctrl_parameter import Parameter
from .ctrl_time import Time


def _flatten(values, prefix=''):
    result = {}
    for key, value in values.items():
        name = prefix + str(key)
        if isinstance(value, dict):
            result.update(_flatten(value, name + '.'))
        else:
            result[name] = value
    return result


def _read_parameter_file(path, node_name, full_name):
    with Path(path).open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError('ROS2 parameter file must be a mapping: ' + str(path))
    result = {}
    # Files in the source use bare node names. Support the source's wildcard
    # form too, with node-specific values overriding wildcard values.
    sections = []
    for key in ('/**', node_name, full_name):
        if key in data and key not in sections:
            sections.append(key)
    for key in sections:
        values = data[key].get('ros__parameters', {})
        if not isinstance(values, dict):
            raise ValueError('ros__parameters must be a mapping in ' + str(path))
        result.update(_flatten(values))
    return result


class _Clock:
    def __init__(self, node):
        self.node = node
        self._ros_time_active = bool(rospy.get_param('/use_sim_time', False))
        self._last = None
        self._lock = threading.Lock()

    def now(self):
        with self._lock:
            stamp = rospy.Time.now()
            self._observe(stamp.to_nsec())
            return Time.from_msg(stamp)

    def observe(self, nanoseconds):
        with self._lock:
            self._observe(nanoseconds)

    def _observe(self, nanoseconds):
        old = self._last
        self._last = nanoseconds
        if self._ros_time_active and old is not None and nanoseconds < old:
            for timer in tuple(self.node.timers):
                timer.on_clock_jump(old, nanoseconds)


class _Logger:
    def __init__(self, name):
        self.name = name
        self._calls = {}
        self._lock = threading.Lock()

    def _log(self, level, message, throttle_duration_sec=None, once=False,
             skip_first=False):
        frame = inspect.currentframe().f_back.f_back
        key = (frame.f_code, frame.f_lineno, level)
        now = time.monotonic()
        with self._lock:
            previous, count = self._calls.get(key, (None, 0))
            self._calls[key] = (previous, count + 1)
            if (once and count > 0) or (skip_first and count == 0):
                return
            if (throttle_duration_sec is not None and previous is not None and
                    now - previous < throttle_duration_sec):
                return
            self._calls[key] = (now, count + 1)
        getattr(rospy, 'log' + level)(message)

    def debug(self, message, **kwargs):
        self._log('debug', message, **kwargs)

    def info(self, message, **kwargs):
        self._log('info', message, **kwargs)

    def warning(self, message, **kwargs):
        self._log('warn', message, **kwargs)

    warn = warning

    def error(self, message, **kwargs):
        self._log('err', message, **kwargs)

    def fatal(self, message, **kwargs):
        self._log('fatal', message, **kwargs)


class Node:
    def __init__(self, node_name, *, parameter_overrides=None,
                 automatically_declare_parameters_from_overrides=False,
                 allow_undeclared_parameters=False):
        _context.ensure_node(node_name)
        self._name = rospy.get_name().rsplit('/', 1)[-1]
        self._full_name = rospy.get_name()
        self.default_callback_group = MutuallyExclusiveCallbackGroup()
        self.executor = None
        self.publishers = []
        self.subscriptions = []
        self.timers = []
        self._destroyed = False
        self._parameters = {}
        self._allow_undeclared = allow_undeclared_parameters
        self._clock = _Clock(self)
        self._logger = _Logger(self._name)
        self._overrides = {}
        parameter_files = rospy.get_param('~_ros2_parameter_files', [])
        if isinstance(parameter_files, str):
            parameter_files = [parameter_files]
        for path in parameter_files:
            self._overrides.update(_read_parameter_file(path, self._name, self._full_name))
        self._cli_overrides = {}
        for path in _context._parameter_files:
            self._cli_overrides.update(_read_parameter_file(path, self._name, self._full_name))
        self._cli_overrides.update(_context._overrides)
        for parameter in parameter_overrides or ():
            self._cli_overrides[parameter.name] = parameter.value
        # rospy owns /clock. This observer only wakes the executor and records
        # every jump; it never invokes an application callback.
        self._clock_subscription = None
        if rospy.get_param('/use_sim_time', False):
            from rosgraph_msgs.msg import Clock
            self._clock_subscription = rospy.Subscriber('/clock', Clock, self._on_clock)
        if automatically_declare_parameters_from_overrides:
            overrides = dict(self._overrides, **self._cli_overrides)
            for name, value in overrides.items():
                self.declare_parameter(name, value)

    @staticmethod
    def _parameter_key(name):
        # ROS1 names cannot contain periods. This encoding leaves Python-side
        # parameter names exactly as in the original calculation code.
        return '~' + name.replace('.', '/')

    def declare_parameter(self, name, value=None):
        if name in self._parameters:
            raise ValueError('Parameter already declared: ' + name)
        default = value
        selected = self._overrides.get(name, default)
        selected = rospy.get_param(self._parameter_key(name), selected)
        selected = self._cli_overrides.get(name, selected)
        parameter = Parameter(name, value=selected)
        if default is not None:
            expected = Parameter.type_from_value(default)
            if expected != parameter.type_:
                raise TypeError('Parameter %s expected %s, got %s' %
                                (name, expected.name, parameter.type_.name))
        if name == 'use_sim_time' and bool(selected) != bool(rospy.get_param('/use_sim_time', False)):
            raise ValueError('ROS1 use_sim_time must match the global /use_sim_time parameter')
        self._parameters[name] = parameter
        return parameter

    def get_parameter(self, name):
        if name in self._parameters:
            return self._parameters[name]
        if self._allow_undeclared:
            return Parameter(name)
        raise KeyError('Parameter not declared: ' + name)

    def get_name(self):
        return self._name

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    def _on_clock(self, msg):
        self._clock.observe(msg.clock.to_nsec())
        self._wake()

    def _wake(self):
        executor = self.executor
        if executor is not None:
            executor.wake()

    def create_publisher(self, msg_type, topic, qos_profile):
        publisher = Publisher(msg_type, topic, qos_profile)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos_profile, *, callback_group=None):
        subscription = Subscription(self, msg_type, topic, callback, qos_profile,
                                    callback_group or self.default_callback_group)
        self.subscriptions.append(subscription)
        self._wake()
        return subscription

    def create_timer(self, timer_period_sec, callback, *, callback_group=None, clock=None):
        timer = Timer(self, timer_period_sec, callback,
                      callback_group or self.default_callback_group, clock or self._clock)
        self.timers.append(timer)
        self._wake()
        return timer

    def destroy_publisher(self, publisher):
        publisher.destroy()
        self.publishers.remove(publisher)
        return True

    def destroy_subscription(self, subscription):
        subscription.destroy()
        self.subscriptions.remove(subscription)
        return True

    def destroy_timer(self, timer):
        timer.destroy()
        self.timers.remove(timer)
        return True

    def destroy_node(self):
        if self._destroyed:
            return
        self._destroyed = True
        if self.executor is not None:
            self.executor.remove_node(self)
        for entity in tuple(self.timers) + tuple(self.subscriptions) + tuple(self.publishers):
            entity.destroy()
        self.timers.clear()
        self.subscriptions.clear()
        self.publishers.clear()
        if self._clock_subscription is not None:
            self._clock_subscription.unregister()
