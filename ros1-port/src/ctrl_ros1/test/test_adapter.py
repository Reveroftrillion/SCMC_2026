"""Executor contract tests, without a ROS master or ROS installation.

Only the transport/time boundary is doubled. Production executor, queue,
parameter, callback-group and timer code is exercised directly.
"""
import importlib
import json
from pathlib import Path
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import patch

PACKAGE = Path(__file__).resolve().parents[1]


def fake_rospy():
    module = types.ModuleType('rospy')
    module.now_ns = 0
    module.initialized = False
    module.stopped = False
    module.params = {}
    module.logs = []
    module.name = '/adapter_test'

    class NativeTime:
        def __init__(self, secs=0, nsecs=0):
            self.secs, self.nsecs = divmod(int(secs * 10**9) + int(nsecs), 10**9)

        def to_nsec(self):
            return self.secs * 10**9 + self.nsecs

        def to_sec(self):
            return self.to_nsec() * 1e-9

        @staticmethod
        def now():
            return NativeTime(nsecs=module.now_ns)

    class NativeDuration(NativeTime):
        pass

    class Publisher:
        def __init__(self, *args, **kwargs):
            self.options = kwargs
            self.messages = []
            self.destroyed = False

        def publish(self, message):
            self.messages.append(message)

        def unregister(self):
            self.destroyed = True

        def get_num_connections(self):
            return 1

    class Subscriber:
        def __init__(self, topic, message_type, callback, **kwargs):
            self.callback = callback
            self.options = kwargs
            self.destroyed = False

        def unregister(self):
            self.destroyed = True

    def init_node(name, argv=None):
        module.initialized = True
        module.name = '/' + name

    module.Time = NativeTime
    module.Duration = NativeDuration
    module.Publisher = Publisher
    module.Subscriber = Subscriber
    module.init_node = init_node
    module.core = types.SimpleNamespace(is_initialized=lambda: module.initialized)
    module.is_shutdown = lambda: module.stopped
    module.get_name = lambda: module.name
    module.get_param = lambda name, default=None: module.params.get(name, default)
    module.signal_shutdown = lambda reason: setattr(module, 'stopped', True)
    for level in ('debug', 'info', 'warn', 'err', 'fatal'):
        setattr(module, 'log' + level, lambda text: module.logs.append(text))
    return module


class AdapterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ros = fake_rospy()
        clock_messages = types.ModuleType('rosgraph_msgs.msg')
        clock_messages.Clock = type('Clock', (), {})
        native_tf = types.ModuleType('tf2_ros')

        class Buffer:
            def __init__(self, cache_time=None, debug=True):
                self.cache_time, self.debug = cache_time, debug
                self.received = []

            def set_transform(self, transform, authority):
                self.received.append(('dynamic', transform, authority))

            def set_transform_static(self, transform, authority):
                self.received.append(('static', transform, authority))

        native_tf.Buffer = Buffer
        tf_messages = types.ModuleType('tf2_msgs.msg')
        tf_messages.TFMessage = type('TFMessage', (), {})
        modules = {'rospy': cls.ros, 'rosgraph_msgs.msg': clock_messages,
                   'tf2_ros': native_tf, 'tf2_msgs.msg': tf_messages}
        try:
            import yaml
        except ImportError:
            # JSON is a YAML subset: the tests below write JSON parameter files.
            yaml = types.ModuleType('yaml')
            yaml.safe_load = lambda data: json.loads(data.read() if hasattr(data, 'read') else data)
            modules['yaml'] = yaml
        cls.module_patch = patch.dict(sys.modules, modules)
        cls.module_patch.start()
        for name in list(sys.modules):
            if name == 'ctrl_ros1' or name.startswith('ctrl_ros1.'):
                del sys.modules[name]
        sys.path.insert(0, str(PACKAGE))
        cls.runtime = importlib.import_module('ctrl_ros1')
        cls.Node = importlib.import_module('ctrl_ros1.ctrl_node').Node
        cls.executors = importlib.import_module('ctrl_ros1.ctrl_executors')
        cls.groups = importlib.import_module('ctrl_ros1.ctrl_callback_groups')
        cls.qos = importlib.import_module('ctrl_ros1.ctrl_qos')

    @classmethod
    def tearDownClass(cls):
        cls.module_patch.stop()
        sys.path.remove(str(PACKAGE))

    def setUp(self):
        self.ros.now_ns = 0
        self.ros.initialized = False
        self.ros.stopped = False
        self.ros.params = {'/use_sim_time': True}
        self.runtime.init(args=[])
        self.nodes = []
        self.pools = []

    def tearDown(self):
        for pool in self.pools:
            pool.shutdown(timeout_sec=2.)
        for node in self.nodes:
            node.destroy_node()

    def node(self, **kwargs):
        node = self.Node('adapter_test', **kwargs)
        self.nodes.append(node)
        return node

    def executor(self, node, multi=False):
        cls = self.executors.MultiThreadedExecutor if multi else self.executors.SingleThreadedExecutor
        executor = cls()
        executor.add_node(node)
        self.pools.append(executor)
        return executor

    def test_ready_snapshot_timer_first_and_one_message_per_entity(self):
        node = self.node()
        result = []
        first = node.create_subscription(object, '/first', lambda x: result.append(x), 5)
        second = node.create_subscription(object, '/second', lambda x: result.append(x), 5)
        node.create_timer(.1, lambda: result.append('timer'))
        first.receive('a1')
        first.receive('a2')
        second.receive('b1')
        self.ros.now_ns = 100_000_000
        executor = self.executor(node)
        for _ in range(4):
            executor.spin_once(timeout_sec=0.)
        self.assertEqual(result, ['timer', 'a1', 'b1', 'a2'])

    def test_keep_last_depth_and_take_at_execution(self):
        node = self.node()
        result = []
        subscription = node.create_subscription(object, '/scan', result.append, 1)
        subscription.receive('old')
        node.create_timer(.1, lambda: subscription.receive('new'))
        self.ros.now_ns = 100_000_000
        executor = self.executor(node)
        executor.spin_once(timeout_sec=0.)
        executor.spin_once(timeout_sec=0.)
        self.assertEqual(result, ['new'])
        other = node.create_subscription(object, '/events', result.append, 2)
        for value in range(5):
            other.receive(value)
        executor.spin_once(timeout_sec=0.)
        executor.spin_once(timeout_sec=0.)
        self.assertEqual(result, ['new', 3, 4])

    def test_ingress_thread_never_executes_application_callback(self):
        node = self.node()
        threads = []
        subscription = node.create_subscription(object, '/input',
                                                lambda value: threads.append(threading.get_ident()), 1)
        ingress = threading.Thread(target=subscription.receive, args=(1,))
        ingress.start()
        ingress.join()
        self.assertEqual(threads, [])
        executor = self.executor(node)
        executor.spin_once(timeout_sec=0.)
        self.assertEqual(threads, [threading.get_ident()])

    def test_ros_clock_pause_backward_jump_and_missed_periods(self):
        node = self.node()
        result = []
        timer = node.create_timer(.1, lambda: result.append(self.ros.now_ns))
        executor = self.executor(node)
        executor.spin_once(timeout_sec=.005)
        self.assertEqual(result, [])
        self.ros.now_ns = 350_000_000
        executor.spin_once(timeout_sec=0.)
        self.assertEqual(result, [350_000_000])
        self.assertEqual(timer.time_until_next_call(), 50_000_000)
        self.ros.now_ns = 50_000_000
        executor.spin_once(timeout_sec=0.)
        self.assertEqual(timer.time_until_next_call(), 100_000_000)
        self.ros.now_ns = 150_000_000
        executor.spin_once(timeout_sec=0.)
        self.assertEqual(result, [350_000_000, 150_000_000])
        timer.cancel()
        self.ros.now_ns = 1_000_000_000
        executor.spin_once(timeout_sec=0.)
        self.assertEqual(len(result), 2)
        timer.reset()
        self.assertEqual(timer.time_until_next_call(), 100_000_000)

    def test_timer_missed_deadline_equality_matches_rcl(self):
        node = self.node()
        result = []
        timer = node.create_timer(.1, lambda: result.append(1))
        executor = self.executor(node)
        self.ros.now_ns = 300_000_000
        executor.spin_once(timeout_sec=0.)
        self.assertEqual(timer.time_until_next_call(), 0)
        executor.spin_once(timeout_sec=0.)
        self.assertEqual(timer.time_until_next_call(), 100_000_000)
        self.assertEqual(len(result), 2)

    def test_default_group_exclusion_and_independent_events_group(self):
        node = self.node()
        tick_started, release_tick, received = threading.Event(), threading.Event(), threading.Event()
        result = []

        def tick():
            tick_started.set()
            if not release_tick.wait(1.):
                raise RuntimeError('Test did not release tick')
            result.append('tick')

        node.create_timer(.1, tick)
        default = node.create_subscription(object, '/default', lambda x: result.append('default'), 1)
        independent = node.create_subscription(object, '/events', lambda x: received.set(), 1,
                                               callback_group=self.groups.MutuallyExclusiveCallbackGroup())
        default.receive(1)
        independent.receive(1)
        self.ros.now_ns = 100_000_000
        executor = self.executor(node, multi=True)
        try:
            executor.spin_once(timeout_sec=0.)
            self.assertTrue(tick_started.wait(1.))
            executor.spin_once(timeout_sec=0.)
            self.assertTrue(received.wait(1.))
            self.assertEqual(result, [])
        finally:
            release_tick.set()
        for future in tuple(executor._futures):
            future.result(timeout=1.)
        executor.spin_once(timeout_sec=0.)
        executor.shutdown(timeout_sec=1.)
        self.assertEqual(result, ['tick', 'default'])

    def test_callback_error_propagates_to_main(self):
        node = self.node()

        def fail(value):
            raise ValueError('visible callback failure')

        subscription = node.create_subscription(object, '/input', fail, 1)
        executor = self.executor(node)
        subscription.receive(1)
        with self.assertRaisesRegex(ValueError, 'visible callback failure'):
            executor.spin_once(timeout_sec=0.)
        self.assertTrue(node.default_callback_group.can_execute(subscription))

    def test_background_callback_error_propagates_while_idle(self):
        node = self.node()

        def fail(value):
            raise ValueError('visible worker failure')

        subscription = node.create_subscription(object, '/input', fail, 1)
        executor = self.executor(node, multi=True)
        subscription.receive(1)
        executor.spin_once(timeout_sec=0.)
        for future in tuple(executor._futures):
            with self.assertRaisesRegex(ValueError, 'visible worker failure'):
                future.result(timeout=1.)
        with self.assertRaisesRegex(ValueError, 'visible worker failure'):
            executor.spin_once(timeout_sec=.1)
        # The failed future has been inspected; permit orderly test cleanup.
        executor._futures.clear()

    def test_reentrant_entity_is_reserved_before_worker_takes_message(self):
        node = self.node()
        result = []
        subscription = node.create_subscription(object, '/input', result.append, 2,
                                                callback_group=self.groups.ReentrantCallbackGroup())
        subscription.receive(1)
        subscription.receive(2)
        executor = self.executor(node)
        dispatched = executor._next(0.)
        self.assertIs(dispatched, subscription)
        self.assertIsNone(executor._next(0.))
        executor._execute(dispatched)
        executor.spin_once(timeout_sec=0.)
        self.assertEqual(result, [1, 2])

    def test_shutdown_waits_for_running_worker(self):
        node = self.node()
        started, release, stopped = threading.Event(), threading.Event(), threading.Event()

        def callback(value):
            started.set()
            if not release.wait(1.):
                raise RuntimeError('Test did not release worker')

        subscription = node.create_subscription(object, '/input', callback, 1)
        subscription.receive(1)
        executor = self.executor(node, multi=True)
        executor.spin_once(timeout_sec=0.)
        self.assertTrue(started.wait(1.))

        def stop():
            executor.shutdown(timeout_sec=1.)
            stopped.set()

        stopper = threading.Thread(target=stop)
        stopper.start()
        try:
            self.assertFalse(stopped.wait(.01))
        finally:
            release.set()
            stopper.join(timeout=1.)
        self.assertTrue(stopped.is_set())

    def test_tf_updates_follow_executor_subscription_order(self):
        tf = importlib.import_module('ctrl_ros1.ctrl_tf2_ros')
        node = self.node()
        buffer = tf.Buffer(cache_time=self.ros.Duration(secs=2))
        listener = tf.TransformListener(buffer, node)
        observed = []
        cloud = node.create_subscription(object, '/cloud', lambda x: observed.append(len(buffer.received)), 1)
        listener.tf_sub.receive(types.SimpleNamespace(transforms=['moving']))
        listener.tf_static_sub.receive(types.SimpleNamespace(transforms=['fixed']))
        cloud.receive(1)
        self.assertEqual(buffer.received, [])
        executor = self.executor(node)
        for _ in range(3):
            executor.spin_once(timeout_sec=0.)
        self.assertEqual(buffer.received, [('dynamic', 'moving', 'default_authority'),
                                           ('static', 'fixed', 'default_authority')])
        self.assertEqual(observed, [2])
        self.assertFalse(buffer.debug)
        self.assertEqual(listener.tf_sub.qos_profile.depth, 100)
        self.assertEqual(listener.tf_static_sub.qos_profile.depth, 100)
        self.assertEqual(listener.tf_static_sub.qos_profile.durability,
                         self.qos.DurabilityPolicy.TRANSIENT_LOCAL)

    def test_parameter_file_explicit_and_constructor_precedence(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'params.yaml'
            path.write_text(json.dumps({'/**': {'ros__parameters': {'roundabout.cap': 5.}},
                                        'adapter_test': {'ros__parameters': {'roundabout': {'cap': 6.}}}}))
            self.ros.params = {'~_ros2_parameter_files': str(path), '~roundabout/cap': 7.}
            parameter = self.runtime.parameter.Parameter('roundabout.cap', value=8.)
            node = self.node(parameter_overrides=[parameter])
            self.assertEqual(node.declare_parameter('roundabout.cap', 1.).value, 8.)
            with self.assertRaises(ValueError):
                node.declare_parameter('roundabout.cap', 1.)

    def test_parameter_file_and_server_preserve_types(self):
        self.ros.params['~roundabout/cap'] = 7.
        node = self.node()
        self.assertEqual(node.declare_parameter('roundabout.cap', 1.).value, 7.)
        self.ros.params['~bad'] = 3
        with self.assertRaises(TypeError):
            node.declare_parameter('bad', 1.)

    def test_ros_clock_notification_keeps_jump_before_next_timer_poll(self):
        node = self.node()
        result = []
        timer = node.create_timer(.1, lambda: result.append(self.ros.now_ns))
        executor = self.executor(node)
        self.ros.now_ns = 1_000_000_000
        executor.spin_once(timeout_sec=0.)
        self.ros.now_ns = 100_000_000
        node._on_clock(types.SimpleNamespace(clock=self.ros.Time(nsecs=self.ros.now_ns)))
        self.ros.now_ns = 150_000_000
        executor.spin_once(timeout_sec=0.)
        self.assertEqual(timer.time_until_next_call(), 50_000_000)
        self.assertEqual(result, [1_000_000_000])

    def test_system_clock_backward_jump_does_not_add_ros_time_reset(self):
        self.ros.params['/use_sim_time'] = False
        node = self.node()
        timer = node.create_timer(.1, lambda: None)
        executor = self.executor(node)
        self.ros.now_ns = 1_000_000_000
        executor.spin_once(timeout_sec=0.)
        self.ros.now_ns = 100_000_000
        executor.spin_once(timeout_sec=0.)
        self.assertEqual(timer.time_until_next_call(), 900_000_000)

    def test_native_timestamp_duration_and_latched_publisher(self):
        from ctrl_ros1.ctrl_time import Time
        from ctrl_ros1.ctrl_duration import Duration
        first, second = Time(seconds=2, nanoseconds=9), Time(seconds=1, nanoseconds=10)
        self.assertEqual((first - second).nanoseconds, 999_999_999)
        self.assertEqual(first.seconds_nanoseconds(), (2, 9))
        self.assertEqual(first.to_msg().nsecs, 9)
        self.assertIsInstance(Duration(seconds=.3).to_msg(), self.ros.Duration)
        node = self.node()
        publisher = node.create_publisher(object, '/mode', self.qos.QoSProfile(
            depth=1, durability=self.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self.assertEqual(publisher._publisher.options, {'queue_size': 1, 'latch': True})
        self.assertEqual(self.qos.qos_profile_sensor_data.depth, 5)

    def test_time_and_duration_truncation_negative_values_and_int64_bounds(self):
        from ctrl_ros1.ctrl_time import Time
        from ctrl_ros1.ctrl_duration import Duration
        cases = [(1.2345678919, 2.9), (-.3, -.9), (0., -1.9), (-1.2, 5.4)]
        for seconds, nanoseconds in cases:
            with self.subTest(seconds=seconds, nanoseconds=nanoseconds):
                expected = int(seconds * 10**9) + int(nanoseconds)
                duration = Duration(seconds=seconds, nanoseconds=nanoseconds)
                self.assertEqual(duration.nanoseconds, expected)
                self.assertEqual(duration.to_msg().to_nsec(), expected)
                self.assertEqual((duration.secs, duration.nsecs), divmod(expected, 10**9))
        for seconds, nanoseconds in [(-1, 2_000_000_000), (1, -1)]:
            with self.assertRaises(ValueError):
                Time(seconds=seconds, nanoseconds=nanoseconds)
        self.assertEqual(Time(nanoseconds=2**63 - 1).nanoseconds, 2**63 - 1)
        self.assertEqual(Duration(nanoseconds=-2**63).nanoseconds, -2**63)
        with self.assertRaises(OverflowError):
            Time(nanoseconds=2**63)
        for value in (-2**63 - 1, 2**63):
            with self.assertRaises(OverflowError):
                Duration(nanoseconds=value)


if __name__ == '__main__':
    unittest.main()
