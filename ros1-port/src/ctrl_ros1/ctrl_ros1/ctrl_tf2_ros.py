"""Keep TF ingress on the obstacle node's original executor.

Native rospy TransformListener executes buffer writes on transport threads.
The Humble listener instead uses node subscriptions in a reentrant group;
obstacle_node spins that node in its single-threaded executor. Recreate that
boundary while using Noetic's native TF buffer and native wire messages.
"""
import tf2_ros as _native
from tf2_msgs.msg import TFMessage

from .ctrl_callback_groups import ReentrantCallbackGroup
from .ctrl_qos import DurabilityPolicy, QoSProfile


class Buffer(_native.Buffer):
    def __init__(self, cache_time=None):
        # Humble Buffer(cache_time=..., node=None) does not create a frame-graph
        # service. Disable Noetic's default debug service to retain that scope.
        super().__init__(cache_time=cache_time, debug=False)


class TransformListener:
    def __init__(self, buffer, node, *, qos=None, static_qos=None):
        self.buffer = buffer
        self.node = node
        self.group = ReentrantCallbackGroup()
        self.tf_sub = node.create_subscription(
            TFMessage, '/tf', self.callback, qos or QoSProfile(depth=100),
            callback_group=self.group)
        self.tf_static_sub = node.create_subscription(
            TFMessage, '/tf_static', self.static_callback,
            static_qos or QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.group)

    def callback(self, data):
        for transform in data.transforms:
            self.buffer.set_transform(transform, 'default_authority')

    def static_callback(self, data):
        for transform in data.transforms:
            self.buffer.set_transform_static(transform, 'default_authority')

    def unregister(self):
        self.node.destroy_subscription(self.tf_sub)
        self.node.destroy_subscription(self.tf_static_sub)
