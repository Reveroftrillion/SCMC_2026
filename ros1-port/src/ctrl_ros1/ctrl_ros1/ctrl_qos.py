"""The QoS settings used by the source, with explicit ROS1 mappings.

Depth applies at the transport and executor queues. TRANSIENT_LOCAL publishers
are latched. TCPROS cannot implement DDS BEST_EFFORT delivery; see README.md.
"""
from enum import IntEnum


class DurabilityPolicy(IntEnum):
    TRANSIENT_LOCAL = 1
    VOLATILE = 2


class ReliabilityPolicy(IntEnum):
    RELIABLE = 1
    BEST_EFFORT = 2


class HistoryPolicy(IntEnum):
    KEEP_LAST = 1
    KEEP_ALL = 2


class QoSProfile:
    def __init__(self, *, depth, durability=DurabilityPolicy.VOLATILE,
                 reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST):
        if history != HistoryPolicy.KEEP_LAST or depth < 1:
            raise ValueError('Only positive-depth KEEP_LAST is used by this port')
        self.depth = int(depth)
        self.durability = DurabilityPolicy(durability)
        self.reliability = ReliabilityPolicy(reliability)
        self.history = HistoryPolicy(history)


qos_profile_sensor_data = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


def profile(value):
    return QoSProfile(depth=value) if isinstance(value, int) else value
