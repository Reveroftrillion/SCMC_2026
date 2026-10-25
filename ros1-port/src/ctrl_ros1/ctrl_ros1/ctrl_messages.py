"""Verified generated-message default differences at the ROS1 boundary.

Humble Quaternion.msg defaults w to 1; Noetic's generated Quaternion defaults
all four fields to 0. A Marker constructed without a pose therefore needs its
nested quaternion default restored, including DELETEALL markers. Explicit
poses retain the caller's values. Schema metadata and serialization remain
those of the native Noetic Marker class.
"""
from visualization_msgs.msg import Marker as _NativeMarker

_POSE_INDEX = _NativeMarker.__slots__.index('pose')


class Marker(_NativeMarker):
    # Retain inherited __slots__: genpy.Message uses that field list when
    # initializing native positional/keyword arguments.
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if len(args) <= _POSE_INDEX and 'pose' not in kwargs:
            self.pose.orientation.w = 1.0
