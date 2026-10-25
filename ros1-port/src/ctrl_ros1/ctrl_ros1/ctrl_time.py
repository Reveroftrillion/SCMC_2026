"""Time with rclpy calculation methods and native ROS1 wire type."""
import rospy
from .ctrl_duration import Duration


class Time(rospy.Time):
    def __init__(self, *, seconds=0, nanoseconds=0):
        if seconds < 0 or nanoseconds < 0:
            raise ValueError('Time seconds and nanoseconds must not be negative')
        total = int(seconds * 1_000_000_000) + int(nanoseconds)
        if total >= 2**63:
            raise OverflowError('Total nanoseconds exceeds signed 64-bit time storage')
        secs, nsecs = divmod(total, 1_000_000_000)
        super().__init__(secs, nsecs)

    @property
    def nanoseconds(self):
        return self.to_nsec()

    def seconds_nanoseconds(self):
        return self.secs, self.nsecs

    def to_msg(self):
        return rospy.Time(self.secs, self.nsecs)

    @classmethod
    def from_msg(cls, msg):
        return cls(nanoseconds=msg.to_nsec())

    def __sub__(self, other):
        difference = self.to_nsec() - other.to_nsec()
        if isinstance(other, rospy.Time):
            return Duration(nanoseconds=difference)
        return Time(nanoseconds=difference)

    def __add__(self, other):
        if not isinstance(other, rospy.Duration):
            return NotImplemented
        return Time(nanoseconds=self.to_nsec() + other.to_nsec())
