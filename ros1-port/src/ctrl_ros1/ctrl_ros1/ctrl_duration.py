"""Duration with rclpy calculation methods and native ROS1 wire type."""
import rospy


class Duration(rospy.Duration):
    def __init__(self, *, seconds=0, nanoseconds=0):
        total = int(seconds * 1_000_000_000) + int(nanoseconds)
        if not -2**63 <= total < 2**63:
            raise OverflowError('Total nanoseconds exceeds signed 64-bit duration storage')
        secs, nsecs = divmod(total, 1_000_000_000)
        super().__init__(secs, nsecs)

    @property
    def nanoseconds(self):
        return self.to_nsec()

    def to_msg(self):
        return rospy.Duration(self.secs, self.nsecs)

    @classmethod
    def from_msg(cls, msg):
        return cls(nanoseconds=msg.to_nsec())
