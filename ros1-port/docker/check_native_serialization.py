#!/usr/bin/env python3
"""Real generated genpy messages, PointCloud2 wire bytes and native tf2 lookup."""
import io
import json
import math
import struct
import sys

import rospy
import tf2_py
import tf2_ros
from ctrl_interfaces import msg as interfaces
from ctrl_ros1.ctrl_messages import Marker
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def roundtrip(message):
    stream = io.BytesIO()
    message.serialize(stream)
    recovered = type(message)().deserialize(stream.getvalue())
    again = io.BytesIO()
    recovered.serialize(again)
    assert stream.getvalue() == again.getvalue(), message._type
    return recovered, len(stream.getvalue())


def main():
    report = dict(python=sys.version, tf2_py=tf2_py.__file__, messages={}, clouds=[])
    for name in ('ControlCommand', 'VehicleStatus', 'Event', 'DrivingTarget'):
        message = getattr(interfaces, name)()
        for slot, kind in zip(message.__slots__, message._slot_types):
            if kind == 'std_msgs/Header':
                value = Header(seq=42, stamp=rospy.Time(123, 456789012), frame_id='base_link')
            elif kind == 'string':
                value = 'native_' + slot
            elif kind == 'bool':
                value = True
            elif kind == 'float64[]':
                value = [0., .125, -1.25]
            elif kind == 'float64':
                value = .125
            elif kind == 'int8':
                value = 1
            else:
                raise RuntimeError('Unhandled message field: ' + kind)
            setattr(message, slot, value)
        recovered, length = roundtrip(message)
        assert message == recovered, name
        report['messages'][name] = dict(bytes=length, md5=message._md5sum)
    for bigendian in (False, True):
        data = bytearray(b'P' * 128)
        fields = [PointField(name=n, offset=i * 8, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
        for row in range(4):
            for col in range(4):
                struct.pack_into(('>' if bigendian else '<') + 'f', data, row * 32 + col * 8,
                                 math.nan if row == col == 2 else row + col * .25)
        cloud = PointCloud2(header=Header(seq=7, stamp=rospy.Time(3, 17), frame_id='lidar'),
                            width=2, height=2, fields=fields, is_bigendian=bigendian,
                            point_step=32, row_step=64, data=bytes(data), is_dense=False)
        recovered, length = roundtrip(cloud)
        assert bytes(recovered.data) == bytes(data)
        assert recovered.header == cloud.header
        assert (recovered.width, recovered.height, recovered.point_step, recovered.row_step) == (2, 2, 32, 64)
        report['clouds'].append(dict(bigendian=bigendian, bytes=length, organized=True,
                                     nan_intensity_and_padding_preserved=True))
    marker, length = roundtrip(Marker())
    assert marker.pose.orientation.w == 1.
    report['messages']['Marker'] = dict(bytes=length, quaternion_w=1.)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'lidar'
    transform.transform.translation.x = 2.
    transform.transform.translation.z = 2.11
    transform.transform.rotation.w = 1.
    buffer = tf2_ros.Buffer(debug=False)
    buffer.set_transform_static(transform, 'native_verification')
    result = buffer.lookup_transform('base_link', 'lidar', rospy.Time(0))
    assert (result.transform.translation.x, result.transform.translation.z) == (2., 2.11)
    report['native_tf2_lookup'] = True
    report['passed'] = True
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
