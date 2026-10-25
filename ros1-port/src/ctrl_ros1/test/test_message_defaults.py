"""Check a verified default override without changing native wire metadata."""
import importlib.util
import io
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest.mock import patch

PACKAGE = Path(__file__).resolve().parents[1]


def load_adapter():
    spec = importlib.util.spec_from_file_location('_marker_defaults', PACKAGE / 'ctrl_ros1/ctrl_messages.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Marker


class NativeMarkerDouble:
    __slots__ = ('header', 'ns', 'id', 'type', 'action', 'pose')
    _type = 'visualization_msgs/Marker'
    _md5sum = 'test-double-wire-schema'
    DELETEALL = 3

    def __init__(self, *args, **kwargs):
        defaults = [None, '', 0, 0, 0, types.SimpleNamespace(
            orientation=types.SimpleNamespace(x=0., y=0., z=0., w=0.))]
        values = list(args) if args else [kwargs.get(key, value)
                                         for key, value in zip(self.__slots__, defaults)]
        for key, value in zip(self.__slots__, values):
            setattr(self, key, value)

    def serialize(self, stream):
        orientation = self.pose.orientation
        stream.write(struct.pack('<ii4d', self.id, self.action, orientation.x,
                                 orientation.y, orientation.z, orientation.w))


class MarkerDefaultsTest(unittest.TestCase):
    def setUp(self):
        message_module = types.ModuleType('visualization_msgs.msg')
        message_module.Marker = NativeMarkerDouble
        self.modules = patch.dict(sys.modules, {'visualization_msgs.msg': message_module})
        self.modules.start()
        self.Marker = load_adapter()

    def tearDown(self):
        self.modules.stop()

    def test_default_deleteall_identity_quaternion_inherited_wire_and_metadata(self):
        marker = self.Marker(action=self.Marker.DELETEALL)
        self.assertIsInstance(marker, NativeMarkerDouble)
        self.assertEqual(marker.pose.orientation.w, 1.)
        self.assertEqual((marker.pose.orientation.x, marker.pose.orientation.y,
                          marker.pose.orientation.z), (0., 0., 0.))
        self.assertEqual(marker._type, NativeMarkerDouble._type)
        self.assertEqual(marker._md5sum, NativeMarkerDouble._md5sum)
        self.assertIs(self.Marker.serialize, NativeMarkerDouble.serialize)
        # genpy.Message.__init__ uses self.__slots__ to assign args/kwargs;
        # the subclass must retain the native field list through inheritance.
        self.assertIs(self.Marker.__slots__, NativeMarkerDouble.__slots__)
        stream = io.BytesIO()
        marker.serialize(stream)
        self.assertEqual(struct.unpack('<ii4d', stream.getvalue()), (0, 3, 0., 0., 0., 1.))
        self.assertEqual((marker.header, marker.ns, marker.id, marker.type), (None, '', 0, 0))

    def test_explicit_keyword_and_positional_pose_preserved(self):
        pose = types.SimpleNamespace(orientation=types.SimpleNamespace(x=.1, y=.2, z=.3, w=0.))
        keyword = self.Marker(pose=pose, id=9)
        positional = self.Marker(None, 'given', 7, 1, 0, pose)
        self.assertIs(keyword.pose, pose)
        self.assertIs(positional.pose, pose)
        self.assertEqual(pose.orientation.w, 0.)
        self.assertEqual(keyword.id, 9)
        self.assertEqual((positional.ns, positional.id), ('given', 7))


try:
    from visualization_msgs.msg import Marker as NativeMarker
    from geometry_msgs.msg import Pose
except ImportError:
    NativeMarker = None


@unittest.skipIf(NativeMarker is None, 'Native Noetic messages required; run on the ROS1 target')
class NativeMarkerSerializationTest(unittest.TestCase):
    def test_full_native_serialization_and_explicit_pose(self):
        Marker = load_adapter()
        marker = Marker(action=Marker.DELETEALL)
        expected = NativeMarker(action=NativeMarker.DELETEALL)
        expected.pose.orientation.w = 1.
        self.assertIsInstance(marker, NativeMarker)
        actual_wire, expected_wire = io.BytesIO(), io.BytesIO()
        marker.serialize(actual_wire)
        expected.serialize(expected_wire)
        self.assertEqual(actual_wire.getvalue(), expected_wire.getvalue())
        received = NativeMarker().deserialize(actual_wire.getvalue())
        self.assertEqual(received.pose.orientation.w, 1.)
        self.assertEqual(received.action, NativeMarker.DELETEALL)
        pose = Pose()
        pose.orientation.w = 0.
        self.assertIs(Marker(pose=pose).pose, pose)
        self.assertEqual(pose.orientation.w, 0.)


if __name__ == '__main__':
    unittest.main()
