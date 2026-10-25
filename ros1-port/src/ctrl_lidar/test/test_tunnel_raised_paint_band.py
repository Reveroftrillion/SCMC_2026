"""A bright raised body is not paint when a supported road plane separates it."""
import json

import numpy as np
import pytest
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

from ctrl_lidar.ctrl_obstacle_node import ObstacleNode


def cloud(rows):
    raw = np.asarray(rows, dtype=np.float32).copy()
    raw[:, :3] -= [2., 0., 2.11]
    fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(['x', 'y', 'z', 'intensity'])]
    return point_cloud2.create_cloud(Header(frame_id='lidar'), fields, raw)


def patch(x, y, z, intensity):
    xx, yy = np.meshgrid(np.linspace(x, x+.3, 10), np.linspace(y, y+1.5, 30))
    return np.c_[xx.ravel(), yy.ravel(), np.full(xx.size, z), np.full(xx.size, intensity)]


def floor(reference):
    xx, yy = np.meshgrid(np.linspace(3.1, 27.9, 60), np.linspace(-4.9, 4.9, 25))
    return np.c_[xx.ravel(), yy.ravel(), np.full(xx.size, reference), np.full(xx.size, 10.)]


@pytest.fixture
def node():
    obj = ObstacleNode(parameter_overrides=[Parameter('use_tf', value=False),
        Parameter('use_route', value=False), Parameter('publish_markers', value=False)])
    obj.on_mission(String(data=json.dumps(dict(mission='TUNNEL'))))
    yield obj
    obj.destroy_node()


@pytest.mark.parametrize('reference', [0., .53])
def test_supported_plane_keeps_raised_bright_body_but_removes_road_paint(node, reference):
    msg = cloud(np.vstack([floor(reference), patch(12., -2., reference+.06, 54.),
                           patch(20., -.5, reference+.35, 54.)]))
    before = point_cloud2.read_points_numpy(msg).copy()
    dets = node.detect(msg, 1.)
    assert len(dets) == 1
    assert abs(dets[0]['x']-20.15) < .1
    assert .34 < dets[0]['z_max'] < .36
    np.testing.assert_array_equal(point_cloud2.read_points_numpy(msg), before)


def test_no_plane_retains_existing_low_paint_filter(node):
    # A compact raised-looking bright patch cannot establish a road plane.
    assert node.detect(cloud(patch(20., -.5, .35, 54.)), 1.) == []


def test_outside_supported_plane_retains_existing_low_paint_filter(node):
    # Broad near support does not authorize its extrapolation past x28.
    msg = cloud(np.vstack([floor(0.), patch(30., -.5, .35, 54.)]))
    assert node.detect(msg, 1.) == []
