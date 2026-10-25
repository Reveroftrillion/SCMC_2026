"""Observed road occupancy must not disappear behind semantic confirmation."""
import json
from pathlib import Path

import numpy as np
import pytest
from rclpy.parameter import Parameter
from std_msgs.msg import String

from ctrl_control.ctrl_core import Route
from ctrl_lidar.ctrl_obstacle_node import ObstacleNode

PORT = Path(__file__).resolve().parents[3]


@pytest.fixture
def node():
    obj = ObstacleNode(parameter_overrides=[Parameter('use_tf', value=False),
        Parameter('use_route', value=False), Parameter('publish_markers', value=False),
        Parameter('link_set', value=str(PORT/'src/ctrl_bringup/maps/link_set.json'))])
    route = Route(np.loadtxt(PORT/'src/ctrl_bringup/routes/global_path.txt'))
    x, y = route.at(1880.)
    obj.ego = (x, y, route.heading_at(1880.))
    obj.ego_s = 1880.
    yield obj
    obj.destroy_node()


def detection(y=1., structure=False):
    return dict(x=24., y=y, size_x=.3, size_y=1.5, size_z=.01, z_max=.35, n=30,
                footprint=[[23.85, y-.75], [24.15, y-.75], [24.15, y+.75], [23.85, y+.75]],
                structure=structure)


def feed(node, mission, detections, time=10., expired=False):
    node.on_mission(String(data=json.dumps(dict(mission=mission))))
    if expired:
        node.mission_received_at -= 2.
    node.odom_buf.append((time, *node.ego))
    node.track_and_publish(detections, node.get_clock().now().to_msg(), time)
    return json.loads(node.footprint_pub.values[-1].data)['objects']


def test_first_tunnel_outline_is_observed_without_a_confirmed_semantic_event(node):
    observed = feed(node, 'TUNNEL', [detection()])
    assert len(observed) == 1
    assert observed[0]['kind'] == 'OBSTACLE'
    assert node.event_pub.values == []
    assert node.tracks[0].hits == 1 and node.confirm_frames == 3
    # A missing scan cannot turn the provisional observation into unseen occupancy.
    assert feed(node, 'TUNNEL', [], 10.1) == []
    assert node.event_pub.values == []


@pytest.mark.parametrize('mission,expired', [('CRUISE', False), ('TUNNEL', True)])
def test_first_outline_stays_gated_outside_fresh_tunnel_ownership(node, mission, expired):
    assert feed(node, mission, [detection()], expired=expired) == []
    assert node.event_pub.values == []


@pytest.mark.parametrize('det', [detection(y=100.), detection(structure=True)])
def test_first_tunnel_outline_keeps_road_and_structure_exclusions(node, det):
    assert feed(node, 'TUNNEL', [det.copy()]) == []
    assert node.event_pub.values == []
