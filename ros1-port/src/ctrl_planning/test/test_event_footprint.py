import json

import pytest
import rclpy
from ctrl_interfaces.msg import Event
from std_msgs.msg import String

from ctrl_planning.ctrl_behavior_node import BehaviorNode


HULL = ((10., 1.), (12., 1.), (12., 2.), (10., 2.))
TRACK = 'lidar:test:7'


@pytest.fixture
def node():
    rclpy.init()
    node = BehaviorNode()
    node.now = lambda: 10.
    yield node
    node.destroy_node()
    rclpy.shutdown()


def event(node, stamp=10., track=TRACK):
    msg = Event()
    msg.type = 'OBSTACLE'
    msg.value = track
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    node.on_event(msg)
    return node.events[('track', track)]


def footprints(node, stamp=10., track=TRACK, xy=HULL, **extra):
    data = dict(stamp=stamp, frame='map', objects=[dict(id=track, xy=xy)])
    data.update(extra)
    node.on_footprints(String(data=json.dumps(data)))


@pytest.mark.parametrize('footprint_first', [True, False])
def test_same_scan_polygon_attaches_in_either_callback_order(node, footprint_first):
    if footprint_first:
        footprints(node, stamp=10.+1e-8)
    info = event(node)
    if not footprint_first:
        footprints(node, stamp=10.+1e-8)
    assert info.footprint == HULL
    assert isinstance(info.footprint, tuple)
    assert all(isinstance(point, tuple) for point in info.footprint)


@pytest.mark.parametrize('stamp,track', [(9.9, TRACK), (10.02, TRACK), (10., 'lidar:other:7')])
def test_different_scan_or_identity_never_attaches(node, stamp, track):
    footprints(node, stamp=stamp, track=track)
    assert event(node).footprint == ()
    footprints(node, stamp=stamp, track=track)
    assert event(node).footprint == ()


@pytest.mark.parametrize('extra', [dict(frame='base_link'), dict(scan_valid=False),
    dict(stamp=float('nan')), dict(stamp=float('inf')), dict(stamp=11.),
    dict(objects=None), dict(objects=[None]), dict(objects=[dict(id=TRACK, xy=[[1., 2.]])]),
    dict(objects=[dict(id=TRACK, xy=[[1., 2.], [2., 3.], [float('nan'), 4.]])]),
    dict(objects=[dict(id=TRACK, xy=[[1., 2.], [2., 3.], [3., 4.]])]),
    dict(objects=[dict(id=TRACK, xy=[[1., 2., 3.], [2., 3.], [4., 1.]])])])
def test_invalid_footprints_are_not_attached(node, extra):
    footprints(node, **extra)
    assert event(node).footprint == ()


def test_latest_scan_replaces_cache_and_ignores_older_packet(node):
    footprints(node, stamp=9.9)
    footprints(node, stamp=10., track='lidar:test:8')
    footprints(node, stamp=9.9)
    assert event(node, stamp=9.9).footprint == ()
    assert event(node, track='lidar:test:8').footprint == HULL


def test_new_event_does_not_inherit_previous_scan_polygon(node):
    footprints(node, stamp=9.9)
    assert event(node, stamp=9.9).footprint == HULL
    assert event(node).footprint == ()


def test_observation_reset_clears_polygon_cache(node):
    footprints(node)
    node.reset_mission_runtime(clear_observations=True)
    assert event(node).footprint == ()
    footprints(node, stamp=9.9)
    assert event(node).footprint == ()
