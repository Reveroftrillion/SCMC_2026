from pathlib import Path
import math

import numpy as np
import pytest

from ctrl_control.ctrl_core import Route
from ctrl_control.ctrl_route_start import start_station_hint
from ctrl_planning.ctrl_behavior_node import BehaviorNode
from ctrl_planning.ctrl_frenet import s_offset_from

BRINGUP = Path(__file__).resolve().parents[2] / 'ctrl_bringup'
CHECKPOINTS = BRINGUP / 'routes/checkpoints.txt'
LIVE_XY = (-132.53059690760048, -428.99823475620883)


def route():
    return Route(np.loadtxt(BRINGUP / 'routes/global_path.txt'))


def test_recorded_start_projects_to_end_without_explicit_hint():
    r = route()
    assert r.project(*LIVE_XY)[0] > r.s[-1]-2.
    assert start_station_hint(r, *LIVE_XY, False, CHECKPOINTS) is None
    hint = start_station_hint(r, *LIVE_XY, True, CHECKPOINTS)
    assert hint == 0.
    assert r.project(*LIVE_XY, previous=hint)[0] == 0.


def test_hint_tracks_first_lap_without_wrapping_to_the_end():
    r = route()
    previous = start_station_hint(r, *LIVE_XY, True, CHECKPOINTS)
    for station in np.arange(0., 41., .5):
        previous, distance, _ = r.project(*r.at(station), previous=previous)
        assert abs(previous-station) < 1.e-6
        assert distance < 1.e-6


@pytest.mark.parametrize('xy', [(math.nan, 0.), (0., math.inf), (-60., -142.)])
def test_hint_rejects_invalid_or_non_start_pose(xy):
    with pytest.raises(ValueError):
        start_station_hint(route(), *xy, True, CHECKPOINTS)


def test_hint_rejects_open_connector_route():
    with pytest.raises(ValueError):
        start_station_hint(Route([[0., 0.], [10., 0.]]), 0., 0., True, CHECKPOINTS)


def test_behavior_uses_same_global_origin_with_finish_runout():
    head = route()
    extra = head.points[(head.s > .5) & (head.s <= 60.)]
    r = Route(np.vstack([head.points, extra]))
    assert s_offset_from(r, *LIVE_XY, 0.) < -50.
    node = object.__new__(BehaviorNode)
    node.route, node.s_global, node.s_offset = r, 0., None
    node.start_at_beginning = True
    node.get_logger = lambda: type('Logger', (), {'info': lambda *args: None})()
    assert node.ego_local_s(*LIVE_XY) == 0.
    assert node.s_offset == 0.


def test_default_behavior_still_uses_existing_offset_resolution():
    r = route()
    node = object.__new__(BehaviorNode)
    node.route, node.s_global, node.s_offset = r, 500., None
    node.get_logger = lambda: type('Logger', (), {'info': lambda *args: None})()
    xy = r.at(500.)
    assert node.ego_local_s(*xy) == pytest.approx(500.)
