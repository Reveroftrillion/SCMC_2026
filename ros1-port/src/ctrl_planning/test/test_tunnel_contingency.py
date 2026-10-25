"""Occupied interiors must not disappear when polygon vertices miss the road."""
import math

import pytest

from ctrl_planning.ctrl_lane_prediction import LaneMap
from ctrl_planning.states.ctrl_avoid import Avoid
from test_tunnel_avoidance import context


def scan_context(hull):
    ctx = context()
    ctx.events = []
    ctx.v = 0.
    ctx.occupancy_stamp = ctx.now
    ctx.occupancy_sensor = (ctx.x, ctx.y)
    ctx.occupancy_hulls = {'wall': hull}
    ctx.occupancy_heights = {'wall': 1.02}
    ctx.occupancy_kinds = {'wall': 'OBSTACLE'}
    return ctx


@pytest.mark.parametrize('mission', ['TUNNEL', 'STATIC_OBSTACLE'])
def test_road_spanning_polygon_with_all_corners_off_road_stops(mission):
    ctx = scan_context(((38., -10.), (41., -10.), (41., 10.), (38., 10.)))
    ctx.mission_name = mission
    assert not ctx.road_map.contains_road(ctx.occupancy_hulls['wall']).any()
    avoid = Avoid()
    assert avoid.check_entry(ctx)
    assert avoid.boxes()
    target = avoid.compute_target(ctx)
    assert target.v_max == 0.
    assert math.isfinite(target.stop_s)


def test_missing_map_preserves_observed_blocker_and_stops():
    ctx = scan_context(((38., -1.), (41., -1.), (41., 1.), (38., 1.)))
    ctx.road_map = None
    avoid = Avoid()
    assert avoid.check_entry(ctx)
    target = avoid.compute_target(ctx)
    assert target.v_max == 0. and math.isfinite(target.stop_s)


@pytest.mark.parametrize('low,high', [(5.05, 6.), (7., 9.)])
def test_shallow_roadside_sliver_and_wholly_off_road_remain_excluded(low, high):
    # The mapped union is [-1.75, 5.25]; a <=0.2 m roadside sliver stays excluded.
    ctx = scan_context(((38., low), (41., low), (41., high), (38., high)))
    avoid = Avoid()
    assert not avoid.check_entry(ctx)
    assert not avoid.boxes()


def test_interior_sampling_does_not_fill_an_unmapped_median():
    ctx = scan_context(((38., 2.2), (41., 2.2), (41., 5.8), (38., 5.8)))
    ctx.road_map = LaneMap([
        dict(idx=str(i), from_node_idx=str(i), to_node_idx=str(i+1), width_start=3.5,
             points=[[x, d, 0.] for x in range(181)]) for i, d in enumerate([0., 8.])])
    avoid = Avoid()
    assert not avoid.check_entry(ctx)
    assert not avoid.boxes()
