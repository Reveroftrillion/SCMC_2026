"""Distant BEV furniture must not create extra avoidance planning stages."""
import math

import numpy as np

from ctrl_control.ctrl_core import Route
from ctrl_planning.ctrl_avoid_planner import Box
from ctrl_planning.ctrl_lane_prediction import LaneMap
from ctrl_planning.ctrl_state import Context
from ctrl_planning.states.ctrl_avoid import Avoid, Remembered


def context():
    route = Route(np.c_[np.arange(101.), np.zeros(101)])
    road = LaneMap([dict(idx='road', from_node_idx='a', to_node_idx='b',
                        points=[[0., 0.], [100., 0.]], width_start=30., width_end=30.)])
    return Context(s=10., x=10., y=0., yaw=0., v=0., now=1., route=route,
                   road_map=road, mission_name='STATIC_OBSTACLE')


def test_unreachable_furniture_is_retained_as_evidence_but_not_a_stage():
    avoid = Avoid()
    radius = math.hypot(max(avoid.front_m, avoid.rear_overhang_m), avoid.static_planner.half)
    reach = max(abs(d) for d in avoid.static_planner.candidates()) + radius
    far = [Box(20., 21., reach + 1., reach + 2.),
           Box(30., 31., -reach - 2., -reach - 1.)]
    avoid.memory = [Remembered(b, 1.) for b in far]
    avoid.obstacle_target(context())
    assert avoid.slalom.groups == []
    assert avoid.boxes() == far


def test_contact_at_reachable_body_boundary_is_kept():
    avoid = Avoid()
    radius = math.hypot(max(avoid.front_m, avoid.rear_overhang_m), avoid.static_planner.half)
    reach = max(abs(d) for d in avoid.static_planner.candidates()) + radius
    boundary = Box(20., 21., reach, reach + 1.)
    avoid.memory = [Remembered(boundary, 1.)]
    avoid.obstacle_target(context())
    assert any(boundary in group for group in avoid.slalom.groups)


def test_distant_furniture_does_not_add_stages_or_change_the_entry():
    near = Box(25., 27., -1., 1.)
    baseline, with_furniture = Avoid(), Avoid()
    baseline.memory = [Remembered(near, 1.)]
    with_furniture.memory = [Remembered(b, 1.) for b in
                            [Box(20., 21., -21., -20.), near,
                             Box(30., 31., 20., 21.)]]
    expected = baseline.obstacle_target(context())
    actual = with_furniture.obstacle_target(context())
    assert actual.chain[0] == expected.chain[0]
    assert actual.stop_s == expected.stop_s or (math.isnan(actual.stop_s) and math.isnan(expected.stop_s))
    assert with_furniture.slalom.groups == baseline.slalom.groups
    assert len(with_furniture.memory) == 3
    # Return length still uses the preserved evidence. Validate its whole chain
    # against all boxes, including those omitted from planning stages.
    from ctrl_planning.ctrl_tunnel_slalom import first_hit
    end = max(start + length for _, start, length in actual.chain)
    assert first_hit(with_furniture.boxes(), actual.chain, with_furniture.slalom_origin,
                     10., end, with_furniture.front_m, with_furniture.rear_overhang_m,
                     with_furniture.static_planner.half) is None
