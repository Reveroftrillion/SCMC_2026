"""Observed convex obstacle outlines must not fill their empty AABB corners."""
import pytest

from foscar_planning.avoid_planner import Box
from foscar_planning.tunnel_slalom import sweep_hits


def diagonal_box():
    box = Box(0., 5., -1., 4.)
    box.vertices = ((0., 0.), (1., -1.), (5., 3.), (4., 4.))
    return box


def test_empty_diagonal_aabb_corner_is_traversable():
    box = diagonal_box()
    args = (3., 3., 0., 1., .2, .8)
    assert sweep_hits([Box(0., 5., -1., 4.)], *args, front=.2, rear=.2, half=.2)
    assert not sweep_hits([box], *args, front=.2, rear=.2, half=.2)


@pytest.mark.parametrize('vertices_reversed', [False, True])
def test_diagonal_observed_body_still_collides(vertices_reversed):
    box = diagonal_box()
    if vertices_reversed:
        box.vertices = tuple(reversed(box.vertices))
    assert sweep_hits([box], 2., 2., 0., 1., 2., 3., front=.2, rear=.2, half=.2)


def test_polygon_sweep_checks_non_grid_endpoint():
    box = Box(1.29, 1.4, -.1, .1)
    box.vertices = ((1.29, -.1), (1.4, -.1), (1.4, .1), (1.29, .1))
    assert sweep_hits([box], 0., 0., 0., 1., 0., .3, front=1., rear=.2, half=.2)


def test_polygon_sweep_checks_rotated_front_corner():
    box = Box(1.55, 1.6, .23, .28)
    box.vertices = ((1.55, .23), (1.6, .23), (1.6, .28), (1.55, .28))
    assert sweep_hits([box], 0., 4./15., 0., 1., .5, .5, front=1., rear=.2, half=.5)


def test_polygon_sweep_preserves_lateral_body_margin():
    box = diagonal_box()
    assert not sweep_hits([box], 1., 1., 0., 1., 0., 0., front=.2, rear=.2, half=.2)
    assert sweep_hits([box], 1., 1., 0., 1., 0., 0., front=.2, rear=.2, half=.9)


def test_polygon_box_builds_perimeter_from_unordered_observations():
    from foscar_planning.avoid_planner import polygon_box
    box = polygon_box([(5., 3.), (2., 1.), (0., 0.), (4., 4.), (1., -1.), (0., 0.)])
    assert (box.s_min, box.s_max, box.d_min, box.d_max) == (0., 5., -1., 4.)
    assert set(box.vertices) == {(0., 0.), (1., -1.), (5., 3.), (4., 4.)}
    assert not sweep_hits([box], 3., 3., 0., 1., .2, .8, front=.2, rear=.2, half=.2)


def test_polygon_box_pad_preserves_square_padding_margin():
    from foscar_planning.avoid_planner import polygon_box
    box = polygon_box(diagonal_box().vertices, pad=.5)
    assert (box.s_min, box.s_max, box.d_min, box.d_max) == (-.5, 5.5, -1.5, 4.5)
    assert sweep_hits([box], 1., 1., 0., 1., 0., 0., front=.2, rear=.2, half=.2)
    assert not sweep_hits([box], 3., 3., 0., 1., .2, .8, front=.2, rear=.2, half=.2)


def test_degenerate_polygon_falls_back_to_conservative_aabb():
    from foscar_planning.avoid_planner import polygon_box
    box = polygon_box([(0., 0.), (1., 1.), (2., 2.)])
    assert box.vertices == ()
    assert sweep_hits([box], 1.8, 1.8, 0., 1., .2, .2, front=.2, rear=.2, half=.2)


@pytest.mark.parametrize('speed_ok', [None, lambda length, shift, start: True])
def test_polygon_entry_can_finish_beyond_front_search_when_full_body_clears(speed_ok):
    from foscar_planning.avoid_planner import polygon_box
    from foscar_planning.tunnel_slalom import plan
    box = polygon_box([(11.2, .86), (18.5, -1.36), (12.1, 3.74), (10.8, 1.56)], pad=.2)
    assert not sweep_hits([box], 0., 5.5, 0., 14., 0., box.s_max+1., 3.8, 1., 1.246)
    result = plan([[box]], lambda _: [5.5], s_now=0., d_now=0., v=0.,
                  clear_s=lambda b: b.s_max+1., front=3.8, rear=1., half=1.246,
                  min_curve=4., curve_time=2., min_move=2., max_curvature=.18,
                  speed_ok=speed_ok,
                  segment_ok=lambda d0, d1, start, length, s_a, s_b: d1 == 0. or length >= 14.)
    assert result.blocked_at is None
    assert result.starts[0] == 0.
    assert result.lengths[0] >= 14.
    assert result.starts[0]+result.lengths[0] > box.s_min-3.8+4.
    assert not sweep_hits([box], 0., result.offsets[0], result.starts[0], result.lengths[0],
                          0., box.s_max+1., 3.8, 1., 1.246)


def test_extended_polygon_entry_still_rejects_blocked_road():
    from foscar_planning.avoid_planner import polygon_box
    from foscar_planning.tunnel_slalom import plan
    box = polygon_box([(11.2, .86), (18.5, -1.36), (12.1, 3.74), (10.8, 1.56)], pad=.2)
    result = plan([[box]], lambda _: [5.5], s_now=0., d_now=0., v=0.,
                  clear_s=lambda b: b.s_max+1., front=3.8, rear=1., half=1.246,
                  min_curve=4., curve_time=2., min_move=2., max_curvature=.18,
                  segment_ok=lambda *args: False)
    assert result.blocked_at == 0


def test_static_entry_uses_widening_road_on_right_instead_of_constant_offset_corridor():
    import numpy as np
    from pathlib import Path
    from foscar_control.core import Route
    from foscar_planning.lane_prediction import LaneMap
    from foscar_planning.state import Context
    from foscar_planning.states.avoid import Avoid, Remembered
    from foscar_planning.avoid_planner import polygon_box
    from foscar_planning.mapped_avoidance import paved_moves
    route = Route(np.c_[np.arange(61.),np.zeros(61)])
    road = LaneMap([
        dict(idx='own',from_node_idx='a',to_node_idx='b',width_start=3.5,
             points=[[0.,0.],[10.,0.],[60.,0.]],right_lane_change_dst_link_idx='right',
             lane_mark_right=['shared']),
        dict(idx='right',from_node_idx='c',to_node_idx='d',width_start=3.5,
             points=[[0.,0.],[10.,-3.5],[60.,-3.5]],left_lane_change_dst_link_idx='own',
             lane_mark_left=['shared']),
        dict(idx='left',from_node_idx='e',to_node_idx='f',width_start=3.5,
             points=[[0.,3.5],[60.,3.5]])])
    c=Context(s=0.,x=0.,y=0.,yaw=0.,v=0.,now=1.,
              route=route,road_map=road,mission_name='STATIC_OBSTACLE')
    a=Avoid(margin_m=.5)
    b=polygon_box([(9.48,1.23),(17.89,-1.45),(18.85,1.55),(10.43,4.23)])
    a.memory=[Remembered(b,c.now)]
    t=a.obstacle_target(c)
    assert a.slalom.blocked_at is None
    assert t.lat_offset<0.
    assert paved_moves(c,a.slalom_origin,t.chain,c.s,max(st+ln for _,st,ln in t.chain),
                       a.front_m,a.rear_overhang_m,a.static_planner.half)
