"""Geometry and complete-route regressions for the ROS-independent tunnel planner."""
from dataclasses import dataclass

from ctrl_planning.ctrl_tunnel_slalom import plan, representatives, sweep_hits


@dataclass
class Box:
    s_min: float
    s_max: float
    d_min: float
    d_max: float


def make_plan(groups, feasible, **kwargs):
    return plan(groups, feasible, s_now=0., d_now=0., v=4., clear_s=lambda b: b.s_max + 1.,
                front=2., rear=1., half=.5, min_curve=4., curve_time=2., min_move=2., **kwargs)


def test_sweep_includes_non_grid_endpoint():
    assert sweep_hits([Box(1.29, 1.4, -.1, .1)], 0., 0., 0., 1., 0., .3,
                      front=1., rear=.2, half=.2)


def test_rotated_front_corner_is_not_culled_by_unrotated_front():
    # At u=.5, tan(yaw)=.5; the front-right corner reaches s=1.618 > s+front=1.5.
    assert sweep_hits([Box(1.55, 1.6, .23, .28)], 0., 4./15., 0., 1., .5, .5,
                      front=1., rear=.2, half=.5)


def test_representatives_keep_each_open_gap():
    boxes = [Box(20., 21., -3., -2.), Box(20., 21., 0., 1.), Box(20., 21., 3., 4.)]
    offsets = [-4., -3.75, -1.25, -1., 1.75, 2., 5., 5.25]
    candidates = representatives(offsets, boxes, [])
    assert any(d < -3. for d in candidates)
    assert any(-2. < d < 0. for d in candidates)
    assert any(1. < d < 3. for d in candidates)
    assert any(d > 4. for d in candidates)


def test_fresh_empty_scan_does_not_make_remembered_collision_safe():
    result = make_plan([[Box(4., 5., .4, 1.6)]], lambda _: [],
                       ongoing=(0., 2., -2., 10.), ongoing_boxes=[])
    assert result.blocked_at == 0
    assert not result.offsets


def test_road_rejection_checks_candidates_before_choice():
    # Both sides clear the obstacle, but pavement permits only the negative side.
    result = make_plan([[Box(20., 21., -.5, .5)]], lambda _: [-2., 1.],
                       segment_ok=lambda d0, d1, start, length, s_a, s_b: max(d0, d1) <= 0.)
    assert result.blocked_at is None
    assert result.offsets[0] == -2.
    assert result.offsets[-1] == 0.


def test_return_feasibility_is_considered_before_choosing_side():
    # The cheaper left entry cannot return across the narrowing road after the obstacle.
    result = make_plan([[Box(20., 21., -.5, .5)]], lambda _: [-2., 1.],
                       segment_ok=lambda d0, d1, start, length, s_a, s_b: not (d0 > 0. and d1 == 0.))
    assert result.blocked_at is None
    assert result.offsets[0] == -2.
    assert result.offsets[-1] == 0.


def test_first_hit_uses_move_starting_inside_sample_interval():
    from ctrl_planning.ctrl_tunnel_slalom import first_hit
    hit = first_hit([Box(.79, .85, 1.9, 2.1)], [(2., .2, .2)], 0., 0., 1.,
                    front=.1, rear=.1, half=.1)
    assert hit is not None


def test_first_hit_does_not_check_past_requested_horizon():
    from ctrl_planning.ctrl_tunnel_slalom import first_hit
    assert first_hit([Box(.9, 1., -.1, .1)], [], 0., 0., .5,
                     front=.1, rear=.1, half=.1) is None


def test_entry_uses_available_distance_when_short_curve_leaves_road():
    result = make_plan([[Box(20., 21., -.5, .5)]], lambda _: [2.],
                       segment_ok=lambda d0, d1, start, length, s_a, s_b: d1 == 0. or length >= 10.)
    assert result.blocked_at is None
    assert result.lengths[0] >= 10.


def test_no_safe_return_is_reported_while_retaining_safe_passage():
    result = make_plan([[Box(20., 21., -.5, .5)]], lambda _: [2.],
                       segment_ok=lambda d0, d1, start, length, s_a, s_b: d1 != 0.)
    assert result.blocked_at is None
    assert result.offsets == [2.]
    assert result.return_blocked


def test_return_can_use_the_length_that_made_entry_fit_the_road():
    result = make_plan([[Box(20., 21., -.5, .5)]], lambda _: [2.],
                       segment_ok=lambda d0, d1, start, length, s_a, s_b: length >= 10.)
    assert result.offsets[-1] == 0.
    assert not result.return_blocked
    assert result.lengths[-1] >= 10.


def test_interval_interior_is_considered_when_edges_fail_the_full_path():
    result = make_plan([[Box(20., 21., -.5, .5)]], lambda _: [1., 1.5, 2., 2.5, 3.],
                       segment_ok=lambda d0, d1, start, length, s_a, s_b: d1 == 0. or d1 == 2.)
    assert result.blocked_at is None
    assert result.offsets[0] == 2.


def test_return_length_policy_is_used_for_each_candidate():
    result = make_plan([[Box(20., 21., -.5, .5)]], lambda _: [2.],
                       segment_ok=lambda d0, d1, start, length, s_a, s_b: d1 != 0. or length >= 20.,
                       return_length=lambda d: 10. * abs(d))
    assert result.offsets[-1] == 0.
    assert result.lengths[-1] >= 20.


def test_entry_search_includes_exact_shortest_length():
    result = make_plan([[Box(20., 21., -.5, .5)]], lambda _: [2.],
                       max_curvature=5.77*2./2.5**2,
                       segment_ok=lambda d0, d1, start, length, s_a, s_b: d1 == 0. or abs(length-2.5) < 1e-9)
    assert result.blocked_at is None
    assert result.lengths[0] == 2.5


def test_return_start_search_includes_exact_horizon():
    result = make_plan([[Box(20., 21., -.5, .5)]], lambda _: [2.],
                       segment_ok=lambda d0, d1, start, length, s_a, s_b: (length <= 16.5 if d1 != 0. else start >= 24.))
    assert not result.return_blocked
    assert result.offsets[-1] == 0.
    assert result.starts[-1] == 24.


def test_return_search_includes_exact_shortest_length():
    result = make_plan([[Box(20., 21., -.5, .5)]], lambda _: [2.],
                       max_curvature=5.77*2./2.5**2,
                       segment_ok=lambda d0, d1, start, length, s_a, s_b: d1 != 0. or abs(length-2.5) < 1e-9)
    assert not result.return_blocked
    assert result.offsets[-1] == 0.
    assert result.lengths[-1] == 2.5


def test_return_cannot_finish_before_still_ahead_obstacle():
    box = Box(30., 32., -.5, .5)
    result = plan([[box]], lambda _: [2.], 0., 2., 4., lambda b: b.s_max+1.,
                  2., 1., .5, 4., 2., 2., return_length=lambda _: 20.)
    from ctrl_planning.ctrl_tunnel_slalom import first_hit
    moves = list(zip(result.offsets, result.starts, result.lengths))
    assert first_hit([box], moves, 2., 0., 40., 2., 1., .5) is None
    assert result.starts[-1] + result.lengths[-1] > box.s_max


def test_safe_connection_skips_intermediate_centre_visit():
    groups = [[Box(10.,11.,3.77,5.91)], [Box(30.,32.,-1.47,.98)]]
    result = plan(groups, lambda g: [0.] if g is groups[0] else [3.],
                  0., -.16, 3., lambda b:b.s_max+1., 2.,1.,.5,4.,2.,2.,
                  return_length=lambda _:26.)
    assert result.offsets == [3., 0.]
    from ctrl_planning.ctrl_tunnel_slalom import first_hit
    assert first_hit([b for g in groups for b in g],
                     list(zip(result.offsets,result.starts,result.lengths)),
                     -.16,0.,60.,2.,1.,.5) is None


def test_centre_visit_is_kept_when_direct_connector_leaves_road():
    groups = [[Box(10.,11.,3.77,5.91)], [Box(30.,32.,-1.47,.98)]]
    result = plan(groups, lambda g: [0.] if g is groups[0] else [3.],
                  0., -.16, 3., lambda b:b.s_max+1., 2.,1.,.5,4.,2.,2.,
                  return_length=lambda _:26.,
                  segment_ok=lambda d0,d1,*args: not (d0<0. and d1>0.))
    assert result.offsets == [0.,3.,0.]
