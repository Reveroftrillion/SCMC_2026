"""A single observed outline must not cancel an adopted curve at zero shift.

Native left-box repeat2: first outline, four empty scans, same outline again;
the early AVOID curve was cancelled before its lateral displacement exceeded .05m.
No extra obstacle memory or relaxed collision/scan gates is requested here.
"""
import math

from ctrl_planning.states.ctrl_avoid import Avoid
from test_bev_avoidance import bev_context


def committed():
    ctx = bev_context()
    ctx.mission_name = 'TUNNEL'
    avoid = Avoid(tunnel_extra_margin_m=0.)
    assert avoid.check_entry(ctx)
    target = avoid.compute_target(ctx)
    assert target.chain and target.v_max > 0.
    return ctx, avoid, target


def empty_scan(ctx):
    ctx.s += .2
    ctx.x = ctx.s
    ctx.now += .1
    ctx.occupancy_stamp = ctx.now
    ctx.occupancy_hulls = {}
    ctx.events = []


def test_initial_dropout_cannot_exit_just_before_the_curve_starts():
    ctx, avoid, target = committed()
    empty_scan(ctx)
    assert abs(avoid.offset_now(ctx.s)) < .05
    assert not avoid.check_exit(ctx)
    assert not avoid.memory and not avoid.bev.remembered


def test_empty_scan_retains_adopted_entry_return_and_speed_cap():
    ctx, avoid, target = committed()
    control_ctx, control, _ = committed()
    empty_scan(ctx)
    empty_scan(control_ctx)
    control_ctx.occupancy_hulls = {'piece': ((38., -1.), (41., -1.), (41., 1.), (38., 1.))}
    retained = avoid.compute_target(ctx)
    continuous = control.compute_target(control_ctx)
    assert retained.chain == target.chain
    # Compare against continuous visibility: the existing held-speed policy
    # allows a gradual rise even with an unchanged outline (5.0 -> 5.03 here).
    assert retained.v_max <= continuous.v_max + 1e-9
    assert not avoid.memory and not avoid.bev.remembered


def test_transient_outline_does_not_keep_the_state_after_the_return_finishes():
    ctx, avoid, target = committed()
    empty_scan(ctx)
    ctx.s = max(start+length for _, start, length in target.chain)+.1
    ctx.x = ctx.s
    assert avoid.check_exit(ctx)


def test_stale_input_still_stops_a_retained_curve():
    ctx, avoid, target = committed()
    empty_scan(ctx)
    avoid.compute_target(ctx)
    ctx.now += 1.
    assert avoid.compute_target(ctx).v_max == 0.
    assert avoid.bev.last_reason == 'stale scan'


def test_new_blocker_still_stops_instead_of_blindly_holding_the_curve():
    ctx, avoid, target = committed()
    empty_scan(ctx)
    ctx.occupancy_hulls = {'wall': ((24., -10.), (26., -10.), (26., 10.), (24., 10.))}
    stopped = avoid.compute_target(ctx)
    assert stopped.v_max == 0. and math.isfinite(stopped.stop_s)


def test_new_wall_corridor_cannot_validate_the_previous_wider_curve():
    from ctrl_planning.ctrl_lane_prediction import LaneMap
    ctx, avoid, target = committed()
    empty_scan(ctx)
    ctx.road_map = LaneMap([dict(idx='narrow', from_node_idx='a', to_node_idx='b', width_start=3.5,
                               points=[[x, 0., 0.] for x in range(181)])])
    assert avoid.compute_target(ctx).v_max == 0.


def test_blocked_plan_can_release_on_fresh_empty_input():
    ctx = bev_context()
    ctx.mission_name = 'TUNNEL'
    ctx.occupancy_hulls = {'wall': ((28., -10.), (31., -10.), (31., 10.), (28., 10.))}
    avoid = Avoid()
    assert avoid.check_entry(ctx)
    assert avoid.compute_target(ctx).v_max == 0.
    empty_scan(ctx)
    assert avoid.check_exit(ctx)
