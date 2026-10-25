import math
import numpy as np
from foscar_planning.states.avoid import Avoid
from foscar_planning import tunnel_slalom
from test_tunnel_avoidance import context
from test_avoid_signal_hold import ctx, HOLD


def diagonal_context():
    c = context(); c.mission_name = 'STATIC_OBSTACLE'
    e = c.events[0]; e.value = 'lidar:diagonal'; e.size_x = 8.; e.size_y = 6.
    e.footprint = ((36., -2.), (37., -3.), (44., 2.), (43., 3.))
    return c


def test_static_memory_keeps_diagonal_empty_corner_free():
    c = diagonal_context(); a = Avoid(); a.observe(c)
    assert not tunnel_slalom.sweep_hits(a.boxes(), 2., 2., 35., 1., 35., 35., 1., .3, .3)
    assert tunnel_slalom.sweep_hits(a.boxes(), 0., 0., 40., 1., 40., 40., 1., .3, .3)
    c.s = 35.; c.now += .1; c.events = []; a.observe(c)
    assert not tunnel_slalom.sweep_hits(a.boxes(), 2., 2., 35., 1., 35., 35., 1., .3, .3)
    assert tunnel_slalom.sweep_hits(a.boxes(), 0., 0., 40., 1., 40., 40., 1., .3, .3)


def test_polygon_replaces_fallback_box_without_retaining_empty_corners():
    c = diagonal_context(); a = Avoid(); polygon = c.events[0].footprint
    c.events[0].footprint = (); a.observe(c)
    c.events[0].footprint = polygon; c.now += .1; a.observe(c)
    assert len(a.memory) == 1
    assert not tunnel_slalom.sweep_hits(a.boxes(), 2., 2., 35., 1., 35., 35., 1., .3, .3)


def test_static_memory_preserves_previous_observed_polygon_when_side_disappears():
    c = diagonal_context(); a = Avoid(); a.observe(c)
    c.events[0].footprint = ((36., -2.), (37., -3.), (40., -.8))
    c.now += .1; a.observe(c)
    assert max(b.s_max for b in a.boxes()) >= 44.
    assert tunnel_slalom.sweep_hits(a.boxes(), 2.5, 2.5, 42.8, 1., 42.8, 42.8, 1., .3, .3)


def test_far_signal_hold_does_not_apply_unreachable_curve_crawl():
    a = Avoid(); c = ctx(HOLD-20., 10., 1.)
    c.events[0].s = HOLD+10.; c.events[0].size_y = 6.
    assert a.check_entry(c)
    far = a.compute_target(c)
    assert far.v_max*3.6 > 30.
    assert far.lat_offset == 0. and not far.chain
    c.s = HOLD-2.; c.now += .1; a.observe(c)
    near = a.compute_target(c)
    assert 0. < near.v_max < far.v_max
    # The approach must allow reaching zero speed at the hold position.
    assert near.v_max**2 <= 2*a.slalom_decel_mps2*(HOLD-c.s)+1e-6
