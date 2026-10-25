"""AVOID waits for the calibrated signal stop line before swerving (user 2026-09-28).

Practice3 intersection_02: rear-axle stop pose s311.475, static box centred at s329 (2 x 3 m).
Swerving before the line left the car stopped across the lane marking and turned camera 7
away from the light, so the green was never read.
"""
import math
import numpy as np
from foscar_control.core import Route
from foscar_planning.lane_prediction import LaneMap

from foscar_planning.state import Context, EventInfo
from foscar_planning.states.avoid import Avoid
from foscar_planning.tunnel_slalom import first_hit

HOLD = 311.475


def ctx(s, v, now, hold=HOLD):
    box = EventInfo(type='OBSTACLE', stamp=now, s=329., d=0., x=329.-s, y=0., size_x=3., size_y=2.,
                    value='lidar:test:1')
    return Context(s=s, x=s, y=0., yaw=0., v=v, now=now, events=[box], mission_name='STATIC_OBSTACLE',
                   lateral_hold_s=hold, route=Route(np.c_[np.arange(401.),np.zeros(401)]),
                   road_map=LaneMap([dict(idx=str(i),from_node_idx=str(i),to_node_idx=str(i+1),
                       width_start=3.5,points=[[x,d,0] for x in range(401)]) for i,d in enumerate([0,3.5])]))


def test_no_lateral_request_before_the_stop_line():
    avoid = Avoid(entry_buffer_m=0.5)
    for i, s in enumerate((295., 300., 305., 310., 311.4)):
        c = ctx(s, 8., i*.1)
        assert avoid.check_entry(c)
        t = avoid.compute_target(c)
        assert t.lat_offset == 0. and not math.isfinite(t.entry_s) and t.entry_length_m == 0.
    assert not avoid.lateral_started


def test_approach_speed_lets_the_curve_fit_between_line_and_box():
    avoid = Avoid(entry_buffer_m=0.5)
    c = ctx(300., 11., 0.)
    avoid.check_entry(c)
    t = avoid.compute_target(c)
    room = avoid.plan.nearest_s-avoid.front_m-avoid.entry_buffer_m-HOLD
    shift = abs(avoid.plan.d)
    fit = avoid.swerve_speed(room, shift)
    assert t.v_max > fit
    assert math.isclose(t.v_max*t.v_max-fit*fit, 2*avoid.slalom_decel_mps2*(HOLD-c.s))


def test_swerve_starts_at_the_line_and_ends_before_the_box():
    avoid = Avoid(entry_buffer_m=0.5)
    c = ctx(310., 3., 0.)
    avoid.check_entry(c)
    avoid.compute_target(c)
    c = ctx(HOLD+.2, 1., 1.)
    avoid.check_entry(c)
    t = avoid.compute_target(c)
    assert avoid.lateral_started and t.lat_offset != 0.
    assert t.entry_s >= HOLD
    end=max(st+ln for _,st,ln in t.chain)
    assert first_hit(avoid.boxes(),t.chain,avoid.slalom_origin,c.s,end,avoid.front_m,avoid.rear_overhang_m,avoid.static_planner.half) is None


def test_without_a_stop_line_the_normal_entry_is_unchanged():
    free = Avoid(entry_buffer_m=0.5)
    c = ctx(305., 8., 0., hold=math.nan)
    free.check_entry(c)
    t = free.compute_target(c)
    assert free.lateral_started and t.lat_offset != 0. and t.entry_s < HOLD


def test_confirmed_go_releases_the_hold_but_stop_colours_keep_it():
    from foscar_planning.signal_stop import SignalStopPolicy
    line = dict(id='intersection_02', start_s=271.99, stop_rear_s=HOLD, maneuver='straight', roi=[0, 0, 1, 1])
    policy = SignalStopPolicy([line])
    assert policy.lateral_hold_s(290.) == HOLD                     # not seen yet: hold
    policy.observe('intersection_02', 'red', 290., 10., 10.)
    assert policy.lateral_hold_s(291.) == HOLD
    policy.observe('intersection_02', 'green', 292., 11., 11.)
    assert math.isnan(policy.lateral_hold_s(293.))                  # will not stop: normal swerve
    assert math.isnan(policy.lateral_hold_s(HOLD+.1))               # past the last line


def test_swerve_accel_rises_when_slow_and_steering_bounds_the_curve():
    # user 2026-09-29: steer much harder at low speed; never sharper than the steering allows.
    a=Avoid()
    for length,shift in ((8.,3.),(12.,3.5),(20.,2.),(40.,3.)):
        v=a.swerve_speed(length,shift);k=a.swerve_curvature(length,shift)
        if k>a.max_curvature_per_m:
            assert v==.5;continue
        kmh=v*3.6;lo,hi=a.swerve_accel_slow_kmh,a.swerve_accel_fast_kmh
        want=a.swerve_accel_slow_mps2 if kmh<=lo else a.swerve_accel_mps2 if kmh>=hi else \
            a.swerve_accel_slow_mps2+(a.swerve_accel_mps2-a.swerve_accel_slow_mps2)*(kmh-lo)/(hi-lo)
        # sharp curves also get the curvature allowance (user 2026-09-29); the larger one counts
        span=a.max_curvature_per_m-a.swerve_sharp_from_per_m
        a_k=a.swerve_accel_mps2+(a.swerve_accel_sharp_mps2-a.swerve_accel_mps2)*min(1.,max(0.,(k-a.swerve_sharp_from_per_m)/span))
        assert v*v*k>=a_k-1e-6 and (abs(v*v*k-want)<1e-6 or abs(v*v*k-a_k)<1e-6),(length,shift,kmh)
    assert a.swerve_speed(12.,3.5)*3.6<30. and a.swerve_speed(12.,3.5)**2*a.swerve_curvature(12.,3.5)>4.5
