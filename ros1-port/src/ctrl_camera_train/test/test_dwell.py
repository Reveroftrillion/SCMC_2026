"""Timed collection stops use real stopped time, not approach time or signal color."""

import math

import pytest

from ctrl_camera_train.ctrl_dwell import CollectionStops


def stopped_ticks(fsm, start, end, s=97., speed=0.):
    decision = None
    for step in range(round((end - start) * 20) + 1):
        decision = fsm.step(start + step / 20, s, speed)
    return decision


def test_waits_full_minute_after_arrival_then_releases_once():
    fsm = CollectionStops([100., 200.], stop_seconds=60., front_axle_m=3.)
    approaching = fsm.step(0., 50., 5.)
    assert approaching.stop_s == 100. and math.isinf(approaching.v_max)
    assert fsm.step(30., 97., .2).state == 'COLLECT_APPROACH'
    assert fsm.step(31., 97., 0.).state == 'COLLECT_HOLD'
    assert stopped_ticks(fsm, 31.05, 90.95).v_max == 0.
    released = fsm.step(91., 97., 0.)
    assert released.state == 'COLLECT_RELEASE' and math.isnan(released.stop_s)
    assert math.isinf(released.v_max)
    # Still at the previous line: do not start the same minute again.
    assert fsm.step(91.05, 97., 0.).stop_s == 200.


@pytest.mark.parametrize('s,speed,healthy', [(97., .3, True), (95., 0., True),
                                           (97., 0., False), (math.nan, 0., True)])
def test_motion_or_lost_position_resets_continuous_stopped_time(s, speed, healthy):
    fsm = CollectionStops([100.], stop_seconds=2., front_axle_m=3.)
    stopped_ticks(fsm, 0., 1.)
    decision = fsm.step(1.05, s, speed, healthy=healthy)
    if not healthy or not math.isfinite(s):
        assert decision.v_max == 0.
    assert stopped_ticks(fsm, 1.1, 3.05).state == 'COLLECT_HOLD'
    assert fsm.step(3.1, 97., 0.).state == 'COLLECT_RELEASE'


def test_sensor_or_clock_gap_does_not_count_as_a_minute_of_observation():
    fsm = CollectionStops([100.], stop_seconds=60., front_axle_m=3.)
    fsm.step(0., 97., 0.)
    decision = fsm.step(80., 97., 0.)
    assert decision.state == 'COLLECT_HOLD'
    assert decision.remaining_s == 60.


def test_skips_lines_behind_start_including_spawn_negative_s():
    fsm = CollectionStops([-20., 100., 200.], stop_seconds=1., front_axle_m=3.)
    assert fsm.step(0., 105., 1.).stop_s == 200.
    fsm2 = CollectionStops([-20., 100.], stop_seconds=1., front_axle_m=3.)
    assert fsm2.step(0., -50., 1.).stop_s == -20.


def test_overshooting_active_stop_does_not_silently_release():
    fsm = CollectionStops([100.], stop_seconds=60., front_axle_m=3.)
    fsm.step(0., 90., 3.)
    decision = fsm.step(.05, 98., 2.)
    assert decision.state == 'COLLECT_MISSED_STOP' and decision.v_max == 0.


def test_configurable_stop_before_line_and_completion():
    fsm = CollectionStops([100.], stop_seconds=1., front_axle_m=3., stop_before_m=10.)
    assert fsm.step(0., 70., 3.).stop_s == 90.
    assert stopped_ticks(fsm, .05, 1.05, s=87.).state == 'COLLECT_RELEASE'
    assert math.isnan(fsm.step(1.1, 87., 0.).stop_s)


@pytest.mark.parametrize('kwargs', [{'stop_seconds': -1}, {'stop_seconds': math.nan},
                                    {'front_axle_m': -1}, {'stop_before_m': -1}])
def test_rejects_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        CollectionStops([100.], **kwargs)


@pytest.mark.parametrize('speed_kmh', [20., 30.])
def test_existing_controller_brakes_holds_and_resumes_at_two_lines(speed_kmh):
    import numpy as np
    from ctrl_control.ctrl_core import Controller, Route, Target

    route = Route(np.column_stack((np.arange(0, 400, .5), np.zeros(800))))
    controller = Controller(route, speed_kmh=speed_kmh, continuous=True)
    fsm = CollectionStops([100., 200.], stop_seconds=60.)
    x = speed = 0.
    held_since = {}
    released = []
    for tick in range(6000):
        now = tick * .05
        decision = fsm.step(now, x, speed)
        assert decision.state != 'COLLECT_MISSED_STOP'
        if decision.state == 'COLLECT_HOLD':
            held_since.setdefault(decision.line_s, now)
            assert abs(x + 3. - decision.line_s) <= .5
        if decision.state == 'COLLECT_RELEASE':
            assert now - held_since[decision.line_s] >= 60. - 1e-9
            released.append(decision.line_s)
        out = controller.step(x, 0., 0., speed, 2, 4, True, .05,
                              Target(stop_s=decision.stop_s, v_max=decision.v_max))
        assert out.state != 'FAULT', out.reason
        speed = max(0., speed + (out.accel * 4 - out.brake * 15 - .3) * .05)
        x += speed * .05
        if x > 220.:
            break
    assert released == [100., 200.]
    assert x > 220.
