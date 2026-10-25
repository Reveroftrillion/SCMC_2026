"""Closed-loop ordinary cruise response, including uncertain actuator delay."""
import math

import numpy as np
import pytest

from foscar_control.core import Controller, Route, Target


def controller(**options):
    route = Route(np.column_stack((np.arange(1001.), np.zeros(1001))))
    return Controller(route, speed_kmh=60., continuous=True, accel_max=1., **options)


def recovery(gain=3.46596, drag=.001617, rolling=-.076345, delay=.125, lag=.08, noise=0., gap_ticks=0, cached=False):
    """Independent plant: measured pedal gain, speed-dependent resistance and delay."""
    c = controller()
    dt, v, x = .05, 52./3.6, 100.
    u = (rolling+drag*v*v)/gain
    c.integral = .3
    c.previous_accel = u
    times, pedals = [-1., 0.], [u, u]
    brakes = [0., 0.]
    first, peak = None, v
    gap_start = None
    cached_speed = None
    for k in range(240):
        t = k*dt
        measured = v+noise*math.sin(k*1.7)
        if gap_ticks and gap_start is None and v*3.6 >= 54.:
            gap_start = k
            cached_speed = measured
        fresh = gap_start is None or k >= gap_start+gap_ticks
        if cached and not fresh:
            measured = cached_speed
        out = c.step(x, 0., 0., measured, 2, 4, fresh or cached, dt,
                     Target(state='CRUISE', v_max=60./3.6), fresh_speed=fresh)
        times.append(t+dt)
        pedals.append(out.accel)
        brakes.append(out.brake)
        delayed = float(np.interp(t+dt-delay, times, pedals))
        delayed_brake = float(np.interp(t+dt-delay, times, brakes))
        u += dt/(lag+dt)*(delayed-u)
        v = max(0., v+dt*(gain*u-rolling-drag*v*v-12.*delayed_brake))
        x += v*dt
        peak = max(peak, v)
        if first is None and v*3.6 >= 58.5:
            first = (k+1)*dt
    return first, peak*3.6


def test_recovers_from_52_to_58_5_without_the_long_throttle_tail():
    elapsed, peak = recovery()
    # Baseline P=0.2 takes about 4 s with this measured plant.
    assert elapsed is not None and elapsed < 3.
    assert peak <= 60.


@pytest.mark.parametrize('gain,drag,rolling,delay,lag,noise', [
    (4.3, .2/(60./3.6)**2, 0., .3, .2, 0.),
    (4.3, .2/(60./3.6)**2, 0., .3, .2, .08),
    (2.8, .6/(60./3.6)**2, 0., .3, .2, .08),
])
def test_delayed_throttle_does_not_cross_the_ordinary_speed_limit(gain, drag, rolling, delay, lag, noise):
    _, peak = recovery(gain, drag, rolling, delay, lag, noise)
    assert peak <= 60.


@pytest.mark.parametrize('gap_ticks', [1, 2, 3, 6, 20])
def test_missing_feedback_during_acceleration_does_not_hold_throttle_into_overspeed(gap_ticks):
    _, peak = recovery(4.3, .2/(60./3.6)**2, 0., .3, .2, .08, gap_ticks=gap_ticks)
    assert peak <= 60.


@pytest.mark.parametrize('gap_ticks', [6, 12, 20])
def test_cached_speed_marked_healthy_does_not_raise_throttle_during_message_loss(gap_ticks):
    _, peak = recovery(4.3, .2/(60./3.6)**2, 0., .3, .2, .08, gap_ticks=gap_ticks, cached=True)
    assert peak <= 60.


@pytest.mark.parametrize('options,state', [
    ({'uncapped_zones': ((20., 200.),)}, 'CRUISE'),
    ({}, 'CRUISE|SIGNAL_STOP'),
    ({}, 'AVOID'),
    ({'emergency_brake': True}, 'HIGHWAY_FOLLOW'),
])
def test_special_missions_keep_their_existing_throttle_response(options, state):
    c = controller(**options)
    c.previous_accel = 1.
    out = c.step(100., 0., 0., 54./3.6, 2, 4, True, .05, Target(state=state, v_max=60./3.6))
    assert out.accel == pytest.approx(.2*(6./3.6)+.1*(6./3.6)*.05)
    assert out.brake == 0.


def test_lowered_speed_cap_brakes_immediately_after_cruise_acceleration():
    c = controller()
    x = 100.
    for v in np.linspace(52./3.6, 58./3.6, 20):
        c.step(x, 0., 0., v, 2, 4, True, .05, Target(state='CRUISE', v_max=60./3.6))
        x += v*.05
    out = c.step(x, 0., 0., v, 2, 4, True, .05, Target(state='CRUISE', v_max=40./3.6))
    assert out.accel == 0.
    assert out.brake > .3


@pytest.mark.parametrize('start,speed', [(174.95,52.2),(176.05,54.96),(177.45,55.24)])
def test_late_signal_stops_within_line_tolerance(start, speed):
    c = controller()
    x, v, dt = start, speed/3.6, .01
    brake = 0.
    commands = [0.]*15
    for k in range(1000):
        if k%5 == 0:
            out = c.step(x, 0., 0., v, 2, 4, True, .05,
                         Target(stop_s=202.485+3., state='CRUISE|SIGNAL_STOP'))
            brake = out.brake
        commands.append(brake)
        applied = commands.pop(0)
        v = max(0., v-dt*(15.*applied+.3))
        x += v*dt
        if v == 0.:
            break
    assert 201.985 <= x <= 202.985


def test_active_red_stop_does_not_release_brake_after_line_overshoot():
    c = controller()
    out = c.step(203.1, 0., 0., 3., 2, 4, True, .05,
                 Target(stop_s=205.485, v_max=0., state='CRUISE|SIGNAL_STOP'))
    assert out.brake == 1.


def test_slow_vehicle_can_approach_a_red_stop_without_crawling():
    c = controller()
    out = c.step(190., 0., 0., 1., 2, 4, True, .05,
                 Target(stop_s=205.485, state='CRUISE|SIGNAL_STOP'))
    assert out.accel > 0.
    assert out.brake == 0.


def test_delayed_brake_stops_near_line_without_an_extra_creep_departure():
    c = controller()
    v, x, dt = 15.265929, 176.046623, .05
    times, pedals, brakes = [-1., 0.], [.3953, .3953], [0., 0.]
    pedal, brake = .3953, 0.
    for k in range(300):
        t = k*dt
        out = c.step(x, 0., 0., v, 2, 4, True, dt,
                     Target(state='CRUISE|SIGNAL_STOP', v_max=60./3.6, stop_s=205.485))
        times.append(t+dt)
        pedals.append(out.accel)
        brakes.append(out.brake)
        pedal += dt/(.08+dt)*(np.interp(t+dt-.125, times, pedals)-pedal)
        brake += dt/(.05+dt)*(np.interp(t+dt-.2, times, brakes)-brake)
        v = max(0., v+dt*(3.466*pedal+.0763-.001617*v*v-15.*brake))
        x += v*dt
        if v < .03:
            break
    assert 201.985 <= x <= 202.485
