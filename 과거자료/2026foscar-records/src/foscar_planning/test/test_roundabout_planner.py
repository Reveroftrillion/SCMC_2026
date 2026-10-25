"""roundabout_planner + roundabout_boxes on the real route and ring."""
import json
import math
import os
import types

import numpy as np
import pytest

from foscar_control.core import Route
from foscar_planning.roundabout_boxes import RingLane
from foscar_planning.roundabout_planner import RoundaboutGeometry, RoundaboutPlanner

ROOT = os.path.join(os.path.dirname(__file__), '..', '..', 'foscar_bringup')
LINE = 876.6283


@pytest.fixture(scope='module')
def parts():
    route = Route(np.loadtxt(os.path.join(ROOT, 'routes', 'global_path.txt')))
    links = {l['idx']: np.asarray(l['points'])[:, :2] for l in json.load(open(os.path.join(ROOT, 'maps', 'link_set.json')))}
    pts = np.vstack([links[k][:-1] for k in ('A2256W000132', 'A2256W000133', 'A2256W000134', 'A2256W000135')])
    o = pts.mean(0); q = pts-o
    fit = np.linalg.lstsq(np.c_[2*q, np.ones(len(q))], (q*q).sum(1), rcond=None)[0]
    circle = types.SimpleNamespace(center=o+fit[:2], radius=math.sqrt(fit[2]+fit[:2]@fit[:2]), direction=1)
    return route, circle, RoundaboutGeometry(circle, route, 0., 820.92, 908.49)


def planner(parts):
    route, circle, g = parts
    return RoundaboutPlanner(g, RingLane(circle, g.origin), LINE, 60/3.6), route


def lidar(route, s):
    x, y = route.at(s); h = route.heading_at(s)
    return x+2*math.cos(h), y+2*math.sin(h)


def car(lane, u, length=4.8, width=1.9):
    """A hull on the ring lane centred at arc u."""
    pts = []
    for du in (-length/2, length/2):
        for dr in (-width/2, width/2):
            a = (lane.origin+(u+du)/lane.circle.radius)*lane.circle.direction
            r = lane.circle.radius+dr
            pts.append((lane.circle.center[0]+r*math.cos(a), lane.circle.center[1]+r*math.sin(a)))
    return pts


def feed(rp, route, s, cars, t0=0., n=8, dt=.12):
    """Scans of ring cars [(u0, speed)] at dt; returns the time of the last one."""
    for k in range(n):
        t = t0+k*dt
        rp.lane.update(t, {f'c{i}': car(rp.lane, u+v*t) for i, (u, v) in enumerate(cars)}, lidar(route, s))
    return t


def test_geometry_and_wait_station(parts):
    rp, _ = planner(parts)
    assert rp.g.s_in < rp.g.s_merged < rp.g.s_out
    assert rp.stop_s < LINE and rp.g.s_in-rp.stop_s > 3.


def test_empty_ring_standing_goes(parts):
    rp, route = planner(parts)
    feed(rp, route, rp.stop_s, [])
    assert rp.plan(rp.stop_s, 0.).decision == 'GO'


def test_a_car_coming_is_waited_for_then_gone_after(parts):
    rp, route = planner(parts)
    feed(rp, route, rp.stop_s, [(-6., 5.8)])
    p = rp.plan(rp.stop_s, 0.)
    assert p.decision == 'WAIT'


def test_approaching_times_the_gap_behind_a_passing_car(parts):
    rp, route = planner(parts)
    s = rp.stop_s-15.
    feed(rp, route, s, [(-15., 5.8)])
    p = rp.plan(s, 9.)
    v, stop = rp.speed(p, s, 9.)
    assert p.decision == 'GO' or v > 5.            # 15 m out: keep rolling, never a stop there


def test_measured_launch_and_approach(parts):
    route, circle, g = parts
    table = json.load(open(os.path.join(os.path.dirname(__file__), '..', 'config', 'roundabout_motion.json')))
    rp = RoundaboutPlanner(g, RingLane(circle, g.origin), LINE, 60/3.6, motion_table=table)
    m = rp.motion(rp.stop_s, 0., 0., 1.)
    t_in = next(t for t, s, v in m if s >= g.s_in)
    assert 1.+rp.p.launch_delay_s+.8 < t_in < 1.+rp.p.launch_delay_s+2.5     # measured launch, after the wait
    s0 = 850.; S = rp.approach_table[6.]; v0 = float(S[np.searchsorted(S[:, 1], s0), 2])
    a = rp.motion(s0, v0, 6., 0.)
    assert a[0][1] == pytest.approx(s0) and a[-1][1] >= g.s_out


def test_soft_launches_are_candidates_and_continue(parts):
    route, circle, g = parts
    table = json.load(open(os.path.join(os.path.dirname(__file__), '..', 'config', 'roundabout_motion.json')))
    rp = RoundaboutPlanner(g, RingLane(circle, g.origin), LINE, 60/3.6, motion_table=table)
    assert [a for a, _ in rp.launches][0] == math.inf and len(rp.launches) >= 2
    full = rp.motion(rp.stop_s, 0., 0., 0.)
    soft = rp.motion(rp.stop_s, 0., 0., 0., launch=min(a for a, _ in rp.launches))
    t_in = lambda m: next(t for t, s, v in m if s >= g.s_in)
    assert t_in(soft) > t_in(full)                     # a soft launch reaches the ring later
    # a soft launch under way continues from where we are
    rp.launch_mps2, rp.launch_elapsed = min(a for a, _ in rp.launches), 2.
    rest = rp.launch_rest(rp.stop_s+1.)
    assert rest[0][0] == pytest.approx(0., abs=.11) and rest[0][1] == pytest.approx(rp.stop_s+1.)
    # an empty ring standing: full pedal, now
    feed(rp, route, rp.stop_s, [])
    p = rp.plan(rp.stop_s, 0.)
    assert p.decision == 'GO' and p.launch_mps2 == math.inf


def test_gaps_between_ring_cars(parts):
    rp, route = planner(parts)
    s = rp.stop_s
    feed(rp, route, s, [(-14., 5.8), (6., 5.8)])
    gaps = rp.lane.gaps()
    mid = [gp for gp in gaps if gp.bounded == (True, True)]
    assert len(mid) == 1
    gp = mid[0]
    t = 7*.12
    assert gp.lo == pytest.approx(-14.+5.8*t+2.4, abs=1.) and gp.hi == pytest.approx(6.+5.8*t-2.4, abs=1.)
    assert gp.rear_speed is not None and gp.front_speed is not None
    assert gp.length == pytest.approx(20.-4.8, abs=1.5)


def test_tuck_waits_for_the_coming_car_then_launches_behind_it(parts):
    route, circle, g = parts
    table = json.load(open(os.path.join(os.path.dirname(__file__), '..', 'config', 'roundabout_motion.json')))
    rp = RoundaboutPlanner(g, RingLane(circle, g.origin), LINE, 60/3.6, motion_table=table)
    s0 = 868.97
    feed(rp, route, s0, [])
    lead, tau, _ = rp.tuck(s0, rp.lane.runs)
    assert lead is None and tau == 0.                      # nothing coming: go
    rp2 = RoundaboutPlanner(g, RingLane(circle, g.origin), LINE, 60/3.6, motion_table=table)
    feed(rp2, route, s0, [(-20., 5.7)])
    lead, tau, _ = rp2.tuck(s0, rp2.lane.runs)
    assert lead is not None and tau is not None and tau > .5   # wait for it to pass, then launch behind it
    # the lead is kept as it passes our nose, and we launch behind it about when predicted
    T = 7*.12
    while T < 12.:
        T += .2
        feed(rp2, route, s0, [(-20., 5.7)], t0=T, n=1)
        lead3, tau3, pred = rp2.tuck(s0, rp2.lane.runs)
        if tau3 is not None and tau3 < .1:
            break
    assert lead3 is not None and pred['front_gap'] >= 0.
    assert abs(T-(7*.12+tau)) < 1.


def test_roll_in_goes_on_when_the_way_in_fits_and_stops_for_a_car_arriving_with_us(parts):
    route, circle, g = parts
    table = json.load(open(os.path.join(os.path.dirname(__file__), '..', 'config', 'roundabout_motion.json')))
    s, a = 862., 8.

    def decide(u_end, n=12):
        # a car measured (12 scans); the ring's posted limit as the behaviour sets it from the map (30 km/h)
        rp = RoundaboutPlanner(g, RingLane(circle, g.origin), LINE, 60/3.6, motion_table=table)
        rp.ring_limit_mps = 30/3.6
        S = rp.approach_table[a]; v = float(S[np.searchsorted(S[:, 1], s), 2])
        feed(rp, route, s, [(u_end-5.7*(n-1)*.12, 5.7)] if u_end is not None else [], n=n)
        m, lead = rp.roll_in(s, v, rp.lane.runs)
        return m is not None, lead is not None
    assert decide(None) == (True, False)    # nothing coming: rolled in on an empty way
    # a car reaching our entry as we do: never an empty-way go -- at most one in behind it, which the behaviour hands
    # to the tuck (rolling in only on a way with no car on it, user 2026-10-03)
    go, behind = decide(-6.)
    assert not go or behind
    assert decide(6.)[0]                    # one already past: we follow it
