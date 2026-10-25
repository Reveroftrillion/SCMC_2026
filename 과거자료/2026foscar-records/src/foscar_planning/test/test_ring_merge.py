"""ring_merge: the roundabout entry as speed planning (user 2026-10-01)."""
import json
import math
import os
import types

import numpy as np
import pytest

from foscar_control.core import Route
from foscar_planning.ring_merge import RingBody, RingMerge, command, commit_speed

ROOT = os.path.join(os.path.dirname(__file__), '..', '..', 'foscar_bringup')
STOP = 876.6283           # behavior.yaml roundabout.stop_at_s
CAP = 60/3.6


@pytest.fixture(scope='module')
def merge():
    route = Route(np.loadtxt(os.path.join(ROOT, 'routes', 'global_path.txt')))
    links = {l['idx']: np.asarray(l['points'])[:, :2] for l in json.load(open(os.path.join(ROOT, 'maps', 'link_set.json')))}
    pts = np.vstack([links[k][:-1] for k in ('A2256W000132', 'A2256W000133', 'A2256W000134', 'A2256W000135')])
    o = pts.mean(0); q = pts-o
    fit = np.linalg.lstsq(np.c_[2*q, np.ones(len(q))], (q*q).sum(1), rcond=None)[0]
    circle = types.SimpleNamespace(center=o+fit[:2], radius=math.sqrt(fit[2]+fit[:2]@fit[:2]), direction=1)
    m = RingMerge(circle, route, 0., 820.92, 908.49)
    m.route = route
    return m


def lidar(m, s):
    x, y = m.route.at(s); h = m.route.heading_at(s)
    return x+2.*math.cos(h), y+2.*math.sin(h)


def drive(m, s, v, bodies_at, until=905., dt=.1, limit=40.):
    """Closed loop: plan every 0.2 s, command every step, ideal speed tracking (3.5 / 4 m/s2)."""
    committed = line = None
    t = 0.; log = []; plan = {}
    while s < until and t < limit:
        bodies = bodies_at(t)
        if committed is None and round(t/dt) % 2 == 0:
            plan = m.plan(s, v, lidar(m, s), bodies, STOP, CAP, t)
        if committed is None:
            goal, stop, commit, state = command(m, plan, s, v, STOP, CAP)
            if commit:
                committed, line = t, plan['commit_line_mps']
        else:
            goal, state = commit_speed(m, line, s, STOP, CAP), 'ENTER'
        goal = min(goal, m.curve_speed(s))
        v = min(goal, v+3.5*dt) if goal > v else max(goal, v-4.*dt)
        s += v*dt; t += dt
        log.append((t, s, v, state))
    return committed, log


def moving(id_, u, v, length=4.8):
    return lambda t: RingBody(id_, u+v*t, length/2, length/2, v, v)


def test_geometry(merge):
    assert 878. < merge.s_in < 880. < merge.s_merged < 890. < merge.s_out < 905.
    assert merge.s_in > STOP


def test_empty_ring_is_driven_through_without_stopping(merge):
    committed, log = drive(merge, 840., 40/3.6, lambda t: [])
    assert committed is not None
    assert log[-1][1] >= 905.
    assert min(v for _, s, v, _ in log) > 3.     # slowed for the entry curve only


def test_far_out_and_unseen_it_keeps_driving(merge):
    plan = merge.plan(822., 50/3.6, lidar(merge, 822.), [], STOP, CAP)
    assert plan['decision'] in ('APPROACH', 'GO')
    speed, stop, commit, _ = command(merge, plan, 822., 50/3.6, STOP, CAP)
    assert not commit and stop is None and speed >= 50/3.6-.01


def test_a_car_about_to_pass_is_waited_for_then_followed(merge):
    car = moving('a', -8., 5.8)
    plan = merge.plan(STOP, 0., lidar(merge, STOP), [car(0.)], STOP, CAP)
    assert plan['decision'] == 'WAIT'
    committed, log = drive(merge, STOP, 0., lambda t: [car(t)])
    assert committed is not None and 0. < committed < 4.
    ego = [(t, s) for t, s, _, _ in log if merge.ego_interval(s) is not None and s < merge.s_merged]
    for t, s in ego:                  # merging, never beside it (merged, the ring following has it)
        lo, hi = merge.ego_interval(s)
        u = car(t).u
        assert hi < u-2.4 or lo > u+2.4


def test_a_one_car_gap_in_a_slow_queue_is_taken(merge):
    """13:33 lap queue: cars at 3.3 m/s; a gap of ~20 m between bumpers with us stopped at the line."""
    lead, rear = moving('lead', 4.2, 3.3), moving('rear', -20.2, 3.3)
    plan = merge.plan(STOP-.2, 0., lidar(merge, STOP), [lead(0.), rear(0.)], STOP, CAP)
    assert plan['decision'] == 'GO'


def test_the_line_is_passed_only_committed(merge):
    plan = dict(decision='APPROACH', line_mps=10.)
    for s in np.arange(870., STOP, .25):
        speed, _, commit, _ = command(merge, plan, s, 0., STOP, CAP)
        assert not commit
    speed, _, _, _ = command(merge, plan, STOP-.2, 0., STOP, CAP)
    assert speed < .5


def test_go_far_out_does_not_commit_yet(merge):
    plan = merge.plan(840., 10., lidar(merge, 840.), [], STOP, CAP)
    _, _, commit, _ = command(merge, plan, 840., 10., STOP, CAP)
    assert not commit


def test_unobserved_traffic_blocks_the_commit_only(merge):
    """Lidar far from the ring: unseen traffic may be close. A stop must stay possible, but nothing waits for it
    before the decision point."""
    seen = merge.visible_upstream(lidar(merge, 830.))
    assert seen < 10.
    plan = merge.plan(830., 12., lidar(merge, 830.), [], STOP, CAP)
    assert plan['decision'] == 'APPROACH'
    speed, _, commit, _ = command(merge, plan, 830., 12., STOP, CAP)
    assert not commit and speed > 11.


def test_a_car_lost_from_the_scan_still_blocks_for_the_coast_time(merge):
    merge.memory = {}
    truck = RingBody('truck', -6., 4.4, 4.4, 3., 3.)
    merge.coast([truck], 0.)
    kept = merge.coast([], 1.)
    assert [b.id for b in kept] == ['truck'] and abs(kept[0].u+3.) < 1e-9
    assert merge.coast([], merge.coast_s+1.5) == []
