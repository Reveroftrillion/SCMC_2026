"""A-1 (2026-10-01): the last change (3->2) may wait longer than max_start_s to pass a slow lane-2 car and land ahead of it."""
import pytest
from dataclasses import replace
from pathlib import Path
from foscar_planning.highway_config import HighwayConfig
from foscar_planning.highway_policy import HighwayPolicy
from foscar_planning.highway_slot_planner import Body, plan_slot
from test_highway_boxes import highway_route


def planner():
    cfg = HighwayConfig.load(Path(__file__).parents[1]/'config/highway.yaml',
                             control_config=Path(__file__).parents[2]/'foscar_bringup/config/low_speed.yaml')
    return HighwayPolicy(highway_route(), cfg)._planner()


S = 1000.
V0 = 20.          # ego, m/s
SLOW = 12.        # lane-2 car, m/s
AHEAD = 95.       # its tail this far ahead of our rear axle: too far to pass within max_start_s 6 s


def slow_car():
    return Body('slow', S+AHEAD, S+AHEAD+4.6, SLOW, .3, 'measured')


def test_within_max_start_the_slow_car_far_ahead_is_landed_behind():
    plan = plan_slot(S, V0, [slow_car()], None, planner(), change=dict(deadline=S+400.))
    assert plan is not None and plan.front == 'slow' and plan.rear == ''


def test_a_longer_start_window_passes_it_and_lands_ahead():
    plan = plan_slot(S, V0, [slow_car()], None, planner(), change=dict(deadline=S+400., max_start=15.))
    assert plan is not None and plan.rear == 'slow' and plan.front == ''
    assert plan.start_t > 6.


def test_the_deadline_still_bounds_a_long_window():
    # 150 m leave no room to pass (the pass needs ~8 s of lane 3) but room to land behind it now: still a plan, inside.
    plan = plan_slot(S, V0, [slow_car()], None, planner(), change=dict(deadline=S+150., max_start=15.))
    assert plan is not None and plan.front == 'slow' and plan.start_s+plan.length_m <= S+150.+1e-6


def test_no_max_start_key_is_the_old_behaviour():
    P = planner(); ch = dict(deadline=S+400.)
    assert plan_slot(S, V0, [slow_car()], None, P, change=ch) == plan_slot(S, V0, [slow_car()], None, P, change={**ch, 'max_start': P.max_start})


from test_highway_route import highway
from test_highway_boxes import policy, drive, ego, car, config, snap

# The 3->2 deadline these policy tests run against: wait_gap uses r.change_deadline(3, vehicle.width_m) = s1562.9 on this
# route (the same map files the stack loads). The s1548 in the spec / plan is tools/inspect_highway_route.py's, which asks
# for width + 2 x margin (2.89 m) of lane-centre separation and so ends 15 m earlier.
DEADLINE_3 = 1562.9


# The 15 s window itself is covered by the planner tests above: with this geometry the ego accelerates, so 6 s already
# reaches the same start and a 6.0 sibling of this test would pass too.
def test_waiting_in_lane_3_the_policy_plans_past_a_slow_lane_2_car(highway):
    # 95 m ahead cannot be passed before the policy's 3->2 deadline (DEADLINE_3) at 20 vs 12 m/s; 60 m ahead, from s1300,
    # can (~8 s later, past the old 6 s window). 12 ticks: the lane-2 car 60 m ahead holds the start (MOTION_PENDING)
    # until its box-edge speed is given, about the tenth scan. That moment is EdgeTracks' (READY_SIGMA, READY_N and the
    # range-growing POS_SIGMA_PER_M; it was boxes.edge_window_s = 1.0 s before the Kalman filter, which no longer reads
    # it): if the box-edge speed comes later, raise `ticks`.
    s = 1300.
    e = ego(highway, s=s, lane=3, v=V0)
    slow = car(highway, 2, s+60., 'slow', v=SLOW)
    p = policy(3); drive(p, e, [slow], ticks=12)
    assert p.plan is not None and p.plan.rear != '' and p.plan.front == '', p.plan


def test_the_last_change_waits_for_the_speed_of_a_far_lane_2_car(highway):
    e = ego(highway, s=1330., lane=3, v=V0)
    out = drive(policy(3), e, [car(highway, 2, 1330.+AHEAD, 'slow', v=SLOW)], ticks=1)
    assert out.reason != 'CHANGE_STARTED' and any(b.startswith('MOTION_PENDING:') for b in out.diagnostics['blocked_by'])


def test_a_non_last_change_does_not_wait_for_a_far_unmeasured_car(highway):
    e = ego(highway, s=1200., lane=4, v=V0)
    out = drive(policy(4), e, [car(highway, 3, 1200.+AHEAD, 'slow', v=SLOW)], ticks=1)
    assert not any(b.startswith('MOTION_PENDING') for b in out.diagnostics.get('blocked_by', []))


def test_a_far_body_that_never_gets_a_speed_does_not_hold_the_last_change_once_late(highway):
    # late from s >= stop_at - lookahead = DEADLINE_3 - 3.8 - .5 - (10+15) - 10 ~ 1523.6. At 12 m/s a change fits
    # there beside a body 45 m ahead whose speed is never ready (a flickering track).
    def tick(s):
        return drive(policy(3), ego(highway, s=s, lane=3, v=12.), [car(highway, 2, s+45., 'frag', ready=False)], ticks=1)
    early, late = tick(1450.), tick(1526.)
    assert any(b.startswith('MOTION_PENDING') for b in early.diagnostics['blocked_by'])
    assert late.reason == 'CHANGE_STARTED' and not late.diagnostics['blocked_by']


def test_the_policy_deadline_for_3_to_2_is_the_one_these_tests_assume(highway):
    assert abs(highway.change_deadline(3, config().vehicle.width_m)-DEADLINE_3) < .1


def changes_planned(monkeypatch, highway, lane, s):
    """The `change` dicts wait_gap hands plan_slot on one tick from `lane` at `s` (empty road)."""
    import foscar_planning.highway_policy as hp
    seen = []
    def spy(*a, **k):
        seen.append(dict(k.get('change') or {}));return plan_slot(*a, **k)
    monkeypatch.setattr(hp, 'plan_slot', spy)
    drive(policy(lane), ego(highway, s=s, lane=lane, v=V0), [], ticks=1)
    assert seen and all('deadline' in ch for ch in seen)
    return seen


def test_only_the_last_change_gets_the_longer_start_window(monkeypatch, highway):
    long_ = config().boxes.last_change_max_start_s
    assert long_ == 15.
    assert all(ch.get('max_start') == long_ for ch in changes_planned(monkeypatch, highway, 3, 1300.))   # 3->2
    assert all('max_start' not in ch for ch in changes_planned(monkeypatch, highway, 4, 1200.))         # 4->3
    assert all('max_start' not in ch for ch in changes_planned(monkeypatch, highway, 5, 1130.))         # 5->4 (entry)


# I1 (final review of A-1): a pass that becomes infeasible near the 3->2 deadline because the lane-2 car speeds up with us.
def closed_loop(highway, s0, v0, gap, vc, match_acc, max_start=None):
    """Tick the policy every 0.1 s from lane 3 with one lane-2 car `gap` m ahead at `vc`. Our speed follows v_max (up at
    boxes.accel_mps2, down at most 9 m/s2: sim brake 1.0). Once a plan lands ahead of the car (we commit to passing it),
    the car speeds up at match_acc toward our speed and never slows again. max_start: force the start window of the
    last change (6.0 = before A-1). Returns per-tick (s, v, v_max, reason, plan) until the change starts, we stop, or
    the car leaves the map."""
    import foscar_planning.highway_policy as hp
    acc = config().boxes.accel_mps2
    if max_start is not None:
        def windowed(*a, **k):
            ch = k.get('change')
            if ch is not None and 'max_start' in ch:k['change'] = {**ch, 'max_start': max_start}
            return plan_slot(*a, **k)
        mp = pytest.MonkeyPatch();mp.setattr(hp, 'plan_slot', windowed)
    try:
        p = policy(3);e = ego(highway, s=s0, lane=3, v=v0);s, v, cs, cv = s0, v0, s0+gap, vc;matched = False;log = []
        for k in range(400):
            t = 1.+.1*k;x, y = highway.at(3, s)
            out = p.tick(replace(e, now=t, s=s, x=x, y=y, v=v), snap(car(highway, 2, cs, 'slow', v=cv), t=t))
            pl = p.plan;matched = matched or (pl is not None and pl.rear != '')
            log.append((round(s, 1), round(v, 2), round(out.v_max, 2), out.reason,
                        None if pl is None else (pl.mode, round(pl.goal, 1), float(pl.start_t), pl.rear, pl.front)))
            if out.reason == 'CHANGE_STARTED' or v <= .1 or cs+cv*.1 > 1580.:break
            nv = min(out.v_max, v+acc*.1) if out.v_max > v else max(out.v_max, v-.9)
            s += (v+nv)/2*.1;v = nv
            if matched:cv = max(cv, min(v, cv+match_acc*.1))
            cs += cv*.1
        return log
    finally:
        if max_start is not None:mp.undo()


def stop_line():
    """wait_gap's stop line for 3->2 (no slot: brake to stand before it) and where `late` begins, on DEADLINE_3."""
    c = config();p = c.policy
    stop_at = DEADLINE_3-c.vehicle.rear_axle_to_front_m-p.margin_m-(p.lookahead_m+p.min_change_length_m)
    return stop_at, stop_at-p.lookahead_m


I1_CASE = (1300., 28., 60., 12.)   # 28 m/s in lane 3, a 12 m/s lane-2 car 60 m ahead: passed at 33 m/s around s1400-1440


def test_a_pass_the_car_lets_happen_lands_ahead_of_it(highway):
    # A car speeding up at <= 1 m/s2 once we pass it: the pass goes through (change at ~s1415, 33 m/s, ahead of it).
    log = closed_loop(highway, *I1_CASE, match_acc=1.)
    assert log[-1][3] == 'CHANGE_STARTED' and log[-1][4][3] != '' and min(r[2] for r in log) >= I1_CASE[1]-1e-6


@pytest.mark.xfail(strict=True, reason='pre-existing (not A-1): no slot left once a passed car speeds up with us near the 3->2 deadline')
def test_a_pass_spoiled_by_the_car_still_ends_in_a_change_without_a_hard_brake(highway):
    # I1 test (a). The car speeds up at 3 m/s2 to our speed once we commit to passing it (at 2 m/s2 we change behind it
    # at s1542, braked 33 -> 10 m/s). Observed: the pass is planned from s1314 (start_t <= 3.5 s), the car gets beside us, NO_SLOT from s1438 and the stop cap
    # brakes us 33 -> 16 m/s (~8 m/s2); landing behind it no longer fits a curve before DEADLINE_3, and past the late
    # line (~s1523.6) v_max drops to 0 for a tick, then ~9 m/s: no change before the car leaves the map (s1552).
    # A-1 plays no part: the trace is identical with the start window held at 6 s (next test), and no plan in it starts
    # later than 6 s, so a reserve on starts beyond max_start_s (I1 fix (b)) cannot change it. Kept as a known gap of the
    # pass rules (strict: it reports when that changes).
    log = closed_loop(highway, *I1_CASE, match_acc=3.)
    stop_at, _ = stop_line()
    assert all(r[1] > .1 for r in log)
    assert min(r[2] for r in log if r[0] < DEADLINE_3) >= I1_CASE[1]/2
    assert log[-1][3] == 'CHANGE_STARTED' and log[-1][0] < stop_at


def test_the_spoiled_pass_is_not_an_a1_decision(highway):
    log15 = closed_loop(highway, *I1_CASE, match_acc=3.)
    assert log15 == closed_loop(highway, *I1_CASE, match_acc=3., max_start=6.)
    assert max(r[4][2] for r in log15 if r[4] is not None) <= 6.
