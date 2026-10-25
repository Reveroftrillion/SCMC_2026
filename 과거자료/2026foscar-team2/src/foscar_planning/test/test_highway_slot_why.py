"""plan_slot's optional rejection tally (2026-10-01 overtake analysis): which check refuses each candidate."""
import math
from pathlib import Path
import numpy as np
from foscar_planning.highway_config import HighwayConfig
from foscar_planning.highway_policy import HighwayPolicy
from foscar_planning.highway_slot_planner import Body, plan_slot
from test_highway_boxes import highway_route
from test_highway_route import highway
from test_highway_boxes import policy, drive, ego, car


def planner():
    cfg = HighwayConfig.load(Path(__file__).parents[1]/'config/highway.yaml',
                             control_config=Path(__file__).parents[2]/'foscar_bringup/config/low_speed.yaml')
    return HighwayPolicy(highway_route(), cfg)._planner()


S = 1300.
CHANGE = dict(deadline=1500.)


def test_empty_lane_counts_accepted_candidates():
    why = {}
    plan = plan_slot(S, 20., [], None, planner(), change=CHANGE, why=why)
    assert plan is not None
    assert sum(n for (mode, reason), n in why.items() if reason == 'ok') > 0


def test_a_stopped_lead_just_ahead_refuses_every_candidate_for_the_lead():
    P = planner()
    lead = Body('lead', S+P.front+1., S+P.front+5., 0., 0., 'lead')
    why = {}
    assert plan_slot(S, 20., [], lead, P, change=CHANGE, why=why) is None
    reasons = {reason for (mode, reason) in why}
    assert 'ok' not in reasons
    assert reasons & {'lead_only', 'braked_lead'}
    assert all(r in ('lead_only', 'ok') or r.startswith('braked_lead') or r in ('latest_start', 'deadline', 'horizon', 'ease_late', 'enter_late')
               for r in reasons)


def test_a_lead_and_a_target_lane_car_beside_both_count():
    # Our own lead stopped close ahead and a target-lane car beside us at our speed: a later start fails both checks,
    # starting now fails braking for the lead and the car beside (behind our front when we enter).
    P = planner()
    lead = Body('lead', S+P.front+3., S+P.front+7.5, 0., 0., 'lead')
    side = Body('side', S-2., S+2.5, 20., 0., 'measured')
    why = {}
    assert plan_slot(S, 20., [side], lead, P, change=CHANGE, why=why) is None
    reasons = {reason for (mode, reason) in why}
    assert 'lead+body' in reasons
    assert any(r.startswith('braked_') and '+' in r for r in reasons), reasons


def test_the_tally_does_not_change_the_plan():
    P = planner()
    bodies = [Body('side', S-3., S+2., 18., 0.3, 'measured'), Body('ahead', S+30., S+34.5, 15., 0.3, 'measured')]
    lead = Body('lead', S+40., S+44.5, 12., 0.3, 'lead')
    why = {}
    plan = plan_slot(S, 20., bodies, lead, P, change=CHANGE, why=why)
    assert plan is not None and plan == plan_slot(S, 20., bodies, lead, P, change=CHANGE)
    assert any(reason != 'ok' and n > 0 for (mode, reason), n in why.items())
    # No plan at all: still None either way.
    stopped = Body('lead', S+P.front+3., S+P.front+7.5, 0., 0., 'lead')
    side = [Body('side', S-2., S+2.5, 20., 0., 'measured')]
    assert plan_slot(S, 20., side, stopped, P, change=CHANGE) is None
    assert plan_slot(S, 20., side, stopped, P, change=CHANGE, why={}) is None
    # The commit re-check (start now, given curve length), with and without a goal.
    for only in ((None, plan.length_m), (plan.goal, plan.length_m), (None, plan.length_m+20.)):
        assert plan_slot(S, 20., bodies, lead, P, change=CHANGE, only=only) == \
            plan_slot(S, 20., bodies, lead, P, change=CHANGE, only=only, why={})


def test_each_candidate_window_is_counted_once():
    # v0 = vmax = 20 m/s: goals are the whole m/s from v0-10 to v0 (11); no deadline or latest start cuts the start
    # times short, so every goal evaluates every start (0 .. max_start every start_step) plus the taper window.
    P = planner()
    bodies = [Body('side', S-3., S+2., 18., 0.3, 'measured'), Body('ahead', S+30., S+34.5, 15., 0.3, 'measured')]
    lead = Body('lead', S+25., S+29.5, 12., 0.3, 'lead')
    why = {}
    plan_slot(S, 20., bodies, lead, P, change=dict(deadline=1e9), taper=(S+40., S+60., S+80.), vmax=20., why=why)
    starts = len(np.arange(0., P.max_start+1e-9, P.start_step))
    assert sum(why.values()) == 11*(starts+1)
    assert len({reason for (mode, reason) in why}) >= 3


def test_policy_reports_the_tally_only_when_asked(highway):
    e = ego(highway, s=1360., lane=4, v=20.)
    world = [car(highway, 4, 1360.+12., 'lead', v=8.), car(highway, 3, 1360.+1., 'side', v=20.)]
    quiet = policy(4); out = drive(quiet, e, world, ticks=4)
    assert 'slot_why' not in out.diagnostics
    traced = policy(4); traced.trace_slots = True; out = drive(traced, e, world, ticks=4)
    assert isinstance(out.diagnostics.get('slot_why'), dict) and out.diagnostics['slot_why']
    assert all(':' in k for k in out.diagnostics['slot_why'])
