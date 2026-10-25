import math
from ctrl_planning.ctrl_avoid_choice import side_summary
from ctrl_planning.ctrl_avoid_planner import AvoidPlanner, Box
from ctrl_planning.states.ctrl_avoid import Avoid
from test_tunnel_avoidance import context, road


def test_tunnel_chosen_side_and_blocked_side_reason():
    for left in (True, False):
        c=context(left);a=Avoid(tunnel_extra_margin_m=0.);assert a.check_entry(c);t=a.compute_target(c)
        rows=side_summary(a.plan,tunnel=True)
        chosen,other=('LEFT','RIGHT') if left else ('RIGHT','LEFT')
        assert rows[chosen]['status']=='CHOSEN' and rows[chosen]['d']==t.lat_offset
        assert rows[chosen]['reason']=='OTHER SIDE BLOCKED'
        assert rows[other]['status']=='BLOCKED' and 'OFF ROAD' in rows[other]['reason']


def test_tunnel_replan_keeps_explaining_the_blocked_side():
    # The tunnel slalom may change side on a re-plan (no locked side since 2026-09-29), so the
    # second tick still explains the choice by the other side being off the road.
    c=context();a=Avoid(tunnel_extra_margin_m=0.);a.check_entry(c);a.compute_target(c)
    c.now+=.1;a.compute_target(c)
    rows=side_summary(a.plan,tunnel=True)
    assert rows['LEFT']['reason']=='OTHER SIDE BLOCKED'
    assert rows['RIGHT']['status']=='BLOCKED'


def test_ordinary_lower_cost_and_tie_default_left():
    p=AvoidPlanner()
    right_box=[Box(20.,23.,-1.2,.2)]   # box sits right of centre: left is the short way round
    rows=side_summary(p.plan(right_box,0.),left_bias=p.w_left_bias)
    assert rows['LEFT']['status']=='CHOSEN' and rows['LEFT']['reason']=='LOWER COST'
    assert rows['RIGHT']['status']=='FREE'
    centred=[Box(20.,23.,-.7,.7)]
    rows=side_summary(p.plan(centred,0.),left_bias=p.w_left_bias)
    assert rows['LEFT']['status']=='CHOSEN' and rows['LEFT']['reason']=='TIE / LEFT DEFAULT'


def test_everything_blocked_shows_both_reasons():
    p=AvoidPlanner()
    wall=[Box(20.,23.,-6.,6.)]
    plan=p.plan(wall,0.)
    rows=side_summary(plan)
    assert plan.blocked
    assert all(r['status']=='BLOCKED' and r['reason'].startswith('HITS BOX') for r in rows.values())
