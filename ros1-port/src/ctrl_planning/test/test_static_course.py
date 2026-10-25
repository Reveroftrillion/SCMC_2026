import math
from pathlib import Path

import numpy as np

from ctrl_control.ctrl_core import Route
from ctrl_planning.ctrl_avoid_planner import polygon_box
from ctrl_planning.ctrl_frenet import to_frenet
from ctrl_planning.ctrl_lane_prediction import LaneMap
from ctrl_planning.ctrl_state import Context
from ctrl_planning.states.ctrl_avoid import Avoid, Remembered
from ctrl_planning.ctrl_static_course import MOVES, StaticCourse
from ctrl_planning.ctrl_tunnel_slalom import first_hit


def course_context():
    import json
    root = Path(__file__).parents[2]/'ctrl_bringup'
    route = Route(np.loadtxt(root/'routes/global_path.txt'))
    road = LaneMap(json.loads((root/'maps/link_set.json').read_text()))
    return Context(s=309.88, x=-60., y=-160., yaw=math.pi/2, v=0., now=1.,
                   route=route, road_map=road, mission_name='STATIC_OBSTACLE', lateral_hold_s=430.9174)


def test_fixed_course_clears_full_supplied_box_map_and_cp4():
    ctx = course_context()
    avoid = Avoid(static_course=True, margin_m=.5)
    x,y,yaw = -60.61000061035156,-142.17799377441406,math.radians(-83.219)
    co,si = math.cos(yaw),math.sin(yaw)
    corners = [(x+a*co-b*si,y+a*si+b*co) for a,b in [(-1,-1.5),(1,-1.5),(1,1.5),(-1,1.5)]]
    box = polygon_box([to_frenet(ctx.route,*p,329.) for p in corners])
    avoid.memory = [Remembered(box,ctx.now)]
    out = avoid.course.target(ctx, avoid)
    assert out.chain == MOVES and out.v_max > 0.
    assert avoid.course.paved
    assert first_hit([box],out.chain,0.,300.,360.,avoid.front_m,avoid.rear_overhang_m,avoid.static_planner.half) is None
    s=345.14052925086577
    u=(s-MOVES[1][1])/MOVES[1][2]
    d=MOVES[0][0]*(1-u**3*(10-15*u+6*u*u))
    assert abs(d)<3.


def test_observed_new_obstacle_stops_the_fixed_course_without_losing_chain():
    from ctrl_planning.ctrl_avoid_planner import Box
    ctx=course_context();avoid=Avoid(static_course=True)
    # Put the new obstruction on the configured course, independent of side.
    d = MOVES[0][0]
    avoid.memory=[Remembered(Box(330.,332.,d-1.,d+1.),ctx.now)]
    out=avoid.course.target(ctx,avoid)
    assert out.v_max == 0. and out.chain == MOVES


def test_fixed_course_keeps_red_approach_straight_and_rejects_late_entry():
    ctx=course_context();avoid=Avoid(static_course=True)
    ctx.s=300.;ctx.lateral_hold_s=309.8735
    assert avoid.course.target(ctx,avoid) is None
    ctx.s=320.;ctx.lateral_hold_s=430.9174
    assert avoid.course.target(ctx,avoid).v_max == 0.


def test_other_route_or_mission_uses_existing_planner():
    ctx=course_context();avoid=Avoid(static_course=True)
    ctx.mission_name='TUNNEL'
    assert avoid.course.target(ctx,avoid) is None


def test_follower_finish_runout_keeps_the_same_course_identity():
    ctx=course_context();avoid=Avoid(static_course=True)
    extra=ctx.route.points[(ctx.route.s>.5)&(ctx.route.s<=60.)]
    ctx.route=Route(np.vstack((ctx.route.points,extra)))
    assert avoid.course.target(ctx,avoid).chain == MOVES
    ctx.mission_name='STATIC_OBSTACLE';ctx.route=Route(np.c_[np.arange(100.),np.zeros(100)])
    assert avoid.course.target(ctx,avoid) is None
