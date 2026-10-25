import math
import numpy as np
from ctrl_control.ctrl_core import Route
from ctrl_planning.ctrl_state import Context, EventInfo
from ctrl_planning.ctrl_prediction import VehicleTrack, GapPredictor, integrate_turn, rectangles_overlap


def test_turn_model_matches_quarter_circle_and_straight_limit():
    x,y,h=integrate_turn(0,0,0,2,1,math.pi/2)
    assert abs(x-2)<1e-6 and abs(y-2)<1e-6
    assert abs(h-math.pi/2)<1e-6
    assert integrate_turn(1,2,0,3,0,2)==(7,2,0)


def event(t, x, y, vx=0., vy=0., sx=4.8, sy=2.):
    return EventInfo(type='VEHICLE',stamp=t,s=x,d=y,x=x,y=y,vx=vx,vy=vy,size_x=sx,size_y=sy,value='lidar:test:1')


def test_track_does_not_shrink_with_partial_view_or_age_on_repeated_cache():
    tr=VehicleTrack()
    for t in (0.,.2,.4,.6,.8):tr.update(event(t,10+t*3,0,3,0))
    size=tr.radius
    tr.update(event(1.,13,0,3,0,sx=.4,sy=.3))
    assert tr.radius>=size
    count=len(tr.samples)
    tr.update(event(1.,13,0,3,0))
    assert len(tr.samples)==count
    assert tr.ready


def test_turn_rate_uses_motion_direction_and_handles_angle_wrap():
    tr=VehicleTrack()
    for t in np.arange(0,1.5,.1):
        h=3.0+.2*t
        tr.update(event(float(t),10,0,4*math.cos(h),4*math.sin(h)))
    assert tr.ready
    assert abs(tr.turn_rate-.2)<.02


def test_rectangle_overlap_includes_rear_and_full_other_body():
    assert rectangles_overlap((0,0,0,4.8,2.),(-3,0,math.pi/2,4.8,2.))
    assert not rectangles_overlap((0,0,0,4.8,2.),(0,5,0,4.8,2.))


def ctx(t=1.):
    route=Route(np.c_[np.linspace(0,100,201),np.zeros(201)])
    return Context(s=0,x=0,y=0,yaw=0,v=0,now=t,route=route)


def test_crossing_vehicle_conflicts_but_far_departing_vehicle_does_not():
    p=GapPredictor()
    for t in np.arange(0,1.01,.1):p.observe([event(float(t),8,8-2*t,0,-2)],float(t))
    assert p.evaluate(ctx())['decision']=='WAIT'
    p=GapPredictor()
    for t in np.arange(0,1.01,.1):p.observe([event(float(t),20,12+2*t,0,2)],float(t))
    assert p.evaluate(ctx())['decision']=='CANDIDATE_GAP'


def test_missing_observations_never_mean_permission_to_go():
    p=GapPredictor()
    assert p.evaluate(ctx())['decision']=='UNKNOWN'
    for t in np.arange(0,1.01,.1):p.observe([event(float(t),20,12+2*t,0,2)],float(t))
    assert p.evaluate(ctx(2.))['decision']=='UNKNOWN'
    assert p.evaluate(ctx(4.))['decision']=='UNKNOWN'


def test_map_candidates_are_added_without_removing_unmodeled_motion():
    from ctrl_planning.ctrl_lane_prediction import LaneMap
    lanes=LaneMap([dict(idx='road',from_node_idx='a',to_node_idx='b',points=[[0,10],[100,10]])])
    tr=VehicleTrack(lane_map=lanes)
    for t in np.arange(0,1.01,.1):tr.update(event(float(t),20+2*t,10,2,0))
    assert tr.lane_motion.complete(3.)
    assert len(list(tr.poses(3.)))>=4
    p=GapPredictor(lane_map=lanes)
    p.observe([event(1.,25,10,2,0)],1.)
    result=p.evaluate(ctx())
    assert result['mode']=='shadow'
    assert result['lane_tracks']==1


def test_incomplete_map_coverage_cannot_report_a_candidate_gap():
    from ctrl_planning.ctrl_lane_prediction import LaneMap
    lanes=LaneMap([dict(idx='road',from_node_idx='a',to_node_idx='b',points=[[20,10],[25,10]])])
    p=GapPredictor(lane_map=lanes)
    for t in np.arange(0,1.01,.1):p.observe([event(float(t),21+t,10,1,0)],float(t))
    assert p.evaluate(ctx())['decision']=='UNKNOWN'


def test_prediction_reports_stale_track_evidence_for_entry_control():
    p=GapPredictor()
    for t in np.arange(0,1.01,.1):p.observe([event(float(t),20,12+2*t,0,2)],float(t))
    result=p.evaluate(ctx(2.))
    assert result['evaluated_at']==2.
    assert result['evidence_stamp']==1.
    assert 'stale' in result['uncertainties']['lidar:test:1']
