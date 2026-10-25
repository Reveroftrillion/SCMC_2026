import numpy as np
from foscar_control.core import Route
from foscar_planning.state import EventInfo,Context
from foscar_planning.prediction import GapPredictor
from foscar_planning.lane_prediction import LaneMap


def event(t,identity='old',x=None,y=15.,vx=3.,vy=0.):
    return EventInfo(type='VEHICLE',stamp=t,x=20+3*t if x is None else x,y=y,
                     vx=vx,vy=vy,size_x=4.8,size_y=2.,box_yaw=0.,value='lidar:test:'+identity)


def trained(lanes=None):
    p=GapPredictor(lane_map=lanes)
    for t in np.arange(0,1.01,.1):p.observe([event(float(t))],float(t))
    return p


def test_unique_consistent_id_change_keeps_motion_history_and_ignores_old_cache():
    p=trained();old=next(iter(p.tracks.values()));assert old.ready
    p.observe([event(1.),event(1.12,'new')],1.12)
    assert len(p.tracks)==1 and next(iter(p.tracks.values())) is old
    assert old.ready and old.event.value.endswith(':new')
    p.observe([event(1.),event(1.22,'new')],1.22)
    assert len(p.tracks)==1


def test_two_simultaneous_vehicles_or_ambiguous_matches_are_not_merged():
    p=trained()
    p.observe([event(1.12),event(1.12,'other',y=16.)],1.12)
    assert len(p.tracks)==2
    p=trained()
    p.observe([event(1.12,'a',y=15.3),event(1.12,'b',y=14.7)],1.12)
    assert len(p.tracks)==3


def test_different_velocity_or_long_gap_does_not_inherit_history():
    for t,vx in ((1.12,-3.),(2.,3.)):
        p=trained();old=next(iter(p.tracks.values()))
        p.observe([event(t,'new',vx=vx)],t)
        assert any(tr is not old and tr.event.value.endswith(':new') for tr in p.tracks.values())


def context(t):
    return Context(s=0,x=0,y=0,yaw=0,v=0,now=t,
                   route=Route(np.c_[np.arange(101),np.zeros(101)]))


def test_short_gap_requires_mature_map_track_and_fresh_independent_observations():
    lanes=LaneMap([dict(idx='lane',from_node_idx='a',to_node_idx='b',points=[[0,15],[200,15]])])
    p=trained(lanes)
    # A separate observed vehicle supplies new evidence; the missing old one is
    # still checked with an enlarged body, not dropped from collision checks.
    for t in np.arange(1.,1.701,.1):p.observe([event(round(float(t),2),'fresh',x=80+3*t)],float(t))
    result=p.evaluate(context(1.7))
    assert result['decision']=='CANDIDATE_GAP'
    assert result['bridged_tracks']==1 and result['evidence_stamp']>1.6
    assert p.evaluate(context(1.9))['decision']=='UNKNOWN'
    q=trained()
    assert q.evaluate(context(1.7))['decision']=='UNKNOWN'


def test_bridged_track_still_blocks_if_its_uncertain_body_conflicts():
    lanes=LaneMap([dict(idx='lane',from_node_idx='a',to_node_idx='b',points=[[0,0],[200,0]])])
    p=GapPredictor(lane_map=lanes)
    for t in np.arange(0,1.01,.1):p.observe([event(float(t),x=5+t,y=0.,vx=1.)],float(t))
    assert p.evaluate(context(1.7))['decision']=='WAIT'


def test_no_fresh_observation_still_holds_even_with_complete_map_and_short_gap():
    lanes=LaneMap([dict(idx='lane',from_node_idx='a',to_node_idx='b',points=[[0,15],[200,15]])])
    p=trained(lanes)
    assert p.evaluate(context(1.7))['decision']=='UNKNOWN'


def test_history_transfer_preserves_largest_observed_body():
    p=trained();old=next(iter(p.tracks.values()));old.length=7.;old.width=2.5
    e=event(1.12,'new');e.size_x=.5;e.size_y=.5
    p.observe([e],1.12)
    tr=next(iter(p.tracks.values()))
    assert len(p.tracks)==1 and tr.length>=7. and tr.width>=2.5
