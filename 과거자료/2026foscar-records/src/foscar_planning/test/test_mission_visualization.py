import math
import numpy as np
from builtin_interfaces.msg import Time
from foscar_control.core import Route
from foscar_planning.state import Behavior,Context,EventInfo,Target
from foscar_planning.states.yield_ import Yield
from foscar_planning.mission_markers import mission_markers, yield_status
from foscar_planning.prediction_markers import prediction_markers
from foscar_planning.prediction import GapPredictor


def scene(mission='PEDESTRIAN'):
    c=Context(s=450.,x=450.,y=0.,yaw=0.,v=0.,now=10.,mission_name=mission,
              mission_pedestrian=mission=='PEDESTRIAN',route=Route(np.c_[np.arange(1001),np.zeros(1001)]))
    b=Behavior([Yield()]);return c,b


def test_pedestrian_clear_confirmation_and_missing_observation_are_different():
    c,b=scene();y=b.states[0]
    b.engaged['YIELD']=True;y.target_type='PEDESTRIAN';y.target_s=455.;y.last_seen=9.9
    y.clear_since=9.7;y.reason='pedestrian'
    d=yield_status(b,c)
    assert d['kind']=='PEDESTRIAN' and d['phase']=='CLEAR_CONFIRMATION'
    assert 0<d['progress']<1
    y.clear_since=None
    d=yield_status(b,c)
    assert d['phase']=='OBSERVATION_MISSING' and d['progress'] is None
    arr=mission_markers(c,Target(state='YIELD',v_max=0.),b,{},Time())
    assert any('OBSERVATION MISSING' in m.text for m in arr.markers)


def test_vehicle_guard_is_not_labelled_pedestrian_yield():
    c,b=scene('ROUNDABOUT');y=b.states[0];b.engaged['YIELD']=True
    y.target_type='VEHICLE';y.target_s=455.;y.last_seen=10.;y.reason='VEHICLE'
    assert yield_status(b,c)['kind']=='VEHICLE_GUARD'
    a=mission_markers(c,Target(state='ROUNDABOUT_ENTER',v_max=0.,stop_s=449.),b,{},Time())
    assert 'VEHICLE GUARD' in '\n'.join(m.text for m in a.markers)


def test_normal_screen_has_only_relevant_object_and_large_short_status():
    c,b=scene('HIGHWAY')
    c.events=[EventInfo('PEDESTRIAN',10.,s=451.,d=0.,x=451.,size_x=.5,size_y=.5)]
    a=mission_markers(c,Target(),b,{},Time(),control={'stamp':10.,'target_speed_mps':12.,'limit_source':'PATH/STEERING'})
    labels=[m for m in a.markers if m.type==m.TEXT_VIEW_FACING]
    assert len(labels)==1 and labels[0].scale.z>=1.
    assert '43' in labels[0].text and 'PATH/STEERING' in labels[0].text
    assert not any(m.ns=='yield_object' for m in a.markers)


def test_old_control_telemetry_not_displayed_as_live_speed():
    c,b=scene('HIGHWAY')
    a=mission_markers(c,Target(),b,{},Time(),control={'stamp':1.,'target_speed_mps':12.})
    assert '43' not in '\n'.join(m.text for m in a.markers)


def test_prediction_markers_clear_other_missions_and_hide_rear_after_commit():
    c,b=scene('HIGHWAY');p=GapPredictor()
    a=prediction_markers(p,c,dict(decision='UNKNOWN'),Target(),Time())
    assert len(a.markers)==1 and a.markers[0].action==3
    c.mission_name='ROUNDABOUT';c.vehicle_following_active=True
    result=dict(model='circular_gap',decision='FOLLOW',following_model='measured_route_clearance',
                following_gap_m=6.,observed_corridor_polygons=[])
    a=prediction_markers(p,c,result,Target(state='ROUNDABOUT_FOLLOW'),Time())
    assert any(m.ns=='measured_corridor' for m in a.markers)
    assert not any(m.ns in ('rear_decision_panel','ring_occupied','decision') for m in a.markers)


def test_release_does_not_reuse_previously_blocking_object():
    c,b=scene();y=b.states[0];y.target_type='PEDESTRIAN';y.target_s=455.;y.last_seen=10.
    assert yield_status(b,c)['kind']=='NONE'
    a=mission_markers(c,Target(),b,{},Time())
    assert not any(m.ns=='yield_object' for m in a.markers)


def test_following_display_shows_actual_stop_clearance_not_rear_prediction():
    c,b=scene('ROUNDABOUT');c.vehicle_following_active=True
    a=mission_markers(c,Target(state='ROUNDABOUT_FOLLOW'),b,{'following_gap_m':6.},Time())
    line=next(m for m in a.markers if m.ns=='following_stop_boundary')
    assert all(abs(p.x-(c.x+3.8+.8))<1e-6 for p in line.points)


def test_observation_stale_brake_is_amber_not_a_pedestrian_request():
    c,b=scene('ROUNDABOUT');c.vehicle_following_active=True
    a=mission_markers(c,Target(state='ROUNDABOUT_BRAKE',v_max=0.),b,
                     dict(decision='FOLLOW',following_gap_m=10.6,evidence_stamp=9.53,evaluated_at=9.8),Time(),
                     gate_reason='imminent conflict')
    m=next(m for m in a.markers if m.ns=='mission_status')
    assert 'OBSERVATION STALE' in m.text and m.color.g>.5


def test_cached_front_surface_does_not_move_with_ego_and_stale_evidence_is_amber():
    c,b=scene('ROUNDABOUT');c.vehicle_following_active=True
    result=dict(following_gap_m=6.,following_reference_s=450.,evaluated_at=10.,evidence_stamp=10.)
    c.s=c.x=451.;c.now=10.1
    a=mission_markers(c,Target(state='ROUNDABOUT_FOLLOW'),b,result,Time())
    edge=next(m for m in a.markers if m.ns=='following_surface')
    assert all(abs(p.x-459.8)<1e-6 for p in edge.points)
    c.now=10.6
    a=mission_markers(c,Target(state='ROUNDABOUT_BRAKE',v_max=0.),b,result,Time(),gate_reason='imminent conflict')
    edge=next(m for m in a.markers if m.ns=='following_surface')
    assert edge.color.r==1. and .5<edge.color.g<.8


def test_disabled_roundabout_control_does_not_display_wait_veto():
    c,b=scene('ROUNDABOUT')
    a=mission_markers(c,Target(),b,{'display_gate_enabled':False},Time())
    assert 'ENTRY CONTROL OFF' in '\n'.join(m.text for m in a.markers)
    a=prediction_markers(GapPredictor(),c,{'gate_enabled':False},Target(),Time())
    assert len(a.markers)==1
