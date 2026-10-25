import math
import numpy as np
from builtin_interfaces.msg import Time
from foscar_control.core import Route
from foscar_planning.state import Context,EventInfo,Target
from foscar_planning.prediction import GapPredictor
from foscar_planning.prediction_markers import prediction_markers
from foscar_planning.circle_reference import CircularReference
from foscar_planning.lane_prediction import LaneMap


def test_empty_and_single_vehicle_admission_render_without_pair_boundaries():
    from test_circular_gap import scene
    gap,pred,ctx=scene(offsets=())
    for pair in ([],['lidar:single']):
        result=dict(model='circular_gap',decision='CANDIDATE_GAP',reason='verified corridor',
                    gap_pair=pair,empty_lane=not pair)
        assert prediction_markers(pred,ctx,result,Target(),Time(),circle=gap.circle).markers


def test_closed_map_ring_fits_circle_and_rejects_incomplete_arc():
    lanes=[]
    for i in range(4):
        a=np.linspace(i*math.pi/2,(i+1)*math.pi/2,21)
        lanes.append(dict(idx=str(i),from_node_idx=str(i),to_node_idx=str((i+1)%4),
                          points=np.c_[10+18*np.cos(a),20+18*np.sin(a)].tolist()))
    model=LaneMap(lanes)
    circle=CircularReference.from_lanes(model,['0','1','2','3'])
    assert np.allclose(circle.center,[10,20]) and abs(circle.radius-18)<1e-6
    assert circle.rms<1e-6
    assert CircularReference.from_lanes(model,['0']) is None


def test_visualization_distinguishes_observed_and_assumed_vehicle_size_and_intent():
    e=EventInfo(type='DYNAMIC_UNKNOWN',stamp=1.,x=8.,y=5.,s=8.,d=5.,
                size_x=1.3,size_y=1.4,box_yaw=0.,vx=3.,value='lidar:test:12',path_relevant=False)
    ctx=Context(s=0,x=0,y=0,yaw=0,v=0,now=1.,observations=[e],
                route=Route(np.c_[np.arange(101),np.zeros(101)]))
    predictor=GapPredictor();predictor.observe([e],1.)
    arr=prediction_markers(predictor,ctx,dict(decision='UNKNOWN',reason='cold motion'),
                           Target(state='ROUNDABOUT_WAIT',v_max=0.),Time(sec=1),'UNKNOWN',show_details=True)
    assert arr.markers[0].action==3  # remove stale predictions
    labels='\n'.join(m.text for m in arr.markers)
    assert 'DYNAMIC_UNKNOWN' in labels and '1.30x1.40' in labels
    assert '4.80x2.00' in labels and 'OBS ONLY' in labels
    assert 'command cap' in labels and 'ROUNDABOUT_WAIT' in labels
    boxes=[m for m in arr.markers if m.ns=='prediction_body']
    assert boxes and boxes[0].scale.x>4.8 and boxes[0].scale.y>2.
    assert not any(m.ns in ('ego_future','ego_departure_hypotheses','object_future') for m in arr.markers)
    assert all(m.header.frame_id=='map' for m in arr.markers[1:])


def test_no_route_or_no_tracks_still_displays_reason_without_inventing_free_space():
    ctx=Context(s=0,x=0,y=0,yaw=0,v=0,now=1.)
    arr=prediction_markers(GapPredictor(),ctx,dict(decision='UNKNOWN',reason='no tracked vehicle'),
                           Target(),Time(sec=1),'UNKNOWN',show_details=True)
    labels='\n'.join(m.text for m in arr.markers)
    assert 'no tracked vehicle' in labels
    assert not any(m.ns=='ego_future' for m in arr.markers)


def test_unknown_stale_body_conflict_is_visible_with_age_and_observed_size():
    ctx=Context(s=0,x=0,y=0,yaw=0,v=0,now=10.)
    result=dict(model='circular_gap',decision='UNKNOWN',reason='stale body',
        conflict_xy=[3.,4.],conflict_in_s=2.,conflict_side='other',track='lidar:test:7',
        occupied_intervals=[dict(id='lidar:test:7',age=6.,observed_length=.4,length=4.8)])
    arr=prediction_markers(GapPredictor(),ctx,result,Target(),Time(sec=10),show_details=True)
    label='\n'.join(m.text for m in arr.markers if m.ns=='conflict_label')
    assert 'other' in label and '6.00s' in label and '0.40m' in label
    assert any(m.ns=='conflict' for m in arr.markers)


def test_rear_rejection_highlights_actual_rear_and_displays_negative_reserve():
    ctx=Context(s=880,x=0,y=0,yaw=0,v=2,now=10.,observations=[
        EventInfo('VEHICLE',10.,x=-5.,y=1.,value='lidar:rear')])
    result=dict(model='circular_gap',decision='WAIT',conflict_side='rear',
                reason='rear following clearance exhausted after handoff',
                gap_pair=['lidar:rear','lidar:front'],handoff_rear_reserve_m=-.42)
    arr=prediction_markers(GapPredictor(),ctx,result,Target(state='ROUNDABOUT_WAIT',v_max=0.),Time(),show_details=True)
    status=next(m for m in arr.markers if m.ns=='decision')
    assert 'REAR TOO CLOSE AFTER MERGE' in status.text and '-0.42m' in status.text
    assert 'rear' in status.text and status.color.r>status.color.g
    assert any(m.ns=='blocking_object' for m in arr.markers)
    assert not any(m.ns=='conflict_ego' for m in arr.markers)


def test_candidate_does_not_claim_go_when_another_behavior_requires_stop():
    ctx=Context(s=880,x=0,y=0,yaw=0,v=0,now=10.)
    result=dict(decision='CANDIDATE_GAP',reason='fresh gap confirmed',
                upstream_state='YIELD',upstream_stop=True)
    arr=prediction_markers(GapPredictor(),ctx,result,Target(state='YIELD',v_max=0.),Time())
    status=next(m.text for m in arr.markers if m.ns=='decision')
    assert status=='OTHER STOP'


def test_candidate_with_old_evidence_explains_gate_rejection():
    from foscar_planning.prediction_markers import decision_label
    ctx=Context(s=880,x=0,y=0,yaw=0,v=0,now=10.)
    text,_=decision_label(ctx,dict(decision='CANDIDATE_GAP',admission_model='three_vehicle_entry',
                                 evidence_stamp=9.7),Target(state='ROUNDABOUT_WAIT'),'CANDIDATE_GAP')
    assert 'OBSERVATION TOO OLD' in text and '0.30s' in text


def test_ring_display_uses_observed_outline_and_tangent_edges_instead_of_box():
    circle=CircularReference(np.zeros(2),20.,0.,0.,1)
    a=.7;h=a+math.pi/2;center=20*np.array([math.cos(a),math.sin(a)])
    axes=np.array([[math.cos(h),-math.sin(h)],[math.sin(h),math.cos(h)]])
    points=np.array([[-5.,-1.3],[5.,-1.3],[5.,1.3],[-5.,1.3]])@axes.T+center
    row=dict(id='truck',age=0.,ready=True,lo=9.,hi=19.,s=14.,length=10.,speed=2.,polygon_xy=points.tolist())
    ctx=Context(s=880,x=0,y=0,yaw=0,v=0,now=1.)
    result=dict(model='circular_gap',decision='UNKNOWN',occupied_intervals=[row])
    arr=prediction_markers(GapPredictor(),ctx,result,Target(),Time(),circle=circle)
    assert not any(m.ns=='ring_observed_box' for m in arr.markers)
    edges=[m for m in arr.markers if m.ns=='measured_boundary']
    assert len(edges)==2
    for edge in edges:
        delta=np.array([edge.points[1].x-edge.points[0].x,edge.points[1].y-edge.points[0].y])
        assert abs(delta@axes[:,0])<1e-6
        assert abs(np.linalg.norm(delta)-2.6)<1e-6


def test_rear_panel_shows_speed_shortfall_and_separate_margin_bands():
    from test_circular_gap import scene
    gap,pred,ctx=scene(offsets=(14.,-3.));gap.local_entry=True
    result=gap.evaluate(ctx,pred)
    arr=prediction_markers(pred,ctx,result,Target(state='ROUNDABOUT_WAIT'),Time(),circle=gap.circle)
    panel=next(m.text for m in arr.markers if m.ns=='rear_decision_panel')
    assert '18 km/h' in panel and 'SHORT' in panel
    assert len(panel.splitlines())<=3
    assert any(m.ns=='rear_safety_margin' for m in arr.markers)
    assert any(m.ns=='ego_rear_at_check' for m in arr.markers)
    labels=[m for m in arr.markers if m.type==m.TEXT_VIEW_FACING]
    assert len(labels)<=2 and all(m.scale.z>=1. for m in labels)
