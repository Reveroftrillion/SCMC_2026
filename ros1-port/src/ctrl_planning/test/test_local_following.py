import math
from dataclasses import replace
from test_circular_gap import scene
from ctrl_planning.ctrl_local_following import evaluate_following


def test_live_footprint_distance_does_not_stop_for_inferred_truck_length():
    import numpy as np
    from ctrl_control.ctrl_core import Route
    gap,pred,ctx=scene(offsets=(14.,));ctx.s=0.;ctx.v=4.;gap.scan_stamp=ctx.now
    angles=np.linspace(0.,3.,601);ctx.route=Route(np.c_[20*np.cos(angles),20*np.sin(angles)])
    tr=gap.tracks['lidar:test:0'];tr.length=25.;tr.ready=False
    poly=[[r*np.cos(a),r*np.sin(a)] for r,a in [(19.,.5),(21.,.5),(21.,.9),(19.,.9)]]
    gap.footprints[tr.event.value]=(ctx.now,poly)
    result=evaluate_following(gap,ctx,pred)
    assert result['decision']=='FOLLOW'
    assert result['following_model']=='measured_route_clearance'
    assert result['following_gap_m']>4.
    assert result['following_speed_mps']>3.
    # A real return close to the front bumper still stops immediately.
    gap.footprints[tr.event.value]=(ctx.now,[[19.,2.],[21.,2.],[21.,4.],[19.,4.]])
    assert evaluate_following(gap,ctx,pred)['decision']=='WAIT'


def test_following_does_not_need_a_rear_pair_but_brakes_for_front():
    gap,pred,ctx=scene(offsets=(14.,))
    assert evaluate_following(gap,ctx,pred)['decision']=='FOLLOW'
    gap,pred,ctx=scene(offsets=(2.,))
    assert evaluate_following(gap,ctx,pred)['decision']=='WAIT'


def test_missing_sensor_updates_still_stops_following():
    gap,pred,ctx=scene();ctx.now=2.
    assert evaluate_following(gap,ctx,pred)['decision']=='UNKNOWN'


def test_trailing_vehicle_does_not_issue_ego_braking_after_admission():
    gap,pred,ctx=scene(offsets=(-10.,))
    gap.tracks['lidar:test:0'].speed=15.
    assert evaluate_following(gap,ctx,pred)['decision']=='FOLLOW'


def test_distant_off_circle_vehicle_does_not_stop_following():
    gap,pred,ctx=scene()
    gap.off_circle=['lidar:test:0']
    assert evaluate_following(gap,ctx,pred)['decision']=='FOLLOW'


def test_lost_lead_does_not_stop_when_observation_stream_is_live():
    gap,pred,ctx=scene()
    assert evaluate_following(gap,ctx,pred)['decision']=='FOLLOW'
    ctx.now=1.5
    rear=gap.tracks['lidar:test:1'];rear.event=replace(rear.event,stamp=1.5)
    result=evaluate_following(gap,ctx,pred)
    assert result['decision']=='FOLLOW'


def test_lost_front_blocker_does_not_stop_with_new_observations():
    gap,pred,ctx=scene(offsets=(2.,-16.))
    gap.tracks['lidar:test:0'].ready=False
    assert evaluate_following(gap,ctx,pred)['decision']=='WAIT'
    ctx.now=1.5
    rear=gap.tracks['lidar:test:1'];rear.event=replace(rear.event,stamp=1.5)
    assert evaluate_following(gap,ctx,pred)['decision']=='FOLLOW'


def test_admitted_front_loss_does_not_revoke_departure():
    from ctrl_planning.ctrl_local_following import begin_following
    gap,pred,ctx=scene();ctx.now=1.25
    begin_following(gap,dict(gap_pair=['lidar:test:1','lidar:test:0']))
    ctx.now=1.46
    rear=gap.tracks['lidar:test:1'];rear.event=replace(rear.event,stamp=1.46)
    assert evaluate_following(gap,ctx,pred)['decision']=='FOLLOW'


def test_admitted_rear_uncertainty_is_not_promoted_to_lead():
    from ctrl_planning.ctrl_local_following import begin_following
    gap,pred,ctx=scene(offsets=(14.,-6.))
    rear=gap.tracks['lidar:test:1'];rear.length=12.;rear.observed_length=1.
    begin_following(gap,dict(gap_pair=['lidar:test:1','lidar:test:0']))
    result=evaluate_following(gap,ctx,pred)
    assert result['decision']=='FOLLOW'
    assert gap.following_lead is gap.tracks['lidar:test:0']


def test_committed_rear_overlap_does_not_restart_braking():
    from ctrl_planning.ctrl_local_following import begin_following
    gap,pred,ctx=scene(offsets=(14.,-1.))
    begin_following(gap,dict(gap_pair=['lidar:test:1','lidar:test:0']))
    result=evaluate_following(gap,ctx,pred)
    assert result['decision']=='FOLLOW'
    assert gap.following_lead is gap.tracks['lidar:test:0']


def test_clear_exit_does_not_require_circulating_vehicle_observations():
    import math
    from types import SimpleNamespace
    gap,pred,ctx=scene();gap.tracks={}
    ctx.s=915.;ctx.x=30.;ctx.y=0.;ctx.yaw=0.
    ctx.route=SimpleNamespace(at=lambda s:(30.,0.),heading_at=lambda s:0.)
    result=evaluate_following(gap,ctx,pred,entered_ring=True)
    assert result['decision']=='EXITED'
    assert evaluate_following(gap,ctx,pred,entered_ring=False)['decision']=='UNKNOWN'
    ctx.yaw=math.pi
    assert evaluate_following(gap,ctx,pred,entered_ring=True)['decision']=='UNKNOWN'


def test_clear_exit_refreshes_between_predictions_before_stale_front_brake():
    import math
    from types import SimpleNamespace
    from ctrl_planning.ctrl_local_following import refresh_exit_prediction
    from ctrl_planning.ctrl_entry_gate import EntryGate
    from ctrl_planning.ctrl_state import Target
    gap,pred,ctx=scene()
    ctx.s=917.;ctx.x=30.;ctx.y=0.;ctx.yaw=0.;ctx.now=2.
    old=dict(model='circular_gap',admission_model='three_vehicle_entry',
             decision='FOLLOW',evaluated_at=1.8,evidence_stamp=1.5)
    gate=EntryGate(enabled=True);gate.committed=True;gate.entered_ring=True
    result=refresh_exit_prediction(old,ctx,gap.circle,entered_ring=True)
    target=gate.apply(Target(v_max=5.),ctx,result)
    assert result['decision']=='EXITED' and gate.finished
    assert target.v_max==5.
    # A red light/pedestrian stop is not cleared by the geometric exit.
    assert gate.apply(Target(v_max=0.,stop_s=918.),ctx,result).v_max==0.
    assert refresh_exit_prediction(old,ctx,gap.circle,entered_ring=False) is old
    ctx.x=20.;ctx.yaw=math.pi/2
    assert refresh_exit_prediction(old,ctx,gap.circle,entered_ring=True) is old


def test_empty_fresh_scan_continues_without_old_lead_or_rear_pair():
    gap,pred,ctx=scene()
    ctx.now=2.
    gap.following_lead=gap.tracks['lidar:test:0']
    gap.on_footprints(dict(frame='map',stamp=2.,objects=[]))
    result=evaluate_following(gap,ctx,pred,entered_ring=True)
    assert result['decision']=='FOLLOW'
    assert result['following_speed_mps']==math.inf          # no ring speed limit of its own (user 2026-09-29)
    assert result['evidence_stamp']==2.
    ctx.now=2.6
    assert evaluate_following(gap,ctx,pred,entered_ring=True)['decision']=='UNKNOWN'


def test_near_off_circle_object_still_blocks_actual_departure_path():
    gap,pred,ctx=scene(offsets=(2.,))
    gap.off_circle=['lidar:test:0']
    result=evaluate_following(gap,ctx,pred)
    assert result['decision']=='WAIT'
    assert result['conflict_in_s']==0.


def test_gate_keeps_committed_following_on_empty_scan_and_brakes_on_sensor_loss():
    from ctrl_planning.ctrl_entry_gate import EntryGate
    from ctrl_planning.ctrl_state import Target
    gap,pred,ctx=scene();ctx.now=2.
    gap.on_footprints(dict(frame='map',stamp=2.,objects=[]))
    gate=EntryGate(enabled=True);gate.committed=True;gate.entered_ring=True
    result=evaluate_following(gap,ctx,pred,entered_ring=True)
    assert gate.apply(Target(v_max=5.),ctx,result).v_max==5.
    assert gate.committed
    ctx.now=2.6
    result=evaluate_following(gap,ctx,pred,entered_ring=True)
    assert gate.apply(Target(v_max=5.),ctx,result).v_max==0.
    assert gate.committed


def test_new_identity_close_fast_rear_does_not_brake_committed_following():
    gap,pred,ctx=scene(offsets=(14.,-1.))
    rear=gap.tracks['lidar:test:1'];rear.speed=12.
    rear.event=replace(rear.event,value='lidar:new:rear')
    assert evaluate_following(gap,ctx,pred)['decision']=='FOLLOW'


def test_off_circle_rear_does_not_brake_committed_following():
    gap,pred,ctx=scene(offsets=(14.,-1.))
    gap.off_circle=['lidar:test:1']
    gap.tracks.pop('lidar:test:1')
    assert evaluate_following(gap,ctx,pred)['decision']=='FOLLOW'
