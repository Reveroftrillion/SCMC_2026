import math
from foscar_planning.state import Context, Target
from foscar_planning.entry_gate import EntryGate


def ctx(t=1.,s=880.,v=0.):
    return Context(s=s,x=0,y=0,yaw=0,v=v,now=t)


def evidence(t,decision='CANDIDATE_GAP',stamp=None):
    return dict(evaluated_at=t,evidence_stamp=t if stamp is None else stamp,
                decision=decision,lane_tracks=1,tracks=1,conflict_in_s=1. if decision=='WAIT' else math.inf)


def test_disabled_and_other_missions_are_unchanged():
    target=Target(v_max=10.,lat_offset=2.)
    assert EntryGate().apply(target,ctx(),None)==target
    assert EntryGate(enabled=True).apply(target,ctx(s=460),None)==target


def test_unknown_and_conflict_hold_at_entry_and_preserve_stricter_target():
    gate=EntryGate(enabled=True)
    for data in (None,evidence(1.,'WAIT'),evidence(1.,'UNKNOWN')):
        result=gate.apply(Target(),ctx(),data)
        assert result.v_max==0 and result.state=='ROUNDABOUT_WAIT'
    target=Target(state='STOP_LINE',v_max=0.,stop_s=870.,lat_offset=1.)
    result=gate.apply(target,ctx(),None)
    assert result.stop_s==870 and result.lat_offset==1.


def test_repeated_cached_clear_evidence_does_not_release():
    gate=EntryGate(enabled=True)
    for t in (1.,1.2,1.4,1.6,1.8):
        result=gate.apply(Target(),ctx(t),evidence(t,stamp=1.))
        assert result.v_max==0


def test_fresh_clear_evidence_releases_and_new_conflict_revokes_before_merge():
    gate=EntryGate(enabled=True)
    for t in (1.,1.2,1.4,1.6,1.8):
        result=gate.apply(Target(),ctx(t),evidence(t))
    assert result.v_max==20/3.6 and math.isnan(result.stop_s)
    result=gate.apply(Target(),ctx(2.),evidence(2.,'WAIT'))
    assert result.v_max==0


def test_stale_prediction_or_missing_map_never_releases():
    for bad in (evidence(0.),dict(evidence(1.),lane_tracks=0),dict(evidence(1.),evaluated_at=2.)):
        gate=EntryGate(enabled=True)
        assert gate.apply(Target(),ctx(),bad).v_max==0


def test_merge_does_not_reapply_entry_stop_but_preserves_emergency_and_respawn_reset():
    gate=EntryGate(enabled=True)
    for t in (1.,1.2,1.4,1.6,1.8):gate.apply(Target(),ctx(t),evidence(t))
    result=gate.apply(Target(),ctx(2.,s=887.,v=3.),evidence(2.,'UNKNOWN'))
    assert math.isnan(result.stop_s) and result.v_max>0
    result=gate.apply(Target(),ctx(2.2,s=888.,v=3.),evidence(2.2,'WAIT'))
    assert result.v_max==0
    assert gate.apply(Target(),ctx(3.,s=880.),None).v_max==0


def test_node_enabled_gate_restricts_published_target_before_transmission():
    import rclpy
    from nav_msgs.msg import Odometry
    from foscar_planning.behavior_node import BehaviorNode
    rclpy.init(args=['--ros-args','-p','roundabout.enabled:=true','-p','markers:=false'])
    node=BehaviorNode()
    published=[]
    class Capture:
        def publish(self,msg):published.append(msg)
    try:
        node.target_pub=Capture()
        node.on_odom(Odometry())
        node.s_global=880.
        node.tick()
        assert published[-1].state=='ROUNDABOUT_WAIT'
        assert published[-1].v_max==0.
        assert node.prediction_result['decision']=='UNKNOWN'
    finally:
        node.destroy_node();rclpy.shutdown()


def test_other_stop_is_not_released_by_confirmed_gap():
    gate=EntryGate(enabled=True)
    target=Target(state='YIELD',v_max=0.,stop_s=879.)
    for t in (1.,1.2,1.4,1.6,1.8):
        result=gate.apply(target,ctx(t),evidence(t))
    assert gate.permitted
    assert result.v_max==0. and result.stop_s==879. and result.state=='YIELD'


def test_gap_on_centerline_does_not_authorize_an_unmodeled_avoidance_path():
    gate=EntryGate(enabled=True)
    target=Target(state='AVOID',v_max=5.,lat_offset=2.)
    for t in (1.,1.2,1.4,1.6,1.8):
        result=gate.apply(target,ctx(t),evidence(t))
    assert result.v_max==0.
    assert not gate.permitted


def test_distant_approach_does_not_crawl_at_staging_speed():
    gate=EntryGate(enabled=True)
    c=Context(s=865.,x=0.,y=0.,yaw=0.,v=0.,now=1.)
    d=dict(model='circular_gap',decision='UNKNOWN',evaluated_at=1.,evidence_stamp=1.,staging_s=886.)
    result=gate.apply(Target(),c,d)
    assert result.v_max>2. and result.stop_s<=883.


def circular_evidence(t,stage=886.,decision='CANDIDATE_GAP'):
    return dict(evidence(t,decision),model='circular_gap',staging_s=stage,
                cooperative_rear=True,gap_pair=['rear','front'],ego_overlaps_ring=False)


def test_circular_entry_waits_for_standstill_at_latched_stage():
    gate=EntryGate(enabled=True)
    for t in (1.,1.2,1.4):
        out=gate.apply(Target(),ctx(t,s=884.,v=.8),circular_evidence(t))
        assert out.state=='ROUNDABOUT_APPROACH' and not gate.permitted
    for t in (1.6,1.8,2.):
        out=gate.apply(Target(),ctx(t,s=885.8,v=0.),circular_evidence(t))
    assert gate.permitted and out.state=='ROUNDABOUT_ENTER'


def test_waiting_stage_cannot_creep_forward_with_fluctuating_observations():
    gate=EntryGate(enabled=True)
    gate.apply(Target(),ctx(1.,s=885.8),circular_evidence(1.,decision='WAIT'))
    out=gate.apply(Target(),ctx(1.2,s=885.8),circular_evidence(1.2,stage=887.,decision='WAIT'))
    assert out.v_max==0. and out.stop_s<=889.
    # A newly stricter bound still applies, without attempting to reverse.
    out=gate.apply(Target(),ctx(1.4,s=885.8),circular_evidence(1.4,stage=885.,decision='WAIT'))
    assert out.v_max==0.


def test_cached_gap_before_stop_does_not_count_as_stationary_confirmation():
    gate=EntryGate(enabled=True)
    for t in (1.,1.2,1.4):gate.apply(Target(),ctx(t,s=885.8,v=.5),circular_evidence(t))
    out=gate.apply(Target(),ctx(1.5,s=885.8,v=0.),circular_evidence(1.5))
    assert not gate.permitted and out.v_max==0.


def test_confirmed_stationary_departure_continues_when_speed_increases():
    gate=EntryGate(enabled=True)
    for t in (1.,1.2,1.4):gate.apply(Target(),ctx(t,s=885.8),circular_evidence(t))
    assert gate.permitted
    out=gate.apply(Target(),ctx(1.5,s=885.9,v=.4),circular_evidence(1.5))
    assert out.state=='ROUNDABOUT_ENTER' and out.v_max>0.
    out=gate.apply(Target(),ctx(1.6,s=886.,v=.5),circular_evidence(1.6,decision='WAIT'))
    assert out.v_max==0. and not gate.stopped_at_entry


def test_changed_gap_during_departure_requires_stopping_again_before_commit():
    gate=EntryGate(enabled=True)
    for t in (1.,1.2,1.4):gate.apply(Target(),ctx(t,s=885.8),circular_evidence(t))
    for t in (1.5,1.7,1.9):
        data=dict(circular_evidence(t),gap_pair=['new_rear','front'])
        out=gate.apply(Target(),ctx(t,s=885.9,v=.4),data)
        assert not gate.permitted and out.v_max==0.


def test_local_entry_confirms_two_fresh_stationary_observations():
    gate=EntryGate(enabled=True)
    for t in (1.,1.25):
        data=dict(circular_evidence(t),cooperative_rear=False,admission_model='three_vehicle_entry')
        out=gate.apply(Target(),ctx(t,s=885.8),data)
    assert gate.permitted and out.state=='ROUNDABOUT_ENTER'


def test_local_launch_is_latched_before_ring_contact_and_external_stop_survives():
    gate=EntryGate(enabled=True)
    for t in (1.,1.25):
        data=dict(circular_evidence(t),cooperative_rear=False,admission_model='three_vehicle_entry')
        gate.apply(Target(),ctx(t,s=885.8),data)
    assert gate.committed
    # Re-evaluating a departure gap is no longer permission to brake on launch.
    data=dict(circular_evidence(1.4,decision='WAIT'),admission_model='three_vehicle_entry',
              cooperative_rear=False,conflict_side='rear',conflict_in_s=1.)
    assert gate.apply(Target(),ctx(1.4,s=885.8,v=.2),data).v_max>0.
    assert gate.apply(Target(state='YIELD',v_max=0.),ctx(1.4,s=885.8,v=.2),data).v_max==0.


def test_interrupted_local_launch_must_recheck_rear_before_moving():
    gate=EntryGate(enabled=True)
    for t in (1.,1.25):
        d=dict(circular_evidence(t),cooperative_rear=False,admission_model='three_vehicle_entry')
        gate.apply(Target(),ctx(t,s=885.8),d)
    assert gate.committed
    d=dict(circular_evidence(1.4),admission_model='three_vehicle_entry',phase='following',decision='FOLLOW')
    gate.apply(Target(state='YIELD',v_max=0.),ctx(1.4,s=885.8),d)
    assert not gate.committed
    d=dict(circular_evidence(3.,decision='WAIT'),admission_model='three_vehicle_entry',conflict_side='rear')
    assert gate.apply(Target(),ctx(3.,s=885.8),d).v_max==0.


def test_launch_without_motion_expires_before_ring_contact():
    gate=EntryGate(enabled=True)
    for t in (1.,1.25):gate.apply(Target(),ctx(t,s=885.8),dict(circular_evidence(t),admission_model='three_vehicle_entry'))
    d=dict(circular_evidence(2.),admission_model='three_vehicle_entry',phase='following',decision='FOLLOW')
    gate.apply(Target(),ctx(2.,s=885.8),d)
    assert not gate.committed


def test_local_admission_uses_one_latest_decision_on_established_tracks():
    gate=EntryGate(enabled=True)
    d=dict(circular_evidence(1.),admission_model='three_vehicle_entry',cooperative_rear=False)
    assert gate.apply(Target(),ctx(1.,s=885.8),d).state=='ROUNDABOUT_ENTER'
    assert gate.committed
    gate=EntryGate(enabled=True)
    d['evidence_stamp']=.54
    assert gate.apply(Target(),ctx(1.,s=885.8),d).v_max==0.


def test_sensor_loss_after_ring_entry_does_not_reset_to_admission():
    gate=EntryGate(enabled=True)
    d=dict(circular_evidence(1.),admission_model='three_vehicle_entry')
    gate.apply(Target(),ctx(1.,s=885.8),d)
    d=dict(circular_evidence(1.2),admission_model='three_vehicle_entry',phase='following',decision='FOLLOW',ego_overlaps_ring=True)
    gate.apply(Target(),ctx(1.2,s=889.,v=2.),d)
    d=dict(model='circular_gap',admission_model='three_vehicle_entry',phase='following',decision='UNKNOWN',
           evaluated_at=2.,evidence_stamp=0.,ego_overlaps_ring=False)
    result=gate.apply(Target(),ctx(2.,s=891.,v=1.),d)
    assert gate.committed and result.state=='ROUNDABOUT_BRAKE' and result.v_max==0.


def test_completed_local_exit_releases_gate_without_releasing_other_stop():
    gate=EntryGate(enabled=True);gate.committed=True;gate.entered_ring=True
    d=dict(model='circular_gap',admission_model='three_vehicle_entry',decision='EXITED',evaluated_at=1.)
    target=Target(v_max=4.)
    assert gate.apply(target,ctx(1.,s=915.),d)==target
    assert not gate.owns_vehicle_prediction(ctx(1.,s=915.))
    stop=Target(state='YIELD',v_max=0.)
    assert gate.apply(stop,ctx(1.2,s=915.),None)==stop


def test_stationary_wait_pose_does_not_toggle_approach_on_small_map_jitter():
    gate=EntryGate(enabled=True)
    ctx=Context(886.48,0.,0.,0.,0.,1.)
    for i,delta in enumerate((.21,.19,.22,.18)):
        ctx.now=1.+.1*i
        data=dict(model='circular_gap',admission_model='three_vehicle_entry',
                  evaluated_at=ctx.now,evidence_stamp=ctx.now,decision='WAIT',
                  staging_s=886.48+delta)
        result=gate.apply(Target(),ctx,data)
        assert result.state=='ROUNDABOUT_WAIT'
        assert result.v_max==0.
        assert abs(result.stop_s-889.48)<1e-6


def test_fixed_gps_waiting_point_overrides_old_entry_and_dynamic_stage():
    gate=EntryGate(enabled=True,stop_at_s=885.712)
    for s in (870.,880.,884.):
        data=dict(evidence(1.,'WAIT'),model='circular_gap',staging_s=888.)
        target=gate.apply(Target(v_max=10.),ctx(s=s,v=1.),data)
        assert target.stop_s==888.712
    data=dict(evidence(1.,'WAIT'),model='circular_gap',admission_model='three_vehicle_entry',staging_s=880.)
    assert gate.apply(Target(),ctx(s=880.),data).v_max>0
    assert not gate.committed
    assert gate.apply(Target(),ctx(s=885.712,v=1.),data).v_max==0
    assert not gate.committed
    data['decision']='CANDIDATE_GAP'
    assert gate.apply(Target(),ctx(s=885.712,v=1.),data).v_max>0
    assert gate.committed


def test_local_gate_uses_same_freshness_limit_as_boundary_prediction():
    for age,allowed in ((.33,True),(.46,False)):
        gate=EntryGate(enabled=True)
        data=dict(evidence(1.,stamp=1.-age),model='circular_gap',admission_model='three_vehicle_entry')
        target=gate.apply(Target(),ctx(),data)
        assert (target.v_max>0)==allowed


def test_ring_not_visible_yet_holds_speed_until_the_last_stop_point():
    # 2026-09-27 laps: waiting only for visibility braked 54 -> 30 km/h over s834-859.
    g=EntryGate(enabled=True,approach_s=830.,entry_s=880.,merge_s=886.,end_s=920.,max_speed_kmh=60.,late_brake_mps2=4.)
    unseen=dict(decision='UNKNOWN',evaluated_at=1.,evidence_stamp=1.,lane_tracks=0,model='circular_gap')
    far=g.apply(Target(v_max=60/3.6),ctx(s=835.,v=15.),unseen)
    assert far.state=='ROUNDABOUT_WAIT' and not math.isfinite(far.stop_s) and far.v_max>14.
    near=g.apply(Target(v_max=60/3.6),ctx(s=850.,v=15.),unseen)      # 30 m left < 3.75+28+1
    assert math.isfinite(near.stop_s)
    # User 2026-09-29: with cars in the ring too, brake late and carry the speed -- the stop only from the last point.
    seen=dict(unseen,decision='WAIT',lane_tracks=2,conflict_in_s=3.)
    h=EntryGate(enabled=True,approach_s=830.,entry_s=880.,merge_s=886.,end_s=920.,max_speed_kmh=60.,late_brake_mps2=4.)
    assert not math.isfinite(h.apply(Target(v_max=60/3.6),ctx(s=835.,v=15.),seen).stop_s)
    assert math.isfinite(h.apply(Target(v_max=60/3.6),ctx(s=850.,v=15.),seen).stop_s)
