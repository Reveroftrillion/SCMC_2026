import math
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
from ctrl_planning.ctrl_circle_reference import CircularReference
from ctrl_planning.ctrl_prediction import GapPredictor
from ctrl_planning.ctrl_circular_gap import CircularGap
from ctrl_planning.ctrl_state import EventInfo, Context, Target
from ctrl_planning.ctrl_entry_gate import EntryGate


def scene(offsets=(14.,-16.),length=4.8,exact_ends=True):
    circle=CircularReference(np.array([0.,0.]),20.,0.,0.,1)
    pred=GapPredictor(); ring=CircularGap(circle)
    for t in np.arange(0.,1.01,.1):
        es=[]
        for i,s in enumerate(offsets):
            a=(s+5*(t-1))/20; h=a+math.pi/2
            es.append(EventInfo('VEHICLE',float(t),x=20*math.cos(a),y=20*math.sin(a),
                vx=5*math.cos(h),vy=5*math.sin(h),size_x=length,size_y=2.,box_yaw=h,value=f'lidar:test:{i}'))
        pred.observe(es,float(t));ring.observe(pred,float(t))
    # These admission tests use ground-truth ends and motion. Raw-hull evidence
    # and missing-boundary rejection are tested separately; never enable a
    # production AABB fallback merely to support this synthetic fixture.
    class ExactMotion:
        def __init__(self,tr):self.tr=tr
        ready=True
        @property
        def speed(self):return self.tr.speed
        @property
        def stamp(self):return self.tr.event.stamp
    for tr in (ring.tracks.values() if exact_ends else []):
        tr.has_polygon=True;tr.arc_lo=-length/2;tr.arc_hi=length/2
        tr.front_observed=tr.rear_observed=True
        tr.front_motion=tr.rear_motion=ExactMotion(tr)
        tr.observe_boundaries=lambda sensor,circle:None
    # A short entry, circular travel, then exit. Ground-truth test profile.
    def ego(ctx,dt,accel=1.):
        a=5*dt/20; r=20. if dt<4. else 30.
        return (r*math.cos(a),r*math.sin(a),a+math.pi/2,4.635,1.892)
    pred.ego_pose=ego
    ctx=Context(s=880.,x=20.,y=0.,yaw=math.pi/2,v=5.,now=1.,route=SimpleNamespace(at=lambda s:(20.,0.),heading_at=lambda s:math.pi/2))
    return ring,pred,ctx


def test_clear_pair_bounds_entire_ego_interaction():
    ring,pred,ctx=scene()
    d=ring.evaluate(ctx,pred)
    assert d['decision']=='CANDIDATE_GAP'
    assert len(d['gap_pair'])==2 and d['minimum_gap_margin_m']>0
    assert d['interaction_end_s']>=3.8


def test_rear_diagnostic_exposes_actual_collision_margin_components():
    ring,pred,ctx=scene(offsets=(14.,-3.));ring.local_entry=True
    result=ring.evaluate(ctx,pred);d=result['rear_check']
    assert abs(d['speed_mps']-5.)<1e-8
    assert d['clearance_m']<=0.
    assert abs(d['body_gap_m']-d['required_margin_m']-d['clearance_m'])<1e-6
    assert abs(d['base_margin_m']+d['future_margin_m']+d['age_margin_m']+d['ego_margin_m']-d['required_margin_m'])<1e-6


def test_raw_diagonal_truck_uses_tangent_dimensions_before_length_history():
    circle=CircularReference(np.zeros(2),20.,0.,0.,1)
    ring=CircularGap(circle);ring.local_entry=True;pred=GapPredictor()
    a=.7;h=a+math.pi/2;center=20*np.array([math.cos(a),math.sin(a)])
    axes=np.array([[math.cos(h),-math.sin(h)],[math.sin(h),math.cos(h)]])
    hull=np.array([[-5.,-1.3],[5.,-1.3],[5.,1.3],[-5.,1.3]])@axes.T+center
    ring.on_footprints(dict(frame='map',stamp=1.,objects=[dict(id='truck',xy=hull.tolist(),
        center=center.tolist(),vx=2*math.cos(h),vy=2*math.sin(h),on_road=True)]))
    e=ring.raw_events['truck']
    assert abs(e.size_x-10.)<1e-6 and abs(e.size_y-2.6)<1e-6
    assert abs(e.box_yaw-h)<1e-6
    ring.observe(pred,1.)
    assert ring.tracks['truck'].length<11.


def test_tight_gap_and_long_vehicle_block():
    ring,pred,ctx=scene(offsets=(4.,-6.))
    assert ring.evaluate(ctx,pred)['decision']=='WAIT'
    ring,pred,ctx=scene(offsets=(14.,-16.),length=25.)
    assert ring.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'


def test_missing_pair_stale_or_invalid_map_never_releases():
    ring,pred,ctx=scene(offsets=(14.,))
    assert ring.evaluate(ctx,pred)['decision']=='UNKNOWN'
    ring,pred,ctx=scene()
    assert ring.evaluate(replace(ctx,now=2.),pred)['decision']=='UNKNOWN'
    assert CircularGap(None).evaluate(ctx,pred)['decision']=='UNKNOWN'


def test_partial_views_do_not_collapse_length_and_wrap_is_continuous():
    ring,pred,ctx=scene(offsets=(62.,35.),length=7.)
    previous=[x.length for x in ring.tracks.values()]
    events=[replace(t.event,stamp=1.1,size_x=.5,size_y=.5) for t in pred.tracks.values()]
    pred.observe(events,1.1);ring.observe(pred,1.1)
    assert all(x.length>=6.5 for x in ring.tracks.values())
    assert all(abs(x.speed-5)<1 for x in ring.tracks.values())


def test_conflicting_pair_changes_do_not_accumulate_clear_time():
    gate=EntryGate(enabled=True)
    for i in range(8):
        t=1+i*.2;ctx=Context(880,0,0,0,0,t)
        data=dict(evaluated_at=t,evidence_stamp=t,decision='CANDIDATE_GAP',lane_tracks=2,
                  gap_pair=['rear',f'front{i}'])
        assert gate.apply(Target(),ctx,data).v_max==0


def test_lost_third_vehicle_remains_occupancy_and_blocks():
    ring,pred,ctx=scene(offsets=(14.,-16.,0.))
    pred.tracks={k:t for k,t in pred.tracks.items() if not t.event.value.endswith(':2')}
    ring.observe(pred,1.1)
    assert any(t.event.value.endswith(':2') for t in ring.tracks.values())
    assert ring.evaluate(replace(ctx,now=1.1),pred)['decision']!='CANDIDATE_GAP'


def test_out_of_circle_vehicle_is_not_silently_dropped():
    ring,pred,ctx=scene()
    pred.observe([EventInfo('VEHICLE',1.,x=30.,y=0.,vx=-4.,value='lidar:test:outside')],1.)
    ring.observe(pred,1.)
    assert ring.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'


def test_front_only_view_encloses_unseen_rear_of_vehicle():
    ring,pred,ctx=scene(exact_ends=False)
    tr=next(iter(ring.tracks.values()))
    tr.observed_length=.3
    lo,hi=ring.extents(tr,0.)
    assert lo<=-tr.length and hi>=tr.length


def test_oversized_wrap_gap_is_not_inferred_from_unseen_back_of_circle():
    ring,pred,ctx=scene(offsets=(30.,40.))
    assert ring.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'


def test_wait_reports_first_conflict_for_emergency_braking():
    ring,pred,ctx=scene(offsets=(4.,-10.))
    d=ring.evaluate(ctx,pred)
    assert d['decision']=='WAIT'
    assert d['conflict_in_s']==0.


def test_circular_rviz_shows_chosen_pair_without_legacy_motion_branches():
    from builtin_interfaces.msg import Time
    from ctrl_planning.ctrl_prediction_markers import prediction_markers
    ring,pred,ctx=scene()
    d=ring.evaluate(ctx,pred)
    markers=prediction_markers(pred,ctx,d,Target(),Time(sec=1),circle=ring.circle,show_details=True).markers
    assert any(m.ns=='selected_gap' for m in markers)
    assert any(m.ns=='gap_status' and 'minimum margin' in m.text for m in markers)
    assert not any(m.ns=='motion_hypotheses' for m in markers)


def test_staging_pose_stays_outside_swept_ring_and_other_stop_survives():
    from ctrl_control.ctrl_core import Route
    ring,pred,ctx=scene()
    ctx=replace(ctx,s=880.,x=0.,y=-35.,yaw=math.pi/2,v=0.,
                route=Route(np.c_[np.zeros(60),np.arange(-35.,25.)]),s_offset=-880.)
    stage=ring.staging_s(ctx)
    assert 880.<stage<=888.
    assert ring.body_inner_radius(ctx.route,stage+ctx.s_offset)>21.15+.6
    gate=EntryGate(enabled=True)
    d=dict(model='circular_gap',decision='WAIT',evaluated_at=1.,evidence_stamp=1.,staging_s=stage)
    target=gate.apply(Target(),ctx,d)
    assert target.state=='ROUNDABOUT_APPROACH' and 0<target.v_max<=.8
    stopped=gate.apply(Target(v_max=0.,stop_s=881.),ctx,d)
    assert stopped.v_max==0 and stopped.stop_s==881.


def test_circular_commit_requires_actual_ring_contact():
    gate=EntryGate(enabled=True);gate.permitted=True
    ctx=Context(886.1,0,0,0,1,1.)
    d=dict(model='circular_gap',decision='UNKNOWN',evaluated_at=1.,evidence_stamp=1.,ego_overlaps_ring=False)
    gate.apply(Target(),ctx,d)
    assert not gate.committed


def test_measured_polygon_replaces_inflated_aabb_radial_extent():
    ring,pred,ctx=scene()
    tr=next(iter(ring.tracks.values()));e=tr.event
    angle=math.atan2(e.y,e.x);tangent=np.array([-math.sin(angle),math.cos(angle)])
    radial=np.array([math.cos(angle),math.sin(angle)])
    points=[(np.array([e.x,e.y])+u*tangent+v*radial).tolist() for u,v in [(-2.,-1.),(2.,-1.),(2.,1.),(-2.,1.)]]
    ring.on_footprints(dict(stamp=e.stamp,frame='map',objects=[dict(id=e.value,xy=points)]))
    ring.observe(pred,ctx.now)
    assert tr.has_polygon
    assert tr.radial_hi<1.5 and tr.radial_lo>-1.5
    assert tr.observed_length>3.5


def test_stale_or_malformed_polygon_cannot_replace_current_measurement():
    ring,pred,ctx=scene(exact_ends=False)
    tr=next(iter(ring.tracks.values()))
    ring.on_footprints(dict(stamp=-2.,frame='map',objects=[dict(id=tr.event.value,xy=[[0.,0.],[1.,0.]])]))
    ring.observe(pred,ctx.now)
    assert not tr.has_polygon
    ring.on_footprints(dict(stamp=1.,frame='wrong',objects=[]))


def test_explicit_occluded_conflict_brakes_even_when_decision_unknown():
    gate=EntryGate(enabled=True);gate.committed=True
    ctx=Context(890,0,0,0,3,1.)
    d=dict(model='circular_gap',decision='UNKNOWN',evaluated_at=1.,evidence_stamp=1.,conflict_in_s=.4)
    assert gate.apply(Target(),ctx,d).v_max==0.


def test_faster_vehicle_gets_earlier_staging_limit():
    from ctrl_control.ctrl_core import Route
    ring,pred,ctx=scene()
    route=Route(np.c_[np.zeros(60),np.arange(-30.,30.)])
    ctx=replace(ctx,s=880.,x=0.,y=-30.,yaw=math.pi/2,v=0.,route=route,s_offset=-880.)
    slow=ring.staging_s(ctx);fast=ring.staging_s(replace(ctx,v=3.))
    assert fast is None or fast<slow


def test_view_direction_alone_does_not_prove_unoccluded_vehicle_end():
    ring,pred,ctx=scene(exact_ends=False);tr=next(iter(ring.tracks.values()))
    tr.has_polygon=True;tr.arc_lo=-.2;tr.arc_hi=.2;tr.observed_length=.4
    for view in (-1.,1.):
        lo,hi=ring.extents(tr,0.,view)
        assert lo<-4. and hi>4.


def test_cold_moving_fragment_does_not_freeze_at_old_position():
    ring,pred,ctx=scene();e=EventInfo('VEHICLE',1.,x=20.,y=0.,vy=5.,size_x=1.,size_y=1.,value='lidar:test:new')
    pred.observe([e],1.);ring.observe(pred,1.)
    tr=next(t for t in ring.tracks.values() if t.event.value==e.value)
    assert not tr.ready and tr.speed>4.


def test_unique_circle_id_handoff_preserves_history_without_cached_resurrection():
    from ctrl_planning.ctrl_prediction import VehicleTrack
    ring,pred,ctx=scene()
    old=next(iter(pred.tracks.values()));e=old.event
    a=math.atan2(e.y,e.x)+.5/20;h=a+math.pi/2
    new=VehicleTrack();new.update(replace(e,stamp=1.1,value='lidar:test:replacement',
        x=20*math.cos(a),y=20*math.sin(a),vx=5*math.cos(h),vy=5*math.sin(h)))
    pred.tracks[('track','lidar:test:replacement')]=new
    ring.observe(pred,1.1)
    assert len(ring.tracks)==2
    assert any(t.event.value.endswith('replacement') and len(t.samples)>5 for t in ring.tracks.values())
    ring.observe(pred,1.15)
    assert len(ring.tracks)==2


def test_cooperative_mode_hands_off_to_front_following_after_alignment():
    ring,pred,ctx=scene();ring.cooperative_rear=True
    d=ring.evaluate(ctx,pred)
    assert d['decision']=='CANDIDATE_GAP'
    assert d['following_handoff_in_s']<=.8
    assert 'following_speed_mps' in d


def test_simulator_following_gap_uses_two_fresh_evaluations_not_fixed_long_wait():
    gate=EntryGate(enabled=True)
    for t in (1.,1.23):
        ctx=Context(887,0,0,0,0,t)
        d=dict(model='circular_gap',cooperative_rear=True,evaluated_at=t,evidence_stamp=t,
               decision='CANDIDATE_GAP',lane_tracks=2,gap_pair=['rear','front'])
        target=gate.apply(Target(),ctx,d)
    assert target.v_max>0 and math.isnan(target.stop_s)


def test_on_road_raw_cluster_tracks_motion_without_vehicle_classification():
    circle=CircularReference(np.array([0.,0.]),20.,0.,0.,1)
    ring=CircularGap(circle);pred=GapPredictor()
    for t in np.arange(0.,1.01,.1):
        a=5*t/20.;x,y=20*math.cos(a),20*math.sin(a)
        ring.on_footprints(dict(frame='map',stamp=float(t),objects=[dict(id='lidar:raw',kind='OBSTACLE',
            on_road=True,center=[x,y],vx=-5*math.sin(a),vy=5*math.cos(a),
            xy=[[x-.2,y-.2],[x+.2,y-.2],[x+.2,y+.2]])]))
        ring.observe(pred,float(t))
    assert not pred.tracks
    assert ring.tracks['lidar:raw'].ready
    assert abs(ring.tracks['lidar:raw'].speed-5.)<.01
    assert ring.tracks['lidar:raw'].has_polygon


def test_offroad_and_offcircle_raw_clusters_do_not_enter_ring_tracking():
    circle=CircularReference(np.array([0.,0.]),20.,0.,0.,1)
    ring=CircularGap(circle)
    ring.on_footprints(dict(frame='map',stamp=1.,objects=[
        dict(id='offroad',on_road=False,center=[20.,0.],vx=0.,vy=5.,xy=[[20.,0.],[20.,1.]]),
        dict(id='offcircle',on_road=True,center=[30.,0.],vx=0.,vy=5.,xy=[[30.,0.],[30.,1.]])]))
    ring.observe(GapPredictor(),1.)
    assert not ring.tracks


def test_cooperative_handoff_keeps_full_front_and_other_collision_horizon():
    ring,pred,ctx=scene();ring.cooperative_rear=True
    result=ring.evaluate(ctx,pred)
    assert result['following_handoff_in_s']<=.8
    assert result['interaction_end_s']>=3.8


def test_rear_conflict_cannot_hide_imminent_front_collision_after_commit():
    ring,pred,ctx=scene(offsets=(8.,-3.));ring.cooperative_rear=True
    def ego(ctx,dt,accel=1.):
        a=20*dt/20.;r=20. if dt<2. else 30.
        return (r*math.cos(a),r*math.sin(a),a+math.pi/2,4.635,1.892)
    pred.ego_pose=ego
    result=ring.evaluate(ctx,pred)
    assert result['conflict_side']=='front'
    assert result['conflict_in_s']<=.4
    gate=EntryGate(enabled=True);gate.committed=True
    assert gate.apply(Target(),ctx,result).v_max==0.


def test_cooperative_mode_checks_stopped_body_beyond_alignment_time():
    ring,pred,ctx=scene(offsets=(14.,-16.,12.));ring.cooperative_rear=True
    tr=ring.tracks['lidar:test:2'];tr.ready=False;tr.speed=0.
    result=ring.evaluate(ctx,pred)
    assert result['decision']=='UNKNOWN'
    assert result['conflict_side']=='other'
    assert .6<result['conflict_in_s']<=2.
    gate=EntryGate(enabled=True);gate.committed=True
    assert gate.apply(Target(),ctx,result).v_max==0.


def test_body_outside_pair_centers_still_blocks_if_its_extent_overlaps():
    ring,pred,ctx=scene(offsets=(14.,-16.,16.),exact_ends=False);ring.cooperative_rear=True
    tr=ring.tracks['lidar:test:2'];tr.ready=False;tr.speed=0.
    tr.length=40.;tr.observed_length=40.
    result=ring.evaluate(ctx,pred)
    assert result['decision']=='UNKNOWN'
    assert result['conflict_side']=='other' and result['conflict_in_s']==0.


def test_unready_stationary_track_diagnostic_is_json_serializable():
    import json
    ring,pred,ctx=scene()
    tr=ring.tracks['lidar:test:0']
    for stamp in np.arange(1.1,2.2,.1):
        tr.update(replace(tr.event,stamp=float(stamp),vx=0.,vy=0.),ring.circle)
    json.dumps(ring.evaluate(replace(ctx,now=2.1),pred))


def test_committed_single_front_body_is_checked_without_bounded_pair():
    ring,pred,ctx=scene(offsets=(4.,));ring.cooperative_rear=True
    result=ring.evaluate(ctx,pred)
    assert result['decision']=='UNKNOWN'
    assert result['conflict_in_s']==0.
    gate=EntryGate(enabled=True);gate.committed=True
    assert gate.apply(Target(),ctx,result).v_max==0.


def test_independent_body_box_covers_asymmetric_radial_bounds():
    ring,pred,ctx=scene(offsets=(0.,))
    tr=ring.tracks['lidar:test:0'];tr.event=replace(tr.event,x=18.,y=0.)
    tr.radial_lo=-3.;tr.radial_hi=1.15
    pred.ego_pose=lambda ctx,dt,accel=1.:(21.05,0.,math.pi/2,.2,.2)
    assert ring.independent_conflict(ctx,pred)['conflict_in_s']==0.


def test_measured_clearance_removes_old_track_without_cached_resurrection():
    ring,pred,ctx=scene()
    key='lidar:test:0'
    ring.on_footprints(dict(frame='map',stamp=1.5,objects=[],cleared=[dict(id=key,last_seen=1.)]))
    ring.observe(pred,1.5)
    assert key not in ring.tracks and ring.cleared_count==1
    # A late clearance response must not delete a newer actual observation.
    survivor='lidar:test:1'
    ring.on_footprints(dict(frame='map',stamp=1.6,objects=[],cleared=[dict(id=survivor,last_seen=.9)]))
    assert survivor in ring.tracks


def test_same_sweep_observation_wins_over_late_clearance_response():
    ring,pred,ctx=scene();key='lidar:test:0'
    tr=ring.tracks[key];x,y=tr.event.x,tr.event.y
    ring.on_footprints(dict(frame='map',stamp=1.5,objects=[dict(id=key,on_road=True,
        center=[x,y],vx=tr.event.vx,vy=tr.event.vy,xy=[[x-.2,y-.2],[x+.2,y+.2]])],
        cleared=[dict(id=key,last_seen=1.)]))
    ring.observe(pred,1.5)
    assert key in ring.tracks and ring.tracks[key].event.stamp==1.5


def test_wide_thin_end_face_anchors_hidden_body_away_from_sensor():
    ring,pred,ctx=scene(exact_ends=False);tr=next(iter(ring.tracks.values()))
    tr.has_polygon=True;tr.arc_lo=-.15;tr.arc_hi=.15;tr.observed_length=.3
    tr.raw_radial_width=1.8
    lo,hi=ring.extents(tr,0.,view=1.)
    assert 4.8<=hi-lo<6.7
    assert lo< -4. and hi<1.0


def test_fragment_clearance_contains_radially_skewed_observed_hull():
    from ctrl_planning.ctrl_circular_gap import RingTrack
    circle=CircularReference(np.array([0.,0.]),20.,0.,0.,1)
    ring=CircularGap(circle);tr=RingTrack()
    tr.update(EventInfo('OBSTACLE',1.,x=20.,y=0.,value='fragment'),circle)
    points=[[19.9,-.1],[22.,-.1],[22.,.1],[19.9,.1]]
    tr.apply_polygon(1.,points,circle);ring.tracks['fragment']=tr
    q=ring.clearance_queries(1.3)['objects'][0]
    for x,y in points:
        dx,dy=x-q['x'],y-q['y']
        assert abs(-dx*math.sin(q['yaw'])+dy*math.cos(q['yaw']))<=q['width']/2


def test_third_body_keeps_anchored_short_end_in_collision_check():
    ring,pred,ctx=scene(offsets=(14.,-16.,6.),exact_ends=False)
    ctx.now=1.6
    for key,tr in ring.tracks.items():
        if key!='lidar:test:2':tr.event=replace(tr.event,stamp=1.6)
    tr=ring.tracks['lidar:test:2']
    tr.speed=0.;tr.ready=True;tr.has_polygon=True
    tr.body_bounds=(-.2,6.)
    # A stale but motion-established body has its short end toward ego.
    # Symmetrizing to +/-6 falsely blocks ego at the very start.
    pred.ego_pose=lambda ctx,dt,accel=1.:(20. if dt<.8 else 30.,0.,math.pi/2,4.635,1.892)
    d=ring.evaluate(ctx,pred)
    assert d['decision']=='CANDIDATE_GAP',d['reason']


def test_local_entry_can_handoff_without_predicting_full_exit():
    ring,pred,ctx=scene();ring.local_entry=True
    pred.ego_pose=lambda ctx,dt,accel=1.:(20*math.cos(5*dt/20),20*math.sin(5*dt/20),5*dt/20+math.pi/2,4.635,1.892)
    d=ring.evaluate(ctx,pred)
    assert d['decision']=='CANDIDATE_GAP',d['reason']
    assert d['admission_model']=='three_vehicle_entry'
    assert d['interaction_end_s']<4.


def test_local_entry_rejects_fast_approaching_rear_without_assuming_braking():
    ring,pred,ctx=scene(offsets=(14.,-7.));ring.local_entry=True;ring.cooperative_rear=True
    ring.tracks['lidar:test:1'].speed=15.
    d=ring.evaluate(ctx,pred)
    assert d['decision']!='CANDIDATE_GAP'


def test_local_rear_margin_trial_opens_borderline_gap_without_removing_body_clearance():
    ring,pred,ctx=scene(offsets=(14.,-5.6));ring.local_entry=True
    ring.rear_margin_m=.35  # explicit historical trial, not the default
    ring.rear_uncertainty_mps=.35
    assert ring.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'
    ring.rear_uncertainty_mps=.15
    assert ring.evaluate(ctx,pred)['decision']=='CANDIDATE_GAP'
    close,pred,ctx=scene(offsets=(14.,-3.));close.local_entry=True
    close.rear_uncertainty_mps=.15
    assert close.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'


def test_moving_entry_accepts_closer_rear_only_when_ego_progress_clears_it():
    from ctrl_control.ctrl_core import Route
    from ctrl_planning.ctrl_following_departure import FollowingDeparture
    def decision(speed,rear):
        ring,pred,ctx=scene(offsets=(20.,rear));ring.local_entry=True
        angles=np.linspace(0.,3.,601)
        ctx.route=Route(np.c_[20*np.cos(angles),20*np.sin(angles)])
        ctx.s=0.;ctx.v=speed;pred.max_speed=5.56
        departure=FollowingDeparture(pred,ring.circle,list(ring.tracks.values()),follow_before_contact=True)
        return ring.evaluate(ctx,departure)['decision']
    # Same observed geometry and 18 km/h rear: rolling ego can pull away;
    # a stopped ego cannot use that same short interval.
    assert decision(0.,-6.)=='WAIT'
    assert decision(5.5,-6.)=='CANDIDATE_GAP'
    assert decision(5.5,-2.)!='CANDIDATE_GAP'


def test_local_entry_requires_alignment_before_handoff():
    ring,pred,ctx=scene();ring.local_entry=True
    pred.ego_pose=lambda ctx,dt,accel=1.:(20.,0.,0.,4.635,1.892)
    assert ring.evaluate(ctx,pred)['decision']=='UNKNOWN'


def test_local_entry_keeps_fresh_third_body_collision_veto():
    ring,pred,ctx=scene(offsets=(14.,-16.,0.));ring.local_entry=True
    ring.tracks['lidar:test:2'].ready=False
    assert ring.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'


def test_local_entry_does_not_use_stale_pair_to_authorize_departure():
    ring,pred,ctx=scene();ring.local_entry=True;ctx.now=2.
    assert ring.evaluate(ctx,pred)['decision']=='UNKNOWN'


def test_local_entry_retains_front_collision_after_alignment():
    ring,pred,ctx=scene();ring.local_entry=True
    # Alignment completes early; the front becomes unsafe in the following preview.
    pred.ego_pose=lambda ctx,dt,accel=1.:(20*math.cos((5*dt+(8 if dt>=1. else 0))/20),20*math.sin((5*dt+(8 if dt>=1. else 0))/20),
                                       (5*dt+(8 if dt>=1. else 0))/20+math.pi/2,4.635,1.892)
    assert ring.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'


def test_old_unselected_track_only_vetoes_immediate_local_collision():
    from ctrl_planning.ctrl_circular_gap import inspect_unselected_body
    ctx=SimpleNamespace(v=0.)
    assert inspect_unselected_body(True,ctx,2.,1.8)
    assert not inspect_unselected_body(True,ctx,2.,4.)
    assert inspect_unselected_body(True,ctx,.4,4.)
    assert inspect_unselected_body(True,ctx,.9,4.)
    assert inspect_unselected_body(False,ctx,2.,4.)
    assert inspect_unselected_body(True,SimpleNamespace(v=6.),2.,4.)


def test_stale_third_vehicle_far_ahead_does_not_block_verified_current_pair():
    ring,pred,ctx=scene(offsets=(14.,-16.,24.));ring.local_entry=True;ctx.now=2.3;ctx.v=0.
    for key,tr in ring.tracks.items():
        if key!='lidar:test:2':tr.event=replace(tr.event,stamp=ctx.now)
    old=ring.tracks['lidar:test:2'];old.ready=False;old.ever_ready=True;old.speed=0.
    assert ring.evaluate(ctx,pred)['decision']=='CANDIDATE_GAP'


def test_local_entry_does_not_forget_established_vehicle_occluded_inside_gap():
    ring,pred,ctx=scene(offsets=(14.,-16.,0.));ring.local_entry=True;ctx.now=1.6
    for key,tr in ring.tracks.items():
        if key!='lidar:test:2':tr.event=replace(tr.event,stamp=1.6)
    tr=ring.tracks['lidar:test:2'];tr.ever_ready=True;tr.speed=0.
    assert ring.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'


def test_local_entry_ignores_rear_catchup_only_after_completed_insertion():
    ring,pred,ctx=scene(offsets=(14.,-9.));ring.local_entry=True
    ring.tracks['lidar:test:1'].speed=8.
    result=ring.evaluate(ctx,pred)
    assert result['decision']=='CANDIDATE_GAP',result['reason']
    assert 'handoff_rear_reserve_m' not in result
    assert result['rear_check']['at_s']<=result['following_handoff_in_s']
    # The same rear can still veto an actual collision DURING insertion.
    ring.tracks['lidar:test:1'].speed=20.
    assert ring.evaluate(ctx,pred)['decision']=='WAIT'



def test_local_wide_observed_entry_gap_is_not_rejected_by_35m_cutoff():
    ring,pred,ctx=scene(offsets=(18.,-22.));ring.local_entry=True
    result=ring.evaluate(ctx,pred)
    assert result['decision']=='CANDIDATE_GAP',result['reason']
    assert result['gap_pair']==['lidar:test:1','lidar:test:0']


def test_local_wide_gap_keeps_third_body_collision_guard():
    ring,pred,ctx=scene(offsets=(18.,-22.,0.));ring.local_entry=True
    ring.tracks['lidar:test:2'].ready=False
    assert ring.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'


def test_local_entry_does_not_choose_unobserved_long_arc_around_back_of_ring():
    ring,pred,ctx=scene(offsets=(5.,-5.));ring.local_entry=True
    assert ring.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'


def test_unconfirmed_fragment_uses_measured_extent_not_two_invented_car_lengths():
    from ctrl_planning.ctrl_circular_gap import RingTrack
    tr=RingTrack();tr.has_polygon=True;tr.arc_lo=-.2;tr.arc_hi=.2
    tr.observed_length=.4
    lo,hi=CircularGap.extents(tr,0.)
    assert abs(lo-(-.8))<1e-9 and abs(hi-.8)<1e-9
    # Established vehicles still retain their unseen physical extent.
    tr.ever_ready=True
    lo,hi=CircularGap.extents(tr,0.)
    assert lo<-4 and hi>4


def test_local_admission_checks_fast_approach_not_only_average_speed():
    ring,pred,ctx=scene(offsets=(14.,-8.));ring.local_entry=True
    assert ring.evaluate(ctx,pred)['decision']=='CANDIDATE_GAP'
    rear=min(ring.tracks.values(),key=lambda tr:tr.samples[-1][1])
    rear.front_motion.approach_speed=12.
    result=ring.evaluate(ctx,pred)
    assert result['decision']!='CANDIDATE_GAP'
    assert result['gap_rear_speed']==12.
