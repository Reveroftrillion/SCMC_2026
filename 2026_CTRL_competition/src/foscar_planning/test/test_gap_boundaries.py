"""Facing gap edges must not depend on the unobserved opposite end."""
import math
import numpy as np
from foscar_planning.circle_reference import CircularReference
from foscar_planning.circular_gap import CircularGap, RingTrack
from foscar_planning.state import EventInfo


def observed(points, length=4.8, direction=1):
    circle=CircularReference(np.array([0.,0.]),20.,0.,0.,direction)
    tr=RingTrack()
    tr.update(EventInfo('VEHICLE',1.,x=20.,y=0.,vy=5.*direction,
                        size_x=2.,size_y=4.,value='car'),circle)
    tr.apply_polygon(1.,points,circle);tr.length=length
    return circle,tr


def test_rear_corner_visible_from_side_anchors_gap_without_known_length():
    # Heading +y, rear plane y=-2, outer side x=21. Sensor mostly side-on.
    points=[[19.,-2.],[21.,-2.],[21.,1.]]
    bounds=[]
    for length in (4.8,9.):
        circle,tr=observed(points,length)
        tr.observe_boundaries((30.,-3.),circle)
        lo,hi=CircularGap.extents(tr,0.)
        assert tr.rear_observed and not tr.front_observed
        assert abs(lo-(tr.arc_lo-.6))<1e-6
        assert hi>=tr.arc_hi+.6
        bounds.append(lo)
    assert bounds[0]==bounds[1]


def test_approaching_front_face_does_not_expand_toward_ego_with_length():
    for length in (4.8,10.):
        circle,tr=observed([[19.,2.],[21.,2.],[21.,-1.]],length)
        tr.observe_boundaries((30.,3.),circle)
        assert tr.front_observed and not tr.rear_observed
        lo,hi=CircularGap.extents(tr,0.)
        assert abs(hi-(tr.arc_hi+.6))<1e-6
        assert lo<=tr.arc_lo-.6


def test_side_segment_and_diagonal_hull_are_not_end_face_evidence():
    for points in ([[21.,-2.],[21.,2.]], [[19.,-2.],[21.,0.],[21.,2.]]):
        circle,tr=observed(points)
        tr.observe_boundaries((30.,-3.),circle)
        assert not tr.rear_observed and not tr.front_observed
        tr.ever_ready=True  # established vehicle retains unknown ends
        lo,hi=CircularGap.extents(tr,0.)
        assert lo<tr.arc_lo-.8


def test_back_face_on_far_side_of_view_is_not_a_visible_rear():
    circle,tr=observed([[19.,-2.],[21.,-2.],[21.,1.]])
    tr.observe_boundaries((30.,3.),circle)
    assert not tr.rear_observed


def test_boundary_orientation_supports_clockwise_lane():
    circle,tr=observed([[19.,2.],[21.,2.],[21.,-1.]],direction=-1)
    tr.observe_boundaries((30.,3.),circle)
    assert tr.rear_observed and not tr.front_observed


def test_new_scan_without_face_does_not_keep_old_observed_status():
    circle,tr=observed([[19.,-2.],[21.,-2.],[21.,1.]])
    tr.observe_boundaries((30.,-3.),circle)
    assert tr.rear_observed
    from dataclasses import replace
    tr.update(replace(tr.event,stamp=1.1),circle)
    tr.observe_boundaries((30.,-3.),circle)
    assert not tr.rear_observed and tr.body_bounds is None


def test_rviz_distinguishes_observed_facing_end_from_estimated_end():
    from types import SimpleNamespace
    from builtin_interfaces.msg import Time
    from foscar_planning.state import Context,Target
    from foscar_planning.prediction_markers import prediction_markers
    circle,_=observed([[19.,-2.],[21.,-2.],[21.,1.]])
    result=dict(model='circular_gap',decision='WAIT',reference_angle=0.,
                occupied_intervals=[],gap_pair=['rear','lead'],gap_rear_s=-8.,gap_front_s=4.,
                gap_rear_boundary_source='estimated',gap_front_boundary_source='observed_face',
                minimum_gap_margin_m=-.5,interaction_start_s=1.,interaction_end_s=4.)
    ctx=Context(880.,30.,0.,math.pi/2,0.,1.)
    markers=prediction_markers(SimpleNamespace(max_speed=5.5),ctx,result,Target(),Time(),circle=circle,show_details=True).markers
    labels=[m.text for m in markers if m.ns=='gap_boundary_label']
    assert any('LEAD REAR' in s and 'observed_face' in s for s in labels)
    assert any('APPROACH FRONT' in s and 'estimated' in s for s in labels)


def test_subdivided_bumper_face_counts_as_one_end():
    # Rounded/dense hulls split a broad face into many individually short edges.
    points=[[19.,-1.9],[19.3,-2.],[19.8,-2.05],[20.2,-2.05],
            [20.7,-2.],[21.,-1.9],[21.,1.]]
    circle,tr=observed(points)
    tr.observe_boundaries((30.,-3.),circle)
    assert tr.rear_observed


def test_local_entry_pairs_position_with_polygon_when_event_arrives_first():
    from dataclasses import replace
    from types import SimpleNamespace
    circle,tr=observed([[19.,-2.],[21.,-2.],[21.,1.]])
    gap=CircularGap(circle);gap.local_entry=True
    e=tr.event
    gap.on_footprints(dict(stamp=1.,frame='map',objects=[dict(id=e.value,
        xy=tr.polygon_xy,center=[e.x,e.y],vx=e.vx,vy=e.vy,on_road=True)]))
    newer=replace(e,stamp=1.1,y=.5,size_y=.3)
    predictor=SimpleNamespace(tracks={'car':SimpleNamespace(event=newer)})
    gap.observe(predictor,1.1)
    assert gap.tracks['car'].has_polygon
    assert gap.tracks['car'].event.stamp==1.
    gap.on_footprints(dict(stamp=1.1,frame='map',objects=[dict(id=e.value,
        xy=[[x,y+.5] for x,y in tr.polygon_xy],center=[20.,.5],vx=0.,vy=5.,on_road=True)]))
    gap.observe(predictor,1.11)
    assert gap.tracks['car'].has_polygon and gap.tracks['car'].event.stamp==1.1


def test_frame_pairing_never_hides_prolonged_missing_polygon_or_legacy_event():
    from dataclasses import replace
    from types import SimpleNamespace
    for local,stamp in ((True,1.2),(False,1.1)):
        circle,tr=observed([[19.,-2.],[21.,-2.],[21.,1.]])
        gap=CircularGap(circle);gap.local_entry=local;e=tr.event
        gap.on_footprints(dict(stamp=1.,frame='map',objects=[dict(id=e.value,
            xy=tr.polygon_xy,center=[e.x,e.y],vx=e.vx,vy=e.vy,on_road=True)]))
        predictor=SimpleNamespace(tracks={'car':SimpleNamespace(event=replace(e,stamp=stamp))})
        gap.observe(predictor,stamp)
        if local:
            # Only the last complete scan is authoritative, not a newer
            # unpaired event; loss of scans separately expires the decision.
            assert gap.tracks['car'].event.stamp==1.
            assert gap.tracks['car'].has_polygon
        else:
            assert gap.tracks['car'].event.stamp==stamp
            assert not gap.tracks['car'].has_polygon


def test_visible_side_endpoint_needs_fresh_positive_ray_evidence():
    circle,tr=observed([[21.,-2.],[21.,2.]])
    tr.observe_boundaries((30.,0.),circle)
    assert not tr.rear_observed
    tr.side_proofs={'rear':dict(stamp=tr.event.stamp,edge_s=tr.samples[-1][1]+tr.arc_lo,speed=0.)}
    tr.observe_boundaries((30.,0.),circle)
    assert tr.rear_observed and not tr.front_observed
    assert abs(CircularGap.extents(tr,0.)[0]-(tr.arc_lo-.6))<1e-6
    tr.side_proofs['rear']['stamp']-=.5
    tr.observe_boundaries((30.,0.),circle)
    assert not tr.rear_observed


def test_endpoint_queries_use_map_direction_without_fabricating_car_length():
    from foscar_planning.observed_boundary import endpoint_queries
    circle,tr=observed([[21.,-2.],[21.,2.]],length=12.)
    queries=endpoint_queries(tr,circle,tr.event.stamp)
    assert {q['side'] for q in queries}=={'rear','front'}
    for q in queries:
        assert q['purpose']=='ring_boundary' and q['length']<1.
        expected=tr.samples[-1][1]+(tr.arc_lo if q['side']=='rear' else tr.arc_hi)
        assert abs(q['edge_s']-expected)<1e-6


def test_local_slot_uses_supported_end_motion_before_center_track_warms_up():
    from test_circular_gap import scene
    ring,pred,ctx=scene(offsets=());ring.local_entry=True
    ring.sensor_xy=(ctx.x,ctx.y)
    for t in (1.,1.12,1.24,1.36):
        objects=[];proofs=[]
        for key,offset in (('lead',14.),('rear',-16.)):
            center_arc=offset+5.*(t-1.)
            a=center_arc/20.;h=a+math.pi/2
            center=21.*np.array([math.cos(a),math.sin(a)])
            points=[(21.*np.array([math.cos((center_arc+d)/20.),math.sin((center_arc+d)/20.)])).tolist() for d in (-2.,0.,2.)]
            objects.append(dict(id=key,xy=points,center=center.tolist(),vx=5.*math.cos(h),vy=5.*math.sin(h),on_road=True))
            for side,ds in (('rear',-2.),('front',2.)):
                proofs.append(dict(id=key,side=side,stamp=t,edge_s=center_arc+ds,speed=5.))
        ring.on_footprints(dict(frame='map',stamp=t,objects=objects,boundary_evidence=proofs))
        ring.observe(pred,t)
    ctx.now=t
    assert all(not tr.ready for tr in ring.tracks.values())  # center estimator needs .45s
    result=ring.evaluate(ctx,pred)
    assert result['decision']=='CANDIDATE_GAP',result['reason']
    assert abs(result['gap_front_speed']-5.)<1e-6
    assert abs(result['gap_rear_speed']-5.)<1e-6
    assert all(tr.ever_ready for tr in ring.tracks.values())
    # A fresh sweep that fails the beyond-end ray test revokes old certificates.
    ring.on_footprints(dict(frame='map',stamp=1.48,objects=objects,boundary_evidence=[]))
    ring.observe(pred,1.48);ctx.now=1.48
    assert all(not tr.front_observed and not tr.rear_observed for tr in ring.tracks.values())
    assert ring.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'
    # Boundary-established vehicles remain modeled when later occluded.
    ctx.now=2.
    result=ring.evaluate(ctx,pred)
    assert result['decision']=='UNKNOWN'
    assert result['reason']=='latest lidar scan unavailable'
    assert not result['occupied_intervals']


def test_local_entry_never_falls_back_to_aabb_when_endpoint_evidence_missing():
    from test_circular_gap import scene
    ring,pred,ctx=scene(exact_ends=False);ring.local_entry=True
    assert all(tr.ready for tr in ring.tracks.values())
    assert ring.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'
    ring,pred,ctx=scene(offsets=(-16.,),exact_ends=False);ring.local_entry=True
    assert ring.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'


def test_single_rear_rejected_when_same_sweep_endpoint_support_is_revoked():
    from test_circular_gap import scene
    ring,pred,ctx=scene(offsets=(-16.,));ring.local_entry=True
    ring.empty_entry.permitted=lambda *args,**kwargs:True
    tr=next(iter(ring.tracks.values()))
    tr.front_observed=False  # cached motion can still be ready for the same stamp
    assert ring.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'


def side_sequence():
    from dataclasses import replace
    circle,tr=observed([[21.,-2.],[21.,2.]])
    for t in (1.,1.12,1.24,1.36):
        a=2.*(t-1.)/20.
        rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
        center=rot@np.array([20.,0.])
        tr.update(replace(tr.event,stamp=t,x=center[0],y=center[1]),circle)
        tr.apply_polygon(t,(np.array([[21.,-2.],[21.,2.]])@rot.T).tolist(),circle)
        tr.observe_boundaries((30.,0.),circle)
    return circle,tr


def test_stable_side_rear_is_provisional_before_face_confirmation():
    circle,tr=side_sequence()
    assert not tr.rear_observed
    assert tr.rear_motion.ready
    assert tr.rear_estimated
    assert tr.rear_uncertainty>=.5
    assert CircularGap.extents(tr,0.)[0]<=tr.arc_lo-tr.rear_uncertainty-.6+1e-9
    # A missing hull revokes admission and clears the motion warmup.
    from dataclasses import replace
    tr.update(replace(tr.event,stamp=1.48),circle)
    tr.observe_boundaries((30.,0.),circle)
    assert not tr.rear_estimated and not tr.rear_motion.ready


def test_side_confirmation_does_not_start_motion_warmup_from_scratch():
    circle,tr=side_sequence()
    tr.side_proofs={'rear':dict(stamp=tr.event.stamp,edge_s=tr.samples[-1][1]+tr.arc_lo,speed=2.)}
    tr.observe_boundaries((30.,0.),circle)
    assert tr.rear_observed and tr.rear_motion.ready


def test_provisional_rear_can_bound_a_slot_without_rear_face_or_ray_proof():
    from test_circular_gap import scene
    ring,pred,ctx=scene(offsets=());ring.local_entry=True
    ring.sensor_xy=(ctx.x,ctx.y)
    for t in (1.,1.12,1.24,1.36):
        objects=[];proofs=[]
        for key,offset in (('lead',14.),('rear',-16.)):
            arc=offset+5.*(t-1.);a=arc/20.;h=a+math.pi/2
            center=21.*np.array([math.cos(a),math.sin(a)])
            points=[(21.*np.array([math.cos((arc+d)/20.),math.sin((arc+d)/20.)])).tolist() for d in (-2.,0.,2.)]
            objects.append(dict(id=key,xy=points,center=center.tolist(),vx=5.*math.cos(h),vy=5.*math.sin(h),on_road=True))
            if key=='rear':proofs.append(dict(id=key,side='front',stamp=t,edge_s=arc+2.,speed=5.))
        ring.on_footprints(dict(frame='map',stamp=t,objects=objects,boundary_evidence=proofs))
        ring.observe(pred,t)
    ctx.now=t
    lead=ring.tracks['lead']
    assert not lead.rear_observed and lead.rear_estimated
    result=ring.evaluate(ctx,pred)
    assert result['decision']=='CANDIDATE_GAP',result['reason']
    row=next(r for r in result['occupied_intervals'] if r['id']=='lead')
    assert row['rear_boundary_ready'] and row['rear_uncertainty_m']>=.5
    assert row['back_extent']>=-lead.arc_lo+.5
    assert result['gap_front_boundary_source']=='estimated'


def test_clipped_rear_side_cannot_invent_forward_motion_of_guarded_boundary():
    from dataclasses import replace
    circle,tr=observed([[21.,-2.],[21.,2.]])
    for t in (1.12,1.24,1.36,1.48):
        tr.update(replace(tr.event,stamp=t,vx=0.,vy=0.),circle)
        points=[[21.*math.cos(d/20.),21.*math.sin(d/20.)] for d in (-2.+2.*(t-1.),0.,2.)]
        tr.apply_polygon(t,points,circle)
        tr.observe_boundaries((30.,0.),circle)
    assert tr.rear_estimated
    assert tr.rear_motion.speed>1.9  # visible rear moves only because of clipping
    assert tr.rear_boundary_motion.speed<.05  # conservative rear stays still


def test_revealing_front_cannot_turn_shrinking_uncertainty_into_rear_speed():
    from dataclasses import replace
    circle,tr=observed([[21.,-2.],[21.,2.]])
    for t in (1.12,1.24,1.36,1.48):
        tr.update(replace(tr.event,stamp=t,vx=0.,vy=0.),circle)
        points=[[21.*math.cos(d/20.),21.*math.sin(d/20.)] for d in (-2.,0.,.5+5.*(t-1.))]
        tr.apply_polygon(t,points,circle)
        tr.observe_boundaries((30.,0.),circle)
    assert tr.rear_estimated
    assert tr.estimated_rear_motion.speed>4.
    assert tr.rear_boundary_motion.speed<.05
