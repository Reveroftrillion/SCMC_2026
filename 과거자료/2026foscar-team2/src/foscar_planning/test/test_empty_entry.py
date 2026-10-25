from test_circular_gap import scene
from foscar_planning.entry_gate import EntryGate
from foscar_planning.state import Target


def visible(gap):
    # A complete scan must actually include the cars this fixture sees.
    newest=max((t.event.stamp for t in gap.tracks.values()),default=0.)
    return [dict(id=k,xy=[[t.event.x-.1,t.event.y-.1],[t.event.x+.1,t.event.y+.1]])
            for k,t in gap.tracks.items() if t.event.stamp==newest]


def test_healthy_scan_alone_is_not_empty_road_proof():
    gap,pred,ctx=scene(offsets=());gap.local_entry=True;gap.scan_stamp=ctx.now
    assert gap.evaluate(ctx,pred)['decision']=='UNKNOWN'


def test_confirmed_empty_probes_allow_no_vehicle_entry_but_stale_or_obstructed_do_not():
    gap,pred,ctx=scene(offsets=());gap.local_entry=True
    queries=gap.clearance_queries(ctx.now,ctx)
    assert len(queries['objects'])>=10
    cleared=[dict(id=q['id'],last_seen=q['last_seen']) for q in queries['objects']]
    ctx.now+=.3
    gap.on_footprints(dict(frame='map',stamp=ctx.now,objects=visible(gap),cleared=cleared))
    result=gap.evaluate(ctx,pred)
    assert result['decision']=='CANDIDATE_GAP' and result['empty_lane']
    ctx.now+=.5
    assert gap.evaluate(ctx,pred)['decision']=='UNKNOWN'


def test_verified_empty_admission_releases_while_approaching():
    from test_entry_gate import ctx
    gate=EntryGate(enabled=True)
    for t,v in ((1.,1.),):
        data=dict(model='circular_gap',admission_model='three_vehicle_entry',empty_lane=True,
                  decision='CANDIDATE_GAP',lane_tracks=0,evaluated_at=t,evidence_stamp=t)
        result=gate.apply(Target(),ctx(t=t,v=v),data)
    assert gate.committed and result.v_max>0


def test_vehicle_gap_decision_releases_while_approaching_but_wait_still_stops():
    from test_entry_gate import ctx
    gate=EntryGate(enabled=True,stop_at_s=885.712)
    data=dict(model='circular_gap',admission_model='three_vehicle_entry',
              decision='WAIT',lane_tracks=2,evaluated_at=1.,evidence_stamp=1.)
    assert gate.apply(Target(),ctx(s=885.,v=2.),data).v_max==0
    data['decision']='CANDIDATE_GAP'
    assert gate.apply(Target(),ctx(s=885.,v=2.),data).v_max>0
    assert gate.committed


def test_single_vehicle_requires_its_own_collision_decision_even_with_clear_rays():
    for offset,allowed in ((25.,True),(2.,False),(-2.,False)):
        gap,pred,ctx=scene(offsets=(offset,));gap.local_entry=True
        queries=gap.clearance_queries(ctx.now,ctx)
        cleared=[dict(id=q['id'],last_seen=q['last_seen']) for q in queries['objects']]
        gap.on_footprints(dict(frame='map',stamp=ctx.now,objects=visible(gap),cleared=cleared))
        result=gap.evaluate(ctx,pred)
        assert (result['decision']=='CANDIDATE_GAP')==allowed
        assert not result.get('empty_lane',False)


def test_missing_patch_and_new_occupied_scan_revoke_empty_proof():
    gap,pred,ctx=scene(offsets=());gap.local_entry=True
    rows=gap.clearance_queries(ctx.now,ctx)['objects']
    proofs=[dict(id=q['id'],last_seen=q['last_seen']) for q in rows]
    gap.on_footprints(dict(frame='map',stamp=ctx.now,objects=visible(gap),cleared=proofs[:-1]))
    assert gap.evaluate(ctx,pred)['decision']=='UNKNOWN'
    ctx.now+=.1
    gap.on_footprints(dict(frame='map',stamp=ctx.now,objects=visible(gap),cleared=proofs))
    assert gap.evaluate(ctx,pred)['decision']=='CANDIDATE_GAP'
    ctx.now+=.1
    gap.on_footprints(dict(frame='map',stamp=ctx.now,objects=visible(gap),cleared=[]))
    assert gap.evaluate(ctx,pred)['decision']=='UNKNOWN'
def test_waiting_point_query_waits_for_route_at_startup():
    from foscar_planning.empty_entry import EmptyEntry
    from types import SimpleNamespace
    assert EmptyEntry().queries(SimpleNamespace(route=None),None,885.)==[]


def test_rear_only_vehicle_does_not_require_visibility_behind_that_vehicle():
    import math
    gap,pred,ctx=scene(offsets=(-16.,));gap.local_entry=True
    rows=gap.clearance_queries(ctx.now,ctx)['objects']
    # Rear traffic hides the upstream road. Front corridor remains observed.
    proofs=[dict(id=q['id'],last_seen=q['last_seen']) for q in rows
            if math.atan2(q['y'],q['x'])*gap.circle.radius>-12.]
    gap.on_footprints(dict(frame='map',stamp=ctx.now,objects=visible(gap),cleared=proofs))
    result=gap.evaluate(ctx,pred)
    assert result['decision']=='CANDIDATE_GAP'
    assert result.get('rear_only')
    gap.on_footprints(dict(frame='map',stamp=ctx.now+.1,objects=visible(gap),cleared=[]));ctx.now+=.1
    assert gap.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'


def test_rear_only_entry_rejects_fast_approach_and_unobserved_front():
    import math
    for fast,missing_front in ((True,False),(False,True)):
        gap,pred,ctx=scene(offsets=(-16.,));gap.local_entry=True
        rows=gap.clearance_queries(ctx.now,ctx)['objects']
        proofs=[dict(id=q['id'],last_seen=q['last_seen']) for q in rows
                if math.atan2(q['y'],q['x'])*gap.circle.radius>-12.]
        if fast:next(iter(gap.tracks.values())).speed=30.
        if missing_front:proofs=proofs[:-1]
        gap.on_footprints(dict(frame='map',stamp=ctx.now,objects=visible(gap),cleared=proofs))
        assert gap.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'


def test_two_upstream_cars_do_not_force_entry_into_the_gap_behind_ego():
    gap,pred,ctx=scene(offsets=(-14.,-28.));gap.local_entry=True
    queries=gap.clearance_queries(ctx.now,ctx)['objects']
    proofs=[dict(id=q['id'],last_seen=q['last_seen']) for q in queries]
    gap.on_footprints(dict(frame='map',stamp=ctx.now,objects=visible(gap),cleared=proofs))
    result=gap.evaluate(ctx,pred)
    assert result['decision']=='CANDIDATE_GAP',result['reason']
    assert result['rear_only'] and result['gap_pair']==['lidar:test:0']


def test_open_front_still_checks_second_approaching_car_and_missing_rays():
    for speed,proof in ((50.,True),(5.,False)):
        gap,pred,ctx=scene(offsets=(-14.,-28.));gap.local_entry=True
        gap.tracks['lidar:test:1'].speed=speed
        queries=gap.clearance_queries(ctx.now,ctx)['objects']
        cleared=[dict(id=q['id'],last_seen=q['last_seen']) for q in queries] if proof else []
        gap.on_footprints(dict(frame='map',stamp=ctx.now,objects=visible(gap),cleared=cleared))
        assert gap.evaluate(ctx,pred)['decision']!='CANDIDATE_GAP'


def test_open_front_drops_object_absent_from_current_complete_scan():
    from dataclasses import replace
    gap,pred,ctx=scene(offsets=(-14.,-28.,0.));gap.local_entry=True
    old=gap.tracks['lidar:test:2'];old.event=replace(old.event,stamp=ctx.now-.8)
    old.speed=0.;old.ready=False;old.ever_ready=True
    queries=gap.clearance_queries(ctx.now,ctx)['objects']
    # Prove patches only: do not issue a track-deletion certificate.
    cleared=[dict(id=q['id'],last_seen=q['last_seen']) for q in queries if q.get('purpose')=='empty_entry']
    gap.on_footprints(dict(frame='map',stamp=ctx.now,objects=visible(gap),cleared=cleared))
    result=gap.evaluate(ctx,pred)
    assert result['rear_only'] and old.event.value not in result['additional_guard_ids']
    assert result['decision']=='CANDIDATE_GAP'
