from ctrl_planning.states.ctrl_avoid import Avoid
from ctrl_planning.ctrl_state import EventInfo
from test_tunnel_avoidance import context


def bev_context():
    c=context();c.mission_name='STATIC_OBSTACLE';c.events=[]
    c.occupancy_stamp=c.now
    c.occupancy_hulls={'piece':((38.,-1.),(41.,-1.),(41.,1.),(38.,1.))}
    c.occupancy_sensor=(c.x,c.y)
    return c


def test_complete_bev_scan_drives_avoidance_without_object_type_or_id():
    c=bev_context();a=Avoid()
    assert a.check_entry(c)
    assert a.boxes()
    before=[(b.s_min,b.s_max,b.d_min,b.d_max) for b in a.boxes()]
    c.events=[EventInfo(type='PEDESTRIAN',stamp=c.now,s=40,d=0,size_x=20,size_y=20)]
    c.now+=.1;c.occupancy_stamp=c.now
    c.occupancy_hulls={'new-id':c.occupancy_hulls['piece']}
    a.observe(c)
    assert [(b.s_min,b.s_max,b.d_min,b.d_max) for b in a.boxes()]==before


def test_missing_bev_scan_is_not_treated_as_new_free_space():
    c=bev_context();a=Avoid();assert a.check_entry(c)
    c.now+=1.
    target=a.compute_target(c)
    assert target.v_max==0.


def test_bev_guard_checks_actual_held_offset_without_a_chain():
    from ctrl_planning.ctrl_state import Target
    c=bev_context();c.occupancy_hulls={'piece':((22.,2.),(23.,2.),(23.,4.),(22.,4.))}
    a=Avoid();a.observe(c)
    out=a.bev.guard(c,Target(lat_offset=3.,v_max=10.),a)
    assert out.v_max==0.
    assert a.bev.last_reason=='occupied'


def test_nearby_occupied_fragments_share_motion_without_filling_their_gap():
    c=bev_context();c.occupancy_hulls={
        'body':((38.,-1.),(41.,-1.),(41.,1.),(38.,1.)),
        'piece':((41.35,-.5),(41.7,-.5),(41.7,.5),(41.35,.5))}
    a=Avoid();a.observe(c)
    assert len(a.bev.motion.regions)==1
    assert len(a.bev.field.hulls)==2


def test_unmeasured_motion_does_not_turn_a_distant_body_into_immediate_collision():
    from ctrl_planning.ctrl_state import Target
    c=bev_context();c.v=0.
    a=Avoid();a.observe(c)
    out=a.bev.guard(c,Target(v_max=10.),a)
    assert out.v_max==10.
    assert a.bev.last_reason=='clear'


def test_unmeasured_body_on_path_still_requires_stopping():
    from ctrl_planning.ctrl_state import Target
    c=bev_context();c.v=0.
    c.occupancy_hulls={'body':((22.,-1.),(24.,-1.),(24.,1.),(22.,1.))}
    a=Avoid();a.observe(c)
    assert a.bev.guard(c,Target(v_max=10.),a).v_max==0.
    assert a.bev.last_reason=='occupied'


def test_measured_crossing_motion_still_blocks_before_current_shape_reaches_path():
    from ctrl_planning.ctrl_state import Target
    from ctrl_planning.ctrl_bev_motion import OccupancyRegion
    c=bev_context();c.v=4.
    poly=((25.,3.),(26.,3.),(26.,4.),(25.,4.))
    c.occupancy_hulls={'crossing':poly}
    a=Avoid();a.observe(c)
    a.bev.motion.regions=[OccupancyRegion(poly,c.now,velocity=(0.,-3.),sigma=(.1,.1),ready=True)]
    out=a.bev.guard(c,Target(v_max=10.),a)
    assert out.v_max<10.
    assert a.bev.last_reason=='predicted occupancy'


def test_guard_diagnostic_identifies_cells_and_clears_previous_stop():
    from ctrl_planning.ctrl_state import Target
    c=bev_context();c.v=0.
    c.occupancy_hulls={'body':((22.,-1.),(24.,-1.),(24.,1.),(22.,1.))}
    a=Avoid();a.observe(c);a.bev.guard(c,Target(v_max=10.),a)
    assert a.bev.diagnostic['reason']=='occupied'
    assert len(a.bev.diagnostic['cells'])>0
    assert len(a.bev.diagnostic['body'])==4
    c.occupancy_cleared=({'id':'body','last_seen':c.occupancy_stamp},)
    c.now+=.4;c.occupancy_stamp=c.now;c.occupancy_hulls={};a.observe(c)
    a.bev.guard(c,Target(v_max=10.),a)
    assert a.bev.diagnostic['reason']=='clear'
    assert 'cells' not in a.bev.diagnostic
    a.bev.guard(c,Target(v_max=0.),a)
    assert a.bev.diagnostic['reason']=='planner hold'


def test_prediction_diagnostic_shows_measured_shape_and_expanded_shape():
    from ctrl_planning.ctrl_state import Target
    from ctrl_planning.ctrl_bev_motion import OccupancyRegion
    c=bev_context();poly=((25.,3.),(26.,3.),(26.,4.),(25.,4.))
    c.occupancy_hulls={'crossing':poly};a=Avoid();a.observe(c)
    a.bev.motion.regions=[OccupancyRegion(poly,c.now,velocity=(0.,-3.),sigma=(.1,.1),ready=True)]
    a.bev.guard(c,Target(v_max=10.),a)
    d=a.bev.diagnostic
    assert d['reason']=='predicted occupancy'
    assert d['region']==poly and len(d['prediction'])>=4
    assert d['velocity']==(0.,-3.) and d['sigma']==(.1,.1)


def test_rviz_markers_name_reason_and_show_both_prediction_and_body():
    from builtin_interfaces.msg import Time
    from ctrl_planning.ctrl_mission_markers import bev_stop_markers
    from ctrl_planning.ctrl_state import Target
    from ctrl_planning.ctrl_bev_motion import OccupancyRegion
    c=bev_context();poly=((25.,3.),(26.,3.),(26.,4.),(25.,4.))
    c.occupancy_hulls={'crossing':poly};a=Avoid();a.observe(c)
    a.bev.motion.regions=[OccupancyRegion(poly,c.now,velocity=(0.,-3.),sigma=(.1,.1),ready=True)]
    t=a.bev.guard(c,Target(state='AVOID',v_max=10.),a)
    markers={m.ns:m for m in bev_stop_markers(c,t,a.bev.diagnostic,Time())}
    assert 'PREDICTED' in markers['bev_stop_reason'].text
    assert 'object 10.80 km/h' in markers['bev_stop_reason'].text
    assert all(len(markers[ns].points)>=5 for ns in ['bev_stop_body','bev_stop_observed','bev_stop_prediction'])
    c.now+=1.;a.bev.guard(c,t,a)
    stale=bev_stop_markers(c,t,a.bev.diagnostic,Time())
    assert len(stale)==1 and 'STALE SCAN' in stale[0].text


def test_offset_candidates_cover_the_same_occupied_cells_as_collision_checks():
    c=bev_context();c.occupancy_hulls={'piece':((38.1,-.9),(41.1,-.9),(41.1,1.1),(38.1,1.1))}
    a=Avoid();a.observe(c)
    b=a.boxes()[0];cells=a.bev.field.occupied_indices;res=a.bev.field.resolution
    assert b.s_min<=cells[:,0].min()*res
    assert b.s_max>=(cells[:,0].max()+1)*res
    assert b.d_min<=cells[:,1].min()*res
    assert b.d_max>=(cells[:,1].max()+1)*res


def test_stationary_measured_region_does_not_inflate_into_nearby_clear_path():
    from ctrl_planning.ctrl_state import Target
    from ctrl_planning.ctrl_bev_motion import OccupancyRegion
    c=bev_context();c.v=0.
    poly=((23.,2.),(25.,2.),(25.,3.),(23.,3.))
    c.occupancy_stamp=c.now-.2
    c.occupancy_hulls={'stationary':poly};a=Avoid();a.observe(c)
    a.bev.motion.regions=[OccupancyRegion(poly,c.now-.2,velocity=(.001,-.001),sigma=(.31,.31),ready=True)]
    out=a.bev.guard(c,Target(v_max=10.),a)
    assert a.bev.last_reason!='predicted occupancy' and out.v_max>0.


def test_tunnel_complete_scan_uses_bev_and_stale_scan_guard():
    c=bev_context();c.mission_name='TUNNEL';a=Avoid()
    assert a.check_entry(c)
    assert a.bev is not None and a.boxes()
    c.now+=1.
    assert a.compute_target(c).v_max==0.
    assert a.bev.last_reason=='stale scan'


def seen_twice(c,a):
    # an outline is remembered once seen in two scans (a one-scan deer piece blocked the car, 2026-10-11)
    a.observe(c);c.now+=.1;c.occupancy_stamp=c.now;a.observe(c)


def test_one_scan_outline_is_not_remembered():
    c=bev_context();a=Avoid();a.observe(c)
    c.now+=.1;c.occupancy_stamp=c.now;c.occupancy_hulls={}
    a.observe(c)
    assert not a.bev.remembered


def test_missing_occupancy_requires_matching_raw_clearance_evidence():
    c=bev_context();a=Avoid();seen_twice(c,a)
    original=c.occupancy_stamp
    c.now+=1.;c.occupancy_stamp=c.now;c.occupancy_hulls={}
    a.observe(c)
    assert a.boxes() and a.bev.clearance_queries(c.now)['objects']
    c.now+=.1;c.occupancy_stamp=c.now
    c.occupancy_cleared=({'id':'piece','last_seen':original-.1},)
    a.observe(c);assert a.boxes()
    c.now+=.1;c.occupancy_stamp=c.now
    c.occupancy_cleared=({'id':'piece','last_seen':original},)
    a.observe(c);assert not a.boxes()


def test_retained_passed_occupancy_does_not_reenter_forward_plan():
    c=bev_context();a=Avoid();seen_twice(c,a)
    c.s=60.;c.x=60.;c.now+=.2;c.occupancy_stamp=c.now
    c.occupancy_hulls={};a.observe(c)
    assert a.bev.remembered
    assert not a.boxes()


def test_stopped_blocked_replan_starts_at_measured_offset(monkeypatch):
    from ctrl_planning import ctrl_tunnel_slalom as tunnel_slalom, ctrl_tunnel_simple as tunnel_simple
    c=bev_context();a=Avoid();a.check_entry(c);a.compute_target(c)
    a.slalom.blocked_at=0
    c.y=2.;c.v=0.;c.now+=.2;c.occupancy_stamp=c.now
    seen=[]
    for module in (tunnel_slalom,tunnel_simple):   # whichever planner is configured
        def capture(*args,_original=module.plan,**kwargs):
            seen.append(args[3])
            return _original(*args,**kwargs)
        monkeypatch.setattr(module,'plan',capture)
    a.compute_target(c)
    assert seen and all(abs(d-2.)<1e-6 for d in seen)


def test_measured_crossing_wait_preserves_path_and_keeps_occupied_stop(monkeypatch):
    from ctrl_planning.ctrl_bev_motion import OccupancyRegion
    c=bev_context();c.mission_name='TUNNEL';a=Avoid();a.check_entry(c);a.compute_target(c)
    previous=a.slalom
    c.now+=.1;c.occupancy_stamp=c.now
    poly=((22.,-1.),(24.,-1.),(24.,1.),(22.,1.))
    c.occupancy_hulls={'lidar:crossing':poly};c.yield_target_key=('track','lidar:crossing')
    a.observe(c)
    a.bev.motion.regions=[OccupancyRegion(poly,c.now,velocity=(0.,2.),sigma=(.1,.1),ready=True)]
    called=[]
    original=a._compute_target
    monkeypatch.setattr(a,'_compute_target',lambda ctx:(called.append(True),original(ctx))[1])
    out=a.compute_target(c)
    assert not called and a.slalom is previous
    assert out.v_max==0. and a.bev.last_reason=='occupied'
    a.bev.motion.regions=[]
    a.compute_target(c)
    assert not called  # missing motion evidence must not release the crossing wait
    a.bev.motion.regions=[OccupancyRegion(poly,c.now,velocity=(0.,0.),sigma=(.1,.1),ready=True)]
    a.compute_target(c)
    assert called  # measured stopping allows a fresh safe path around the occupied body
    called.clear()
    c.yield_target_key=None
    a.compute_target(c)
    assert called


def test_pedestrian_label_without_measured_motion_does_not_freeze_replanning():
    c=bev_context();c.mission_name='TUNNEL';c.yield_target_key=('track','piece')
    a=Avoid();a.check_entry(c);a.compute_target(c)
    assert not a._crossing_waiting


def test_tunnel_shadow_reaches_only_as_far_as_a_low_body_hides():
    # the lidar (2.11 m) looks over a 1 m barricade 20 m ahead: 10 m behind it is in sight (user 2026-10-11)
    from ctrl_planning.ctrl_bev_occupancy import BevOccupancy, FREE, UNKNOWN
    bar={'bar':[(20.,-1.5),(20.5,-1.5),(20.5,1.5),(20.,1.5)]}
    f=BevOccupancy(.25,40.);f.update(1.,bar,(0.,0.))
    assert f.query([(30.,0.)])[0]==UNKNOWN                 # no height: shadowed all the way, as before
    f=BevOccupancy(.25,40.);f.update(1.,bar,(0.,0.),heights={'bar':1.},sensor_z=2.11,low_m=.8)
    assert f.query([(21.5,0.)])[0]==UNKNOWN                # right behind it a low body may hide
    assert f.query([(30.,0.)])[0]==FREE


def test_tunnel_remembered_outline_unseen_where_rings_meet_it_is_forgotten():
    from ctrl_planning.ctrl_bev_avoidance import BevAvoidance
    from types import SimpleNamespace
    b=BevAvoidance(.25,40.)
    piece=((8.,-.3),(8.6,-.3),(8.6,.3),(8.,.3))
    b.remembered['piece']=(0.,piece,1.0)                    # a 1 m outline 8 m ahead: rings -11 and -9 meet it
    ctx=SimpleNamespace(occupancy_sensor=(0.,0.),yaw=0.)
    b.field.update(1.,{},(0.,0.))
    for k in range(BevAvoidance.ABSENT_SCANS-1):
        b.clear_absent(ctx,{});assert 'piece' in b.remembered
    b.clear_absent(ctx,{});assert 'piece' not in b.remembered
    b.remembered['low']=(0.,((3.,-.3),(3.6,-.3),(3.6,.3),(3.,.3)),1.0)   # 3 m ahead: the rings pass over it, kept
    b.remembered['far']=(0.,((15.,-.3),(15.6,-.3),(15.6,.3),(15.,.3)),1.0)  # 15 m: one ring only, kept
    for k in range(5): b.clear_absent(ctx,{})
    assert 'low' in b.remembered and 'far' in b.remembered
