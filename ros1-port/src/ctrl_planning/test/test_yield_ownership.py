import numpy as np
from ctrl_control.ctrl_core import Route
from ctrl_planning.ctrl_state import Context, EventInfo
from ctrl_planning.states.ctrl_yield import Yield


def context():
    return Context(s=0,x=0,y=0,yaw=0,v=0,now=1.,
                   route=Route(np.c_[np.arange(101),np.zeros(101)]))


def road_user(kind='VEHICLE',s=15.,d=0.,stamp=1.,sy=2.):
    return EventInfo(type=kind,stamp=stamp,s=s,d=d,x=s,y=d,
                     size_x=4.8 if kind=='VEHICLE' else .6,size_y=sy,box_yaw=0.,
                     value='lidar:test:1')


def test_predictor_owns_distant_vehicle_but_immediate_vehicle_still_yields():
    y=Yield();c=context();c.vehicle_prediction_active=True
    c.events=[road_user()]
    assert not y.check_entry(c)
    c.events=[road_user(s=5.)]
    assert y.check_entry(c)


def test_vehicle_handoff_releases_old_wait_without_waiting_for_disappearance():
    y=Yield();c=context();c.events=[road_user()]
    assert y.check_entry(c)
    c.vehicle_prediction_active=True
    assert y.check_exit(c)


def test_handoff_does_not_clear_pedestrian_or_changed_obstacle():
    for kind in ('PEDESTRIAN','VEHICLE'):
        y=Yield();c=context();c.events=[road_user(kind)]
        assert y.check_entry(c)
        c.vehicle_prediction_active=True
        if kind=='VEHICLE':c.events[0].type='OBSTACLE'
        assert not y.check_exit(c)


def test_person_body_clear_releases_before_fixed_two_point_five_meter_threshold():
    y=Yield();c=context();c.events=[road_user('PEDESTRIAN',d=0.,sy=.5)]
    assert y.check_entry(c)
    for stamp in (1.1,1.3,1.5):
        c.now=stamp;c.events=[road_user('PEDESTRIAN',d=1.9,stamp=stamp,sy=.5)]
        exited=y.check_exit(c)
    assert exited
    assert not y.check_entry(c)


def test_person_body_still_overlaps_or_returns_toward_lane_does_not_release():
    for sy,vy in ((2.,0.),(.5,-1.)):
        y=Yield();c=context();c.events=[road_user('PEDESTRIAN',sy=sy)]
        assert y.check_entry(c)
        for stamp in (1.1,1.3,1.5):
            c.now=stamp;c.events=[road_user('PEDESTRIAN',d=1.9,stamp=stamp,sy=sy)]
            c.events[0].vy=vy
            assert not y.check_exit(c)


def test_vehicle_legacy_behavior_outside_prediction_zone_is_preserved():
    c=context();c.events=[road_user()]
    assert Yield().check_entry(c)


def test_large_pedestrian_box_overlapping_corridor_still_blocks_beyond_center_threshold():
    y=Yield();c=context();c.events=[road_user('PEDESTRIAN',d=2.6,sy=3.)]
    assert y.check_entry(c)


def test_stale_body_clear_cache_cannot_trigger_immediate_departure():
    y=Yield();c=context();c.events=[road_user('PEDESTRIAN',sy=.5)]
    assert y.check_entry(c)
    c.events=[road_user('PEDESTRIAN',d=1.9,sy=.5,stamp=1.1)]
    for t in (1.4,1.5,1.6,1.7):
        c.now=t
        assert not y.check_exit(c)


def test_pedestrian_trial_switch_keeps_vehicle_and_unknown_yield():
    y=Yield(pedestrian_enabled=False);c=context();c.vehicle_prediction_active=True
    c.events=[road_user('PEDESTRIAN',s=3.)]
    assert not y.check_entry(c)
    for kind in ('VEHICLE','DYNAMIC_UNKNOWN'):
        c.events=[road_user(kind,s=3.)]
        assert y.check_entry(c)
    c.events=[road_user('PEDESTRIAN',s=3.)]
    assert Yield().check_entry(c)


def test_disabling_pedestrian_wait_does_not_release_another_vehicle_threat():
    y=Yield();c=context();c.events=[road_user('PEDESTRIAN',s=3.)]
    assert y.check_entry(c)
    y.pedestrian_enabled=False
    assert y.check_exit(c)
    y=Yield();assert y.check_entry(c)
    y.pedestrian_enabled=False
    c.events.append(road_user('VEHICLE',s=4.))
    assert not y.check_exit(c)


def test_committed_following_does_not_stop_for_departing_lead_inside_absolute_stop_distance():
    c=context();c.vehicle_prediction_active=True;c.vehicle_following_active=True
    c.v=4.452;c.events=[road_user(s=10.364)]
    e=c.events[0];e.size_x=3.982;e.vx=4.137
    assert not Yield().check_entry(c)


def test_committed_following_keeps_close_stationary_crossing_and_closing_guards():
    for distance,vx,vy,stamp in ((5.,4.137,0.,1.),(10.364,0.,0.,1.),
                                 (10.364,4.137,3.,1.),(10.364,-4.,0.,1.),
                                 (10.364,4.137,0.,.4),(7.,1.,0.,1.)):
        c=context();c.vehicle_prediction_active=True;c.vehicle_following_active=True
        c.v=4.452;c.events=[road_user(s=distance,stamp=stamp)]
        e=c.events[0];e.size_x=3.982;e.vx=vx;e.vy=vy
        assert Yield().check_entry(c)


def test_pre_admission_keeps_absolute_braking_guard():
    c=context();c.vehicle_prediction_active=True;c.v=4.452
    c.events=[road_user(s=10.364)];c.events[0].vx=4.137
    assert Yield().check_entry(c)


def test_committed_entry_does_not_start_static_detour_for_far_fragment():
    from ctrl_planning.states.ctrl_avoid import Avoid
    c=context();c.vehicle_prediction_active=True;c.vehicle_following_active=True
    c.v=3.8;c.events=[road_user('OBSTACLE',s=15.,sy=.2)]
    c.events[0].size_x=.32
    assert not Avoid().check_entry(c)
    assert not Yield().check_entry(c)
    c.vehicle_following_active=False
    assert Avoid().check_entry(c)


def test_committed_static_obstacle_still_brakes_without_lateral_detour():
    c=context();c.vehicle_prediction_active=True;c.vehicle_following_active=True
    c.v=3.8;c.events=[road_user('OBSTACLE',s=5.,sy=.2)]
    c.events[0].size_x=.32;c.events[0].vx=4. # static label has no trusted motion
    assert Yield().check_entry(c)


def test_committed_rear_does_not_trigger_yield_even_with_overlapping_box():
    for kind in ('VEHICLE','DYNAMIC_UNKNOWN','OBSTACLE'):
        c=context();c.s=20.;c.x=20.;c.v=3.
        c.vehicle_prediction_active=True;c.vehicle_following_active=True
        c.events=[road_user(kind,s=19.,sy=2.)];c.events[0].size_x=6.
        assert not Yield().check_entry(c)
        c.vehicle_following_active=False
        if kind!='OBSTACLE':assert Yield().check_entry(c)


def test_fresh_measured_corridor_owns_only_already_evaluated_nonpedestrian():
    c=context();c.vehicle_prediction_active=True;c.vehicle_following_active=True
    c.v=5.;e=road_user(s=7.);c.events=[e]
    c.measured_clearance_ids=frozenset([e.value]);c.measured_clearance_stamp=1.
    assert not Yield().check_entry(c)
    e.stamp=1.01
    assert Yield().check_entry(c)  # new observation still gets immediate guard
    e.stamp=1.;e.value='new'
    assert Yield().check_entry(c)
    e.value='lidar:test:1';e.type='PEDESTRIAN'
    assert Yield().check_entry(c)
    e.type='VEHICLE';c.now=1.5
    assert Yield().check_entry(c)  # stale corridor cannot own braking


def test_pedestrian_mission_zone_excludes_roundabout_but_keeps_mission_approach():
    y=Yield(pedestrian_zone_s_from=[430.],pedestrian_zone_s_to=[470.])
    c=context();c.s=865.;c.vehicle_prediction_active=True
    e=EventInfo('PEDESTRIAN',c.now,s=889.,d=.5,value='lidar:fragment')
    assert not y.threatens(e,c)
    c.s=435.;e.s=450.
    assert y.threatens(e,c)


def test_committed_roundabout_never_starts_or_retains_pedestrian_yield():
    y=Yield();c=context()
    c.mission_name='ROUNDABOUT';c.mission_pedestrian=False
    c.vehicle_prediction_active=True;c.vehicle_following_active=True
    c.events=[road_user('PEDESTRIAN',s=5.)]
    assert not y.check_entry(c)
    # A genuinely close vehicle still uses the physical guard.
    c.events=[road_user('VEHICLE',s=5.)]
    assert y.check_entry(c) and y.compute_target(c).v_max==0
    # Reclassification to PEDESTRIAN cannot revive the pedestrian wait timer.
    c.events=[road_user('PEDESTRIAN',s=5.)]
    assert y.check_exit(c)
