import pytest
from foscar_planning.highway_observation import HighwayObservations

def row(x=0., identity='npc'):
    return dict(id=identity,xy=[[x,0.],[x+4,0.],[x+4,2.],[x,2.]],center=[x+2,1.],vx=30.,vy=0.,on_road=True)

def scan(t,objects,valid=True):
    return dict(stamp=t,frame='map',objects=objects,scan_valid=valid)

def test_missing_then_same_id_starts_new_history():
    o=HighwayObservations(history_s=.6,max_age_s=.25,min_history_s=.2,velocity_error_mps=.5,max_gap_s=.3)
    assert o.accept(scan(1.,[row()]),1.)
    assert o.accept(scan(1.1,[]),1.1)
    assert not o.snapshot(1.1).objects
    o.accept(scan(1.2,[row()]),1.2)
    assert o.snapshot(1.2).objects['npc'].sample_count==1
    assert not o.snapshot(1.2).objects['npc'].speed_ready
    assert not o.snapshot(1.5).valid

def test_continuous_speed_and_uncertainty_not_cached_velocity():
    o=HighwayObservations(.6,.25,.2,.5,.3)
    for t,x in [(1.,0.),(1.1,1.),(1.2,2.),(1.3,3.)]:o.accept(scan(t,[row(x)]),t)
    c=o.snapshot(1.3).objects['npc']
    assert c.speed_ready
    assert c.velocity_bounds[0]<=10.<=c.velocity_bounds[1]<15.
    assert c.velocity_bounds[2]<=0.<=c.velocity_bounds[3]

def test_malformed_duplicate_out_of_order_never_clears():
    o=HighwayObservations(.6,.25,.2,.5,.3)
    o.accept(scan(1.,[row()]),1.)
    for data in (scan(.9,[]),scan(1.,[]),scan(1.1,[dict(id='bad',xy=[])]),scan(2.,[])):
        assert not o.accept(data,1.1)
    assert 'npc' in o.snapshot(1.1).objects
    assert not o.snapshot(1.3).valid

def test_invalid_scan_revokes_permission_and_next_scan_starts_new_history():
    o=HighwayObservations(.6,.25,.2,.5,.3)
    o.accept(scan(1.,[row()]),1.)
    o.accept(scan(1.1,[],False),1.1)
    assert not o.snapshot(1.1).valid
    o.accept(scan(1.2,[row()]),1.2)
    assert o.snapshot(1.2).objects['npc'].sample_count==1

def test_two_objects_stay_separate_and_nonfinite_is_rejected():
    o=HighwayObservations(.6,.25,.2,.5,.3)
    assert o.accept(scan(1.,[row(),row(5.,'other')]),1.)
    assert len(o.snapshot(1.).objects)==2
    assert not o.accept(scan(1.1,[row(float('nan'))]),1.1)


def test_revealed_body_end_does_not_become_extreme_vehicle_speed():
    o=HighwayObservations(.6,.25,.2,.5,.3)
    for i in range(6):
        t=1+i*.1;r=row(i)
        if i==5:
            # More of the rear/side appears in one frame; the other end remains stable.
            r['xy'][1][0]+=3.;r['xy'][2][0]+=3.
        o.accept(scan(t,[r]),t)
    b=o.snapshot(t).objects['npc'].velocity_bounds
    assert 8<=(b[0]+b[1])/2<=12


def test_unique_visible_replacement_can_keep_recent_measured_motion():
    o=HighwayObservations(.6,.25,.2,.5,.3)
    for i in range(4):o.accept(scan(1+i*.1,[row(i)]),1+i*.1)
    o.accept(scan(1.4,[row(4.,'new-id')]),1.4)
    assert o.snapshot(1.4).objects['new-id'].speed_ready
    assert o.snapshot(1.4).objects['new-id'].velocity_bounds[0]<=10.


def test_motion_replacement_does_not_chain_without_new_measurements():
    o=HighwayObservations(.6,.25,.2,.5,.3)
    for i in range(4):o.accept(scan(1+i*.1,[row(i)]),1+i*.1)
    for i in range(4,9):o.accept(scan(1+i*.1,[row(i,str(i))]),1+i*.1)
    assert not o.snapshot(1.8).objects['8'].speed_ready


def test_two_new_fragments_cannot_both_inherit_one_vehicle_speed():
    o=HighwayObservations(.6,.25,.2,.5,.3)
    for i in range(4):o.accept(scan(1+i*.1,[row(i)]),1+i*.1)
    o.accept(scan(1.4,[row(4.,'a'),row(4.2,'b')]),1.4)
    assert not any(x.speed_ready for x in o.snapshot(1.4).objects.values())


def test_measured_track_missing_a_scan_keeps_its_motion():
    # 2026-09-27: the lead lost its history for one scan and was braked for as if stopped.
    o=HighwayObservations(.6,.25,.2,.5,.3)
    for t,x in [(1.,0.),(1.1,1.),(1.2,2.),(1.3,3.)]:o.accept(scan(t,[row(x)]),t)
    assert o.snapshot(1.3).objects['npc'].speed_ready
    o.accept(scan(1.4,[]),1.4)
    o.accept(scan(1.8,[row(8.)]),1.8)          # gap > max_gap: history restarts
    c=o.snapshot(1.8).objects['npc']
    assert c.speed_ready and c.motion_source=='recent_id'
    assert c.velocity_bounds[0]<=10.<=c.velocity_bounds[1]
    o.accept(scan(3.,[row(20.)]),3.)            # beyond the 1 s memory: unknown again
    assert not o.snapshot(3.).objects['npc'].speed_ready


def test_a_measured_car_that_drops_out_coasts_for_two_seconds():
    # 12:11 run 4: the car being passed vanished from the lidar 45 m ahead and the fallback cut in.
    o=HighwayObservations(.6,.25,.2,.5,.3)
    for t,x in [(1.,0.),(1.1,1.),(1.2,2.),(1.3,3.)]:o.accept(scan(t,[row(x)]),t)
    for k in range(1,11):o.accept(scan(1.3+.1*k,[]),1.3+.1*k)            # scans keep coming, the car is gone
    t=2.3
    c=o.snapshot(2.3).objects['npc']
    assert c.motion_source=='coasting' and c.speed_ready and c.xy[:,0].min()>8.   # moved on at ~10 m/s
    for k in range(1,12):o.accept(scan(2.3+.1*k,[]),2.3+.1*k)
    assert 'npc' not in o.snapshot(3.4).objects                                     # gone after coast_s


def test_an_id_change_does_not_leave_a_coasting_duplicate():
    o=HighwayObservations(.6,.25,.2,.5,.3)
    for t,x in [(1.,0.),(1.1,1.),(1.2,2.),(1.3,3.)]:o.accept(scan(t,[row(x)]),t)
    o.accept(scan(1.4,[row(4.,'new')]),1.4)
    assert set(o.snapshot(1.4).objects)=={'new'}


def test_sight_range_rides_along_with_the_scan():
    from foscar_planning.highway_observation import HighwayObservations
    obs = HighwayObservations(1., .5, .2, .5, .3)
    row = dict(id='a', xy=[[0., 0.], [1., 1.]])
    assert obs.accept(dict(frame='map', stamp=1., objects=[row], scan_valid=True, sight=dict(range_m=44.)), 1.)
    assert obs.snapshot(1.).sight_m == 44.
    assert obs.accept(dict(frame='map', stamp=1.1, objects=[row], scan_valid=True), 1.1)
    assert obs.snapshot(1.1).sight_m == float('inf')
