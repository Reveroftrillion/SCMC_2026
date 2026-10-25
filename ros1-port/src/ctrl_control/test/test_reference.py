import numpy as np
import pytest
from ctrl_control.ctrl_reference import ReferenceLine
from ctrl_control.ctrl_core import Controller, Route, Target

def test_arc_length_is_independent_of_global_station():
    r=ReferenceLine('one',[100,102,104],[[0,0],[.8,.6],[1.6,1.2]])
    assert r.global_at_q(1.)==pytest.approx(102.)
    assert r.q_at_global(102.)==pytest.approx(1.)

@pytest.mark.parametrize('station,xy', [([0,0],[[0,0],[1,0]]),([0,1],[[0,0],[float('nan'),1]]),([0,1],[[0,0],[0,0]]),([0,1],[[0,0],[2,0]])])
def test_bad_reference_rejected(station,xy):
    with pytest.raises(ValueError):ReferenceLine('bad',station,xy)

def test_actual_lane_reference_not_original_cross_track_controls_fault():
    route=Route(np.c_[np.arange(201),np.zeros(201)])
    ref=ReferenceLine('lane',np.arange(151),np.c_[np.arange(151),np.ones(151)*7])
    c=Controller(route,speed_kmh=30,continuous=True)
    out=c.step(10,7,0,3,2,4,True,target=Target(reference=ref,v_max=5))
    assert out.state=='CRUISE' and out.error<.01
    assert out.progress==pytest.approx(10)
    stopped=c.step(10.2,7,0,3,2,4,True,target=Target(v_max=0,state='NO_TARGET'))
    assert stopped.accel==0 and stopped.target_y==pytest.approx(7)
    assert stopped.target_speed==0

def test_same_id_cannot_replace_geometry_and_release_cannot_jump():
    route=Route(np.c_[np.arange(201),np.zeros(201)])
    c=Controller(route,speed_kmh=30,continuous=True)
    r=ReferenceLine('one',np.arange(151),np.c_[np.arange(151),np.ones(151)*7])
    c.step(10,7,0,3,2,4,True,target=Target(reference=r,v_max=5))
    changed=ReferenceLine('one',np.arange(151),np.c_[np.arange(151),np.ones(151)*4])
    out=c.step(10.1,7,0,3,2,4,True,target=Target(reference=changed,v_max=5))
    assert out.accel==0 and out.target_y==pytest.approx(7)
    out=c.step(10.2,7,0,3,2,4,True,target=Target(reference_release=True,v_max=5))
    assert out.accel==0 and out.target_y==pytest.approx(7)

def test_global_stop_is_converted_to_real_reference_distance():
    route=Route(np.c_[np.arange(201),np.zeros(201)])
    gs=np.arange(0,101,.5)
    r=ReferenceLine('stretch',gs,np.c_[gs*1.2,np.zeros(len(gs))])
    assert r.q_at_global(20)==pytest.approx(24)
    c=Controller(route,speed_kmh=30,continuous=True)
    out=c.step(12,0,0,2,2,4,True,target=Target(reference=r,v_max=5,stop_s=20))
    assert out.target_speed<5

def test_new_id_with_disconnected_geometry_keeps_last_reference_and_brakes():
    route=Route(np.c_[np.arange(201),np.zeros(201)])
    c=Controller(route,speed_kmh=30,continuous=True)
    old=ReferenceLine('old',np.arange(151),np.c_[np.arange(151),np.ones(151)*7])
    c.step(10,7,0,3,2,4,True,target=Target(reference=old,v_max=5))
    bad=ReferenceLine('new',np.arange(151),np.c_[np.arange(151),np.ones(151)*20])
    out=c.step(10.1,7,0,3,2,4,True,target=Target(reference=bad,v_max=5))
    assert out.accel==0 and out.target_y==pytest.approx(7)
    assert c.active_reference.id=='old'

def test_outer_sensor_fault_cannot_be_unlatched_by_reference_controller():
    route=Route(np.c_[np.arange(201),np.zeros(201)])
    c=Controller(route,speed_kmh=30,continuous=True)
    r=ReferenceLine('one',np.arange(151),np.c_[np.arange(151),np.ones(151)*7])
    t=Target(reference=r,v_max=5)
    c.step(10,7,0,3,2,4,True,target=t)
    c.fail('IMU/status disagree')
    out=c.step(10.1,7,0,3,2,4,True,target=t)
    assert out.state=='FAULT' and out.accel==0

def test_reference_preserves_global_highway_speed_policy_and_exit_braking():
    ss=np.arange(0.,2001.,.5)
    route=Route(np.c_[ss,np.zeros(len(ss))])
    owner=Controller(route,speed_kmh=60.,continuous=True,uncapped_zones=[(100.,1500.)])
    gs=np.arange(1000.,1350.1,.5)
    ref=ReferenceLine('uncapped',gs,np.c_[gs,np.zeros(len(gs))])
    owner.step(1001,0,0,20,2,4,True,.05,Target(reference=ref))
    out=owner.step(1001.5,0,0,20,2,4,True,.05,Target(reference=ref))
    assert out.target_speed>60/3.6
    gs=np.arange(1450.,1550.1,.5)
    ref=ReferenceLine('exit',gs,np.c_[gs,np.zeros(len(gs))])
    fresh=Controller(route,speed_kmh=60.,continuous=True,uncapped_zones=[(100.,1500.)])
    fresh.step(1490,0,0,15,2,4,True,.05,Target(reference=ref))
    inner=fresh.reference_controller
    assert inner.profile[0]>inner.profile[99]
    assert np.max(inner.profile[gs>=1500.])<=60/3.6+1e-8


def test_rejoining_reference_inherits_original_profile_before_release():
    ss=np.arange(201.)
    route=Route(np.c_[ss,ss*0]);owner=Controller(route,speed_kmh=60,continuous=True)
    # Original route requires a slower passage after the join. Resampling the
    # local geometry must not lose that profile when ownership is handed back.
    owner.profile[:]=15.;owner.profile[100:]=4.
    owner.braking_profile[:]=15.;owner.braking_profile[100:]=4.
    gs=np.arange(20.,181.,.5)
    offset=3.5*np.clip((100.-gs)/50.,0.,1.)
    ref=ReferenceLine('join',gs,np.c_[gs,offset])
    owner.step(30,3.5,0,8,2,4,True,.05,Target(reference=ref))
    inner=owner.reference_controller
    assert np.max(inner.profile[gs>=100.])<=4.
    q=ref.q_at_global(90.)
    assert inner.route.speed_at(q,inner.profile)<=np.sqrt(4.**2+2*1.5*(ref.q_at_global(100.)-q))+.1


def test_release_uses_actual_speed_instead_of_bypassed_curve_acceleration_history():
    ss=np.arange(201.)
    route=Route(np.c_[ss,ss*0]);owner=Controller(route,speed_kmh=60,continuous=True)
    owner.profile[80:130]=4.  # Old path's recovery after a bend, not a future constraint.
    gs=np.arange(20.,181.,.5)
    ref=ReferenceLine('bypass',gs,np.c_[gs,gs*0])
    owner.step(99.,0.,0.,10.,2,4,True,.05,Target(reference=ref))
    assert owner.reference_controller.route.speed_at(80.,owner.reference_controller.profile)>10.
    owner.state='CRUISE';owner.reference_controller.state='CRUISE'
    out=owner.step(99.5,0.,0.,10.,2,4,True,.05,Target(reference_release=True))
    assert owner.active_reference is None
    assert out.state!='FAULT'
    assert owner.route.speed_at(100.,owner.profile)>=10.
    assert np.all(owner.profile<=owner.braking_profile+1e-9)


def test_rolling_reference_end_is_not_a_highway_stop_point():
    ss=np.arange(2001.)
    owner=Controller(Route(np.c_[ss,ss*0]),speed_kmh=60.,continuous=True,
                     uncapped_zones=[(100.,1500.)])
    gs=np.arange(500.,601.,.5)
    ref=ReferenceLine('rolling',gs,np.c_[gs,gs*0])
    out=owner.step(510.,0.,0.,20.,2,4,True,.05,Target(reference=ref))
    assert out.target_speed>20.
    stopped=owner.step(510.5,0.,0.,20.,2,4,True,.05,Target(reference=ref,stop_s=530.))
    assert stopped.target_speed<10.
    exhausted=owner.step(598.,0.,0.,20.,2,4,True,.05,Target(reference=ref))
    assert exhausted.accel==0. and exhausted.target_speed==0.


def test_reference_covering_actual_route_end_still_brakes():
    ss=np.arange(201.)
    owner=Controller(Route(np.c_[ss,ss*0]),speed_kmh=60.,continuous=True,
                     uncapped_zones=[(10.,200.)])
    gs=np.arange(100.,200.1,.5)
    ref=ReferenceLine('final',gs,np.c_[gs,gs*0])
    out=owner.step(180.,0.,0.,5.,2,4,True,.05,Target(reference=ref))
    assert 0.<out.target_speed<7.
