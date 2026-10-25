import math
import pytest
from foscar_planning.mission import MissionSupervisor
from foscar_planning.state import Behavior, BehaviorState, Context, Target
from foscar_planning.states.cruise import Cruise
from foscar_planning.states.yield_ import Yield


def config():
    return dict(default=dict(name='CRUISE', allowed_states=['CRUISE','YIELD'], speed_kmh=50),
                zones=[dict(name='BOX',start_s=10,end_s=20,allowed_states=['AVOID'],speed_kmh=50),
                       dict(name='PEDESTRIAN',start_s=20,end_s=30,allowed_states=['YIELD'],pedestrian=True,speed_kmh=20),
                       dict(name='HIGHWAY',start_s=40,end_s=50,allowed_states=['CRUISE'],speed_kmh=None)])


def ctx(s, now=1, v=5):
    return Context(s=s,x=s,y=0,yaw=0,v=v,now=now)


def test_handoff_does_not_wait_for_lower_fsm_and_reconstructs_shared_states():
    class Stuck(BehaviorState):
        name='YIELD'
        defaults={'gap':6.}
        def __init__(self, **kw):
            super().__init__(**kw); self.calls=0
        def check_entry(self,c): return c.s<30
        def check_exit(self,c): return False
        def compute_target(self,c):
            self.calls+=1
            return Target(stop_s=25,v_max=0)
    b=Behavior([Stuck(gap=9)])
    m=MissionSupervisor(config())
    c=ctx(25); m.update(c); b.reset(); assert b.tick(c,allowed_states=m.policy.allowed_states).v_max==0
    c=ctx(31,2); assert m.update(c)
    b.reset()
    result=b.tick(c,allowed_states=m.policy.allowed_states)
    assert math.isnan(result.stop_s)
    assert not b.engaged['YIELD']
    assert b.states[0].gap==9 and b.states[0].calls==0


def test_boundary_jitter_and_start_midway_and_respawn():
    m=MissionSupervisor(config())
    assert m.update(ctx(25)) and m.policy.name=='PEDESTRIAN'
    assert not m.update(ctx(24.8,1.1))
    assert m.update(ctx(30,2)) and m.policy.name=='CRUISE'
    assert not m.update(ctx(29.8,2.1)) and m.policy.name=='CRUISE'
    assert m.update(ctx(15,3)) and m.reset_observations and m.policy.name=='BOX'


def test_skipped_zone_clock_rewind_and_invalid_station():
    m=MissionSupervisor(config()); m.update(ctx(15))
    assert m.update(ctx(45,1.1)) and m.reset_observations and m.policy.name=='HIGHWAY'
    assert m.update(ctx(46,.1)) and m.reset_observations
    assert m.update(ctx(math.nan,2)) and m.policy.name=='LOCALIZATION_INVALID'
    assert m.policy.speed_mps==0
    assert m.update(ctx(46,3)) and m.policy.name=='HIGHWAY'


def test_highway_removes_legacy_cruise_cap_without_removing_safety_stop():
    m=MissionSupervisor(config()); c=ctx(45);m.update(c);m.apply_context(c)
    assert math.isinf(Cruise().compute_target(c).v_max)
    t=m.limit(Target(stop_s=48,v_max=0)); assert t.v_max==0 and t.stop_s==48
    c=ctx(25);m.update(c);m.apply_context(c)
    assert Cruise().compute_target(c).v_max==pytest.approx(20/3.6)


def test_pedestrian_permission_is_independent_of_global_switch():
    m=MissionSupervisor(config()); c=ctx(35);m.update(c);m.apply_context(c)
    assert not Yield().pedestrian_active(c)
    c=ctx(25);m.update(c);m.apply_context(c)
    assert Yield().pedestrian_active(c)
    assert not Yield(pedestrian_enabled=False).pedestrian_active(c)


@pytest.mark.parametrize('mutation', ['overlap','duplicate','negative_speed','nan','typo'])
def test_bad_config_rejected(mutation):
    d=config()
    if mutation=='overlap':d['zones'][1]['start_s']=19
    elif mutation=='duplicate':d['zones'][1]['name']='BOX'
    elif mutation=='negative_speed':d['zones'][0]['speed_kmh']=-1
    elif mutation=='nan':d['zones'][0]['start_s']=math.nan
    else:d['zones'][0]['pedestrain']=True
    with pytest.raises(ValueError): MissionSupervisor(d)


def test_skipping_an_entire_zone_still_resets_two_default_intervals():
    m=MissionSupervisor(config())
    m.update(ctx(5.,now=0.,v=20.))
    assert m.update(ctx(35.,now=2.,v=20.))
    assert m.policy.name=='CRUISE' and not m.reset_observations


def test_pedestrian_speed_cap_released_only_after_actual_yield_and_verified_clear():
    d=config();d['zones'][1]['after_pedestrian_yield_kmh']=50.
    m=MissionSupervisor(d);c=ctx(25.);m.update(c)
    assert not m.observe_pedestrian_yield(active=False,pedestrian=False,verified_clear=True)
    assert m.speed_mps==pytest.approx(20/3.6)
    m.observe_pedestrian_yield(active=True,pedestrian=True,verified_clear=False)
    assert not m.observe_pedestrian_yield(active=False,pedestrian=False,verified_clear=False)
    assert m.speed_mps==pytest.approx(20/3.6)  # disappearance is not verified passage
    assert m.observe_pedestrian_yield(active=False,pedestrian=False,verified_clear=True)
    assert m.speed_mps==pytest.approx(50/3.6)
    m.apply_context(c)
    assert c.mission_pedestrian and Cruise().compute_target(c).v_max==pytest.approx(50/3.6)
    assert m.limit(Target(v_max=0,stop_s=28)).v_max==0
    m.update(ctx(15,2));m.update(ctx(25,3))
    assert not m.pedestrian_yield_completed and m.speed_mps==pytest.approx(20/3.6)


def test_pedestrian_cap_latch_resets_on_respawn_inside_same_mission():
    d=config();d['zones'][1]['start_s']=20.;d['zones'][1]['end_s']=39.
    d['zones'][1]['after_pedestrian_yield_kmh']=50.
    m=MissionSupervisor(d);m.update(ctx(35.))
    m.observe_pedestrian_yield(active=True,pedestrian=True,verified_clear=False)
    m.observe_pedestrian_yield(active=False,pedestrian=False,verified_clear=True)
    m.update(ctx(25.,2))
    assert not m.pedestrian_yield_completed and m.speed_mps==pytest.approx(20/3.6)


def test_a_vehicle_label_in_the_pedestrian_mission_is_a_pedestrian():
    # 2026-09-30 11:10 lap: a person crossing 4-10 m ahead carried a VEHICLE label (sized as a car on the sidewalk) and
    # the pedestrian mission ignored it; no NPC vehicle appears there.
    from foscar_planning.state import EventInfo
    m=MissionSupervisor(config())
    e=EventInfo('VEHICLE',1.,s=26.,d=0.5,x=26.,y=.5,vx=0.,vy=-1.5,size_x=.6,size_y=.5,value='lidar:x:130',path_relevant=True)
    c=ctx(25.);c.events=[e];m.update(c);m.apply_context(c)
    assert [x.type for x in c.events]==['PEDESTRIAN']
    c=ctx(5.);c.events=[e];m.update(c);m.apply_context(c)     # outside it a vehicle stays a vehicle
    assert [x.type for x in c.events]==['VEHICLE']
    from dataclasses import replace
    c=ctx(25.);c.events=[replace(e,size_x=4.8,size_y=2.)];m.update(c);m.apply_context(c)    # car-sized: not a person
    assert [x.type for x in c.events]==['VEHICLE']
