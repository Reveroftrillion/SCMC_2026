import math
import numpy as np
from ctrl_control.ctrl_core import Controller, Route, Target


def test_highway_has_no_fixed_cap_and_exit_brakes_to_general_limit():
    route=Route(np.c_[np.arange(1601.),np.zeros(1601)])
    c=Controller(route,speed_kmh=60.,continuous=True,uncapped_zones=[(200.,1400.)])
    assert np.isfinite(c.profile).all()
    # Not a replacement 100 km/h ceiling either.
    assert route.speed_at(800.,c.profile)>100/3.6
    assert np.max(c.profile[route.s<200.])<=60/3.6+1e-8
    assert np.max(c.profile[route.s>=1400.])<=60/3.6+1e-8
    assert route.speed_at(1390.,c.profile)<route.speed_at(1000.,c.profile)
    c.step(800.,0.,0.,20.,2,4,True,.05,Target())
    out=c.step(800.5,0.,0.,20.,2,4,True,.05,Target())
    assert out.target_speed>60/3.6
    out=c.step(801.,0.,0.,20.,2,4,True,.05,Target(v_max=10.))
    assert out.target_speed<=10.


def test_no_highway_policy_keeps_existing_cap_and_global_offset_is_applied():
    route=Route(np.c_[np.arange(1601.),np.zeros(1601)])
    default=Controller(route,speed_kmh=60.)
    assert default.profile.max()<=60/3.6
    shifted=Controller(route,speed_kmh=60.,s_offset=100.,uncapped_zones=[(200.,1300.)])
    assert shifted.profile[299]<=60/3.6 and shifted.profile[1400]<=60/3.6
    assert shifted.profile[800]>60/3.6


def test_speed_policy_uses_highway_only_and_respects_configured_cap(tmp_path):
    import yaml
    from ctrl_control.ctrl_speed_policy import uncapped_highway_zones
    path=tmp_path/'missions.yaml'
    zones=[dict(name='HIGHWAY',start_s=200,end_s=1400,speed_kmh=None),
           dict(name='TUNNEL',start_s=1400,end_s=1500,speed_kmh=None)]
    path.write_text(yaml.safe_dump(dict(zones=zones)))
    assert uncapped_highway_zones(str(path))==[(200.,1400.)]
    zones[0]['speed_kmh']=60
    path.write_text(yaml.safe_dump(dict(zones=zones)))
    assert uncapped_highway_zones(str(path))==[]
    assert uncapped_highway_zones('')==[]

def test_speed_limit_is_met_at_exit_between_route_samples():
    route=Route(np.c_[np.arange(0.,2001.,.5),np.zeros(4002)])
    c=Controller(route,speed_kmh=60.,continuous=True,uncapped_zones=[(100.,1500.2)])
    assert route.speed_at(1500.2,c.profile)<=60/3.6+1e-8

def test_ordinary_limit_is_kept_before_entry_between_route_samples():
    route=Route(np.c_[np.arange(0.,2001.,.5),np.zeros(4002)])
    c=Controller(route,speed_kmh=60.,continuous=True,uncapped_zones=[(100.2,1500.2)])
    assert route.speed_at(100.1,c.profile)<=60/3.6+1e-8
