"""Tunnel road users must yield rather than become stationary slalom boxes."""
import math
import pytest
from foscar_planning.state import Behavior, EventInfo
from foscar_planning.states.avoid import Avoid
from foscar_planning.states.yield_ import Yield
from test_tunnel_avoidance import context


@pytest.mark.parametrize('kind', ['PEDESTRIAN', 'DYNAMIC_UNKNOWN'])
def test_tunnel_road_user_is_not_a_static_slalom_box(kind):
    c=context();c.mission_pedestrian=True
    c.events=[EventInfo(type=kind,stamp=c.now,s=28,d=0,size_x=1.,size_y=.7,vy=1.,value='lidar:mover')]
    a=Avoid()
    assert not a.check_entry(c)
    assert not a.memory
    assert Yield().check_entry(c)


def test_tunnel_moving_track_removes_its_old_static_memory():
    c=context();a=Avoid();assert a.check_entry(c)
    c.now+=.1
    c.events=[EventInfo(type='DYNAMIC_UNKNOWN',stamp=c.now,s=40,d=0,size_x=3,size_y=2,vy=1.)]
    a.observe(c)
    assert not a.memory


def test_yield_preserves_the_static_obstacle_avoidance_curve():
    c=context();c.mission_pedestrian=True
    a=Avoid(tunnel_extra_margin_m=0.);b=Behavior([Yield(),a])
    initial=b.tick(c)
    assert initial.lat_offset != 0.
    c.now+=.1
    c.events.append(EventInfo(type='PEDESTRIAN',stamp=c.now,s=28,d=0,size_x=.6,size_y=.6,vy=1.,value='lidar:person'))
    out=b.tick(c)
    assert out.state=='YIELD' and math.isfinite(out.stop_s)
    lateral=b.last_targets['AVOID']
    assert (out.lat_offset,out.entry_s,out.entry_length_m,out.chain)==(
        lateral.lat_offset,lateral.entry_s,lateral.entry_length_m,lateral.chain)
    assert len(a.memory)==1  # the separate parked obstacle remains


@pytest.mark.parametrize('identified', [False, True])
def test_nearby_pedestrian_does_not_erase_a_separate_static_obstacle(identified):
    c=context();c.events[0].value='lidar:barrier' if identified else ''
    a=Avoid();assert a.check_entry(c)
    c.now+=.1;c.events[0].stamp=c.now
    c.events.append(EventInfo(type='PEDESTRIAN',stamp=c.now,s=40,d=2.,size_x=.6,size_y=.6,
                             value='lidar:person' if identified else ''))
    a.observe(c)
    assert len(a.memory)==1


def test_same_identified_track_reclassified_as_moving_loses_static_memory():
    c=context();c.events[0].value='lidar:changing';a=Avoid();assert a.check_entry(c)
    c.now+=.1;c.events=[EventInfo(type='PEDESTRIAN',stamp=c.now,s=40,d=0,size_x=3,size_y=2,value='lidar:changing')]
    a.observe(c)
    assert not a.memory
    c.now+=.1;c.events[0].type='OBSTACLE';c.events[0].stamp=c.now
    a.observe(c)
    assert not a.memory  # a transient stationary label cannot recreate the same moving body


def test_moving_member_of_merged_memory_does_not_delete_missing_static_member():
    c=context();c.events[0].value='lidar:barrier'
    c.events.append(EventInfo(type='OBSTACLE',stamp=c.now,s=40,d=1.2,size_x=.7,size_y=.7,value='lidar:person'))
    a=Avoid();assert a.check_entry(c)
    assert len(a.memory)==1
    c.now+=.1;c.events[0].stamp=c.now;c.events[1].stamp=c.now;c.events[1].type='PEDESTRIAN'
    a.observe(c)
    assert len(a.memory)==1
    c.now+=.1;c.events=c.events[1:];c.events[0].stamp=c.now
    a.observe(c)
    assert len(a.memory)==1
    assert a.memory[0].track_ids=={'lidar:barrier'}


def test_tunnel_parked_vehicle_is_avoided_without_yield():
    c=context();c.mission_pedestrian=True;c.events[0].type='VEHICLE'
    a=Avoid()
    assert a.check_entry(c)
    assert a.memory
    assert not Yield().check_entry(c)


def test_tunnel_yield_releases_clear_bypass_but_still_stops_on_actual_curve():
    c=context();c.mission_pedestrian=True;c.events=[]
    e=EventInfo(type='PEDESTRIAN',stamp=c.now,s=30,d=0,size_x=4.3,size_y=1.7,box_yaw=0.,value='lidar:parked')
    c.events=[e];y=Yield();assert y.check_entry(c)
    c.avoidance_offset=lambda s:3.25
    c.now+=.1;e.stamp=c.now
    assert not y.threatens(e,c)
    assert y.check_exit(c)
    e.d=3.25
    assert y.check_entry(c)
