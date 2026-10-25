import math
import numpy as np
from foscar_planning.state import EventInfo
from foscar_planning.states.avoid import Avoid
from foscar_planning.avoid_planner import Box
from foscar_planning import tunnel_slalom
from test_tunnel_avoidance import context, road


def test_static_mission_returns_a_whole_body_checked_chain():
    c=context();c.mission_name='STATIC_OBSTACLE'
    a=Avoid();assert a.check_entry(c)
    t=a.compute_target(c)
    assert t.chain and t.chain[-1][0]==0.
    end=max(st+ln for _,st,ln in t.chain)
    assert tunnel_slalom.first_hit(a.boxes(),t.chain,0.,c.s,end,a.front_m,a.rear_overhang_m,a.planner.half) is None


def test_static_mission_keeps_objects_outside_original_path_before_swerving():
    c=context();c.mission_name='STATIC_OBSTACLE'
    c.events.append(EventInfo(type='OBSTACLE',stamp=1,s=43,d=3.4,size_x=.6,size_y=.6,value='lidar:side'))
    a=Avoid();a.observe(c)
    assert any(b.d_max>=3.7 for b in a.boxes())


def test_current_fragments_cannot_be_cut_off_by_robust_memory_quantiles():
    c=context();a=Avoid()
    c.events=[EventInfo(type='OBSTACLE',stamp=1,s=40,d=d,size_x=.5,size_y=.5,value=f'lidar:{i}')
              for i,d in enumerate([0.,.25,.5,.75,1.0,1.25,1.5])]
    a.observe(c)
    assert max(b.d_max for b in a.boxes())>=1.75
    assert min(b.d_min for b in a.boxes())<=-.25


def test_rotated_event_extents_are_projected_to_route():
    c=context();c.events=[EventInfo(type='OBSTACLE',stamp=1,s=40,d=0,size_x=4,size_y=1,box_yaw=math.pi/2)]
    a=Avoid();a.observe(c)
    assert max(b.d_max for b in a.boxes())>=2.


def test_same_count_changed_geometry_invalidates_committed_chain_immediately():
    c=context();a=Avoid(tunnel_extra_margin_m=0.);a.check_entry(c);a.compute_target(c)
    c.now+=.1
    # Same number of objects, but the object now blocks the committed path.
    c.events=[EventInfo(type='OBSTACLE',stamp=c.now,s=40,d=2.5,size_x=3,size_y=2,value='lidar:grown')]
    a.observe(c);t=a.compute_target(c)
    if t.v_max>0 and t.chain:
        end=max(st+ln for _,st,ln in t.chain)
        assert tunnel_slalom.first_hit(a.boxes(),t.chain,a.slalom_origin,c.s,end,a.front_m,a.rear_overhang_m,a.tunnel_planner.half) is None
    else:
        assert t.v_max==0 or math.isfinite(t.stop_s)


def test_confirmed_small_fragments_remain_collision_constraints():
    c=context();c.events=[EventInfo(type='OBSTACLE',stamp=1,s=40,d=d,size_x=.2,size_y=.2,value=f'lidar:{i}')
                           for i,d in enumerate([-.8,-.4,0.,.4,.8])]
    a=Avoid();assert a.check_entry(c)
    t=a.compute_target(c)
    assert t.lat_offset!=0 or t.v_max==0


def test_occluded_fragment_extents_survive_until_rear_body_clears():
    c=context();a=Avoid(tunnel_extra_margin_m=0.)
    c.events=[EventInfo(type='OBSTACLE',stamp=1,s=40,d=d,size_x=.5,size_y=.5,value=f'lidar:{i}')
              for i,d in enumerate([0.,.25,.5,.75,1.,1.25,1.5])]
    a.observe(c)
    c.s=37.;c.now+=.1;c.events=[]
    a.observe(c)
    assert tunnel_slalom.sweep_hits(a.boxes(),2.6,2.6,37.,1.,37.,41.5,a.front_m,a.rear_overhang_m,a.tunnel_planner.half)
    c.s=45.;c.now+=1.;a.observe(c)
    assert not a.boxes()


def test_static_wide_body_can_use_all_mapped_pavement():
    c=context();c.mission_name='STATIC_OBSTACLE';c.events[0].size_y=4.
    a=Avoid();a.check_entry(c);out=a.compute_target(c)
    assert not math.isfinite(out.stop_s)
    assert out.chain and out.lat_offset>3.


def test_longitudinal_fragments_survive_until_actual_rear_clearance():
    c=context();a=Avoid(tunnel_extra_margin_m=0.)
    c.events=[EventInfo(type='OBSTACLE',stamp=1,s=s,d=0,size_x=.5,size_y=.5,value=f'lidar:{i}')
              for i,s in enumerate([38,38.5,39,39.5,40,40.5,41])]
    a.observe(c)
    c.s=37.;c.now+=.1;c.events=[];a.observe(c)
    assert max(b.s_max for b in a.boxes())>=41.25
    c.s=41.6;c.now+=.1;a.observe(c)
    assert max(b.s_max for b in a.boxes())>=41.25
    c.s=45.;c.now+=1;a.observe(c)
    assert not a.boxes()


def test_static_mission_vehicle_shape_is_an_occupied_obstacle_not_a_mover():
    c=context();c.mission_name='STATIC_OBSTACLE';c.events[0].value='lidar:truck'
    a=Avoid();assert a.check_entry(c)
    c.now+=.1;c.events[0].stamp=c.now;c.events[0].type='VEHICLE'
    a.observe(c)
    assert a.memory and not a.dynamic
    out=a.compute_target(c)
    assert out.chain or out.v_max==0.
