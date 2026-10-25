import math
from pathlib import Path
import numpy as np
import yaml
from ctrl_control.ctrl_mission_config import load_mission_config
from ctrl_control.ctrl_core import Route
from ctrl_planning.ctrl_lane_prediction import LaneMap
from ctrl_planning.ctrl_state import Context, EventInfo, Behavior
from ctrl_planning.states.ctrl_avoid import Avoid
from ctrl_planning.states.ctrl_tunnel import Tunnel
from ctrl_planning.states.ctrl_yield import Yield
from ctrl_planning.ctrl_mission import MissionSupervisor


def road(left=True):
    # Own lane and opposite lane: crossing their shared centre line is allowed.
    return LaneMap([dict(idx=str(i),from_node_idx=str(i),to_node_idx=str(i+1),
                        width_start=3.5,points=[[x,d,0] for x in range(181)])
                    for i,d in enumerate([0,3.5 if left else -3.5])])


def context(left=True):
    c=Context(s=20,x=20,y=0,yaw=0,v=4,now=1,loc_mode='DR',mission_name='TUNNEL',
              route=Route(np.c_[np.arange(181.),np.zeros(181)]),
              events=[EventInfo(type='OBSTACLE',stamp=1,s=40,d=0,size_x=3,size_y=2)])
    c.road_map=road(left)
    return c


def test_tunnel_mission_enables_avoid_with_requested_practice_speed_cap():
    cfg=load_mission_config(Path(__file__).parents[1]/'config/missions.yaml')
    m=MissionSupervisor(cfg);c=context();c.s=1900;m.update(c)
    assert 'AVOID' in m.policy.allowed_states
    assert math.isclose(m.speed_mps, 50./3.6)
    # a person seen ahead: slower the nearer, down to 20 km/h close to them (user 2026-10-10, 2026-10-11)
    c.events=[EventInfo(type='PEDESTRIAN',stamp=0.,s=1920.,d=4.,x=0.,y=0.,size_x=.5,size_y=.5,value='lidar:p')]
    m.apply_context(c)
    assert 20./3.6 < m.speed_mps < 50./3.6
    c.events[0].s=1906.
    m.apply_context(c)
    assert math.isclose(m.speed_mps, 20./3.6)


def test_map_selects_left_or_right_without_centreline_penalty():
    for left in [True,False]:
        a=Avoid(tunnel_extra_margin_m=0.);c=context(left);assert a.check_entry(c)
        t=a.compute_target(c)
        assert not math.isfinite(t.stop_s)
        assert t.lat_offset*(1 if left else -1)>0
        assert abs(t.lat_offset)>=1.+a.tunnel_planner.half


def test_tunnel_missing_map_or_narrow_road_stops():
    for mapped in [None,LaneMap([dict(idx='a',from_node_idx='a',to_node_idx='b',width_start=3.5,
                                    points=[[x,0,0] for x in range(181)])])]:
        a=Avoid(tunnel_extra_margin_m=0.);c=context();c.road_map=mapped;assert a.check_entry(c)
        t=a.compute_target(c)
        assert math.isfinite(t.stop_s) and t.v_max==0.


def test_pedestrian_body_clear_releases_on_first_fresh_observation():
    c=context();c.mission_name='PEDESTRIAN';c.events=[EventInfo(type='PEDESTRIAN',stamp=1,s=35,d=0,
        size_x=.6,size_y=.6,vy=1,value='lidar:test:1')]
    y=Yield();assert y.check_entry(c)
    p=EventInfo(type='PEDESTRIAN',stamp=1.1,s=35,d=1.7,size_x=.6,size_y=.6,vy=1,
                value='lidar:test:1',box_yaw=0.)
    c.now=1.1;c.events=[p]
    assert y.check_exit(c)
    assert y.pedestrian_passage_confirmed


def test_pedestrian_overlap_or_returning_still_blocks():
    for d,vy in [(1.2,1.),(1.7,-1.)]:
        c=context();c.events=[EventInfo(type='PEDESTRIAN',stamp=1,s=35,d=0,size_x=.6,size_y=.6,vy=1,value='lidar:test:1')]
        y=Yield();assert y.check_entry(c)
        c.now=1.1;c.events=[EventInfo(type='PEDESTRIAN',stamp=1.1,s=35,d=d,size_x=.6,size_y=.6,vy=vy,value='lidar:test:1',box_yaw=0.)]
        assert not y.check_exit(c)


def test_tunnel_avoidance_keeps_offset_without_fixed_dr_speed_limit():
    c=context();b=Behavior([Avoid(tunnel_extra_margin_m=0.),Tunnel()]);t=b.tick(c)
    assert t.state=='AVOID' and t.lat_offset>0 and t.v_max>=c.v   # holds speed; no fixed cap


def test_committed_tunnel_side_that_closes_replans_to_the_open_side():
    # Committed side no longer paved, other side free: a new manoeuvre, not a stop
    # (was: hold the side and stop; 2026-09-26 sim loop stopped at a passable barricade).
    c=context();a=Avoid(tunnel_extra_margin_m=0.);assert a.check_entry(c);t=a.compute_target(c)
    assert t.lat_offset>0
    c.now+=.1;c.road_map=road(False)
    switched=a.compute_target(c)
    assert switched.lat_offset<0 and not math.isfinite(switched.stop_s) and switched.v_max>0


def test_map_union_does_not_bridge_a_wide_median():
    m=LaneMap([dict(idx=str(i),from_node_idx=str(i),to_node_idx=str(i+1),width_start=3.5,
                   points=[[x,d,0] for x in range(181)]) for i,d in enumerate([0,5])])
    assert not m.contains_road([[20,2.5]])[0]


def test_another_pedestrian_still_blocks_after_first_passes():
    c=context();p=EventInfo(type='PEDESTRIAN',stamp=1,s=35,d=0,size_x=.6,size_y=.6,vy=1,value='lidar:test:1')
    c.events=[p];y=Yield();assert y.check_entry(c)
    p.stamp=1.1;p.d=1.9;c.now=1.1
    c.events=[p,EventInfo(type='PEDESTRIAN',stamp=1.1,s=36,d=0,size_x=.6,size_y=.6,value='lidar:test:2')]
    assert not y.check_exit(c)


def drive_back(a, c, s_to, step=1.):
    """Drive the plan on (the slalom plans its way back once it clears the boxes) and return the way
    back's end: the chain's last move to 0."""
    back=None
    while c.s<s_to:
        c.s+=step;c.x=c.s;c.now+=.5
        a.check_exit(c)
        t=a.compute_target(c)
        moves=[m for m in (t.chain or ()) if m[0]==0.]
        if moves:
            back=moves[-1]
    return back


def test_reclassified_road_user_leaves_static_memory_and_returns_to_route():
    for kind in ['PEDESTRIAN', 'DYNAMIC_UNKNOWN']:
        c=context();a=Avoid(tunnel_extra_margin_m=0.);assert a.check_entry(c);t=a.compute_target(c)
        c.s=t.entry_s+2;c.now+=.1
        c.events=[EventInfo(type=kind,stamp=c.now,s=40,d=0,size_x=3,size_y=2)]
        assert not a.check_exit(c)
        held=a.compute_target(c)
        assert not a.memory  # road users now belong to YIELD in the tunnel
        assert held.chain and held.chain[-1][0] == 0.
        # Return through the slalom instead of retaining a phantom static road user.
        c.events=[];c.now+=10
        back=drive_back(a,c,60.)
        assert back is not None
        c.s=back[1]+back[2]+.1;c.x=c.s;c.now+=.5
        assert a.check_exit(c)


def test_tunnel_blocked_during_avoidance_preserves_curve_while_stopping():
    c=context();a=Avoid(tunnel_extra_margin_m=0.);assert a.check_entry(c);t=a.compute_target(c)
    # Neither side paved any more (own lane only): stop, keeping the committed curve.
    c.now+=.1;c.road_map=LaneMap([dict(idx='a',from_node_idx='a',to_node_idx='b',width_start=3.5,
                                       points=[[x,0,0] for x in range(181)])])
    stop=a.compute_target(c)
    assert stop.v_max==0.
    assert (stop.entry_s,stop.entry_length_m,stop.return_s,stop.return_length_m)==(
        t.entry_s,t.entry_length_m,t.return_s,t.return_length_m)


def test_tunnel_large_offset_gets_gentler_return_at_driving_speed():
    c=context();c.v=45.5/3.6;a=Avoid(tunnel_extra_margin_m=0.);assert a.check_entry(c)
    t=a.compute_target(c)
    assert not math.isfinite(t.stop_s)
    # Clear of the box by the body + margin (tunnel boxes carry no pad since 2026-09-29).
    box=a.boxes()[0]
    assert abs(t.lat_offset)-a.tunnel_planner.half>=box.d_max-1e-9 or abs(t.lat_offset)+a.tunnel_planner.half<=box.d_min+1e-9
    # The way back (planned once it clears the box) is as long as transition_time_s of travel at this
    # speed when there is room (user 2026-09-29: curve length grows with speed), searched in 1 m steps.
    # (events emptied here stand for a box out of sight; one gone from where the lidar must see it is dropped since
    # 2026-10-05, so that rule is off for this return-length check)
    c.events=[];a.tunnel_view_m=0.
    back=drive_back(a,c,70.)
    assert back is not None and back[2]>=c.v*a.transition_time_s-1.-1e-9


def test_tunnel_widens_committed_offset_instead_of_stopping():
    # 2026-09-26 run 044400: a side fragment grew the box into the held offset
    # at 49 km/h; the car stopped mid-swerve although a wider offset was clear.
    c=context()
    c.road_map=LaneMap([dict(idx=str(i),from_node_idx=str(i),to_node_idx=str(i+1),width_start=3.5,
                            points=[[x,d,0] for x in range(181)]) for i,d in enumerate([0,3.5,7.])])
    a=Avoid(tunnel_extra_margin_m=0.);assert a.check_entry(c);t=a.compute_target(c)
    assert t.lat_offset>0
    held=t.lat_offset
    c.s=t.entry_s+2;c.now+=.1
    # Same box (d within match_d_m), now reaching 0.1 m into the held body.
    reach=held-a.tunnel_planner.half+.1
    c.events=[EventInfo(type='OBSTACLE',stamp=c.now,s=40,d=1.,size_x=3,size_y=2*(reach-1.))]   # tunnel: no pad
    a.check_exit(c)
    assert not any(x.feasible for x in a.tunnel_planner.plan(a.boxes(),c.s,held).candidates if x.d==held)
    wider=a.compute_target(c)
    assert not math.isfinite(wider.stop_s) and wider.v_max>=c.v
    assert wider.lat_offset>held


def test_avoid_holds_speed_until_the_swerve_is_over():
    # 2026-09-27: between two tunnel barricades the car sped up 40 -> 53 km/h while
    # returning from the first, saw the second only 15 m ahead and hit it.
    c=context();c.v=11.;a=Avoid(tunnel_extra_margin_m=0.);assert a.check_entry(c)
    first=a.compute_target(c);assert first.lat_offset!=0 and first.v_max<=11.+1e-9
    c.s+=5;c.x=c.s;c.now+=.5;c.v=9.;a.check_exit(c)
    # follows the car down, allowing one second of the slow rise (2026-09-28 hold ramp)
    assert a.compute_target(c).v_max<=9.+a.hold_ramp_mps2+1e-9
    c.s+=5;c.x=c.s;c.now+=.5;c.v=10.;a.check_exit(c)
    assert a.compute_target(c).v_max<=9.+a.hold_ramp_mps2*1.5+1e-9   # only the slow rise, not the car's jump
    c.events=[]
    back=drive_back(a,c,80.)
    c.s=back[1]+back[2]+1.;c.x=c.s;c.now+=10.
    assert a.check_exit(c) and math.isinf(a.hold_v)    # released when AVOID ends


def test_static_zone_avoids_a_box_misread_as_dynamic_unknown():
    # 2026-09-27 s325: the parked box came through as DYNAMIC_UNKNOWN at 59 km/h and was hit.
    def ctx_with(kind, mission):
        c=Context(s=20,x=20,y=0,yaw=0,v=15,now=1,mission_name=mission,
                  route=Route(np.c_[np.arange(181.),np.zeros(181)]),
                  events=[EventInfo(type=kind,stamp=1,s=45,d=0,size_x=1,size_y=1.5)])
        c.road_map=road();return c
    a=Avoid(tunnel_extra_margin_m=0.);assert a.check_entry(ctx_with('DYNAMIC_UNKNOWN','STATIC_OBSTACLE'))
    assert Avoid(tunnel_extra_margin_m=0.).check_entry(ctx_with('OBSTACLE','STATIC_OBSTACLE'))
    assert not Avoid(tunnel_extra_margin_m=0.).check_entry(ctx_with('DYNAMIC_UNKNOWN','CRUISE'))   # elsewhere: still a mover
