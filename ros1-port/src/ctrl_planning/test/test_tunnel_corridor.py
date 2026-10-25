import math
import numpy as np
from ctrl_planning.ctrl_frenet import to_frenet
from ctrl_planning.ctrl_state import EventInfo
from ctrl_planning.states.ctrl_avoid import Avoid
from ctrl_planning.ctrl_tunnel_slalom import first_hit
from ctrl_planning.ctrl_mapped_avoidance import paved_moves
from ctrl_planning.ctrl_tunnel_corridor import WallMemory
from test_tunnel_avoidance import context


def walls(right=-3.7, left=7.4, s0=0., s1=90.):
    rows=[]
    for x in np.arange(s0,s1,1.):
        rows+= [[float(x),right,'R'],[float(x),left,'L']]
    return rows


def barricade_context():
    # 2026-09-26 tunnel: 3 m barricade reaching d=0.8; mapped pavement only -1.75..5.25.
    c=context();c.x,c.y,c.s,c.v=20.,0.,20.,8.
    c.events=[EventInfo(type='OBSTACLE',stamp=1,s=45,d=2.3,size_x=.4,size_y=3.)]
    return c


def test_map_only_blocks_but_measured_walls_open_the_right_side():
    c=barricade_context()
    # reaching d=0.6: with no pad on tunnel boxes (2026-09-29) the map pavement then has no room on the right
    c.events=[EventInfo(type='OBSTACLE',stamp=1,s=45,d=2.3,size_x=.4,size_y=3.4)]
    a=Avoid();assert a.check_entry(c)
    t=a.compute_target(c)
    assert t.v_max==0. and math.isfinite(t.stop_s)                  # map pavement: no room
    memory=WallMemory();assert memory.add(walls(),c.route,c.s,0.,to_frenet)
    c.road_map=memory.corridor(c.road_map)
    a=Avoid();assert a.check_entry(c);t=a.compute_target(c)
    assert not math.isfinite(t.stop_s) and t.v_max>=c.v
    assert t.lat_offset<0                                             # right, between wall and barricade


def test_walls_bound_the_offset_and_unseen_stations_use_the_map():
    c=barricade_context()
    memory=WallMemory();memory.add(walls(right=-1.9,left=7.4),c.route,c.s,0.,to_frenet)
    corridor=memory.corridor(c.road_map)
    # Measured wall at -1.9 (+0.2 clearance): a body edge at -1.8 is outside.
    assert not corridor.contains_road([[30.,-1.8]])[0] and corridor.contains_road([[30.,-1.6]])[0]
    # Beyond the measured stations the map decides again (edge -1.75 + seam 0.15).
    assert corridor.contains_road([[150.,-1.8]])[0] and not corridor.contains_road([[150.,-2.5]])[0]
    c.road_map=corridor
    a=Avoid();a.check_entry(c);t=a.compute_target(c)
    assert t.v_max==0.                                                # no room either side


def test_memory_keeps_walls_beside_and_behind_the_car():
    c=barricade_context();memory=WallMemory()
    memory.add(walls(s0=20.,s1=60.),c.route,20.,0.,to_frenet)
    memory.add(walls(s0=60.,s1=90.),c.route,60.,0.,to_frenet)   # later scans only see further ahead
    corridor=memory.corridor(None)
    assert corridor.contains_road([[25.,-3.],[85.,6.5]]).all()


def test_new_obstacle_beyond_the_cleared_one_is_a_new_manoeuvre():
    # Slalom: first barricade reaches from the left, second from the right.
    c=barricade_context();memory=WallMemory();memory.add(walls(s1=180.),c.route,c.s,0.,to_frenet)
    c.road_map=memory.corridor(c.road_map)
    a=Avoid();assert a.check_entry(c);first=a.compute_target(c)
    assert first.lat_offset<0
    # Past the first barricade (memory forgets it), the second appears further on.
    back=[m for m in first.chain if m[0]==0.][-1]                   # the way back (chain's last move)
    c.s=back[1]+1.;c.x=c.s;c.now+=5.
    c.events=[EventInfo(type='OBSTACLE',stamp=c.now,s=c.s+25.,d=.7,size_x=.4,size_y=3.)]
    a.check_exit(c)
    second=a.compute_target(c)
    assert not math.isfinite(second.stop_s)
    assert second.lat_offset>0
    # A new curve from the physical offset (partway into the return), complete before the box's
    # front, driven at the lateral limit at most.
    front=min(b.s_min for b in a.boxes())-a.front_m-a.entry_buffer_m
    end=max(st+ln for _,st,ln in second.chain)
    assert first_hit(a.boxes(),second.chain,a.slalom_origin,c.s,end,a.front_m,a.rear_overhang_m,a.tunnel_planner.half) is None
    assert paved_moves(c,a.slalom_origin,second.chain,c.s,end,a.front_m,a.rear_overhang_m,a.tunnel_planner.half)
    shift=second.lat_offset-a.start_offset
    assert first.lat_offset<=a.start_offset<=0.
    fit=a.swerve_speed(second.entry_length_m,shift)
    assert second.v_max<=math.sqrt(fit*fit+2*a.slalom_decel_mps2*max(0.,second.entry_s-c.s))+1e-6   # slowing down on the way in
    c.s=second.entry_s+second.entry_length_m+.5;c.x=c.s;c.now+=.1
    c.events=[EventInfo(type='OBSTACLE',stamp=c.now,s=second.entry_s+25.,d=.7,size_x=.4,size_y=3.)]
    a.check_exit(c)
    assert a.compute_target(c).v_max>=c.v                          # swerve cap released; speed held, not reduced


def test_committed_offset_moves_inward_when_the_wall_is_closer_than_assumed():
    # Committed farther out than needed (e.g. chosen before the box was fully seen);
    # the measured wall then leaves room only nearer the box: move inward, not stop.
    c=barricade_context();memory=WallMemory();memory.add(walls(),c.route,c.s,0.,to_frenet)
    c.road_map=memory.corridor(c.road_map,extend_m=0.)
    a=Avoid();assert a.check_entry(c);first=a.compute_target(c)
    assert first.lat_offset<0
    a.prev_d=held=first.lat_offset-1.                                  # farther out than minimal
    tight=WallMemory();tight.add(walls(right=held-a.tunnel_planner.half-.2+.05),c.route,c.s,0.,to_frenet)
    c.road_map=tight.corridor(None,extend_m=0.);c.now+=.1;c.s+=1.;c.x=c.s
    second=a.compute_target(c)
    assert not math.isfinite(second.stop_s)
    assert held<second.lat_offset<0

def test_blocked_committed_side_replans_to_the_other_side_instead_of_stopping():
    # A partial view makes left the smaller move; the full width then closes the left.
    c=barricade_context();memory=WallMemory();memory.add(walls(right=-4.8),c.route,c.s,0.,to_frenet)
    c.road_map=memory.corridor(c.road_map)
    c.events=[EventInfo(type='OBSTACLE',stamp=1,s=45,d=-.4,size_x=.4,size_y=1.2,value='lidar:t:1')]
    a=Avoid();assert a.check_entry(c);first=a.compute_target(c)
    assert first.lat_offset>0 and a.lateral_started
    c.now+=.1;c.s+=1.;c.x=c.s
    c.events=[EventInfo(type='OBSTACLE',stamp=c.now,s=45,d=2.1,size_x=.4,size_y=5.6,value='lidar:t:1')]
    a.check_exit(c)
    second=a.compute_target(c)
    assert not math.isfinite(second.stop_s) and second.lat_offset<0
    assert second.v_max>0


def test_tunnel_takes_the_smallest_move_that_clears_the_obstacle():
    # Barricade reaching from the left (d 0.8..3.8): right needs ~1.5 m, left ~5.3 m.
    c=barricade_context();memory=WallMemory();memory.add(walls(),c.route,c.s,0.,to_frenet)
    c.road_map=memory.corridor(c.road_map)
    a=Avoid();assert a.check_entry(c);t=a.compute_target(c)
    # the simple planner grows a box seen from afar away from the route only (its near face is what is seen)
    near=0. if a.slalom_planner=='simple' else a.tunnel_range_slack_per_m*max(0.,a.boxes()[0].s_min-c.s)
    box=a.boxes()[0];need=box.d_min-a.tunnel_planner.half-a.tunnel_box_jitter_m-near
    assert need-a.tunnel_planner.d_step<t.lat_offset<=need+1e-9

def test_wall_faces_on_the_pavement_or_far_away_do_not_move_the_wall():
    # A barricade's zero-intensity returns (on the road) must not become the wall, and
    # a far return at the tunnel mouth (d=+22 m) must not stick as the wall either.
    c=barricade_context();m=WallMemory()
    for _ in range(3):
        m.add(walls(),c.route,c.s,0.,to_frenet,c.road_map)
    m.add([[x,1.6,'L'] for x in range(30,40)],c.route,c.s,0.,to_frenet,c.road_map)   # on pavement
    m.add([[x,22.,'L'] for x in range(30,40)],c.route,c.s,0.,to_frenet,c.road_map)   # beyond 12 m
    m.add([[x,9.5,'L'] for x in range(30,40)],c.route,c.s,0.,to_frenet,c.road_map)   # one outlier
    assert all(abs(m.faces['L'][b][2]-7.4)<1e-6 for b in range(30,40))


def test_missing_pavement_beside_an_obstacle_holds_offset_and_brakes():
    c=barricade_context();memory=WallMemory();memory.add(walls(),c.route,c.s,0.,to_frenet)
    c.road_map=memory.corridor(c.road_map)
    a=Avoid();assert a.check_entry(c);first=a.compute_target(c)
    # Alongside the barricade (front wheels past its stop point) the plan turns blocked.
    c.s=44.;c.x=44.;c.now+=.1;c.road_map=None
    held=a.compute_target(c)
    assert held.v_max==0. and held.lat_offset==first.lat_offset


def test_short_wall_gap_behind_an_obstacle_is_bridged():
    c=barricade_context();m=WallMemory()
    rows=[r for r in walls() if not (40<=r[0]<47 and r[2]=='L')]                  # 7 m hidden
    m.add(rows,c.route,c.s,0.,to_frenet,c.road_map)
    corridor=m.corridor(None,extend_m=0.)
    assert len(corridor.runs)==1 and corridor.contains_road([[43.,6.5]])[0]
    m2=WallMemory();m2.add([r for r in walls() if not (30<=r[0]<60 and r[2]=='L')],c.route,c.s,0.,to_frenet)
    assert len(m2.corridor(None,extend_m=0.).runs)==2                               # 30 m: not bridged


def test_wall_side_follows_the_route_offset_not_the_lidar_label():
    # Car yawed in a swerve: right-wall faces arrive labelled 'L' (2026-09-26 bag).
    c=barricade_context();m=WallMemory()
    m.add(walls(),c.route,c.s,0.,to_frenet,c.road_map)
    m.add([[x,-3.7,'L'] for x in range(30,60)],c.route,c.s,0.,to_frenet,c.road_map)
    line=m.wall_line('L')
    assert all(abs(line[b]-7.4)<.05 for b in range(30,60))


def test_straight_wall_fit_ignores_a_few_inward_faces():
    # Off the pavement (d > 5.25) yet 1.4 m inside the wall at a few stations.
    c=barricade_context();m=WallMemory()
    m.add(walls(),c.route,c.s,0.,to_frenet,c.road_map)
    for _ in range(5):
        m.add([[x,6.,'L'] for x in (40,41,42)],c.route,c.s,0.,to_frenet,c.road_map)
    assert m.faces['L'][41][2]==6.                    # the raw median did move there
    line=m.wall_line('L')
    assert all(abs(line[b]-7.4)<.1 for b in range(35,48))
    assert m.corridor(None,extend_m=0.).contains_road([[41.,6.5]])[0]


def test_events_use_the_pose_at_scan_time():
    from collections import deque
    from types import SimpleNamespace
    from ctrl_planning.ctrl_behavior_node import BehaviorNode
    n=SimpleNamespace(pose_history=deque([(1.0,0.,0.,0.),(1.1,1.,0.,.1),(1.2,2.,0.,.2)]))
    x,y,yaw=BehaviorNode.pose_at(n,1.15)
    assert abs(x-1.5)<1e-9 and abs(yaw-.15)<1e-9
    assert BehaviorNode.pose_at(n,.5) is None                       # older than the history
    assert BehaviorNode.pose_at(n,1.22)==(2.,0.,.2)                 # just after the latest
