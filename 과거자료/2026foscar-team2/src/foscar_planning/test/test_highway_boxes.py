import pytest
import math
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
from foscar_planning.highway_boxes import lane_boxes,FREE,OCCUPIED,UNKNOWN
from foscar_planning.highway_box_tracks import lane_runs,run_speeds,EdgeTracks
from foscar_planning.highway_config import HighwayConfig
from foscar_planning.highway_policy import HighwayPolicy
from foscar_planning.highway_observation import ObservationSnapshot
from foscar_planning.state import Context
from test_highway_route import highway, MAPS
from foscar_planning.highway_route import HighwayRoute


def highway_route():
    return HighwayRoute.from_files(MAPS/'maps/link_set.json',MAPS/'routes/global_path.txt',MAPS/'routes/checkpoints.txt',
        start_checkpoint=10,hold_checkpoint=None,rejoin_checkpoint=12,end_checkpoint=13,excluded_lanes=[1])

S=1300.


def config():
    return HighwayConfig.load(Path(__file__).parents[1]/'config/highway.yaml',
                              control_config=Path(__file__).parents[2]/'foscar_bringup/config/low_speed.yaml')


def ego(road,s=S,lane=4,v=14.):
    x,y=road.at(lane,s);a=np.array(road.at(lane,s+1.))-np.array(road.at(lane,s-1.))
    return Context(s=s,x=x,y=y,yaw=math.atan2(a[1],a[0]),v=v,now=1.,mission_name='HIGHWAY')


def car(road,lane,s,key,ready=True,v=14.):
    pts=[]
    for ds in (-2.,2.):
        c=np.array(road.at(lane,s+ds));a=np.array(road.at(lane,s+ds+.5))-np.array(road.at(lane,s+ds-.5))
        u=a/np.linalg.norm(a);n=np.array([-u[1],u[0]])
        pts+=[c+.9*n,c-.9*n]
    u=np.array(road.at(lane,s+1.))-np.array(road.at(lane,s-1.));u=u/np.linalg.norm(u)*v
    return SimpleNamespace(id=key,xy=np.array(pts),speed_ready=ready,velocity_bounds=(u[0],u[0],u[1],u[1]) if ready else (-math.inf,math.inf,-math.inf,math.inf))


def snap(*objs,t=1.):return ObservationSnapshot(t,True,'OK',{o.id:o for o in objs})


def moved(obj,dt):
    """The car `dt` s later at its own velocity (speeds come from how box edges move, so the world must move)."""
    vx,_,vy,_=obj.velocity_bounds
    if not all(map(math.isfinite,(vx,vy))):return obj
    return SimpleNamespace(**{**vars(obj),'xy':obj.xy+np.array([vx,vy])*dt})


def drive(p,e,objs,ticks=8):
    """Tick a moving world every 0.1 s: cars at their velocity, ego along its heading at its speed. Speeds are known
    from the third scan on (box edges followed over 0.2 s)."""
    out=None
    for k in range(ticks):
        t=1.+.1*k;d=e.v*(t-1.)
        c=replace(e,now=t,s=e.s+d,x=e.x+d*math.cos(e.yaw),y=e.y+d*math.sin(e.yaw))
        out=p.tick(c,snap(*[moved(o,t-1.) for o in objs],t=t))
        if out.reason=='CHANGE_STARTED':break
    return out


def policy(lane,cp11=True):
    p=HighwayPolicy(highway_route(),config());p.current_lane=lane;p.cp11=cp11;return p


def state_at(boxes,s):return next(b[2] for b in boxes if b[0]<=s<b[1])


def test_empty_target_lane_is_all_free_boxes(highway):
    e=ego(highway);boxes=lane_boxes(highway,config(),e,snap(),3)
    assert boxes and all(b[2] in (FREE,UNKNOWN) for b in boxes)
    assert all(b[2]==FREE for b in boxes if abs((b[0]+b[1])/2-S)<10.)


def test_car_beside_occupies_the_boxes_it_covers(highway):
    boxes=lane_boxes(highway,config(),ego(highway),snap(car(highway,3,S+1.,'side')),3)
    assert state_at(boxes,S+1.)==OCCUPIED


def test_boxes_hidden_behind_a_car_are_unknown_not_free(highway):
    # Seen past a car 10 m ahead in the target lane, the lane just behind it is not known to be empty.
    boxes=lane_boxes(highway,config(),ego(highway),snap(car(highway,3,S+10.,'front')),3)
    assert state_at(boxes,S+10.)==OCCUPIED
    assert state_at(boxes,S+16.)==UNKNOWN
    assert state_at(boxes,S+4.)==FREE


def test_a_car_on_the_far_lane_line_does_not_fill_the_target_lane(highway):
    # 00:53 lap 3: a lane-2 car whose edge sat on the 2/3 line filled lane-3 boxes for 190 m.
    e=ego(highway)
    a=np.array(highway.at(3,S));b=np.array(highway.at(2,S));n=(b-a)/np.linalg.norm(b-a)
    half=highway.width(3,S)/2
    def body(inside):   # lane-2 car whose inner edge reaches `inside` m into lane 3
        c=a+n*(half-inside+.95)
        u=np.array([-n[1],n[0]])
        return SimpleNamespace(id='far',xy=np.array([c-.95*n-2*u,c+.95*n-2*u,c+.95*n+2*u,c-.95*n+2*u]),speed_ready=True,velocity_bounds=(0,0,0,0))
    assert state_at(lane_boxes(highway,config(),e,snap(body(.2)),3),S)==FREE
    assert state_at(lane_boxes(highway,config(),e,snap(body(.8)),3),S)==OCCUPIED


def test_deliberating_a_change_keeps_the_safe_distance_to_our_lead(highway):
    # User 2026-09-28: close to our own lead all the while it considers a change is too dangerous; only once
    # the change starts may it close in. (The 00:53 stoppable-gap-only 'waiting' relaxation is gone.)
    p=HighwayPolicy(highway,config());e=ego(highway,lane=4,v=16.)
    p.current_lane=4;p.cp11=True;ref=p._reference(e)
    lead=car(highway,4,S+25.,'lead',v=12.5)
    p.state='FOLLOW';following,*_=p._follow_cap(e,snap(lead),ref)
    p.state='WAIT_GAP';waiting,*_=p._follow_cap(e,snap(lead),ref)
    assert waiting==following


def test_a_car_closing_on_our_rear_corner_is_not_an_emergency_stop(highway):
    # 08:10 run 8: mid-change, a lane-3 car closing on our rear corner read as CURRENT_OCCUPIED, the car
    # braked 0.5 in front of it and was hit from behind.
    from foscar_planning.highway_maneuver import build_maneuver
    p=HighwayPolicy(highway,config());e=ego(highway,lane=4,v=12.)
    p.current_lane=4;ref=p._reference(e)
    x,y=np.array(highway.at(4,S-2.5))                 # overlapping our rear half in our own corridor
    behind=SimpleNamespace(id='behind',xy=np.array([[x-.9,y+2.],[x+.9,y+2.],[x+.9,y-2.],[x-.9,y-2.]]),speed_ready=True,velocity_bounds=(0,0,-15.,-15.))
    cap,reason,*_=p._follow_cap(e,snap(behind),ref)
    assert reason!='CURRENT_OCCUPIED' and cap>0.


def test_mid_change_the_target_lane_car_ahead_is_followed_outside_the_curve(highway):
    # 08:38 run 5: a curve laid out for 108 km/h kept a 39 km/h lane-4 car 35 m ahead outside the
    # corridor; the car sped up 62 -> 88 km/h and braked 11 m short of it (hit).
    from foscar_planning.highway_maneuver import build_maneuver
    cfg=config();p=HighwayPolicy(highway,cfg);e=ego(highway,lane=4,v=20.)
    p.current_lane=4;p.committed=m=build_maneuver(highway,4,3,S+2.,90.,20.,cfg)
    lead=car(highway,3,S+20.,'lead',v=10.)                        # 14 m past our front: shed 10 m/s needs ~17 m
    cap,*_=p._follow_cap(e,snap(lead),m.reference)
    assert cap<20. and p.follow_object=='lead'
    beside=car(highway,3,S+1.,'beside',v=20.)                    # target-lane car beside: not a lead
    cap,*_=p._follow_cap(e,snap(beside),m.reference)
    assert p.follow_object!='beside'


def test_mid_change_a_lead_only_needs_a_stoppable_gap(highway):
    # User 2026-09-28: keep speed while changing lanes and open the gap afterwards.
    from foscar_planning.highway_maneuver import build_maneuver
    cfg=config();e=ego(highway,lane=4,v=20.)
    lead=car(highway,4,S+14.,'lead',v=18.)             # ~8 m past our front, 2 m/s slower
    p=HighwayPolicy(highway,cfg);p.current_lane=4
    following,*_=p._follow_cap(e,snap(lead),p._reference(e))
    q=HighwayPolicy(highway,cfg);q.current_lane=4;q.committed=m=build_maneuver(highway,4,3,S+2.,60.,20.,cfg)
    changing,*_=q._follow_cap(e,snap(lead),m.reference)
    assert changing>following and changing>=18.


def test_ordinary_following_reopens_the_safe_distance_a_cut_in_may_sit_inside_it(highway):
    # User 2026-09-28: tuck in close only while cutting in (time_gap_change_s < time_gap_s); afterwards slow gently
    # back to a safe distance.
    from foscar_planning.highway_maneuver import build_maneuver
    cfg=config();e=ego(highway,lane=3,v=15.)
    lead=car(highway,3,S+10.,'lead',v=15.)                     # ~4 m past our front at equal speed
    p=HighwayPolicy(highway,cfg);p.current_lane=3
    ordinary,*_=p._follow_cap(e,snap(lead),p._reference(e))
    assert ordinary<15.
    q=HighwayPolicy(highway,cfg);q.current_lane=4;q.committed=m=build_maneuver(highway,4,3,S-10.,45.,15.,cfg)
    changing,*_=q._follow_cap(e,snap(lead),m.reference)
    assert changing>ordinary                       # mid-change: the shorter change time gap brakes less


def test_a_small_unmeasured_fragment_of_a_moving_lead_moves_with_it(highway):
    # 2026-09-28 10:16: fragments split off a moving lead (new id, never measured) were braked for as stationary.
    # In the boxes they are part of the lead's run and get its speed.
    e=ego(highway,lane=3,v=15.)
    lead=car(highway,3,S+14.,'lead',v=15.)
    c=np.array(highway.at(3,S+11.6))
    p=HighwayPolicy(highway,config());p.current_lane=3;p.cp11=True
    for k in range(5):
        dt=.1*k;fr=SimpleNamespace(id=f'frag{k}',xy=np.array([c+[.2,.2],c+[-.2,.2],c+[.2,-.2]])+moved(lead,dt).xy.mean(axis=0)-lead.xy.mean(axis=0),
                                    speed_ready=False,velocity_bounds=(-math.inf,math.inf,-math.inf,math.inf))
        occ,speed,s2=p._occupancy(replace(e,now=1.+dt,s=e.s+15.*dt),snap(moved(lead,dt),fr,t=1.+dt),(3,2))
    assert speed[f'frag4']==pytest.approx(speed['lead']) and speed['lead']==pytest.approx(15.,abs=.5)

def test_mid_change_keeps_the_speed_behind_a_close_lead(highway):
    # User 2026-09-30: through the cut-in keep the speed (change values: short gap, firm approach); the safe distance
    # reopens once our body has left the old lane. A lead 6 m past our front at our speed does not brake the change.
    from foscar_planning.highway_maneuver import build_maneuver
    cfg=config();e=ego(highway,lane=3,v=15.)
    lead=car(highway,3,S+12.,'lead',v=15.)                     # ~6 m past our front: inside the ~8.2 m safe distance
    q=HighwayPolicy(highway,cfg);q.current_lane=4;q.committed=m=build_maneuver(highway,4,3,S-10.,45.,15.,cfg)
    changing,*_=q._follow_cap(e,snap(lead),m.reference)
    assert 15.<=changing<math.inf


REAR=S-(4.635-3.8)          # our rear bumper station at S


def test_empty_target_lane_changes_now_at_our_speed(highway):
    p=policy(4);out=drive(p,ego(highway,lane=4,v=15.),[])
    assert out.reason=='CHANGE_STARTED'
    assert out.v_max>15.                              # no speed limit of its own while changing


def test_a_car_beside_at_our_speed_is_passed_not_followed(highway):
    # Relative speed 0: in its frame the box ahead of it stands still and we reach it by speeding up.
    p=policy(4);out=drive(p,ego(highway,lane=4,v=15.),[car(highway,3,S+1.,'side',v=15.)],ticks=4)
    assert p.plan.mode=='ACCEL' and p.plan.rear!='' and p.plan.front==''          # ahead of the car beside


def test_stuck_behind_a_slow_lead_the_change_speeds_up_past_a_closing_car_behind(highway):
    # 13:01-14:36 batches: ~900 ticks refused a lane-3 car 3-10 m back closing ~3 m/s that speeding up outruns
    # (user: "왼쪽 뒤에 있는차가 멀리있는데도 끼질 못하네").
    s=1360.;e=ego(highway,s=s,lane=4,v=10.);rear_s=s-(4.635-3.8)
    lead=car(highway,4,s+26.,'lead',v=10.)
    rear=car(highway,3,rear_s-8.-2.,'rear',v=13.)                # 8 m back in lane 3, closing 3 m/s
    p=policy(4);out=drive(p,e,[lead,rear])
    assert out.reason=='CHANGE_STARTED' and out.diagnostics['admission']=='SLOT_CUT_IN',(out.reason,p.plan)


def test_a_fast_car_closing_from_behind_is_let_by_first(highway):
    # 10 m/s faster 6 m back: no speed-up outruns it within the change.
    p=policy(4);drive(p,ego(highway,lane=4,v=15.),[car(highway,3,REAR-6.-2.,'fast',v=25.)],ticks=6)
    assert p.plan is None or (p.plan.rear=='' and p.plan.front!='')                 # it goes by first, we land behind


def test_a_slow_lane_four_car_at_entry_is_passed_and_cut_in_ahead_of(highway):
    # TuningPoint 3: pass the slower lane-4 car beside us at entry instead of going in behind it (5 m/s faster, our
    # own speed already passes it; a same-speed car needs the speed-up).
    s=1135.;e=ego(highway,s=s,lane=5,v=16.)
    p=policy(5,cp11=False);drive(p,e,[car(highway,4,s+3.,'slow',v=11.)],ticks=4)
    assert p.plan is not None and p.plan.rear!='' and p.plan.front==''
    q=policy(5,cp11=False);drive(q,e,[car(highway,4,s+3.,'even',v=16.)],ticks=4)
    assert q.plan is not None and q.plan.mode=='ACCEL' and q.plan.rear!='' and q.plan.front==''


def test_two_lane_four_cars_at_entry_the_front_one_is_passed(highway):
    # User 2026-09-28: with two cars in lane 4 at entry, pass the front-most.
    s=1135.;e=ego(highway,s=s,lane=5,v=16.)
    cars=[car(highway,4,s+3.,'first',v=11.),car(highway,4,s+14.,'second',v=11.)]
    p=policy(5,cp11=False);drive(p,e,cars,ticks=4)
    # (landing ahead of it and of the lane it hides from us: its shadow moves with it)
    assert p.plan is not None and p.plan.front=='' and len(p.plan.passing)==2,p.plan


def test_tail_in_only_into_a_narrow_slot_else_the_safe_distance(highway):
    # User 2026-09-28: tuck right behind the car ahead only when the target-lane slot is narrow. (Our own lead
    # 14 m ahead at the same speed rules out passing the target-lane car instead.)
    e=ego(highway,lane=4,v=15.);lead=car(highway,4,S+14.,'lead',v=15.)
    ahead=car(highway,3,S+10.,'ahead',v=15.)            # ~4 m past our front, same speed
    p=policy(4);drive(p,e,[lead,ahead],ticks=4)
    assert p.candidate is None and not (p.plan is not None and p.plan.start_t==0. and p.plan.front=='ahead')   # roomy
    behind=car(highway,3,REAR-8.-2.,'behind',v=15.)      # a car 8 m behind: narrow slot -> tail in now
    q=policy(4);out=drive(q,e,[lead,ahead,behind])
    assert out.reason=='CHANGE_STARTED' and q.committed is not None,(out.reason,q.plan)


def test_the_lead_in_the_lane_we_leave_does_not_block_a_change_we_can_stop_behind_it_in(highway):
    # ~4 m past our front at our speed: beyond the stop gap (standstill + position error; 1.2 m overlapped it, 2026-09-30)
    p=policy(4);out=drive(p,ego(highway,lane=4,v=12.),[car(highway,4,S+10.,'lead',v=12.)])
    assert out.reason=='CHANGE_STARTED'


def test_a_body_beside_without_measured_speed_holds_the_change(highway):
    # (a run needs 0.2 s of edge motion before its speed is known)
    p=policy(4);out=drive(p,ego(highway,lane=4,v=15.),[car(highway,3,S,'new',v=15.)],ticks=2)
    assert out.reason!='CHANGE_STARTED' and p.candidate is None


def test_lane_four_changes_to_three_without_waiting_for_the_old_cp11(highway):
    # CP11 (lane 4, s1286) was dropped from the rules (2026-09-30): nothing to pass in lane 4 before 4->3.
    p=HighwayPolicy(highway_route(),config());p.current_lane=4
    out=drive(p,ego(highway,s=1200.,lane=4,v=15.),[],ticks=4)
    assert out.reason=='CHANGE_STARTED' and p.committed.target_lane==3


def test_lane_five_past_the_last_curve_start_keeps_the_global_path_into_lane_four(highway):
    s=highway.change_deadline(5,config().policy.cut_in_min_sep_m)+3.
    x,y=highway.original.at(s);e=Context(s=s,x=x,y=y,yaw=highway.original.heading_at(s),v=15.,now=1.,mission_name='HIGHWAY')
    p=policy(5,cp11=False);out=drive(p,e,[],ticks=4)
    assert out.reason.startswith('TAPER_') and p.plan.kind=='TAPER' and p.committed is None


def test_lane_five_with_lane_four_packed_stops_before_the_taper(highway):
    s=1215.;x,y=highway.original.at(s);e=Context(s=s,x=x,y=y,yaw=highway.original.heading_at(s),v=15.,now=1.,mission_name='HIGHWAY')
    wall=[car(highway,4,s+k,f'c{k:+.0f}',v=0.) for k in np.arange(-30.,45.,5.5)]
    p=policy(5,cp11=False);out=drive(p,e,wall,ticks=4)
    assert out.reason=='NO_SLOT' and out.v_max<15.


def test_a_car_s_speed_comes_from_how_its_box_edges_move_whatever_its_ids(highway):
    # User 2026-09-28: no object ids -- the lidar's churned every few scans. The run's edges moving 22 m/s is the car.
    c=config();e=ego(highway,v=14.);tr=EdgeTracks(c.boxes.edge_window_s);v=None
    for k in range(6):
        t=1.+.1*k;body=moved(car(highway,3,S-15.,f'id{k}',v=22.),.1*k)      # a new id every scan
        ext=[];boxes=lane_boxes(highway,c,replace(e,now=t),snap(body,t=t),3,ext)
        runs=lane_runs(boxes,ext);v=run_speeds(tr,t,runs)
    assert len(runs)==1 and v[0]==pytest.approx(22.,abs=.5)

def test_planning_is_fast_enough_for_the_behavior_tick(highway):
    import time
    e=ego(highway,lane=4,v=20.)
    cars=[car(highway,3,S+k,f'c{k:+.0f}',v=18.) for k in (-40.,-25.,-12.,9.,22.,35.)]+[car(highway,4,S+25.,'lead',v=17.)]
    p=policy(4);p.tick(e,snap(*cars))
    t0=time.perf_counter()
    for k in range(10):p.tick(replace(e,now=1.+.01*k),snap(*cars,t=1.+.01*k))
    assert (time.perf_counter()-t0)/10<.04


def closed_loop(highway,lane,s0,v0,npcs,seconds=20.,cp11=True):
    """Ideal-tracking closed loop: NPCs (lane, station, speed) keep lane and speed (they do not brake for us); the ego
    follows the policy's reference exactly and its speed cap at +3.5 / -4 m/s2. Returns (lanes reached, closest
    approach (bumper gap) to any NPC overlapping us laterally, reasons). Stops at s1540: past it lanes 3 and 2
    converge and an NPC kept on lane 3's centre drives into lane 2 (it would merge in reality)."""
    from foscar_planning.highway_maneuver import heading_at
    p=policy(lane,cp11=cp11);s=s0;v=v0;ref=None;reasons=[];closest=math.inf;lanes=[lane]
    for k in range(int(seconds/.1)):
        t=1.+.1*k
        if s>1540.:break
        if ref is None:x,y=highway.at(lane,s);yaw=heading_at(highway,lane,s,1.)
        else:q=ref.q_at_global(s);x,y=ref.at_q(q);yaw=ref.heading_at(q)
        objs=[car(highway,l,s1+sv*.1*k,f'n{i}',v=sv) for i,(l,s1,sv) in enumerate(npcs) if s1+sv*.1*k<1585.]
        out=p.tick(Context(s=s,x=x,y=y,yaw=yaw,v=v,now=t,mission_name='HIGHWAY'),snap(*objs,t=t))
        reasons.append(out.reason)
        if out.reference is not None:ref=out.reference
        if p.current_lane!=lanes[-1]:lanes.append(p.current_lane)
        u=np.array([math.cos(yaw),math.sin(yaw)]);n=np.array([-u[1],u[0]])
        for o in objs:
            rel=o.xy.mean(axis=0)-np.array([x,y]);a=float(rel@u)-(3.8-(4.635-3.8))/2;b=abs(float(rel@n))
            if b<1.892:closest=min(closest,abs(a)-4.4)           # bumper gap to a body in our corridor
        v=float(np.clip(min(out.v_max,40.),v-.4,v+.35));v=max(0.,v);s+=v*.1
    return lanes,closest,reasons


def test_closed_loop_a_car_slowly_overtaking_beside_us_behind_a_slow_lead(highway):
    # 14:50 run 6 (s1285-1333): lane-3 car beside at 47-51 km/h, our lane-4 lead 15 m ahead at 41 km/h; the old
    # logic stayed beside it for 50 m. Easing off lets it by and the change follows it in.
    lanes,closest,reasons=closed_loop(highway,4,1300.,11.8,[(4,1319.,11.5),(3,1300.,13.)],seconds=15.)
    assert lanes[:2]==[4,3] and closest>1.,(lanes,closest,[r for i,r in enumerate(reasons) if i%10==0])


def test_closed_loop_dense_target_lane_gets_in_without_touching_anyone(highway):
    # Lane 3 at 15 m/s with 12-16 m gaps (cars 4 m long), our lead in lane 4 at 12 m/s.
    npcs=[(3,1290.+k*18.,15.) for k in range(6)]+[(4,1330.,12.)]
    lanes,closest,reasons=closed_loop(highway,4,1300.,13.,npcs,seconds=20.)
    assert lanes[:2]==[4,3] and closest>1.,(lanes,closest,[r for i,r in enumerate(reasons) if i%10==0])


def test_closed_loop_entry_passes_two_slow_lane_four_cars(highway):
    # User 2026-09-28: two cars in lane 4 at entry -- pass them and cut in ahead of the front one.
    npcs=[(4,1128.,11.),(4,1140.,11.)]
    lanes,closest,reasons=closed_loop(highway,5,1122.,15.,npcs,seconds=10.,cp11=False)
    assert lanes[:2]==[5,4] and closest>1.,(lanes,closest,[r for i,r in enumerate(reasons) if i%10==0])


def test_closed_loop_right_after_cp11_behind_a_close_lead_with_a_faster_car_behind(highway):
    # 13:24-14:50 batches, 20 runs: at CP11, 40 km/h 6 m behind a 40 km/h lead, a lane-3 car 8-13 m back at 50
    # km/h. The old logic swung PASS_AHEAD / FALL_BEHIND beside it for 40-100 m. Passing is blocked by the lead;
    # easing off lets it by and the change follows it in within a few seconds.
    npcs=[(4,1284.+12.,11.1),(3,1284.-12.,13.9)]
    lanes,closest,reasons=closed_loop(highway,4,1284.,11.1,npcs,seconds=10.)
    started=reasons.index('CHANGE_STARTED') if 'CHANGE_STARTED' in reasons else None
    assert lanes[:2]==[4,3] and started is not None and started*.1<5. and closest>1.,(lanes,closest,started,reasons[::5])


def test_mid_change_a_car_the_plan_passes_is_not_braked_for_before_we_reach_its_lane(highway):
    # 20:30 batch: every 5->4 pass braked for the lane-4 car it was passing (68 -> 17 km/h over 96 m, run 8).
    from foscar_planning.highway_maneuver import build_maneuver
    cfg=config();e=ego(highway,lane=4,v=20.)
    slow=car(highway,3,S+10.,'slow',v=12.)
    p=HighwayPolicy(highway,cfg);p.current_lane=4;p.committed=m=build_maneuver(highway,4,3,S+20.,60.,20.,cfg)
    braked,*_=p._follow_cap(e,snap(slow),m.reference)
    p.change_passing=((1.,S+8.,S+12.,12.),)
    passing,*_=p._follow_cap(e,snap(slow),m.reference)
    assert braked<20. and passing>=20.


def test_a_target_lane_car_re_identified_under_a_new_id_is_not_a_missing_blocker(highway):
    # 23:02 run 2: a lane-3 car's id churned (297 -> 364 -> 335 -> 371 ...), each old id read as "gone" and held a
    # clear change for 18 m.
    e=ego(highway,lane=4,v=15.)
    p=policy(4);p.recent['old']=(.9,S+20.,S+24.,15.)       # seen 0.1 s ago 20 m ahead
    out=p.tick(e,snap(car(highway,3,S+23.5,'new',v=15.)))    # the same car, new id, where it should be
    assert not any(x.startswith('BLOCKER_UNSEEN') for x in out.diagnostics['blocked_by']),out.diagnostics['blocked_by']
    q=policy(4);q.recent['old']=(.9,S+20.,S+24.,15.)
    out=q.tick(e,snap())                                      # nothing there now: still a missing blocker
    assert 'BLOCKER_UNSEEN:old' in out.diagnostics['blocked_by']


def test_a_car_past_40_m_gets_its_speed_from_the_boxes(highway):
    # 2026-09-30 run 4: boxes reached 40 m; a car 45-58 m ahead had no box speed, read as standing, and its stopping
    # cap held us at 65-73 km/h for 180 m. Speeds come from box edges (not the tracker), so the boxes reach that far.
    e=ego(highway,lane=2,v=20.)          # lane 2: no change to start
    far=car(highway,2,S+55.,'far',v=18.)
    p=policy(2)
    for k in range(10):
        t=1.+.1*k;d=e.v*(t-1.)
        out=p.tick(replace(e,now=t,s=e.s+d,x=e.x+d*math.cos(e.yaw),y=e.y+d*math.sin(e.yaw)),snap(moved(far,t-1.),t=t))
    d=next(x for x in out.diagnostics['following_objects'] if x['id']=='far')
    assert d['motion_source']=='box_edges' and abs(d['lead_speed_mps']-18.)<1.,d


def test_a_rear_edge_that_jumps_with_a_fragment_keeps_its_speed():
    # 2026-09-30 run 6 s1187: a rear fragment joined 4 m behind a 45 km/h lead; the rear edge restarted and the lead
    # read as standing (brake 0.5). The outline jumped, the car did not.
    tr=EdgeTracks(.6);rates=[]
    for k in range(8):
        t=.1*k;s=100.+12.5*t-(4. if k>=5 else 0.)
        rates.append(tr.update(t,{'lo':[(s,True)],'hi':[]})['lo'][0])
    assert all(r is not None and abs(r-12.5)<.5 for r in rates[5:]),rates     # right after the jump too


def test_a_car_at_our_rear_corner_is_passed_before_we_cut_in(highway):
    # 2026-09-30 s1330: a 3->2 pass started with a lane-2 car at our rear corner that sped up with us; the change went
    # on into it. Speed up, get ahead, then cut in (user): no change while it is beside us, one once it is behind.
    e=ego(highway,s=1400.,lane=3,v=18.)
    p=policy(3);out=drive(p,e,[car(highway,2,1400.-2.,'beside',v=18.)],ticks=6)
    assert out.reason!='CHANGE_STARTED' and out.reason in ('SLOT_ACCEL','SLOT_PASS','SLOT_YIELD','SLOT_KEEP'),out.reason
    q=policy(3);out=drive(q,e,[car(highway,2,1400.-12.,'behind',v=18.)],ticks=6)
    assert out.reason=='CHANGE_STARTED',out.reason


def test_each_phase_has_its_own_gap_values(highway):
    # User 2026-09-30: the gap rule per phase, so tuning one situation does not move another.
    from dataclasses import replace
    cfg=config();F=cfg.following
    wide=replace(cfg,following=SimpleNamespace(**{**vars(F),'yield':SimpleNamespace(**{**vars(getattr(F,'yield')),'time_gap_s':1.5})}))
    e=ego(highway,lane=4,v=16.);lead=car(highway,4,S+20.,'lead',v=14.)
    caps={}
    for phase in ('SCOUT','YIELD','PASS'):
        p=HighwayPolicy(highway,wide);p.current_lane=4;p.phase=phase
        caps[phase],*_=p._follow_cap(e,snap(lead),p._reference(e))
    assert caps['YIELD']<caps['SCOUT']


def test_the_phase_follows_the_decision(highway):
    p=policy(4);drive(p,ego(highway,s=1200.,lane=4,v=15.),[],ticks=4)
    assert p.state=='CHANGE' and p.phase=='CHANGE'
    q=policy(2);out=q.tick(ego(highway,s=1400.,lane=2,v=15.),snap())
    assert q.phase=='CRUISE' and out.diagnostics['phase']=='CRUISE'


def test_passing_our_own_lead_is_only_a_stopping_cap(highway):
    # User 2026-09-30: while speeding up to pass (PASS) our own lead only has to stay stoppable, not comfortable.
    e=ego(highway,lane=4,v=16.);lead=car(highway,4,S+25.,'lead',v=14.)
    caps={}
    for phase in ('SCOUT','PASS'):
        p=HighwayPolicy(highway,config());p.current_lane=4;p.phase=phase
        caps[phase],*_=p._follow_cap(e,snap(lead),p._reference(e))
    assert caps['PASS']>caps['SCOUT']


def test_a_hidden_front_reaches_only_as_far_as_a_vehicle_of_its_kind_can_be_long():
    # 2 m boxes from s 0 to 40: a car seen over s 10-14, its front hidden from 14 on (user 2026-10-01: a 4.9 m car
    # read 11 m and a truck 20 m behind their own shadows). The hidden boxes past that length are nobody's.
    from foscar_planning.highway_boxes import run_bodies
    cfg=config();b=cfg.boxes;e=SimpleNamespace(s=0.,v=20.)
    def scene(seen_to):
        boxes=[(float(s),float(s+2),OCCUPIED if 10<=s<seen_to else UNKNOWN if seen_to<=s<30 else FREE,'lidar:car',None)
               for s in range(0,40,2)]
        ext=[(max(10.,s),min(float(seen_to),s+2.)) if b_[2]==OCCUPIED else None for s,b_ in zip(range(0,40,2),boxes)]
        return boxes,lane_runs(boxes,ext)
    boxes,runs=scene(14)
    for top,length in ((1.5,b.car_max_length_m),(2.5,b.large_max_length_m),(math.nan,b.car_max_length_m)):
        bodies=run_bodies(cfg,e,boxes,runs,[15.],lambda k:None,lambda k:top)
        assert [(x.lo,round(x.hi,2),x.kind) for x in bodies]==[(10.,round(10.+length,2),'measured')]
    # Seen longer than any vehicle: cars nose to tail, one body with no room between them.
    boxes,runs=scene(22)
    bodies=run_bodies(cfg,e,boxes,runs,[15.],lambda k:None,lambda k:1.5)
    assert [(x.lo,x.hi) for x in bodies]==[(10.,22.)]
    # A hidden rear keeps every hidden box behind it (a car may follow close behind the one we see).
    boxes=[(float(s),float(s+2),UNKNOWN if 0<s<20 else OCCUPIED if 20<=s<24 else FREE,'lidar:car',None) for s in range(0,40,2)]
    ext=[(float(s),float(s+2)) if b_[2]==OCCUPIED else None for s,b_ in zip(range(0,40,2),boxes)]
    bodies=run_bodies(cfg,e,boxes,lane_runs(boxes,ext),[15.],lambda k:None,lambda k:1.5)
    assert [(x.lo,x.hi) for x in bodies]==[(2.,24.)]


def test_passing_a_car_into_the_box_ahead_of_it_is_done_at_full_speed_up():
    # 2026-10-01 run 4: beside a 40 km/h car the cheapest plan held 62 km/h and the change started ~1.5 s late.
    from foscar_planning.highway_slot_planner import Plan,slot_cost
    P=SimpleNamespace(speed_cost=.25)
    def plan(goal,end_t,passing):return Plan('CHANGE','ACCEL',goal,0.,0.,50.,1.,end_t,'','',0.,(0.,0.),0.,passing)
    v0=17.
    # Passing: the earliest landing wins whatever the speed-up, and on a tie the faster plan.
    assert slot_cost(plan(25.,5.,('car',)),v0,P)<slot_cost(plan(18.,5.5,('car',)),v0,P)
    assert slot_cost(plan(25.,5.,('car',)),v0,P)<slot_cost(plan(20.,5.,('car',)),v0,P)
    # Otherwise a speed change still costs: 0.5 s earlier is not worth 8 m/s more.
    assert slot_cost(plan(18.,5.5,()),v0,P)<slot_cost(plan(25.,5.,()),v0,P)
