import math
from pathlib import Path
import numpy as np
from foscar_planning.highway_policy import HighwayPolicy
from foscar_planning.highway_config import HighwayConfig
from foscar_planning.highway_observation import ObservationSnapshot
from foscar_planning.state import Context
from test_highway_route import highway
from highway_fixtures import calibrated_config

def ctx(route,s,t=1.,lane=4,v=5.):
    x,y=route.at(lane,s)
    return Context(s=s,x=x,y=y,yaw=-math.pi/2,v=v,now=t,mission_name='HIGHWAY')

def empty(t):return ObservationSnapshot(t,True,'OK',{})

def test_default_valid_scan_is_evaluated_without_evidence_file(highway):
    cfg=HighwayConfig.load(Path(__file__).resolve().parents[1]/'config/highway.yaml')
    p=HighwayPolicy(highway,cfg)
    out=p.tick(ctx(highway,1150,lane=5,v=12),empty(1.))
    assert out.v_max>0
    assert 'CALIBRATION' not in out.reason


def test_no_checkpoint_holds_the_second_change(highway,tmp_path):
    # CP11 dropped from the rules (2026-09-30): the 4->3 change is judged before where it was.
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    out=p.tick(ctx(highway,1270,lane=4),empty(1.))
    assert out.diagnostics['cp11_passed'] and out.reason!='CHECKPOINT_PENDING'

def test_normal_empty_scan_evaluates_change_without_patch_handshake(highway,tmp_path):
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    out=p.tick(ctx(highway,1150,lane=5,v=12),empty(1.))
    assert out.reason=='CHANGE_STARTED'


def test_invalid_scan_stops_and_respawn_resets_cp(highway,tmp_path):
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    p.tick(ctx(highway,1285,lane=4),empty(1.))
    p.tick(ctx(highway,1287,t=2.,lane=4),empty(2.))
    out=p.tick(ctx(highway,1290,t=3.5,lane=4),ObservationSnapshot(2.,False,'SCAN_UNKNOWN',{}))
    assert out.v_max==0 and out.reason=='SCAN_UNKNOWN'   # lost for longer than the 1 s hold
    out=p.tick(ctx(highway,1150,t=4.,lane=5),empty(4.))
    assert out.diagnostics['cp11_passed']      # no hold checkpoint any more: nothing to clear

def test_clear_verified_empty_lane_can_start_change_without_restarting_plan(highway,tmp_path):
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    out=p.tick(ctx(highway,1150,t=1.,lane=5,v=15.),empty(1.))
    assert out.reason=='CHANGE_STARTED'
    for t in (1.1,1.2,1.3,1.4,1.5):
        out=p.tick(ctx(highway,1150+(t-1)*15,t=t,lane=5,v=15.),empty(t))
    assert out.state=='CHANGE',out.diagnostics
    assert out.reference is not None

def test_committed_curve_survives_lost_scan_and_settles_on_target(highway,tmp_path):
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    out=p.tick(ctx(highway,1150,t=1.,lane=5,v=15.),empty(1.))
    for t in (1.1,1.2,1.3,1.4,1.5):
        out=p.tick(ctx(highway,1150+(t-1)*15,t=t,lane=5,v=15.),empty(t))
    assert out.state=='CHANGE'
    committed=out.reference
    out=p.tick(ctx(highway,1159,t=1.6,lane=5,v=15.),ObservationSnapshot(1.5,False,'SCAN_UNKNOWN',{}))
    # A brief gap carries on along the committed curve without speeding up (2026-09-28).
    assert out.reason=='SCAN_HOLD' and out.v_max<=15. and out.reference is committed
    m=p.committed
    s=m.end_global_s+.1
    t=1.6+(s-1159)/15.
    # A lane-3 car beside us all along: no CP11 to wait for any more, so a free lane 3 would start 4->3 while settling.
    def beside(s,t):
        from types import SimpleNamespace
        x,y=highway.at(3,s);h=highway.original.heading_at(s);u=np.array([math.cos(h),math.sin(h)]);n=np.array([-u[1],u[0]])
        c0=np.array([x,y])
        snap_=empty(t);snap_.objects['beside']=SimpleNamespace(id='beside',xy=np.array([c0+2*u+.9*n,c0+2*u-.9*n,c0-2*u-.9*n,c0-2*u+.9*n]),
                                                              speed_ready=True,velocity_bounds=(15*u[0],15*u[0],15*u[1],15*u[1]))
        return snap_
    c=ctx(highway,s,t=t,lane=4,v=15.)
    c.yaw=m.reference.heading_at(m.reference.q_at_global(s))
    out=p.tick(c,beside(s,t))
    assert out.state=='SETTLE' and out.reference is committed
    # aligned for a whole sway period (settle_window_s), not one pass through the centre
    t+=.4;s+=.4*15
    c=ctx(highway,s,t=t,lane=4,v=15.)
    c.yaw=m.reference.heading_at(m.reference.q_at_global(s))
    out=p.tick(c,beside(s,t))
    assert out.state=='SETTLE' and out.reason!='CHANGE_COMPLETE'
    t+=p.config.policy.settle_window_s;s+=p.config.policy.settle_window_s*15
    c=ctx(highway,s,t=t,lane=4,v=15.)
    c.yaw=m.reference.heading_at(m.reference.q_at_global(s))
    out=p.tick(c,beside(s,t))
    assert out.reason=='CHANGE_COMPLETE' and p.current_lane==4
    assert out.reference is not committed
    out=p.tick(c,beside(s,t))
    assert out.reason!='CHECKPOINT_PENDING'      # CP11 dropped (2026-09-30): nothing to wait for in lane 4


def test_rejoin_releases_only_when_position_and_heading_match(highway,tmp_path):
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    s=p.rejoin.end_s+1
    x,y=highway.original.at(s)
    c=Context(s=s,x=x,y=y,yaw=highway.original.heading_at(s)+.2,v=5.,now=1.,mission_name='HIGHWAY')
    out=p.tick(c,empty(1.))
    assert out.state=='REJOIN' and not out.reference_release
    c.yaw=highway.original.heading_at(s);c.now=1.1
    out=p.tick(c,empty(1.1))
    assert out.reference_release and out.reason=='REJOIN_COMPLETE'


def test_reusing_one_scan_cannot_keep_a_committed_change_moving(highway,tmp_path):
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    out=p.tick(ctx(highway,1150,lane=5,v=15.),empty(1.))
    for t in (1.01,1.1,1.2,1.4,2.0,2.6):
        out=p.tick(ctx(highway,1150,t=t,lane=5,v=15.),empty(1.))
    # The stale scan is held for 1 s (no speed gain), then the car stops.
    assert out.reason=='SCAN_UNKNOWN' and out.v_max==0.
    assert p.committed is not None  # Keep geometry; stop until fresh observations.

def test_clear_current_lane_does_not_need_remote_patch_confirmation(highway,tmp_path):
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    out=p.tick(ctx(highway,1400,lane=2,v=10.),empty(1.))
    assert out.v_max>0


def test_wide_hull_crossing_reference_is_a_lead_obstacle(highway,tmp_path):
    from types import SimpleNamespace
    from foscar_control.reference import ReferenceLine
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    ss=np.linspace(0,100,201);ref=ReferenceLine('straight',ss,np.c_[ss,np.zeros(len(ss))])
    truck=SimpleNamespace(xy=np.array([[19,-5],[21,-5],[21,5],[19,5]]),speed_ready=True,velocity_bounds=(0,0,0,0))
    c=Context(s=10,x=10,y=0,yaw=0,v=10,now=1.)
    snapshot=empty(1.);snapshot.objects['truck']=truck
    cap,reason,front,_,_=p._follow_cap(c,snapshot,ref)
    assert front<6 and reason in ('LEAD_FOLLOW','LEAD_APPROACH') and cap<4. and p.emergency


def test_follow_reference_ids_do_not_repeat_after_reset(highway,tmp_path):
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    a=p.tick(ctx(highway,1150,lane=5),empty(1.)).reference
    p.reset()
    b=p.tick(ctx(highway,1160,lane=5),empty(1.)).reference
    assert a.id!=b.id

def test_aged_approaching_hull_reduces_follow_gap(highway,tmp_path):
    from types import SimpleNamespace
    from foscar_control.reference import ReferenceLine
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    ss=np.linspace(0,100,201);ref=ReferenceLine('straight',ss,np.c_[ss,np.zeros(len(ss))])
    truck=SimpleNamespace(xy=np.array([[30,-1],[34,-1],[34,1],[30,1]]),speed_ready=True,velocity_bounds=(-20,-20,0,0))
    c=Context(s=10,x=10,y=0,yaw=0,v=10,now=1.2)
    snapshot=empty(1.);snapshot.objects['truck']=truck
    _,_,front,_,_=p._follow_cap(c,snapshot,ref)
    assert front<=12.2+1e-6

def test_rejoin_releases_reference_while_moving(highway,tmp_path):
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    out=None
    for i in range(4):
        s=p.rejoin.end_s+1+i*.5;t=1+i*.1
        x,y=highway.original.at(s)
        c=Context(s=s,x=x,y=y,yaw=highway.original.heading_at(s),v=5.,now=t,mission_name='HIGHWAY')
        out=p.tick(c,empty(t))
        if i:assert out.v_max>0 and out.reference_release

def test_follow_cap_respects_measured_lateral_acceleration(highway,tmp_path):
    from dataclasses import replace
    cfg=calibrated_config(tmp_path)
    cfg=replace(cfg,bounds=replace(cfg.bounds,lat_accel_mps2=.2))
    p=HighwayPolicy(highway,cfg);c=ctx(highway,1400,lane=2,v=5.)
    p.current_lane=2;ref=p._reference(c)
    # The lateral limit applies at the bend, not at the start of its preview.
    peak=int(np.argmax(ref.route.curvature))
    c.s=float(ref.global_s[peak]);c.x,c.y=ref.xy[peak]
    cap,*_=p._follow_cap(c,empty(1.),ref)
    # Rated on the curvature with polyline-joint spikes suppressed (+-10 m average, 2026-09-28).
    assert cap**2*ref.route._curvature(window=10.,dilate=10.)[peak]<=.2+1e-8


def test_late_fast_entry_requests_slowdown_instead_of_forcing_first_change(highway,tmp_path):
    from dataclasses import replace
    cfg=calibrated_config(tmp_path)
    # The 50 m change at 60 km/h needs ~2.5 m/s2 (window-averaged curvature, 2026-09-26);
    # a tighter lateral limit (1.5; 2.0 now fits by lengthening to 56 m) keeps this a slow-down case.
    cfg=replace(cfg,bounds=replace(cfg.bounds,npc_speed_bound_mps=60/3.6,lat_accel_mps2=1.5))
    from types import SimpleNamespace
    cfg=replace(cfg,boxes=SimpleNamespace(**{**vars(cfg.boxes),'change_lat_slow_mps2':1.5,'change_lat_fast_mps2':1.5}))
    p=HighwayPolicy(highway,cfg)
    c=ctx(highway,1212.,lane=5,v=60/3.6)   # near the lane-5 end (the entry-lane limit is s1281 since 2026-09-28)
    first=p.tick(c,empty(c.now))
    c.now+=.1
    out=p.tick(c,empty(c.now))
    # No curve fits at this speed: lane 5 keeps the global path into lane 4 (or eases to a speed that fits).
    assert out.state=='WAIT_GAP' and (out.reason.startswith('TAPER_') or (out.reason=='CHANGE_CURVATURE' and 0<out.v_max<c.v)),out.reason
    assert p.committed is None and out.reference is not None   # near the lane end the follow reference is rebuilt each tick


def test_follow_cap_uses_controller_lateral_limit_without_rewriting_calibration(highway,tmp_path):
    from dataclasses import replace
    from foscar_planning.highway_config import ControlProfile
    cfg=calibrated_config(tmp_path)
    cfg=replace(cfg,control_profile=ControlProfile('test profile',.2,1.5,1.))
    p=HighwayPolicy(highway,cfg);c=ctx(highway,1400,lane=2,v=5.)
    p.current_lane=2;ref=p._reference(c)
    # The lateral limit applies at the bend, not at the start of its preview.
    peak=int(np.argmax(ref.route.curvature))
    c.s=float(ref.global_s[peak]);c.x,c.y=ref.xy[peak]
    cap,*_=p._follow_cap(c,empty(1.),ref)
    # Rated on the curvature with polyline-joint spikes suppressed (+-10 m average, 2026-09-28).
    assert cap**2*ref.route._curvature(window=10.,dilate=10.)[peak]<=.2+1e-8
    assert cfg.bounds.lat_accel_mps2==2.5


def test_lead_stopping_cap_uses_emergency_brake_not_comfort_decel(highway,tmp_path):
    # The controller applies v_max immediately (brake up to 0.5, ~5 m/s2 in sim),
    # so the lead stopping-distance cap is sized by follow_brake, not comfort decel.
    from dataclasses import replace
    from foscar_control.reference import ReferenceLine
    from foscar_planning.highway_config import ControlProfile
    cfg=replace(calibrated_config(tmp_path),control_profile=ControlProfile('test profile',2.5,.4,1.))
    p=HighwayPolicy(highway,cfg)
    ss=np.linspace(0,100,201);ref=ReferenceLine('straight',ss,np.c_[ss,np.zeros(len(ss))])
    c=Context(s=10,x=10,y=0,yaw=0,v=10,now=1.)
    from types import SimpleNamespace
    obstacle=SimpleNamespace(id='stopped',xy=np.array([[90.,-1.],[94.,1.]]),
                             speed_ready=True,velocity_bounds=(0.,0.,0.,0.))
    cap,*_=p._follow_cap(c,ObservationSnapshot(1.,True,'OK',{'stopped':obstacle}),ref)
    fb=cfg.bounds.follow_brake_mps2
    free=80.-cfg.vehicle.rear_axle_to_front_m-cfg.policy.standstill_gap_m-cfg.bounds.position_error_m
    assert 0<cap
    assert cap*cfg.bounds.latency_s+cap**2/(2*fb)<=free+1e-6
    assert cfg.stopping_decel_limit==.4


def test_moving_lead_35m_ahead_does_not_choke_following_speed(highway,tmp_path):
    from foscar_control.reference import ReferenceLine
    from types import SimpleNamespace
    cfg=calibrated_config(tmp_path);p=HighwayPolicy(highway,cfg)
    ss=np.linspace(0,200,401);ref=ReferenceLine('straight',ss,np.c_[ss,np.zeros(len(ss))])
    c=Context(s=10,x=10,y=0,yaw=0,v=12,now=1.)
    lead=SimpleNamespace(id='lead',xy=np.array([[45.,-1.],[49.,1.]]),
                         speed_ready=True,velocity_bounds=(10.,10.,0.,0.))
    cap,*_=p._follow_cap(c,ObservationSnapshot(1.,True,'OK',{'lead':lead}),ref)
    # Old 1.5 m/s2 cap held ~13 m/s behind a 10 m/s lead at 35 m. Now: no braking (above our 12 m/s), on the
    # following.approach_decel profile that reaches the lead's speed at the target gap.
    assert cap>13.5


def test_close_moving_lead_is_not_closed_in_on(highway,tmp_path):
    # CP08 run 0 (2026-09-26): 11 m behind a 10 m/s lead the cap was still ~12 m/s. With the halved
    # following gap and a lead that keeps moving (2026-09-28) the same holds 3.5 m behind it.
    from foscar_control.reference import ReferenceLine
    from types import SimpleNamespace
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    ss=np.linspace(0,200,401);ref=ReferenceLine('straight',ss,np.c_[ss,np.zeros(len(ss))])
    c=Context(s=10,x=10,y=0,yaw=0,v=12,now=1.)
    lead=SimpleNamespace(id='lead',xy=np.array([[13.5+p.config.vehicle.rear_axle_to_front_m,-1.],[17.5+p.config.vehicle.rear_axle_to_front_m,1.]]),
                         speed_ready=True,velocity_bounds=(10.,10.,0.,0.))
    cap,*_=p._follow_cap(c,ObservationSnapshot(1.,True,'OK',{'lead':lead}),ref)
    assert cap<=10.


def test_pose_pause_then_catchup_does_not_erase_passed_checkpoint(highway,tmp_path):
    p=HighwayPolicy(highway,calibrated_config(tmp_path));p.cp11=True
    for s,t in ((1337.85,1.),(1337.85,1.20),(1340.58,1.25)):
        p.tick(ctx(highway,s,t=t,lane=4,v=11.6),empty(t))
    assert p.cp11


def test_missing_lane_changes_cannot_force_global_rejoin(highway,tmp_path):
    p=HighwayPolicy(highway,calibrated_config(tmp_path));p.current_lane=4;p.cp11=True
    c=ctx(highway,1500.,lane=4,v=5.)
    p.follow_reference=p._reference(c)
    c.s=highway.rejoin_s+1.;c.now=2.
    p.previous=None
    out=p.tick(c,empty(c.now))
    # Not a stop (a car stopped at CP12 is the end of the run, 2026-09-29): the global path takes over.
    assert out.reason=='MISSED_REJOIN' and out.v_max>0.
    assert out.reference_release and p.committed is None


def test_single_scan_unknown_cluster_far_ahead_does_not_brake(highway,tmp_path):
    # 2026-09-27: 1-2 scan fragments 23-26 m ahead, treated as stopped, pulled the cap to
    # ~44 km/h (brake 0.5) behind a lead 40 m ahead. Inside emergency range they still count.
    from foscar_control.reference import ReferenceLine
    from types import SimpleNamespace
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    ss=np.linspace(0,200,401);ref=ReferenceLine('straight',ss,np.c_[ss,np.zeros(len(ss))])
    c=Context(s=10,x=10,y=0,yaw=0,v=18.,now=1.)
    def blob(ahead,n):
        x=10+p.config.vehicle.rear_axle_to_front_m+ahead
        return SimpleNamespace(id='blob',xy=np.array([[x,-1.],[x+2,1.]]),speed_ready=False,
                               velocity_bounds=(-np.inf,np.inf,-np.inf,np.inf),sample_count=n)
    far,*_=p._follow_cap(c,ObservationSnapshot(1.,True,'OK',{'blob':blob(35.,1)}),ref)
    assert far>18.
    held,*_=p._follow_cap(c,ObservationSnapshot(1.,True,'OK',{'blob':blob(35.,3)}),ref)
    near,*_=p._follow_cap(c,ObservationSnapshot(1.,True,'OK',{'blob':blob(12.,1)}),ref)
    # 2026-09-30 (user: relative motion): an unmeasured body moves unknown_speed slower than us, not standing -- a
    # flat return 27 m ahead braked a cut-in 0.5. It still caps us by its stopping distance, closer = lower.
    assert held>18. and p.config.boxes.unknown_speed_mps<18. and 18.-p.config.boxes.unknown_speed_mps<near<held


def test_entry_lane_passes_a_slower_target_lane_car_before_changing(highway,tmp_path):
    # 2026-09-27: the car changed from lane 5 at once, 22 m behind a 40 km/h car in lane 4,
    # and was held behind it through CP10-11 although lane 5 was clear to pass it first.
    from types import SimpleNamespace
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    def ahead(s0):
        pos=highway.at(4,s0+6.)                    # beside/just ahead in lane 4 (heading -y here)
        return {'slow':SimpleNamespace(id='slow',xy=pos+np.array([[-1,-2],[1,-2],[1,2],[-1,2]]),speed_ready=True,
                                       velocity_bounds=(0,0,-11.,-11.),sample_count=5)}
    c=ctx(highway,1120.,lane=5,v=16.)
    out=p.tick(c,ObservationSnapshot(c.now,True,'OK',ahead(1120.)))
    assert p.plan is not None and p.plan.rear!='' and p.plan.front=='' and out.v_max>=c.v,(out.reason,p.plan)
    late=HighwayPolicy(highway,calibrated_config(tmp_path))
    c=ctx(highway,1245.,lane=5,v=16.)
    late.tick(c,ObservationSnapshot(c.now,True,'OK',ahead(1245.)))
    assert late.plan is None or late.plan.rear==''     # too late to get ahead of it before lane 5 ends


def test_stopped_at_a_change_deadline_still_changes_once_the_lane_clears(highway,tmp_path):
    # 2026-09-28 08:38 run 6: stopped at the lane-3 deadline (s1566) and never tried again.
    from types import SimpleNamespace
    from foscar_planning.highway_maneuver import heading_at
    s=1566.;x,y=highway.at(3,s);yaw=heading_at(highway,3,s,1.)
    def run(objects):
        p=HighwayPolicy(highway,calibrated_config(tmp_path));p.cp11=True;p.current_lane=3
        reasons=[]
        for k in range(10):
            t=1.+.1*k
            out=p.tick(Context(s=s,x=x,y=y,yaw=yaw,v=0.,now=t,mission_name='HIGHWAY'),ObservationSnapshot(t,True,'OK',objects(t)))
            reasons.append(out.reason)
        return reasons,out
    reasons,_=run(lambda t:{})
    assert 'CHANGE_STARTED' in reasons,reasons
    # A lane-2 car 15 m behind at 40 km/h reaches us within the change: no change until it has gone by; the plan
    # is to land behind it.
    def closing(t):
        c=np.array(highway.at(2,s-15.+11.*(t-1.)));h=heading_at(highway,2,s,1.)
        u=np.array([math.cos(h),math.sin(h)]);n=np.array([-u[1],u[0]])
        xy=np.array([c+2*u+.9*n,c+2*u-.9*n,c-2*u+.9*n,c-2*u-.9*n])
        return {'rear':SimpleNamespace(id='rear',xy=xy,speed_ready=True,velocity_bounds=(11*u[0],11*u[0],11*u[1],11*u[1]))}
    reasons,out=run(closing)
    plan=out.diagnostics['plan']
    assert 'CHANGE_STARTED' not in reasons and (plan is None or plan['front']!=''),(reasons,plan)


def test_the_car_we_pass_dropping_out_of_one_scan_is_not_a_gap(highway,tmp_path):
    # 09:07 run 7: beside a lane-4 car 7 m ahead; it dropped out of one scan, the change latched on that scan and
    # hit it at s1189.
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    p.recent['beside']=(1.,1150.,1154.,15.)          # seen beside us 0.1 s ago, gone now
    out=p.tick(ctx(highway,1150,t=1.1,lane=5,v=15.),empty(1.1))
    assert out.reason!='CHANGE_STARTED' and 'BLOCKER_UNSEEN:beside' in out.diagnostics['blocked_by'],out.reason


def test_the_second_change_is_judged_before_where_cp11_was(highway,tmp_path):
    # CP11 dropped from the rules (2026-09-30): lane 4 before s1286 may change to lane 3 on its first scan.
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    out=p.tick(ctx(highway,1270,lane=4,v=12.),empty(1.))
    assert out.reason=='CHANGE_STARTED'


def test_stopped_at_the_deadline_a_stopped_queue_beside_our_rear_does_not_deadlock_us(highway,tmp_path):
    # 10:57 run 6: stopped at the lane-3 end, the lane-2 queue stopped beside our rear; nobody moved for 20 s.
    from types import SimpleNamespace
    from foscar_planning.highway_maneuver import heading_at
    s=1566.;x,y=highway.at(3,s);yaw=heading_at(highway,3,s,1.)
    c=np.array(highway.at(2,s-1.5));h=heading_at(highway,2,s,1.);u=np.array([math.cos(h),math.sin(h)]);n=np.array([-u[1],u[0]])
    stopped=SimpleNamespace(id='stopped',xy=np.array([c+2*u+.9*n,c+2*u-.9*n,c-2*u+.9*n,c-2*u-.9*n]),speed_ready=True,velocity_bounds=(0.,0.,0.,0.))
    p=HighwayPolicy(highway,calibrated_config(tmp_path));p.cp11=True;p.current_lane=3
    reasons=[]
    for k in range(10):
        t=1.+.1*k
        out=p.tick(Context(s=s,x=x,y=y,yaw=yaw,v=0.,now=t,mission_name='HIGHWAY'),ObservationSnapshot(t,True,'OK',{'stopped':stopped}))
        reasons.append(out.reason)
    assert 'CHANGE_STARTED' in reasons,reasons


def test_competition_configs_build_the_highway_runtime_like_behavior_node(highway):
    # 2026-09-28 11:23: approach_s moved the mission start before CP10 and behavior_node died at startup
    # ("highway mission boundary disagrees with checkpoint geometry"): the map still starts at CP10.
    from foscar_planning.highway_runtime import HighwayRuntime
    from foscar_planning.mission import MissionSupervisor
    from foscar_control.mission_config import load_mission_config
    root=Path(__file__).resolve().parents[1]
    cfg=HighwayConfig.load(root/'config/highway.yaml')
    zone=next(z for z in MissionSupervisor(load_mission_config(root/'config/missions.yaml')).zones if z.name==cfg.mission)
    assert zone.start_s<zone.limit_start_s
    HighwayRuntime(cfg,highway,mission_bounds=(zone.limit_start_s,zone.end_s))


def test_past_the_deadline_a_moving_car_still_changes_into_an_empty_lane(highway,tmp_path):
    # 11:34 run 3: past the lane-3 deadline at 47 km/h with lane 2 empty, it braked 3 s to a stop first.
    from foscar_planning.highway_maneuver import heading_at
    p=HighwayPolicy(highway,calibrated_config(tmp_path));p.cp11=True;p.current_lane=3
    s=1546.;x,y=highway.at(3,s);yaw=heading_at(highway,3,s,1.)
    out=p.tick(Context(s=s,x=x,y=y,yaw=yaw,v=13.,now=1.,mission_name='HIGHWAY'),empty(1.))
    assert out.v_max>=8.,(out.reason,out.v_max)          # 47 km/h: slow to a speed whose curve fits, not to a stop
    q=HighwayPolicy(highway,calibrated_config(tmp_path));q.cp11=True;q.current_lane=3
    out=q.tick(Context(s=s,x=x,y=y,yaw=yaw,v=9.,now=1.,mission_name='HIGHWAY'),empty(1.))
    assert out.reason=='CHANGE_STARTED',(out.reason,out.diagnostics.get('blocked_by'))


def test_change_curve_is_judged_on_the_change_not_the_bend_its_tail_runs_into(highway,tmp_path):
    # 11:34 runs 3/7: the braking-distance tail reached the CP12 joint and no curve 'fitted' at 83-94 km/h.
    from foscar_planning.highway_maneuver import build_maneuver, change_curvature
    cfg=calibrated_config(tmp_path)
    m=build_maneuver(highway,3,2,1467.,90.,23.,cfg)
    assert 23.**2*change_curvature(m)<=2.6 and 23.**2*float(max(m.reference.route.curvature))>5.


def test_a_pass_is_not_dropped_on_one_wobbling_tick(highway,tmp_path):
    # 11:59 batch: the pass flipped for one tick and the car cut in behind the car being passed.
    from types import SimpleNamespace
    p=HighwayPolicy(highway,calibrated_config(tmp_path))
    pos=highway.at(4,1152.);car_xy=np.array(pos)+np.array([[-1,-2],[1,-2],[1,2],[-1,2]])
    even=SimpleNamespace(id='even',xy=car_xy,speed_ready=True,velocity_bounds=(0.,0.,-16.,-16.))
    out=p.tick(ctx(highway,1150,t=1.,lane=5,v=16.),ObservationSnapshot(1.,True,'OK',{'even':even}))
    assert p.plan is not None and p.plan.mode=='ACCEL' and p.plan.rear!='' and out.reason!='CHANGE_STARTED'
    # The car beside drops out of one scan (a wobble): no change into where it still is.
    out=p.tick(ctx(highway,1151,t=1.05,lane=5,v=16.),ObservationSnapshot(1.05,True,'OK',{}))
    assert out.reason!='CHANGE_STARTED' and any(x.startswith('BLOCKER_UNSEEN') for x in out.diagnostics['blocked_by'])


def test_a_restart_never_waits_for_a_checkpoint(highway,tmp_path):
    # 13:13 run 2 stopped at the end of lane 4 waiting for CP11 after a restart; CP11 is gone (2026-09-30).
    for lane in (4,3):
        q=HighwayPolicy(highway,calibrated_config(tmp_path))
        q.tick(ctx(highway,1300,lane=lane,v=15.),empty(1.))
        assert q.cp11


def test_next_change_starts_while_settling_when_the_slot_is_free(highway,tmp_path):
    # Settling after 4->3 with lane 2 empty: the 3->2 change is judged at once and starts without waiting for the
    # settle window (user 2026-09-29).
    p=HighwayPolicy(highway,calibrated_config(tmp_path));p.current_lane=4;p.cp11=True
    t=1.;s=1300.
    out=p.tick(ctx(highway,s,t=t,lane=4,v=20.),empty(t))
    for _ in range(5):
        t+=.1;s+=2.;out=p.tick(ctx(highway,s,t=t,lane=4,v=20.),empty(t))
    assert p.committed is not None and p.committed.target_lane==3,out.reason
    m=p.committed
    s=m.end_global_s+.1;t+=(s-1310.)/20.
    c=ctx(highway,s,t=t,lane=3,v=20.);c.yaw=m.reference.heading_at(m.reference.q_at_global(s))
    out=p.tick(c,empty(t))
    assert out.reason=='CHANGE_STARTED' and p.committed.target_lane==2 and p.current_lane==3,out.reason
