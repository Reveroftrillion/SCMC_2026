"""Deterministic one-lane-at-a-time policy, independent of ROS and NPC truth.

Lane changes are chosen by the slot planner (exp/box-planner, user 2026-09-28): the next lane is cut into boxes,
the bodies there move at their own speeds, and the ego picks the speed profile and start that lands it in a free
box -- passing (speed up into a box ahead of a car), keeping, or yielding (ease off into a box behind it) are one
choice. Lane 5 may also keep the global path, which merges into lane 4 through the taper by itself."""
from dataclasses import dataclass, field, replace
import math
from collections import deque
import hashlib
import sys
from types import SimpleNamespace
import numpy as np
from foscar_control.reference import ReferenceLine
from foscar_control.core import Route, wrap
from .highway_rejoin import build_rejoin
from .highway_maneuver import build_maneuver, heading_at, center_at, motion, change_curvature
from .highway_boxes import lane_boxes, run_bodies, stations
from .highway_box_tracks import EdgeTracks, lane_runs, run_speeds
from .highway_slot_planner import CHANGE_LEAVE, Body, plan_slot
from .brake_shaper import JerkBrake
from .highway_config import change_lat, change_curve_speed


@dataclass
class HighwayDecision:
    state: str
    reason: str
    v_max: float
    stop_s: float=math.nan
    reference: object=None
    diagnostics: dict=field(default_factory=dict)
    reference_release: bool=False


_KAPPA_TABLES={}   # route+config hash -> bend tables (shared by every policy on that route)
KAPPA_FILE=str(__import__("pathlib").Path(__file__).resolve().parents[1]/"config"/"highway_kappa.npz")   # saved bend tables (user 2026-10-09)


def _save_kappa(key,tables):
    """Write the bend tables to KAPPA_FILE for the next start (read back when the key matches)."""
    pairs=list(tables)
    try:
        np.savez(KAPPA_FILE,key=np.array(key),pairs=np.array(pairs,dtype=int),
                 **{f'starts_{i}':tables[p][0] for i,p in enumerate(pairs)},**{f'grid_{i}':tables[p][1] for i,p in enumerate(pairs)})
    except OSError:pass


class HighwayPolicy:
    DROPOUT_HOLD_S=1.0
    def __init__(self,route,config):
        self.route,self.config=route,config
        self.rejoin=build_rejoin(route,config) if route is not None else None
        if route is not None:self._kappa_table()   # at start-up, never inside a tick
        self.reset()

    def reset(self):
        self.follow_details=[]
        # No hold checkpoint (CP11 dropped from the rules, 2026-09-30): 4->3 may follow 5->4 at once.
        self.state='FOLLOW';self.current_lane=None;self.cp11=getattr(self.route,'hold_checkpoint',None) is None;self.previous=None
        self.rejoined=False
        # Highway phase (user 2026-09-30: each situation its own gap values, no cross effects): CRUISE (nothing to change
        # into), SCOUT (a change wanted, no slot chosen), YIELD (landing behind a target-lane car), PASS (speeding up to
        # land ahead of one), CHANGE (on the curve), SETTLE (lead-in before it, settling after it). Set at each decision;
        # the gap rule of the next tick uses it (the curve itself is judged every tick).
        self.phase='CRUISE'
        self.committed=None;self.follow_reference=None;self.candidate=None
        self.settle_since=None
        self.last_scan=-math.inf;self.last_assessment=None
        self.last_cap=None        # last speed cap issued: held through a brief scan gap
        self.brake=JerkBrake(self.config.boxes.brake_jerk_mps3,self.config.boxes.brake_max_mps2);self.emergency=False
        self.box_gate=None;self.edges={}   # lane -> EdgeTracks (box-run edges, no object ids)
        self.plan=None            # current slot plan (display and speed)
        self.recent={}            # target-lane body id -> (last stamp seen near us, lo, hi, speed)
        self.keep_speed=None      # speed latched while keeping (no ratchet down on measured speed)
        self.change_passing=()    # (stamp, lo, hi, speed) of target-lane runs the committed change passes (no leads until we reach their lane)
        self.change_front=None    # (stamp, lo, hi, speed) of the target-lane run the committed change lands behind
        self.change_v0=0.         # speed the change was planned at
        self.taper_committed=False
        self.taper_ride_until=None   # riding lane 5's end into lane 4: the global path is followed up to here
        self.scouted={}           # target lane -> (scans it has been judged on, last scan stamp): SCOUT before any change
        self.commit_scans=0;self.no_pass_until=-math.inf;self.pursuit_view=None;self._pursuit_cache=None;self.aborted=None
        self.pursuit_target=None  # (stamp, lo, hi, speed) of the car a pass is closing in on, followed by position
        self.launching=False      # a change started from a standstill / crawl: full speed-up, the car ahead only a safety cap
        self.box_snapshot=None    # the last scan with box-edge speeds (what the judgement uses; shown in RViz)

    def _gap(self,phase):
        """The following values of a phase (highway.yaml following.<phase>)."""
        return getattr(self.config.following,phase.lower())

    def _phase(self):
        """Phase of the decision just made, from the state and the slot plan."""
        if self.state=='CHANGE':return 'CHANGE'
        if self.state=='SETTLE':return 'SETTLE'
        if self.state=='WAIT_GAP' and self.plan is not None and self.plan.kind=='CHANGE':
            return {'YIELD':'YIELD','ACCEL':'PASS'}.get(self.plan.mode,'SCOUT')
        if self.state=='WAIT_GAP':return 'SCOUT'
        # A change just completed with a lane still to go (FOLLOW for that tick) is scouting the next one.
        r=self.route
        if self.state=='FOLLOW' and r is not None and self.current_lane is not None and self.current_lane!=r.lanes[-1]:return 'SCOUT'
        return 'CRUISE'

    def _reference(self,ctx):
        p=self.config.policy;r=self.route
        if self.committed is not None:return self.committed.reference
        ref=self.follow_reference
        # Always see a bend while it can still be braked for: reusing the window
        # until 60 m remained let 92 km/h meet a 38 km/h bend too late (2026-09-26).
        brake=self.config.stopping_decel_limit
        need=max(p.lookahead_m,self.config.bounds.front_range_m,ctx.v**2/(2*brake)+p.lookahead_m)
        if ref is not None and ref.global_s[0]<=ctx.s and ctx.s+need<ref.global_s[-1]:return ref
        lo=max(r.start_s,ctx.s-p.reference_back_m)
        hi=min(r.original.s[-1],ctx.s+max(p.reference_ahead_m,need+p.reference_ahead_m/2))
        # Lane 5 is the global path, which runs on through the taper into lane 4 by itself.
        # (a middle lane's reference runs on past its end along the global path the lanes converge onto -- cut at the
        # lane end it braked the car to a stop from 122 km/h for want of reference, 2026-10-06 runs)
        if hi-lo<p.path_step_m:raise ValueError('no following reference')
        gs=np.linspace(lo,hi,int(np.ceil((hi-lo)/p.path_step_m))+1)
        # Lane 5 follows the global path: it is lane 5's centre up to s1220 and then keeps right, away from lane 4,
        # as late as the road allows (user's 2026-09-28 14:08 path, up to 0.72 m right of the lane-5 centre).
        ride=self.taper_ride_until
        xy=np.array([np.asarray(r.original.at(s)) if self.current_lane==r.lanes[0] or (ride is not None and s<=ride) else self.rejoin.at(s) if self.rejoin is not None and self.current_lane==r.lanes[-1]
                     and self.rejoin.stations[0]<=s<=self.rejoin.end_s
                     else center_at(r,self.current_lane,s) for s in gs])
        self.follow_reference=ReferenceLine('follow-'+hashlib.sha256(gs.tobytes()+xy.tobytes()).hexdigest(),gs,xy)
        return self.follow_reference
    def _in_lane(self,obj,lane,station):
        r=self.route;table=r.tables[lane];p=self.config.policy
        station=float(np.clip(station,table[0,0],table[-1,0]))
        try:origin=r.at(lane,station);h=heading_at(r,lane,station,p.path_step_m)
        except ValueError:return False
        n=np.array([-math.sin(h),math.cos(h)])
        return abs(float((obj.xy.mean(axis=0)-origin)@n))<=r.width(lane,station)/2

    def _occupancy(self,ctx,snapshot,lanes,path=None):
        """Boxes, occupied runs and run speeds of `lanes` (speed = how fast a run's visible edges move), and the
        snapshot with every hull's speed taken from the run it lies in: no lidar id or tracker speed is used (user
        2026-09-28: ids churned every few scans and each new id restarted its speed). A hull in no run of these lanes
        has no speed."""
        occ={};speed={};low={}
        # Last, a row of boxes along our own path (user 2026-09-28): the lane tables end at CP12 where the path bears
        # right onto the rejoin; the car ahead left the lane-2 boxes there, lost its speed and braked us 58 -> 39 km/h.
        # A body the lanes gave no speed gets it from the run it forms on our path.
        rows=[(lane,None) for lane in dict.fromkeys(lanes)]
        if path is not None:rows.append(('path',np.c_[path.global_s,path.xy]))
        for lane,table in rows:
            ext=[];mem=[]
            boxes=lane_boxes(self.route,self.config,ctx,snapshot,lane if table is None else self.route.lanes[-1],ext,mem,
                             table=table,width=None if table is None else self.route.width(self.route.lanes[-1],min(ctx.s,self.route.rejoin_s)))
            runs=lane_runs(boxes,ext)
            if lane not in self.edges:
                tracker=self.edges[lane]=EdgeTracks(self.config.boxes.edge_window_s)
                # One-way lanes, as on the ring (user 2026-10-04): a backward speed is a wrong start, begun afresh; a
                # new edge's rate is given from its 5th measurement, not its 3rd (a 40 km/h car first read 28-34 km/h
                # and held the change sight cap to 59 km/h, 15:42 run 2). Taking over a nearby edge's state stays on
                # here: an outline jumping behind a lead otherwise read the lead as standing (2026-09-30).
                tracker.NEG_RESET,tracker.READY_N=-1.,5
            tracker=self.edges[lane]
            speeds=run_speeds(tracker,snapshot.stamp,runs,ctx.s,self.config.boxes.unknown_speed_sigma,self.config.boxes.unknown_speed_max_sigma_mps)
            lows=list(tracker.lows)
            self.__dict__.setdefault('_prov',{})[lane]=(list(tracker.lows),list(tracker.highs))
            # A run hidden on its far side from us with no speed of its own moves with the nearer run that hides it
            # (if that one stopped, it would stop first): unknown, cars behind our lead read as standing and braked a
            # change 54 -> 45 km/h until the car behind in the target lane closed in (closed loop, 2026-09-28).
            for k,run in enumerate(runs):
                if speeds[k] is not None:continue
                if run[0]>ctx.s and run[2]:
                    near=[speeds[i] for i in range(k-1,-1,-1) if runs[i][0]>ctx.s and speeds[i] is not None][:1]
                elif run[1]<ctx.s and run[3]:
                    near=[speeds[i] for i in range(k+1,len(runs)) if runs[i][1]<ctx.s and speeds[i] is not None][:1]
                else:near=[]
                if near:speeds[k]=near[0]
                # Our own lane, a run just seen, beyond emergency range: unknown_speed slower than us, not standing
                # (standing, every new car ahead dropped the request 100 -> 43 km/h at once).
                elif lane==lanes[0] and run[0]>ctx.s:
                    car=self.config.vehicle;c=self.config.bounds;p=self.config.policy
                    reach=ctx.v*c.latency_s+ctx.v**2/(2*8.)+p.standstill_gap_m+c.position_error_m
                    if run[0]-ctx.s-car.rear_axle_to_front_m>reach:speeds[k]=max(0.,ctx.v-self.config.boxes.unknown_speed_mps)
            occ[lane]=(boxes,runs,speeds)
            for run,v,lo_v in zip(runs,speeds,lows):
                for k in range(run[4],run[5]+1):
                    for key in mem[k]:
                        if key not in speed or speed[key] is None:speed[key]=v
                        if v is None and lo_v is not None:low[key]=min(low.get(key,math.inf),lo_v)
        ego_u=np.array([math.cos(ctx.yaw),math.sin(ctx.yaw)]);objects={}
        for key,obj in snapshot.objects.items():
            v=speed.get(key)
            # Along the road where the car is, not along our heading (user 2026-10-04): mid-change our heading leans
            # 5-7 deg off the road and a lane-4 car's 0.6 s preview leaned into lane 3 with it -- read as our lead 33 m
            # ahead, lane 3 seemed blocked and the 3->2 change took the gap behind a car it could have passed (15:42
            # run 2). Box-edge speeds are speeds along the lanes.
            u=ego_u
            if v is not None:
                try:
                    st=float(self.route.project_many(obj.xy.mean(axis=0)[None,:])[0])
                    h=self.route.original.heading_at(st);u=np.array([math.cos(h),math.sin(h)])
                except (ValueError,IndexError,AttributeError):u=ego_u
            fields=dict(speed_ready=v is not None,motion_source='box_edges' if v is not None else 'unknown',
                        velocity_bounds=(v*u[0],v*u[0],v*u[1],v*u[1]) if v is not None else (-math.inf,math.inf,-math.inf,math.inf))
            objects[key]=replace(obj,**fields) if hasattr(obj,'__dataclass_fields__') else SimpleNamespace(**{**vars(obj),**fields})
        self.provisional_low=low
        return occ,speed,replace(snapshot,objects=objects)

    def _same_run(self,rec,obj,now,pad=1.5):
        """Is this hull where a recorded run (stamp, lo, hi, speed) should be now?"""
        if rec is None:return False
        t0,lo,hi,v=rec;d=v*max(0.,now-t0);st=stations(self.route,obj.xy)
        return float(st.min())<hi+d+pad and float(st.max())>lo+d-pad

    def _lead(self,ctx,runs,speeds,lows=None):
        """Nearest run ahead in our lane (one whose speed is not known yet is taken as standing, or at the slow side of a
        provisional estimate -- 2026-10-09: L2D 5 runs 76.3 -> 71.6 s, worst gap 0.48 -> 0.81 m)."""
        best=None
        for k,((lo,hi,*_),v) in enumerate(zip(runs,speeds)):
            if lo<=ctx.s:continue
            if v is None and lows is not None and lows[k] is not None:v=max(0.,lows[k])
            if best is None or lo<best.lo:best=Body('lead',lo,hi,0. if v is None else v,self.config.bounds.position_error_m,'lead')
        return best

    def _taper(self):
        """Lane 5's global path into lane 4: (where our body starts entering lane 4, where our centre crosses its
        edge, where it reaches lane 4's centre)."""
        if getattr(self,'_taper_s',None) is None:
            r=self.route;car=self.config.vehicle;p=self.config.policy;lane=r.lanes[1]
            enter=cross=None;done=r.entry_end_s
            for s in np.arange(max(r.start_s,float(r.tables[lane][0,0])),r.entry_end_s,.5):
                d=float(np.linalg.norm(np.asarray(r.original.at(float(s)))-np.asarray(r.at(lane,float(s)))));half=r.width(lane,float(s))/2
                if enter is None and d<half+car.width_m/2+p.margin_m:enter=float(s)
                if cross is None and d<half:cross=float(s)
                if d<p.settle_error_m:done=float(s);break
            self._taper_s=(enter if enter is not None else r.entry_end_s,cross if cross is not None else r.entry_end_s,done)
        return self._taper_s

    TAPER_MEET_M=.1   # the global path counts as on lane 4's centre within this

    def _taper_meets_lane(self):
        """Where the global path, past lane 5's end, comes onto lane 4's centre (it keeps right as late as the road allows)."""
        if getattr(self,'_taper_meet_s',None) is None:
            r=self.route;lane=r.lanes[1];self._taper_meet_s=r.entry_end_s
            for s in np.arange(r.entry_end_s,min(r.entry_end_s+60.,float(r.tables[lane][-1,0])),.5):
                if float(np.linalg.norm(np.asarray(r.original.at(float(s)))-np.asarray(r.at(lane,float(s)))))<self.TAPER_MEET_M:
                    self._taper_meet_s=float(s);break
        return self._taper_meet_s

    def _straight_end(self,lane,target):
        """The last station where `lane` still runs straight beside `target` (headings within 1 deg, the centres no closer
        than 0.1 m less than where the late stretch begins) before they converge: our front stops there."""
        cache=self.__dict__.setdefault('_straight_s',{})
        if (lane,target) not in cache:
            r=self.route;enter=self._converge(lane,target)[0];s0=enter-60.;sep0=r.separation(lane,target,s0);end=enter
            for s in np.arange(s0,enter+1e-9,.5):
                dh=abs(wrap(heading_at(r,lane,float(s),1.)-heading_at(r,target,float(s),1.)))
                if dh>math.radians(1.) or r.separation(lane,target,float(s))<sep0-.1:end=float(s);break
            cache[(lane,target)]=end
        return cache[(lane,target)]

    def _converge(self,lane,target):
        """Where `lane` runs into `target` at the highway's end (user 2026-10-04: no change made, ride our lane as the lanes
        converge and join the global path, yielding to the cars coming from the left): (where our body starts entering
        `target`, where our centre crosses its edge, where the two centres meet), like the lane-5 taper."""
        cache=self.__dict__.setdefault('_converge_s',{})
        if (lane,target) not in cache:
            r=self.route;car=self.config.vehicle;p=self.config.policy
            a=r.tables[lane];end=min(float(a[-1,0]),float(r.tables[target][-1,0]))
            enter=cross=None;done=end
            for s in np.arange(max(float(a[0,0]),float(r.tables[target][0,0])),end,.5):
                d=r.separation(lane,target,float(s));half=r.width(target,float(s))/2
                if enter is None and d<half+car.width_m/2+p.margin_m:enter=float(s)
                if cross is None and d<half:cross=float(s)
                if d<p.settle_error_m:done=float(s);break
            cache[(lane,target)]=(enter if enter is not None else end,cross if cross is not None else end,done)
        return cache[(lane,target)]

    KAPPA_STEP_M=5.                     # start grid of the bend table
    KAPPA_LENGTHS_M=(50.,80.,110.,140.)  # change lengths it is measured for (interpolated between)

    def _kappa_table(self):
        """The lanes' own bending under a change curve, per adjacent lane pair, start (every KAPPA_STEP_M) and length:
        the curve laid there once (as _fit_curve lays it), the quintic's own 5.77 x shift / L^2 taken off its peak
        curvature. Built once per route and config (~0.9 k curves, ~4 s) -- laid on demand it was 18 k curves, 0.5 s per
        wait_gap tick, and the stalled ticks lost the scan and aborted the changes (2026-10-06 full laps)."""
        tables=getattr(self,'_kappa_tables',None)
        if tables is not None:return tables
        r=self.route;b=self.config.boxes
        key=hashlib.sha1(repr((r.lanes,[r.tables[k].tobytes() for k in r.lanes],repr(self.config))).encode()).hexdigest()
        if key not in _KAPPA_TABLES:
            # read from the file saved last time when built for this same route and config (user 2026-10-09: the ~4.6 s
            # build held the start for 7 s with the steering already live); built and saved otherwise
            try:
                with np.load(KAPPA_FILE,allow_pickle=False) as f:
                    if str(f['key'])==key:
                        _KAPPA_TABLES[key]={(int(s),int(d)):(f[f'starts_{i}'],f[f'grid_{i}'])
                                            for i,(s,d) in enumerate(f['pairs'])}
            except (OSError,KeyError,ValueError):pass
        if key not in _KAPPA_TABLES:
            out={}
            for src,dst in zip(r.lanes,r.lanes[1:]):
                lo=max(float(r.tables[src][0,0]),float(r.tables[dst][0,0]));hi=min(float(r.tables[src][-1,0]),float(r.tables[dst][-1,0]))
                starts=np.arange(lo,hi,self.KAPPA_STEP_M);grid=np.zeros((len(starts),len(self.KAPPA_LENGTHS_M)))
                for a,st in enumerate(starts):
                    for c,L in enumerate(self.KAPPA_LENGTHS_M):
                        try:
                            m=build_maneuver(r,src,dst,float(st),L,27.8,self.config)
                            grid[a,c]=max(0.,change_curvature(m)-5.77*b.lateral_ref_m/(L*L))
                        except ValueError:pass       # no room to lay it: the straight-road length stands
                out[(src,dst)]=(starts,grid)
            _KAPPA_TABLES[key]=out
            # saved for the real route only: the tests lay their own routes and wrote over it (2026-10-09)
            if 'pytest' not in sys.modules:_save_kappa(key,out)
        self._kappa_tables=_KAPPA_TABLES[key];return self._kappa_tables

    def _road_kappa(self,lanes,start,length):
        """Bend a change from `start` of `length` lies on, read from _kappa_table (bilinear)."""
        if len(lanes)!=2 or self.route is None:return 0.
        entry=self._kappa_table().get(tuple(lanes))
        if entry is None or not len(entry[0]):return 0.
        starts,grid=entry;Ls=self.KAPPA_LENGTHS_M
        x=min(max((start-starts[0])/self.KAPPA_STEP_M,0.),len(starts)-1.);a=int(x);a1=min(a+1,len(starts)-1);fa=x-a
        L=min(max(length,Ls[0]),Ls[-1]);c=max(k for k in range(len(Ls)) if Ls[k]<=L);c1=min(c+1,len(Ls)-1)
        fc=0. if c1==c else (L-Ls[c])/(Ls[c1]-Ls[c])
        row=grid[a]*(1-fa)+grid[a1]*fa
        return float(row[c]*(1-fc)+row[c1]*fc)

    def _change_length(self,v,start,lanes):
        """Length of a change curve at speed v from `start`: the quintic's own peak 5.77 x shift / L^2 on top of the
        lanes' own bending, within change_lat -- as the curve is then judged when laid (user 2026-10-06: planned at the
        straight-road length, an 86 m curve at 125 km/h needed 94-110 m once laid on the lanes, no longer fitted, and the
        cut-in after a pass came too late). inf when the bend alone uses the whole allowance."""
        b=self.config.boxes;lat=change_lat(b,v);L=v*math.sqrt(5.77*b.lateral_ref_m/lat)
        if start is None or self.route is None:return L
        for _ in range(2):          # the bend measured over the length found, once more
            room=lat-v*v*self._road_kappa(tuple(lanes),start,L)
            if room<=.05*lat:return math.inf
            L=v*math.sqrt(5.77*b.lateral_ref_m/room)
        return L

    def _planner(self,lanes=None):
        c=self.config.bounds;p=self.config.policy;b=self.config.boxes;car=self.config.vehicle
        return SimpleNamespace(accel=b.accel_mps2,brake=b.yield_brake_mps2,front=car.rear_axle_to_front_m,
            back=car.length_m-car.rear_axle_to_front_m,position_error=c.position_error_m,margin=p.margin_m,latency=c.latency_s,
            follow_brake=c.follow_brake_mps2,standstill_gap=p.standstill_gap_m,safe_headway=p.safe_headway_s,
            pass_front_gap=b.pass_front_gap_m,pass_rear_gap=b.pass_rear_min_gap_m,rear_gap=b.rear_min_gap_m,
            closing_error=b.closing_error_mps,closing_error_per_m=b.closing_error_per_m,rear_accel=b.rear_accel_mps2,rear_accel_near=b.rear_accel_near_m,tail_s=b.tail_s,dt=.1,
            start_step=b.start_step_s,max_start=b.max_start_s,lateral_ref=b.lateral_ref_m,max_enter=b.max_enter_s,change_time=p.change_time_s,curve_len=lambda v,start=None:self._change_length(v,start,lanes or ()),lookahead=p.lookahead_m,
            # (committed at once: no clear-hold). Starting slow the curve may begin where we are: the quintic leaves our
            # position and heading unchanged, and a 10 m straight first left a standing start 3 s from the target lane
            # while the queue behind closed in (user 2026-10-08, lane-5 stop line beside a 22 km/h queue).
            lead_in=lambda v:p.lookahead_m*min(1.,max(0.,v)/p.min_merge_speed_mps)+v*(c.latency_s+p.max_scan_age_s),
            min_length=lambda v:8. if v<p.min_merge_speed_mps else p.min_change_length_m,
            speed_cost=b.speed_cost_s,progress_horizon=b.progress_horizon_s)

    def _curve(self,ctx,plan,lane,target,deadline,end=math.inf):
        """(change curve, None) for an immediate plan, lengthened until the tracker's lateral limit holds at the speed
        the change reaches; (None, speed its last curve would fit at) if it no longer fits before the deadline."""
        # Steeper than ordinary bends (user 2026-09-28: gentle 2.5 m/s2 curves were ~90 m at 100 km/h and left few
        # changes at speed).
        p=self.config.policy
        # Laid out for the plan's goal (or our speed if higher). Not for the speed reachable in the change time: those
        # 60+ m curves kept us behind a slow lead in the lane we were leaving until we braked mid-change and the car
        # behind in the target lane closed in (closed loop, 2026-09-28). Mid-change the tracker's own a_lat applies
        # below 60 km/h, so speeding up past the goal is not cut back by this curve there.
        base=max(ctx.v,p.min_change_speed_mps,plan.goal)
        # Laid out for that speed only (user 2026-10-04), no longer for the speed reached accelerating over a clear
        # lane: a 79 km/h change laid out for ~108 km/h ran 80 m (s 1338-1419), sped up through it and then braked
        # 0.9 mid-change for a car found late -- the car turned and the correction swung 7.4 m/s2 (04:00 full lap).
        design=base
        fit=None
        for d in ((design,base) if design>base+1e-6 else (base,)):
            m,fit=self._fit_curve(plan,lane,target,deadline,d,end)
            if m is not None:return m,None
        return None,fit

    def _fit_curve(self,plan,lane,target,deadline,design,end=math.inf):
        p=self.config.policy
        lat=change_lat(self.config.boxes,design)     # 4.5 m/s2 at 40 km/h .. 3.0 at 100 km/h (user 2026-09-29)
        length=plan.length_m;fit=None
        while plan.start_s+CHANGE_LEAVE*length<=deadline and plan.start_s+length<=end:   # as plan_slot judges it
            try:m=build_maneuver(self.route,lane,target,plan.start_s,length,design,self.config)
            except ValueError:break
            k=change_curvature(m);need=design*design*k
            if m.geometry_ok and need<=lat:return m,None
            if m.geometry_ok:fit=math.sqrt(lat/max(k,1e-9))
            length=max(length+(p.path_step_m if m.geometry_ok else 5.),length*math.sqrt(need/lat))
        return None,fit

    def _lane_window(self,ctx,lane):
        """`lane`'s own path from here out to braking distance (as _reference builds it, not stored); None if unmapped."""
        p=self.config.policy;r=self.route
        if r is None:return None
        brake=self.config.stopping_decel_limit
        need=max(p.lookahead_m,self.config.bounds.front_range_m,ctx.v**2/(2*brake)+p.lookahead_m)
        lo=max(r.start_s,ctx.s-p.reference_back_m);hi=min(r.original.s[-1],ctx.s+need+p.lookahead_m)
        # (a middle lane's reference runs on past its end along the global path the lanes converge onto -- cut at the
        # lane end it braked the car to a stop from 122 km/h for want of reference, 2026-10-06 runs)
        if hi-lo<p.path_step_m:return None
        gs=np.linspace(lo,hi,int(np.ceil((hi-lo)/p.path_step_m))+1)
        try:
            xy=np.array([np.asarray(r.original.at(s)) if lane==r.lanes[0] else self.rejoin.at(s) if self.rejoin is not None and lane==r.lanes[-1]
                         and self.rejoin.stations[0]<=s<=self.rejoin.end_s else center_at(r,lane,s) for s in gs])
            return ReferenceLine('lane-'+hashlib.sha256(gs.tobytes()+xy.tobytes()).hexdigest(),gs,xy)
        except ValueError:return None

    def _sight_cap(self,ctx,snapshot,changing_into=None):
        """Speed that stops (bounds.latency_s, then the braking below) behind the slowest car measured on this highway
        at the end of the reliable sight, ahead of our nose. (user 2026-10-04) Past ~32 m only the -1 deg ring meets a
        car, at window height, and cars dropped out of the scans (40-50 m 42 %, 50-60 m 15 % found): a car 47 m ahead in
        the target lane went unseen until 26 m, we were at 108 km/h against its 60 and braked 0.9 mid-change (the turn
        that followed swung 7.4 m/s2, 04:00 full lap). Straight on: boxes.sight_brake_mps2; changing lanes:
        boxes.change_sight_brake_mps2 (hard braking while steering turned the car)."""
        b=self.config.boxes;lat=self.config.bounds.latency_s
        sight=getattr(snapshot,'sight_m',math.inf)
        if not math.isfinite(sight):return math.inf       # no sight given (no scan geometry): no cap from it
        reach=min(sight,b.reliable_sight_m)
        room=max(0.,reach-self.config.vehicle.rear_axle_to_front_m-b.sight_margin_m)
        # the gentle braking only on the curve itself (CHANGE): settling we are in the new lane already, and held to it
        # the next change could not plan to pass a 60 km/h car 35 m ahead at 80 km/h (16:08 runs)
        # changing_into: the cap the curve into that lane will be driven at (judging a change before it starts)
        changing=changing_into is not None or (self.committed is not None and self.state=='CHANGE')
        a=b.change_sight_brake_mps2 if changing else b.sight_brake_mps2
        closing=a*(-lat+math.sqrt(lat*lat+2.*room/a))
        # the lane the hidden car would be in: ours, or the one we are changing into
        lane=(changing_into if changing_into is not None else
              getattr(self.committed,'target_lane',None) if self.committed is not None else self.current_lane)
        return self._slowest_seen(snapshot,lane,ctx)+closing

    def _slowest_seen(self,snapshot,lane=None,ctx=None):
        """The slowest cars measured in `lane` on this highway -- p10 over cars, each car once at the median of its own
        speeds, and only cars that tell what may hide ahead: seen 10 scans 5 m or more ahead of us, really moving (its
        outline's centre moved at least half the speed read over 1 s or more -- a roadside thing read 9 km/h stood
        still; outlines of cars ahead are 0.7-1.7 m, no size tells them apart) and holding their speed (p10-p90
        within 3 m/s). Per-scan values let a car's first slow readings hold the
        cap down (a 40 km/h car read 28 km/h when first seen, 15:42 run 2); counted too were a car spawned behind us
        accelerating 13 -> 43 km/h and a 1.2 m roadside thing read 9 km/h (15:53 run). The whole highway's while that
        lane has none, boxes.hidden_vehicle_mps while the highway has fewer than 2. Taken over the whole highway,
        the 40 km/h cars of lanes 4-5 held lane 2 (60 km/h cars only) to 93 km/h (15:15 lap)."""
        cars=self.__dict__.setdefault('_seen_cars',{})
        stamp=getattr(snapshot,'stamp',None)
        if stamp is not None and stamp!=getattr(self,'_seen_stamp',None):
            self._seen_stamp=stamp
            for key,obj in snapshot.objects.items():
                if not getattr(obj,'speed_ready',False):continue
                v=float(np.hypot(*motion(obj)))
                if v<=2.:continue
                try:st=float(self.route.project_many(obj.xy.mean(axis=0)[None,:])[0])
                except (ValueError,IndexError):continue
                # Only scans 5 m ahead or more are kept (speed, place, lane): a lane-2 car we passed and left behind
                # pushed its readings ahead out of the window and was then counted a lane-5 car (16:06 runs).
                if ctx is not None and st-ctx.s<5.:continue
                rec=cars.setdefault(key,{'v':deque(maxlen=50),'at':deque(maxlen=50),'lanes':{}})
                rec['v'].append(v);rec['at'].append((float(stamp),*map(float,obj.xy.mean(axis=0))))
                for l in self.route.lanes:
                    if self._in_lane(obj,l,st):
                        rec['lanes'][l]=rec['lanes'].get(l,0)+1;break
        steady=[]
        for r in cars.values():
            v=np.asarray(r['v'])
            if len(v)<10 or np.percentile(v,90)-np.percentile(v,10)>3.:continue
            (t0,x0,y0),(t1,x1,y1)=r['at'][0],r['at'][-1]
            if t1-t0<1. or math.hypot(x1-x0,y1-y0)/(t1-t0)<.5*float(np.median(v)):continue
            steady.append((float(np.median(v)),max(r['lanes'],key=r['lanes'].get) if r['lanes'] else None))
        own=[v for v,l in steady if l==lane]
        # one car of our lane is enough (user 2026-10-04): only one lane-2 car steady enough, the 40 km/h cars of lanes 4-5
        # held the empty lane 2 to 94 km/h
        if own:return float(np.percentile(np.asarray(own),10))
        if len(steady)>=2:return float(np.percentile(np.asarray([v for v,_ in steady]),10))
        return self.config.boxes.hidden_vehicle_mps

    def _bend_cap(self,ctx,ref,q0):
        """Speed cap from the bends of `ref` ahead of q0: each bend at its lateral limit, braked for over its distance."""
        c=self.config.bounds;cap=math.inf
        ahead=ref.q>q0
        distance=np.r_[0.,ref.q[ahead]-q0]
        # Rate bends over +-10 m like the tracker does on the highway: polyline joints of the global path
        # read as sharp bends and cut the cap to ~30 km/h at 100 km/h on switching to the fallback route
        # (brake 0.5 to 73 km/h, s1217-1250, 2026-09-28 10:42 run 7), while the tracker itself drove it at 94.
        # (an isolated spike -- over twice its +-10 m average -- is rated at that average, i.e. its heading
        # change / 20 m; arcs, including their ends, keep their curvature.)
        # Rated as the tracker rates highway stations: heading change over +-10 m (core.Controller, uncapped zones).
        # The spike rule on the +-4 m curvature missed the CP12 lane-link joint (5.6 deg in 2.5 m): dilated into a
        # 16 m plateau it read as an R 85 m arc and braked 96 -> 70 km/h on an empty road (2026-09-29, no NPCs).
        key=(ref.id,ref.signature)
        cache=self.__dict__.setdefault('_wide_cache',{})
        if key not in cache:
            if len(cache)>8:cache.clear()
            cache[key]=ref.route._curvature(window=10.,dilate=10.)
        wide=cache[key]
        curvature=np.r_[np.interp(q0,ref.q,wide),wide[ahead]]
        bends=curvature>1e-9
        if np.any(bends):
            # A committed change curve may be driven up to the tracker's own a_lat: it was laid out at
            # 2.5 m/s2 for the speed at its start, and that limit then held a slow change to ~45 km/h with
            # the target lane empty ahead (user 2026-09-28).
            lat=np.full(int(bends.sum()),self.config.lateral_accel_limit)
            gs=np.interp(np.r_[q0,ref.q[ahead]],ref.q,ref.global_s) if hasattr(ref,'global_s') else None
            # From CP12 (the rejoin) on, bends are rated at rejoin_lat (user 2026-09-28: CP12 -> CP13 should keep its
            # speed but for the ruled drop to 60 km/h; only there, not the whole highway).
            if gs is not None and self.route is not None:
                lat=np.where(gs[bends]>=self.route.rejoin_s,max(self.config.lateral_accel_limit,self.config.boxes.rejoin_lat_mps2),lat)
            # The change curve itself (start..end) is driven at change_lat, and only it: past its end the road's own
            # bends are back at the ordinary limit (user 2026-09-28: steep only while turning in, then back down).
            m=self.committed
            if getattr(m,'start_global_s',None) is not None and gs is not None:
                on=((gs>=m.start_global_s-1.)&(gs<=m.end_global_s+1.))[bends]
            curve_speed=np.sqrt(lat/curvature[bends])
            if getattr(m,'start_global_s',None) is not None and gs is not None:
                # speed-dependent: 4.5 m/s2 at 40 km/h .. 3.0 at 100 km/h (user 2026-09-29)
                # (never below the ordinary / rejoin rating of the same point)
                change=np.maximum(change_curve_speed(self.config.boxes,curvature[bends]),curve_speed)
                curve_speed=np.where(on,change,curve_speed)
            # braked for late and hard (user 2026-10-04: comfort is not scored; hold the top speed as long as possible):
            # at stopping_decel_limit (1.5 m/s2) bends 100-200 m ahead held the empty lane 2 near 100 km/h
            brake=self.config.boxes.bend_brake_mps2
            delay=brake*c.latency_s
            approach=-delay+np.sqrt(delay**2+curve_speed**2+2*brake*distance[bends])
            cap=float(np.min(np.maximum(curve_speed,approach)))
        return cap

    def _follow_cap(self,ctx,snapshot,ref):
        c=self.config.bounds;p=self.config.policy;car=self.config.vehicle
        self.follow_object=None;self.follow_details=[]
        # The lane-change relaxations (a lead in the lane we leave ignored when we won't reach it, a stoppable gap only,
        # no gentle approach) hold only while the car is actually changing lanes -- on the change curve -- not while it
        # deliberates, nor on the straight lead-in before the curve or while settling after it (user 2026-09-29).
        m_=self.committed
        # ...and only until half the curve (user 2026-09-30; our body is crossing the lane line there): to the
        # curve's end the car was already in the new lane but still held the change's short gap and ran up close to
        # the car ahead (user 2026-09-30: "이미 settle 중인데 아직 추월과정이라고 생각하고 과하게 붙네").
        in_change=(getattr(m_,'source_lane',None) is not None and m_.source_lane!=m_.target_lane
                   and m_.start_global_s-p.lookahead_m/2<=ctx.s<=m_.start_global_s+.5*(m_.end_global_s-m_.start_global_s))
        # Deliberating a change keeps the safe distance to our own lead (user 2026-09-28: close all the while it
        # considers a change is too dangerous; only from the moment the change starts may it close in).
        cap=math.inf;reason='CRUISE';front=rear=math.inf;side=False
        q0=ref.project(ctx.x,ctx.y)[0];h=ref.heading_at(q0);u=np.array([math.cos(h),math.sin(h)])
        # Each future bend constrains today's speed by its remaining braking
        # distance, rather than imposing the slowest point on the whole window.
        # A committed change is no exception: holding its worst-point speed for
        # the whole change pinned the car near its entry speed (~45 km/h,
        # 2026-09-27). The change curve is built for the speed it will reach.
        cap=self._bend_cap(ctx,ref,q0)
        # While changing or settling, `ref` is the change curve and ends shortly after it: the bends of the target lane
        # beyond it were not seen and the cap read 200 km/h (the car reached 121 km/h in SETTLE, 2026-09-29 no-NPC run).
        # Also rate the target lane's own path out to braking distance.
        target=getattr(self.committed,'target_lane',None)
        if target is not None:
            ext=self._lane_window(ctx,target)
            if ext is not None:cap=min(cap,self._bend_cap(ctx,ext,ext.project(ctx.x,ctx.y)[0]))
        if math.isfinite(cap):reason='LATERAL_ACCELERATION'
        # Highest highway speed (user 2026-10-04: 130 km/h): the planner's goals and the speed driven alike.
        if self.config.boxes.max_speed_mps<cap:cap,reason=self.config.boxes.max_speed_mps,'MAX_SPEED'
        # Sight: a car slower than any highway NPC (boxes.hidden_vehicle_mps) may sit just past where the lidar stops
        # meeting car bodies (pitch-up while accelerating: ~44 m, 00:26 lap collision). Keep able to stop behind it.
        # The slot planner plans goals up to the bends only (user 2026-10-04): with the sight cap in, a 3 m/s2 cap
        # left from the 4->3 curve held its goals to 81 km/h and no pass past a 60 km/h lane-2 car 38 m ahead was
        # planned (16:19 run, s1272). The sight cap still holds the speed actually driven.
        self.bend_cap=cap
        # Only on a change curve (user 2026-10-04): driving a lane the car brakes hard for a car found late (the
        # 130 km/h max speed bounds it); the hard braking that turned the car was mid-change.
        if self.committed is not None and self.state=='CHANGE':
            sight=self._sight_cap(ctx,snapshot)
            if sight<cap:cap,reason=sight,'SIGHT'
        # An empty highway has no blanket cruise cap derived from sensor ROI.
        # Actual objects below and route curvature still constrain speed.
        for key,obj in snapshot.objects.items():
            age=max(0.,ctx.now-snapshot.stamp)
            velocity=motion(obj)
            hull=obj.xy+velocity*age
            # Following speed is governed by objects ahead/alongside now.
            # A rear car's forward preview must not become a front emergency
            # stop. Rear collision admission belongs to the merge assessor.
            actual_upper=max(ref.project(*point)[0] for point in hull)
            if actual_upper<q0-(car.length_m-car.rear_axle_to_front_m):continue
            if obj.speed_ready:
                # Include imminent lateral entry without waiting for the first
                # point to cross our corridor. Keep the measured hull shape.
                hull=np.vstack((hull,hull+velocity*.6))
            projected=[ref.project(*point) for point in hull]
            # Signed span includes bodies whose edges cross the path even when
            # every observed vertex is outside the driving corridor.
            lateral=[]
            for point,row in zip(hull,projected):
                h=ref.heading_at(row[0]);normal=np.array([-math.sin(h),math.cos(h)])
                lateral.append(float(np.dot(point-ref.at_q(row[0]),normal)))
            half=car.width_m/2+c.position_error_m  # Following corridor, not lane-change gap margin.
            lower=min(row[0] for row in projected);upper=max(row[0] for row in projected)
            if self.committed is not None and obj.speed_ready and self._same_run(self.change_front,obj,ctx.now):
                # Changing in behind this car: the plan opens the gap by holding its speed while the car pulls ahead.
                # Until it is the following gap ahead, no faster than planned -- following its speed instead, the change
                # sped up with it and cut in 0.2 m off its bumper (closed loop, 2026-09-28). Not a following knob:
                # it is the lane-change plan being carried out.
                F=self.config.following;lead_v=max(0.,float(motion(obj)@u))
                if lower-q0-car.rear_axle_to_front_m<F.standstill_m+self._gap('CHANGE').time_gap_s*lead_v and self.change_v0<cap:
                    cap=self.change_v0;reason='CHANGE_BEHIND';self.follow_object=key
            if min(lateral)>half or max(lateral)<-half:
                # Mid-change, a car ahead in the target lane is the lead we land behind even where the
                # change curve has not reached its lane yet: a curve laid out for 108 km/h kept a 39 km/h
                # lane-4 car 35 m ahead outside the corridor while we sped up 62 -> 88 km/h, and braked
                # only 11 m short of it (hit, 2026-09-28 08:38 run 5).
                m=self.committed
                # ...except one the change was planned to pass (speed up past it into the box ahead of it): braking for
                # it held every 5->4 pass behind the car it was passing, 68 -> 17 km/h over 96 m (20:30 batch run 8).
                # Once the curve reaches its lane and it is still ahead, the corridor check above brakes for it.
                if (m is None or not obj.speed_ready or m.source_lane==m.target_lane or self.route is None
                        or lower-q0-car.rear_axle_to_front_m<=0. or any(self._same_run(x,obj,ctx.now) for x in self.change_passing)):continue
                st=float(self.route.project_many(obj.xy.mean(axis=0)[None,:])[0])
                if not self._in_lane(obj,m.target_lane,st):continue
            if upper<q0-(car.length_m-car.rear_axle_to_front_m):rear=min(rear,q0-upper);continue
            gap=lower-q0-car.rear_axle_to_front_m
            now_mid=float(np.mean([row[0] for row in projected[:len(obj.xy)]]))   # current body, not the 0.6 s preview
            if gap<=0 and now_mid<q0+(car.rear_axle_to_front_m-(car.length_m-car.rear_axle_to_front_m))/2:
                # Overlapping our body from behind (a car closing on our rear corner mid-change): braking
                # cannot help and stopped us in front of it (hit from behind at s1315, 2026-09-28 08:10).
                rear=min(rear,0.);continue
            if gap<=0:
                cap=0.;reason='CURRENT_OCCUPIED';self.follow_object=key;side=True
                self.follow_details.append(dict(id=key,gap_m=gap,mode='BODY_OCCUPIED',hard_cap_mps=0.))
                continue
            # On the highway only NPC cars appear (no static obstacles). A cluster seen for
            # under 0.2 s with no motion yet is not braked for as if stopped unless it is
            # already inside emergency stopping range: flickering fragments 23-26 m ahead
            # pulled the cap to ~44 km/h with brake 0.5 behind a 40 m gap (2026-09-27).
            if not obj.speed_ready and getattr(obj,'sample_count',3)<3:
                emergency=ctx.v*c.latency_s+ctx.v**2/(2*8.)+self.config.following.standstill_m
                if gap>emergency:continue
            m=self.committed if in_change else None
            if m is not None and obj.speed_ready and m.source_lane!=m.target_lane and self.route is not None:
                # A car ahead in the lane we are leaving only matters if we reach it before the change
                # is over. Judged at its current spot, the half-shifted curve still overlapped a 35 km/h
                # car 30 m ahead and braked 58 -> 53 km/h mid-change (2026-09-27, s1286).
                a,b,cc,d=obj.velocity_bounds;lead_now=float(np.array([(a+b)/2,(cc+d)/2])@u)
                station=float(self.route.project_many(obj.xy.mean(axis=0)[None,:])[0])
                # ...unless it is already at or over our front: a lane-3 car squeezing in from the right as the
                # lanes closed was skipped while overlapping our front corner; at full throttle we slowed 51 -> 39
                # km/h and were pushed 1.5 m off the path (REFERENCE_DISCONNECTED, 10:37 run 2, s1506-1526).
                if gap>self.config.following.standstill_m and self._in_lane(obj,m.source_lane,station) and not self._in_lane(obj,m.target_lane,station):
                    closing=ctx.v-lead_now
                    if closing<=.1 or station+lead_now*gap/closing>=m.end_global_s:continue
            front=min(front,gap)
            # ---- Speed behind this body: one rule (user 2026-09-30: the interlocked caps could no longer be tuned).
            F=self.config.following
            # On the curve the change values; otherwise those of the phase decided last tick (a committed change
            # before its curve, or settling after it: SETTLE).
            G=self._gap('CHANGE' if in_change else 'SETTLE' if self.committed is not None else self.phase)
            if obj.speed_ready:
                lead=float(motion(obj)@u)                                   # its speed along our path
                vx0,vx1,vy0,vy1=obj.velocity_bounds
                low=min(vx*u[0]+vy*u[1] for vx in (vx0,vx1) for vy in (vy0,vy1))   # slowest it may be going
            else:
                # Not measured yet: moving a little slower than us (boxes.unknown_speed_mps), not standing (user
                # 2026-09-30: judge by relative motion; only NPC cars on the highway). As standing, a flat return 27 m
                # ahead seen for 0.5 s braked a 3->2 cut-in 0.5 at 74 km/h. A real car is measured within ~0.3 s.
                lead=low=max(0.,ctx.v-self.config.boxes.unknown_speed_mps)
                # ...unless its edges, measured a few times, already say it is slower: the slowest it may plausibly be
                # going (box_tracks low_sigma). At 106-110 km/h a 48 km/h lead read "unknown, 88 km/h" for 0.3-0.5 s,
                # its cap stayed 131 km/h and we braked 19 m behind it -- too late at brake 0.5 (2026-10-01).
                est=getattr(self,'provisional_low',{}).get(key)
                if est is not None:lead=low=max(0.,min(low,est))
            desired=F.standstill_m+G.time_gap_s*max(0.,lead)
            error=gap-desired
            # Close in so the closing speed is shed at approach_decel exactly at the desired gap (relative motion:
            # the lead keeps going). sqrt(lead^2+2a*err) -- the standing-car formula -- took 100 m to close 33 -> 12 m
            # behind a 40 km/h lead and never reached 6.7 m (2026-09-30). Inside the gap, open it at gain_hz.
            # Near the target gap the square root is steep (0.1 m of gap noise = 2 km/h of cap, the brake pulsed while
            # following at 40 km/h): there both sides use gain_hz, the same slope through the target.
            follow=(max(0.,lead)+min(math.sqrt(2*G.approach_decel_mps2*error),G.gain_hz*error) if error>=0
                    else max(0.,lead+G.gain_hz*error))
            # Speed not measured yet (a car hidden behind another): only the safety cap below. Approached as a
            # standing car, one 40 m ahead in the target lane held a planned speed-up at 12.5 m/s and the car
            # behind ran into our side (closed loop).
            if not obj.speed_ready:follow=math.inf
            # PASS: speeding up to land ahead of a target-lane car, our own lead only has to stay stoppable (user
            # 2026-09-30); the comfortable approach held the speed-up the pass needed. The slot planner already
            # requires the lead to stay brakeable until we have left the lane.
            if G is self._gap('PASS'):follow=math.inf
            # Launched from a standstill into a moving lane: the car behind closes in until we reach the lane's speed, so
            # speed up at full pedal and keep only the safety cap to the car ahead (user 2026-10-08: the comfortable gap cut
            # the request to 13-15 km/h twice in the first 2 s of a stop-line launch, 25 km/h after 4 s, 19:49 runs 3/4).
            if self.launching and in_change and obj.speed_ready and ctx.v<lead+.5:follow=math.inf
            # Safety: able to stop standstill_m behind it if it brakes at follow_brake after our latency.
            fb=c.follow_brake_mps2;usable=max(0.,gap-F.standstill_m)
            stop=max(0.,low if low>.5 else 0.)+max(0.,-fb*c.latency_s+math.sqrt((fb*c.latency_s)**2+2*fb*usable))
            # Inside the stop gap the safety cap also reopens it (it held the lead's speed 1.2 m behind it, and the next
            # change started from there and overlapped it, 2026-09-30 run 171839).
            if gap<F.standstill_m:stop=max(0.,stop-G.gain_hz*(F.standstill_m-gap))
            # Mid-change, a car still in the lane we leave (and not in the one we enter) only needs the safety cap.
            leaving=False
            if in_change and self.route is not None:
                st=float(self.route.project_many(obj.xy.mean(axis=0)[None,:])[0])
                leaving=self._in_lane(obj,self.committed.source_lane,st) and not self._in_lane(obj,self.committed.target_lane,st)
            speed=stop if leaving else min(follow,stop)
            self.follow_details.append(dict(id=key,gap_m=gap,desired_gap_m=desired,lead_speed_mps=lead if obj.speed_ready else None,
                closing_mps=ctx.v-lead if obj.speed_ready else None,speed_ready=obj.speed_ready,
                motion_source=getattr(obj,'motion_source','measured' if obj.speed_ready else 'unknown'),
                comfort_cap_mps=follow,hard_cap_mps=stop,
                mode='MEASURED_FOLLOW' if obj.speed_ready else 'UNKNOWN_STATIONARY_BOUND'))
            if speed<cap:cap=speed;reason='LEAD_FOLLOW' if follow>=stop or leaving else 'LEAD_APPROACH';self.follow_object=key
        # A body inside emergency range (latency + 8 m/s2 + standstill): brake at once, no jerk shaping.
        reach=ctx.v*c.latency_s+ctx.v**2/(2*8.)+self.config.following.standstill_m
        self.emergency=any(d['gap_m']<reach for d in self.follow_details)
        return cap,reason,front,rear,side
    # A slow or standing start into a moving lane (the lane-5 stop line beside a queue): the car behind may brake for us
    # once we are wholly in its lane ahead of it, at most as hard as the same NPCs braked on the ring (2.5 m/s2, measured).
    NPC_YIELD_BRAKE=2.5
    COMMIT_SCANS=2       # a pass is committed once it fits at our present speed on this many scans in a row
    PURSUE_STEP_S=.5     # pursuing goes on only while landing behind (no pass) still fits after this much more of it
    ABORT_HOLD_S=1.5     # a pass given up is not taken up again for this long

    def _pursue(self,ctx,snapshot,plan,bodies,lead,change,taper,vmax,pass_first,judge,lane,target_lane,diag):
        """A pass (landing ahead of a target-lane car now ahead of our rear): speed up to it first, commit only when it fits
        at the speed we have, give it up while landing behind still fits (user 2026-10-08: find an easy car, close in at
        full speed-up; the moment it looks impossible, give up and slow down in behind it; close enough that it plainly
        works, pass). Returns (plan to drive, still pursuing). Judged once per scan."""
        P=self._planner((lane,target_lane));stamp=snapshot.stamp;c=self._pursuit_cache
        if c is None or c[0]!=stamp or c[1]!=target_lane:
            # Commit: at no more than our present speed (and the curve's own sight cap), the same pass or a bigger one fits
            # starting now -- the boxes as they are, no speed-up counted on (14:35: a pass planned at a 130 km/h goal was
            # held to 93 km/h on the curve and squeezed against the truck it never cleared).
            curve_cap=self._sight_cap(ctx,snapshot,changing_into=target_lane)
            vc=max(.5,min(vmax,ctx.v,curve_cap))
            commit=plan_slot(ctx.s,max(0.,ctx.v),bodies,lead,P,change=change,taper=None,vmax=vc,pass_first=pass_first,**judge)
            ok=(commit is not None and commit.kind=='CHANGE' and commit.start_t==0. and set(plan.passing)<=set(commit.passing))
            # ...and, landing with nobody ahead, the place we land in seen whole: from the pass gap ahead of the last car
            # passed to our length and the front gap beyond, within the reliable sight (user 2026-10-08: a 3->2 pass at 114
            # km/h committed ahead of the 3rd car of a 20 km/h lane-2 queue whose 4th and 5th cars were past the sight; it
            # braked to 30 km/h on the curve and hit the 5th -- keep closing in until the queue's first car is passed too).
            if ok and not commit.front:
                byid={b.id:b for b in bodies}
                past=[byid[k].hi for k in commit.passing if k in byid]
                if past:
                    b=self.config.boxes;car=self.config.vehicle
                    land=max(past)+b.pass_rear_min_gap_m+car.length_m+b.pass_front_gap_m
                    seen=ctx.s+car.rear_axle_to_front_m+min(getattr(snapshot,'sight_m',math.inf),b.reliable_sight_m)
                    ok=land<=seen
            self.commit_scans=self.commit_scans+1 if ok else 0
            # Plan B: landing behind (or between, nobody now ahead of our rear passed), now and after PURSUE_STEP_S more of
            # the pursuit; once it would be gone by then, it is taken now.
            fb=plan_slot(ctx.s,max(0.,ctx.v),bodies,lead,P,change=change,taper=taper,vmax=vmax,pass_first=pass_first,no_pass=True,**judge)
            dt=self.PURSUE_STEP_S;a=P.accel if plan.goal>ctx.v else 0.
            v1=min(plan.goal,ctx.v+a*dt);s1=ctx.s+(ctx.v+v1)/2*dt
            moved=[replace(b,lo=b.lo+b.v*dt,hi=b.hi+b.v*dt) for b in bodies]
            lead1=None if lead is None else replace(lead,lo=lead.lo+lead.v*dt,hi=lead.hi+lead.v*dt)
            fb1=plan_slot(s1,v1,moved,lead1,P,change=change,taper=taper,vmax=vmax,pass_first=pass_first,no_pass=True,**judge) if fb is not None else None
            # ...given up only if by then the pass is gone too: a car beside at our speed leaves no way in behind it after
            # half a second of speeding up, and the pass past it is the plan (test_a_car_beside_at_our_speed...).
            pass1=None
            if fb is not None and fb1 is None:
                pass1=plan_slot(s1,v1,moved,lead1,P,change=change,taper=None,vmax=vmax,pass_first=pass_first,**judge)
            lost=fb is not None and fb1 is None and (pass1 is None or not set(plan.passing)<=set(pass1.passing))
            self._pursuit_cache=c=(stamp,target_lane,commit,ok,fb,fb1,curve_cap,lost)
        _,_,commit,ok,fb,fb1,curve_cap,lost=c
        diag['pursuit']=dict(commit_scans=self.commit_scans,need=self.COMMIT_SCANS,curve_cap_kmh=round(curve_cap*3.6,1) if math.isfinite(curve_cap) else None,
                             fallback=None if fb is None else (fb.kind,fb.mode,fb.rear,fb.front),fallback_after_step=fb1 is not None)
        byid={b.id:b for b in bodies}
        passed=[byid[k] for k in plan.passing if k in byid]
        if passed:
            far=max(passed,key=lambda b:b.lo);self.pursuit_target=(snapshot.stamp,far.lo,far.hi,far.v)
        if ok and self.commit_scans>=self.COMMIT_SCANS:
            self.pursuit_target=None
            self.pursuit_view=self._slot_view(ctx,snapshot,'COMMIT',commit,fb,bodies)
            diag['pursuit']['state']='COMMIT'
            return commit,False
        if lost:
            # the last moment plan B still fits and the pass would not: give the pass up
            self.no_pass_until=stamp+self.ABORT_HOLD_S;self.commit_scans=0;self.pursuit_target=None
            self.aborted=(stamp,self._region(ctx,plan,bodies))
            self.pursuit_view=self._slot_view(ctx,snapshot,'ABORT',fb,None,bodies)
            diag['pursuit']['state']='ABORT'
            return fb,False
        self.pursuit_view=self._slot_view(ctx,snapshot,'PURSUE',plan,fb,bodies)
        diag['pursuit']['state']='PURSUE'
        return plan,True

    def _target_now(self,bodies,stamp):
        """The body the pass in progress is closing in on, found where its recorded speed puts it (box ids change)."""
        rec=self.pursuit_target
        if rec is None:return None
        t0,lo,hi,v=rec;d=v*max(0.,stamp-t0)
        over=[(min(b.hi,hi+d+2.)-max(b.lo,lo+d-2.),b) for b in bodies if b.kind in ('measured','unmeasured','provisional')]
        over=[o for o in over if o[0]>0.]
        return max(over,key=lambda o:o[0])[1].id if over else None

    def _region(self,ctx,plan,bodies):
        """Target-lane stations of a plan's box now: from the nose of the car it lands ahead of to the tail of the one it
        lands behind (open ends shown 15 m long)."""
        if plan is None or plan.kind!='CHANGE':return None
        byid={b.id:b for b in bodies}
        rear=byid.get(plan.rear);front=byid.get(plan.front)
        # an open end reaches as far as we cover in 3 s (an unmeasured car there still matters to landing in it)
        reach=max(15.,3.*ctx.v)
        lo=rear.hi if rear is not None else (front.lo-reach if front is not None else ctx.s-5.)
        hi=front.lo if front is not None else lo+reach
        return (float(lo),float(max(hi,lo+1.)))

    def _slot_view(self,ctx,snapshot,state,plan,alt,bodies):
        """What RViz shows of the slot judgement (highway_scene draws it on the target lane)."""
        if state is None or plan is None:return None
        byid={b.id:b for b in bodies}
        passed=[byid[k] for k in plan.passing if k in byid]
        target=max(passed,key=lambda b:b.lo) if passed else None
        kind=('PASS x%d'%len(passed) if passed and not plan.front else 'BETWEEN' if passed else
              'BEHIND' if plan.front else 'OPEN') if plan.kind=='CHANGE' else 'TAPER'
        ab=self.aborted if self.aborted is not None and snapshot.stamp-self.aborted[0]<=1. else None
        return dict(state=state,kind=kind,slot=self._region(ctx,plan,bodies),alt=self._region(ctx,alt,bodies),
                    target=None if target is None else (float(target.lo),float(target.hi)),
                    aborted=None if ab is None else ab[1],commit_scans=self.commit_scans,need=self.COMMIT_SCANS)

    def tick(self,ctx,snapshot):
        self.box_gate=None;self.last_assessment=None  # Current-tick display evidence only.
        p=self.config.policy;c=self.config.bounds;r=self.route;car=self.config.vehicle
        continuous=True
        if self.previous is not None:
            old_s,old_t,old_v,old_xy=self.previous;dt=ctx.now-old_t
            continuous=dt>=0 and abs(ctx.s-old_s)<=max(old_v,ctx.v)*dt+p.projection_jump_margin_m
            if not continuous:
                # A pose correction must not erase a checkpoint already behind us. At 69 km/h a
                # 2.9 m GPS step reset the policy, cleared CP11 and the car waited for it at the
                # end of lane 3 until it stopped (2026-09-27). A respawn before CP11 still clears it.
                passed=r.hold_checkpoint is None or (self.cp11 and ctx.s>=r.hold_s)
                self.reset();self.cp11=passed
        if not all(math.isfinite(v) for v in (ctx.s,ctx.x,ctx.y,ctx.yaw,ctx.v,ctx.now)):
            return HighwayDecision('FOLLOW','LOCALIZATION_INVALID',0.)
        if r.hold_checkpoint is None:self.cp11=True
        elif self.previous is not None and continuous and not self.cp11:
            self.cp11=r.checkpoint_passed(r.hold_checkpoint,self.previous[3],(ctx.x,ctx.y))
        # Repeated stale poses must not move the continuity time anchor.
        # Otherwise the next fresh pose appears to jump in one control tick.
        if self.previous is None or abs(ctx.s-self.previous[0])>1e-6 or np.linalg.norm(np.array([ctx.x,ctx.y])-self.previous[3])>1e-6:
            self.previous=(ctx.s,ctx.now,ctx.v,(ctx.x,ctx.y))
        if self.current_lane is None:
            if ctx.s>=r.rejoin_s:self.current_lane=r.lanes[-1]
            else:self.current_lane=r.match(ctx.x,ctx.y,ctx.s)[0]
            # Starting (or restarting) past CP11 in lane 4 or 5: CP11 lies on lane 4's centre where lane 5 merges
            # into it, so we went through it. A restart right after CP11 (a 0.5 s data gap, 13:13 run 2) dropped
            # the record and the car stopped at the end of lane 4 still waiting for CP11.
            if not self.cp11 and r.hold_s is not None and ctx.s>=r.hold_s+1. and self.current_lane in r.lanes[:2]:self.cp11=True
        # The lane-5 global path runs through the taper into lane 4; past a lane's end the map says where we are.
        if self.committed is None and ctx.s<r.rejoin_s and ctx.s>r.tables[self.current_lane][-1,0]:
            # Ridden to lane 5's end, the global path reaches lane 4's centre only ~12 m later (it keeps right as late as
            # the road allows): switched to lane 4's centre at s1276 the reference jumped 0.4-0.5 m left and the 4->3 change
            # started on the same tick, the car swung across lane 4's centre and back (13:31, 13:55 runs; user 2026-10-08).
            # Lane 4 then follows the global path until it meets lane 4's centre, and no change starts before that.
            if self.current_lane==r.lanes[0]:
                done=self._taper_meets_lane()
                self.taper_ride_until=done if ctx.s<done else None
            self.current_lane=r.match(ctx.x,ctx.y,ctx.s)[0]
            self.follow_reference=None;self.taper_committed=False;self.plan=None
        if self.taper_ride_until is not None and (ctx.s>=self.taper_ride_until or self.current_lane!=r.lanes[1]):
            self.taper_ride_until=None;self.follow_reference=None
        lane=self.current_lane;idx=r.lanes.index(lane)
        target_lane=r.lanes[min(idx+1,len(r.lanes)-1)]
        # A change only needs the two lane centres a car width apart (TuningPoint, 2026-09-27).
        deadline=r.change_deadline(lane,car.width_m)
        diag=dict(state=self.state,reason='',current_link=r.link_at(lane,min(ctx.s,r.rejoin_s)),
            target_link=r.link_at(target_lane,min(ctx.s,r.rejoin_s)),cp11_passed=self.cp11,
            deadline_global_s=deadline,scan_age_s=ctx.now-snapshot.stamp,
            observed_count=len(snapshot.objects),removed_count=snapshot.removed_count,
            front_gap_m=None,rear_gap_m=None,rear_closing_mps=None,
            reference_id='',global_s=ctx.s,reference_q=None,
            speed_cap_mps=0.,speed_reason='',blocked_by=[])
        def result(reason,cap,reference=None,release=False,stop_s=math.nan):
            # Jerk-limited braking (user 2026-09-28: follow a gentle curve into the brake, but reach the braking
            # finally wanted): below our speed, the request starts from it and falls with a deceleration that builds
            # up and eases off at brake_jerk, up to brake_max, until it meets the cap. Bodies inside emergency range
            # and true stops are not shaped.
            cap=self.brake.apply(cap,ctx.v,ctx.now,exempt=self.emergency or reason in
                                 ('SCAN_UNKNOWN','MAP_COVERAGE','CHANGE_DEADLINE','LOCALIZATION_INVALID'))
            self.last_cap=cap
            self.phase=self._phase();diag['phase']=self.phase
            plan=self.plan
            diag['plan']=None if plan is None else dict(kind=plan.kind,mode=plan.mode,goal_kmh=round(plan.goal*3.6,1),
                start_t=round(plan.start_t,2),end_t=round(plan.end_t,2),rear=plan.rear,front=plan.front,
                slot=[round(v,1) if math.isfinite(v) else None for v in plan.slot])
            diag.update(state=self.state,reason=reason,speed_cap_mps=cap,reference_id=reference.id if reference is not None else '',
                        gap_strategy=plan.mode if plan is not None and self.committed is None else '')
            if reference is not None:diag['reference_q']=reference.project(ctx.x,ctx.y)[0]
            return HighwayDecision(self.state,reason,cap,stop_s,reference,diag,release)
        if lane!=r.lanes[-1] and ctx.s>=r.rejoin_s and self.committed is None:
            _,error,heading=r.original.project(ctx.x,ctx.y,ctx.s)
            # Ridden in along a converging lane we are on the global path already (the lanes meet it here): within half a
            # car width and a few degrees it is followed on, not released to a braking recovery (user 2026-10-06: the
            # 0.3 m settle limit gave MISSED_REJOIN at 103 km/h and braked 0.5 at s1585).
            if error>car.width_m/2 or abs(wrap(heading-ctx.yaw))>3*p.settle_heading_rad:
                # Not a stop (user 2026-09-29: a car stopped at CP12 in lane 3 is the end of the run): hand the car back
                # to the global path; the tracker steers onto it (RECOVER, slowed to the route's lateral limit).
                self.follow_reference=None
                return result('MISSED_REJOIN',self.last_cap if self.last_cap is not None else ctx.v,None,release=True)
            # At a converged map endpoint several lane centers coincide. An
            # already aligned car needs no lateral rejoin; don't deadlock it.
            self.current_lane=lane=r.lanes[-1];idx=len(r.lanes)-1;target_lane=lane
            self.follow_reference=None
        try:ref=self._reference(ctx)
        except ValueError:return result('MAP_COVERAGE',0.)
        if not snapshot.valid or not 0<=ctx.now-snapshot.stamp<=p.max_scan_age_s:
            diag['blocked_by']=['SCAN_UNKNOWN']
            # A brief scan gap keeps driving as it was (user 2026-09-28): same reference (a committed change
            # carries on), no speed gain, no new change. Only a longer loss stops.
            if self.last_cap is not None and ctx.now-self.last_scan<=self.DROPOUT_HOLD_S:
                # standing (waiting to launch at a stop line) stays put: held at our speed the car crept 7 m past the
                # lane-5 stop line into the narrowing lane between the lane-4 queue (19:49 run 1)
                return result('SCAN_HOLD',0. if ctx.v<.5 else min(self.last_cap,max(0.,ctx.v)),ref)
            return result('SCAN_UNKNOWN',0.,ref)
        self.last_scan=ctx.now
        m0=self.committed
        # Every lane (ours first): a body in a lane without box speeds read as standing and braked us at CP11 with no
        # car ahead in our lane (a lane-5 car or roadside return in the corridor, 23:48 runs 1-2).
        mine=(m0.source_lane,m0.target_lane) if m0 is not None else (lane,target_lane)
        occ,obj_speed,snapshot=self._occupancy(ctx,snapshot,mine+tuple(l for l in r.lanes if l not in mine),path=ref)
        self.box_snapshot=snapshot
        cap,speed_reason,front,rear,side=self._follow_cap(ctx,snapshot,ref)
        self.front_gap_now=front
        diag['follow_object']=self.follow_object
        diag['following_objects']=self.follow_details
        diag.update(front_gap_m=front if math.isfinite(front) else None,rear_gap_m=rear if math.isfinite(rear) else None,speed_reason=speed_reason)
        def wait_gap(lane,idx,target_lane,deadline,trial=False):
            """Waiting in `lane` for the change into `target_lane`: plan the slot, start it when it can start now.
            trial: judge only (while settling): return the start of a change, else None and issue nothing."""
            def done(*a,**k):return None if trial else result(*a,**k)
            self.state='WAIT_GAP'
            entry=idx==0;taper=self._taper() if entry else None
            if self.taper_committed:
                # Our body is entering lane 4 along the global path (or the lane we converge into at the highway's end):
                # no stopping in the middle of the taper.
                return done('TAPER_MERGE',cap,ref)
            # Stop line when no slot is found: where too little lane is left to change in (lane 5: before its path
            # starts entering lane 4). Past it, still moving, the lanes converge but run on to the lane end: keep judging
            # short changes that end there, and stop before it (the car braked to a stop with lane 2 empty, 11:34 run 3,
            # and stood at s1566 for the rest of a run, 08:38 run 6).
            room=p.lookahead_m+p.min_change_length_m
            end=min(float(r.tables[lane][-1,0]),float(r.tables[target_lane][-1,0]))
            cover=r.coverage_end(lane,target_lane)     # past lane 5's end when it merges into lane 4
            stop_at=taper[0]-car.rear_axle_to_front_m-p.margin_m if entry else deadline-car.rear_axle_to_front_m-p.margin_m-room
            late=not entry and ctx.s>=stop_at-p.lookahead_m
            if late:
                # Past it the lanes converge into one (s1530-1583): short changes are still judged, and riding our own lane
                # into the target lane like the lane-5 taper is one more choice, against the target-lane cars (user
                # 2026-10-04). Stopping, only before our body would enter the target lane: stopped short of the lane end the
                # car stood there for good (20:14 run, s1581).
                taper=self._converge(lane,target_lane)
                stop_at=taper[0]-car.rear_axle_to_front_m-p.margin_m
                if target_lane==r.lanes[-1]:
                    # The last change (3->2), as lane 5 into lane 4 (user 2026-10-08): the converging path itself is the
                    # curve, ridden into a gap judged against the lane-2 cars (TAPER); with no gap, stop where lane 3 still
                    # runs straight beside lane 2 -- not on the converging stretch, where the boxes no longer look down the
                    # lanes and a lane-2 car closing from behind squeezed into our side (20:11 run 2, s1557-1566) -- and
                    # launch from there.
                    stop_at=self._straight_end(lane,target_lane)-car.rear_axle_to_front_m-p.margin_m
            brake=self.config.stopping_decel_limit;delay=brake*c.latency_s
            stopcap=max(0.,-delay+math.sqrt(delay**2+2*brake*max(0.,stop_at-ctx.s)))
            diag['waiting_speed_cap_mps']=stopcap
            # CP11 sits on lane 4's centre (3.5 m from lane 3's): no 4->3 judgement at all before it (user
            # 2026-09-28: preparing it early tailgated the lane-4 lead before CP11). The first tick past CP11
            # judges on its own fresh scan.
            if idx>=1 and not self.cp11:return done('CHECKPOINT_PENDING',min(cap,stopcap),ref)
            if entry:
                # Lane 5: a curve may start while the lane centres are still cut_in_min_sep_m apart (no tiny curve
                # deep in the taper) and must end a car length before lane 5 does.
                change=dict(deadline=end-car.length_m,latest_start=r.change_deadline(lane,p.cut_in_min_sep_m),end=cover,
                            shift=(lambda s,a=lane,b=target_lane:r.separation(a,b,s)) if r.merges_into(lane,target_lane) else None)
            else:change=dict(deadline=end-1. if late else deadline,end=r.original.s[-1] if target_lane==r.lanes[-1] else cover)
            boxes,runs,speeds=occ[target_lane]
            top=lambda key:getattr(snapshot.objects.get(key),'height',math.nan)
            bodies=run_bodies(self.config,ctx,boxes,runs,speeds,obj_speed.get,top,prov=getattr(self,'_prov',{}).get(target_lane))
            rear_info=dict(runs=[dict(lo=round(r_[0],1),hi=round(r_[1],1),v=None if v is None else round(v,2)) for r_,v in zip(runs,speeds)])
            lead=self._lead(ctx,*occ[lane][1:],lows=getattr(self,'_prov',{}).get(lane,(None,))[0])
            vmax=min(self.bend_cap,self.config.boxes.max_speed_mps)
            # Standing (a lane end, a stopped queue beside our rear) there is no getting ahead of a car before the change:
            # the change itself passes it, as before (a pass-first rule deadlocked there, 10:57 run 6 case).
            pass_first=ctx.v>=.5
            # At the highway's end a change is still plan A; riding our lane into the target lane only when none fits.
            # The car ahead in our lane must stay brakeable until our body starts entering the target lane, braking as
            # hard as the car does (brake_max), not until it is half across at the follower's gentle 4 m/s2 (user
            # 2026-10-06: a pass past a 60 km/h lane-2 car with a 50 km/h lane-3 car 88 m ahead was tight but fine).
            judge=dict(lead_until='enter',lead_brake=self.config.boxes.brake_max_mps2)
            # standing or slow (below min_merge_speed): the car behind brakes for us once we are in its lane ahead of it
            # (user 2026-10-08: stopped at the lane-5 stop line beside a 22 km/h queue with 12.6-15.6 m gaps, no gap fitted
            # with the car behind assumed never to brake, and the car stood there for good, 19:00 runs 0 and 3)
            if ctx.v<p.min_merge_speed_mps:judge['rear_yield']=self.NPC_YIELD_BRAKE
            # standing: one judgement per scan (the launch candidates are the costly part; 19:49 run 1 fell behind the scans)
            key=(snapshot.stamp,lane,target_lane,late) if ctx.v<.5 else None
            hit=getattr(self,'_standing_plan',None)
            if key is not None and hit is not None and hit[0]==key:plan=hit[1]
            else:
                plan=plan_slot(ctx.s,max(0.,ctx.v),bodies,lead,self._planner((lane,target_lane)),change=change,taper=None if late else taper,vmax=vmax,pass_first=pass_first,**judge)
                self._standing_plan=(key,plan) if key is not None else None
            if plan is None and late:plan=plan_slot(ctx.s,max(0.,ctx.v),bodies,lead,self._planner((lane,target_lane)),change=change,taper=taper,vmax=vmax,pass_first=pass_first,**judge)
            # Just gave a pass up: no pass for a while (no flip-flop back into the pursuit just abandoned).
            if plan is not None and plan.passing and snapshot.stamp<self.no_pass_until:
                plan=plan_slot(ctx.s,max(0.,ctx.v),bodies,lead,self._planner((lane,target_lane)),change=change,taper=None if late else taper,
                               vmax=vmax,pass_first=pass_first,no_pass=True,**judge)
            # Keep to a pass once closing in on it (user 2026-10-08: find an easy car and close in; give up only when it looks
            # impossible): the best slot flipped pass <-> yield every few ticks beside a 40 km/h lane-4 car, the car neither
            # sped up nor tucked in and was squeezed beside it as lane 5 narrowed (18:10 run 8, s1110-1247).
            tgt=self._target_now(bodies,snapshot.stamp)
            if tgt is not None and not late and snapshot.stamp>=self.no_pass_until:
                kept=plan_slot(ctx.s,max(0.,ctx.v),bodies,lead,self._planner((lane,target_lane)),change=change,taper=None,
                               vmax=vmax,pass_first=pass_first,require_pass=(tgt,),**judge)
                if kept is not None:plan=kept
                elif plan is not None and tgt in plan.passing:pass
                else:
                    # the pass is gone: plan B now
                    self.no_pass_until=snapshot.stamp+self.ABORT_HOLD_S;self.commit_scans=0;self.pursuit_target=None
                    self.aborted=(snapshot.stamp,self._region(ctx,plan,bodies) if plan is not None else None)
                    plan=plan_slot(ctx.s,max(0.,ctx.v),bodies,lead,self._planner((lane,target_lane)),change=change,taper=None if late else taper,
                                   vmax=vmax,pass_first=pass_first,no_pass=True,**judge)
                    diag['pursuit']=dict(state='ABORT',reason='PASS_GONE')
            elif tgt is None:self.pursuit_target=None
            pursuing=False
            if plan is not None and plan.kind=='CHANGE' and plan.passing and not late:
                plan,pursuing=self._pursue(ctx,snapshot,plan,bodies,lead,change,taper,vmax,pass_first,judge,lane,target_lane,diag)
            else:
                self.commit_scans=0
                self.pursuit_view=self._slot_view(ctx,snapshot,'TARGET' if plan is not None else None,plan,None,bodies)
            self.plan=plan
            # Two steps, not one (user 2026-10-06): when no pass fits as judged now but one would if the car ahead in our
            # lane only had to be clear when the change starts, speed up first and look again -- the cut-in is judged
            # anew every tick as above and taken once it fits; speeding up keeps the way back (slow down, in behind).
            if lead is not None and not late and (plan is None or (plan.kind=='CHANGE' and not plan.passing)):
                probe=plan_slot(ctx.s,max(0.,ctx.v),bodies,lead,self._planner((lane,target_lane)),change=change,vmax=vmax,pass_first=pass_first,
                                lead_until='start',lead_brake=self.config.boxes.brake_max_mps2)
                if probe is not None and probe.kind=='CHANGE' and probe.passing and probe.start_t>0.:
                    self.plan=probe;self.keep_speed=None
                    diag['probe']=dict(goal_kmh=round(probe.goal*3.6,1),start_t=probe.start_t,passing=list(probe.passing))
                    return done('SLOT_PROBE',min(cap,probe.goal),ref)
            # The last change (into lane 2) cannot be taken back: while our own lane is clear ahead, keep driving it at
            # our speed (the speed-up of an empty lane) whatever lane 2 shows; the change is judged once a car turns up
            # ahead of us in lane 3 -- pass, speed up and look, or in behind (user 2026-10-05/06: 'nothing ahead in lane 3,
            # go on accelerating; a car there, then decide') -- or once only a curve's length and a change time are left
            # before the deadline. (A 107 km/h change into a lane 2 that looked empty met a 60 km/h car 12 m ahead, 04:22.)
            P_=self._planner((lane,target_lane));v_=max(ctx.v,p.min_change_speed_mps)
            room=P_.lead_in(v_)+max(P_.min_length(v_),P_.curve_len(v_,ctx.s+P_.lead_in(v_)))+v_*p.change_time_s   # a change from here, at our speed
            if (plan is not None and plan.kind=='CHANGE' and target_lane==r.lanes[-1] and lead is None and not late
                    and ctx.s+room<change['deadline']):
                diag['blocked_by']=['LANE_3_CLEAR']
                return done('SLOT_DEFER',cap,ref)
            now=plan is not None and plan.kind=='CHANGE' and plan.start_t==0.
            # A body beside us whose speed is not measured yet may be faster than assumed: plan the speed, but start the
            # change only once it is measured (~0.3 s for a new track).
            back=car.length_m-car.rear_axle_to_front_m;pad=p.standstill_gap_m+p.margin_m
            # So is one behind within the rear range: its assumed speed is relative to ours, and stopped at a lane end a
            # car closing at 11 m/s read as 5 m/s on its first scan.
            pending=[b.id for b in bodies if b.kind in ('unmeasured','provisional') and b.hi>ctx.s-back-self.config.bounds.rear_range_m and b.lo<ctx.s+car.rear_axle_to_front_m+pad]
            # A target-lane car near us that dropped out of this scan is not a gap: 09:07 run 7 changed on the one scan
            # that missed the car beside it and hit it (s1189).
            near=p.lookahead_m+ctx.v*p.change_time_s
            for b in bodies:
                if b.kind in ('measured','unmeasured','provisional') and b.hi>ctx.s-back-near and b.lo<ctx.s+near:self.recent[f'{snapshot.stamp:.2f}{b.id}']=(snapshot.stamp,b.lo,b.hi,b.v)
            self.recent={k:v for k,v in self.recent.items() if snapshot.stamp-v[0]<=p.history_s}
            # A run we saw is missing only if nothing stands where it should be now (by position: no ids).
            live=[b for b in bodies if b.kind in ('measured','unmeasured','provisional')]
            def reidentified(rec):
                t0,lo,hi,v=rec;d=v*(snapshot.stamp-t0)
                return any(b.lo<hi+d+2. and b.hi>lo+d-2. for b in live)
            # ...where it matters to this plan: in its box or beside our body (a fragment flickering 45 m behind held a
            # planned tuck-in behind a lane-4 car, 18:10 run 8); a car gone from the box we would land in still holds it.
            span=self._region(ctx,plan,bodies) if plan is not None else None
            def matters(rec):
                t0,lo,hi,v=rec;d=v*(snapshot.stamp-t0)
                beside=hi+d>ctx.s-back-pad and lo+d<ctx.s+car.rear_axle_to_front_m+pad
                return beside or span is None or (lo+d<span[1] and hi+d>span[0])
            unseen=[k for k,rec in self.recent.items() if not reidentified(rec) and matters(rec)]
            m=fit=None
            # Judged while settling, a change starts only with every body of that lane measured: one still at its assumed
            # speed is not yet known enough to pass or tuck in behind. ...and so while waiting in our lane too, and with our
            # lead's speed measured (user 2026-10-08: a car just picked up is no reason to cut in at once -- follow our lane,
            # watch it, then see first whether we can speed up past it without closing on the car ahead of us, and only
            # then tuck in behind; an unmeasured lead counts as standing, which rules every pass out). Near the deadline
            # (late) the bodies near us still wait (pending), the far ones no longer do.
            lw=getattr(self,'_prov',{}).get(lane,(None,))[0]
            lead_unmeasured=lead is not None and any(v is None and (lw is None or lw[k] is None) for k,((lo_,*_),v) in enumerate(zip(*occ[lane][1:])) if lo_==lead.lo)
            # ...those the plan deals with (passed, landed behind or ahead of, inside its box): a fragment flickering 40 m
            # behind held a planned tuck-in behind a lane-4 car for 30 m (18:10 run 8).
            rel=set() if plan is None else {plan.rear,plan.front,*plan.passing}
            box=self._region(ctx,plan,bodies) if plan is not None else None
            # ...and within the reliable sight: past it an edge seldom gets a speed at all, and the planner already takes such
            # a body slower than us (unknown_speed); waiting on a lane-2 car 46-50 m ahead held a committed pass ahead of a
            # truck for 50 m until the truck was gone by (20:11 run 3, s1368-1420; user 2026-10-08).
            sight=ctx.s+car.rear_axle_to_front_m+self.config.boxes.reliable_sight_m
            unknown=(trial or not late) and (any(b.kind=='unmeasured' and b.lo<sight and (b.id in rel or (box is not None and b.lo<box[1] and b.hi>box[0])) for b in bodies)
                                             or lead_unmeasured)
            riding=self.taper_ride_until is not None and ctx.s<self.taper_ride_until
            # SCOUT first (user 2026-10-08): judged on the first highway tick, a 5->4 change started at 60 km/h into a lane
            # that looked empty; the 40 km/h car 44 m ahead in it was picked up one scan later and we tucked in behind it,
            # where with it seen (2nd run) the car sped up and passed it. A target lane is watched, driving as planned, for
            # as many scans as a car first seen there needs to get its speed (the edge filter's READY_N) before a change
            # starts. Settling judges the next lane, so the scans count from there; near the deadline no waiting.
            n,last=self.scouted.get(target_lane,(0,None))
            if last!=snapshot.stamp:n,last=n+1,snapshot.stamp;self.scouted[target_lane]=(n,last)
            scouting=not late and n<self.edges[target_lane].READY_N
            if now and not pending and not unseen and not unknown and not riding and not scouting and not pursuing:
                m,fit=self._curve(ctx,plan,lane,target_lane,change['deadline'],change['end'])
                if m is not None and m.end_global_s-m.start_global_s>plan.length_m+1e-6:
                    # The curve had to be longer than planned for the lateral limit (road curvature): plan that length
                    # again, starting now at whichever speed clears it.
                    again=plan_slot(ctx.s,max(0.,ctx.v),bodies,lead,self._planner((lane,target_lane)),change=change,vmax=vmax,pass_first=pass_first,
                                    only=(None,m.end_global_s-m.start_global_s),**judge)
                    if again is None:m=None
                    else:plan=self.plan=again
            self.candidate=m
            self.last_assessment=SimpleNamespace(permitted=m is not None)
            diag['blocked_by']=(['NO_SLOT'] if plan is None else ['MOTION_PENDING:'+k for k in pending]+['BLOCKER_UNSEEN:'+k for k in unseen]
                                +(['UNMEASURED'] if unknown and not pending else [])+(['TAPER_SETTLE'] if riding else [])+(['SCOUTING'] if scouting else [])+(['PURSUE'] if pursuing else []))
            self.box_gate=SimpleNamespace(lane=target_lane,permitted=plan is not None,reasons=tuple(diag['blocked_by']),boxes=tuple(boxes),
                region=(plan.slot if plan is not None else (ctx.s,ctx.s)),rear=rear_info)
            diag['bodies']=[dict(id=b.id,lo=round(b.lo,1),hi=round(b.hi,1),v=round(b.v,2),kind=b.kind) for b in bodies]
            diag['lead']=None if lead is None else dict(id=lead.id,lo=round(lead.lo,1),v=round(lead.v,2))
            if m is not None:
                self.committed=m;self.state='CHANGE';self.settle_since=None;self.keep_speed=None;self.recent={}
                rec={b.id:(snapshot.stamp,b.lo,b.hi,b.v) for b in bodies}
                self.change_passing=tuple(rec[k] for k in plan.passing if k in rec);self.change_front=rec.get(plan.front)
                self.change_v0=max(ctx.v,plan.goal);self.candidate=None
                self.launching=ctx.v<p.min_merge_speed_mps
                diag.update(admission='SLOT_CUT_IN' if plan.rear else 'SLOT_'+plan.mode,speed_reason='CHANGE_PLANNED_SPEED',follow_object=None)
                return result('CHANGE_STARTED',cap,m.reference)  # (trial: this starts it)
            if fit is not None:
                # No curve fits at this speed before the deadline: ease toward one that does (~1 m/s per tick at most:
                # jumping to it braked 0.5 from 116 to 77 km/h and ran 1.3 m off the path, 10:16 run 8).
                self.keep_speed=None
                return done('CHANGE_CURVATURE',min(cap,max(fit,ctx.v-1.)),ref)
            if plan is None:
                # No box can be reached. With our own lane clear ahead, speed up (following, bends and the stop line
                # still cap it) and look for a free box ahead of the next car (user 2026-09-30); behind a lead, hold the
                # speed we had (no ratchet down on each measured speed).
                if lead is None:
                    self.keep_speed=None
                    return done('NO_SLOT',min(cap,stopcap),ref)
                if self.keep_speed is None:self.keep_speed=ctx.v
                return done('NO_SLOT',min(cap,stopcap,max(self.keep_speed,p.min_change_speed_mps+.5)),ref)
            if plan.kind=='TAPER' and ctx.s+car.rear_axle_to_front_m>=taper[0]:self.taper_committed=True
            if plan.mode=='KEEP':
                if self.keep_speed is None:self.keep_speed=ctx.v
                speed=self.keep_speed
            else:self.keep_speed=None;speed=plan.goal
            # a standing launch: stay put until it is due
            waiting=getattr(plan,'launch',0.)>0. and plan.start_t>0.
            if waiting:speed=0.
            mode=plan.mode
            if plan.kind=='CHANGE' and plan.passing:
                # Passing a target-lane car takes three steps (user 2026-09-30): ACCEL while it is still ahead of our
                # front, PASS while it is beside us, then the change itself (CUT_IN) only once it is behind.
                front_=ctx.s+car.rear_axle_to_front_m
                mode='ACCEL' if any(b.lo>front_ for b in bodies if b.id in plan.passing) else 'PASS'
            if waiting:mode='LAUNCH_WAIT'
            return done('SLOT_'+mode if plan.kind=='CHANGE' else 'TAPER_'+plan.mode,min(cap,speed),ref)
        if self.committed is not None:
            m=self.committed
            # Settling (and judging the next change) from half the curve (user 2026-09-30): waiting for the curve's end,
            # a 90 m 5->4 curve laid out for the speed reached accelerating held the 4->3 judgement until s1212 though
            # the car was in lane 4 from ~s1180. ...from 0.7 of it (user 2026-10-01): from half, a 4->3 change braked
            # 108 -> 55 km/h mid-curve rode the lane line ~1.2 m off the new centre beside a lane-4 car (~0.4 m apart,
            # s1320-1335) and the next change started ~1 m off centre.
            if ctx.s>=m.start_global_s+.7*(m.end_global_s-m.start_global_s):
                self.state='SETTLE'
                xy=center_at(r,m.target_lane,ctx.s);h=heading_at(r,m.target_lane,ctx.s,p.path_step_m)
                aligned=np.linalg.norm(xy-[ctx.x,ctx.y])<=p.settle_error_m and abs(wrap(h-ctx.yaw))<=p.settle_heading_rad
                self.settle_since=(ctx.now if self.settle_since is None else self.settle_since) if aligned else None
                # No speed limit of its own while settling (user 2026-09-29: held at the change-end speed the car
                # stayed at 40 km/h with the lane ahead empty). The sway is cut at its source: the change curve law (change_lat).
                # Aligned for a whole sway period (settle_window_s), not one pass through the centre.
                # Well past the curve the target lane is ours even if not yet aligned: waiting for it stranded
                # the car in SETTLE at a standstill (2026-09-27, s1466); that distance now also covers the window.
                overrun=ctx.s>=m.end_global_s+2*p.lookahead_m+ctx.v*p.settle_window_s
                # ...or once the change curve no longer runs a braking distance ahead at today's speed (the tail it was
                # built with, at the speed it was built for): sped up 62 -> 92 km/h while settling, the controller
                # ran out of reference and braked 0.5 (REFERENCE_COVERAGE, 2026-09-30 s1443).
                overrun=overrun or m.global_s[-1]-ctx.s<p.lookahead_m+ctx.v**2/(2*self.config.stopping_decel_limit)
                if overrun or (self.settle_since is not None and ctx.now-self.settle_since>=max(p.settle_s,p.settle_window_s)):
                    self.current_lane=m.target_lane;self.committed=None;self.change_passing=();self.change_front=None;self.launching=False
                    self.follow_reference=None;self.state='FOLLOW';self.plan=None;self.keep_speed=None
                    return result('CHANGE_COMPLETE',cap,self._reference(ctx))
                # Settling, the next change is judged as soon as the next lane is known (user 2026-10-04): every lane is
                # perceived from the highway entry on (occupancy and edge speeds above, all lanes each tick), so once our
                # centre is near the new lane's, the lane beside it is judged at once -- by the same slot planner, as if
                # the new lane were ours -- and a change starts only if every body of that lane has its speed measured
                # (trial in wait_gap). Only a plan whose best choice starts now is taken; one that speeds up to pass first
                # (ACCEL, starting later) leaves settling untouched and is picked up once the lane is ours. Waiting for the
                # whole settle (1.8 s aligned) ran ~100 m at 110 km/h and left no room for 3->2 (20:14 run, stopped s1581).
                nxt=m.target_lane;ni=r.lanes.index(nxt)
                # ...from where our centre is within half a car width of the new lane's centre, where the next curve starts:
                # from half the curve the controller refused the new curve (REFERENCE_DISCONNECTED, 2026-09-30).
                near_centre=np.linalg.norm(xy-[ctx.x,ctx.y])<=car.width_m/2
                if nxt!=r.lanes[-1] and ctx.s<r.rejoin_s and (ni<1 or self.cp11) and near_centre:
                    keep=(self.committed,self.current_lane,self.state,self.plan,self.keep_speed,dict(self.recent),self.change_passing,
                          self.follow_reference,self.settle_since,self.candidate,self.taper_committed,self.change_front,self.change_v0,
                          self.box_gate,self.last_assessment,dict(diag))
                    self.current_lane=nxt;self.committed=None;self.change_passing=();self.change_front=None
                    self.follow_reference=None;self.plan=None;self.keep_speed=None
                    started=wait_gap(nxt,ni,r.lanes[min(ni+1,len(r.lanes)-1)],r.change_deadline(nxt,car.width_m),trial=True)
                    if started is not None:return started
                    seen=self.recent
                    (self.committed,self.current_lane,self.state,self.plan,self.keep_speed,self.recent,self.change_passing,
                     self.follow_reference,self.settle_since,self.candidate,self.taper_committed,self.change_front,self.change_v0,
                     self.box_gate,self.last_assessment,saved_diag)=keep
                    # ...but what the trial saw in the next lane is kept from tick to tick: restored each tick, a lane-2 car seen
                    # for 1 s that dropped out of the one deciding scan read as a gap, and a 98 km/h 3->2 change met it 22 m
                    # ahead at 60 km/h and braked to 57 km/h in behind it (12:34 run, s1316; user 2026-10-08).
                    self.recent=seen
                    diag.clear();diag.update(saved_diag)
            # No speed limit of its own while changing (user 2026-09-28): following brakes for real bodies and bends.
            return result('CHANGE_FOLLOW',cap,ref)
        if ctx.s>=r.rejoin_s:
            self.state='REJOIN'
            if self.rejoin is not None and ctx.s<self.rejoin.end_s:
                return result('REJOIN_CURVE',cap,ref)
            _,error,h=r.original.project(ctx.x,ctx.y,ctx.s)
            aligned=error<=p.settle_error_m and abs(wrap(h-ctx.yaw))<=p.settle_heading_rad
            # Once handed back to the global path it stays there (user 2026-10-05): braking for CP13 the car drifted past
            # the alignment limits and was taken back to the lane reference, several times (REJOIN_COMPLETE <->
            # REJOIN_ALIGN, s1673-1685), each switch jumping the target speed (102 -> 75 -> 86 km/h) and the steering.
            if aligned or self.rejoined:
                self.rejoined=True
                return result('REJOIN_COMPLETE',cap,None,release=True)
            return result('REJOIN_ALIGN',cap,ref)
        if lane==r.lanes[-1]:self.state='FOLLOW';return result('TARGET_LANE',cap,ref)
        return wait_gap(lane,idx,target_lane,deadline)
