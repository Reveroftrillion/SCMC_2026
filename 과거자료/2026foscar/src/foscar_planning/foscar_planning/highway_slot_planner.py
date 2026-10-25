"""Slot planner: which free box of the next lane to change into, at which speed, starting when.

User 2026-09-28: a car driving at the speed of the traffic next to us has zero relative speed, so seen from
that traffic its gaps stand still and a lane change is a path into a free box among parked cars. The ego picks
one speed profile (speed up / keep / ease off to `goal`) and a moment to start the change. A plan is
admissible when
  - from where our body starts to enter the target lane until tail_s after the change ends, a target-lane body
    behind stays the pass gap away (rear_min_gap while it is faster than us; its speed error is charged: NPCs do
    not brake for a cut-in), and until the change ends one ahead stays a gap we can still brake behind (at its
    measured speed: it keeps moving and only the closing speed is shed; following then brakes for it as a lead,
    and holds us at its speed once we are in: the car behind is checked against that too);
  - the body ahead in our own lane stays brakeable until we have left the lane;
  - the change ends before the deadline, any easing off is done before it starts, and our body starts entering the
    target lane within max_enter_s of the change start (a curve committed now is not re-checked: a slow creep
    whose body enters 5 s later is a later start, planned again every tick).
Bodies whose speed differs from the traffic simply move in that frame: the check runs on each body's own
speed, so passing and yielding are one question. Per slot (the pair of bodies we land between) the plan landing
earliest wins, each m/s of speed change counted as speed_cost_s later: an empty lane is changed into at once at our
speed, a car beside is passed with a moderate speed-up rather than a flat-out sprint that starts now, and letting a
car by is sped up by easing off a little more (a plan starting later is planned again every tick); across slots the most progress over progress_horizon_s wins, behind the
slot's front body at its speed afterwards (landing behind a slow car is worth less than passing it), then the
earlier landing.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Body:
    id: str
    lo: float            # rear station
    hi: float            # front station
    v: float             # along-lane speed the plan uses
    err: float=0.        # along-lane position error of its edges
    kind: str='measured' # measured | unmeasured | shadow | lead


@dataclass(frozen=True)
class Plan:
    kind: str            # CHANGE: build a change curve at start_s / TAPER: lane 5 keeps the global path into lane 4
    mode: str            # ACCEL / KEEP / YIELD
    goal: float          # speed the profile ramps to and holds
    start_t: float
    start_s: float
    length_m: float
    cross_t: float
    end_t: float
    rear: str            # body we land ahead of ('' = none)
    front: str           # body we land behind ('' = none)
    front_v: float
    slot: tuple          # (rear nose, front tail) stations at crossing, +-inf when open
    progress: float
    passing: tuple=()    # bodies ahead of our rear bumper now that the plan lands ahead of (being passed)


def profile(v0,goal,accel,brake,ts):
    """Station offset and speed of the ramp to `goal` and hold; also the ramp time."""
    rate,sign=(accel,1.) if goal>=v0 else (brake,-1.)
    tr=abs(goal-v0)/rate
    t1=np.minimum(ts,tr)
    return v0*t1+.5*sign*rate*t1*t1+goal*np.maximum(0.,ts-tr),v0+sign*rate*t1,tr


def plan_slot(s0,v0,bodies,lead,P,change=None,taper=None,vmax=33.3,only=None,pass_first=True):
    """Best Plan or None.

    P: accel, brake, front, back, position_error, margin, latency, follow_brake, standstill_gap, safe_headway,
       pass_front_gap, pass_rear_gap, rear_gap, closing_error, closing_error_per_m, rear_accel, rear_accel_near, tail_s, dt,
       start_step, max_start, max_enter, change_time, curve_len(v), lookahead, lead_in(v), min_length(v), speed_cost, progress_horizon.
    change: dict(deadline=, latest_start=) -- a change curve built from our lane; None = none possible.
    taper: (enter_s, cross_s, done_s) -- lane 5 keeping the global path, which merges into lane 4 by itself.
    only: (goal, length) -- evaluate starting now with this curve length (commit re-check); goal None = any goal.
    pass_first: a body we land ahead of must be behind us when the change starts (False: from when our body enters).
    """
    dt=P.dt;H=P.max_start+max(12.,P.change_time*4)+P.tail_s
    ts=np.arange(0.,H+dt/2,dt);N=len(ts);tail=int(round(P.tail_s/dt))
    if only is not None and only[0] is not None:goals=[only[0]]
    else:
        lo=max(1.,v0-10.);hi=max(lo,min(vmax,v0+P.accel*(P.max_start+P.change_time)))
        goals=sorted({round(v0,3)}|{float(g) for g in np.arange(math.ceil(lo),hi+1e-9,1.)}|({hi} if hi>v0 else set()))
        goals=[g for g in goals if g>=lo-1e-9 or g==round(v0,3)]
    n=len(bodies);gap0=np.array([(s0-P.back)-b.hi for b in bodies])
    lo_b=np.array([b.lo-b.err for b in bodies]).reshape(n,1);hi_b=np.array([b.hi+b.err for b in bodies]).reshape(n,1)
    v_front=np.array([max(0.,b.v) for b in bodies]).reshape(n,1)
    v_rear=np.array([b.v+P.closing_error+P.closing_error_per_m*max(0.,g) for b,g in zip(bodies,gap0)]).reshape(n,1)
    v_nom=np.array([b.v for b in bodies]).reshape(n,1)
    # A car beside us or just behind (nose within rear_accel_near_m of our rear bumper) may also start speeding up:
    # one beside read 37 km/h, then went 37 -> 50 km/h within 1.2 s of the decision (14:15 run 6, s1307). Its nose is
    # predicted with rear_accel on top of its speed. One further back is seen whole and judged on its speed (with
    # the allowance for all, a same-speed car 15 m back still refused a tail-in).
    near=np.array([g<P.rear_accel_near for g in gap0],dtype=float).reshape(n,1)
    tail_pos=lo_b+v_front*ts;nose_pos=hi_b+v_rear*ts+.5*P.rear_accel*near*ts*ts
    best={}
    for goal in goals:
        x,v,tr=profile(v0,goal,P.accel,P.brake,ts);se=s0+x
        def at(station):
            i=int(np.searchsorted(se,station-1e-9))
            return i if i<N else None
        clf=np.maximum(0.,v-v_front)
        brake_need=P.position_error+clf*P.latency+clf*clf/(2*P.follow_brake)
        gap_f=tail_pos-(se+P.front)
        tight=np.c_[np.zeros(n,int),np.cumsum(gap_f<P.pass_front_gap+brake_need,axis=1)] if n else None
        # (no 'roomy' rule: a change is judged on stopping distances only, as it is then driven -- user 2026-09-30)
        gap_r=(se-P.back)-nose_pos
        # rear_min_gap while the car behind is measured faster than us; its speed error only moves where it is
        # (charging the error to both left a same-speed car 6 m back blocking a tail-in).
        rear=np.c_[np.zeros(n,int),np.cumsum(gap_r<np.where(v_nom>v,P.rear_gap,P.pass_rear_gap),axis=1)] if n else None
        if lead is not None:
            cl=np.maximum(0.,v-max(0.,lead.v))
            # Starting, the stop gap of the following safety cap (standstill): with position error + margin (0.5 m) only a
            # change started 1.2 m behind our lead and overlapped it (2026-09-30 run 171839). Once under way, position
            # error + margin as before (the stop gap all along refused passes that were possible, user 2026-09-30).
            pad=np.where(ts<dt/2,P.standstill_gap+P.position_error,P.position_error+P.margin)
            ok=lead.lo-lead.err+max(0.,lead.v)*ts-(se+P.front)>=pad+cl*P.latency+cl*cl/(2*P.follow_brake)
            lead_bad=np.r_[0,np.cumsum(~ok)]
        windows=[]
        if change is not None:
            starts=[0.] if only is not None else np.arange(0.,P.max_start+1e-9,P.start_step)
            for t0 in starts:
                i0=int(round(t0/dt));vs=float(v[i0])
                start=float(se[i0])+P.lead_in(vs)
                if start>change.get('latest_start',math.inf):break
                # As long as the tracker's lateral limit needs at the speed the change reaches (a quintic shift of
                # lateral_ref_m peaks at 5.77 x shift / L^2 curvature): a shorter planned curve was lengthened at
                # commit and then failed the re-check.
                vd=max(vs,goal)                                     # the speed the curve is laid out for
                length=only[1] if only is not None else max(P.min_length(vs),(vs+max(goal,vs))/2*P.change_time,P.curve_len(vd))
                if start+length>change['deadline']:break
                enter=start+.35*length-.5*P.lookahead      # the tracker turns in about half its lookahead early
                windows.append(('CHANGE',t0,start,length,enter,start+.5*length,start+.65*length,start+length))
        if taper is not None and only is None:
            e,c,d=taper;windows.append(('TAPER',0.,e,d-e,e,c,c,d))
        def braked(i0,ie,il,idn,end):
            """Our stations if we keep this profile but, like the following safety cap mid-change, shed speed (at most
            follow_brake, back up at accel) behind our own lead until we leave its lane (il) and behind the target-lane
            bodies ahead of our front when we enter (from ie), down to the speed from which we still stop standstill +
            position error behind the one braking at follow_brake (user 2026-09-30: judge a change knowing the speed may
            drop to that cap). None if even so we would close in past that, or a body behind would come too close."""
            fr=[i for i in range(n) if tail_pos[i,ie]>se[ie]+P.front]
            pad=P.standstill_gap+P.position_error;fb=P.follow_brake
            xe=x.copy();ve=v.copy()
            for k in range(i0+1,end+1):
                t=ts[k];safe=math.inf;xk=xe[k-1]+ve[k-1]*dt
                if lead is not None and k<=ic:
                    g=lead.lo-lead.err+max(0.,lead.v)*t-(s0+xk+P.front)
                    if g<pad:return None
                    safe=min(safe,max(0.,lead.v)+math.sqrt(2*fb*(g-pad)))
                if k>=ie and k<=idn:
                    for i in fr:
                        g=float(tail_pos[i,k])-(s0+xk+P.front)
                        if g<pad:return None
                        safe=min(safe,float(v_front[i,0])+math.sqrt(2*fb*(g-pad)))
                xe[k]=xk;ve[k]=min(float(v[k]),ve[k-1]+P.accel*dt,max(ve[k-1]-fb*dt,safe))
            others=[i for i in range(n) if i not in fr]
            if others:
                gr=(s0+xe[i0:end+1]-P.back)-nose_pos[others,i0:end+1]
                if np.any(gr<np.where(v_nom[others]>ve[i0:end+1],P.rear_gap,P.pass_rear_gap)):return None
            return xe,fr,others
        for kind,t0,start,length,enter,cross,leave,done in windows:
            ie,ic,il,idn=at(enter),at(cross),at(leave),at(done)
            if None in (ie,ic,il,idn) or idn+tail>=N:continue
            if goal<v0-.3 and tr>t0+1e-9:continue              # ease off before the change starts, not while changing
            if kind=='CHANGE' and ts[ie]-t0>P.max_enter+1e-9:continue
            end=idn+tail
            # Our lead matters until half the curve, where our body crosses into the new lane (user 2026-09-30: the change
            # counts as done there).
            lead_fail=lead is not None and lead_bad[ic+1]-lead_bad[0]
            # A body we land ahead of must be the pass gap behind us from the moment the change STARTS, not only from
            # when our body enters its lane (user 2026-09-30: speed up, get ahead, then cut in -- three steps). Started
            # on the prediction that we would be ahead by the time we entered, a 3->2 pass went on beside a car that
            # sped up with us and touched it (s1330). A plan starting now is thus judged on where the car is now.
            i0=int(round(t0/dt)) if pass_first else ie
            xb=None
            if n:
                r_ok=rear[:,end+1]-rear[:,i0]==0;t_ok=tight[:,idn+1]-tight[:,ie]==0
            if lead_fail or (n and not np.all(r_ok|t_ok)):
                # Too close ahead at the planned speed: starting now, judge it with the speed shed to the safety cap.
                if kind!='CHANGE' or t0>0.:continue
                xb=braked(i0,ie,il,idn,end)
                if xb is None:continue
            if xb is not None:
                fronts=np.array(xb[1],dtype=int);rears=np.array(xb[2],dtype=int)
                ftail=float(tail_pos[fronts,ic].min()) if len(fronts) else math.inf
                rnose=float(nose_pos[rears,ic].max()) if len(rears) else -math.inf
                fi=int(fronts[np.argmin(tail_pos[fronts,ic])]) if len(fronts) else None
                ri=int(rears[np.argmax(nose_pos[rears,ic])]) if len(rears) else None
            elif n:
                fronts=np.flatnonzero(~r_ok);rears=np.flatnonzero(r_ok)
                ftail=float(tail_pos[fronts,ic].min()) if len(fronts) else math.inf
                rnose=float(nose_pos[rears,ic].max()) if len(rears) else -math.inf
                fi=int(fronts[np.argmin(tail_pos[fronts,ic])]) if len(fronts) else None
                ri=int(rears[np.argmax(nose_pos[rears,ic])]) if len(rears) else None
                if fi is not None and len(rears):
                    # Once in the lane, following holds us at the front body's speed (no closing in past the safe
                    # distance after a cut-in, user 2026-09-28): the plan cannot outrun the car behind by speeding
                    # up beyond it (a 76 km/h plan held at a 71 km/h front car was caught from behind, stress case).
                    hold=max(float(v_front[fi,0]),float(v[ie]))
                    if float(v[ie:end+1].max())>hold+1e-6:
                        ve=np.minimum(v[ie:end+1],hold);xe=x[ie]+np.r_[0.,np.cumsum(ve[:-1])*dt]
                        gr=(s0+xe-P.back)-nose_pos[rears,ie:end+1]
                        if np.any(gr<np.where(v_nom[rears]>ve,P.rear_gap,P.pass_rear_gap)):continue
            else:ftail,rnose,fi,ri=math.inf,-math.inf,None,None
            # After landing we go no faster than the slowest target-lane car from the one we land behind onwards: counted
            # at the speed of that car alone, landing behind a 60 km/h car stuck behind a 50 km/h one beat passing it
            # into the box ahead (which lands behind the 50 km/h car, 3.5-4 s earlier; 2026-09-30 3->2 replays).
            fv=min(b.v for b in bodies if b.lo>=bodies[fi].lo-1e-6) if fi is not None else math.inf
            after=min(fv,max(v0,goal))
            progress=float((x if xb is None else xb[0])[idn])+after*max(0.,P.progress_horizon-ts[idn])
            key=(bodies[ri].id if ri is not None else '',bodies[fi].id if fi is not None else '')
            mode='ACCEL' if goal>v0+.3 else 'YIELD' if goal<v0-.3 else 'KEEP'
            passing=tuple(bodies[i].id for i in (rears if n else ()) if bodies[i].hi>s0-P.back)
            plan=Plan(kind,mode,goal,t0,start,length,float(ts[ic]),float(ts[idn]),key[0],key[1],fv,(rnose,ftail),progress,passing)
            old=best.get(key)
            cost=plan.end_t+P.speed_cost*abs(goal-v0)
            if old is None or cost<old.end_t+P.speed_cost*abs(old.goal-v0)-1e-9:best[key]=plan
    if not best:return None
    return max(best.values(),key=lambda p:(round(p.progress,1),-p.end_t,-abs(p.goal-v0)))
