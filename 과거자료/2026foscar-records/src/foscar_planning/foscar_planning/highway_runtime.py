"""Mission-scoped adapter: shadow decisions cannot change the driving target."""
from dataclasses import replace, asdict
import math
from types import SimpleNamespace
from .highway_observation import HighwayObservations
from .highway_policy import HighwayPolicy
from .highway_maneuver import motion
from .highway_scene import highway_scene
from .highway_boxes import lane_boxes


class HighwayRuntime:
    def __init__(self,config,route,mission_bounds=None):
        if route is not None and mission_bounds is not None:
            if any(not math.isclose(a,b,abs_tol=1e-6) for a,b in zip(mission_bounds,(route.start_s,route.end_s))):
                raise ValueError('highway mission boundary disagrees with checkpoint geometry')
        self.config,self.route=config,route
        p=config.policy
        self.observations=HighwayObservations(p.history_s,p.max_scan_age_s,p.min_history_s,
                                             p.velocity_error_mps,p.max_observation_gap_s)
        self.policy=HighwayPolicy(route,config) if route is not None else None
        self.diagnostic={};self.active=False

    def reset(self):
        # behavior_node resets mission runtimes on a mission change *before* apply(); remember
        # that the highway was driving so the next non-highway tick still hands the reference back.
        if self.active:self.release_pending=True
        self.observations.reset()
        if self.policy is not None:self.policy.reset()
        self.diagnostic={};self.active=False

    def accept(self,data,now):return self.observations.accept(data,now)

    def apply(self,target,ctx):
        if ctx.mission_name!=self.config.mission or self.config.mode=='off':
            if self.active:self.reset()
            if getattr(self,'release_pending',False):
                self.release_pending=False;self.release_until=ctx.now+3.
            # Leaving the mission hands the car back to the global route even if the policy's
            # tight 0.3 m / 3 deg rejoin alignment was not met yet; the controller releases the
            # local reference under its own tolerance. Without it the reference just vanished at
            # CP13 and the controller stopped with REFERENCE_MISSING (2026-09-27, s1759).
            if ctx.now<getattr(self,'release_until',-math.inf) and self.config.mode=='control':
                return replace(target,reference_release=True)
            return target
        if not self.active:self.active_since=ctx.now
        self.active=True
        if self.policy is None:
            self.diagnostic=dict(state='FOLLOW',reason='MAP_UNAVAILABLE',mode=self.config.mode,stamp=ctx.now)
            return replace(target,v_max=0.) if self.config.mode=='control' else target
        snapshot=self.observations.snapshot(ctx.now)
        if ctx.s<self.policy.route.start_s:
            # Mission approach (straight after the post-roundabout curve, before CP10): the lane map starts at
            # CP10, so only observe and show it; speed (60 km/h rule) and path stay the ordinary ones.
            self.diagnostic=dict(state='APPROACH',reason='BEFORE_CP10',mode=self.config.mode,stamp=ctx.now,global_s=ctx.s,
                ego_xy=[ctx.x,ctx.y],ego_yaw=ctx.yaw,ego_speed=ctx.v,observed_count=len(snapshot.objects),
                objects=[dict(id=o.id,xy=o.xy.tolist(),speed_ready=o.speed_ready,velocity=motion(o).tolist()) for o in snapshot.objects.values()])
            return target
        decision=self.policy.tick(ctx,snapshot)
        self.diagnostic=dict(decision.diagnostics,mode=self.config.mode,stamp=ctx.now,
            control_profile=asdict(self.config.control_profile) if self.config.control_profile else None,
            ego_xy=[ctx.x,ctx.y],ego_yaw=ctx.yaw,ego_speed=ctx.v,
            current_lane=self.policy.current_lane,
            objects=[dict(id=o.id,xy=o.xy.tolist(),speed_ready=o.speed_ready,velocity=motion(o).tolist()) for o in snapshot.objects.values()])
        maneuver=self.policy.committed or self.policy.candidate
        if maneuver is not None:
            mask=(maneuver.global_s>=max(ctx.s,maneuver.start_global_s-5.)) & (maneuver.global_s<=maneuver.end_global_s+5.)
            self.diagnostic.update(target_lane=maneuver.target_lane,change_path=maneuver.xy[mask][::2].tolist())
        # speeds shown as the judgement has them (box edges), not the object tracker's (often '?': ids churn)
        self.diagnostic['scene']=highway_scene(self.policy,ctx,self.policy.box_snapshot or snapshot)
        gate=self.policy.box_gate
        # Show the next lane's boxes every tick (user 2026-09-28), also on ticks that planned nothing. Display only.
        lanes=self.policy.route.lanes;cur=self.policy.current_lane
        if gate is None and cur in lanes and cur!=lanes[-1] and snapshot.valid:
            try:
                nxt=lanes[lanes.index(cur)+1]
                gate=SimpleNamespace(lane=nxt,permitted=False,reasons=(),region=(ctx.s,ctx.s),rear=None,
                                     boxes=tuple(lane_boxes(self.policy.route,self.config,ctx,snapshot,nxt)))
            except ValueError:gate=None
        if gate is not None:
            region=[round(v,1) if math.isfinite(v) else round(ctx.s+(1 if v>0 else -1)*(self.config.boxes.front_m if v>0 else self.config.boxes.rear_m),1) for v in gate.region]
            self.diagnostic['boxes']=dict(lane=gate.lane,permitted=gate.permitted,
                reasons=list(gate.reasons),region=region,rear=gate.rear,
                boxes=[[round(s0,1),round(s1,1),state,key,[[round(x,2),round(y,2)] for x,y in poly]] for s0,s1,state,key,poly in gate.boxes])
        if self.config.mode!='control':return target
        v_max=decision.v_max
        if decision.reason=='SCAN_UNKNOWN' and (ctx.now-self.active_since<.6 or ctx.now-self.observations.stamp<.6):
            # Hold speed (no stop request) while a scan gap is short: at mission start the first
            # scan is not in yet (brake 0.5 at s1121 every lap), and mid-highway processing
            # hitches of ~0.3 s pulsed brake 0.5 near s1370 (2026-09-27). Gaps over 0.6 s still stop.
            v_max=max(v_max,ctx.v)
        return replace(target,state='HIGHWAY_'+decision.state,v_max=min(target.v_max,v_max),
            stop_s=decision.stop_s if math.isfinite(decision.stop_s) else target.stop_s,
            reference=decision.reference,reference_release=decision.reference_release,
            lat_offset=0.,entry_s=math.nan,entry_length_m=0.,return_s=math.nan,return_length_m=0.)
