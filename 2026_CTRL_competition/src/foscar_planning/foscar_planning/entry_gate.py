"""Opt-in simulator entry veto. Candidate gaps are not visibility certificates.

Only restricts existing targets; never clears a pedestrian/red-light stop.
Entry coordinates are calibrated global rear-axle s, not a generic junction detector.
"""
import math
from dataclasses import replace

from .ring_merge import command as merge_command, commit_speed


class EntryGate:
    defaults = dict(enabled=False, approach_s=860., entry_s=880., merge_s=886.,
                    end_s=920., stop_at_s=-1., clear_s=.6, max_speed_kmh=20., max_age_s=.45,
                    late_brake_mps2=0.)

    def __init__(self, **params):
        for key, value in self.defaults.items():
            setattr(self, key, params.get(key, value))
        if not self.approach_s < self.entry_s < self.merge_s < self.end_s:
            raise ValueError('entry gate requires ordered approach/entry/merge/end s')
        if self.stop_at_s >= 0. and not self.approach_s <= self.stop_at_s < self.end_s:
            raise ValueError('fixed waiting point must lie inside entry-control region')
        self.ring_speed = 25/3.6   # the ease-off entry's ring speed (behavior_node sets the predictor's)
        self.ring_merge = None     # ring_merge.RingMerge when behavior_node plans the entry with it
        self.reset()
        self.previous_s = None

    def reset(self):
        self.clear_since = None
        self.last_evidence = -math.inf
        self.gap_pair = None
        self.permitted = self.committed = False
        self.reason = 'inactive'
        self.waiting_s = None
        self.stopped_at_entry = False
        self.launch_stamp = None
        self.entered_ring = False
        self.finished = False
        self.strategy = None; self.strategy_speed = 0.   # entry speed strategy admitted on, and our speed then
        self.commit_line = None    # ring_merge: committed line speed, held until the body reaches the ring

    def owns_vehicle_prediction(self, ctx):
        end=self.end_s if getattr(self,'circular_mode',False) else self.merge_s
        return self.enabled and not self.finished and self.approach_s <= ctx.s < end

    def apply(self, target, ctx, prediction):
        if not self.enabled:
            return target
        if self.previous_s is not None and ctx.s < self.previous_s-2.:
            self.reset()
        self.previous_s = ctx.s
        if not self.approach_s <= ctx.s <= self.end_s:
            self.reset()
            return target
        fixed = self.stop_at_s >= 0.
        entry = self.stop_at_s if fixed else self.entry_s
        data=prediction or {}
        if (self.committed and self.entered_ring and data.get('admission_model')=='three_vehicle_entry'
                and data.get('decision')=='EXITED' and 0<=ctx.now-data.get('evaluated_at',-math.inf)<=self.max_age_s):
            self.finished=True
        if self.finished:
            self.reason='exited'
            return target
        # Predictor currently evaluates the centerline, not an avoidance path.
        if abs(target.lat_offset) > 1e-3:
            self.clear_since = None
            self.permitted = False
            self.reason = 'unmodeled lateral path'
            return replace(target, state='ROUNDABOUT_WAIT', v_max=0.)
        data = prediction or {}
        if data.get('model') == 'ring_merge' and not self.committed:
            return self.apply_merge(target, ctx, data)
        stamp = data.get('evidence_stamp', -math.inf)
        fresh = (0 <= ctx.now-data.get('evaluated_at', -math.inf) <= self.max_age_s
                 and 0 <= ctx.now-stamp <= self.max_age_s)
        # Once the body enters the merge, don't command a stop behind the car.
        # Existing YIELD and imminent predicted conflicts still brake normally.
        contact=(bool(data.get('ego_overlaps_ring')) if data.get('model')=='circular_gap'
                 else ctx.s>=self.merge_s)
        if self.committed and contact:self.entered_ring=True
        if self.committed and data.get('admission_model')=='three_vehicle_entry' and not self.entered_ring:
            interrupted=(target.v_max<=0. or math.isfinite(target.stop_s) or not fresh
                or (self.launch_stamp is not None and ctx.now-self.launch_stamp>.6 and abs(ctx.v)<.1)
                or (data.get('conflict_side')!='rear' and data.get('conflict_in_s',math.inf)<=2.))
            if interrupted:
                self.committed=self.permitted=self.stopped_at_entry=False
                self.clear_since=None
                self.launch_stamp=None
                self.commit_line=None
        if self.permitted and contact:
            self.committed = True
        limit = self.max_speed_kmh/3.6
        if self.committed:
            # Evidence a little late (<= 1 s) while plainly following in the ring: hold speed rather
            # than stop. Stale evidence pulsed v_max 0 / brake 0.42 behind a safe lead (2026-09-27,
            # TuningPoint #1). Older evidence or a predicted conflict still stops.
            late = (not fresh and data.get('decision') == 'FOLLOW'
                    and 0 <= ctx.now-data.get('evaluated_at', -math.inf) <= 1.
                    and 0 <= ctx.now-stamp <= 1.)
            if late:
                self.reason = 'evidence late: holding speed'
                return replace(target, v_max=min(target.v_max, max(0., ctx.v)), state=target.state)
            emergency = (not fresh or (data.get('decision') in ('WAIT','UNKNOWN')
                         and data.get('conflict_in_s', math.inf) <= max(2.,max(0.,ctx.v)/1.5+.4)
                         and not ((data.get('cooperative_rear') or data.get('admission_model')=='three_vehicle_entry') and data.get('conflict_side')=='rear')))
            self.reason = 'imminent conflict' if emergency else 'merging'
            follow=min(limit,data.get('following_speed_mps',limit))
            if self.commit_line is not None and self.ring_merge is not None:
                follow=min(follow,commit_speed(self.ring_merge,self.commit_line,ctx.s,self.stop_at_s,limit))
            return replace(target, v_max=min(target.v_max, 0. if emergency else follow),
                           state='ROUNDABOUT_BRAKE' if emergency else
                           ('ROUNDABOUT_FOLLOW' if data.get('ego_aligned',False) else 'ROUNDABOUT_ENTER') if data.get('model')=='circular_gap' else target.state)
        circular=data.get('model')=='circular_gap'
        stage=entry if fixed else data.get('staging_s',entry) if fresh else entry
        stage=max(entry,min(stage,entry+8.))
        if not fixed and circular and fresh and 'staging_s' in data:
            # Latch the first measured waiting limit. A tighter limit can stop
            # approach, but a later wider estimate must not restart creeping.
            self.waiting_s=stage if self.waiting_s is None else min(self.waiting_s,stage)
        if circular and self.waiting_s is not None:
            stage=min(stage,self.waiting_s)
        at_stage=ctx.s>=stage-.3
        if circular and fresh and at_stage and abs(ctx.v)<=.1:
            self.stopped_at_entry=True
            # Once stopped inside the approach deadband, hold this pose.
            # Do not alternate APPROACH/WAIT around a floating 0.2m gap.
            self.waiting_s=min(stage,ctx.s)
            stage=self.waiting_s
        pair = tuple(data.get('gap_pair', ()))
        if pair != self.gap_pair:
            if circular and abs(ctx.v)>.1:self.stopped_at_entry=False
            self.clear_since = None
            self.permitted = False
            self.gap_pair = pair
        local_admission=data.get('admission_model')=='three_vehicle_entry'
        clear = (fresh and data.get('decision') == 'CANDIDATE_GAP'
                 and (data.get('lane_tracks', 0) > 0 or data.get('empty_lane',False))
                 and (local_admission or not circular or self.stopped_at_entry))
        if not clear:
            if circular and abs(ctx.v)>.1:self.stopped_at_entry=False
            self.clear_since = None
            self.permitted = False
        elif stamp > self.last_evidence:
            if self.clear_since is None:
                self.clear_since = stamp
            required=0. if local_admission else .2 if data.get('cooperative_rear') else self.clear_s
            self.permitted = stamp-self.clear_since >= required
            self.last_evidence = stamp
        if self.permitted:
            if data.get('admission_model')=='three_vehicle_entry' and target.v_max>0. and not math.isfinite(target.stop_s):
                self.committed=True
                self.launch_stamp=ctx.now
            # Drive the entry the admission was judged on (user 2026-09-29): speed up -> the rule speed, keep -> the speed
            # we had when admitted, ease off -> the ring speed.
            strategy=data.get('ego_strategy','decel')
            if strategy!=self.strategy:self.strategy=strategy;self.strategy_speed=max(0.,ctx.v)
            speed=limit if strategy=='accel' else max(self.strategy_speed,1.) if strategy=='keep' else min(limit,self.ring_speed)
            self.reason = f'fresh gap confirmed ({strategy})'
            return replace(target, v_max=min(target.v_max, speed),state='ROUNDABOUT_ENTER' if target.v_max>0. and not math.isfinite(target.stop_s) else target.state)
        self.reason = data.get('decision', 'UNKNOWN')
        # Target.stop_s is front axle s; calibrated waiting pose is rear axle s.
        creeping=(not fixed and data.get('model')=='circular_gap' and fresh and stage>ctx.s+.2)
        # Not admitted: the base "ease off" strategy is the only braking before the entry (user 2026-09-29: the choice of
        # speed up / keep / ease off is the braking logic). One profile that stops at the entry at late_brake_mps2: far
        # out it is above the rule speed (speed kept), near the entry it eases us down continuously, and an admission on
        # any strategy lifts it at once. Removed: the hold until the last stop point, the entry stop target sent from
        # far out (the controller's 1 m/s2 stop ramp braked ~50 m early) and the 3 m/s2 zero cut (it fought the hold:
        # v_max 60 <-> 0 every few ticks at s853, 00:42 run 2). The stop target (which holds the exact waiting pose)
        # is sent only in the last 2 m.
        far_approach=ctx.s<entry-2.
        gap=(stage if creeping else entry)-ctx.s
        # (less our speed x 0.4 s of brake response: without it the car stood 2-2.7 m past the waiting pose, 01:02 runs)
        v=max(0.,ctx.v)
        speed=math.sqrt(2.*self.late_brake_mps2*max(0.,gap-.3-.4*v)) if self.late_brake_mps2>0. else (limit if gap>.3 else 0.)
        if creeping and not far_approach:speed=min(speed,.8)
        stop=target.stop_s
        if gap<=2.:
            stop=min(stop,max(entry,ctx.s)+3.) if math.isfinite(stop) else max(entry,ctx.s)+3.
        return replace(target, state='ROUNDABOUT_APPROACH' if creeping else 'ROUNDABOUT_WAIT', stop_s=stop,
                       v_max=min(target.v_max, limit, speed))

    def apply_merge(self, target, ctx, data):
        """Before the commit, ring_merge plans the entry: drive on, slow down, or stop at the line; commit at the
        point where the next plan could no longer stop (ring_merge.command). Stale plans only keep a stop possible."""
        merge=self.ring_merge
        limit=self.max_speed_kmh/3.6
        stop_s=self.stop_at_s if self.stop_at_s>=0. else self.entry_s
        fresh=0<=ctx.now-data.get('evaluated_at',-math.inf)<=self.max_age_s
        if merge is None or merge.table is None:
            self.reason='ring_merge unavailable'
            return replace(target,state='ROUNDABOUT_WAIT',v_max=min(target.v_max,limit) if ctx.s<stop_s-2. else 0.)
        plan=data if fresh else dict(decision='WAIT')
        upstream_stop=target.v_max<=0. or math.isfinite(target.stop_s)
        speed,stop,commit,state=merge_command(merge,plan,ctx.s,ctx.v,stop_s,limit)
        if commit and not upstream_stop:
            self.committed=self.permitted=True
            self.launch_stamp=ctx.now
            self.commit_line=plan['commit_line_mps']
            self.strategy='ring_merge'
            self.reason=plan.get('reason','')
            return replace(target,v_max=min(target.v_max,speed),state='ROUNDABOUT_ENTER')
        self.reason=plan.get('reason','stale plan') if fresh else 'stale plan'
        out=replace(target,v_max=min(target.v_max,speed),state=state)
        if stop is not None:
            front=stop+3.     # Target.stop_s is front axle s; the waiting pose is rear axle s
            out=replace(out,stop_s=min(target.stop_s,front) if math.isfinite(target.stop_s) else front)
        return out

