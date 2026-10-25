"""Roundabout entry as speed planning (user 2026-10-01): drive on, slow down or stop only when the ring says so.

The old entry (circular_gap.evaluate + entry_gate) asked for proof before moving: an empty ring had to be shown
empty by ground returns, which the 16-ring lidar rarely gives (2026-10-01 23:08 lap: 0-10 of 13 patches cleared on
an empty ring), a gap needed two bounding cars with ready speeds, and the gate stopped at the line first. The car
stopped every lap, missed one-car gaps and decided too late for a gap it could have timed.

Here:
- Until the car can no longer stop at the waiting line (stop_decel_mps2 after latency_s), nothing has to be proven.
  The car keeps the fastest approach speed that a stop at the line still allows.
- Among approach speeds, the fastest whose entry clears every observed ring car is driven. Lower speeds time the
  arrival behind a car instead of stopping for it.
- To commit (the point of no stop, or launching from the line), the entry must also clear unobserved traffic: a car
  may be just beyond the lidar's reach (visible_range_m) coming at ring_speed_mps. Ring cars nearer than that are
  seen: 2026-10-01 laps 22:45 and 23:08, ring cars detected 100 % up to ~38 m from the lidar, 33-54 % at 38-48 m.
  A car hidden behind another ring car cannot reach the entry before the car hiding it (one lane).

Arc coordinate u: metres along the ring in its direction of travel, 0 where our body first reaches the ring lane.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class RingBody:
    """A ring car on the arc coordinate: centre u now, extent behind/ahead of the centre, speed range."""
    id: str
    u: float
    back: float
    front: float
    v_lo: float
    v_hi: float
    top: float = math.inf          # speed it can reach: the car ahead of it in the queue (queue_caps)


class RingMerge:
    defaults = dict(
        lane_half_m=1.35,          # ring cars' radial half-width (circular_gap radial_lo/hi 1.15 + 0.2)
        band_margin_m=.3,
        visible_range_m=38.,       # measured (module docstring)
        ring_speed_mps=22.5/3.6,   # ring cars' top speed, 9 laps of 2026-10-01 (p50 20.5, p99 22.0, max 22.5 km/h)
        a_lat_mps2=4.5,            # low_speed.yaml a_lat_mps2: the curve speed the controller drives
        accel_mps2=3.5,            # entry at full pedal: 5.2 m/s2 over the first second, 3.3-3.8 over two (departure_profile)
        launch_delay_s=.2,         # admission to moving from a standstill (departure_profile)
        launch_accel_mps2=5.2,     # full-pedal launch, first launch_accel_s (departure_profile: 5.2-5.3 m/s2 over the
        launch_accel_s=1.,         # first second; 2026-10-02 rolling entry: 5.6 m in 1.6 s to 7.3 m/s, 3.5 predicted 2.1 s)
        decel_mps2=4.,             # follows the approach envelope (approach_decel_mps2)
        stop_decel_mps2=5.,        # brake 0.5 ~ 5 m/s2 (measured)
        approach_decel_mps2=4.,    # the approach keeps a stop at the line within this (behavior.yaml late_brake_mps2)
        decide_ahead_s=.2,         # one planning period: decide while the next one could still stop
        latency_s=.4,
        front_margin_m=.5,         # to a car ahead in the ring
        rear_margin_m=.5,          # to a car behind, plus closing speed x rear_headway_s
        rear_headway_s=.6,
        rear_brake_mps2=2.5,       # a ring car slowing for us: ring cars braked at most 2.49 m/s2 over 0.5 s (p0.1, 36326
                                   # windows 2026-10-01/02); they are not seen to brake harder
        kf_sigmas=3.9,             # speed range = Kalman estimate +- this x sigma: |error|/sigma p99 3.87 (RingTrack.kf_update)
        body_decel_mps2=0.,        # the car ahead is taken not to brake (user 2026-10-02: tuck in behind it); measured p1 0.4
        lead_floor_mps=5.2/3.6,    # ...nor to stop: ring cars on the lane moved >= 5.2 km/h 95 % of the time (6190 samples
                                   # 2026-10-01/02; 4.9 % below 5 km/h -- queues, the ring following brakes for those)
        body_accel_mps2=1.5,
        queue_gap_m=15.,           # a car closer than this behind another is held to its speed
        position_growth_mps=.15,   # our own position error growth along the plan
        horizon_s=12.,
        follow_gap_m=1.,           # ring following in the plan (profile)
        follow_decel_mps2=3.,
        step_s=.1,
        lidar_x_m=2.,              # lidar ahead of the rear axle (bringup static tf)
        coast_s=3.5,               # ring cars within 30 m dropped out of the scan for p50 0.14, p99 3.35, max 3.47 s
                                   # (47 dropouts, 16 CP08 runs 2026-10-02): a lost car keeps moving this long
        speeds_mps=(16.7, 12., 10., 9., 8., 7., 6., 5., 4., 3., 2.),   # below 2: stop and wait instead of creeping in
    )

    def __init__(self, circle, route, s_offset=0., s_from=820., s_to=910., **params):
        for key, value in self.defaults.items():
            setattr(self, key, params.get(key, value))
        self.circle = circle
        self.table = None
        self.memory = {}
        if circle is not None and route is not None:
            self.build(route, s_offset, s_from, s_to)

    # ------------------------------------------------------------------ geometry
    def build(self, route, s_offset, s_from, s_to):
        """Per rear-axle s: our body's arc interval when it overlaps the ring lane, and the curve speed."""
        c, R = self.circle.center, self.circle.radius
        band = self.lane_half_m+self.band_margin_m
        ss = np.arange(s_from, s_to+.001, .25)
        rows = []
        for s in ss:
            x, y = route.at(s+s_offset); h = route.heading_at(s+s_offset)
            co, si = math.cos(h), math.sin(h)
            cx, cy = x+1.4825*co, y+1.4825*si
            pts = np.array([(cx+u*co-v*si, cy+u*si+v*co) for u in (-2.3175, 0., 2.3175) for v in (-.946, 0., .946)])
            r = np.linalg.norm(pts-c, axis=1)
            ang = np.arctan2(pts[:, 1]-c[1], pts[:, 0]-c[0])*self.circle.direction
            rows.append((r.min()-R, r.max()-R, ang))
        inside = [lo <= band and hi >= -band for lo, hi, _ in rows]
        if not any(inside):
            self.table = None
            return
        first = inside.index(True)
        self.s_in = float(ss[first])
        within = [i for i, (lo, hi, _) in enumerate(rows) if i > first and lo >= -band and hi <= band]
        self.s_merged = float(ss[within[0]]) if within else float(ss[first])+10.
        self.s_out = float(ss[len(inside)-1-inside[::-1].index(True)])+.25
        self.origin = float(np.min(rows[first][2]))           # u = 0: the first contact
        lo = np.full(len(ss), np.nan); hi = np.full(len(ss), np.nan)
        for i, (_, _, ang) in enumerate(rows):
            if inside[i]:
                u = np.array([wrap(a-self.origin) for a in ang])*R
                lo[i], hi[i] = u.min(), u.max()
        k = np.array([abs(wrap(route.heading_at(s+s_offset+1.)-route.heading_at(s+s_offset-1.)))/2. for s in ss])
        vcurve = np.sqrt(self.a_lat_mps2/np.maximum(k, 1e-4))
        self.table = dict(s=ss, lo=lo, hi=hi, inside=np.array(inside), vcurve=vcurve)

    def ego_interval(self, s):
        t = self.table
        i = int(round((s-t['s'][0])/.25))
        if i < 0 or i >= len(t['s']) or not t['inside'][i]:
            return None
        return t['lo'][i], t['hi'][i]

    def curve_speed(self, s):
        t = self.table
        i = min(max(int(round((s-t['s'][0])/.25)), 0), len(t['s'])-1)
        j = min(i+8, len(t['s']))          # the next 2 m: the controller slows ahead of a curve
        return float(np.min(t['vcurve'][i:j]))

    def ring_u(self, x, y):
        a = math.atan2(y-self.circle.center[1], x-self.circle.center[0])*self.circle.direction
        return wrap(a-self.origin)*self.circle.radius

    def visible_upstream(self, lidar_xy):
        """Arc length upstream of u=0 the lidar sees without a gap (ring points within visible_range_m)."""
        c, R = self.circle.center, self.circle.radius
        seen = 0.
        for a in np.arange(0., 80., 1.):
            ang = (self.origin-a/R)*self.circle.direction
            if math.hypot(c[0]+R*math.cos(ang)-lidar_xy[0], c[1]+R*math.sin(ang)-lidar_xy[1]) > self.visible_range_m:
                break
            seen = float(a)
        return seen

    # ------------------------------------------------------------------ our motion
    def envelope(self, s, stop_s, v_line):
        """Approach speed limit: v_line at the waiting line, braking toward it at approach_decel_mps2."""
        return math.sqrt(v_line*v_line+2.*self.approach_decel_mps2*max(0., stop_s-s))

    def profile(self, s0, v0, v_cap, stop_s, v_line, wait=0., bodies: Sequence[RingBody] = ()):
        """Rear-axle s and speed every step. Up to the waiting line as fast as v_cap, the curve and a braking
        envelope to v_line at the line allow; v_line 0 stops there for wait seconds. From the line to the ring
        (s_in) at most v_line (when > 0), then the curve / rule speed. In the ring lane it follows the nearest car
        ahead as the ring following does: a speed that still stops follow_gap_m behind it at
        follow_decel_mps2. Ends once the body has left the ring lane, or at the horizon."""
        out = []
        s, v = s0, max(0., v0)
        held = launch = None
        for k in range(int(self.horizon_s/self.step_s)+1):
            t = k*self.step_s
            out.append((t, s, v))
            if s >= self.s_out:
                break
            if v_line <= 0. and held is None and s >= stop_s-.3 and v < .2:
                held = t
            if v_line <= 0. and (held is None or t < held+wait):
                goal = self.envelope(s, stop_s, 0.) if held is None else 0.
            elif s < stop_s:
                goal = self.envelope(s, stop_s, v_line)
            elif s < self.s_in and v_line > 0.:
                goal = v_line
            else:
                goal = v_cap
            goal = min(goal, v_cap, self.curve_speed(s))
            iv = self.ego_interval(s)
            if iv is not None and bodies:
                # Merging, we keep the clearance the check asks for (the car's slow case, our position growth);
                # merged, the ring following has it: its nominal motion (over many seconds the slow case would
                # stop the queue and us with it).
                merging = s < self.s_merged
                ahead = [self.body_at(b, t, True) if merging else (b.u+(b.v_lo+b.v_hi)/2*t, (b.v_lo+b.v_hi)/2)
                         for b in bodies]
                ahead = [(u-b.back-iv[1], vb) for (u, vb), b in zip(ahead, bodies) if u >= (iv[0]+iv[1])/2]
                if ahead:
                    gap, vb = min(ahead)
                    room = max(0., gap-self.follow_gap_m-(self.position_growth_mps*t if merging else 0.))
                    goal = min(goal, math.sqrt(vb*vb+2.*self.follow_decel_mps2*room))
            if v < goal:
                if v < .1 and goal > 0. and launch is None:
                    launch = t+self.launch_delay_s
                if launch is None or t >= launch:
                    a = self.launch_accel_mps2 if launch is not None and t < launch+self.launch_accel_s else self.accel_mps2
                    v = min(goal, v+a*self.step_s)
            else:
                launch = None
                v = max(goal, v-max(self.decel_mps2, (v-goal)/self.step_s if goal <= 0. else 0.)*self.step_s)
            s += v*self.step_s
        return out

    # ------------------------------------------------------------------ clearance
    def body_at(self, b: RingBody, t, slow, top=math.inf):
        """Centre u and speed at t. Slow case: v_lo braking at body_decel_mps2. Fast case: v_hi speeding up at
        body_accel_mps2 to ring_speed_mps. Measured on 9 laps of 2026-10-01 (ring cars, constant-speed prediction
        error over 1/3/5 s): late p1 -0.24/-1.97/-4.44 m (~0.4 m/s2), early p95 +1.2/+7.6/+16.7 m from cars
        speeding up after a slow-down (9 -> 21 km/h in 2.4 s); a car already at ring speed hardly gains.
        top: what is ahead of it allows no more (queue_caps; us, once it is behind us)."""
        if slow:
            a = self.body_decel_mps2
            v0 = max(b.v_lo, min(self.lead_floor_mps, b.v_hi))
            tt = min(t, v0/a) if a > 0. else t
            return b.u+v0*tt-.5*a*tt*tt, max(0., v0-a*tt)
        a = self.body_accel_mps2
        top = max(b.v_hi, min(self.ring_speed_mps, b.top, top))
        tt = min(t, (top-b.v_hi)/a) if a > 0. else 0.
        return b.u+b.v_hi*tt+.5*a*tt*tt+(b.v_hi+a*tt)*(t-tt), b.v_hi+a*tt

    def rear_room(self, v_rear, v):
        """Room a car behind needs: margin, reaction at the closing speed, and its braking to our speed."""
        closing = max(0., v_rear-v)
        return self.rear_margin_m+closing*self.rear_headway_s+(v_rear*v_rear-min(v, v_rear)**2)/(2.*self.rear_brake_mps2)

    def queue_caps(self, bodies: Sequence[RingBody]):
        """A ring car speeds up only as far as the car ahead of it lets it (2026-10-01 13:33 lap: 4-6 cars in
        the ring at ~11 km/h for 30 s): its top is the next car ahead's v_hi within queue_gap_m, else ring speed."""
        order = sorted(bodies, key=lambda b: b.u)
        for i, b in enumerate(order):
            ahead = next((o for o in order[i+1:] if o.u-o.back-(b.u+b.front) <= self.queue_gap_m), None)
            b.top = ahead.v_hi if ahead is not None else math.inf
        return bodies

    def conflict(self, plan, bodies: Sequence[RingBody], frontier: Optional[float]):
        """First (time, id) at which our body in the ring lane is too close to a ring car, or None.
        frontier: u of the nearest place unobserved traffic can be now (it moves up at ring_speed_mps).
        Cars ahead count until our body is wholly in the lane (s_merged); a car behind until then, and at that
        moment once more for the room to slow to our speed (rear_room); afterwards it follows us.
        A plan that has not left the ring lane by the horizon is (horizon, 'horizon'): not checked, not cleared."""
        merged_checked = False
        for t, s, v in plan:
            iv = self.ego_interval(s)
            if iv is None:
                continue
            if s >= self.s_merged:
                # Merged: the ring following has the car ahead; a car behind follows us (ring cars keep to the car
                # in front: 13:33 queue). Check once that it can still slow to our speed behind us.
                if merged_checked:
                    continue
                merged_checked = True
            spread = self.position_growth_mps*t
            e_lo, e_hi = iv[0]-spread, iv[1]+spread
            for b in bodies:
                for slow in (True, False):
                    u, vb = self.body_at(b, t, slow)
                    if u < (e_lo+e_hi)/2 and not slow:
                        u, vb = self.body_at(b, t, False, v)     # behind us: it follows us
                    lo, hi = u-b.back, u+b.front
                    if u >= (e_lo+e_hi)/2:       # ahead of us
                        if s < self.s_merged and lo-e_hi < self.front_margin_m:
                            return t, b.id
                    elif e_lo-hi < self.rear_room(vb, v):
                        return t, b.id
            if frontier is not None:
                # One lane: unobserved traffic stays behind the visible cars upstream of it.
                f = min([frontier+self.ring_speed_mps*t]+[self.body_at(b, t, False)[0]-b.back for b in bodies if b.u > frontier])
                if e_lo-f < self.rear_room(self.ring_speed_mps, v):
                    return t, 'unobserved'
        if plan[-1][1] < self.s_out:
            return plan[-1][0], 'horizon'
        return None

    def coast(self, bodies: List[RingBody], now):
        """Bodies plus those seen within coast_s that the scan lost, moved on over their speed range. 2026-10-02 settle-18
        lap: a dump truck at the entry dropped out of the scan for ~2 s and the entry was committed into it. A body seen
        where it would be takes its place."""
        out = list(bodies)
        seen = {b.id for b in bodies}
        for key, (stamp, b) in self.memory.items():
            age = now-stamp
            if key in seen or not 0 <= age <= self.coast_s:
                continue
            u = b.u+b.v_lo*age
            back, front = b.back, b.front+(b.v_hi-b.v_lo)*age
            if any(c.u-c.back < u+front and u-back < c.u+c.front for c in bodies):
                continue
            out.append(RingBody(key, u, back, front, b.v_lo, b.v_hi))
        for b in bodies:
            self.memory[b.id] = (now, b)
        self.memory = {k: v for k, v in self.memory.items() if 0 <= now-v[0] <= self.coast_s}
        return out

    # ------------------------------------------------------------------ decision
    def candidates(self, v_cap):
        """(v_line, wait), roughly earliest arrival first: through the line at a speed, then stop and wait."""
        out = [(min(x, v_cap), 0.) for x in self.speeds_mps]
        out += [(0., w) for w in np.arange(0., self.horizon_s-2., .5)]
        return out

    def wait_station(self, stop_s):
        """Where to wait: far enough back from the ring that a launch reaches ring speed at the first contact (s_in) --
        entering at ring speed, the car behind need not brake for us (2 s spawn ring, 2026-10-02: standing at the line
        we met the ring at ~2 m/s, needed > 10 m behind and waited 30 s with 7 m gaps). Launch as in profile."""
        v, d, a1 = 0., 0., self.launch_accel_mps2*self.launch_accel_s
        # launch phase, then accel_mps2
        if self.ring_speed_mps <= a1:
            d = self.ring_speed_mps**2/(2.*self.launch_accel_mps2)
        else:
            d = a1**2/(2.*self.launch_accel_mps2)+(self.ring_speed_mps**2-a1**2)/(2.*self.accel_mps2)
        return min(stop_s, self.s_in-d)

    def plan(self, s, v, lidar_xy, bodies: List[RingBody], stop_s, v_cap, now=0.):
        """The earliest approach clearing the observed ring cars (drive), and the earliest also clearing
        unobserved traffic without a wait (commit); see the module docstring."""
        if self.table is None:
            return dict(model='ring_merge', decision='UNKNOWN', reason='ring geometry unavailable', evaluated_at=now)
        stop_s = self.wait_station(stop_s)
        seen = self.visible_upstream(lidar_xy)
        frontier = -seen
        self.queue_caps(bodies)
        best = commit = None
        blocked = []
        standing = s >= stop_s-.5 and v < .3
        for v_line, wait in self.candidates(v_cap):
            if standing and v_line > 0.:
                continue     # standing at the wait station: launch at full pedal (a wait), never creep in
            p = self.profile(s, v, v_cap, stop_s, v_line, wait, bodies)
            hit = self.conflict(p, bodies, None)
            if hit is not None and hit[1] != 'horizon':
                blocked.append((round(v_line, 1), wait, round(hit[0], 1), hit[1]))
                continue
            if best is None:
                best = (v_line, wait)     # past the horizon: no observed car against it yet
            # A stop-at-the-line plan commits only standing there: committed on the move, it was driven through the
            # line without the stop (commit_speed has no stop) into the car it waited for (2026-10-02 settle-6 run).
            standing = s >= stop_s-.5 and v < .3
            # A commit must not rely on slowing down afterwards: committed at 23 km/h on a 2 m/s line plan, the car
            # did not shed the speed in time (ring brake shaping, tracker response) and met the car it was to let by
            # (2026-10-02 settle-10 run). The same entry kept at our present speed must clear as well.
            keep = p if v_line >= v else self.profile(s, v, v_cap, stop_s, max(v_line, v), 0., bodies)
            if (hit is None and wait == 0. and (v_line > 0. or standing) and self.conflict(p, bodies, frontier) is None
                    and (keep is p or self.conflict(keep, bodies, frontier) is None)):
                commit = (v_line, wait)
                break
            if best[1] > 0.:
                break                     # waiting at the line is the plan: commit only when its wait is 0
        base = dict(model='ring_merge', evaluated_at=now, visible_upstream_m=round(seen, 1), s_merged=self.s_merged,
                    stop_gap_m=round(stop_s-s, 2), bodies=[b.id for b in bodies], blocked=blocked[:6],
                    s_in=self.s_in, s_out=self.s_out)
        if commit is not None:
            return dict(base, decision='GO', line_mps=best[0], wait_s=best[1], commit_line_mps=commit[0],
                        reason=f'entry clears ring cars and unobserved traffic (line {commit[0]:.1f} m/s)')
        if best is not None:
            return dict(base, decision='APPROACH' if best[1] == 0. else 'WAIT', line_mps=best[0], wait_s=best[1],
                        reason=(f'line {best[0]:.1f} m/s clears observed ring cars; unobserved traffic not ruled out'
                                if best[1] == 0. else f'stop at the line, {best[1]:.1f} s wait clears the ring cars'))
        return dict(base, decision='WAIT', line_mps=0., wait_s=math.inf, reason='nothing clears the ring cars')


def stop_speed(gap, decel, latency):
    """Highest speed that still stops within gap metres (latency at speed, then decel)."""
    if gap <= 0.:
        return 0.
    return decel*(-latency+math.sqrt(latency*latency+2.*gap/decel))


def command(merge: RingMerge, result: dict, s: float, v: float, stop_s: float, v_cap: float):
    """Speed command before the entry is committed: (v_max, stop_s or None, commit, state).
    - GO where the next planning period could no longer stop at the line: commit. Its line speed holds until our
      body reaches the ring (s_in); then the ring following takes over.
    - Otherwise the earliest approach's envelope (through the line at its speed, or a stop there), never above a
      stop at the line at approach_decel_mps2: the line is passed only committed."""
    v = max(0., v)
    stop_s = merge.wait_station(stop_s)
    gap = stop_s-s-.3
    cap = stop_speed(gap, merge.approach_decel_mps2, merge.latency_s+merge.decide_ahead_s)
    decide = v >= cap-.3 or gap <= .5
    decision = result.get('decision')
    if decision == 'GO' and decide:
        return commit_speed(merge, result['commit_line_mps'], s, stop_s, v_cap), None, True, 'ROUNDABOUT_ENTER'
    line = result.get('line_mps', 0.) if decision in ('GO', 'APPROACH') else 0.
    if line > 0.:
        return min(v_cap, merge.envelope(s, stop_s, line), cap), None, False, 'ROUNDABOUT_APPROACH'
    return min(v_cap, cap, merge.envelope(s, stop_s, 0.)), (stop_s if gap <= 2. else None), False, 'ROUNDABOUT_WAIT'


def commit_speed(merge: RingMerge, line: float, s: float, stop_s: float, v_cap: float):
    """After the commit: the committed line speed until the body reaches the ring, then the rule speed."""
    stop_s = merge.wait_station(stop_s)
    if s >= merge.s_in or line <= 0.:
        return v_cap
    return min(v_cap, merge.envelope(s, stop_s, line) if s < stop_s else line)


def ring_bodies(tracks, merge: RingMerge, now, max_age=.45, extents=None) -> List[RingBody]:
    """circular_gap RingTracks -> RingBody. Speed range: the track's Kalman estimate +- kf_sigmas x its sigma
    (RingTrack.kf_update), within standing .. ring_speed_mps; before two measurements, that whole range."""
    out = []
    R = merge.circle.radius
    for tr in tracks:
        age = now-tr.event.stamp
        if not 0 <= age <= max_age or not tr.samples:
            continue
        u = wrap(tr.samples[-1][1]/R-merge.origin)*R
        if extents is not None:
            lo, hi = extents(tr, age)
        else:
            lo, hi = -tr.length/2, tr.length/2
        est = tr.kf_speed() if hasattr(tr, 'kf_speed') else None
        if est is not None:
            v, sd = est
            v_lo, v_hi = max(0., v-merge.kf_sigmas*sd), min(max(merge.ring_speed_mps, v), v+merge.kf_sigmas*sd)
            v_hi = max(v_hi, v_lo)
        else:
            v_lo, v_hi = 0., merge.ring_speed_mps
        # extents are about the track's last sample; move it to now at the low speed (the high speed adds its own)
        out.append(RingBody(str(tr.event.value), u+v_lo*age, -lo, hi+(v_hi-v_lo)*age, v_lo, v_hi))
    return out
