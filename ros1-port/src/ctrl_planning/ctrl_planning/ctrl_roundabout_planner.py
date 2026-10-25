"""Roundabout entry, rebuilt (user 2026-10-02): get into a gap from a standstill, and while approaching find the
speed that meets a gap and drive it.

Input: the ring lane's runs from the current scan (roundabout_boxes.RingLane: occupied stretches of the arc, their
edges followed by the highway's edge Kalman filter). No tracker ids, no tracker speeds.

Each plan:
  bodies   every run with its speed range (edge Kalman estimate +- kf_sigmas sigma, within lead_floor .. ring_speed:
           the car ahead is taken to keep moving), and the unseen stretch upstream (past the sight, or in a shadow) as
           a body that may come at ring speed;
  motions  our own candidate motions to the ring: approach at a speed (through the wait station at it), or stand at the
           wait station and launch after a wait; launch, accel and brake as measured;
  fit      a motion fits when, until our body is wholly in the lane (s_merged), no body ahead comes within margin of our
           front and no body behind within margin + its braking room to our speed of our rear;
  choice   the earliest fitting motion. Until the point where a stop at the wait station is no longer possible, its
           speed is driven (never above a stop there); there the entry commits if it fits -- also kept at our present
           speed, as nothing after the commit may rely on slowing down. Committed, the speed keeps behind the car ahead
           (gap keeping) until our body has left the ring lane.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from .ctrl_roundabout_boxes import FREE, RingLane, Run, UNKNOWN, wrap


@dataclass
class Body:
    name: str
    lo: float          # rear edge u now
    hi: float          # front edge u now
    v_lo: float
    v_hi: float

    def at(self, t, fast):
        v = self.v_hi if fast else self.v_lo
        return self.lo+v*t, self.hi+v*t, v


class RoundaboutGeometry:
    """Our body against the ring lane along the route: arc interval per rear-axle s while it overlaps the lane band,
    first contact (s_in), wholly in the lane (s_merged), clear of it (s_out); curve speeds."""

    def __init__(self, circle, route, s_offset, s_from, s_to, lane_half_m=1.75, a_lat_mps2=4.5, step=.25):
        self.circle, self.step = circle, step
        self.route, self.s_offset = route, s_offset
        c, R = np.asarray(circle.center, dtype=float), circle.radius
        self.ss = np.arange(s_from, s_to+1e-9, step)
        rows = []
        for s in self.ss:
            x, y = route.at(s+s_offset); h = route.heading_at(s+s_offset)
            co, si = math.cos(h), math.sin(h)
            cx, cy = x+1.4825*co, y+1.4825*si
            pts = np.array([(cx+u*co-v*si, cy+u*si+v*co) for u in (-2.3175, 0., 2.3175) for v in (-.946, 0., .946)])
            r = np.hypot(pts[:, 0]-c[0], pts[:, 1]-c[1])-R
            ang = np.arctan2(pts[:, 1]-c[1], pts[:, 0]-c[0])*circle.direction
            rows.append((r.min(), r.max(), ang))
        inside = np.array([lo <= lane_half_m and hi >= -lane_half_m for lo, hi, _ in rows])
        first = int(np.argmax(inside))
        self.ok = bool(inside.any())
        self.s_in = float(self.ss[first])
        self.origin = float(np.min(rows[first][2]))
        within = [i for i in range(first+1, len(rows)) if rows[i][0] >= -lane_half_m and rows[i][1] <= lane_half_m]
        self.s_merged = float(self.ss[within[0]]) if within else self.s_in+10.
        last = len(inside)-1-int(np.argmax(inside[::-1]))
        self.s_out = float(self.ss[last])+step
        self.lo = np.full(len(self.ss), np.nan); self.hi = np.full(len(self.ss), np.nan)
        for i, (_, _, ang) in enumerate(rows):
            if inside[i]:
                u = np.array([wrap(a-self.origin) for a in ang])*R
                self.lo[i], self.hi[i] = u.min(), u.max()
        k = np.array([abs(wrap(route.heading_at(s+s_offset+1.)-route.heading_at(s+s_offset-1.)))/2. for s in self.ss])
        self.vcurve = np.sqrt(a_lat_mps2/np.maximum(k, 1e-4))

    def index(self, s):
        return min(max(int(round((s-self.ss[0])/self.step)), 0), len(self.ss)-1)

    def interval(self, s):
        i = self.index(s)
        return None if math.isnan(self.lo[i]) or s < self.ss[0] else (self.lo[i], self.hi[i])

    def curve_speed(self, s):
        i = self.index(s)
        return float(np.min(self.vcurve[i:i+8]))


@dataclass
class Params:
    ring_speed_mps: float = 22.5/3.6   # ring cars' top speed (9 laps 2026-10-01: p99 22.0, max 22.5 km/h)
    lead_floor_mps: float = 5.2/3.6    # the car ahead keeps moving (user); ring cars >= 5.2 km/h 95 % of the time
    kf_sigmas: float = 3.9             # speed range = estimate +- this x sigma (|error|/sigma p99 3.87, ring KF calibration)
    car_length_m: float = 5.2          # a hidden car end: up to an ordinary car length (highway car_max_length_m)
    margin_m: float = .6               # edge position noise (EdgeTracks POS_SIGMA 0.15 m) x kf_sigmas; the 0.5 m boxes already
                                       # round each run end outward. 1 m on top of 1 m boxes left no 6-8 m gap (2 s ring)
    rear_brake_mps2: float = 2.5       # ring cars braked at most 2.49 m/s2 over 0.5 s (p0.1, 36326 windows)
    # Full-pedal launch from the wait station, 5 clean launches 2026-10-02 (roundabout-rebuild runs, truth): +0.5/1.0/
    # 1.5/2.0 s -> 5-8 / 15-17 / 21-25 / 25-27 km/h, 0.3 / 2.0 / 4.9 / 7.7 m. Fit: delay 0.17 s, 5.5 m/s2 for the
    # first second, 3.9 after (the 9-29 departure_profile model, 0.2 s / 5.2 / 3.5, ran 0.3-0.6 m behind).
    launch_delay_s: float = .17
    launch_accel_mps2: float = 5.5
    launch_accel_s: float = 1.
    accel_mps2: float = 3.9
    brake_mps2: float = 4.             # behavior.yaml late_brake_mps2: the approach keeps a stop within this
    latency_s: float = .6              # command latency + one 0.2 s planning period (planner runs at 5 Hz)
    stand_m: float = 1.5               # stopped this close to the wait station counts as standing there (the stop lands
                                       # 0.2-0.8 m short of it); motion, plan and speed share it
    step_s: float = .1
    horizon_s: float = 12.
    speeds_mps: tuple = (12., 10., 8., 7., 6., 5., 4., 3.)   # measured probes (roundabout_motion.json)
    # Filled from roundabout_motion.json 'measured' (Params.measured); the values above are their fallbacks.
    ring_typical_mps: float = 5.73     # ring cars near our entry, p50 (measured)
    ring_car_width_m: float = 2.26     # widest ring car (truth)
    follow_gap_err: tuple = ()         # ((s_lo, s_hi, p99 m)): how much the measured gap to the car ahead runs long
    lead_speed_by_est: tuple = ()      # ((est_lo, est_hi, p50 of the truth mean speed over the next 3.5 s))
    nominal: bool = False              # ring cars at their typical speed given the estimate (p50), not p1..p99
    speed_by_est: tuple = ()           # ((est_lo, est_hi, truth p1, truth p99)) for sigma within speed_err[0][0]
    speed_err: tuple = ()              # ((sigma_max, p1, p99) of truth - edge estimate), m/s
    edge_ext_rear_m: float = 0.        # a run's rear when its rear is out of view: p99 of run rear - truth rear bumper
    edge_ext_front_m: float = 0.       # ...front: -p1 of run front - truth front bumper
    stop_table: tuple = ()             # ((speed m/s, stop distance m)) with v_max dropped to 0, worst per speed
    plan_period_s: float = .2          # the planner's own period: a stop decided now starts up to this late

    @classmethod
    def measured(cls, table):
        m = (table or {}).get('measured', {})
        p = cls()
        for k, v in m.items():
            if hasattr(p, k):
                setattr(p, k, tuple(tuple(x) for x in v) if isinstance(v, list) and v and isinstance(v[0], list)
                        else tuple(v) if isinstance(v, list) else v)
        return p
    waits_s: tuple = tuple(np.arange(0., 10.01, .25))


@dataclass
class Plan:
    decision: str                  # GO (commit now) / APPROACH (drive approach_mps) / WAIT (stop at the wait station)
    approach_mps: float = 0.
    commit: bool = False
    reason: str = ''
    wait_s: float = math.inf
    launch_mps2: float = math.inf  # standing: the launch strength (v_max ramp, measured; inf = full pedal)
    bodies: list = field(default_factory=list)
    blocked: list = field(default_factory=list)


class RoundaboutPlanner:
    def __init__(self, geometry: RoundaboutGeometry, lane: RingLane, wait_s_line: float, v_cap: float,
                 params: Params = None, motion_table: dict = None):
        self.g, self.lane, self.p = geometry, lane, params or Params.measured(motion_table)
        # Our own entry motion as measured (config/roundabout_motion.json: no-NPC entries at fixed approach speeds):
        # per approach speed the (t, s, v) samples, and from probe 0 the launch from a standstill at the station.
        self.approach_table, self.launch_table = {}, None
        if motion_table:
            for key, row in motion_table.get('probes', {}).items():
                S = np.asarray(row['samples'], dtype=float)
                if float(key) > 0.:
                    self.approach_table[float(key)] = S
                else:
                    still = np.where((S[:, 1] > S[0, 1]+20.) & (S[:, 2] < .1))[0]
                    if len(still):
                        L = S[still[-1]:]
                        self.launch_table = np.c_[L[:, 0]-L[0, 0], L[:, 1]-L[0, 1], L[:, 2]]
        # Launches by strength: full pedal, then the measured v_max ramps (FOSCAR_RB_LAUNCH_A probes), strongest first.
        # A softer launch now can tuck in behind a car that a full-pedal launch would have to wait for.
        self.launches = [(math.inf, self.launch_table)] if self.launch_table is not None else []
        for key, row in sorted((motion_table or {}).get('launches', {}).items(), key=lambda kv: -float(kv[0])):
            self.launches.append((float(key), np.asarray(row['samples'], dtype=float)))
        # Full-pedal launch from the new stop line as measured, t from the GO command (launch delay inside).
        line = (motion_table or {}).get('launch_line')
        self.line_launch = np.asarray(line['samples'], dtype=float) if line else None
        self._line_s = float(line['s0'])-.25 if line else None      # launch logged at the rear axle +0.25
        self.v_cap = v_cap
        self.stop_s = self.wait_station(wait_s_line)
        self.committed = False
        self.last = None

    # ------------------------------------------------------------------ our motion
    def wait_station(self, line_s):
        """Wait back from the ring so a launch reaches ring speed at first contact: entering at ring speed the car
        behind need not brake for us (standing at the line we met the ring at ~2 m/s and waited 30 s, 2026-10-02)."""
        p = self.p
        a1 = p.launch_accel_mps2*p.launch_accel_s
        if p.ring_speed_mps <= a1:
            d = p.ring_speed_mps**2/(2*p.launch_accel_mps2)
        else:
            d = a1**2/(2*p.launch_accel_mps2)+(p.ring_speed_mps**2-a1**2)/(2*p.accel_mps2)
        return min(line_s, self.g.s_in-d)

    def stop_dist(self, v):
        """Measured distance to a standstill from v once v_max drops (stop_table; past it the top stop's
        deceleration), fallback latency + brake_mps2."""
        tab = self.p.stop_table
        if not tab:
            return v*self.p.latency_s+v*v/(2*self.p.brake_mps2)
        vs = [0.]+[x for x, _ in tab]; ds = [0.]+[d for _, d in tab]
        if v <= vs[-1]:
            return float(np.interp(v, vs, ds))
        return ds[-1]+(v*v-vs[-1]**2)/(2*vs[-1]**2/(2*ds[-1]))

    def follow_speed(self, gap, v_lead):
        """Fastest speed from which we still brake to v_lead within gap (measured stops, one planning period late)."""
        if not hasattr(self, '_follow_v'):
            self.follow([(0., 0., 0.)], [])
        if gap <= 0.:
            return 0.
        return float(np.interp(gap+self.stop_dist(v_lead), self._follow_d, self._follow_v))

    def front_arc(self, s):
        """Our front's arc on the ring lane; before we reach the lane, the entry's arc less the way still to go."""
        iv = self.g.interval(s)
        if iv is not None:
            return iv[1]
        if not hasattr(self, '_entry_iv'):
            s_first = next((x for x in np.arange(self.g.s_in-5., self.g.s_merged, .1) if self.g.interval(x) is not None),
                           self.g.s_in)
            self._entry_iv = (s_first, self.g.interval(s_first))
        s_first, iv0 = self._entry_iv
        return (iv0[1] if iv0 is not None else 0.)-(s_first-s)

    def follow(self, motion, bodies, typical=False):
        """Our motion as we drive it: the launch / approach as measured, but once on the lane behind a car (its slow
        case) no faster than lets us brake to its speed in the gap (measured stops, one planning period late) --
        full pedal in, then hold behind the car ahead. Deceleration no harder than the hardest measured stop; where
        that is not enough we run into it and fits says so."""
        if not hasattr(self, '_follow_v'):
            V = np.arange(0., 25., .05)
            self._follow_v = V
            self._follow_d = np.array([self.stop_dist(x)+x*self.p.plan_period_s for x in V])
            tab = self.p.stop_table
            self._max_decel = max(x*x/(2*d) for x, d in tab) if tab else self.p.brake_mps2
        ahead = [b for b in bodies if b.name != 'unseen']
        out = [motion[0]]
        for (t0, s0, v0), (t1, s1, v1) in zip(motion, motion[1:]):
            dt = t1-t0
            if dt <= 0.:
                continue
            tp, sp, vp = out[-1]
            v = min(v1, vp+max(0., v1-v0))           # as measured, never quicker than the table's own rise
            iv = self.g.interval(sp)
            if iv is not None and sp < self.g.s_merged+30.:
                mid = (iv[0]+iv[1])/2
                for b in ahead:
                    lo, _, vl = b.at(t1, False)
                    if typical:
                        vm = getattr(b, 'v_mid', (b.v_lo+b.v_hi)/2)
                        lo, vl = b.lo+vm*t1, vm
                    if lo >= mid:
                        gap = lo-iv[1]
                        v_ok = float(np.interp(gap+self.stop_dist(vl), self._follow_d, self._follow_v)) if gap > 0. else vl
                        v = min(v, max(v_ok, vl if gap > 0. else 0.))
            v = max(v, vp-self._max_decel*dt, 0.)
            free = abs(v-v1) < 1e-6 and abs(vp-v0) < 1e-6
            out.append((t1, sp+((s1-s0) if free else (vp+v)/2*dt), v))
        return out

    def stop_speed(self, s):
        """Highest speed from which we still stop at the wait station: the measured stopping distance (stop_table:
        v_max dropped to 0, worst per speed) plus one planning period at that speed; without the table, latency and
        brake_mps2."""
        gap = self.stop_s-s
        if gap <= 0.:
            return 0.
        tab = self.p.stop_table
        if tab:
            vs = [0.]+[v for v, _ in tab]; ds = [0.]+[d for _, d in tab]
            a_top = vs[-1]**2/(2*ds[-1])          # past the measured speeds: the top measured stop's deceleration

            def dist(v):
                return float(np.interp(v, vs, ds)) if v <= vs[-1] else ds[-1]+(v*v-vs[-1]**2)/(2*a_top)
            best = 0.
            for v in np.arange(0., 40., .1):
                if dist(v)+v*self.p.plan_period_s > gap:
                    break
                best = float(v)
            return best
        a, L = self.p.brake_mps2, self.p.latency_s
        return a*(-L+math.sqrt(L*L+2*gap/a))

    def motion(self, s0, v0, approach, wait, launch=math.inf):
        """[(t, s, v)] until our body leaves the lane band or the horizon. approach > 0: cruise and brake toward the
        wait station so as to pass it at `approach`, then on to the ring and up to curve / rule speed. approach 0:
        stop at the wait station, stand `wait` s, launch at full pedal."""
        p, g = self.p, self.g
        measured = self.measured_motion(s0, v0, approach, wait, launch)
        if measured is not None:
            return measured
        out, s, v, t = [], s0, max(0., v0), 0.
        held = None
        launch = None
        while t <= p.horizon_s:
            out.append((t, s, v))
            if s >= g.s_out:
                break
            if approach <= 0.:
                if held is None and s >= self.stop_s-p.stand_m and v < .2:
                    held = t
                if held is None:
                    goal = math.sqrt(2*p.brake_mps2*max(0., self.stop_s-s))
                elif t < held+wait:
                    goal = 0.
                else:
                    goal = self.v_cap
            elif s < self.stop_s:
                goal = math.sqrt(approach**2+2*p.brake_mps2*(self.stop_s-s))
            elif s < g.s_in:
                goal = max(approach, v)      # past the station nothing slows us before the ring
            else:
                goal = self.v_cap
            goal = min(goal, self.v_cap, g.curve_speed(s))
            if v < goal:
                if v < .1 and launch is None:
                    launch = t+p.launch_delay_s
                if t >= (launch if launch is not None else t):
                    a = p.launch_accel_mps2 if launch is not None and t < launch+p.launch_accel_s else p.accel_mps2
                    v = min(goal, v+a*p.step_s)
            else:
                launch = None
                v = goal if goal <= 0. and v < p.brake_mps2*p.step_s else max(goal, v-p.brake_mps2*p.step_s)
            s += v*p.step_s
            t += p.step_s
        return out

    def measured_motion(self, s0, v0, approach, wait, launch=math.inf):
        """The motion from the measured table when it covers this state, else None (then the kinematic model).
        Standing at the station: stand `wait` + launch delay, then the measured launch. Approaching: the probe at this
        approach speed from where it passed s0, if it was then going about our speed (within 1.5 m/s or 20 %)."""
        p, g = self.p, self.g
        horizon = p.horizon_s
        if approach <= 0.:
            table = dict(self.launches).get(launch)
            if table is None or not (s0 >= self.stop_s-p.stand_m and v0 < .2):
                return None
            hold = wait+p.launch_delay_s
            out = [(t, s0, 0.) for t in np.arange(0., hold, p.step_s)]
            out += [(hold+dt, s0+ds, v) for dt, ds, v in table if hold+dt <= horizon]
            return out
        S = self.approach_table.get(approach)
        if S is None:
            return None
        i = int(np.searchsorted(S[:, 1], s0))
        if i >= len(S) or abs(S[i, 2]-v0) > max(1.5, .2*v0):
            return None
        return [(tk-S[i, 0], sk+(s0-S[i, 1]), vk) for tk, sk, vk in S[i:] if tk-S[i, 0] <= horizon]

    # ------------------------------------------------------------------ the ring
    def bodies(self, s_ego, v_ego):
        """Runs as bodies, plus the unseen upstream stretch; speeds from the edge filter (Params)."""
        p = self.p
        out = []
        sensor = getattr(self.lane, 'sensor_xy', None)
        for i, r in enumerate(self.lane.runs):
            # what it may be going: its own measured estimate, else standing to the ring's posted limit (user
            # 2026-10-09: no practice tables or typical speeds)
            v_lo, v_hi = self.speed_range(r)
            v_mid = None
            lo, hi = r.lo, r.hi
            # The lidar sees one end of a ring car: the run falls short of the hidden end by up to a car length
            # (measured edge errors). Facing its front (we are ahead of it along the ring), its rear is hidden.
            if sensor is not None and (p.edge_ext_rear_m or p.edge_ext_front_m):
                mid = (lo+hi)/2
                xy = self.lane.xy_of(mid); xy2 = self.lane.xy_of(mid+1.)
                tang = (xy2-xy)/max(1e-6, float(np.hypot(*(xy2-xy))))
                if float((np.asarray(sensor)-xy)@tang) >= 0.:
                    lo -= p.edge_ext_rear_m
                else:
                    hi += p.edge_ext_front_m
            else:
                if r.lo_hidden:
                    lo = min(lo, hi-p.car_length_m)
                if r.hi_hidden:
                    hi = max(hi, lo+p.car_length_m)
            b = Body(f'run{i}@{lo:.0f}', lo, hi, v_lo, v_hi)
            b.v_mid = min(max(v_mid, v_lo), v_hi) if v_mid is not None else (v_lo+v_hi)/2
            out.append(b)
        # Hidden stretches (highway_boxes' rule): a shadow behind a car is a body over that stretch moving with the car
        # that casts it (the nearest run ahead of it); past the sight, upstream, a stream may come at ring speed (it
        # cannot pass a car we see: fits). One unseen body over every shadow up to -1000 m held a 20 m gap stream off
        # for 12 s (2026-10-02 far-approach run 10:44).
        stretches = []
        for u0, u1, state, key in self.lane.boxes:
            if state != UNKNOWN or any(b.lo <= u0 < b.hi for b in out):
                continue
            kind = 'range' if key in ('RANGE', 'ISLAND') else 'shadow'
            if stretches and stretches[-1][2] == kind and abs(stretches[-1][1]-u0) < 1e-6:
                stretches[-1][1] = u1
            else:
                stretches.append([u0, u1, kind])
        for u0, u1, kind in stretches:
            if kind == 'range':
                if u1 <= 0.:
                    out.append(Body('unseen', -1e3, u1, p.ring_speed_mps, p.ring_speed_mps))
                continue
            caster = min((b for b in out if b.lo >= u1-1e-6 and b.name != 'unseen'), key=lambda b: b.lo, default=None)
            v_lo, v_hi = (caster.v_lo, caster.v_hi) if caster is not None else (p.lead_floor_mps, p.ring_speed_mps)
            out.append(Body(f'shadow@{u0:.0f}', u0, u1, v_lo, v_hi))
        return out

    def fits(self, motion, bodies):
        """None if the motion fits, else (t, body) of the first clash (module docstring). The unseen stretch cannot
        pass a car we see upstream (one lane)."""
        p, g = self.p, self.g
        seen = [b for b in bodies if b.name != 'unseen']
        # Behind the car ahead we hold its speed: for the car behind us, at its typical speed (p50 given the
        # estimate); for the car ahead, at its slowest (p1) -- we must be able to stay off it whatever it does.
        motion, slow = self.follow(motion, bodies, typical=True), self.follow(motion, bodies)
        merged_checked = False
        for (t, s, v), (_, s_slow, _) in zip(motion, slow):
            iv = g.interval(s)
            if iv is None:
                continue
            iv_slow = g.interval(s_slow) or iv
            if s >= g.s_merged:
                if merged_checked:
                    continue          # merged: the gap keeping has the car ahead; the one behind follows us
                merged_checked = True
            e_lo, e_hi = iv
            mid = (e_lo+e_hi)/2
            for b in bodies:
                lo_s, hi_s, _ = b.at(t, False)
                lo_f, hi_f, v_f = b.at(t, True)
                if b.name == 'unseen':
                    hi_f = min([hi_f]+[x.at(t, True)[0] for x in seen if x.lo >= b.hi-1e-6])
                if hi_f <= mid:                                  # behind us
                    room = p.margin_m+max(0., v_f*v_f-v*v)/(2*p.rear_brake_mps2)
                    if e_lo-hi_f < room:
                        return t, b.name
                elif lo_s >= mid:                                # ahead of us
                    if s < g.s_merged and lo_s-iv_slow[1] < p.margin_m:
                        return t, b.name
                else:
                    return t, b.name                             # beside us
        if motion[-1][1] < g.s_out:
            return motion[-1][0], 'horizon'
        return None

    # ------------------------------------------------------------------ decision
    def plan(self, s, v):
        p = self.p
        bodies = self.bodies(s, v)
        standing = s >= self.stop_s-p.stand_m and v < .3
        cands = [] if standing else [(min(a, self.v_cap), 0., math.inf) for a in p.speeds_mps]
        # standing: the soonest launch, the strongest that fits at that wait
        launches = [a for a, _ in self.launches] if standing and self.launches else [math.inf]
        cands += [(0., w, a) for w in p.waits_s for a in launches]
        best, blocked = None, []
        for approach, wait, launch in cands:
            m = self.motion(s, v, approach, wait, launch)
            clash = self.fits(m, bodies)
            if clash is not None and clash[1] != 'horizon':
                blocked.append((round(approach, 1), wait, round(clash[0], 1), clash[1]))
                continue
            best = (approach, wait, clash is None, launch)
            break
        if best is None:
            return Plan('WAIT', reason='no motion fits', bodies=[b.name for b in bodies], blocked=blocked[:4])
        approach, wait, checked, launch = best
        how = '' if launch == math.inf else f' at {launch:g} m/s2'
        decide = standing or v >= self.stop_speed(s)-.3
        if checked and decide and (wait == 0.):
            # nothing after the commit may rely on slowing down: our present speed must fit too
            keep = approach >= v or self.fits(self.motion(s, v, max(approach, v), 0.), bodies) is None
            if keep:
                return Plan('GO', approach_mps=approach, commit=True, reason='fits; committed'+how, launch_mps2=launch,
                            bodies=[b.name for b in bodies], blocked=blocked[:4])
        if approach > 0.:
            return Plan('APPROACH', approach_mps=approach, reason=f'pass the wait station at {approach:.1f} m/s',
                        bodies=[b.name for b in bodies], blocked=blocked[:4])
        return Plan('WAIT', wait_s=wait, reason=f'stand, launch in {wait:.2f} s'+how, launch_mps2=launch,
                    bodies=[b.name for b in bodies],
                    blocked=blocked[:4])

    def speed(self, plan: Plan, s, v):
        """(v_max, stop at wait station?) to drive this tick before the commit."""
        cap = self.stop_speed(s)
        if v < .3 and s >= self.stop_s-self.p.stand_m and plan.decision != 'APPROACH':
            return 0., True       # standing at the wait station: hold until the launch, no creeping up to it
            return min(self.v_cap, cap, math.sqrt(plan.approach_mps**2+2*self.p.brake_mps2*max(0., self.stop_s-s))), False
        return min(self.v_cap, cap), True

    def committed_speed(self, s, v):
        """Committed, not yet wholly in the lane: the fastest approach from our present state that still fits the
        cars we see and their shadows (the unseen stream past the sight is left out: it moves with our viewpoint and
        flipped commits on and off, 10:45 run). Nothing fits: before the wait station give the commit back (None) --
        we wait there as planned; past it, stop short of the ring (0). Merged: keep_gap."""
        if s >= self.g.s_in:
            return self.keep_gap(s, v), 'keep gap'      # on the ring: never stop there for a car behind
        bodies = [b for b in self.bodies(s, v) if b.name != 'unseen']
        rest = self.launch_rest(s)
        if rest is not None:
            clash = self.fits(rest, bodies)
            if clash is None or clash[1] == 'horizon':
                return math.inf, f'launch at {self.launch_mps2:g} m/s2 fits'
        # never creep in: the approach speeds (>= 3 m/s), our present speed if above them
        speeds = {min(x, self.v_cap) for x in self.p.speeds_mps}
        if v >= min(self.p.speeds_mps):
            speeds.add(v)
        for a in sorted(speeds, reverse=True):
            clash = self.fits(self.motion(s, v, a, 0.), bodies)
            if clash is None or clash[1] == 'horizon':
                return min(self.v_cap, a), f'fits at {a:.1f} m/s'
        if s < self.stop_s-.3:
            return None, 'nothing fits: back to waiting'
        return 0., 'nothing fits: stop short of the ring'

    def launch_rest(self, s):
        """The rest of a soft launch under way (launch_mps2, launch_t set by the caller), from where we are now."""
        a = getattr(self, 'launch_mps2', math.inf)
        if a == math.inf or getattr(self, 'launch_elapsed', None) is None:
            return None
        L = dict(self.launches).get(a)
        el = self.launch_elapsed-self.p.launch_delay_s
        if L is None or el > L[-1, 0]:
            return None
        i = int(np.searchsorted(L[:, 0], max(0., el)))
        return [(tk-L[i, 0], s+(sk-L[i, 1]), vk) for tk, sk, vk in L[i:] if tk-L[i, 0] <= self.p.horizon_s]

    # our body: rear axle s on the route, centre 1.4825 m ahead of it, 4.635 x 1.892 m (MORAI ego, as loop_runner)
    EGO_CENTRE, EGO_LEN, EGO_W = 1.4825, 4.635, 1.892

    @staticmethod
    def _box(cx, cy, h, length, width):
        co, si = math.cos(h), math.sin(h)
        return [(cx+a*co-b*si, cy+a*si+b*co) for a, b in ((length/2, width/2), (length/2, -width/2),
                                                          (-length/2, -width/2), (-length/2, width/2))]

    @staticmethod
    def body_gap(A, B):
        """Separation of two convex quads (0 when they overlap)."""
        best = 0.
        for P in (A, B):
            for i in range(4):
                ax = (P[(i+1) % 4][1]-P[i][1], P[i][0]-P[(i+1) % 4][0]); n = math.hypot(*ax)
                if n < 1e-9:
                    continue
                pa = [(p[0]*ax[0]+p[1]*ax[1])/n for p in A]; pb = [(p[0]*ax[0]+p[1]*ax[1])/n for p in B]
                best = max(best, min(pb)-max(pa), min(pa)-max(pb))
        return best

    def ego_box(self, s):
        g = self.g
        x, y = g.route.at(s+g.s_offset); h = g.route.heading_at(s+g.s_offset)
        return self._box(x+self.EGO_CENTRE*math.cos(h), y+self.EGO_CENTRE*math.sin(h), h, self.EGO_LEN, self.EGO_W)

    def ring_box(self, lo, hi):
        """A ring car over arc [lo, hi] as a box along the lane there."""
        mid = (lo+hi)/2
        p = self.lane.xy_of(mid); q = self.lane.xy_of(mid+.5)
        return self._box(p[0], p[1], math.atan2(q[1]-p[1], q[0]-p[0]), max(hi-lo, .5), self.p.ring_car_width_m)

    # Extra room to the lead's tail while tucking in, over the bare braking room (user 2026-10-02: 0.5 m less close),
    # until our body is this far into the lane (user: 85 %); then the ring's own following gap
    ENTRY_EXTRA_M = .4                 # user 2026-10-04: 0.5 -> 0.4 m while coming in
    IN_RING_SHARE = .85
    # the switch, by the user's eye (2026-10-02): 12 m past the new stop line (s 868.97 -> 880.97), 2.05 m past our
    # first touch of the lane; the 85 % point (s 884.8) came too late
    IN_RING_PAST_TOUCH_M = 2.05
    # On the ring the same following, with this much room (user 2026-10-02: about 1 m), grown from ENTRY_EXTRA_M over
    # RING_STANDOFF_RAMP_M of our way past the 85 % point so the gap opens gently
    # user 2026-10-02: 1 m, 2 m, then 1.5 m; 2 m again 2026-10-09: room to speed up out of the ring behind a slow car
    RING_STANDOFF_M = 2.
    RING_STANDOFF_RAMP_M = 5.

    def standoff(self, s):
        """Room kept to the lead's tail on top of the braking room, at our station s."""
        s85 = self.in_ring_s()
        if s < s85:
            return self.ENTRY_EXTRA_M
        f = min(1., (s-s85)/self.RING_STANDOFF_RAMP_M)
        return self.ENTRY_EXTRA_M+f*(self.RING_STANDOFF_M-self.ENTRY_EXTRA_M)

    def launch_accel(self, v):
        """Full-pedal acceleration at speed v, from the measured launch from the new stop line."""
        if not hasattr(self, '_acc_v'):
            L = self.line_launch
            a = np.gradient(L[:, 2], L[:, 0])
            k = int(np.argmax(L[:, 2]))                    # up to the launch's top speed
            vs, acc = L[:k+1, 2], np.maximum(a[:k+1], 0.)
            order = np.argsort(vs)
            self._acc_v = (vs[order], acc[order])
        vs, acc = self._acc_v
        return float(np.interp(v, vs, acc, right=0.))

    def in_ring_s(self):
        """Where tucking in turns into following on the ring: IN_RING_PAST_TOUCH_M past our first touch of the lane."""
        return self.g.s_in+self.IN_RING_PAST_TOUCH_M

    def share_in_lane_s(self):
        """Our station once IN_RING_SHARE of our body is inside the lane band."""
        if not hasattr(self, '_in_ring_s'):
            g = self.g; c = np.asarray(g.circle.center, dtype=float); R = g.circle.radius
            self._in_ring_s = g.s_merged
            for s in np.arange(g.s_in, g.s_merged+1e-9, .05):
                x, y = g.route.at(s+g.s_offset); h = g.route.heading_at(s+g.s_offset)
                co, si = math.cos(h), math.sin(h); cx, cy = x+self.EGO_CENTRE*co, y+self.EGO_CENTRE*si
                P = np.array([(cx+u*co-v*si, cy+u*si+v*co) for u in np.linspace(-self.EGO_LEN/2, self.EGO_LEN/2, 11)
                              for v in np.linspace(-self.EGO_W/2, self.EGO_W/2, 5)])
                if np.mean(np.abs(np.hypot(P[:, 0]-c[0], P[:, 1]-c[1])-R) <= self.lane.lane_half) >= self.IN_RING_SHARE:
                    self._in_ring_s = float(s)
                    break
        return self._in_ring_s

    # The ring's posted limit (map max_speed of the ring links, set by the behaviour node): the most a car we have not
    # measured yet can be doing. Never an average or a practice table (user 2026-10-09: what the cars in the final will
    # do is not known). inf until set: an unmeasured car behind us then rules every go out.
    ring_limit_mps = math.inf

    def measured(self, run):
        """Is this car's speed measured (an estimate within SURE_SD)?"""
        e = self.estimate(run) if run is not None else None
        return e is not None and e[1] <= self.SURE_SD

    def speed_range(self, run):
        """(slowest, fastest) this car may be going: its own estimate's sigma band once measured; before that anything
        from standing to the ring's posted limit (user 2026-10-09: no average speeds)."""
        if self.measured(run):
            v, sd = self.estimate(run)
            return max(0., v-sd), v+sd
        return 0., self.ring_limit_mps

    def est_speed(self, run, fallback):
        """Its measured estimate itself (no sigma side), else `fallback` (user 2026-10-09: the judge on the gap behind the
        lead took the lead slow and the car behind fast and gave up gaps that fitted by 0.5-0.85 s; gap_check already
        takes the estimate itself, GAP_MIN_M / REAR_MIN_M holding its error)."""
        return self.estimate(run)[0] if self.measured(run) else fallback

    def rear_speed(self, run):
        """The car behind the gap: the fast side of what it may be going -- a faster one is the danger."""
        return self.speed_range(run)[1]

    def rear_sure(self, run):
        """Is this car's speed measured well enough to act on? An estimate a ring car can have, within the measured
        sigma band, from this scan (not carried) and its front in the open (not cut off by a shadow)."""
        if run is None or self.sane(run) is None or run.coasted > 0. or run.hi_hidden:
            return False
        return bool(self.p.speed_err) and run.speed[1] <= self.p.speed_err[0][0]

    def rear_assumed(self, run):
        """The speed a car behind us is taken at: the fast side of what it may be going (no floor: user 2026-10-09,
        no fixed speeds)."""
        return self.rear_speed(run)

    def tuck_path(self, s0, tau, lead, vx, runs, age=0., rear=None, bare=False, v0=0.):
        """Our way in launching tau s from now, driven as we drive it (user 2026-10-02: follow while coming in): the
        measured full-pedal launch, from the start no faster than lets us brake to the lead's speed in the gap along
        the ring (our way in continued onto it before the lane; measured stops; the gap's measured error off and, until
        wholly in, ENTRY_EXTRA_M) and no harder braking than the hardest measured stop; no body touching the lead's,
        every other ring car's body clear of ours before the lane. None if that runs into anything; else (t, s, v, gap to the lead's tail) as we
        are wholly in the lane."""
        L = self.line_launch
        self.tuck_why = None
        if not hasattr(self, '_follow_v'):
            self.follow([(0., 0., 0.)], [])
        others = [r for r in runs if r is not lead]
        t_p, s_p, v_p = 0., s0, float(v0)                  # v0: already rolling (re-checked on the way in)
        samples = [(0., s0, float(v0))]
        # The launch table only lends its time steps (speeds come from launch_accel and the following): continued past
        # its end at its last step up to the planning horizon -- cut at the table's 5.9 s, following a 10 km/h lead in
        # read as no way in until we were at the line (user 2026-10-09; the typical 5.73 m/s lead had hidden it)
        dt_end = float(L[-1, 0]-L[-2, 0])
        times = list(L[1:, 0])+list(np.arange(L[-1, 0]+dt_end, self.p.horizon_s+1e-9, dt_end))
        for t_k in times:
            dt = t_k-t_p
            if dt <= 0.:
                continue
            moved = vx*(age+tau+t_k)
            # speed from the gap along the ring, our way in continued onto it (merge timing: the lead crossing in front
            # moves out of our way, its body gap says nothing about closing); bodies only to rule out a touch
            arc = lead.lo+moved-self.front_arc(s_p) if lead is not None else math.inf    # None: nobody ahead
            # full pedal: the launch's acceleration at the speed we are doing (measured), not at the time since the
            # launch -- held back by the lead a moment, the time-indexed rise had already flattened out
            v = v_p+self.launch_accel(v_p)*dt
            if s_p >= self.g.s_in or arc > 0.:
                # Before the lane our way in only stands in for the ring: the lead still behind that stand-in has not
                # crossed our entry yet and is no reason to wait (it held every launch until its tail passed our
                # stand-in nose, 10 m back: late, then full pedal on the ring to catch up, 20:36 runs).
                # bare: only the real danger (no measured margin, no room kept) -- checking a launch already planned
                gap = arc if bare else arc-self.gap_margin(s_p)-self.standoff(s_p)
                v = min(v, self.follow_speed(gap, vx))
            # the entry curve slows us as the path follower does (user 2026-10-03: fast entries are candidates too,
            # predicted as driven -- left out, a go at 15 m/s counted on 15 m/s to s 864 while we were down to 4 m/s
            # there and the car behind the lead came in first, 17:40 settle 3)
            v = min(v, self.curve_cap(s_p))
            v = max(v, v_p-self._max_decel*dt, 0.)
            s_k = s_p+(v_p+v)/2*dt
            if lead is None:
                pass
            elif s_k < self.g.s_in:
                if self.body_gap(self.ego_box(s_k), self.ring_box(lead.lo+moved, lead.hi+moved)) <= 0.:
                    self.tuck_why = 'lead_body'
                    return None                             # before the lane only bodies can touch
            elif self.lead_gap(s_k, lead, moved) <= 0.:
                self.tuck_why = 'lead_gap'
                return None
            if s_k < self.g.s_in:
                pass
            elif rear is not None:
                # The gap's rear car (user 2026-10-02, cut-ins that made it brake hard were followed by ring cars
                # hitting each other): its front stays behind our tail, and coming faster it needs no harder braking
                # than ring cars do (measured rear_brake_mps2)
                iv = self.g.interval(s_k)
                if iv is not None:
                    vr = self.rear_assumed(rear)
                    room = iv[0]-(rear.hi+vr*(age+tau+t_k))
                    if room <= 0. or (vr > v and room < (vr*vr-v*v)/(2*self.p.rear_brake_mps2)):
                        self.tuck_why = 'rear'
                        return None
            if s_k < self.g.s_in and not self.clear_of_all(s_k, age+tau+t_k, others):
                self.tuck_why = 'crossing'
                return None
            samples.append((float(t_k), float(s_k), float(v)))
            if s_k >= self.g.s_merged:
                self.path_samples = samples
                return t_k, s_k, v, (lead.lo+moved-self.front_arc(s_k) if lead is not None else math.inf)
            t_p, s_p, v_p = t_k, s_k, v
        return None

    def lead_gap(self, s, lead, moved=0.):
        """Gap from our front to the lead's tail (moved on `moved` m): along the ring, and until we are wholly in the
        lane also body to body -- coming in at 30-40 deg our front corner met its rear corner with 1 m left along
        the ring (16:37 runs)."""
        gap = lead.lo+moved-self.front_arc(s)
        if s < self.g.s_merged:
            gap = min(gap, self.body_gap(self.ego_box(s), self.ring_box(lead.lo+moved, lead.hi+moved)))
        return gap

    def clear_of_all(self, s, t, runs):
        """Our body at s clear of every ring car moved on t s at its typical speed (before the lane)."""
        # body to body a touch is a touch: the measured gap error (negative on the lane) never lets bodies overlap
        # (with it a margin of -0.53 m switched this check off, 2026-10-02)
        me = self.ego_box(s); m = max(0., self.gap_margin(self.g.s_in))
        for r in runs:
            for v in set(self.speed_range(r)):      # crossing our way: anything it may be going
                if self.body_gap(me, self.ring_box(r.lo+v*t, r.hi+v*t)) <= m:
                    return False
        return True

    def launch_clear(self, elapsed, s, lead, age=0.):
        """Launched `elapsed` s ago, now at s: does the rest of the measured launch stay clear of every ring car's
        body (moving on at its typical speed) until we reach the lane?"""
        L = self.line_launch
        if L is None or lead is None:
            return lead is None
        i = int(np.searchsorted(L[:, 0], elapsed))
        if i >= len(L):
            return False
        vx = self.lead_speed(lead)
        for t, ds, _ in L[i:]:
            s_t = s+ds-L[i, 1]
            if s_t >= self.g.s_in:
                return True
            if not self.clear_of_all(s_t, age+t-L[i, 0], [r for r in self.lane.runs if r is not lead]):
                return False
        return True

    def real_gap(self, s, measured):
        """The likely real gap behind a measured one at our station s (measured p50 of measured - truth)."""
        row = next((b for b in self.p.follow_gap_err if b[0] <= s < b[1] and len(b) > 3), None)
        return measured-(row[3] if row else 0.)

    def gap_margin(self, s):
        """How much the gap to the car ahead may read long at our station s (measured p99, follow_gap_err); before
        the lane, the lane's first stretch (body to body there)."""
        s = max(s, self.g.s_in)
        return next((b[2] for b in self.p.follow_gap_err if b[0] <= s < b[1]), 0.)

    def sane(self, run):
        """The run's own speed estimate if it is one a ring car can have (0 .. twice the ring's top speed), else None:
        an edge filter read -17.6 m/s on a car slowing to yield (23:24 run 5)."""
        if run is None or run.speed is None or not run.speed[1] == run.speed[1]:
            return None
        v = float(run.speed[0])
        return run.speed if 0. <= v <= 2*self.p.ring_speed_mps else None

    def lead_speed(self, run):
        """The car we tuck in behind / follow: its measured estimate itself -- following, the gap is measured again every
        scan and corrects for it. Its slow side (estimate - sigma) had us crawl behind a 10 km/h car at 5-7 km/h once
        the sigmas told the truth (ACCEL_SIGMA 3; user 2026-10-09). Not measured yet: standing (we wait for it)."""
        return self.est_speed(run, 0.)

    # The lead and the car after it are marked once chosen and followed scan by scan (user 2026-10-04): found again
    # where their own measured speed puts them, their estimates collected all the way in, so the speed we act on is
    # theirs, averaged while we come up -- not a typical ring car from practice.
    SURE_SD = 1.            # an estimate this close (m/s, 1 sigma) is taken as it is; vaguer, the ring's cars now
    MARK_KEEP_S = 2.27      # a marked car out of the scans this long is let go (the measured dropout limit)
    MARK_AVG_S = 2.5        # its speed: its estimates of the last 2.5 s (one slowing to yield shows within it)

    def estimate(self, run):
        """(speed, sigma) of a car: a marked car's average, else the run's own estimate; None without a usable one."""
        for m in getattr(self, 'marks', {}).values():
            if m is not None and m.get('run') is run and m.get('avg') is not None:
                return m['avg']
        return self.sane(run)

    def mark(self, role, run):
        """Mark run as the lead / the rear car ('lead', 'rear'); None lets it go. The same car marked again keeps the
        estimates collected for it."""
        if not hasattr(self, 'marks'):
            self.marks = {}
        if run is None:
            self.marks[role] = None
            return
        m = self.marks.get(role)
        if m is not None and (m.get('run') is run or self._over(m, run) > 0.):
            m.update(run=run, t=self.lane.stamp, lo=run.lo, hi=run.hi)
        else:
            m = self.marks[role] = dict(run=run, t=self.lane.stamp, lo=run.lo, hi=run.hi, samples=[], avg=None)
        self._sample(m, run)

    def _over(self, m, run):
        """How much run overlaps where the marked car can be by now: moved on at its speed, or anything down to a
        stop (it may slow to yield), or 1 m/s faster."""
        dt = max(0., self.lane.stamp-m['t'])
        v = m['avg'][0]+1. if m.get('avg') else self.ring_limit_mps
        return min(run.hi, m['hi']+v*dt)-max(run.lo, m['lo'])

    def _sample(self, m, run):
        """Add this scan's estimate of the marked car and average the last MARK_AVG_S s (weighted by sigma; the spread
        of the estimates kept in the sigma, so a car changing speed reads vaguer)."""
        e = self.sane(run) if run.coasted == 0. else None
        S = m['samples']
        if e is not None and (not S or S[-1][0] != self.lane.stamp):
            S.append((self.lane.stamp, float(e[0]), float(e[1])))
        S[:] = [x for x in S if x[0] >= self.lane.stamp-self.MARK_AVG_S]
        if not S:
            m['avg'] = None
            return
        # an older estimate says less about the speed now: its sigma grown by what a ring car can change in that time
        # (the lane's ACCEL_SIGMA) -- the even 2.5 s average lagged a car slowing to the entry (user 2026-10-09)
        a = getattr(self.lane, 'ACCEL_SIGMA', 1.)
        w = np.array([1./max(math.hypot(sd, a*(self.lane.stamp-t)), .05)**2 for t, _, sd in S])
        v = np.array([x[1] for x in S])
        mean = float(np.sum(w*v)/np.sum(w))
        spread = float(np.sqrt(np.sum(w*(v-mean)**2)/np.sum(w)))
        m['avg'] = (mean, float(math.hypot(min(x[2] for x in S), spread)))

    def found(self, role, runs):
        """The marked car in this scan (the run overlapping where it can be; the most downstream of them, the rear car
        behind the lead), its estimate added. Lost a moment it moves on at its speed; None once lost MARK_KEEP_S."""
        m = getattr(self, 'marks', {}).get(role)
        if m is None:
            return None
        over = [r for r in runs if self._over(m, r) > 0.]
        if role == 'rear':
            lead = self.marks.get('lead')
            if lead is not None and lead.get('run') is not None:
                over = [r for r in over if r is not lead['run'] and r.hi <= lead['run'].lo+.1]
        if not over:
            dt = self.lane.stamp-m['t']
            # Carried on only at its own measured speed (user 2026-10-08): without one it was moved on at the ring's
            # average (a practice typical speed when nothing was measured) and drawn and followed where no car was. Lost.
            if dt > self.MARK_KEEP_S or not m.get('avg'):
                self.marks[role] = None
                return None
            v = m['avg'][0]
            run = Run(m['lo']+v*dt, m['hi']+v*dt, False, False, None, None, dt, m['avg'] or (v, 0.))
            m['run'] = run
            return run
        run = max(over, key=lambda r: r.hi)
        m.update(run=run, t=self.lane.stamp, lo=run.lo, hi=run.hi)
        self._sample(m, run)
        return run

    def exit_arc(self):
        """The ring arc where our body leaves the lane: a car ahead past it keeps going round, we turn off."""
        if not hasattr(self, '_exit_arc'):
            ivs = [self.g.interval(x) for x in np.arange(self.g.s_in, self.g.s_out+1e-9, .25)]
            self._exit_arc = max((iv[1] for iv in ivs if iv is not None), default=math.inf)
        return self._exit_arc

    def planned_speed(self, elapsed, s):
        """Following the launch we planned (path_samples kept at the launch): its speed now, plus closing on where it
        said we would be (1 s to catch up) when we lag it. None past its end."""
        P = getattr(self, 'go_plan', None)
        if not P or elapsed > P[-1][0]:
            return None
        ts = [p[0] for p in P]
        v = float(np.interp(elapsed, ts, [p[2] for p in P]))
        s_plan = float(np.interp(elapsed, ts, [p[1] for p in P]))
        return max(0., v+max(0., s_plan-s)/1.)

    def line_s(self):
        """The new stop line (where the measured launch from the line starts), rear-axle station."""
        L = getattr(self, '_line_s', None)
        return L if L is not None else self.stop_s

    DECIDE_SIGHT_M = 38.    # ring cars nearer than this were detected 100 % (2026-10-01 laps): decisions start and
    #                         are made on what is within it; the boxes reach further only to perceive early

    def box_dist(self, b):
        """How far the box b is from the lidar at the last scan (inf before the first)."""
        sensor = getattr(self.lane, 'sensor', None)
        if sensor is None:
            return math.inf
        return float(np.hypot(*(self.lane.xy_of((b[0]+b[1])/2)-sensor)))

    def decisive(self, s, v, age=0.):
        """Is a decision taken here for real? Only once every stretch upstream a ring car could come from before we merge
        (the ring's top speed over our time to merge) is within DECIDE_SIGHT_M. Before that a go is only provisional:
        data is gathered from the mission start on, but 30 m out we went, and gave it up 10 m on as the ring came in
        sight (user 2026-10-03)."""
        reach = self.p.ring_speed_mps*(age+self.free_motion(s, v)[-1][0])+6.
        entry = self.front_arc(self.g.s_in)
        return not any(entry-reach <= b[1] <= entry and (b[2] == UNKNOWN and b[3] == 'RANGE'
                                                         or self.box_dist(b) > self.DECIDE_SIGHT_M)
                       for b in self.lane.boxes)

    def roll_in(self, s, v, runs, age=0.):
        """Coming in at speed: can we go on in without stopping at the line? Our way in as measured (the approach probe
        about our speed); every ring car must stay either ahead of us (its tail ahead of our nose, and slow enough to
        follow by measured stops, as on the ring) or behind us (its nose behind our tail, room for it to brake to our
        speed at the ring cars' measured braking); before the lane no body may touch ours. Hidden stretches are no data
        of ours (user 2026-10-03): a go is real only once the ring we need is in sight (decisive). Stand-in cars filled
        them, up to 37 of them, and a call took 200 ms. Returns (motion, lead) or (None, None)."""
        p = self.p
        # a car that ends up behind us may come as fast as ring cars go (their measured top), or faster if we read it so:
        # a fresh estimate read 3.8 m/s on one that came at 6-7 and grazed our rear corner (23:26 run 1)
        # Every car we hold, a moment-lost one too (only cars with a speed are carried): leaving out a car lost for one
        # scan let us roll in beside it (23:29 run 1)
        # A car behind us at its measured estimate itself, REAR_MIN_M kept to our tail -- as gap_check (user 2026-10-09:
        # its fast side, estimate + a now honest sigma of 0.7 m/s, gave up at the line entries the truth cleared by
        # 1.2-3.3 m in 31 of 34 slow-ring runs); not measured yet: the ring's posted limit
        bodies = [(r, self.lead_speed(r), self.est_speed(r, self.ring_limit_mps)) for r in runs]
        # Our way in as we will drive it once going (user 2026-10-02: no slowing down then): from our speed, full pedal
        # (the measured acceleration at each speed), only the entry curve slowing us. The approach probes braked
        # toward their approach speed on the way and read us 3.5 m/s at the lane where we would come in at 7.
        m = self.free_motion(s, v)
        if m[-1][1] < self.g.s_merged:
            self.roll_why, self.roll_block = 'short', None
            return None, None
        ok, lead = self.roll_fits(m, bodies, age)
        if ok:
            self.roll_top = None
            return m, lead
        # Too fast behind the car ahead at full pedal: come in at its pace instead, held to a lower top (ring speeds);
        # full pedal only came to its tail at 7 m/s and gave up a gap we could follow into (03:31 run 1).
        # Slower is harder on the car behind: that check stands.
        top = max(q[2] for q in m)
        for cap in self.ROLL_CAPS:
            if self.roll_why != 'front':
                break
            if cap >= top:                 # the same way in as at full pedal
                continue
            mc = self.free_motion(s, v, cap=cap)
            if mc[-1][1] < self.g.s_merged:
                break
            ok, lead = self.roll_fits(mc, bodies, age)
            if ok:
                self.roll_top = cap           # driven at it too, not only judged (user 2026-10-03)
                return mc, lead
        return None, None

    ROLL_CAPS = (7., 6.5, 6., 5.5, 5., 4.5, 4.)

    def curve_cap(self, s):
        """The fastest we may be at s for the entry curve ahead: its speed (the path follower's lateral limit) with
        braking room before it (brake_mps2, the approach's braking)."""
        return min(math.sqrt(self.g.curve_speed(s+d)**2+2*self.p.brake_mps2*d) for d in np.arange(0., 15.1, 1.))

    def free_motion(self, s, v, step=.1, horizon=8., cap=math.inf):
        out = [(0., s, v)]
        t = 0.
        while t < horizon and s < self.g.s_merged+.5:
            # braking ahead of the entry curve, not at it; down to a cap at the same braking
            ahead = self.curve_cap(s)
            v1 = min(v+self.launch_accel(v)*step, ahead, self.v_cap, max(cap, v-self.p.brake_mps2*step))
            s += (v+v1)/2*step; t += step; v = v1
            out.append((t, s, v))
        return out

    def roll_fits(self, motion, bodies, age):
        g = self.g
        role = {}
        lead = None
        self.roll_why = None
        self.roll_block_run = None
        if not hasattr(self, '_follow_v'):
            self.follow([(0., 0., 0.)], [])
        for t, s, v in motion:
            if s >= g.s_merged:
                break
            T = age+t
            if s < g.s_in:
                me = self.ego_box(s)
                for r, vf, vr in bodies:
                    for vv in (vf, vr):
                        if self.body_gap(me, self.ring_box(r.lo+vv*T, r.hi+vv*T)) <= 0.:
                            self.roll_why, self.roll_block_run = 'pre', r
                            self.roll_block = (round(r.lo, 1), round(r.hi, 1), round(vv, 2), r.speed, round(t, 1), round(s, 1), round(v, 2))
                            return False, None
                continue
            iv = g.interval(s)
            if iv is None:
                continue
            mid = (iv[0]+iv[1])/2
            nose = self.front_arc(s)
            for i, (r, vf, vr) in enumerate(bodies):
                if i not in role:                      # ahead of us or behind us: decided where we first touch the lane
                    role[i] = 'front' if (r.lo+r.hi)/2+vf*T >= mid else 'rear'
                if role[i] == 'front':
                    lo = r.lo+vf*T
                    if lo <= iv[1] or v > self.follow_speed(lo-nose-self.gap_margin(s)-self.standoff(s), vf)+1e-6:
                        self.roll_why, self.roll_block_run = 'front', r
                        self.roll_block = (round(r.lo, 1), round(r.hi, 1), round(vf, 2), r.speed, round(t, 1), round(s, 1), round(v, 2))
                        return False, None
                    if lead is None or r.lo < lead.lo:
                        lead = r
                else:
                    hi = r.hi+vr*T
                    room = iv[0]-hi
                    need = (vr*vr-v*v)/(2*self.p.rear_brake_mps2) if vr > v else 0.
                    if room <= self.REAR_MIN_M or room < need:
                        self.roll_why = 'rear'
                        self.roll_block_run = r
                        self.roll_block = (round(r.lo, 1), round(r.hi, 1), round(vr, 2), r.speed, round(t, 1), round(s, 1), round(v, 2))
                        return False, None
                    # coming in at an angle our rear corner sticks out behind our tail's ring arc: bodies too until
                    # wholly in (a yielding car behind grazed our rear corner with ring room left, 23:24 run 5)
                    if s < g.s_merged and self.body_gap(self.ego_box(s), self.ring_box(r.lo+vr*T, hi)) < max(need, .5):
                        self.roll_why = 'rear'
                        self.roll_block_run = r
                        self.roll_block = (round(r.lo, 1), round(r.hi, 1), round(vr, 2), r.speed, round(t, 1), round(s, 1), round(v, 2))
                        return False, None
        return True, lead

    # The gap we squeeze into (user 2026-10-04): the tightest ring gaps we went into and came out of are the limit --
    # only gaps smaller than those are let go. Slow-ring practice (10 km/h, 1.5 s spawns): ring gaps p25 6.5, p50
    # 7.1 m; entries into 6.0-7.1 m gaps passed (16 of 19; the 3 hits were a stop at the lane edge, a wrong lead and
    # a hard-braking rear car), the tightest 6.0 m.
    GAP_MIN_M = 6.0
    # Our tail to the rear car's nose as we are wholly in the lane: the closest that entries came out of (0.57 m).
    REAR_MIN_M = .5

    def gap_check(self, lead, rear, t, s_m):
        """The gap behind the lead, for coming in t s from the scan and wholly in at s_m: None if it fits; 'small' (the
        gap is tighter than GAP_MIN_M), 'unseen' (no car behind it within GAP_MIN_M and that stretch not seen free --
        decided only on what is seen), 'rear' (the car behind, at its measured speed, within REAR_MIN_M of our tail)."""
        if rear is not None and lead.lo-rear.hi < self.GAP_MIN_M:
            return 'small'
        lo = lead.lo-self.GAP_MIN_M
        if rear is None or rear.hi < lo:
            inside = [b for b in self.lane.boxes if b[0] >= lo-1e-6 and b[1] <= lead.lo+1e-6]
            if not inside or any(b[2] != FREE for b in inside):
                return 'unseen'
        if rear is not None:
            iv = self.g.interval(s_m)
            tail = iv[0] if iv is not None else self.front_arc(s_m)-self.EGO_LEN
            # its measured speed itself, not the fast side: GAP_MIN_M / REAR_MIN_M come from entries that went well and
            # already hold the estimates' error -- the fast rear on top of the slow lead took ~3 m off every gap over a
            # 5 s launch from the line, and only 8-11 m gaps were taken (01:30 runs, user 2026-10-04)
            e = self.estimate(rear)
            if rear.hi+(e[0] if self.measured(rear) else self.ring_limit_mps)*t > tail-self.REAR_MIN_M:
                return 'rear'
        return None

    def rear_of(self, lead, runs):
        """The car behind the lead (its front nearest the lead's tail)."""
        behind = [r for r in runs if r.hi <= lead.lo+.1 and r is not lead]
        return max(behind, key=lambda r: r.hi) if behind else None

    def tuck_fits(self, s0, lead, runs, age=0., v0=0.):
        """Going on in now at our speed: behind the lead as driven (tuck_path) and into a gap that fits (gap_check)."""
        path = self.tuck_path(s0, 0., lead, self.lead_speed(lead), runs, age, None, v0=v0)
        if path is None:
            return False
        self.gap_why = self.gap_check(lead, self.rear_of(lead, runs), age+path[0], path[1])
        return self.gap_why is None

    def held_lead(self, runs):
        """The lead chosen (marked), found again in this scan; it is also the car followed after the launch, never
        re-chosen from the cars ahead (re-choosing sent us at the gap ahead of it, 2026-10-02). None once lost."""
        lead = self.found('lead', runs)
        if getattr(self, 'marks', {}).get('rear') is not None:
            self.found('rear', runs)
        return lead

    @property
    def tuck_lead(self):
        return getattr(self, 'marks', {}).get('lead')

    @tuck_lead.setter
    def tuck_lead(self, value):
        """Only ever cleared from outside: letting the lead go lets the car after it go too."""
        self.mark('lead', None)
        self.mark('rear', None)

    def tuck(self, s0, runs, step=.1, latest=10., now=None, v0=0.):
        """Standing on the stop line: the gap to go into -- behind the nearest car still coming (the lead), ahead of the car after it (the rear) -- and in how many seconds to launch (full pedal,
        measured) to come in right behind the lead: the earliest launch that keeps us, until merged, slow enough to
        brake to its speed in the gap (measured stops). The rear car is not judged yet, only predicted.
        Returns (lead, tau, prediction): lead None and tau 0 when no car is coming; tau None when no launch within
        `latest` s does. Rolling (v0 > 0, latest 0): does going on in now at our speed fit behind the lead?"""
        L = self.line_launch
        if L is None:
            return None, None, {}
        motion = [(t, s0+ds, v) for t, ds, v in L if s0+ds <= self.g.s_merged+.5]
        noses = [self.front_arc(s) for _, s, _ in motion]
        merge_nose = self.front_arc(self.g.s_merged)
        # the lead: the nearest car still coming, its tail not yet past our nose (one already crossing is gone by the
        # time a standing launch gets there)
        # Once chosen the lead is kept: its tail passes our nose before we can launch behind it, so re-choosing every
        # scan always picked the next one (3 of 5 runs never left, 15:28). It is found again where its edge-filter
        # speed puts it: the run overlapping its predicted stretch.
        age = max(0., now-self.lane.stamp) if now is not None else 0.
        # Following the car in front of us is checked first, every scan (user 2026-10-03): the car going by in front
        # (part of it past our nose, not yet past where we merge; the rearmost of such). When going now fits behind
        # it, it is the lead and the tuck's own choice is given up -- the tuck kept a car still coming and went in
        # behind the one after the car crossing in front of us, past a gap that was there.
        # Cars are told apart as seen from the line (or from where we are, once past it): further back our own nose
        # stands in far upstream on the ring, the car about to cross in front of us read as gone and the one after it
        # was taken -- held until 2 m before the line, then the go came at once (17:21 settle 6 and 7; user 2026-10-03)
        ref = max(noses[0], self.front_arc(self.line_s()))
        # The lead is the nearest car seen in this scan that has not yet passed where we merge, chosen afresh every
        # scan: a nearer one showing up is the lead at once, one that does not fit is waited out and the next nearest
        # follows (user 2026-10-04: from the new line the cars close by are the ones seen best -- held leads and let-go
        # gaps sent us to cars far upstream whose own car behind was not yet seen, 01:55 run 5)
        skip, c_why = None, None
        coming = [r for r in runs if r.lo < merge_nose and r.coasted == 0.]   # a car in this scan, not a memory
        if not coming:
            self.tuck_lead = None
            self.tuck_view = dict(state='CLEAR', lead=None, rear=None, tau=0.)
            self.tuck_diag = dict(result='clear')
            return None, 0., {}
        # Nearest first; a gap no launch can still make (too tight, or the car behind already at us -- later only
        # closer) is passed over at once for the gap behind the next car (user 2026-10-04: standing at the line we
        # sat on the gap behind a car already crossing, too late for it, until it had gone by). Rolling, only going
        # on now is judged, decided again further in: the nearest only.
        order = sorted(coming, key=lambda r: -r.lo)
        # The car go / no-go gave the go up for (it would have hit us): the tuck goes in behind it, never past it to the car
        # after it (user 2026-10-09). Let go once it has passed where we merge or is lost; then the nearest car again.
        if getattr(self, 'marks', {}).get('handoff') is not None:
            hb = self.found('handoff', runs)
            if hb is None or hb.lo >= merge_nose:
                self.mark('handoff', None)
            elif hb.coasted == 0.:
                got = self._tuck_gap(s0, hb, runs, age, latest, step, v0, c_why, skip)
                if got is not None:
                    self.mark('lead', hb)
                    self.mark('rear', self.rear_of(hb, runs))
                    return got
                self.tuck_diag['handoff'] = [hb.lo, hb.hi]
                self.tuck_lead = None
                return hb, None, {}
        if latest > 0.:
            got = self._tuck_ahead(s0, order[0], runs, age, merge_nose)
            if got is not None:
                return got
        passed = []
        for lead in (order if latest > 0. else order[:1]):
            got = self._tuck_gap(s0, lead, runs, age, latest, step, v0, c_why, skip)
            if got is not None:
                if passed:
                    self.tuck_diag['passed'] = passed
                # marked once chosen (its speed averaged scan to scan), not each car looked at on the way
                self.mark('lead', lead)
                self.mark('rear', self.rear_of(lead, runs))
                return got
            # passed over only behind a car already crossing in front of us (its front past the line reference): one
            # still coming is waited for -- passed over too, three cars went at once and the gap behind one still
            # coming (10 m) with them (02:25 run 3; user 2026-10-04)
            if not (getattr(self, 'tuck_why', None) == 'gap' and self.gap_why in ('small', 'rear') and lead.hi > ref):
                break
            passed.append([lead.lo, lead.hi, self.gap_why])
        if passed:
            self.tuck_diag['passed'] = passed
        self.tuck_lead = None
        return lead, None, {}

    def _tuck_ahead(self, s0, near, runs, age, merge_nose):
        """Standing on the line: launching now into the gap ahead of the nearest car coming (user 2026-10-09: a bus
        coming with the ring empty ahead of it held us on the line -- the tuck only ever went in behind it). The car
        coming is the gap's rear, judged at its own measured speed (no estimate yet: not decided, the tuck below
        waits); behind whatever is already past our entry, as any lead. (lead or None, 0., prediction) when it fits."""
        e = self.estimate(near)
        if e is None or near.coasted > 0.:
            return None
        # nothing unseen between the car coming and our way in (a car hidden there would come first)
        own = [(r.lo-.5, r.hi+.5) for r in runs]
        if any(b[2] != FREE and b[1] > near.hi and b[0] < merge_nose and not any(a <= b[0] and b[1] <= z for a, z in own)
               for b in self.lane.boxes):
            return None
        past = [r for r in runs if r.lo >= merge_nose]
        lead = min(past, key=lambda r: r.lo) if past else None
        vx = self.lead_speed(lead) if lead is not None else 0.
        path = self.tuck_path(s0, 0., lead, vx, runs, age, near)
        if path is None:
            self.tuck_diag = dict(result='ahead_no', why=getattr(self, 'tuck_why', None), rear=[near.lo, near.hi],
                                  rear_v=[round(e[0], 2), round(e[1], 2)])
            return None
        t_m, s_m, v_m, gap_m = path
        iv = self.g.interval(s_m)
        vr = self.rear_assumed(near)
        pred = dict(ahead=True, merge_in=round(float(t_m), 2), v_merge=round(float(v_m), 2), rear_v=round(vr, 2),
                    front_gap=None if lead is None else round(float(gap_m), 2),
                    rear_gap=None if iv is None else round(float(iv[0]-(near.hi+vr*(age+t_m))), 2))
        self.mark('lead', lead)
        self.mark('rear', near)
        self.tuck_view = dict(state='WAIT', lead=None if lead is None else (lead.lo, lead.hi), rear=(near.lo, near.hi),
                              tau=0.)
        self.tuck_diag = dict(result='ahead', rear=[near.lo, near.hi], rear_v=[round(e[0], 2), round(e[1], 2)],
                              lead=None if lead is None else [lead.lo, lead.hi], tau=0.)
        return lead, 0., pred

    def _tuck_gap(self, s0, lead, runs, age, latest, step, v0, c_why=None, skip=None):
        """tuck() for one lead: (lead, tau, prediction) when a launch fits its gap or it waits to be seen; None when no
        launch fits (tuck_why / gap_why say why)."""
        rear = self.rear_of(lead, runs)
        vx = self.lead_speed(lead)
        vr = self.rear_assumed(rear)
        self.tuck_rear = rear
        # practice analysis (RB_TUCK_EVAL): what this call decided and why
        e_r = None if rear is None else self.estimate(rear)
        self.tuck_diag = dict(front=c_why, skip=None if skip is None else [skip.lo, skip.hi],
                              lead=[lead.lo, lead.hi], lead_v=round(vx, 2),
                              rear=None if rear is None else [rear.lo, rear.hi],
                              rear_v=None if e_r is None else [round(e_r[0], 2), round(e_r[1], 2)],
                              gap_len=None if rear is None else round(float(lead.lo-rear.hi), 2))
        # The runs are where the cars were at the scan; by now they have moved on (0.1-0.2 s: the lead's tail read
        # 1.25 m behind at the launch, 0.5 m at the scan itself; 16:51-17:01 runs): age, above.
        for tau in np.arange(0., latest+1e-9, step):
            # The rear car is predicted and logged, not judged (user 2026-10-02 practice; kept so 2026-10-03): judged,
            # it held back 4 of 24 entries that went in well and let all 3 hits of 21:02 through
            path = self.tuck_path(s0, tau, lead, vx, runs, age, None, v0=v0)
            if path is None:
                if self.tuck_why == 'rear':
                    break       # a later launch only lets the car behind come closer (and searched on, 0.25 s a call)
                continue
            t_m, s_m, v_m, gap_m = path
            # the gap itself (user 2026-10-04): tight but no tighter than we squeeze into, and seen. Where we are wholly
            # in is taken behind the lead at its measured speed, like the car behind: the slow lead against the
            # measured rear took ~3.5 m off 6-7 m gaps over a 5.5 s launch from the line (02:09 runs)
            t_c, s_c = t_m, s_m
            e_l = self.estimate(lead)
            if rear is not None and e_l is not None and e_l[0] > vx:
                kept = self.path_samples                    # the launch driven stays the one behind the slow lead
                path_c = self.tuck_path(s0, tau, lead, float(e_l[0]), runs, age, None, v0=v0)
                if path_c is not None:
                    t_c, s_c = path_c[0], path_c[1]
                self.path_samples = kept
            self.gap_why = self.gap_check(lead, rear, age+tau+t_c, s_c)
            if self.gap_why == 'unseen':
                # not decided on what is not seen: keep the lead, decide once the stretch behind it is in sight
                self.tuck_view = dict(state='WAIT', lead=(lead.lo, lead.hi), rear=None, tau=None)
                self.tuck_diag.update(result='unseen', tau=round(float(tau), 1))
                return lead, None, {}
            if self.gap_why is not None:
                self.tuck_why = 'gap'
                break           # tighter than we fit, or the car behind already at us: a later launch is no better
            iv = self.g.interval(self.g.s_merged)
            pred = dict(merge_in=round(float(tau+t_m), 2), lead_v=round(vx, 2), front_gap=round(float(gap_m), 2),
                        gap_len=None if rear is None else round(float(lead.lo-rear.hi), 2),
                        v_merge=round(float(v_m), 2), rear_v=None if rear is None else round(vr, 2),
                        rear_gap=None if rear is None or iv is None else round(float(iv[0]-(rear.hi+vr*(age+tau+t_m))), 2))
            self.tuck_view = dict(state='WAIT', lead=(lead.lo, lead.hi), rear=None if rear is None else (rear.lo, rear.hi),
                                  tau=float(tau))
            self.tuck_diag.update(result='fits', tau=round(float(tau), 1))
            return lead, float(tau), pred
        self.tuck_view = dict(state='WAIT', lead=(lead.lo, lead.hi), rear=None if rear is None else (rear.lo, rear.hi),
                              tau=None)
        # no launch fits this gap
        gap_fail = getattr(self, 'tuck_why', None) == 'gap'
        self.tuck_diag.update(result=('gap_'+str(self.gap_why)) if gap_fail else 'no_launch',
                              why=None if gap_fail else getattr(self, 'tuck_why', None))
        return None


    def keep_gap(self, s, v):
        """Committed: the speed that keeps margin_m behind the car ahead (its slow case) at brake_mps2."""
        p, g = self.p, self.g
        iv = g.interval(s)
        if iv is None:
            return self.v_cap
        e_hi = iv[1]
        ahead = [b for b in self.bodies(s, v) if b.lo >= (iv[0]+iv[1])/2]
        if not ahead:
            return self.v_cap
        b = min(ahead, key=lambda x: x.lo)
        room = max(0., b.lo-e_hi-p.margin_m)
        return min(self.v_cap, math.sqrt(b.v_lo**2+2*p.brake_mps2*room))
