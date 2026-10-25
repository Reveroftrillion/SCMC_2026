"""Vehicle occupancy prediction for shadow evaluation, never a driving command.

CANDIDATE_GAP means no modeled conflict in the finite horizon. It does not
certify visibility, right of way, unseen traffic, or successful lane merging.
"""
from collections import deque
import math

import numpy as np

from .state import event_key
from .lane_prediction import LaneMotion


def integrate_turn(x, y, heading, speed, turn_rate, dt):
    if abs(turn_rate) < 1e-5:
        return (x + speed * math.cos(heading) * dt,
                y + speed * math.sin(heading) * dt, heading)
    future = heading + turn_rate * dt
    return (x + speed / turn_rate * (math.sin(future) - math.sin(heading)),
            y - speed / turn_rate * (math.cos(future) - math.cos(heading)), future)


def rectangles_overlap(a, b):
    """SAT for center x/y, heading, full length/width rectangles."""
    ax, ay, ah, al, aw = a
    bx, by, bh, bl, bw = b
    if math.hypot(ax-bx, ay-by) > (math.hypot(al, aw)+math.hypot(bl, bw))/2:
        return False
    au, av = (math.cos(ah), math.sin(ah)), (-math.sin(ah), math.cos(ah))
    bu, bv = (math.cos(bh), math.sin(bh)), (-math.sin(bh), math.cos(bh))
    for nx, ny in (au, av, bu, bv):
        projection = abs((ax-bx)*nx + (ay-by)*ny)
        ra = al/2*abs(au[0]*nx+au[1]*ny) + aw/2*abs(av[0]*nx+av[1]*ny)
        rb = bl/2*abs(bu[0]*nx+bu[1]*ny) + bw/2*abs(bv[0]*nx+bv[1]*ny)
        if projection > ra + rb:
            return False
    return True


class VehicleTrack:
    def __init__(self, min_length=4.8, min_width=2.0, lane_map=None, horizon=9.):
        self.lane_motion = LaneMotion(lane_map,horizon) if lane_map is not None else None
        self.samples = deque(maxlen=16)
        self.event = None
        self.length, self.width = min_length, min_width
        self.heading = self.turn_rate = 0.
        self.ready = False

    @property
    def radius(self):
        return math.hypot(self.length, self.width)/2

    def update(self, e):
        if self.event is not None and e.stamp <= self.event.stamp:
            return
        if not all(math.isfinite(v) for v in (e.stamp,e.x,e.y,e.vx,e.vy,e.size_x,e.size_y)):
            return
        self.event = e
        if e.speed >= .5:
            self.heading = math.atan2(e.vy, e.vx)
            self.samples.append((e.stamp, self.heading))
        else:
            self.samples.clear()
            self.turn_rate = 0.
        if math.isfinite(e.box_yaw):
            c, s = abs(math.cos(self.heading-e.box_yaw)), abs(math.sin(self.heading-e.box_yaw))
            self.length = max(self.length, c*e.size_x+s*e.size_y)
            self.width = max(self.width, s*e.size_x+c*e.size_y)
        else:
            # Unknown orientation: enclose the entire measured box.
            side = math.hypot(e.size_x,e.size_y)
            self.length, self.width = max(self.length,side), max(self.width,side)
        self.ready = False
        if len(self.samples) >= 5:
            samples = np.asarray(self.samples)
            ts = samples[:,0]-samples[-1,0]
            recent = ts >= -1.5
            ts, headings = ts[recent], np.unwrap(samples[recent,1])
            if len(ts) >= 5 and ts[-1]-ts[0] >= .6:
                design = np.c_[ts,np.ones(len(ts))]
                rate, intercept = np.linalg.lstsq(design,headings,rcond=None)[0]
                residual = float(np.max(np.abs(headings-(rate*ts+intercept))))
                self.turn_rate = float(np.clip(rate,-.6,.6))
                self.ready = abs(rate) <= .6 and residual < .3
        # Stationary bodies still occupy space, without a reliable motion heading.
        if e.speed < .5:
            self.ready = True
        if self.lane_motion is not None:
            self.lane_motion.update(e,self.heading,self.turn_rate)

    def pose_hypotheses(self, dt):
        """Stable candidate identities also used by the diagnostic display."""
        e = self.event
        if self.lane_motion is not None:
            for i,path in enumerate(self.lane_motion.paths):
                pose=path.at(self.lane_motion.speed*dt)
                if pose is not None:
                    yield 'lane',i,pose
        for rate in {0., self.turn_rate, self.turn_rate-.04, self.turn_rate+.04}:
            yield 'motion',rate,integrate_turn(e.x,e.y,self.heading,e.speed,rate,dt)

    def poses(self, dt):
        for _,_,pose in self.pose_hypotheses(dt):
            yield pose


class GapPredictor:
    def __init__(self, horizon=7., step=.2, max_speed=20/3.6,
                 min_length=4.8, min_width=2.0, lane_map=None):
        self.lane_map = lane_map
        self.horizon, self.step, self.max_speed = horizon, step, max_speed
        self.min_length, self.min_width = min_length, min_width
        self.tracks = {}
        self.retired = {}  # old identity -> last accepted stamp; cached packets cannot resurrect it
        self.reassociations = 0

    def observe(self, events, now):
        incoming = {}
        for e in events:
            if e.type not in ('VEHICLE', 'DYNAMIC_UNKNOWN') or not e.value.startswith('lidar:'):
                continue
            if not all(math.isfinite(v) for v in (e.stamp,e.x,e.y,e.vx,e.vy)):
                continue
            key = event_key(e)
            if e.stamp <= self.retired.get(key, -math.inf):
                continue
            if key not in incoming or e.stamp > incoming[key].stamp:
                incoming[key] = e
        refreshed = {k for k,e in incoming.items()
                     if k in self.tracks and e.stamp > self.tracks[k].event.stamp}
        candidates = {}
        for key,e in incoming.items():
            if key in self.tracks:
                continue
            possible = []
            for old,tr in self.tracks.items():
                dt = e.stamp-tr.event.stamp
                if old in refreshed or not .05 <= dt <= .35 or len(tr.samples) < 3:
                    continue
                if e.speed < .8 or tr.event.speed < .8:
                    continue
                if math.hypot(e.vx-tr.event.vx,e.vy-tr.event.vy) > 2.:
                    continue
                heading = math.atan2(e.vy,e.vx)
                if math.cos(heading-tr.heading) < math.cos(math.radians(25)):
                    continue
                x,y,_ = integrate_turn(tr.event.x,tr.event.y,tr.heading,
                                       tr.event.speed,tr.turn_rate,dt)
                if math.hypot(e.x-x,e.y-y) <= 1.25+.5*dt:
                    possible.append(old)
            candidates[key] = possible
        # Mutual uniqueness: two fragments / neighboring cars cannot claim one history.
        claims = {}
        for possible in candidates.values():
            for old in possible:
                claims[old] = claims.get(old,0)+1
        for key,possible in candidates.items():
            if len(possible) == 1 and claims[possible[0]] == 1:
                old = possible[0]
                tr = self.tracks.pop(old)
                self.retired[old] = tr.event.stamp
                self.tracks[key] = tr
                self.reassociations += 1
        for key,e in incoming.items():
            if e.stamp <= self.retired.get(key, -math.inf):
                continue
            if key not in self.tracks:
                self.tracks[key] = VehicleTrack(self.min_length,self.min_width,self.lane_map,self.horizon+1.5)
            self.tracks[key].update(e)
        self.tracks = {k:t for k,t in self.tracks.items()
                       if t.event is not None and 0 <= now-t.event.stamp <= 1.5}
        self.retired = {k:t for k,t in self.retired.items() if now-t <= 2.}

    # Our departure along the route (user 2026-09-29: coming in fast, we get further into the ring before the gap
    # closes -- judge the entry timing on it). From our current speed, after the 0.4 s reaction, speed up at `accel`
    # up to what the route's curvature allows (a_lat EGO_A_LAT, braking EGO_BRAKE ahead of tighter bends, at most
    # EGO_V_MAX). It replaces "ramp to max_speed (25 km/h), or hold the current speed if faster", which cut a slow
    # start short and let a fast one run through the ring's bends at approach speed.
    EGO_A_LAT, EGO_BRAKE, EGO_V_MAX = 4.5, 3., 60/3.6

    def _ego_speed_cap(self, route):
        cache = getattr(self, '_ego_caps', None)
        if cache is not None and cache[0] is route:return cache[1]
        cap = np.minimum(self.EGO_V_MAX, np.sqrt(self.EGO_A_LAT/np.maximum(route.curvature, 1e-6)))
        ds = np.diff(route.s)
        for i in range(len(cap)-2, -1, -1):
            cap[i] = min(cap[i], math.sqrt(cap[i+1]**2+2*self.EGO_BRAKE*ds[i]))
        self._ego_caps = (route, cap)
        return cap

    def _ego_travel(self, ctx, accel):
        key = (id(ctx.route), ctx.now, ctx.s, ctx.v, accel)
        cache = getattr(self, '_ego_paths', {})
        if key in cache:return cache[key]
        cap = self._ego_speed_cap(ctx.route);rs = ctx.route.s
        s0 = ctx.s+ctx.s_offset;v = max(0., ctx.v);dt = .05
        ts = [0.];xs = [0.];x = 0.;t0 = 0.
        while t0 < self.horizon+6.:
            here = float(np.interp(s0+x, rs, cap))
            if t0 >= .4:v = min(max(v, 0.)+accel*dt, max(here, 0.)) if v <= here else here
            x += v*dt;t0 += dt;ts.append(t0);xs.append(x)
        out = (np.array(ts), np.array(xs))
        if len(cache) > 32:cache = {}
        cache[key] = out;self._ego_paths = cache
        return out

    def ego_pose(self, ctx, dt, accel=1.):
        """Body center on the exact departure hypothesis used by collision checks."""
        ts, xs = self._ego_travel(ctx, accel)
        s = ctx.s+ctx.s_offset+float(np.interp(dt, ts, xs))
        x,y = ctx.route.at(s); heading = ctx.route.heading_at(s)
        return (x+1.4825*math.cos(heading),y+1.4825*math.sin(heading),heading,4.635,1.892)

    @staticmethod
    def body_margin(tr, dt, age, bridged=False):
        margin = .35+.15*(dt+age)
        if bridged:
            margin += 1.5*age*age + tr.radius*.2*age
        return margin

    def evaluate(self, ctx):
        base = dict(mode='shadow', evaluated_at=ctx.now,
                    evidence_stamp=max((t.event.stamp for t in self.tracks.values()), default=-math.inf),
                    horizon_s=self.horizon, tracks=len(self.tracks),
                    lane_tracks=sum(bool(t.lane_motion and t.lane_motion.paths) for t in self.tracks.values()))
        if ctx.route is None:
            return dict(base, decision='UNKNOWN', reason='route unavailable')
        tracks = list(self.tracks.values())
        if not tracks:
            return dict(base, decision='UNKNOWN', reason='no tracked vehicle; visibility unverified')
        uncertainties = {}
        bridged = set()
        base['reassociations'] = self.reassociations
        for tr in tracks:
            reasons = []
            if not tr.ready:
                reasons.append('cold_motion')
            age = ctx.now-tr.event.stamp
            coverage = (tr.lane_motion is not None and
                        tr.lane_motion.complete(self.horizon+max(0.,age)))
            if .5 < age <= .8 and tr.ready and coverage:
                bridged.add(tr.event.value)
            elif not 0 <= age <= .5:
                reasons.append('stale')
            if self.lane_map is not None and not tr.lane_motion.complete(
                    self.horizon+max(0.,ctx.now-tr.event.stamp)):
                reasons.append('incomplete_lane')
            if reasons:
                uncertainties[tr.event.value] = reasons
        base['uncertainties'] = uncertainties
        base['bridged_tracks'] = len(bridged)
        base['bridged_ids'] = sorted(bridged)
        uncertain = bool(uncertainties) or not 0 <= ctx.now-base['evidence_stamp'] <= .45
        # Rear axle to body center shift: front3.8, rear0.835.
        for dt in np.arange(0.,self.horizon+self.step/2,self.step):
            for accel in (.7,1.,1.3):
                ego = self.ego_pose(ctx,dt,accel)
                for tr in tracks:
                    age = max(0.,ctx.now-tr.event.stamp)
                    margin = self.body_margin(tr,dt,age,tr.event.value in bridged)
                    for ox,oy,oh in tr.poses(dt+age):
                        other = (ox,oy,oh,tr.length+2*margin,tr.width+2*margin)
                        if rectangles_overlap(ego,other):
                            model = 'lane' if (tr.lane_motion is not None and
                                any(math.hypot(ox-p[0],oy-p[1]) < 1e-6
                                    for p in tr.lane_motion.poses(dt+age))) else 'kinematic'
                            return dict(base,decision='WAIT',reason='predicted footprint conflict',
                                        conflict_model=model, conflict_xy=[float(ox),float(oy)],
                                        conflict_heading=float(oh), conflict_ego=list(ego),
                                        conflict_accel_mps2=accel,
                                        conflict_in_s=round(float(dt),2),track=tr.event.value)
        if uncertain:
            return dict(base,decision='UNKNOWN',reason='cold/stale motion or incomplete lane coverage')
        return dict(base,decision='CANDIDATE_GAP',reason='no modeled conflict; visibility unverified')
