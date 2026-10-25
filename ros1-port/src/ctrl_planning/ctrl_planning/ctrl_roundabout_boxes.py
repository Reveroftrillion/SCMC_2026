"""The roundabout's ring lane as boxes along the arc, from the current lidar scan only (highway_boxes on a circle).

Arc coordinate u: metres along the ring in its direction of travel, 0 where our body first reaches the ring lane
(RoundaboutGeometry.origin). A box is OCCUPIED when a hull has points inside the lane band over it, UNKNOWN when the
lidar's line of sight to it passes behind a hull ('shadow:<id>') or it is beyond the measured sight ('RANGE'),
FREE otherwise. Consecutive occupied boxes form runs; each run's rear (lo) and front (hi) edge is followed by the
highway's edge Kalman filter (highway_box_tracks.EdgeTracks): no tracker id or tracker speed is used.
"""
import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from .ctrl_highway_box_tracks import EdgeTracks

FREE, OCCUPIED, UNKNOWN = 'FREE', 'OCCUPIED', 'UNKNOWN'


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Run:
    lo: float                 # rear edge u
    hi: float                 # front edge u
    lo_hidden: bool           # the box behind it is a shadow: its rear may be further back
    hi_hidden: bool
    lo_speed: Optional[tuple]  # (speed, sigma) of the rear edge (EdgeTracks), None before two measurements
    hi_speed: Optional[tuple]
    coasted: float = 0.        # s since it was last in a scan (0: in this scan)
    speed: Optional[tuple] = None  # (speed, sigma) used for the run: its edges' or, carried, the run it was last scan
    height: float = 0.             # tallest hull seen in it so far (carried scan to scan like the speed)
    lo_clear: bool = False         # its rear edge measured this scan (a seen box past it, within sight)
    hi_clear: bool = False


@dataclass
class Gap:
    lo: float                      # rear end: the front of the car behind (or the lane window's upstream end)
    hi: float                      # front end: the tail of the car ahead (or the window's downstream end)
    rear_speed: Optional[tuple]    # (speed, sigma) of the car behind, None if none / unknown
    front_speed: Optional[tuple]
    sure: bool                     # every box between is FREE (no shadow, island or range inside it)
    bounded: tuple = (True, True)  # a car at each end (False: the window ends there)

    @property
    def length(self):
        return self.hi-self.lo

    def at(self, t):
        """Ends after t s, each moving at its car's speed."""
        lo = self.lo+(self.rear_speed[0]*t if self.rear_speed else 0.)
        hi = self.hi+(self.front_speed[0]*t if self.front_speed else 0.)
        return lo, hi


class RingLane:
    """Boxes, runs and edge speeds on the ring lane.

    Lane band: circle radius +- lane_half_m; a hull counts once it reaches intrusion_m into it. Sight: ring cars
    nearer than 38 m to the lidar were detected 100 % (2026-10-01 laps); boxes are kept up to sight_m (100 m, user
    2026-10-03: perceive from the mission start) and decisions only use what is within DECIDE_SIGHT_M of the planner."""

    def __init__(self, circle, origin, up_m=45., down_m=35., box_m=.5, lane_half_m=1.75, intrusion_m=.3,
                 sight_m=100., shadow_pad_rad=.02, coast_s=.5, island_in_m=4.5):
        self.circle, self.origin = circle, origin
        self.edges = np.arange(-up_m, down_m+1e-9, box_m)
        self.box_m = box_m
        self.lane_half, self.intrusion, self.sight, self.pad = lane_half_m, intrusion_m, sight_m, shadow_pad_rad
        self.tracks = EdgeTracks()
        # Ring edges (2026-10-03, replayed on 22 practice runs' per-scan runs): an end not measured in the clear goes in
        # at 0.5 m noise instead of being left out, and a new end starts at a typical ring car's speed -- runs given a
        # speed 68 -> 91 % for cars about to cross our entry, |err| p90 8.4 -> 4.9 km/h; 30 m out p90 12.6 -> 7.2 km/h
        self.tracks.VAGUE_SIGMA = self.VAGUE_SIGMA
        self.tracks.INIT_SPEED, self.tracks.INIT_SPEED_SIGMA = self.INIT_SPEED, self.INIT_SPEED_SIGMA
        # swept on the same replay (accel 0.5-2, position 0.15-0.5 m, vague 0.5-1.5 m, initial sigma 1-3 m/s), kept
        # the one no worse for slow (< 14 km/h) and speed-changing cars: 30 m out p90 7.1 -> 5.4, at our entry 6.5 -> 5.3
        # km/h. A tighter initial speed (1 m/s) read typical cars better still but slow ones p90 11 -> 15 km/h.
        self.tracks.ACCEL_SIGMA, self.tracks.POS_SIGMA = self.ACCEL_SIGMA, self.POS_SIGMA
        # One-way ring (user 2026-10-04): no car goes backwards, and a car first seen starts from its own edges -- a car
        # crossing our entry first read -30 km/h (its new front edge took over another's state) and took 5 s to reach
        # its 10 km/h (02:58 run 1)
        self.tracks.NEG_RESET, self.tracks.INHERIT = self.NEG_RESET, False
        self.tracks.BIRTH_N, self.tracks.BIRTH_UNSURE_SD = self.BIRTH_N, self.BIRTH_UNSURE_SD
        self.tracks.INIT_SPEED_SIGMA = self.BIRTH_SPEED_SIGMA
        self.stamp = -math.inf
        self.boxes = []
        self.runs: List[Run] = []
        # A run that drops out of the scan keeps moving at its last speed this long (0.5 s by the user, 2026-10-03:
        # longer memories stood as afterimages that held the entry back); one dropped scan once let the entry commit
        # into the car it was waiting for (roundabout-rebuild view run 10:31).
        self.coast_s = coast_s
        # The central island hides the far side: a ring car whose sight line passes more than this inside the lane
        # centreline was missed 33-100 % of scans (5..17 m), within 4 m 8-20 % (g3/g4 runs 2026-10-02, 6236 cars).
        self.island_r = circle.radius-island_in_m
        self.memory = []          # [(stamp, Run)] of runs as last seen

    def watched_s(self):
        """How long the ring has been watched; an empty view means nothing only after the measured dropout limit."""
        return self.stamp-self.first_stamp if getattr(self, 'first_stamp', None) is not None else 0.

    def gaps(self):
        """The spaces between the runs (cars) along the window, upstream to downstream."""
        runs = sorted(self.runs, key=lambda r: r.lo)
        out = []
        prev_hi, prev = self.edges[0], None
        for r in runs+[None]:
            lo = prev_hi
            hi = r.lo if r is not None else float(self.edges[-1])
            if hi > lo:
                inside = [b for b in self.boxes if b[0] >= lo-1e-6 and b[1] <= hi+1e-6]
                sure = all(b[2] == FREE for b in inside)
                out.append(Gap(lo, hi, prev.speed if prev is not None else None, r.speed if r is not None else None,
                               sure, (prev is not None, r is not None)))
            if r is not None and r.hi > prev_hi:
                prev_hi, prev = r.hi, r
        return out

    def u_of(self, x, y):
        a = math.atan2(y-self.circle.center[1], x-self.circle.center[0])*self.circle.direction
        return wrap(a-self.origin)*self.circle.radius

    def xy_of(self, u):
        a = (self.origin+u/self.circle.radius)*self.circle.direction
        c, R = self.circle.center, self.circle.radius
        return np.array([c[0]+R*math.cos(a), c[1]+R*math.sin(a)])

    # Ring cars as measured (150 practice runs 2026-10-03, 15898 car pairs): never closer than 1.93 m nose to tail
    # (0.01 % under 2 m), none longer than 8.83 m (a truck)
    MIN_CAR_GAP_M = 1.93
    MAX_CAR_LEN_M = 8.83
    # m/s2: ring cars' speed change (EdgeTracks process noise). 3 since 2026-10-09 (rb_speed_replay.py, 43 bags vs truth):
    # at 1 a car speeding up read 1.2 m/s slow with a 0.3 m/s sigma and hit us from behind; the sigma given was beaten
    # 2x by the real error 20 % of the time (44 % for cars changing speed), at 3 3.3 % (9 %), accuracy alike. (The
    # slow-ring stops it seemed to cause came from following a lead at its slow side: lead_speed now the estimate.)
    ACCEL_SIGMA = 3.
    POS_SIGMA = .5           # m: an end's position noise in the clear (the highway keeps 0.15)
    VAGUE_SIGMA = .5         # m: an end not measured in the clear, its position noise (EdgeTracks)
    INIT_SPEED = 0.          # m/s: a new end's speed before measured (no typical speed, user 2026-10-09; BIRTH_SPEED_SIGMA) ...
    INIT_SPEED_SIGMA = 2.    # ... and its sigma
    TRAFFIC_SD = 1.          # m/s: runs measured at least this well set a new end's starting speed (see update)
    NEG_RESET = -1.          # m/s: an end read slower than this is started afresh (no ring car goes backwards)
    BIRTH_N = 5              # an end's first measurements given out as unsure ...
    BIRTH_UNSURE_SD = 3.     # ... at least this sigma (m/s)
    BIRTH_SPEED_SIGMA = 10.  # m/s: a new end's speed comes from its own measurements, hardly from the start speed

    VAGUE_SPEED_K = 3.       # an end not measured in the clear counts at 3x its sigma in a run's speed ...
    # m/s: ... and run speeds read this much slow, steady cars too (38 practice runs, 2026-10-03). 0 since 2026-10-09: with
    # carried speeds no longer frozen they read 0.19 m/s fast with it (median of 629 ring cars vs truth), the car behind a
    # gap taken 20-27 % fast and gaps we fitted into given up (user: no fixed offsets)
    SPEED_OFFSET = 0.

    @classmethod
    def fuse(cls, lo, hi, lo_clear=True, hi_clear=True):
        """A run's own speed: its two ends' estimates weighted by their certainty (inverse variance), or the one there
        is; None if neither. Picking one end (the tail first) left the ring cars about to cross our entry without a
        speed 32 % of the time. An end not measured in the clear weighs less: an approaching car's tail, hidden by
        itself, shows more of the car as it comes and read 2-3.6 km/h slow; with it at 3x sigma and the offset, standing
        on the line |err| p90 5.9 -> 4.4 km/h, bias -2.0 -> -0.2 (2026-10-03)."""
        own = [(e[0], e[1]*(1. if clear else cls.VAGUE_SPEED_K)) for e, clear in ((lo, lo_clear), (hi, hi_clear))
               if e is not None and e[1] == e[1] and e[1] > 0.]
        if not own:
            return None
        w = [1./(e[1]*e[1]) for e in own]
        return (sum(wi*e[0] for wi, e in zip(w, own))/sum(w)+cls.SPEED_OFFSET, math.sqrt(1./sum(w)))

    READY_SIGMA = .3     # m/s: an edge speed this sure is used as it is (the speed bands start there)
    # An empty view means an empty ring once watched this long: ring cars upstream of our entry (u -30..0, within the
    # sight) dropped out of the scan for at most 2.27 s (p99 1.46, 167 dropouts, practice runs 2026-10-02 evening)
    EMPTY_WATCH_S = 2.27

    def update(self, stamp, hulls, sensor_xy, heights=None):
        """hulls: {id: Nx2 map points} of the scan at `stamp`; sensor_xy: lidar position then; heights: {id: m}."""
        heights = heights or {}
        if stamp <= self.stamp:
            return False
        dt = stamp-self.stamp if self.runs else 0.
        if getattr(self, 'first_stamp', None) is None:
            self.first_stamp = stamp
        self.stamp = stamp
        self.sensor_xy = tuple(sensor_xy)
        e = self.edges
        n = len(e)-1
        state = [FREE]*n
        key = ['']*n
        c, R = np.asarray(self.circle.center, dtype=float), self.circle.radius
        band = self.lane_half-self.intrusion
        for hid, pts in hulls.items():
            pts = np.asarray(pts, dtype=float)
            if pts.ndim != 2 or len(pts) < 2:
                continue
            r = np.hypot(pts[:, 0]-c[0], pts[:, 1]-c[1])-R
            if r.min() > band or r.max() < -band:
                continue
            us = np.array([self.u_of(x, y) for x, y in pts])
            lo, hi = us.min(), us.max()
            if hi-lo > math.pi*R:           # straddles the wrap point: not on our stretch
                continue
            k0 = int(np.searchsorted(e, lo, 'right'))-1
            k1 = int(np.searchsorted(e, hi, 'right'))-1
            for k in range(max(0, k0), min(n-1, k1)+1):
                state[k] = OCCUPIED
                key[k] = hid
        sensor = np.asarray(sensor_xy, dtype=float)
        self.sensor = sensor
        shadows = []
        for hid, pts in hulls.items():
            rel = np.asarray(pts, dtype=float)-sensor
            if rel.ndim != 2 or len(rel) < 2:
                continue
            rng = np.hypot(rel[:, 0], rel[:, 1])
            if rng.min() < .5:
                continue
            mid = math.atan2(*rel.mean(axis=0)[::-1])
            ang = np.array([wrap(math.atan2(y, x)-mid) for x, y in rel])
            shadows.append((hid, mid, ang.min()-self.pad, ang.max()+self.pad, float(rng.min())))
        for k in range(n):
            if state[k] != FREE:
                continue
            p = self.xy_of((e[k]+e[k+1])/2)-sensor
            d = float(np.hypot(*p))
            if d > self.sight:
                state[k], key[k] = UNKNOWN, 'RANGE'
                continue
            q = float(np.clip(((c-sensor)@p)/max(1e-9, p@p), 0., 1.))
            if float(np.hypot(*(sensor+q*p-c))) < self.island_r:
                state[k], key[k] = UNKNOWN, 'ISLAND'
                continue
            a = math.atan2(p[1], p[0])
            for hid, mid, a0, a1, rmin in shadows:
                if a0 <= wrap(a-mid) <= a1 and d > rmin+.5:
                    state[k], key[k] = UNKNOWN, 'shadow:'+hid
                    break
        self.boxes = [(float(e[k]), float(e[k+1]), state[k], key[k]) for k in range(n)]
        # Runs of occupied boxes; an edge next to a shadow box is not measured (the car may extend into it).
        raw = []
        clear = []
        k = 0
        while k < n:
            if state[k] != OCCUPIED:
                k += 1
                continue
            j = k
            while j+1 < n and state[j+1] == OCCUPIED:
                j += 1
            lo_hidden = k > 0 and state[k-1] == UNKNOWN and key[k-1] != 'RANGE'
            hi_hidden = j < n-1 and state[j+1] == UNKNOWN and key[j+1] != 'RANGE'
            h = max([float(heights.get(key[x]) or 0.) for x in range(k, j+1)], default=0.)
            raw.append((float(e[k]), float(e[j+1]), lo_hidden, hi_hidden, h))
            # An edge is measured only in the clear: a seen box beyond it and within sight. Next to the island or past
            # the sight a car is cut off where it shows, and the cut moving as we come read 9-14 m/s on cars going 5.7
            # (04:04-04:10 runs); those are no data of ours (user 2026-10-03).
            clear.append((k > 0 and state[k-1] != UNKNOWN and float(np.hypot(*(self.xy_of(e[k])-sensor))) <= self.sight,
                          j < n-1 and state[j+1] != UNKNOWN and float(np.hypot(*(self.xy_of(e[j+1])-sensor))) <= self.sight))
            k = j+1
        # One car broken into pieces (part of its side not reaching into the lane, a fragment, a shadow over its middle)
        # is put back together: runs closer than any two ring cars ever came (MIN_CAR_GAP_M) are one car, as long as
        # together they are no longer than the longest ring car (user 2026-10-03). The boxes between are taken as
        # occupied and the inner ends, no car's ends, are not measured.
        merged, mclear = [], []
        for r, cl in zip(raw, clear):
            # (lengths in whole boxes: each end may read up to a box long)
            if merged and r[0]-merged[-1][1] < self.MIN_CAR_GAP_M and r[1]-merged[-1][0] <= self.MAX_CAR_LEN_M+2*self.box_m:
                a = merged[-1]
                merged[-1] = (a[0], r[1], a[2], r[3], max(a[4], r[4]))
                mclear[-1] = (mclear[-1][0], cl[1])
                for q in range(n):
                    if a[1]-1e-6 <= e[q] and e[q+1] <= r[0]+1e-6 and state[q] != OCCUPIED:
                        self.boxes[q] = (self.boxes[q][0], self.boxes[q][1], OCCUPIED, 'joined')
                continue
            merged.append(r); mclear.append(cl)
        raw, clear = merged, mclear
        # A new end starts from its own measurements (BIRTH_SPEED_SIGMA), not from the ring's average (user 2026-10-09)
        self.tracks.update(stamp, {'lo': [(lo, ok) for (lo, _, _, _, _), (ok, _) in zip(raw, clear)],
                                   'hi': [(hi, ok) for (_, hi, _, _, _), (_, ok) in zip(raw, clear)]})
        prov = self.tracks.provisional
        runs = [Run(lo, hi, lh, hh, prov['lo'][i], prov['hi'][i], height=h, lo_clear=clear[i][0], hi_clear=clear[i][1])
                for i, (lo, hi, lh, hh, h) in enumerate(raw)]
        # One car's edges come and go behind shadows and the island (the front one as it crosses our entry), and a new
        # edge track starts unsure: the run keeps the speed of the run it was a scan ago (the one it overlaps, moved
        # on at its speed) unless its own edges know better. Boxes only, no object identity.
        for r in runs:
            best = self.fuse(r.lo_speed, r.hi_speed, r.lo_clear, r.hi_clear)
            prev = [(min(r.hi, x.hi+(x.speed[0] if x.speed else 0.)*dt)-max(r.lo, x.lo+(x.speed[0] if x.speed else 0.)*dt), x)
                    for x in self.runs]
            prev = [p for p in prev if p[0] > 0.]
            if prev:
                was = max(prev, key=lambda p: p[0])[1]
                r.height = max(r.height, was.height)
                # carried on, a speed grows vaguer by what ring cars can change it in dt (ACCEL_SIGMA) and its own edges
                # pull it along: kept as it was, a speed measured well stood unchanged for 1.5 s while the car slowed
                # to the entry (4.11 m/s read, 3.0 true, 14:35 keep run; user 2026-10-09)
                if was.speed is not None:
                    carried = (was.speed[0], math.hypot(was.speed[1], self.ACCEL_SIGMA*dt))
                    if best is None:
                        best = carried
                    elif carried[1] < best[1]:
                        w0, w1 = 1./carried[1]**2, 1./best[1]**2
                        best = ((w0*carried[0]+w1*best[0])/(w0+w1), math.sqrt(1./(w0+w1)))
            r.speed = best
            # The run ends where its points end, hidden end too (user 2026-10-02): a car is a box, its side runs to
            # its tail; seen far off it may read short, near it the side shows it whole. No filling by car class.
        kept = []
        for seen_at, r in self.memory:
            if r.speed is None:
                continue          # no speed, nowhere to carry it: kept still it became a ghost a launch aimed at (22:38)
            age = stamp-seen_at
            v = r.speed[0] if r.speed is not None else 0.
            lo, hi = r.lo+v*age, r.hi+v*age
            if lo > self.edges[-1] or hi < self.edges[0]:
                continue
            # A lost car moves on at its speed for coast_s, hidden or not. Kept on while hidden, a memory read ~0 m/s
            # stood behind a shadow for seconds and held our entry back; coast_s 0.5 s by the user (2026-10-03) --
            # ring cars came back into sight within p50 0.27 / p90 0.59 / max 2.27 s (167 dropouts).
            if age > self.coast_s:
                continue
            if any(x.lo < hi and lo < x.hi for x in runs):
                continue          # something seen where it would be: that is it
            kept.append((seen_at, r))
            runs.append(Run(lo, hi, r.lo_hidden, r.hi_hidden, r.lo_speed, r.hi_speed, age, r.speed))
        self.memory = [(stamp, r) for r in runs if r.coasted == 0.]+kept
        self.runs = runs
        # Hidden boxes no car can be in: free by inference -- a hidden stretch no seen or carried car covers, entered
        # from upstream through a box we see (free, or a car). Fed from past the sight, from behind the island or from
        # the window's end, a car we never saw may be in it (truth: 7.5-17 % of such boxes held one, 3.6-5.6 % of the
        # ones fed from a seen box; g3/g4 2026-10-02): unknown.
        boxes = self.boxes
        covered = [any(r.lo < b1 and r.hi > b0 for r in runs) for b0, b1, _, _ in boxes]
        cand = [s == UNKNOWN and k != 'RANGE' and not cov for (_, _, s, k), cov in zip(boxes, covered)]
        out = list(boxes)
        k = 0
        while k < len(boxes):
            if not cand[k]:
                k += 1
                continue
            j = k
            while j+1 < len(boxes) and cand[j+1]:
                j += 1
            seen_car = k > 0 and any(r.coasted == 0. and r.lo < boxes[k-1][1] and r.hi > boxes[k-1][0] for r in runs)
            fed = k > 0 and (boxes[k-1][2] in (FREE, OCCUPIED) or seen_car)
            if fed:
                for x in range(k, j+1):
                    out[x] = (boxes[x][0], boxes[x][1], FREE, 'inferred')
            k = j+1
        self.boxes = out
        return True
