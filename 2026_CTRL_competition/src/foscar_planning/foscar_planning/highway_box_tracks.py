"""Speeds from box occupancy alone, no object identity (user 2026-09-28).

The lidar's object ids churn (a lane-3 car went 297 -> 364 -> 335 -> 371 ... in two seconds) and the tracker's speed
restarts with every new id. Instead each lane is cut into boxes; consecutive occupied boxes form a run (one car or
several close together). A run's rear and front edges are the lowest and highest station of the hull points in it,
and how fast an edge moves along the lane -- how fast the boxes in front of or behind it change state -- is the
speed of what is there. Edges are followed from scan to scan by position only (the nearest edge of the same kind
where the last one was heading), never by an object id. An edge next to a box hidden behind another body (or at the
end of the box range) is where the view ends, not where a car ends: it is not sampled, and it keeps the speed it
had while visible for a second.
"""
import numpy as np
from .highway_boxes import OCCUPIED, UNKNOWN


def lane_runs(boxes, extents):
    """Occupied runs of a box row: (lo, hi, lo_hidden, hi_hidden, first box, last box)."""
    out = [];i = 0;n = len(boxes)
    while i < n:
        if boxes[i][2] != OCCUPIED:i += 1;continue
        j = i
        while j+1 < n and boxes[j+1][2] == OCCUPIED:j += 1
        ext = [extents[k] for k in range(i, j+1) if extents[k] is not None]
        lo = min(e[0] for e in ext) if ext else boxes[i][0]
        hi = max(e[1] for e in ext) if ext else boxes[j][1]
        out.append((lo, hi, i == 0 or boxes[i-1][2] == UNKNOWN, j == n-1 or boxes[j+1][2] == UNKNOWN, i, j))
        i = j+1
    return out


class EdgeTracks:
    """Rear ('lo') and front ('hi') edges of the runs in one lane, followed by position; rate = speed along the lane.

    Each edge's station is filtered by a 1-D constant-velocity Kalman filter (2026-10-01, replacing a 1 s line
    fit). Replayed against the NPC truth of 10 HW980 runs (speed_truth.py): runs off by more
    than 5 km/h 8 -> 6 %, p90 4.5 -> 3.7 km/h, our-lane lead p90 2.8 -> 2.1 km/h, 30-60 m ahead 17 -> 8 %, worst
    29.8 -> 24.5 km/h, with about as many runs given a speed (15164 -> 14878). A lane-2 lead read 45-48 km/h while
    doing 52-60 and the car hung 2x the following gap behind it (2026-10-01 run 5).
    """
    MAX_SPEED = 40.     # m/s an edge may move between scans before its rate is known
    KEEP_S = .3         # an edge not seen (hidden, or a missed scan) is kept this long
    HOLD_S = 1.         # a hidden edge keeps the speed measured while it was visible this long
    JUMP_M = 5.         # an edge that jumps up to a car length (a fragment joined or left the outline) keeps its motion
    # Kalman filter, swept on the same replay (accel 0.7/1/1.5/2/3, range noise 0.01/0.02/0.03, ready 0.8/1.2).
    # Range noise 0.02 was 1 % better but gave a car 55 m ahead no speed within 1 s (test_a_car_past_40_m...).
    ACCEL_SIGMA = 1.0      # m/s2: how fast an NPC's speed changes (process noise)
    POS_SIGMA = .15        # m: edge position noise next to us ...
    POS_SIGMA_PER_M = .01  # ... growing with range (coarser boxes and outlines far away)
    GATE_SIGMA = 3.        # a measured edge this many sigma (and GATE_MIN_M) off its prediction is an outline jump:
    GATE_MIN_M = 1.        # it moves the edge, not its speed
    READY_SIGMA = 1.2      # m/s: a rate is given once its standard deviation is below this ...
    READY_N = 3            # ... after this many measurements
    ASSOC_SIGMA = 3.       # m/s: a speed estimate this sure (not yet given) already predicts where its edge went
    # Off on the highway (as swept there); the ring lane sets them (roundabout_boxes.RingLane):
    VAGUE_SIGMA = None     # m: an edge not measured in the clear still goes in, at this position noise (None: left out)
    INIT_SPEED = 0.        # m/s, and its sigma: a new edge's speed before it is measured
    INIT_SPEED_SIGMA = 20.
    # One-way lanes (the ring sets them): a speed below NEG_RESET is no car's -- an edge that took over another's state
    # (or a growing sliver's motion) is started afresh from its own measurements; a new edge takes no other edge's
    # state (INHERIT off) and is given out as unsure (sigma >= BIRTH_UNSURE_SD) for its first BIRTH_N measurements.
    NEG_RESET = None
    INHERIT = True
    BIRTH_N = 0
    BIRTH_UNSURE_SD = 0.

    def __init__(self, window=None):
        self.tracks = {'lo': [], 'hi': []};self.ego_s = None

    def _r(self, s):
        d = abs(s-self.ego_s) if self.ego_s is not None else 0.
        return (self.POS_SIGMA+self.POS_SIGMA_PER_M*d)**2

    def _predict(self, tr, stamp):
        dt = max(0., stamp-tr['kt']);q = self.ACCEL_SIGMA**2
        F = np.array([[1., dt], [0., 1.]]);Q = q*np.array([[dt**4/4, dt**3/2], [dt**3/2, dt**2]])
        tr['x'] = F@tr['x'];tr['P'] = F@tr['P']@F.T+Q;tr['kt'] = stamp

    def _measure(self, tr, stamp, s, R=None):
        R = self._r(s) if R is None else R
        if 'x' not in tr:
            tr['x'] = np.array([s, self.INIT_SPEED]);tr['P'] = np.diag([R, self.INIT_SPEED_SIGMA**2]);tr['kt'] = stamp;tr['n'] = 1
            tr['mt'] = stamp;return
        # One scan is one measurement (user 2026-10-08): the policy ticks 2-3 times per scan and fed the same edge each
        # time; the filter took three copies for three measurements, a truck's edge went from sigma 5.6 to 0.6 m/s in
        # one scan at -0.4 m/s while it did 5.6, and the 4->3 pass planned ahead of it hit it (13:29 run 2, s1350).
        if tr.get('mt') == stamp:
            return
        tr['mt'] = stamp
        self._predict(tr, stamp)
        y = s-tr['x'][0];S = tr['P'][0, 0]+R
        if tr['n'] >= self.READY_N and abs(y) > max(self.GATE_MIN_M, self.GATE_SIGMA*np.sqrt(S)):
            tr['x'][0] = s;tr['P'][0, 0] = R;tr['P'][0, 1] = tr['P'][1, 0] = 0.;return
        # Joseph form, kept symmetric: the short form P - K H P lost positive definiteness over long tracks (sqrt of a
        # negative variance switched the jump gate off, 2026-10-01 replays).
        K = tr['P'][:, 0]/S;tr['x'] = tr['x']+K*y
        A = np.eye(2)-np.outer(K, [1., 0.]);P = A@tr['P']@A.T+R*np.outer(K, K);tr['P'] = (P+P.T)/2;tr['n'] += 1

    def _rate(self, tr):
        if tr.get('n', 0) < self.READY_N or tr['P'][1, 1] > self.READY_SIGMA**2:return None
        return float(tr['x'][1])

    def update(self, stamp, edges, ego_s=None):
        """edges: {'lo': [(station, visible)], 'hi': [...]} -> {'lo': [rate or None], 'hi': [...]} (same order)."""
        self.ego_s = ego_s
        out = {};self.provisional = {}
        for kind, values in edges.items():
            old = self.tracks[kind];used = set();new = [];rates = [];prov = []
            for s, visible in values:
                best = None
                for k, tr in enumerate(old):
                    if k in used:continue
                    t0, s0 = tr['last'];dt = max(0., stamp-t0)
                    # ...and an edge is predicted on with the speed it is estimated at, given or not yet: predicted as
                    # standing until given, a stale edge left 0.7 s earlier lay nearer the truck's rear than the edge
                    # following it at 5.5 m/s, and took its measurement (13:29 run 2).
                    v = tr['rate']
                    if v is None and 'x' in tr and tr['P'][1, 1] <= self.ASSOC_SIGMA**2:v = float(tr['x'][1])
                    pred = s0+(v or 0.)*dt
                    gate = 1.5+(5. if tr['rate'] is not None else self.MAX_SPEED)*dt
                    d = abs(s-pred)
                    if d <= gate and (best is None or d < best[0]):best = (d, k)
                if best is None and visible and self.INHERIT:
                    # The outline jumped, not the car: a rear fragment joining 4 m behind a lead restarted its rear
                    # edge, the lead read as standing and braked 0.5 (2026-09-30 run 6, s1187). Follow the nearest
                    # measured edge within a car length and move it to the jump, keeping its rate.
                    for k, tr in enumerate(old):
                        if k in used or tr['rate'] is None:continue
                        t0, s0 = tr['last'];d = abs(s-(s0+tr['rate']*max(0., stamp-t0)))
                        if d <= self.JUMP_M and (best is None or d < best[0]):best = (d, k)
                    if best is not None and 'x' in old[best[1]]:
                        # reset as the gate does, cross terms too: left in, the covariance went indefinite (a negative
                        # speed variance) once ring edges were fed every scan (2026-10-03 replays)
                        tr = old[best[1]];self._predict(tr, stamp);tr['x'][0] = s;tr['P'][0, 0] = self._r(s)
                        tr['P'][0, 1] = tr['P'][1, 0] = 0.
                if best is not None:used.add(best[1]);tr = old[best[1]]
                else:tr = dict(rate=None, last=(stamp, s))
                measured = visible or self.VAGUE_SIGMA is not None
                if measured:
                    self._measure(tr, stamp, s, None if visible else self.VAGUE_SIGMA**2)
                    if self.NEG_RESET is not None and tr['x'][1] < self.NEG_RESET:
                        tr['x'] = np.array([s, self.INIT_SPEED]);tr['P'] = np.diag([self._r(s), self.INIT_SPEED_SIGMA**2])
                        tr['kt'] = stamp;tr['n'] = 1
                    tr['rate'] = self._rate(tr)
                    tr['last'] = (stamp, s)
                    if visible:
                        tr['visible'] = (stamp, s)
                else:
                    t0, s0 = tr['last'];tr['last'] = (stamp, s0+(tr['rate'] or 0.)*max(0., stamp-t0))
                tr['seen'] = stamp
                # Hidden: the speed measured while it was visible holds for HOLD_S (a car behind the one ahead of us
                # has both edges hidden; unknown, it read as standing and braked a change 54 -> 46 km/h).
                held = tr['rate'] if 'visible' in tr and stamp-tr['visible'][0] <= self.HOLD_S else None
                new.append(tr);rates.append(tr['rate'] if measured else held)
                # Before its rate is given: the filter's estimate and its sigma so far (from the 2nd measurement).
                sd = float(np.sqrt(tr['P'][1, 1])) if 'P' in tr else 0.
                if tr.get('n', 0) < self.BIRTH_N:
                    sd = max(sd, self.BIRTH_UNSURE_SD)
                prov.append((float(tr['x'][1]), sd) if measured and tr.get('n', 0) >= 2 else None)
            new += [tr for k, tr in enumerate(old) if k not in used and stamp-tr.get('seen', stamp) <= self.KEEP_S]
            self.tracks[kind] = new;out[kind] = rates;self.provisional[kind] = prov
        return out


def run_speeds(tracker, stamp, runs, ego_s=None, low_sigma=None, max_sigma=None):
    """Speed of each run from its visible edges' rates (mean of the known ones), None if neither is known yet.
    low_sigma: also tracker.lows -- per run without a speed, the slowest it may plausibly be going (provisional
    estimate - low_sigma x its sigma, the lower of its two edges), None if no edge has two measurements yet or its sigma
    is above max_sigma (60 m ahead after 2-3 scans it read ~0, i.e. standing, and a 60 km/h car braked us 0.8 at 111 km/h)."""
    rates = tracker.update(stamp, {'lo': [(r[0], not r[2]) for r in runs], 'hi': [(r[1], not r[3]) for r in runs]}, ego_s)
    out = [];lows = [];highs = []
    for k, (lo_rate, hi_rate) in enumerate(zip(rates['lo'], rates['hi'])):
        known = [x for x in (lo_rate, hi_rate) if x is not None]
        out.append(float(np.mean(known)) if known else None)
        p = [x for x in (tracker.provisional['lo'][k], tracker.provisional['hi'][k])
             if x is not None and (max_sigma is None or x[1] <= max_sigma)] if low_sigma is not None else []
        lows.append(min(v-low_sigma*sd for v, sd in p) if p and not known else None)
        highs.append(max(v+low_sigma*sd for v, sd in p) if p and not known else None)
    tracker.lows = lows;tracker.highs = highs
    return out
