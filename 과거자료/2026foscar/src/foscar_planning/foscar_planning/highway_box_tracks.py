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
    """Rear ('lo') and front ('hi') edges of the runs in one lane, followed by position; rate = speed along the lane."""
    MAX_SPEED = 40.     # m/s an edge may move between scans before its rate is known
    KEEP_S = .3         # an edge not seen (hidden, or a missed scan) is kept this long
    HOLD_S = 1.         # a hidden edge keeps the speed measured while it was visible this long
    JUMP_M = 5.         # an edge that jumps up to a car length (a fragment joined or left the outline) keeps its motion

    def __init__(self, window):
        self.window = window;self.tracks = {'lo': [], 'hi': []}

    def update(self, stamp, edges):
        """edges: {'lo': [(station, visible)], 'hi': [...]} -> {'lo': [rate or None], 'hi': [...]} (same order)."""
        out = {}
        for kind, values in edges.items():
            old = self.tracks[kind];used = set();new = [];rates = []
            for s, visible in values:
                best = None
                for k, tr in enumerate(old):
                    if k in used:continue
                    t0, s0 = tr['last'];dt = max(0., stamp-t0)
                    pred = s0+(tr['rate'] or 0.)*dt
                    gate = 1.5+(5. if tr['rate'] is not None else self.MAX_SPEED)*dt
                    d = abs(s-pred)
                    if d <= gate and (best is None or d < best[0]):best = (d, k)
                if best is None and visible:
                    # The outline jumped, not the car: a rear fragment joining 4 m behind a lead restarted its rear
                    # edge, the lead read as standing and braked 0.5 (2026-09-30 run 6, s1187). Follow the nearest
                    # measured edge within a car length and shift its samples by the jump, keeping its rate.
                    for k, tr in enumerate(old):
                        if k in used or tr['rate'] is None:continue
                        t0, s0 = tr['last'];d = abs(s-(s0+tr['rate']*max(0., stamp-t0)))
                        if d <= self.JUMP_M and (best is None or d < best[0]):best = (d, k)
                    if best is not None:
                        tr = old[best[1]];t0, s0 = tr['last'];shift = s-(s0+tr['rate']*max(0., stamp-t0))
                        tr['samples'] = [(t, x+shift) for t, x in tr['samples']]
                if best is not None:used.add(best[1]);tr = old[best[1]]
                else:tr = dict(samples=[], rate=None, last=(stamp, s))
                if visible:
                    if not tr['samples'] or tr['samples'][-1][0] < stamp-1e-6:tr['samples'].append((stamp, s))
                    tr['samples'] = [x for x in tr['samples'] if stamp-x[0] <= self.window]
                    tr['rate'] = self._fit(tr['samples'])
                    tr['last'] = (stamp, s)
                else:
                    t0, s0 = tr['last'];tr['last'] = (stamp, s0+(tr['rate'] or 0.)*max(0., stamp-t0))
                tr['seen'] = stamp
                # Hidden: the speed measured while it was visible holds for HOLD_S (a car behind the one ahead of us
                # has both edges hidden; unknown, it read as standing and braked a change 54 -> 46 km/h).
                held = tr['rate'] if tr['samples'] and stamp-tr['samples'][-1][0] <= self.HOLD_S else None
                new.append(tr);rates.append(tr['rate'] if visible else held)
            new += [tr for k, tr in enumerate(old) if k not in used and stamp-tr.get('seen', stamp) <= self.KEEP_S]
            self.tracks[kind] = new;out[kind] = rates
        return out

    @staticmethod
    def _fit(samples):
        if len(samples) < 3 or samples[-1][0]-samples[0][0] < .2:return None
        t = np.array([x[0] for x in samples]);s = np.array([x[1] for x in samples])
        return float(np.polyfit(t-t[0], s, 1)[0])


def run_speeds(tracker, stamp, runs):
    """Speed of each run from its visible edges' rates (mean of the known ones), None if neither is known yet."""
    rates = tracker.update(stamp, {'lo': [(r[0], not r[2]) for r in runs], 'hi': [(r[1], not r[3]) for r in runs]})
    out = []
    for lo_rate, hi_rate in zip(rates['lo'], rates['hi']):
        known = [x for x in (lo_rate, hi_rate) if x is not None]
        out.append(float(np.mean(known)) if known else None)
    return out
