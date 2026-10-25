"""Tunnel slalom as a side choice per obstacle group (no ROS). Same inputs and output as tunnel_slalom.plan.

tunnel_slalom.plan searches every candidate offset x every way to reach it x every curve length and start, sweeping the
rotated body each time: 0.05 s on a good tick, 1-5 s when candidates fail, and the node stopped publishing meanwhile
(2026-10-10). Here the question is made small first, as the lidar's points were made BEV cells and boxes:

    point    boxes grown by the body (half width + margin sideways, front/rear overhang along the road): the car is
             its rear axle, a point; a curve's heading widens the growth by front x slope
    sides    per group: pass it on the left or on the right, at the feasible offset of that side nearest the offset the
             car comes from -- 2^n combinations for n groups (n <= 4 in the tunnel)
    moves    between groups one quintic; its length is the window left between the groups (from where the previous
             one is passed to where the next one starts), at least what the steering and the speed allow
    check    a move or hold is clear when its points stay out of the grown boxes (array compares), then the chosen
             chain once through segment_ok (mapped pavement, BEV cells) -- the next best when it fails
    choice   drivable chains first, then the least lateral travel; the offset is held, the way back starts at
             return_from (the tunnel end) and earlier only when that fails
"""
import itertools
import math
from typing import Callable, Optional

import numpy as np

from .tunnel_slalom import Slalom

STEP = .5          # m between checked points
MAX_GROUPS = 4     # sides decided for the nearest groups; further ones join the next plan


def _profile(d0, d1, start, length, ss):
    u = np.clip((ss - start)/max(length, 1e-6), 0., 1.)
    d = d0 + (d1 - d0)*u**3*(10 - 15*u + 6*u*u)
    slope = (d1 - d0)*30*u*u*(1 - u)**2/max(length, 1e-6)
    return d, slope


def plan(groups: list, feasible: Callable, s_now: float, d_now: float, v: float, clear_s: Callable,
         front: float, rear: float, half: float, min_curve: float, curve_time: float, min_move: float,
         max_curvature: float = math.inf, ongoing=None, lead: float = 0., hold_tol: float = 1e-6,
         slack: float = 0., ongoing_boxes=None, speed_ok: Optional[Callable] = None,
         range_slack: float = 0., segment_ok: Optional[Callable] = None,
         return_length: Optional[Callable] = None, return_from: Optional[Callable] = None,
         memo: Optional[dict] = None, move_from: float = -math.inf) -> Slalom:
    groups = list(groups)[:MAX_GROUPS]
    boxes = [b for g in groups for b in g]
    fronts = [min(b.s_min for b in g) - front for g in groups]
    clears = [max(clear_s(b) for b in g) for g in groups]
    # grown boxes as arrays: s span of the rear axle that puts the body over the box, d span with half + slack, wider
    # by range_slack per metre ahead (a box seen from afar reads narrow). It grows away from the route: from afar the
    # part near the path is what is seen, the rest toward the wall comes later (slalom_side, 2026-09-30). Grown toward
    # us too, a deer standing at the right edge pushed the car left early, then again for the next box (2026-10-10).
    # A box across the centre line grows both ways.
    # Each box as points along its outline (its AABB without one): what the body covers at a rear-axle s is the outline
    # between s - rear and s + front, so a corner piece of a car seen at an angle is not its whole bounding box (the AABB
    # read 0.3-0.5 m wider up close and blocked an offset already held, 2026-10-10)
    outlines = []
    for b in boxes:
        corners = np.asarray(b.vertices if len(getattr(b, 'vertices', ())) >= 3 else
                             ((b.s_min, b.d_min), (b.s_max, b.d_min), (b.s_max, b.d_max), (b.s_min, b.d_max)), float)
        edges = zip(corners, np.roll(corners, -1, axis=0))
        pts = np.vstack([np.linspace(a, c, max(2, int(np.hypot(*(c - a))/.2) + 1)) for a, c in edges])
        g = range_slack*max(0., b.s_min - s_now)
        outlines.append((pts[:, 0], pts[:, 1], b.d_min > 0., b.d_max < 0., g))
    width = half + slack

    def clear(d0, d1, start, length, s_a, s_b, extra=None):
        """The body (half + slack, wider by front x slope on a curve) over the rear-axle points of d0 -> d1 (quintic
        over [start, start+length]) from s_a to s_b meets no outline. extra: slack instead of the plan's."""
        if not boxes or s_b <= s_a:
            return True
        sl = slack if extra is None else extra
        ss = np.r_[np.arange(s_a, s_b, STEP), s_b]
        for st in ((start, start - lead) if lead > 0 and abs(d1 - d0) > 1e-9 else (start,)):
            d, slope = _profile(d0, d1, st, length, ss)
            w = half + sl
            for ps, pd, left, right, g in outlines:
                u = ps[None, :] - ss[:, None]                 # where along the body (from the rear axle) each point is
                window = (u >= -rear) & (u <= front)
                hit = window.any(axis=1)
                if not hit.any():
                    continue
                # in the body's frame: the body side at u is d + u x slope (turned about the rear axle). Widened by
                # front x slope on both sides for the whole body, 0.13 m of room beside a barricade was unusable
                # (2026-10-10).
                q = pd[None, :] - u*slope[:, None]
                lo = np.where(window, q, np.inf).min(axis=1)
                hi = np.where(window, q, -np.inf).max(axis=1)
                if sl > 0:
                    # grown away from the route only (both ways across the centre line)
                    lo = lo - (0. if left else g); hi = hi + (0. if right else g)
                if np.any(hit & (d + w > lo) & (d - w < hi)):
                    return False
        return True

    def deficit(end_check):
        """How much of half + slack the car's own offset now lacks against the first group (0 when it has it)."""
        for short in np.arange(0., half + slack, .02):
            if clear(d_now, d_now, s_now, 1., s_now, end_check, slack - short):
                return short
        return half + slack

    def min_length(shift):
        # as short as the steering allows, as tunnel_slalom.plan: min_move bounding it asked 6 m for a 0.13 m
        # correction 1.3 m before a barricade and the car stopped there (2026-09-29, again here 2026-10-10)
        lo = math.sqrt(5.77*shift/max_curvature) if math.isfinite(max_curvature) else min_move
        return max(1., lo)

    def move(d0, d1, ready, after, end_by, end_check, extra=None, checked_to=-math.inf):
        """(start, length, drivable) of the move d0 -> d1 that starts at or after `after` (the previous group passed)
        and ends by end_by, else starting earlier down to `ready`; None when no such move is clear."""
        shift = abs(d1 - d0)
        lo = min_length(shift)
        best = None
        for end in (end_by - 1.5, end_by, end_by + 2.):
            starts = [max(ready, after)] + [s for s in np.arange(max(ready, after) - 2., ready - 1e-6, -2.)]
            for start in starts:
                length = end - start
                if length < lo:
                    continue
                # the stretch the previous group's own check covered is not checked again (with another slack: a hold
                # 'at it' passed without slack, then failed here with it, 2026-10-10) unless this move starts there
                if not clear(d0, d1, start, length, max(ready, min(start, checked_to)), max(end_check, end + front + rear),
                             extra):
                    continue
                drivable = speed_ok is None or speed_ok(float(length), shift, float(start))
                if drivable:
                    return float(start), float(length), True
                if best is None:
                    best = (float(start), float(length), False)
        return best

    def sides(g):
        cands = sorted(set(feasible(g)))
        centre = (min(b.d_min for b in g) + max(b.d_max for b in g))/2.
        left = [d for d in cands if d >= centre]
        right = [d for d in cands if d < centre]
        return left, right

    side_cands = [sides(g) for g in groups]

    # No move starts before move_from (the tunnel entry: lane changes outside it are scored, user 2026-10-10) when our
    # line is clear up to there; an obstacle before it is avoided as before. A parked car seen 35 m ahead started a 3 m
    # move 4-10 m before the tunnel.
    first_ready = s_now
    if move_from > s_now and clear(d_now, d_now, s_now, 1., s_now, move_from + front + rear, 0.):
        first_ready = move_from

    def preferred(choice):
        """Per group the offset to try first, from the last group back: the next group's offset when it passes this
        group too (one move for both instead of a small one and then the rest, 2026-10-10), else the offset of this
        side nearest it; the last group's nearest 0."""
        want, nxt = [None]*len(groups), 0.
        for k in range(len(groups) - 1, -1, -1):
            options = side_cands[k][0 if choice[k] > 0 else 1]
            if options:
                nxt = want[k] = min(options, key=lambda x: (abs(x - nxt) > hold_tol, abs(x - nxt)))
        return want

    def chain(choice):
        """Offsets, moves and costs for one side per group; (k, None) when group k cannot be reached."""
        want = preferred(choice)
        d_prev, ready = d_now, first_ready
        checked_to = -math.inf              # how far the previous group's check went
        offsets, starts, lengths = [], [], []
        steep, travel = 0., 0.
        for k, g in enumerate(groups):
            options = side_cands[k][0 if choice[k] > 0 else 1]
            end_check = max(b.s_max for b in g) + rear
            after = clears[k-1] if k > 0 else s_now
            m = None
            # the offset of that side nearest the one we come from that is clear
            for d in sorted(options, key=lambda x: (abs(x - d_prev) > hold_tol,
                                                    abs(x - want[k]) > hold_tol, abs(x - d_prev))):
                if abs(d - d_prev) < hold_tol:
                    at_it = abs(d_prev - d_now) < 1e-9 and abs(ready - s_now) < 1e-9
                    if clear(d_prev, d_prev, ready, 1., ready, end_check, 0. if at_it else None):
                        m = 'hold'; break
                    continue
                if (k == 0 and ongoing is not None and abs(ongoing[1] - d) < hold_tol
                        and clear(ongoing[0], ongoing[1], ongoing[2], ongoing[3], s_now, end_check)):
                    m, d = (ongoing[2], ongoing[3], True), ongoing[1]      # the move being driven goes on
                    break
                m = move(d_prev, d, ready, after, fronts[k], end_check, None, checked_to)
                if m is None and k == 0 and abs(d_prev - d_now) < 1e-9:
                    # Already closer than the margin where the car is: a move that makes it no worse is fine. Standing
                    # 0.03 m inside it beside a parked car, every move from there 'hit' and the car never left
                    # (2026-10-10).
                    m = move(d_prev, d, ready, after, fronts[k], end_check, -deficit(end_check))
                if m is not None:
                    break
            if m is None:
                return k, None
            if m == 'hold':
                # ready stays: the next move may start beside this group when it clears it (moving away from a box on
                # the other side); starting once it is passed is only preferred (move's `after`). Pushed to the group's
                # end, a 3.6 m move to pass a parked car after a deer had 10.4 m of the 10.7 m it needed (2026-10-10).
                offsets.append(d_prev); starts.append(ready); lengths.append(0.)
                checked_to = end_check
                continue
            start, length, drivable = m
            if not drivable:
                steep = max(steep, abs(d - d_prev)/length**2)
            travel += abs(d - d_prev)
            offsets.append(d); starts.append(start); lengths.append(length)
            d_prev, ready = d, start + length
            checked_to = max(end_check, start + length + front + rear)
        return len(groups), (offsets, starts, lengths, steep, travel, d_prev, ready, checked_to)

    def way_back(last, ready, checked_to=-math.inf):
        """(start, length) back to 0 after the last group: from return_from on, else as early as clear."""
        if abs(last) < 1e-6:
            return 0., 0.
        lo = min_length(abs(last))
        pref = max(lo, v*curve_time, min_curve, return_length(last) if return_length is not None else 0.)
        horizon = max([b.s_max for b in boxes], default=ready) + rear + front
        late = return_from(last) if return_from is not None else math.inf
        firsts = ([max(ready, late)] if math.isfinite(late) else []) + [ready]
        for first in firsts:
            for length in np.r_[np.arange(pref, lo, -2.), lo]:
                for start in np.r_[np.arange(first, max(first, horizon), 2.), max(first, horizon)]:
                    if clear(last, 0., start, length, max(min(ready, start), min(start, checked_to)),
                             max(horizon, start + length + front + rear)):
                        return float(start), float(length)
        return None

    def road_ok(offsets, starts, lengths, back):
        if segment_ok is None:
            return True
        prev = d_now
        for d, st, ln in zip(offsets, starts, lengths):
            if ln > 0 and not segment_ok(prev, d, st, ln, max(s_now, st - front - rear), st + ln + front + rear):
                road_fail.append('move %+.2f->%+.2f s%.1f+%.1f' % (prev, d, st, ln))
                return False
            prev = d
        if back is not None and back[1] > 0:
            st, ln = back
            if not segment_ok(prev, 0., st, ln, st, st + ln + front + rear):
                road_fail.append('back %+.2f s%.1f+%.1f' % (prev, st, ln))
                return False
        return True

    results, reach = [], 0
    road_fail = []   # which move failed the pavement / BEV check (judgement log)
    for choice in itertools.product((1, -1), repeat=len(groups)):
        k, c = chain(choice)
        reach = max(reach, k)
        if c is None:
            continue
        offsets, starts, lengths, steep, travel, last, ready, checked = c
        back = way_back(last, ready, checked)
        moves = sum(1 for ln in c[2] if ln > 0)
        results.append(((steep, round(travel + .5*abs(last), 6), back is None, moves), choice, c, back))
    results.sort(key=lambda r: r[0])
    for cost, choice, c, back in results:
        offsets, starts, lengths, steep, travel, last, ready, checked = c
        if not road_ok(offsets, starts, lengths, back):
            continue
        per_group = list(offsets)
        if back is not None and back[1] > 0:
            offsets = offsets + [0.]; starts = starts + [back[0]]; lengths = lengths + [back[1]]
        prev = [ongoing[0] if (ongoing is not None and starts and abs(starts[0] - ongoing[2]) < 1e-6) else d_now] + offsets[:-1]
        return Slalom(offsets, groups, fronts, clears, lengths, starts, None,
                      [abs(a - b) for a, b in zip(offsets, prev)], steep, [], back is None and abs(last) > 1e-6, per_group)
    if not groups:
        back = way_back(d_now, s_now)
        if back is None:
            return Slalom([], [], [], [], [], [], None, [], 0., [], abs(d_now) > 1e-6, [])
        offs = [0.] if back[1] > 0 else []
        return Slalom(offs, [], [], [], [back[1]] if offs else [], [back[0]] if offs else [], None,
                      [abs(d_now)] if offs else [], 0., [], False, [])
    why = ['sides %s: %s' % ([sorted(set(l + r)) for l, r in side_cands], 'no clear chain' if not results else
           'no chain on mapped pavement: ' + '; '.join(road_fail[:4]))]
    return Slalom([], groups, fronts, clears, [], [], min(reach, len(groups) - 1) if not results else 0,
                  [], 0., why, False, [])
