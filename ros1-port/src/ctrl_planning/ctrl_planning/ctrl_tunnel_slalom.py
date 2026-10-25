"""Tunnel slalom over every known obstacle at once (no ROS).

The old tunnel avoidance held one pre-made curve (entry, offset, return) and only looked at bodies on
the reference line. A barricade on that curve was missed and hit (2026-09-29). Here the curve is
thrown away whenever the obstacle picture changes and a new one is planned from where the car is:

    groups   boxes in route order; two with too little room between them to move over
             (min_curve) must be passed with one offset, when one exists
    offsets  every feasible grid candidate per group, including interior gaps between pieces
    moves    between groups a quintic shift; a move is possible when the car body (rotated with the
             curve, +- half incl. margin) sweeping along it overlaps no box. The gentlest (longest)
             such curve is taken, down to the steering limit (max_curvature) and the tracker's
             shortest curve (min_move). No fixed "finished x m before the box" rule: that declared a
             7 m move for a barricade seen late impossible and stopped the car (user 2026-09-29).
    choice   one offset per group from the car's offset now, minimising first the steepest move
             (shift / length^2, ~ lateral acceleration at a given speed), then the lateral travel
    back     the way back to 0 after the last group is the plan's last move, checked the same way.
             No separate return mechanism: the tracker's scheduled return and the "hold the return"
             target cut across the next barricade (user 2026-09-29).
"""
import math
from dataclasses import dataclass, field, replace
from typing import Callable, List, Optional, Sequence

import numpy as np


@dataclass
class Slalom:
    offsets: List[float]                  # one per group actually reachable
    groups: list                          # boxes per group
    fronts: List[float]                   # first box front per group (rear-axle s of the front bumper there)
    clears: List[float]                   # rear-axle s at which the group is passed
    lengths: List[float]                  # curve length of the move into each group (0: no move)
    starts: List[float]                   # where that move starts
    blocked_at: Optional[int] = None      # first group no reachable offset passes (stop before it)
    shifts: List[float] = field(default_factory=list)
    steepest: float = 0.
    why: List[str] = field(default_factory=list)   # blocked: why each candidate failed (judgement log)
    return_blocked: bool = False          # the pass is safe, but no checked way back to the route exists
    per_group: List[float] = field(default_factory=list)   # the offset chosen for each group (before merging)


def group_boxes(boxes: Sequence, clear_s: Callable, front: float, buffer: float, min_curve: float,
                shared: Callable) -> list:
    out = []
    for b in sorted(boxes, key=lambda b: b.s_min):
        if out:
            room = b.s_min - front - buffer - max(clear_s(x) for x in out[-1])
            if room < min_curve and shared(out[-1] + [b]):
                out[-1].append(b)
                continue
        out.append([b])
    return out


def sweep_hits(boxes: Sequence, d0: float, d1: float, start: float, length: float, s_a: float, s_b: float,
               front: float, rear: float, half: float, step: float = .25) -> bool:
    """Does the body following d0 -> d1 (quintic over [start, start+length]) from rear-axle s_a to s_b
    overlap any box? Test the rotated body against its convex outline, or its AABB when absent."""
    if s_b < s_a or not boxes:
        return False
    ss = np.unique(np.r_[np.arange(s_a, s_b, step), s_b])
    u = np.clip((ss - start) / max(length, 1e-6), 0., 1.)
    off = d0 + (d1 - d0) * u**3 * (10 - 15*u + 6*u*u)
    slope = (d1 - d0) * 30 * u*u * (1 - u)**2 / max(length, 1e-6)
    yaw = np.arctan(slope)
    co, si = np.cos(yaw), np.sin(yaw)
    ac, asi = np.abs(co), np.abs(si)
    half_length = (front + rear) / 2.
    center_shift = (front - rear) / 2.
    cs = ss + center_shift*co
    cd = off + center_shift*si
    extent_s = half_length*ac + half*asi
    extent_d = half_length*asi + half*ac
    min_s, max_s = (cs - extent_s).min(), (cs + extent_s).max()
    for b in boxes:
        if b.s_max < min_s or b.s_min > max_s:
            continue
        box_half_s = (b.s_max - b.s_min) / 2.
        box_half_d = (b.d_max - b.d_min) / 2.
        ds = (b.s_min + b.s_max) / 2. - cs
        dd = (b.d_min + b.d_max) / 2. - cd
        # Separating axes of the axis-aligned box and the rotated vehicle body.
        # Projection radii avoid constructing and projecting four corners at every station.
        sep = ((np.abs(ds) >= extent_s + box_half_s)
               | (np.abs(dd) >= extent_d + box_half_d)
               | (np.abs(ds*co + dd*si) >= half_length + box_half_s*ac + box_half_d*asi)
               | (np.abs(-ds*si + dd*co) >= half + box_half_s*asi + box_half_d*ac))
        if sep.all():
            continue
        vertices = getattr(b, 'vertices', ())
        if len(vertices) >= 3:
            # Keep the AABB as a cheap broad phase, then check the observed convex outline.
            active = ~sep
            c, sn = co[active], si[active]
            x, y = cs[active], cd[active]
            polygon = np.asarray(vertices, dtype=float)
            dx = polygon[:, 0, None] - x
            dy = polygon[:, 1, None] - y
            along, across = dx*c + dy*sn, -dx*sn + dy*c
            separated = ((along.min(axis=0) >= half_length)
                         | (along.max(axis=0) <= -half_length)
                         | (across.min(axis=0) >= half)
                         | (across.max(axis=0) <= -half))
            edges = np.roll(polygon, -1, axis=0) - polygon
            edges = edges[np.any(edges != 0., axis=1)]
            axes = np.column_stack((-edges[:, 1], edges[:, 0]))
            projected = polygon @ axes.T
            centers = axes[:, 0, None]*x + axes[:, 1, None]*y
            radii = (half_length*np.abs(axes[:, 0, None]*c + axes[:, 1, None]*sn)
                     + half*np.abs(-axes[:, 0, None]*sn + axes[:, 1, None]*c))
            separated |= ((centers + radii <= projected.min(axis=0)[:, None])
                          | (centers - radii >= projected.max(axis=0)[:, None])).any(axis=0)
            if not separated.all():
                return True
        else:
            return True
    return False


def first_hit(boxes: Sequence, moves: Sequence, d_origin: float, s_from: float, s_to: float,
              front: float, rear: float, half: float, step: float = 1.):
    """(rear-axle s, offset there, box) where the body following the given moves (d, start, length)
    from d_origin first overlaps a box, None when it clears them all up to s_to."""
    prev, segs = d_origin, []
    for d, st, ln in moves:
        segs.append((prev, d, st, ln)); prev = d
    boundaries = sorted(set(np.arange(s_from, s_to, step)) | {s_to}
                        | {st for _, _, st, _ in segs if s_from < st < s_to})
    for s, end in zip(boundaries, boundaries[1:]):
        # the last move started by s (it holds its end offset after it), else the offset it starts from
        seg = next(((d0, d1, st, ln) for d0, d1, st, ln in reversed(segs) if st <= s),
                   (d_origin, d_origin, s, 1.))
        for b in boxes:
            if sweep_hits([b], seg[0], seg[1], seg[2], seg[3], s, end, front, rear, half):
                u = min(1., max(0., (s - seg[2]) / max(seg[3], 1e-6)))
                return float(s), seg[0] + (seg[1] - seg[0]) * u**3 * (10 - 15*u + 6*u*u), b
    return None


def representatives(offsets: Sequence[float], group: Sequence, keep: Sequence[float]) -> List[float]:
    """Retain each feasible offset: interval ends can fail the entry or return sweep."""
    return sorted(set(offsets))


def plan(groups: list, feasible: Callable, s_now: float, d_now: float, v: float, clear_s: Callable,
         front: float, rear: float, half: float, min_curve: float, curve_time: float, min_move: float,
         max_curvature: float = math.inf, ongoing=None, lead: float = 0., hold_tol: float = 1e-6,
         slack: float = 0., ongoing_boxes=None, speed_ok: Optional[Callable] = None,
         range_slack: float = 0., segment_ok: Optional[Callable] = None,
         return_length: Optional[Callable] = None, return_from: Optional[Callable] = None,
         memo: Optional[dict] = None) -> Slalom:
    """ongoing: (d_from, d_to, start, length) of the move being driven. When it still clears every box
    it is kept as is and only what follows is planned: re-planning it as a fresh move from the offset
    reached so far needed min_move of room beside the barricade and stopped the car (2026-09-29).
    lead: pure pursuit steers for a point one lookahead ahead and so takes a curve earlier than drawn
    (corner cutting); the way back to 0 is also checked moved lead metres earlier. Without it the car came
    back from the last barricade early and scraped it in every run; on the moves between barricades it
    ruled out the 10 m gap between the first two (2026-09-29).
    hold_tol: a candidate this close to the offset held (e.g. half the candidate grid) is that offset, held
    as it is (checked at the real offset). A 0.001 m "move" needed min_move of room beside the barricade
    and stopped the car at 2.999 vs 3.0 (2026-09-29).
    Moves are as short as the steering allows (max_curvature); min_move no longer bounds them: a 0.25 m
    correction in front of a barricade needed 6 m and stopped the car 2 m before it (2026-09-29).
    slack: a new curve (and a new hold) clears the boxes by this much more than the body + margin: the box
    edges move by up to this between plans (measured), and a curve drawn to just clear them was found hitting
    on the next plan and thrown away, again and again, until no room was left (2026-09-29, B). The move
    being driven only has to clear them (no slack), including remembered boxes missing from the current scan.
    range_slack: with slack, each box is also widened on both sides by this much per metre it is ahead of the car.
    A box seen from afar is narrower than it is: the second tunnel box read 1.2 m wide at 27 m and 3.5 m at 5 m;
    curves hugging what was seen ended 5 m before it with no room left to reach the offset needed (2026-09-30,
    2 of 5 laps). Measured over every tunnel box: edges grew by up to ~0.03 m per metre of range (p90).
    Each move is checked from where the previous one ended, the stretch still held before it included.
    speed_ok(length, shift, start): the car can be at the speed that curve allows by its start. Moves the car
    can drive come first; only when there is none is any geometric fit taken.
    segment_ok(d0, d1, start, length, s_a, s_b): optional road-body validation for each candidate
    segment, including holds, ongoing moves, and the early return followed by the tracker.
    return_length(offset): existing return-length policy, used as an additional search length.
    return_from(offset): where the way back to 0 may start at the earliest; the offset is held (checked the same
    way) until then. None, or no way back from there: from where the last move ends, as before.
    memo: body/road check results by (boxes, curve, span, slack), shared by the caller's re-plans of one tick (the
    same boxes, s_now and checks). Each curve is checked once, as a lattice checks each edge once: the same curves
    were checked again for every state, for the drivable and then the any-fit pass, and for every re-plan without
    slack or on the other side -- 0.5-4 s plans that left the tracker without a target (2026-10-10)."""
    memo = {} if memo is None else memo
    box_keys = {}
    def hits(boxes_, d0, d1, start, length, s_a, s_b, early=0., extra=slack):
        if id(boxes_) not in box_keys:
            box_keys[id(boxes_)] = (boxes_, tuple((b.s_min, b.s_max, b.d_min, b.d_max, getattr(b, 'vertices', ()))
                                                  for b in boxes_))
        key = (box_keys[id(boxes_)][1], float(d0), float(d1), float(start), float(length), float(s_a), float(s_b),
               float(early), float(extra))
        if key not in memo:
            memo[key] = check(boxes_, d0, d1, start, length, s_a, s_b, early, extra)
        return memo[key]
    def check(boxes_, d0, d1, start, length, s_a, s_b, early, extra):
        h = half + extra
        if extra > 0 and range_slack > 0:
            boxes_ = [replace(b, d_min=b.d_min - range_slack*max(0., b.s_min - s_now),
                              d_max=b.d_max + range_slack*max(0., b.s_min - s_now)) for b in boxes_]
        if (sweep_hits(boxes_, d0, d1, start, length, s_a, s_b, front, rear, h)
                or (early > 0 and sweep_hits(boxes_, d0, d1, start - early, length, s_a, s_b, front, rear, h))):
            return True
        return segment_ok is not None and (not segment_ok(d0, d1, start, length, s_a, s_b)
                or (early > 0 and not segment_ok(d0, d1, start - early, length, s_a, s_b)))
    boxes = [b for g in groups for b in g]
    fronts = [min(b.s_min for b in g) - front for g in groups]
    clears = [max(clear_s(b) for b in g) for g in groups]
    # d -> (steepest, travel, offsets, lengths, starts, s where the last move ends)
    states = {d_now: (0., 0., [], [], [], s_now)}
    if ongoing is not None:
        d_from, d_to, o_start, o_len = ongoing
        o_end = o_start + o_len
        check_to = max([b.s_max for b in boxes], default=o_end) + rear
        span = (s_now, min(check_to, o_end + front + rear))
        if (o_end > s_now and abs(d_to - d_from) > 1e-6
                and not hits(boxes, d_from, d_to, o_start, o_len, *span, extra=0.)
                and (ongoing_boxes is None
                     or not hits(ongoing_boxes, d_from, d_to, o_start, o_len, *span, extra=0.))):
            # Completing an old return is still lateral travel, not a free
            # waypoint at the centre before the next avoidance manoeuvre.
            states[d_to] = (abs(d_to-d_from)/o_len**2, abs(d_to-d_now),
                            [d_to], [o_len], [o_start], o_end)
    # The move being driven, once it has cleared every box over its span, is carried past the groups inside
    # that span even though its end offset is no candidate of theirs (it heads for the next barricade while
    # passing this one): it was dropped every plan and re-made further on until no room was left (2026-09-29).
    committed = None
    if ongoing is not None and ongoing[1] in states and abs(ongoing[1] - d_now) > 1e-9:
        committed = (ongoing[1], span[1])
    blocked_at = None
    why = []
    def gaps(d, extra, g_):
        # lateral room between the body at offset d (half + extra) and each box of the group (negative: overlap)
        h = half + extra
        return ' '.join('box s%.1f d[%+.2f,%+.2f] gap %+.2f' % (b.s_min, b.d_min, b.d_max,
                        max(b.d_min - (d + h), (d - h) - b.d_max)) for b in g_)
    for k, g in enumerate(groups):
        tried = []
        polygon_group = any(getattr(b, 'vertices', ()) for b in g)
        cands = representatives(feasible(g), g, [d_now, 0.] + list(states))
        end_check = max(b.s_max for b in g) + rear
        nxt = {}
        for d in cands:
            ordered_states = sorted(states.items(), key=lambda item: (
                item[1][0], item[1][1] + (0. if abs(d - item[0]) < hold_tol else abs(d - item[0]))))
            for dp, (worst, travel, path, lengths, starts, ready) in ordered_states:
                shift, d_to = abs(d - dp), d
                target = dp if shift < hold_tol else d
                travel_bound = travel + (0. if shift < hold_tol else shift)
                # Every completed move has cost >= this bound. Such a candidate cannot
                # replace the existing state under the same strict cost comparison below.
                if target in nxt and (worst, travel_bound) >= nxt[target][:2]:
                    continue
                if shift < hold_tol:
                    # holding the offset the car is at now needs no slack (it is already there)
                    now = (abs(dp - d_now) < 1e-9 and abs(ready - s_now) < 1e-9) or (
                        committed is not None and abs(dp - committed[0]) < 1e-9 and len(path) == 1 and ongoing is not None and starts[0] == ongoing[2])
                    if hits(boxes, dp, dp, ready, 1., ready, end_check, extra=0. if now else slack):
                        tried.append('hold %+.2f (%s, slack %.2f): %s' % (dp, 'at it' if now else 'not at it',
                                     0. if now else slack, gaps(dp, 0. if now else slack, boxes)))
                        continue
                    d_to, shift = dp, 0.
                    move = (0., 0., ready, ready)
                else:
                    move = None
                    lo = max(1., math.sqrt(5.77*shift/max_curvature) if math.isfinite(max_curvature) else min_move)
                    # Search the available approach as well as the speed-based length: a short
                    # slow-speed curve can swing the front corner outside otherwise usable pavement.
                    hi = max(lo, v*curve_time, fronts[k] - ready)
                    if polygon_group:
                        # An angled outline may leave room to finish beside its empty AABB corner.
                        hi = max(hi, clears[k] - ready)
                    # The move is complete before the front bumper reaches the box (1.5 m short of it first,
                    # then at it), at any length the steering allows; only when no such move exists may it
                    # end past the box front. Longest-first over all end points picked a 17.4 m move ending
                    # 2 m past a 3 m wide barricade: the car was still swerving, 0.26 m clear, at its face
                    # and hit it (2026-09-29, full lap s1961).
                    # Only a move the car can drive at the speed it can reach counts first (2026-09-29 CP14:
                    # a 6 m move entered at 50 km/h where it allowed 25 moved the car 0.3 of 0.75 m and passed
                    # a 3 m wide barricade 0.06 m clear; a 12 m move ending past its face was drivable).
                    # Drivable ones: complete earliest first. None drivable: the gentlest (longest) that fits,
                    # the one the car follows best, wherever it ends.
                    ends = (fronts[k] - 1.5, fronts[k], fronts[k] + 2., fronts[k] + 4.)
                    sizes = np.r_[np.arange(hi, lo, -1.), lo]
                    if polygon_group:
                        sizes = np.unique(np.r_[hi, np.arange(math.floor(hi), lo, -1.), lo])[::-1]
                    orders = ([(e, l, True) for e in ends for l in sizes] if speed_ok is not None else []) + \
                             [(e, l, False) for l in sizes for e in ends]
                    if polygon_group:
                        # Starting now supplies physical candidates beyond the legacy front offsets.
                        entry_starts = np.arange(ready, max(ready, fronts[k]) + 1e-6, 1.)
                        orders = ([(e, l, True) for e in ends for l in sizes]
                                  + [(st+l, l, True) for st in entry_starts for l in sizes]
                                  if speed_ok is not None else []) + \
                                 [(e, l, False) for l in sizes for e in (*ends, *(st+l for st in entry_starts))]
                    # A drivable move that starts once the previous group is passed comes first: the longest curve
                    # started beside the barricade still being passed, curving back toward it, and was thrown away
                    # on the next scan as the barricade read a little wider (2026-10-10, both runs, last obstacle).
                    if k > 0 and speed_ok is not None:
                        orders = [o for o in orders if o[2] and o[0] - o[1] >= clears[k-1] - 1e-6] + orders
                    for end, length, drivable in orders:
                        start = end - length
                        if start < ready - 1e-6:
                            continue
                        if drivable and not speed_ok(float(length), shift, start):
                            continue
                        check_to = max(end_check, end) if polygon_group else end_check
                        if not hits(boxes, dp, d, start, length, ready, check_to):
                            # A curve the car can drive at its speed costs no steepness: among drivable plans the least
                            # lateral travel wins. Gentlest-first sent the car 5.25 m out on a 29.5 m sweep past a parked
                            # car that 3.25 m on a drivable 14 m curve passed (2026-10-10).
                            move = (0. if drivable else shift/length**2, float(length), start, end)
                            break
                    if move is None:
                        tried.append('move %+.2f->%+.2f shift %.2f: no curve %.0f-%.0f m from s%.1f clears (front s%.1f)' % (
                                     dp, d, shift, lo, hi, ready, fronts[k]))
                        continue
                need, length, start, end = move
                cand = (max(worst, need), travel + shift, path + [d_to], lengths + [length], starts + [start],
                        max(end, ready))
                if d_to not in nxt or cand[:2] < nxt[d_to][:2]:
                    nxt[d_to] = cand
        if committed is not None and committed[0] in states and end_check <= committed[1] + 1e-6:
            c = states[committed[0]]
            if committed[0] not in nxt or c[:2] < nxt[committed[0]][:2]:
                nxt[committed[0]] = c
        if not nxt:
            blocked_at = k
            why = ['candidates %s from %s' % ([round(c, 2) for c in cands], [round(x, 2) for x in states])] + tried
            break
        states = nxt
    # Finish each reachable state before comparing costs: a cheap entry can have an unsafe return.
    completed = {}
    state_rank = {d: rank for rank, d in enumerate(states)}

    def final_cost(last, state):
        # Preserve the original states order when rounded curvature and travel tie.
        return round(state[0], 6), state[1] + .5*abs(last), state_rank[last]

    if blocked_at is None:
        returns = []
        for last, state in states.items():
            worst, travel, path, lengths, starts, ready = state
            shift = abs(last)
            if shift <= 1e-6:
                returns.append((final_cost(last, state), last, state, 0., 0.))
                continue
            lo = max(1., math.sqrt(5.77*shift/max_curvature) if math.isfinite(max_curvature) else min_move)
            pref = max(lo, v*curve_time, min_curve, max(lengths, default=0.),
                       return_length(last) if return_length is not None else 0.)
            # Every searched length is <= pref, so no return can beat this bound.
            bound = (round(max(worst, shift/pref**2), 6),
                     travel + shift + .5*shift, state_rank[last])
            returns.append((bound, last, state, lo, pref))
        best_completed = None
        for bound, last, state, lo, pref in sorted(returns, key=lambda candidate: candidate[0]):
            if best_completed is not None and bound >= best_completed:
                continue
            worst, travel, path, lengths, starts, ready = state
            shift = abs(last)
            if shift <= 1e-6:
                completed[last] = state
                best_completed = final_cost(last, state)
                continue
            horizon = max([b.s_max for b in boxes], default=ready) + rear + front
            done = None
            late = return_from(last) if return_from is not None else math.inf
            for first in ((max(ready, late),) if math.isfinite(late) else ()) + (ready,):
                for length in np.r_[np.arange(pref, lo, -1.), lo]:
                    for start in np.r_[np.arange(first, max(first, horizon), 1.), max(first, horizon)]:
                        if not hits(boxes, last, 0., start, length, min(ready, start - lead),
                                    max(horizon, start + length + front + rear), lead):
                            done = (float(length), float(start))
                            break
                    if done:
                        break
                if done:
                    break
            if done:
                length, start = done
                completed[last] = (max(worst, shift/length**2), travel + shift,
                                   path + [0.], lengths + [length], starts + [start], start + length)
                cost = final_cost(last, completed[last])
                best_completed = cost if best_completed is None else min(best_completed, cost)
    choices = completed or states
    best_key, best = min(choices.items(), key=lambda kv: final_cost(*kv))
    offsets, lengths, starts = list(best[2]), list(best[3]), list(best[4])
    # states hold each chain before its way back: its last len(groups) offsets are one per group
    per_group = list(states[best_key][2][-len(groups):]) if groups and blocked_at is None else []
    last = offsets[-1] if offsets else d_now
    first_from = ongoing[0] if (ongoing is not None and offsets and lengths and starts
                                and abs(starts[0] - ongoing[2]) < 1e-6 and abs(lengths[0] - ongoing[3]) < 1e-6) else d_now
    # A centre waypoint between two avoidance offsets has no intrinsic value.
    # Replace the pair by one smooth connector only after the same body, road,
    # curvature and speed checks. Never rewrite a segment already being driven.
    i = 0
    while i + 1 < len(offsets):
        origin = first_from if i == 0 else offsets[i-1]
        start = starts[i]
        end = starts[i+1] + lengths[i+1]
        length = end-start
        shift = abs(offsets[i+1]-origin)
        if (abs(offsets[i]) < 1e-9 and abs(origin) > 1e-9
                and abs(offsets[i+1]) > 1e-9 and start >= s_now and length > 0.
                and 5.77*shift/length**2 <= max_curvature
                and (speed_ok is None or speed_ok(length,shift,start))
                and not hits(boxes,origin,offsets[i+1],start,length,
                             s_now if i == 0 else starts[i-1]+lengths[i-1],
                             max(end+front+rear,max((b.s_max for b in boxes),default=end)+rear))):
            offsets[i:i+2] = [offsets[i+1]]
            starts[i:i+2] = [start]
            lengths[i:i+2] = [length]
        else:
            i += 1
    prev = [first_from] + offsets[:-1]
    return Slalom(offsets, groups, fronts, clears, lengths, starts, blocked_at,
                  [abs(a - b) for a, b in zip(offsets, prev)], best[0], why, blocked_at is None and not completed and abs(last) > 1e-6,
                  per_group)
