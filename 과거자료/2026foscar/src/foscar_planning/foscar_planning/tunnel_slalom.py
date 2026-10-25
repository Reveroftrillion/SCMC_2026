"""Tunnel slalom over every known obstacle at once (no ROS).

The old tunnel avoidance held one pre-made curve (entry, offset, return) and only looked at bodies on
the reference line. A barricade on that curve was missed and hit (2026-09-29). Here the curve is
thrown away whenever the obstacle picture changes and a new one is planned from where the car is:

    groups   boxes in route order; two with too little room between them to move over
             (min_curve) must be passed with one offset, when one exists
    offsets  per group a few candidates: hugging it on either side, the offset now, and 0
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
    overlap any box? Separating-axis test of the rotated body against each axis-aligned box."""
    if s_b < s_a or not boxes:
        return False
    ss = np.arange(s_a, s_b + 1e-9, step)
    u = np.clip((ss - start) / max(length, 1e-6), 0., 1.)
    off = d0 + (d1 - d0) * u**3 * (10 - 15*u + 6*u*u)
    slope = (d1 - d0) * 30 * u*u * (1 - u)**2 / max(length, 1e-6)
    yaw = np.arctan(slope)
    co, si = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
    a = np.array([-rear, -rear, front, front])[None, :]
    w = np.array([-half, half, -half, half])[None, :]
    cs = ss[:, None] + a*co - w*si
    cd = off[:, None] + a*si + w*co
    for b in boxes:
        if b.s_max < s_a - rear or b.s_min > s_b + front:
            continue
        sep = (cs.max(1) <= b.s_min) | (cs.min(1) >= b.s_max) | (cd.max(1) <= b.d_min) | (cd.min(1) >= b.d_max)
        bs = np.array([b.s_min, b.s_min, b.s_max, b.s_max])[None, :]
        bd = np.array([b.d_min, b.d_max, b.d_min, b.d_max])[None, :]
        for ax, ay in ((co, si), (-si, co)):
            cp = cs*ax + cd*ay
            bp = bs*ax + bd*ay
            sep |= (cp.max(1) <= bp.min(1)) | (bp.max(1) <= cp.min(1))
        if not sep.all():
            return True
    return False


def first_hit(boxes: Sequence, moves: Sequence, d_origin: float, s_from: float, s_to: float,
              front: float, rear: float, half: float, step: float = 1.):
    """(rear-axle s, offset there, box) where the body following the given moves (d, start, length)
    from d_origin first overlaps a box, None when it clears them all up to s_to."""
    prev, segs = d_origin, []
    for d, st, ln in moves:
        segs.append((prev, d, st, ln)); prev = d
    for s in np.arange(s_from, s_to, step):
        # the last move started by s (it holds its end offset after it), else the offset it starts from
        seg = next(((d0, d1, st, ln) for d0, d1, st, ln in reversed(segs) if st <= s),
                   (d_origin, d_origin, s, 1.))
        for b in boxes:
            if sweep_hits([b], seg[0], seg[1], seg[2], seg[3], s, s + step, front, rear, half):
                u = min(1., max(0., (s - seg[2]) / max(seg[3], 1e-6)))
                return float(s), seg[0] + (seg[1] - seg[0]) * u**3 * (10 - 15*u + 6*u*u), b
    return None


def representatives(offsets: Sequence[float], group: Sequence, keep: Sequence[float]) -> List[float]:
    """A few offsets worth trying: hugging the group on each side, plus the given ones when feasible."""
    offsets = sorted(offsets)
    if not offsets:
        return []
    mid = (min(b.d_min for b in group) + max(b.d_max for b in group)) / 2.
    left, right = [d for d in offsets if d > mid], [d for d in offsets if d < mid]
    out = ([min(left)] if left else []) + ([max(right)] if right else [])
    for d in keep:
        near = min(offsets, key=lambda x: abs(x - d))
        if abs(near - d) < .13:
            out.append(near)
    return sorted(set(out))


def plan(groups: list, feasible: Callable, s_now: float, d_now: float, v: float, clear_s: Callable,
         front: float, rear: float, half: float, min_curve: float, curve_time: float, min_move: float,
         max_curvature: float = math.inf, ongoing=None, lead: float = 0., hold_tol: float = 1e-6,
         slack: float = 0., ongoing_boxes=None, speed_ok: Optional[Callable] = None,
         range_slack: float = 0.) -> Slalom:
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
    being driven only has to clear them (no slack), or ongoing_boxes (what the lidar shows now) when given.
    range_slack: with slack, each box is also widened on both sides by this much per metre it is ahead of the car.
    A box seen from afar is narrower than it is: the second tunnel box read 1.2 m wide at 27 m and 3.5 m at 5 m;
    curves hugging what was seen ended 5 m before it with no room left to reach the offset needed (2026-09-30,
    2 of 5 laps). Measured over every tunnel box: edges grew by up to ~0.03 m per metre of range (p90).
    Each move is checked from where the previous one ended, the stretch still held before it included.
    speed_ok(length, shift, start): the car can be at the speed that curve allows by its start. Moves the car
    can drive come first; only when there is none is any geometric fit taken."""
    def hits(boxes_, d0, d1, start, length, s_a, s_b, early=0., extra=slack):
        h = half + extra
        if extra > 0 and range_slack > 0:
            boxes_ = [replace(b, d_min=b.d_min - range_slack*max(0., b.s_min - s_now),
                              d_max=b.d_max + range_slack*max(0., b.s_min - s_now)) for b in boxes_]
        return (sweep_hits(boxes_, d0, d1, start, length, s_a, s_b, front, rear, h)
                or (early > 0 and sweep_hits(boxes_, d0, d1, start - early, length, s_a, s_b, front, rear, h)))
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
                and (not hits(boxes, d_from, d_to, o_start, o_len, *span, extra=0.)
                     or (ongoing_boxes is not None
                         and not hits(ongoing_boxes, d_from, d_to, o_start, o_len, *span, extra=0.)))):
            states[d_to] = (0., 0., [d_to], [o_len], [o_start], o_end)
    # The move being driven, once it has cleared every box over its span, is carried past the groups inside
    # that span even though its end offset is no candidate of theirs (it heads for the next barricade while
    # passing this one): it was dropped every plan and re-made further on until no room was left (2026-09-29).
    committed = None
    if ongoing is not None and ongoing[1] in states and abs(ongoing[1] - d_now) > 1e-9:
        committed = (ongoing[1], span[1])
    blocked_at = None
    for k, g in enumerate(groups):
        cands = representatives(feasible(g), g, [d_now, 0.] + list(states))
        end_check = max(b.s_max for b in g) + rear
        nxt = {}
        for d in cands:
            for dp, (worst, travel, path, lengths, starts, ready) in states.items():
                shift, d_to = abs(d - dp), d
                if shift < hold_tol:
                    # holding the offset the car is at now needs no slack (it is already there)
                    now = (abs(dp - d_now) < 1e-9 and abs(ready - s_now) < 1e-9) or (
                        committed is not None and abs(dp - committed[0]) < 1e-9 and not travel and not worst)
                    if hits(boxes, dp, dp, ready, 1., ready, end_check, extra=0. if now else slack):
                        continue
                    d_to, shift = dp, 0.
                    move = (0., 0., ready, ready)
                else:
                    move = None
                    lo = max(1., math.sqrt(5.77*shift/max_curvature) if math.isfinite(max_curvature) else min_move)
                    # Preferred length grows with speed (curve_time s of travel; user 2026-09-29): short
                    # and sharp when slow, long and gentle when fast; shorter down to lo when there is no room.
                    hi = max(lo, v*curve_time)
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
                    sizes = np.arange(hi, lo - 1e-6, -1.)
                    orders = ([(e, l, True) for e in ends for l in sizes] if speed_ok is not None else []) + \
                             [(e, l, False) for l in sizes for e in ends]
                    for end, length, drivable in orders:
                        start = end - length
                        if start < ready - 1e-6:
                            continue
                        if drivable and not speed_ok(float(length), shift, start):
                            continue
                        if not hits(boxes, dp, d, start, length, ready, end_check):
                            move = (shift/length**2, float(length), start, end)
                            break
                    if move is None:
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
            break
        states = nxt
    best = min(states.items(), key=lambda kv: (round(kv[1][0], 6), kv[1][1] + .5*abs(kv[0])))[1]
    offsets, lengths, starts = list(best[2]), list(best[3]), list(best[4])
    last = offsets[-1] if offsets else d_now
    if blocked_at is None and abs(last) > 1e-6:
        # Back to 0: earliest start (after the last move) whose curve clears every box, gentlest first.
        ready = best[5]
        shift = abs(last)
        lo = max(1., math.sqrt(5.77*shift/max_curvature) if math.isfinite(max_curvature) else min_move)
        pref = max(lo, v*curve_time)
        horizon = max([b.s_max for b in boxes], default=ready) + rear + front
        done = None
        for length in np.arange(pref, lo - 1e-6, -1.):
            for start in np.arange(ready, max(ready, horizon) + 1e-6, 1.):
                if not hits(boxes, last, 0., start, length, min(ready, start - lead), start + length + front + rear, lead):
                    done = (float(length), float(start))
                    break
            if done:
                break
        if done:
            offsets.append(0.); lengths.append(done[0]); starts.append(done[1])
    first_from = ongoing[0] if (ongoing is not None and offsets and lengths and starts
                                and abs(starts[0] - ongoing[2]) < 1e-6 and abs(lengths[0] - ongoing[3]) < 1e-6) else d_now
    prev = [first_from] + offsets[:-1]
    return Slalom(offsets, groups, fronts, clears, lengths, starts, blocked_at,
                  [abs(a - b) for a, b in zip(offsets, prev)], best[0])
