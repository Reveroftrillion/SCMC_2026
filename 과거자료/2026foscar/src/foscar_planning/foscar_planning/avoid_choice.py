"""Explain the AVOID side choice from the planner's own candidates (display only)."""
import math

REASONS = (('hits', 'HITS BOX'), ('turning body', 'TURN OFF ROAD'),
           ('outside mapped', 'OFF ROAD'), ('keep committed', 'COMMITTED OTHER SIDE'))


def short_reason(text):
    return next((label for key, label in REASONS if text.startswith(key)), text.upper() or 'BLOCKED')


def side_summary(plan, tunnel=False, left_bias=0.):
    """{'LEFT'|'RIGHT': dict(status, free, reason, cost, d)} for the current plan.

    status: CHOSEN / FREE / BLOCKED. reason explains the choice (CHOSEN) or the
    nearest rejected offset's cause (BLOCKED). No new decision is made here.
    """
    chosen = 0 if plan is None or plan.blocked else (plan.d > 0) - (plan.d < 0)
    out = {}
    for name, sign in (('LEFT', 1), ('RIGHT', -1)):
        cands = sorted((c for c in (plan.candidates if plan else []) if c.d * sign > 0), key=lambda c: abs(c.d))
        free = [c for c in cands if c.feasible]
        rejected = [c for c in cands if not c.feasible]
        row = dict(status='FREE' if free else 'BLOCKED', free=(abs(free[0].d), abs(free[-1].d)) if free else None,
                   cost=min((c.cost for c in free), default=math.inf), d=None, reason='')
        if not free:
            # Report why the smallest offset on this side failed, then the widest one.
            row['reason'] = short_reason(rejected[0].reason) if rejected else 'NO CANDIDATE'
            far = short_reason(rejected[-1].reason) if rejected else ''
            if far and far != row['reason']:
                row['reason'] += ' / ' + far
        else:
            row['d'] = free[0].d
        out[name] = row
    for name, sign in (('LEFT', 1), ('RIGHT', -1)):
        if chosen != sign:
            continue
        row, other = out[name], out['RIGHT' if sign > 0 else 'LEFT']
        row.update(status='CHOSEN', d=plan.d)
        other_rows = [c for c in plan.candidates if c.d * sign < 0]
        if any(c.reason.startswith('keep committed') for c in other_rows):
            # mapped_plan refused box-free offsets there only to keep the side.
            row['reason'] = 'COMMITTED SIDE'
        elif other['status'] == 'BLOCKED':
            row['reason'] = 'OTHER SIDE BLOCKED'
        elif tunnel:
            # mapped_plan takes the smallest lateral move that clears the obstacle.
            # The slalom may choose an offset the one-box view here calls blocked (it plans every box and
            # the path being driven); then there is no free range to compare. That None crashed the
            # behavior node right before the second tunnel barricade (2026-09-29).
            if row['free'] is None or other['free'] is None:
                row['reason'] = 'SLALOM PLAN'
            else:
                row['reason'] = 'SMALLER MOVE' if row['free'][0] < other['free'][0] - 1e-9 else 'EQUAL MOVE / TIE'
        else:
            # The planner's small left bias is a tie-break, not a real cost gap.
            row['reason'] = 'LOWER COST' if row['cost'] < other['cost'] - left_bias - 1e-9 else 'TIE / LEFT DEFAULT'
    return out
