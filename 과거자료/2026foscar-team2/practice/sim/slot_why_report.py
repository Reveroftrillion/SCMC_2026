#!/usr/bin/env python3
"""PRACTICE ONLY: why the slot planner found no slot ahead (2026-10-01 overtake work).

For every recorded waiting tick whose decision was NO_SLOT or SLOT_YIELD, replay the current HighwayPolicy with
trace_slots on and add up why each candidate plan was refused, per lane change and recorded decision. ACCEL / KEEP
rows are the plans that would have got ahead of a target-lane car.

Every highway tick of a bag goes through one policy in order, so its box-edge speed estimator (EdgeTracks) is warm
as it was in the run: replaying only the waiting ticks started it cold and read unmeasured leads as standing (sample
A 4->3 NO_SLOT: 56/63 agreed, 262 spurious YIELD:ok). Before each tick the decision state is set from the record
(lane, CP11, nothing committed, waiting), so the policy never goes off on a change of its own; only the selected
ticks are tallied. Per stage and recorded decision it prints how many replayed ticks reached the same decision.

    python3 practice/sim/slot_why_report.py practice/sim/runs/<stamp> [...] [--reasons NO_SLOT,SLOT_YIELD]
"""
import argparse, collections, glob, os, re, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from slot_replay import load, snapshot                                  # noqa: E402
from cut_in_log import read_highway                                     # noqa: E402
from foscar_planning.highway_policy import HighwayPolicy                # noqa: E402
from foscar_planning.state import Context                               # noqa: E402


def logs(run_dir):
    for log in sorted(glob.glob(os.path.join(run_dir, 'launch_*.log'))):
        m = re.search(r'(run_[0-9_]+)_0\.db3', open(log, errors='ignore').read())
        if not m:continue
        bag = os.path.expanduser('~/foscar_logs/bags/'+m.group(1))
        if os.path.exists(bag):yield [d for d in read_highway(bag) if 'global_s' in d and d.get('ego_xy')]


def decide(p, d):
    """Decision state from the record; the estimator state (edges, brake, last scan) is kept."""
    p.current_lane = d['current_lane']; p.cp11 = bool(d.get('cp11_passed')); p.previous = None
    p.committed = None; p.change_passing = (); p.change_front = None; p.change_v0 = 0.
    p.state = 'WAIT_GAP'; p.settle_since = None; p.candidate = None; p.taper_committed = False
    p.follow_reference = None; p.plan = None; p.keep_speed = None


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('runs', nargs='+'); ap.add_argument('--reasons', default='NO_SLOT,SLOT_YIELD')
    a = ap.parse_args(); want = set(a.reasons.split(','))
    cfg, road = load()
    ticks = collections.Counter(); agree = collections.Counter(); why = collections.defaultdict(collections.Counter)
    t_start = time.time(); fed = 0
    for run in a.runs:
        for L in logs(run):
            p = HighwayPolicy(road, cfg)
            for d in L:
                if d.get('current_lane') is None:continue
                sel = d.get('state') == 'WAIT_GAP' and d.get('reason') in want and d['current_lane'] != road.lanes[-1]
                decide(p, d); p.trace_slots = sel
                ctx = Context(s=d['global_s'], x=d['ego_xy'][0], y=d['ego_xy'][1], yaw=d['ego_yaw'], v=d['ego_speed'],
                              now=d['stamp'], mission_name='HIGHWAY')
                out = p.tick(ctx, snapshot(d)); fed += 1
                if not sel:continue
                lane = d['current_lane']
                key = f"{lane}->{road.lanes[road.lanes.index(lane)+1]} {d['reason']}"
                ticks[key] += 1; agree[key] += out.reason == d['reason']
                why[key].update(out.diagnostics.get('slot_why') or {})
    print(f'fed {fed} highway ticks in {time.time()-t_start:.0f} s')
    for key in sorted(ticks):
        print(f'{key}: {ticks[key]} ticks, replay agrees on {agree[key]}')
        for reason, n in why[key].most_common():print(f'    {reason:24s} {n}')


if __name__ == '__main__':
    main()
