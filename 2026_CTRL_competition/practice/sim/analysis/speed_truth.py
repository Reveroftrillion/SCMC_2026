#!/usr/bin/env python3
"""PRACTICE ONLY: how right are the highway's box-edge speeds? Replays the recorded /behavior/highway ticks
(ego pose + every hull) through the CURRENT HighwayPolicy._occupancy and compares each run's speed with the
true speed of the NPC it covers (objects_<n>.jsonl, logged by loop_runner since 2026-10-01).

    python3 practice/sim/analysis/speed_truth.py <bag> <objects_n.jsonl> [<bag> <objects> ...]

Per run that has a speed and lies over exactly one true car (same lane, station ranges overlapping), the error
is run speed - true speed (km/h). Reported for all, for our-lane runs ahead (the following lead) and by distance.
"""
import bisect, json, math, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'practice/sim'))
from cut_in_log import read_highway                       # noqa: E402
from slot_replay import load, snapshot                    # noqa: E402
from foscar_planning.highway_policy import HighwayPolicy  # noqa: E402
from foscar_planning.state import Context                 # noqa: E402


def evaluate(bag, objects_path, cfg, road):
    truth = [json.loads(l) for l in open(objects_path)];ts = [r['t'] for r in truth]
    p = HighwayPolicy(road, cfg);rows = []
    for d in read_highway(bag):
        if 'ego_xy' not in d or 'objects' not in d:continue
        if not road.start_s-60 <= d['global_s'] <= road.rejoin_s:continue
        snap = snapshot(d)
        ctx = Context(s=d['global_s'], x=d['ego_xy'][0], y=d['ego_xy'][1], yaw=d['ego_yaw'], v=d['ego_speed'], now=snap.stamp,
                      mission_name='HIGHWAY')
        occ, _, _ = p._occupancy(ctx, snap, tuple(road.lanes))
        i = bisect.bisect_left(ts, snap.stamp)
        k = min((j for j in (i-1, i) if 0 <= j < len(ts)), key=lambda j: abs(ts[j]-snap.stamp), default=None)
        if k is None or abs(ts[k]-snap.stamp) > .1:continue
        cars = []
        for o in truth[k]['objects']:
            if o['type'] != 1:continue
            st = float(road.project_many(np.array([[o['x'], o['y']]]))[0])
            near = []
            for l in road.lanes:
                try:near.append((math.dist(road.at(l, st), (o['x'], o['y'])), l))
                except ValueError:pass        # outside that lane's coverage
            if not near or min(near)[0] > 1.75:continue
            lane = min(near)[1]
            cars.append((lane, st-o['l']/2, st+o['l']/2, o['v']))
        for lane, (boxes, runs, speeds) in occ.items():
            if lane == 'path':continue
            for (lo, hi, *_), v in zip(runs, speeds):
                if v is None or not -40 <= (lo+hi)/2-ctx.s <= 80:continue
                hit = [c for c in cars if c[0] == lane and c[1] < hi+1. and c[2] > lo-1.]
                if len(hit) != 1:continue
                rows.append(dict(lane=lane, own=lane == d.get('current_lane') and lo > ctx.s, ahead=(lo+hi)/2-ctx.s,
                                 est=v*3.6, true=hit[0][3], err=v*3.6-hit[0][3]))
    return rows


def report(rows, name):
    if not rows:print(f'{name}: no matched runs');return
    e = np.array([r['err'] for r in rows])
    print(f'{name}: n={len(e)} bias {e.mean():+.1f} km/h, |err| median {np.median(abs(e)):.1f} p90 {np.percentile(abs(e), 90):.1f} '
          f'max {abs(e).max():.1f}, >5 km/h {100*(abs(e) > 5).mean():.0f} %')


if __name__ == '__main__':
    cfg, road = load();rows = []
    for bag, objs in zip(sys.argv[1::2], sys.argv[2::2]):rows += evaluate(bag, objs, cfg, road)
    report(rows, 'all runs')
    report([r for r in rows if r['own']], 'our-lane lead')
    for a, b in ((-40, 0), (0, 30), (30, 60), (60, 80)):report([r for r in rows if a <= r['ahead'] < b], f'  {a:+d}..{b:+d} m')
