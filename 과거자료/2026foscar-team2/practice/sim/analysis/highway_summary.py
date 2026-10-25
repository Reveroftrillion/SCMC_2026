#!/usr/bin/env python3
"""PRACTICE ONLY: highway section summary of loop_runner runs (2026-10-01 overtake work, baseline vs changes).

Per run: CP10 -> CP13 time from the runner's splits, collisions, pass_log contacts, the lowest speed seen while
deciding each lane change (pass_records 'decision' rows, about one per second) and the decision reasons.

    python3 practice/sim/analysis/highway_summary.py practice/sim/runs/<stamp> [...]
"""
import csv, json, os, re, statistics, sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'practice/sim/analysis'))
from cp_splits import checkpoints  # noqa: E402
CP10, CP13 = '1118.7', '1741.7'

LANE2_SLOW_KMH = 70.   # "stuck behind a slow lane-2 car": HIGHWAY_FOLLOW below this (lane 2 runs ~100 km/h when free)


def cp_times(run_dir, n, names=('10', '12'), max_m=3.):
    """Closest-approach time of the true pose to each named checkpoint, each searched after the previous one.
    Independent of cp_splits.splits(): that walks CP1.. in order from the start, which fails for runs that start
    mid-route (Practice3_CP08) -- every checkpoint before the start snaps to garbage. None if farther than max_m."""
    path = os.path.join(run_dir, f'truth_{n}.jsonl')
    if not os.path.exists(path):return {k:None for k in names}
    tr = np.array([(r['t'], r['x'], r['y']) for r in map(json.loads, open(path))])
    cps = {k:(x, y) for k, x, y in checkpoints()}
    out = {}; k0 = 0
    for k in names:
        d = np.hypot(tr[k0:, 1]-cps[k][0], tr[k0:, 2]-cps[k][1]); i = int(np.argmin(d))
        out[k] = round(float(tr[k0+i, 0]), 2) if d[i] <= max_m else None
        if out[k] is not None:k0 += i
    return out


def follow_csv(run_dir, n):
    m = re.search(r'log=(\S+follow_\S+\.csv)', open(os.path.join(run_dir, f'launch_{n}.log'), errors='ignore').read())
    return m.group(1) if m and os.path.exists(m.group(1)) else None


def lane2_follow_s(path, rejoin_s=1586.5):
    rows = list(csv.DictReader(open(path)))
    last_change = max((i for i, r in enumerate(rows) if r['reason'] in ('HIGHWAY_CHANGE', 'HIGHWAY_SETTLE')), default=None)
    if last_change is None:return 0.
    tot = 0.
    for a, b in zip(rows[last_change:], rows[last_change+1:]):
        if a['reason'] == 'HIGHWAY_FOLLOW' and float(a['s_m']) < rejoin_s and float(a['speed_mps'])*3.6 < LANE2_SLOW_KMH:
            tot += float(b['elapsed_s'])-float(a['elapsed_s'])
    return round(tot, 2)


def records(run_dir):
    name = os.path.basename(run_dir.rstrip('/'))
    return [r for r in map(json.loads, open(ROOT/'practice/sim/pass_records.jsonl')) if r.get('run_dir') == name]


def summarize(run_dir):
    rec = records(run_dir); rows = []
    for r in map(json.loads, open(os.path.join(run_dir, 'results.jsonl'))):
        sp = r.get('splits', {})
        hw = round(sp[CP13]-sp[CP10], 2) if CP10 in sp and CP13 in sp else None
        mine = [x for x in rec if x.get('run') == r['run']]
        waits = {}
        for x in mine:
            if x['kind'] == 'decision':waits.setdefault(x['lanes'], []).append(x['v_kmh'])
        c = cp_times(run_dir, r['run'])
        fc = follow_csv(run_dir, r['run'])
        rows.append(dict(run=r['run'], result=r['result'], hw_s=hw, collisions=len(r['collisions']),
                         cp10_cp12_s=round(c['12']-c['10'], 2) if c['10'] is not None and c['12'] is not None else None,
                         lane2_follow_s=lane2_follow_s(fc) if fc else None,
                         contacts=sum(x['kind'] == 'contact' for x in mine),
                         wait_min_kmh={k: min(v) for k, v in sorted(waits.items(), reverse=True)},
                         reasons=sorted({x.get('reason') for x in mine if x['kind'] == 'decision'})))
    return rows


def main():
    rows = []
    for d in sys.argv[1:]:
        for row in summarize(d):
            row['dir'] = os.path.basename(d.rstrip('/'));rows.append(row)
            print(f"{row['dir']} run{row['run']} {row['result']:<24} CP10->CP13 {row['hw_s']} s  CP10->CP12 {row['cp10_cp12_s']} s  "
                  f"lane2 slow follow {row['lane2_follow_s']} s  coll {row['collisions']} "
                  f"contact {row['contacts']}  wait min km/h {row['wait_min_kmh']}  {row['reasons']}")
    hw = [r['hw_s'] for r in rows if r['hw_s'] is not None]
    if hw:
        print(f'CP10->CP13: n={len(hw)} median {statistics.median(hw):.2f} s, min {min(hw):.2f}, max {max(hw):.2f}; '
              f"collisions {sum(r['collisions'] for r in rows)}, contacts {sum(r['contacts'] for r in rows)}")
    for key, label in (('cp10_cp12_s', 'CP10->CP12'), ('lane2_follow_s', 'lane2 slow follow')):
        v = [r[key] for r in rows if r[key] is not None]
        if v:print(f'{label}: n={len(v)} median {statistics.median(v):.2f} s')


if __name__ == '__main__':
    main()
