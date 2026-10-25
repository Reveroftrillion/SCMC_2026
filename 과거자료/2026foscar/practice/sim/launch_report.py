#!/usr/bin/env python3
"""PRACTICE ONLY: roundabout standstill-entry report (Practice3_RB885 runs).

Per run: how long we stood at the entry before moving, and the launch: delay from the first pedal to 0.5 km/h,
mean acceleration over the first 1.0 and 2.0 s after moving, pedal used. From follow_*.csv of the run.

    python3 practice/sim/launch_report.py practice/sim/runs/<stamp> [...]
"""
import csv, datetime, glob, json, os, sys

LOGS = os.path.expanduser('~/foscar_logs')


def follow_log(started):
    t0 = datetime.datetime.fromisoformat(started);best = None
    for f in glob.glob(os.path.join(LOGS, 'follow_*.csv')):
        try:t = datetime.datetime.strptime(os.path.basename(f)[7:22], '%Y%m%d_%H%M%S')
        except ValueError:continue
        d = (t-t0).total_seconds()
        if 0 <= d <= 30 and (best is None or d < best[0]):best = (d, f)
    return best and best[1]


for run_dir in sys.argv[1:]:
    print('==', run_dir)
    for line in open(os.path.join(run_dir, 'results.jsonl')):
        r = json.loads(line);f = follow_log(r['started'])
        if f is None:print(r['run']+1, r['result'], 'no log');continue
        rows = [dict(t=float(x['elapsed_s']), v=float(x['speed_mps']), a=float(x['accel']), s=float(x['s_m']), reason=x['reason'])
                for x in csv.DictReader(open(f))]
        rows = [x for x in rows if x['s'] >= 883.]
        # The entry launch: the first pedal after the last standstill at the entry (s < 888).
        still = [x for x in rows if x['v'] < .14 and x['s'] < 888.]
        if not still:print(r['run']+1, r['result'], 'never stood at the entry');continue
        first_still = still[0]['t'];last_still = still[-1]['t']
        pedal = next((x for x in rows if x['a'] > .05 and x['t'] >= last_still-.3), None)
        move = next((x for x in rows if x['v'] > .14 and pedal and x['t'] >= pedal['t']), None)
        if pedal is None or move is None:print(r['run']+1, r['result'], 'no launch');continue
        def at(dt):return next((x['v'] for x in rows if x['t'] >= move['t']+dt), None)
        v1, v2 = at(1.), at(2.)
        wait = pedal['t']-first_still
        peak = max(x['a'] for x in rows if pedal['t'] <= x['t'] <= pedal['t']+2.)
        print(f"{r['run']+1} {r['result']} stood at entry {wait:.1f}s | pedal->move {move['t']-pedal['t']:.2f}s | "
              f"a 0-1s {v1/1.:.2f}, 0-2s {v2/2.:.2f} m/s2 | pedal max {peak:.2f} | {pedal['reason']}")
