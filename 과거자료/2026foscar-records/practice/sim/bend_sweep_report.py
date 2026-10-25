#!/usr/bin/env python3
"""PRACTICE ONLY: highway bend lateral-acceleration sweep report (user 2026-09-28).

Goal: from CP12 (s1593) to CP13 (s1741) no slowing other than the ruled drop to 60 km/h, by the bend limit alone.
Per run: speed entering CP12, the lowest speed over s1593-1690 (before the 60 km/h approach), the dip between them,
max |cross-track| over the highway (CP10-CP13) and over CP12-13 (follow_*.csv of the run), the measured peak lateral
acceleration over s1590-1700 (results ay_bins) and the CP10-CP13 time.

    python3 practice/sim/bend_sweep_report.py practice/sim/runs/<stamp> [...]
"""
import csv, datetime, glob, json, os, sys

LOGS = os.path.expanduser('~/foscar_logs')


def follow_log(started):
    t0 = datetime.datetime.fromisoformat(started)
    best = None
    for f in glob.glob(os.path.join(LOGS, 'follow_*.csv')):
        try:t = datetime.datetime.strptime(os.path.basename(f)[7:22], '%Y%m%d_%H%M%S')
        except ValueError:continue
        d = (t-t0).total_seconds()
        if 0 <= d <= 30 and (best is None or d < best[0]):best = (d, f)
    return best and best[1]


def main():
    for run_dir in sys.argv[1:]:
        print('==', run_dir)
        for line in open(os.path.join(run_dir, 'results.jsonl')):
            r = json.loads(line);sp = r['splits']
            hw = round(sp['1741.7']-sp['1118.7'], 1) if '1741.7' in sp and '1118.7' in sp else None
            f = follow_log(r['started'])
            if f is None or hw is None:print(r['run']+1, r['result'], 'no data');continue
            rows = [(float(x['s_m']), float(x['speed_mps'])*3.6, abs(float(x['cross_track_m']))) for x in csv.DictReader(open(f))]
            def at(s):return min(rows, key=lambda x: abs(x[0]-s))[1]
            mid = [x for x in rows if 1593 <= x[0] <= 1690]
            v12 = at(1593.);vmin = min(v for _, v, _ in mid)
            ct_hw = max(c for s, _, c in rows if 1118 <= s <= 1745);ct_12 = max(c for s, _, c in rows if 1593 <= s <= 1741)
            ay = max([v for k, v in r.get('ay_bins', {}).items() if 1590 <= float(k) <= 1700] or [0.])
            print(f"{r['run']+1} {r['result']} hw {hw}s | CP12 {v12:.0f} km/h, min 1593-1690 {vmin:.0f} (dip {v12-vmin:.1f}) "
                  f"| |ct| max highway {ct_hw:.2f} m, CP12-13 {ct_12:.2f} m | ay peak 1590-1700 {ay:.2f}")


if __name__ == '__main__':
    main()
