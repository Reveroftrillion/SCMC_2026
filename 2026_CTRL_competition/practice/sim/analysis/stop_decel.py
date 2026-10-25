#!/usr/bin/env python3
"""Practice only: how hard the car braked into every full stop of a run (from the truth pose log).

    python3 practice/sim/analysis/stop_decel.py practice/sim/runs/<stamp>/truth_0.jsonl

Per stop: the speed it braked from, time to standstill, mean decel, and how that time splits between the first
and the second half of the speed (a long second half = creeping up to the line, 2026-10-06: 20 -> 0 km/h took 3.3 s
on the 1.0 m/s2 stop ramp).
"""
import json
import math
import sys


def speeds(rows, window_s=0.3):
    out = []
    for i, r in enumerate(rows):
        j = i
        while j > 0 and r['t'] - rows[j]['t'] < window_s:
            j -= 1
        dt = r['t'] - rows[j]['t']
        out.append(math.hypot(r['x'] - rows[j]['x'], r['y'] - rows[j]['y']) / dt if dt > 0 else 0.)
    return out


def main(path):
    rows = [json.loads(l) for l in open(path)][1:]      # the first row is the pose before the scenario load
    v = speeds(rows)
    for i in [k for k in range(1, len(v)) if v[k] < .1 <= v[k - 1]]:
        window = [k for k in range(i) if rows[i]['t'] - rows[k]['t'] < 12.]
        k = max(window, key=lambda a: v[a])
        k = max(a for a in window if a >= k and v[a] > v[k] - .3)    # last moment still at that speed
        total = rows[i]['t'] - rows[k]['t']
        half = next(a for a in range(k, i + 1) if v[a] <= v[k] / 2)
        print('stop xy(%.0f,%.0f): %.0f km/h -> 0 in %.1f s, mean %.1f m/s2 | first half %.1f s, second half %.1f s' % (
            rows[i]['x'], rows[i]['y'], v[k] * 3.6, total, v[k] / total if total else 0.,
            rows[half]['t'] - rows[k]['t'], rows[i]['t'] - rows[half]['t']))


if __name__ == '__main__':
    main(sys.argv[1])
