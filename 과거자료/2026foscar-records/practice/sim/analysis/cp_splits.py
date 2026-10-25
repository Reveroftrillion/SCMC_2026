#!/usr/bin/env python3
"""PRACTICE ONLY: checkpoint-to-checkpoint times from the true ego track (truth_<n>.jsonl), independent of the
route's stations -- for comparing laps on different global paths (2026-10-01 racing line).

    python3 practice/sim/analysis/cp_splits.py <run_dir> [...]

Per checkpoint: the time of closest approach of the true pose and that distance (the rule needs <= 3 m).
"""
import sys, os, re, json, math
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))


def checkpoints():
    txt = open(os.path.join(ROOT, 'src/foscar_bringup/routes/checkpoints.txt')).read()
    out = [];name = None
    for line in txt.splitlines():
        m = re.match(r'\[(.+)\]', line.strip())
        if m:name = m.group(1);continue
        nums = re.findall(r'-?\d+\.\d+', line)
        if name and len(nums) == 3 and name != '체크 포인트':out.append(('S' if 'START' in name else name, float(nums[0]), float(nums[1])));name = None
    return out


def splits(run_dir, n=0):
    tr = np.array([(r['t'], r['x'], r['y']) for r in map(json.loads, open(os.path.join(run_dir, f'truth_{n}.jsonl')))])
    moving = np.flatnonzero(np.linalg.norm(np.diff(tr[:, 1:], axis=0), axis=1) > .01)
    t_start = tr[moving[0], 0] if len(moving) else tr[0, 0]
    out = [];k0 = 0
    for name, x, y in checkpoints()[1:]:
        d = np.hypot(tr[k0:, 1]-x, tr[k0:, 2]-y);i = int(np.argmin(d))
        out.append((name, round(tr[k0+i, 0]-t_start, 2), round(float(d[i]), 2)));k0 += i
    return out


if __name__ == '__main__':
    for d in sys.argv[1:]:
        s = splits(d)
        print(os.path.basename(d.rstrip('/')), ' '.join(f'CP{n}:{t:.1f}s({m:.1f}m)' for n, t, m in s))
