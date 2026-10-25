#!/usr/bin/env python3
"""Tunnel pedestrian variants of T1_CP14_Dyn with a random activation distance and walking speed.

MORAI starts a waypoint pedestrian once the ego car is within its activeDistance, so the distance sets when
it steps out in front of us. Writes practice/scenarios/T1_Rand_NN.json + T1_Rand.jsonl (what each one got)
and copies them (plus the ego network file) to the simulator PC.

  python3 practice/sim/random_ped.py --count 10 --seed 1
  python3 practice/sim/loop_runner.py T1_Rand_03 --end-s 2165 --watch-from 1790 --stop-s 8
"""
import argparse
import json
import random
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parents[1] / 'scenarios'
SIM = '~/바탕화면/MoraiLauncher/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025'
PED = 44   # the crossing pedestrian at s1914 (user-made in tunnel_test1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', default='T1_CP14_Dyn')
    ap.add_argument('--count', type=int, default=10)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--dist', type=float, nargs=2, default=(4., 30.), help='activation distance range [m]')
    ap.add_argument('--speed', type=float, nargs=2, default=(5., 15.), help='walking speed range [km/h]')
    ap.add_argument('--no-upload', action='store_true')
    a = ap.parse_args()
    rng = random.Random(a.seed)
    base = json.loads((HERE / f'{a.base}.json').read_text())
    rows, names = [], []
    for i in range(1, a.count + 1):
        name = f'T1_Rand_{i:02d}'
        s = json.loads(json.dumps(base))
        ped = next(p for p in s['pedestrianList'] if p['UNIQUEID'] == PED)
        ped['activeDistance'] = round(rng.uniform(*a.dist), 1)
        ped['speed'] = round(rng.uniform(*a.speed), 1)
        (HERE / f'{name}.json').write_text(json.dumps(s, ensure_ascii=False))
        rows.append(dict(name=name, seed=a.seed, active_m=ped['activeDistance'], speed_kmh=ped['speed']))
        names.append(name)
        print(rows[-1])
    (HERE / 'T1_Rand.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    if a.no_upload:
        return
    subprocess.run(['scp', '-q', *[str(HERE / f'{n}.json') for n in names], f'morai:{SIM}/'], check=True)
    copies = ' && '.join(f'cp {SIM}/EgoNetwork/{a.base}_MN.json {SIM}/EgoNetwork/{n}_MN.json' for n in names)
    subprocess.run(['ssh', 'morai', copies], check=True)
    print('uploaded', len(names))


if __name__ == '__main__':
    main()
