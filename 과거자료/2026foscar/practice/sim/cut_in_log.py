#!/usr/bin/env python3
"""PRACTICE ONLY: record every highway lane change of loop_runner runs with its boundary values and outcome.

The user asked (2026-09-28) to keep a record of successful cut-ins so the admissible range (how close
to the car ahead / behind, at what closing speed) is judged from what actually worked, not guessed.

For each CHANGE_STARTED on /behavior/highway:
  lanes, ego speed, sequence (rear mode PASS/NORMAL, the preparation that led to it),
  rear car: gap, closing speed, required gap (boxes.rear),
  target-lane car ahead: bumper gap and its speed (from the diagnostics' scene lanes),
  outcome: OK (change completed, no collision until then + 3 s) / COLLISION / INCOMPLETE.

    python3 practice/sim/cut_in_log.py practice/sim/runs/<stamp> [...]     # append records
    python3 practice/sim/cut_in_log.py --summary                            # boundary values of OKs
"""
import argparse, glob, json, math, os, re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'practice/sim/cut_in_records.jsonl'
FRONT = 3.8          # rear axle to front bumper (highway.yaml vehicle: length 4.635)
BACK = 0.835


def read_highway(bag):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = SequentialReader(); r.open(StorageOptions(uri=bag, storage_id=''), ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == '/behavior/highway':
            out.append(json.loads(deserialize_message(data, get_message(types[topic])).data))
    return out


def along(d, xy):
    c, s = math.cos(d['ego_yaw']), math.sin(d['ego_yaw'])
    ex, ey = d['ego_xy']
    return [(x-ex)*c+(y-ey)*s for x, y in xy]


def records(run_dir):
    results = [json.loads(l) for l in open(os.path.join(run_dir, 'results.jsonl'))]
    for res in results:
        log = os.path.join(run_dir, f"launch_{res['run']}.log")
        if not os.path.exists(log):continue
        m = re.search(r'(run_[0-9_]+)_0\.db3', open(log, errors='ignore').read())
        if not m:continue
        bag = os.path.expanduser('~/foscar_logs/bags/'+m.group(1))
        if not os.path.exists(bag):continue
        L = read_highway(bag)
        hits = [c['s'] for c in res.get('collisions', [])]
        before = ''
        for k, d in enumerate(L):
            if d['reason'] != 'CHANGE_STARTED':
                if d['state'] in ('WAIT_GAP', 'ROUTE_FALLBACK'):before = d.get('gap_strategy') or before
                continue
            src, dst, s0 = d.get('current_lane'), d.get('target_lane'), d['global_s']
            objs = {o['id']: o for o in d.get('objects', [])}
            scene = (d.get('scene') or {}).get('objects', {})
            ahead = []
            for key, info in scene.items():
                if info.get('lane') != dst or key not in objs:continue
                a = along(d, objs[key]['xy'])
                if min(a) > FRONT - .5:ahead.append((min(a)-FRONT, info.get('speed_kmh')))
            front = min(ahead) if ahead else (None, None)
            rear = ((d.get('boxes') or {}).get('rear')) or {}
            end = next((e['global_s'] for e in L[k+1:] if e['reason'] == 'CHANGE_COMPLETE'), None)
            hit = [h for h in hits if s0-5 <= h <= (end if end is not None else s0+80)+3*max(1., d['ego_speed'])]
            outcome = 'COLLISION' if hit else 'OK' if end is not None else 'INCOMPLETE'
            yield dict(run_dir=os.path.basename(run_dir.rstrip('/')), run=res['run'], s=round(s0, 1), lanes=f'{src}->{dst}',
                       v_kmh=round(d['ego_speed']*3.6, 1), prepared=before, rear_mode=rear.get('mode'),
                       rear_gap_m=rear.get('gap_m'), rear_closing_mps=rear.get('closing_mps'), rear_need_m=rear.get('need_m'),
                       front_gap_m=None if front[0] is None else round(front[0], 2),
                       front_speed_kmh=None if front[1] is None else round(front[1], 1),
                       front_closing_kmh=None if front[1] is None else round(d['ego_speed']*3.6-front[1], 1),
                       tail_in=d.get('tail_in'), outcome=outcome, collision_s=hit[0] if hit else None)
            before = ''


def summary():
    rows = [json.loads(l) for l in open(OUT)] if OUT.exists() else []
    print(f'{len(rows)} changes:', {o: sum(r['outcome'] == o for r in rows) for o in ('OK', 'COLLISION', 'INCOMPLETE')})
    ok = [r for r in rows if r['outcome'] == 'OK']
    def edge(label, rs, key):
        v = [r[key] for r in rs if r[key] is not None]
        print(f'  {label}: n={len(v)}' + (f' min={min(v):.2f} p10={sorted(v)[len(v)//10]:.2f}' if v else ''))
    for mode in ('PASS', 'NORMAL'):
        rs = [r for r in ok if r['rear_mode'] == mode]
        edge(f'OK rear gap, {mode}', rs, 'rear_gap_m')
    for lo, hi in ((-99, 0), (0, 2), (2, 4), (4, 99)):
        rs = [r for r in ok if r['rear_mode'] == 'NORMAL' and r['rear_closing_mps'] is not None and lo <= r['rear_closing_mps'] < hi]
        edge(f'OK rear gap, closing {lo}..{hi} m/s', rs, 'rear_gap_m')
    for lo, hi in ((-99, 0), (0, 10), (10, 99)):
        rs = [r for r in ok if r['front_closing_kmh'] is not None and lo <= r['front_closing_kmh'] < hi]
        edge(f'OK front gap, closing {lo}..{hi} km/h', rs, 'front_gap_m')
    for r in rows:
        if r['outcome'] != 'OK':print('  ', r['outcome'], r['run_dir'], r['run']+1, r['s'], r['lanes'], r['v_kmh'], 'km/h',
                                      'rear', r['rear_gap_m'], r['rear_closing_mps'], r['rear_mode'], 'front', r['front_gap_m'], r['front_closing_kmh'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='*')
    ap.add_argument('--summary', action='store_true')
    a = ap.parse_args()
    seen = set()
    if OUT.exists():
        seen = {(r['run_dir'], r['run'], r['s']) for r in map(json.loads, open(OUT))}
    added = 0
    with open(OUT, 'a') as f:
        for run_dir in a.runs:
            for r in records(run_dir):
                if (r['run_dir'], r['run'], r['s']) in seen:continue
                f.write(json.dumps(r, ensure_ascii=False)+'\n'); added += 1
    if a.runs:print(f'{added} records added -> {OUT}')
    if a.summary or not a.runs:summary()


if __name__ == '__main__':
    main()
