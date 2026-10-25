#!/usr/bin/env python3
"""PRACTICE ONLY: highway safety summary per run (2026-10-01 rear-end contacts during back-to-back changes).

    python3 practice/sim/analysis/hw_safety.py <run_dir> [...]

- closest true body gap to a car ahead in our path (|lateral| < 2 m in our frame) and to anyone, from
  truth_<n>.jsonl + objects_<n>.jsonl (both poses ~0.28 m ahead of the rear axle; NPC body taken as origin -1 m
  .. length -1 m)
- ticks where an unmeasured lead was capped below "us - unknown_speed" (the provisional Kalman estimate acting)
- ticks braking past 0.5 while moving (the emergency ramp acting)
- highway time CP10 -> CP13 (runner splits) and the lane changes (pass_records.jsonl)
"""
import sys, os, json, math, bisect
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from steer_report import bag_for  # noqa: E402
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quad(x, y, yaw, back, front, w):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([(x+c*dx-s*dy, y+s*dx+c*dy) for dx, dy in ((back, -w/2), (front, -w/2), (front, w/2), (back, w/2))])


def sep(A, B):
    best = 0.
    for P in (A, B):
        for i in range(4):
            e = P[(i+1) % 4]-P[i];n = np.array([-e[1], e[0]]);n /= np.linalg.norm(n);a = A@n;b = B@n
            best = max(best, b.min()-a.max(), a.min()-b.max())
    return best


def gaps(run):
    ego = [json.loads(l) for l in open(os.path.join(run, 'truth_0.jsonl'))]
    objs = [json.loads(l) for l in open(os.path.join(run, 'objects_0.jsonl'))];ot = [o['t'] for o in objs]
    ahead = anyone = (math.inf, None)
    for e in ego:
        i = bisect.bisect_left(ot, e['t'])
        k = min((j for j in (i-1, i) if 0 <= j < len(ot)), key=lambda j: abs(ot[j]-e['t']), default=None)
        if k is None or abs(ot[k]-e['t']) > .1:continue
        E = quad(e['x'], e['y'], e['yaw'], -1.08, 3.52, 1.892);c, s = math.cos(e['yaw']), math.sin(e['yaw'])
        for o in objs[k]['objects']:
            if o['type'] != 1 or math.dist((e['x'], e['y']), (o['x'], o['y'])) > 15:continue
            g = sep(E, quad(o['x'], o['y'], math.radians(o['yaw']), -1., o['l']-1., o['w']))
            dx, dy = o['x']-e['x'], o['y']-e['y'];lon, lat = c*dx+s*dy, -s*dx+c*dy
            rec = (round(g, 2), f"t={e['t']-ego[0]['t']:.1f}s npc{o['id']} lon{lon:+.1f} lat{lat:+.1f} {o['v']:.0f}km/h")
            if g < anyone[0]:anyone = rec
            if lon > 0 and abs(lat) < 2. and g < ahead[0]:ahead = rec
    return ahead, anyone


def bag_events(run):
    r = SequentialReader();r.open(StorageOptions(uri=bag_for(run), storage_id=''), ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    prov = [];emerg = [];s = 0.
    while r.has_next():
        tp, d, t = r.read_next()
        if tp == '/localization/s':s = deserialize_message(d, get_message(types[tp])).data
        elif tp == '/behavior/highway':
            j = json.loads(deserialize_message(d, get_message(types[tp])).data)
            for o in j.get('following_objects') or []:
                if o.get('mode', '').startswith('UNKNOWN') and o['hard_cap_mps'] < j['ego_speed']-5.:
                    prov.append((round(j['global_s']), round(j['ego_speed']*3.6), round(o['gap_m'], 1), round(o['hard_cap_mps']*3.6)))
        elif tp == '/control/telemetry':
            j = json.loads(deserialize_message(d, get_message(types[tp])).data)
            if j.get('brake', 0) > .5 and j.get('speed_mps', 0) > 1.:emerg.append((round(s), round(j['speed_mps']*3.6), round(j['target_speed_mps']*3.6), round(j['brake'], 2)))
    return prov, emerg


if __name__ == '__main__':
    recs = [json.loads(l) for l in open(os.path.join(os.path.dirname(__file__), '../pass_records.jsonl'))]
    for run in sys.argv[1:]:
        name = os.path.basename(run.rstrip('/'))
        res = json.loads(open(os.path.join(run, 'results.jsonl')).readline());sp = res['splits']
        hw = round(res['drive_s']-sp['1118.7'], 1) if '1118.7' in sp and res.get('drive_s') else None   # CP10 to the run's end
        ahead, anyone = gaps(run);prov, emerg = bag_events(run)
        ch = [f"{x['lanes']}@{x['s']:.0f}/{x['v_kmh']:.0f}km/h {x['plan']['mode']}" for x in recs if x.get('run_dir') == name and x['kind'] == 'change']
        print(f"{name} {res['result']} hw {hw}s | closest ahead {ahead[0]} m {ahead[1] or ''} | closest any {anyone[0]} m")
        print(f"    provisional caps {len(prov)} {prov[:4]} | emergency brake ticks {len(emerg)} {emerg[:4]}")
        print(f"    changes {ch}")
