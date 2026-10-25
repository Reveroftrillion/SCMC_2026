#!/usr/bin/env python3
"""Practice only: how hard a swerve can be at a given speed, measured (user 2026-09-29).

For every run of a loop_runner batch (bag + launch log + the scenario's ground-truth obstacles):
    moves     each move the tunnel slalom handed to the tracker (/behavior/target entry spec): entry speed,
              shift, curve length, peak curvature k = 5.77 shift / L^2, demanded lateral acceleration v^2 k,
              and the largest deviation of the actual lateral position from that quintic
    clearance per scenario barricade, the smallest gap between the actual car body (odometry pose and
              heading, rear axle -0.8 .. +3.8 m, width 1.892) and the barricade's true 1.2 m footprint;
              negative = contact

    python3 practice/sim/swerve_analysis.py <runs dir> [<runs dir> ...] --scenario practice/scenarios/X.json
"""
import argparse
import glob
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
REAR, FRONT, HALF_W = .835, 3.8, 1.892/2.


def route():
    P = np.loadtxt(ROOT/'src/foscar_bringup/routes/global_path.txt')[:, :2]
    S = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
    return P, S


def to_frenet(P, S, x, y):
    i = int(np.argmin(np.hypot(P[:, 0]-x, P[:, 1]-y)))
    j = min(i+1, len(P)-1)
    t = P[j]-P[i]; t /= np.linalg.norm(t)
    q = np.array([x, y])-P[i]
    return S[i]+q@t, t[0]*q[1]-t[1]*q[0], math.atan2(t[1], t[0])


def quintic(d0, d1, start, length, s):
    u = min(1., max(0., (s-start)/max(length, 1e-6)))
    return d0+(d1-d0)*u**3*(10-15*u+6*u*u)


def bag_series(bag):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = SequentialReader(); r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    s = None; tg = None; pose = []; targets = []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic not in ('/localization/s', '/localization/odometry', '/behavior/target'):
            continue
        m = deserialize_message(data, get_message(types[topic]))
        if topic == '/localization/s':
            s = m.data
        elif topic == '/behavior/target':
            targets.append((t*1e-9, s, m.lat_offset, m.entry_s, m.entry_length_m, m.state))
        elif s is not None:
            q = m.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            pose.append((t*1e-9, s, m.pose.pose.position.x, m.pose.pose.position.y, yaw, m.twist.twist.linear.x))
    return pose, targets


def barricades(scenario, P, S):
    d = json.loads(Path(scenario).read_text())
    out = []
    for o in d['objectList']:
        x, y = float(o['pos']['x']), float(o['pos']['y'])
        s, dd, h = to_frenet(P, S, x, y)
        half = .6*float(o['scale']['x'])
        if 1760 < s < 2010:
            out.append(dict(x=x, y=y, s=s, d=dd, half=half))
    return out


def body_gap(px, py, yaw, ob):
    """Smallest distance between the car rectangle and an axis-free square barricade (sampled)."""
    c, sn = math.cos(yaw), math.sin(yaw)
    # barricade corners -> car frame
    pts = []
    for ax in np.linspace(-ob['half'], ob['half'], 7):
        for ay in (-ob['half'], ob['half']):
            for (bx, by) in ((ax, ay), (ay, ax)):
                wx, wy = ob['x']+bx, ob['y']+by
                dx, dy = wx-px, wy-py
                pts.append((c*dx+sn*dy, -sn*dx+c*dy))
    best = math.inf
    for lx, ly in pts:
        ox = max(-REAR-lx, 0., lx-FRONT); oy = max(-HALF_W-ly, 0., ly-HALF_W)
        inside = -REAR <= lx <= FRONT and -HALF_W <= ly <= HALF_W
        g = -min(lx+REAR, FRONT-lx, ly+HALF_W, HALF_W-ly) if inside else math.hypot(ox, oy)
        best = min(best, g)
    return best


def analyse_run(bag, P, S, obs):
    pose, targets = bag_series(bag)
    if not pose:
        return None
    ps = np.array([p[1] for p in pose])
    d_act = []
    for t, s, x, y, yaw, v in pose:
        _, dd, _ = to_frenet(P, S, x, y); d_act.append(dd)
    d_act = np.array(d_act)
    # moves: entry specs actually driven (still the current spec until 80 % of the curve was driven)
    specs = []                                   # (s when it became current, spec, offset held before)
    prev_d = 0.
    for t, s, lat, es, el, state in targets:
        if s is None:
            continue
        spec = (round(es, 2), round(el, 2), round(lat, 2)) if (el > 0 and math.isfinite(es)) else None
        if not specs or specs[-1][1] != spec:
            specs.append((s, spec, prev_d))
        prev_d = lat
    moves = []
    for i, (s_on, spec, held) in enumerate(specs):
        if spec is None:
            continue
        es, el, lat = spec
        s_off = specs[i+1][0] if i+1 < len(specs) else math.inf
        if s_off < es + .8*el:                   # replaced before it was driven
            continue
        i0 = int(np.argmin(abs(ps-max(es, s_on))))
        d0 = float(d_act[i0])
        shift = abs(lat-d0)
        if shift < .2:
            continue
        v = pose[i0][5]
        sel = (ps >= max(es, s_on)) & (ps <= min(es+el+8., s_off))
        dev = max((abs(d_act[i]-quintic(d0, lat, max(es, s_on), es+el-max(es, s_on), ps[i])) if ps[i] <= es+el
                   else abs(d_act[i]-lat)) for i in np.nonzero(sel)[0]) if sel.any() else math.nan
        length = es+el-max(es, s_on)
        k = 5.77*shift/max(length, 1e-6)**2
        moves.append(dict(start=round(max(es, s_on), 1), v_kmh=round(v*3.6, 1), shift=round(shift, 2),
                          length=round(length, 1), k=round(k, 3), a_dem=round(v*v*k, 2), dev=round(dev, 2)))
    gaps = []
    for ob in obs:
        near = [p for p in pose if abs(p[1]-ob['s']) < 8.]
        if not near:
            continue
        g = min(body_gap(x, y, yaw, ob) for _, s, x, y, yaw, v in near)
        v = min(near, key=lambda p: abs(p[1]-ob['s']))[5]
        gaps.append(dict(s=round(ob['s'], 1), d=round(ob['d'], 2), gap=round(g, 2), v_kmh=round(v*3.6, 1)))
    return dict(moves=moves, gaps=gaps)


def phantoms(log, obs):
    """Planner boxes (SLALOM lines) whose centre is more than 1.5 m from every true barricade."""
    out = {}
    for line in Path(log).read_text().splitlines():
        m = re.search(r'SLALOM s=([\d.]+) d_now=\S+ v=([\d.-]+) groups=(\[.*?\]\]) ', line)
        if not m:
            continue
        s, v = float(m.group(1)), float(m.group(2))
        for bs, dmin, dmax in re.findall(r'\(([\d.]+), (-?[\d.]+), (-?[\d.]+)\)', m.group(3)):
            bs, dmin, dmax = float(bs), float(dmin), float(dmax)
            cd = (dmin+dmax)/2
            if all(abs(bs+.6-o['s']) > 1.5 or abs(cd-o['d']) > 1.5 for o in obs):
                key = round(bs)
                if key not in out:
                    out[key] = dict(box_s=bs, d=(dmin, dmax), first_seen_at=s, v_kmh=v)
    return list(out.values())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    P, S = route()
    obs = barricades(a.scenario, P, S)
    rows = []
    for d in a.runs:
        res = [json.loads(l) for l in open(Path(d)/'results.jsonl')]
        for r in res:
            log = Path(d)/f"launch_{r['run']}.log"
            m = re.search(r'run_2026\d{4}_\d{6}', log.read_text()) if log.exists() else None
            if not m:
                continue
            bag = str(Path.home()/'foscar_logs/bags'/m.group(0))
            out = analyse_run(bag, P, S, obs)
            if out is None:
                continue
            row = dict(batch=Path(d).name, run=r['run'], result=r['result'],
                       pushed=[p['id'] for p in r.get('pushed', [])], phantoms=phantoms(log, obs), **out)
            rows.append(row)
            print(json.dumps(row, ensure_ascii=False))
    if a.out:
        Path(a.out).write_text('\n'.join(json.dumps(r) for r in rows)+'\n')


if __name__ == '__main__':
    main()
