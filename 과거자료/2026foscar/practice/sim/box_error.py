#!/usr/bin/env python3
"""Practice only: how far the planner's remembered barricade boxes are from the true barricades, and why
(user 2026-09-29: the A box was estimated wider than it is and the car had no room left for B).

Per loop_runner run (bag + truth_<n>.jsonl written by the runner + the scenario's true obstacles):
    boxes      the AVOID boxes as drawn on /behavior/markers (ns 'obstacles'), in the map frame the planner
               uses (dead-reckoned in the tunnel)
    total      box vs true barricade, both in route Frenet coordinates: what the planner works with
    loc        the car pose error (odometry vs truth), lateral / along the route
    percept    the box moved with the car's pose error removed (box relative to the car, placed with the
               true pose) vs the true barricade: the perception/estimation part
The truth reference point is not the rear axle: a constant car-frame offset between odometry and truth is
estimated before the tunnel (GPS good) and removed.

    python3 practice/sim/box_error.py <runs dir> --scenario practice/scenarios/X.json
"""
import argparse, bisect, json, math, re, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from swerve_analysis import route, to_frenet, barricades

HALF_TRUE = .6 + .2          # 1.2 m cube + the planner's event pad (box_from_event pad_m)


def read_bag(bag):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = SequentialReader(); r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    odom, boxes = [], []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == '/localization/odometry':
            m = deserialize_message(data, get_message(types[topic]))
            q = m.pose.pose.orientation
            odom.append((t*1e-9, m.pose.pose.position.x, m.pose.pose.position.y,
                         math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)), m.twist.twist.linear.x))
        elif topic == '/behavior/markers':
            m = deserialize_message(data, get_message(types[topic]))
            polys = [[(p.x, p.y) for p in mk.points[:4]] for mk in m.markers if mk.ns == 'obstacles' and len(mk.points) >= 4]
            if polys:
                boxes.append((t*1e-9, polys))
    return odom, boxes


def interp(series, t):
    ts = [s[0] for s in series]
    i = bisect.bisect_left(ts, t)
    if i <= 0: return series[0]
    if i >= len(series): return series[-1]
    a, b = series[i-1], series[i]
    w = (t-a[0])/max(b[0]-a[0], 1e-9)
    yaw = a[3] + math.remainder(b[3]-a[3], 2*math.pi)*w
    return (t, a[1]+(b[1]-a[1])*w, a[2]+(b[2]-a[2])*w, yaw) + tuple(a[4:])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs'); ap.add_argument('--scenario', required=True)
    a = ap.parse_args()
    P, S = route(); obs = barricades(a.scenario, P, S)
    names = {round(o['s']): chr(ord('A')+i) for i, o in enumerate(sorted(obs, key=lambda o: o['s']))}
    rows = []
    for res in [json.loads(l) for l in open(Path(a.runs)/'results.jsonl')]:
        n = res['run']
        log = (Path(a.runs)/f'launch_{n}.log').read_text()
        m = re.search(r'run_2026\d{4}_\d{6}', log)
        tf = Path(a.runs)/f'truth_{n}.jsonl'
        if not m or not tf.exists():
            continue
        truth = [json.loads(l) for l in open(tf)]
        truth = [(r['t'], r['x'], r['y'], r['yaw']) for r in truth]
        odom, boxes = read_bag(str(Path.home()/'foscar_logs/bags'/m.group(0)))
        if not odom or not truth or not boxes:
            continue
        # constant car-frame offset odometry -> truth, estimated before the tunnel (route s < 1860)
        offs = []
        for t, x, y, yaw, v in odom[::10]:
            s, d, _ = to_frenet(P, S, x, y)
            if 1800 < s < 1860 and truth[0][0] <= t <= truth[-1][0]:
                g = interp(truth, t)
                dx, dy = g[1]-x, g[2]-y
                offs.append((math.cos(yaw)*dx+math.sin(yaw)*dy, -math.sin(yaw)*dx+math.cos(yaw)*dy))
        ox, oy = (np.median([o[0] for o in offs]), np.median([o[1] for o in offs])) if offs else (0., 0.)
        for t, polys in boxes:
            if not truth[0][0] <= t <= truth[-1][0]:
                continue
            o = interp(odom, t); g = interp(truth, t)
            # the truth pose moved back to the odometry reference point
            gx = g[1] - (math.cos(g[3])*ox - math.sin(g[3])*oy); gy = g[2] - (math.sin(g[3])*ox + math.cos(g[3])*oy)
            s_o, d_o, _ = to_frenet(P, S, o[1], o[2]); s_g, d_g, _ = to_frenet(P, S, gx, gy)
            for poly in polys:
                cx, cy = np.mean([p[0] for p in poly]), np.mean([p[1] for p in poly])
                fr = [to_frenet(P, S, px, py) for px, py in poly]
                ss, dd = [f[0] for f in fr], [f[1] for f in fr]
                s_b, d_b = (min(ss)+max(ss))/2, (min(dd)+max(dd))/2
                # same box relative to the car, placed with the true pose
                c, sn = math.cos(o[3]), math.sin(o[3]); lx, ly = c*(cx-o[1])+sn*(cy-o[2]), -sn*(cx-o[1])+c*(cy-o[2])
                c2, s2 = math.cos(g[3]), math.sin(g[3]); tx, ty = gx+c2*lx-s2*ly, gy+s2*lx+c2*ly
                s_t, d_t, _ = to_frenet(P, S, tx, ty)
                ob = min(obs, key=lambda q: abs(q['s']-s_t) + abs(q['d']-d_t))
                if abs(ob['s']-s_t) > 4 or abs(ob['d']-d_t) > 2.5:
                    continue
                rows.append(dict(run=n, bar=names[round(ob['s'])], range=round(ob['s']-s_g, 1), v=round(o[4]*3.6, 1),
                                 total_d=d_b-ob['d'], percept_d=d_t-ob['d'], loc_d=d_o-d_g, loc_s=s_o-s_g,
                                 width=max(dd)-min(dd), dmax_over=max(dd)-(ob['d']+HALF_TRUE),
                                 dmin_over=(ob['d']-HALF_TRUE)-min(dd)))
    if not rows:
        print('no rows'); return
    def summary(sel, label):
        if not sel: return
        f = lambda k: np.median([r[k] for r in sel])
        p90 = lambda k: np.percentile([abs(r[k]) for r in sel], 90)
        print(f"{label:22s} n={len(sel):4d}  centre total {f('total_d'):+.2f} (|p90| {p90('total_d'):.2f})  "
              f"percept {f('percept_d'):+.2f}  loc_d {f('loc_d'):+.2f} loc_s {f('loc_s'):+.2f}  "
              f"width {f('width'):.2f} (true {2*HALF_TRUE:.1f})  over +side {f('dmax_over'):+.2f} -side {f('dmin_over'):+.2f}")
    for bar in sorted({r['bar'] for r in rows}):
        sel = [r for r in rows if r['bar'] == bar]
        summary(sel, f'{bar} all')
        for lo, hi in ((20, 40), (10, 20), (5, 10), (0, 5), (-10, 0)):
            summary([r for r in sel if lo <= r['range'] < hi], f'{bar} range {lo}..{hi} m')


if __name__ == '__main__':
    main()
