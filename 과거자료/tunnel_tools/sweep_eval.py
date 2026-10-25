"""Tunnel speed sweep metrics per run (practice only): python3 sweep_eval.py RUN_DIR [...]"""
import os, sys, glob, json, math, bisect
from datetime import datetime
import numpy as np, yaml, rosbag2_py
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from foscar_control.core import Route
from foscar_control.mission_config import geometry_path
ROOT = Path('/home/foscar/Desktop/2026Morai/2026foscar-competition')
p = ROOT/'src/foscar_planning/config/missions.yaml'; g = yaml.safe_load(p.read_text())['geometry']
route = Route(np.loadtxt(geometry_path(g['route'], p.parent)))
FRONT, REAR, HALF = 3.8, .835, .946
def bag_for(objs_file):
    first = json.loads(open(objs_file).readline())['t']; best = None
    for b in glob.glob(os.path.expanduser('~/foscar_logs/bags/run_*')):
        try: t0 = datetime.strptime(os.path.basename(b)[4:], '%Y%m%d_%H%M%S').timestamp()
        except ValueError: continue
        if -5 <= first-t0 <= 300 and (best is None or abs(first-t0) < abs(best[0])): best = (first-t0, b)
    return best[1]
def body_gap(e, cx, cy, r):
    c, s = math.cos(e['yaw']), math.sin(e['yaw'])
    dx, dy = cx-e['x'], cy-e['y']; u = dx*c+dy*s; w = -dx*s+dy*c
    du = max(0., u-FRONT, -REAR-u); dw = max(0., abs(w)-HALF)
    return math.hypot(du, dw)-r
for run in sys.argv[1:]:
    res = {json.loads(l)['run']: json.loads(l) for l in open(run+'/results.jsonl')}
    for of in sorted(glob.glob(run+'/objects_*.jsonl')):
        idx = int(of[-7:-6]); rr = res.get(idx, {})
        L = [json.loads(x) for x in open(of)]; ot = [o['t'] for o in L]
        T = [json.loads(x) for x in open(of.replace('objects_', 'truth_'))]; tt = np.array([t['t'] for t in T])
        for e in T:
            if abs(e['yaw']) > 2*math.pi+1e-6 or 'yaw_deg' in e: e['yaw'] = math.radians(e['yaw'])
        xy = np.array([[e['x'], e['y']] for e in T]); v = np.r_[0, np.hypot(*np.diff(xy, axis=0).T)/np.maximum(np.diff(tt), 1e-3)]
        v = np.convolve(v, np.ones(5)/5, 'same')
        ss = np.array([route.project(e['x'], e['y'])[0] for e in T[::5]]); ts = tt[::5]
        def t_at(s): i = np.argmax(ss >= s); return ts[i] if ss[i] >= s else math.nan
        tun = t_at(1990.3)-t_at(1849.6)
        inside = (np.interp(tt, ts, ss) > 1849.6) & (np.interp(tt, ts, ss) < 1990.3)
        b = bag_for(of); r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=b, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
        types = {t.name: t.type for t in r.get_all_topics_and_types()}; Mt = get_message(types['/behavior/target'])
        r.set_filter(rosbag2_py.StorageFilter(topics=['/behavior/target'])); ys = []; last = ''
        while r.has_next():
            _, raw, tb = r.read_next(); m = deserialize_message(raw, Mt)
            if m.state.startswith('YIELD') and not last.startswith('YIELD'): ys.append(tb*1e-9)
            last = m.state
        out = []
        for pid, name in ((13, 'deer'), (59, 'person')):
            t0 = None; prev = None
            for o in L:
                q = [x for x in o['objects'] if x['id'] == pid][0]
                if prev and math.hypot(q['x']-prev[0], q['y']-prev[1]) > .05: t0 = o['t']; break
                prev = (q['x'], q['y'])
            gaps = []
            for o in L[::2]:
                q = [x for x in o['objects'] if x['id'] == pid][0]; e = T[min(int(np.searchsorted(tt, o['t'])), len(T)-1)]
                yaw = math.radians(q['yaw']); a = q['l']/2
                gaps.append(body_gap(e, q['x']+a*math.cos(yaw), q['y']+a*math.sin(yaw), min(q['l'], q['w'])/2))
            if t0 is None:
                out.append(f'{name}: no start'); continue
            i = int(np.searchsorted(tt, t0)); e = T[i]
            y = next((t for t in ys if t >= t0-1.), None)
            out.append(f"{name} start {math.hypot(prev[0]-e['x'], prev[1]-e['y']):.1f} m @ {v[i]*3.6:.0f} km/h, yield {('%+.2f s' % (y-t0)) if y else '-'}, closest body {min(gaps):.2f} m")
        print(f"{os.path.basename(run)}/{idx} {rr.get('result')} drive {rr.get('drive_s')} s tunnel {tun:.1f} s vmax {v[inside].max()*3.6:.0f} km/h stops {rr.get('full_stops')} coll {len(rr.get('collisions', []))} | " + ' | '.join(out))
