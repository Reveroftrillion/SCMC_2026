"""False positives in the tunnel against truth: ghosts, wrong kinds, outline errors (practice only).
python3 tun_fp.py RUN_DIR [RUN_DIR...]   (all objects_<i>.jsonl of each)"""
import os, sys, glob, json, math, bisect
from datetime import datetime
from collections import Counter, defaultdict
import numpy as np, yaml, rosbag2_py
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from foscar_control.core import Route
from foscar_control.mission_config import geometry_path
ROOT = Path('/home/foscar/Desktop/2026Morai/2026foscar-competition')
p = ROOT/'src/foscar_planning/config/missions.yaml'; g = yaml.safe_load(p.read_text())['geometry']
route = Route(np.loadtxt(geometry_path(g['route'], p.parent)))
def sd(x, y):
    s, dist, h = route.project(x, y); P = np.interp(s, route.s, route.points[:, 0]), np.interp(s, route.s, route.points[:, 1])
    return s, -(x-P[0])*math.sin(h)+(y-P[1])*math.cos(h)
def bag_for(objs_file):
    first = json.loads(open(objs_file).readline())['t']; best = None
    for b in glob.glob(os.path.expanduser('~/foscar_logs/bags/run_*')):
        try: t0 = datetime.strptime(os.path.basename(b)[4:], '%Y%m%d_%H%M%S').timestamp()
        except ValueError: continue
        if -5 <= first-t0 <= 300 and (best is None or abs(first-t0) < abs(best[0])): best = (first-t0, b)
    return best[1]
def shapes(o):
    out = []
    for v in o['objects']:
        if v['type'] == 1: continue
        yaw = math.radians(v['yaw']); c, s = math.cos(yaw), math.sin(yaw); a = v['l']/2-v.get('ro', 0.)
        cx, cy = v['x']+a*c, v['y']+a*s
        if v['type'] == 0:   # people/animals: circle (the deer's truth l/w look swapped)
            out.append((v['id'], 'PED', ('circle', cx, cy, max(v['l'], v['w'])/2), v.get('v', 0.)))
        else:
            out.append((v['id'], 'OBS', ('box', cx, cy, c, s, v['l']/2, v['w']/2), 0.))
    return out
def dist(P, sh):
    if sh[0] == 'circle':
        return float(max(0., np.min(np.hypot(P[:, 0]-sh[1], P[:, 1]-sh[2]))-sh[3]))
    _, cx, cy, c, s, hl, hw = sh
    X = P-[cx, cy]; u = X[:, 0]*c+X[:, 1]*s; w = -X[:, 0]*s+X[:, 1]*c
    return float(np.min(np.hypot(np.maximum(abs(u)-hl, 0), np.maximum(abs(w)-hw, 0))))
def moving(pid, o):
    i = ot_index[id(o)]; j = max(0, i-3)
    a = [v for v in Lcur[j]['objects'] if v['id'] == pid][0]; b = [v for v in o['objects'] if v['id'] == pid][0]
    return math.hypot(b['x']-a['x'], b['y']-a['y'])/max(1e-3, o['t']-Lcur[j]['t']) > .3
per_obj = defaultdict(Counter)
tot = Counter(); ghosts = []; wrong = defaultdict(list); frames = 0
if __name__ != '__main__': sys.argv = sys.argv[:1]
for run in sys.argv[1:]:
    for of in sorted(glob.glob(run+'/objects_*.jsonl')):
        L = [json.loads(x) for x in open(of)]; ot = [o['t'] for o in L]
        Lcur = L; ot_index = {id(o): i for i, o in enumerate(L)}
        T = [json.loads(x) for x in open(of.replace('objects_', 'truth_'))]; tt = [t['t'] for t in T]
        b = bag_for(of); r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=b, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
        r.set_filter(rosbag2_py.StorageFilter(topics=['/perception/footprints']))
        while r.has_next():
            _, raw, _ = r.read_next(); d = json.loads(deserialize_message(raw, String).data); st = d.get('stamp')
            if d.get('frame') != 'map' or st is None: continue
            o = L[min(bisect.bisect(ot, st), len(L)-1)]; e = T[min(bisect.bisect(tt, st), len(T)-1)]
            if abs(o['t']-st) > .2: continue
            es, _ = sd(e['x'], e['y'])
            if not 1800 <= es <= 1990: continue
            frames += 1
            S = shapes(o)
            for f in d.get('objects', []):
                if not isinstance(f, dict) or not f.get('xy') or len(f['xy']) < 3: continue
                P = np.asarray(f['xy'], float)[:, :2]; c = P.mean(axis=0)
                fs, fd = sd(*c)
                ahead = fs-es
                if not -2 < ahead < 40: continue
                k = f.get('kind'); tot[k] += 1
                dd, best = min(((dist(P, sh), (i, typ, sp)) for i, typ, sh, sp in S), default=(99, None))
                if dd <= .7: per_obj[best[:2]][k] += 1
                if dd > .7:
                    ghosts.append((round(es, 1), round(ahead, 1), round(fs, 1), round(fd, 2), k, round(f.get('height', 0), 2),
                                   round(float(np.ptp(P[:, 0])), 1), round(float(np.ptp(P[:, 1])), 1), round(dd, 1), os.path.basename(run), of[-7:-6]))
                elif best[1] == 'OBS' and k == 'PEDESTRIAN':
                    wrong[('OBS->PED', best[0])].append((round(es, 1), round(ahead, 1), round(f.get('height', 0), 2), os.path.basename(run)+'/'+of[-7:-6]))
                elif best[1] == 'PED' and k != 'PEDESTRIAN' and moving(best[0], o):
                    wrong[('moving PED->'+str(k), best[0])].append((round(es, 1), round(ahead, 1), round(f.get('height', 0), 2), os.path.basename(run)+'/'+of[-7:-6]))
print('frames', frames, 'footprints by kind', dict(tot))
print('ghosts (no truth object within 0.7 m):', len(ghosts))
by = Counter((round(gh[2]/5)*5, round(gh[3])) for gh in ghosts)
for (s, d), n in by.most_common(15):
    ex = [gh for gh in ghosts if round(gh[2]/5)*5 == s and round(gh[3]) == d]
    print(f'   s~{s} d~{d:+d}: {n} frames, kinds {dict(Counter(x[4] for x in ex))}, height {min(x[5] for x in ex)}-{max(x[5] for x in ex)}, ahead {min(x[1] for x in ex)}-{max(x[1] for x in ex)} m, runs {sorted(set(x[9]+"/"+x[10] for x in ex))[:4]}')
print('labels per truth object:')
for key in sorted(per_obj, key=str): print('  ', key, dict(per_obj[key]))
print('wrong kinds:')
for k, v in wrong.items():
    print('  ', k, len(v), 'frames; first', v[0], 'runs', sorted(set(x[3] for x in v)))
