"""Per run: result, deer/person yield latency and gap, cone/barricade first detection, footprint latency, ghosts
(practice only). python3 compare.py RUN_DIR [RUN_DIR...]"""
import sys, os, glob, json, math, re, subprocess
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
S = os.path.dirname(os.path.abspath(__file__))
LANES = None


def sd(x, y):
    s, dist, h = route.project(x, y); qx, qy = route.at(s)
    return s, dist*np.sign(math.cos(h)*(y-qy)-math.sin(h)*(x-qx))


def bag_for(R, i):
    t0 = json.loads(open(f'{R}/objects_{i}.jsonl').readline())['t']; best = None
    for b in glob.glob(os.path.expanduser('~/foscar_logs/bags/run_*')):
        try: tb = datetime.strptime(os.path.basename(b)[4:19], '%Y%m%d_%H%M%S').timestamp()
        except ValueError: continue
        if -5 <= t0-tb <= 300 and (best is None or t0-tb < best[0]): best = (t0-tb, b)
    return best[1]


def scan(b):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=b, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {x.name: x.type for x in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/perception/footprints', '/localization/s']))
    Ms, M = get_message(types['/localization/s']), get_message(types['/perception/footprints'])
    es = 0.; lat = []; cone = None; bar = None; ghosts = 0
    while r.has_next():
        tp, raw, t = r.read_next()
        if tp == '/localization/s':
            es = deserialize_message(raw, Ms).data; continue
        d = json.loads(deserialize_message(raw, M).data)
        if not 1868 < es < 1960: continue
        lat.append(t*1e-9-d['stamp'])
        for f in d['objects']:
            q = [sd(*pp) for pp in f['xy']]
            if not q: continue
            ms, md = np.mean([a for a, _ in q]), np.mean([c for _, c in q])
            if cone is None and 1955.5 < ms < 1958 and md > 1.5: cone = 1956.6-es
            if bar is None and 1933.5 < ms < 1937.5 and -2.5 < md < 2.5: bar = 1935.6-es
            # on the lanes (d -1.75..5.25) where nothing stands (cars, deer, person, barricade, cones excluded by s)
            if -1.5 < md < 5. and ms > es and not (1878 < ms < 1884 or 1892 < ms < 1899 or 1910 < ms < 1918
                                                  or 1933 < ms < 1938 or 1955 < ms < 1958):
                ghosts += 1
    lat = np.array(lat)
    return cone, bar, lat, ghosts


for R in sys.argv[1:]:
    for i in range(20):
        if not os.path.exists(f'{R}/objects_{i}.jsonl') or not os.path.exists(f'{R}/launch_{i}.log'): continue
        res = [json.loads(l) for l in open(f'{R}/results.jsonl')]
        rr = next((x for x in res if x.get('run') == i), {})
        ev = subprocess.run(['python3', f'{S}/sweep_eval.py', R], capture_output=True, text=True).stdout.splitlines()
        line = next((l for l in ev if l.startswith(f'{os.path.basename(R)}/{i} ')), '')
        m = re.findall(r'(deer|person) start [\d.]+ m @ \d+ km/h, yield ([+-][\d.]+) s, closest body ([\d.]+) m', line)
        cone, bar, lat, ghosts = scan(bag_for(R, i))
        log = open(f'{R}/launch_{i}.log', errors='ignore').read()
        hz = re.findall(r'lidar_bridge\]: ([\d.]+) Hz', log)
        print('%s/%d %-8s lidar %s Hz | %s | cone %.1f m bar %.1f m | latency %.0f/%.0f/%.0f ms | lane ghosts %d | yieldIN %d' % (
            os.path.basename(R), i, rr.get('result', '?')[:8], '/'.join(hz[:2]),
            ' '.join('%s %s s %s m' % x for x in m), cone or -1, bar or -1, lat.mean()*1e3, np.percentile(lat, 95)*1e3,
            lat.max()*1e3, ghosts, len(re.findall(r'behavior_node.*→ YIELD', log))))
