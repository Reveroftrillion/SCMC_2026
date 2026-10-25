"""Which lidar detections were real road users? Matches /perception/footprints (map frame) against the
ObjectInfo ground truth the runner logs (objects_<n>.jsonl, 2026-10-01) and prints the detections with no
object near them -- phantoms -- with where they were relative to us.

    python3 phantom_check.py ~/foscar_logs/bags/run_<stamp> practice/sim/runs/<stamp>/objects_0.jsonl [s_min s_max]

A detection is real if any of its outline points lies within MARGIN of a true object's box (centre pose,
length x width; the box is inflated because the logged pose and the scan are up to 0.05 s apart).
"""
import sys, json, bisect, math
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

MARGIN = 1.5
bag, objects_path = sys.argv[1], sys.argv[2]
s_min, s_max = (float(sys.argv[3]), float(sys.argv[4])) if len(sys.argv) > 4 else (-math.inf, math.inf)
truth = [json.loads(l) for l in open(objects_path)]
stamps = [r['t'] for r in truth]


def near_truth(t, pts):
    i = bisect.bisect_left(stamps, t)
    best = None
    for k in (i-1, i):
        if 0 <= k < len(truth) and (best is None or abs(truth[k]['t']-t) < abs(truth[best]['t']-t)):best = k
    if best is None or abs(truth[best]['t']-t) > .3:return None, None
    for o in truth[best]['objects']:
        c, s = math.cos(math.radians(o['yaw'])), math.sin(math.radians(o['yaw']))   # ObjectInfo heading is in degrees
        d = pts-[o['x'], o['y']]
        lon, lat = d@[c, s], d@[-s, c]
        if np.any((np.abs(lon) <= o['l']/2+MARGIN) & (np.abs(lat) <= o['w']/2+MARGIN)):return True, o
    return False, None


r = SequentialReader();r.open(StorageOptions(uri=bag, storage_id=''), ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
ego = gs = None;real = fake = 0;rows = []
while r.has_next():
    tp, d, t = r.read_next()
    if tp == '/behavior/highway':
        j = json.loads(deserialize_message(d, get_message(types[tp])).data)
        if 'ego_xy' in j:ego = (np.array(j['ego_xy']), j['ego_yaw']);gs = j['global_s']
        continue
    if tp != '/perception/footprints' or ego is None or not s_min <= gs <= s_max:continue
    m = json.loads(deserialize_message(d, get_message(types[tp])).data)
    xy, yaw = ego;R = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    for o in m['objects']:
        P = np.array(o['xy'])
        ok, _ = near_truth(m['stamp'], P)
        if ok is None:continue
        rel = (R@(P-xy).T).T
        if ok:real += 1
        else:
            fake += 1
            rows.append((gs, o['id'].split(':')[-1], rel[:, 0].min(), rel[:, 0].max(), rel[:, 1].min(), rel[:, 1].max(), len(P), o.get('kind')))
print(f'detections: real {real}, phantom {fake} ({100*fake/max(1, real+fake):.1f} %)')
for g, i, x0, x1, y0, y1, n, kind in rows:
    print(f'  s={g:7.1f} id {i:>4} x[{x0:5.1f}..{x1:5.1f}] y[{y0:+5.1f}..{y1:+5.1f}] n={n:2d} {kind}')
