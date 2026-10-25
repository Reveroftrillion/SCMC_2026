"""Ring speed estimates replayed from run bags against the NPC truth (practice only).

Feeds /perception/footprints (and the lidar position from /localization/odometry) of each bag into a fresh RingLane,
exactly as behavior_node does, and compares every run speed given as measured (sigma <= SURE_SD) with the truth
speed of the ring car under it (objects_<i>.jsonl of the loop_runner run the bag belongs to).

    python3 practice/sim/rb_speed_replay.py practice/sim/runs/<run> [...] [--set ACCEL_SIGMA=2 ...]

Prints per setting: count, bias, p90 |err|, the share of measured estimates off by more than 1 m/s, and the same
for cars changing speed (truth |accel| > 0.3 m/s2) -- the 2026-10-09 contact was a car speeding up read 1.2 m/s slow
with a 0.3 m/s sigma.
"""
import argparse
import glob
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import rb_rear_log as R                                                  # noqa: E402
from foscar_planning.roundabout_boxes import RingLane                   # noqa: E402

BAGS = Path.home()/'foscar_logs'/'bags'
SURE_SD = 1.


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/perception/footprints', '/localization/odometry']))
    feet, odo = [], []
    while r.has_next():
        topic, raw, t = r.read_next()
        if topic == '/perception/footprints':
            feet.append(json.loads(deserialize_message(raw, String).data))
        else:
            m = deserialize_message(raw, Odometry)
            q = m.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            odo.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9, m.pose.pose.position.x, m.pose.pose.position.y, yaw))
    return feet, np.array(odo)


def truth_index(objs_file):
    objs = [json.loads(x) for x in open(objs_file)]
    return objs, np.array([o['t'] for o in objs])


def pairs(run_dirs):
    """(bag, objects file) by time overlap."""
    bags = sorted(BAGS.glob('run_*'))
    out = []
    for d in run_dirs:
        for f in sorted(glob.glob(os.path.join(d, 'objects_*.jsonl'))):
            first = json.loads(open(f).readline())['t']
            best = None
            for b in bags:
                meta = (b/'metadata.yaml')
                if not meta.exists():
                    continue
                # bag name run_YYYYmmdd_HHMMSS (local time) -> compare with the objects' first stamp
                from datetime import datetime
                try:
                    t0 = datetime.strptime(b.name[4:], '%Y%m%d_%H%M%S').timestamp()
                except ValueError:
                    continue
                if 0. <= first-t0 <= 40. and (best is None or first-t0 < best[0]):
                    best = (first-t0, b)
            if best is not None:
                out.append((best[1], f))
    return out


def replay(bag, objs_file, settings, ring):
    route, c, g, _, _ = ring
    for k, v in settings.items():
        setattr(RingLane, k, v)
    lane = RingLane(c, g.origin)
    feet, odo = read_bag(bag)
    objs, ot = truth_index(objs_file)
    # truth speed history per id (for acceleration)
    hist = {}
    for o in objs:
        for v in o['objects']:
            if v['type'] == 1:
                hist.setdefault(v['id'], []).append((o['t'], v['v']/3.6))
    acc = {}
    for i, h in hist.items():
        h = np.array(h)
        if len(h) > 5:
            acc[i] = (h[:, 0], np.gradient(h[:, 1], h[:, 0]))
    errs = []
    for data in feet:
        stamp = data.get('stamp')
        if data.get('frame') != 'map' or not data.get('scan_valid', True) or stamp is None or not len(odo):
            continue
        k = int(np.argmin(abs(odo[:, 0]-stamp)))
        _, x, y, yaw = odo[k]
        hulls = {str(o['id']): o['xy'] for o in data.get('objects', []) if isinstance(o, dict) and o.get('xy')}
        heights = {str(o['id']): o.get('height') for o in data.get('objects', []) if isinstance(o, dict) and o.get('xy')}
        if lane.update(stamp, hulls, (x+2.*math.cos(yaw), y+2.*math.sin(yaw)), heights) is False:
            continue
        o = objs[int(np.argmin(abs(ot-stamp)))]
        if abs(o['t']-stamp) > .1:
            continue
        tr = []
        for v in o['objects']:
            if v['type'] != 1:
                continue
            vy = math.radians(v['yaw']); px, py = v['x']+R.NPC_AHEAD*math.cos(vy), v['y']+R.NPC_AHEAD*math.sin(vy)
            if abs(math.hypot(px-c.center[0], py-c.center[1])-c.radius) > 3.:
                continue
            u = lane.u_of(px, py)
            tr.append((u-v['l']/2, u+v['l']/2, v['v']/3.6, v['id']))
        for r in lane.runs:
            if r.coasted > 0. or r.speed is None or not r.speed[1] == r.speed[1] or r.speed[1] > SURE_SD:
                continue
            if not -30. < r.hi < 15.:
                continue
            m = [t for t in tr if min(t[1], r.hi)-max(t[0], r.lo) > 1.]
            if len(m) != 1:
                continue
            a = 0.
            if m[0][3] in acc:
                ts, aa = acc[m[0][3]]
                a = float(aa[int(np.argmin(abs(ts-stamp)))])
            errs.append((r.speed[0]-m[0][2], r.speed[1], a, r.hi))
    return errs


def report(name, errs):
    e = np.array(errs)
    if not len(e):
        print(name, 'no data'); return
    d, sd, a, hi = e[:, 0], e[:, 1], e[:, 2], e[:, 3]
    ch = abs(a) > .3
    near = (hi > -12.) & (hi < -4.)
    def line(mask):
        x = d[mask]
        if not len(x):
            return 'n 0'
        k = 3.6   # reported in km/h (user)
        return (f"n {len(x)} bias {x.mean()*k:+.1f} km/h p90|err| {np.percentile(abs(x), 90)*k:.1f} km/h "
                f">3.6 km/h {np.mean(abs(x) > 1)*100:.1f}% read >3.6 km/h fast {np.mean(x > 1)*100:.1f}% "
                f"outside 2sd {np.mean(abs(x) > 2*sd[mask])*100:.1f}%")
    print(f"{name:28s} all: {line(np.ones(len(d), bool))}\n{'':28s} changing speed: {line(ch)}"
          f"\n{'':28s} just before our entry: {line(near)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--set', action='append', default=[], help='RingLane attribute=value, sweeps with ","')
    args = ap.parse_args()
    ring = R.ring()
    grid = [{}]
    for s in args.set:
        k, vals = s.split('=')
        grid = [dict(g, **{k: float(v)}) for g in grid for v in vals.split(',')]
    prs = pairs(args.runs)
    print(len(prs), 'bags')
    base = {k: getattr(RingLane, k) for g in grid for k in g}
    for g in grid:
        errs = []
        for bag, f in prs:
            errs += replay(bag, f, g, ring)
        for k, v in base.items():
            setattr(RingLane, k, v)
        report(json.dumps(g) if g else 'current', errs)


if __name__ == '__main__':
    main()
