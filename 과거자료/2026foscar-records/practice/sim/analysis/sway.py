#!/usr/bin/env python3
"""PRACTICE ONLY: highway lane-keeping sway by speed. For FOLLOW ticks (no change under way) the car's offset from
its lane centre and heading error, from /behavior/highway (scene lane edges) and /control/telemetry steering.

    python3 practice/sim/analysis/sway.py <bag> [<bag> ...]

Per speed band: RMS offset, p95 |offset|, RMS heading error, RMS steering rate (deg/s) -- the sway the user saw
at 100-117 km/h in lane 2 (2026-10-01).
"""
import sys, json, math
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def lane_offset(scene, lane, P, yaw):
    for L in scene['lanes']:
        if L['lane'] != lane:continue
        a = np.array(L['edges'][0]);b = np.array(L['edges'][1])
        ia = np.argmin(np.linalg.norm(a-P, axis=1));A = a[ia];B = b[np.argmin(np.linalg.norm(b-P, axis=1))];u = B-A;w = np.linalg.norm(u)
        f = (P-A)@u/(u@u)
        j = min(ia+1, len(a)-1);i = j-1;h = math.atan2(*(a[j]-a[i])[::-1])
        return (f-.5)*w, (yaw-h+math.pi) % (2*math.pi)-math.pi
    return None



if __name__ == '__main__':
    rows = []
    for bag in sys.argv[1:]:
        r = SequentialReader();r.open(StorageOptions(uri=bag, storage_id=''), ConverterOptions('', ''))
        types = {t.name: t.type for t in r.get_all_topics_and_types()}
        steer = []
        while r.has_next():
            tp, d, t = r.read_next()
            if tp == '/control/telemetry':
                try:j = json.loads(deserialize_message(d, get_message(types[tp])).data);steer.append((t*1e-9, j.get('steering_rad', 0.)))
                except Exception:pass
                continue
            if tp != '/behavior/highway':continue
            j = json.loads(deserialize_message(d, get_message(types[tp])).data)
            if j.get('state') != 'FOLLOW' or 'scene' not in j:continue
            o = lane_offset(j['scene'], j.get('current_lane'), np.array(j['ego_xy']), j['ego_yaw'])
            if o is None or abs(o[0]) > 1.5:continue
            rows.append((t*1e-9, j['ego_speed']*3.6, o[0], math.degrees(o[1]), bag))
        st = np.array(steer)
        if len(st) > 2:
            rate = np.degrees(np.gradient(st[:, 1], st[:, 0]))
            for k, row in enumerate(rows):
                if row[4] == bag and len(row) == 5:
                    i = min(np.searchsorted(st[:, 0], row[0]), len(st)-1);rows[k] = row+(rate[i],)
    rows = [r for r in rows if len(r) == 6]
    print(f'FOLLOW ticks: {len(rows)}')
    for lo, hi in ((50, 70), (70, 90), (90, 100), (100, 110), (110, 125)):
        sel = [r for r in rows if lo <= r[1] < hi]
        if not sel:continue
        off = np.array([r[2] for r in sel]);hd = np.array([r[3] for r in sel]);sr = np.array([r[5] for r in sel])
        print(f'{lo:3d}-{hi:3d} km/h n={len(sel):4d}  offset RMS {np.sqrt(np.mean(off**2)):.2f} m p95 {np.percentile(abs(off), 95):.2f} m'
              f'  heading RMS {np.sqrt(np.mean(hd**2)):.2f} deg  steer rate RMS {np.sqrt(np.mean(sr**2)):.2f} deg/s')
