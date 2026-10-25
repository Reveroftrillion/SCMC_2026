#!/usr/bin/env python3
"""PRACTICE ONLY: response to the test steering kick (low_speed.yaml steer_kick_*, 2026-10-01).

    python3 practice/sim/analysis/kick_report.py <run_dir> [...]     (bag found as the first one after the run dir)

From the kick (first tick at s >= KICK_S in lane-keeping) on: signed offset from the lane centre (highway scene),
peak |offset|, how many times it swung past +-0.05 m (half-cycles), the time until it stays within 0.05 m for 1 s,
and the steering peak.
"""
import sys, os, json, math
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from steer_report import bag_for  # noqa: E402
from sway import lane_offset      # noqa: E402  (module runs nothing without bags in argv)
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

KICK_S = float(os.environ.get('KICK_S', 1450.))
BAND = .08     # above the ambient +-0.05 m


def response(run_dir):
    r = SequentialReader();r.open(StorageOptions(uri=bag_for(run_dir), storage_id=''), ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    rows = [];steer = []
    while r.has_next():
        tp, d, t = r.read_next()
        if tp == '/control/telemetry':
            try:steer.append((t*1e-9, math.degrees(json.loads(deserialize_message(d, get_message(types[tp])).data)['steering_rad'])))
            except Exception:pass
        elif tp == '/behavior/highway':
            j = json.loads(deserialize_message(d, get_message(types[tp])).data)
            if 'scene' not in j or j['global_s'] < KICK_S-30 or j.get('state') not in ('FOLLOW', 'SETTLE'):continue
            o = lane_offset(j['scene'], j.get('current_lane'), np.array(j['ego_xy']), j['ego_yaw'])
            if o is not None:rows.append((t*1e-9, j['global_s'], j['ego_speed']*3.6, o[0]))
    a = np.array(rows);k0 = int(np.searchsorted(a[:, 1], KICK_S));t0 = a[k0, 0]
    pre = a[(a[:, 0] < t0) & (a[:, 0] > t0-1.5), 3];after = a[k0:];after = after[after[:, 0]-t0 < 8.]
    off = after[:, 3]-(np.median(pre) if len(pre) else 0.)
    # Half-cycle peaks: the largest |offset| between sign changes (past +-BAND), first 4.
    peaks = [];sign = 0;cur = 0.
    for v in off:
        s = 1 if v > BAND else -1 if v < -BAND else 0
        if s and s != sign:
            if sign:peaks.append(round(cur, 2))
            sign = s;cur = 0.
        if s:cur = max(cur, abs(v))
    if sign:peaks.append(round(cur, 2))
    big = max(range(len(peaks)), key=lambda i: peaks[i]) if peaks else None
    # Decay: how much of the largest swing is left two half-cycles (one period) later.
    decay = round(peaks[big+2]/peaks[big], 2) if big is not None and big+2 < len(peaks) else None
    st = np.array(steer);ws = (st[:, 0] >= t0) & (st[:, 0] < t0+8.)
    return dict(v_kmh=round(float(a[k0, 2]), 1), peaks_m=peaks[:5], decay_per_period=decay,
                steer_peak_deg=round(float(abs(st[ws, 1]).max()), 2))


if __name__ == '__main__':
    for d in sys.argv[1:]:print(os.path.basename(d.rstrip('/')), response(d))
