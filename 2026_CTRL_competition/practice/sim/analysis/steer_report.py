#!/usr/bin/env python3
"""PRACTICE ONLY: highway steering quality per run (2026-10-01 lookahead sweep).

    python3 practice/sim/analysis/steer_report.py <run_dir> [...]     (bag found as the first one after the run dir)

From /control/state (error = cross-track to the active reference) and /control/telemetry (steering) in the highway
section (CP10 s1118.7 - CP13 s1741.7): max |error|, RMS error and steering-rate RMS above 90 km/h, max |error|
on the CP12-CP13 rejoin bends (s1594+), highway time.
"""
import sys, json, re, glob, os, math
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

STATE = re.compile(r'v=(-?[\d.]+)km/h s=([\d.]+)m error=([\d.]+)m')


def bag_for(run_dir):
    stamp = os.path.basename(run_dir.rstrip('/'))
    bags = sorted(b for b in glob.glob(os.path.expanduser('~/foscar_logs/bags/run_*')) if not b.endswith('_points'))
    return next(b for b in bags if os.path.basename(b)[4:] > stamp)


def metrics(run_dir):
    bag = bag_for(run_dir)
    r = SequentialReader();r.open(StorageOptions(uri=bag, storage_id=''), ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    st = [];steer = []
    while r.has_next():
        tp, d, t = r.read_next()
        if tp == '/control/state':
            m = STATE.search(deserialize_message(d, get_message(types[tp])).data)
            if m:st.append((t*1e-9, float(m[1]), float(m[2]), float(m[3])))
        elif tp == '/control/telemetry':
            try:steer.append((t*1e-9, json.loads(deserialize_message(d, get_message(types[tp])).data)['steering_rad']))
            except Exception:pass
    st = np.array(st);sr = np.array(steer)
    rate = np.degrees(np.gradient(sr[:, 1], sr[:, 0]))
    hw = st[(st[:, 2] >= 1118.7) & (st[:, 2] <= 1741.7)]
    fast = hw[hw[:, 1] >= 90]
    idx = np.searchsorted(sr[:, 0], fast[:, 0]).clip(0, len(sr)-1) if len(fast) else []
    bend = hw[hw[:, 2] >= 1594]
    res = json.loads(open(os.path.join(run_dir, 'results.jsonl')).readline());sp = res['splits']
    return dict(result=res['result'], hw_s=round(sp['1741.7']-sp['1118.7'], 1) if '1741.7' in sp else None,
                vmax=round(float(hw[:, 1].max()), 1), err_max=round(float(hw[:, 3].max()), 2),
                err_rms_fast=round(float(np.sqrt(np.mean(fast[:, 3]**2))), 3) if len(fast) else None,
                steer_rate_rms_fast=round(float(np.sqrt(np.mean(rate[idx]**2))), 2) if len(fast) else None,
                bend_err_max=round(float(bend[:, 3].max()), 2) if len(bend) else None, n_fast=len(fast))


if __name__ == '__main__':
    for d in sys.argv[1:]:print(os.path.basename(d.rstrip('/')), metrics(d))
