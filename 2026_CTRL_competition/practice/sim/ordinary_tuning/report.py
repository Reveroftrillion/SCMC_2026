#!/usr/bin/env python3
"""Practice-only ordinary-section timing and safety report for a recorded run."""
import argparse
import json
import re
from pathlib import Path

import numpy as np
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CP10, CP13, END = 1108.7, 1731.6, 2155.3
SECTIONS = [("start", 1., 167.), ("signal1", 167., 210.), ("straight1", 210., 270.),
            ("static", 270., 388.), ("pedestrian", 403., 493.), ("straight2", 493., 565.),
            ("signal4", 565., 603.), ("middle", 603., 801.), ("roundabout", 801., 920.),
            ("pre_highway", 920., CP10), ("highway", CP10, CP13),
            ("post_highway", CP13, 1808.), ("tunnel", 1808., 1991.), ("finish", 1991., END)]


def report(run):
    run = Path(run)
    log = (run / 'launch_0.log').read_text()
    match = re.search(r'(run_\d{8}_\d{6})_0\.db3', log)
    if not match:
        raise ValueError('Bag path missing from launch log')
    bag = Path.home() / 'foscar_logs/bags' / match[1]
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag), storage_id=''), ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(StorageFilter(topics=['/localization/s', '/control/telemetry', '/control/state',
                                           '/behavior/target', '/behavior/target_path', '/vehicle/status']))
    state_re = re.compile(r'v=(-?[\d.]+)km/h s=([\d.]+)m error=([\d.]+)m')
    s = 0.
    behavior = ''
    samples, status, tracking = [], [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = stamp * 1e-9
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == '/localization/s':
            s = float(msg.data)
        elif topic == '/behavior/target':
            behavior = msg.state
        elif topic == '/control/telemetry':
            d = json.loads(msg.data)
            samples.append({**d, 't': t, 's': s, 'state': behavior})
        elif topic == '/vehicle/status':
            status.append((t, s, msg.speed_mps * 3.6))
        elif topic == '/control/state':
            m = state_re.search(msg.data)
            if m and behavior == 'CRUISE':
                tracking.append((float(m[2]), float(m[3])))
    def crossing(station):
        for a, b in zip(samples, samples[1:]):
            if a['s'] < station <= b['s']:
                return a['t'] + (b['t']-a['t']) * (station-a['s']) / (b['s']-a['s'])
        return None
    def duration(lo, hi):
        a, b = crossing(lo), crossing(hi)
        return round(b-a, 3) if a is not None and b is not None else None
    stats = []
    for name, lo, hi in SECTIONS:
        a, b = crossing(lo), crossing(hi)
        rows = [x for x in samples if lo <= x['s'] < hi]
        if not rows or a is None or b is None:
            continue
        stopped = sum(min(.1, y['t']-x['t']) for x, y in zip(rows, rows[1:]) if abs(x['speed_mps']) < .05)
        stats.append(dict(section=name, elapsed=round(b-a, 3), stopped=round(stopped, 3),
                          moving=round(b-a-stopped, 3), min_kmh=round(min(x['speed_mps']*3.6 for x in rows), 2)))
    ordinary = [x for x in status if 1 <= x[1] < CP10 or CP13 <= x[1] <= END]
    overs = [x for x in ordinary if x[2] > 60.]
    cruise_errors = [e for st, e in tracking if 1 <= st < CP10 or CP13 <= st <= END]
    tracking_by_section = {}
    for name, lo, hi in SECTIONS:
        es = [e for st, e in tracking if lo <= st < hi]
        if es:
            tracking_by_section[name] = dict(max_m=round(max(es), 3), p95_m=round(float(np.percentile(es, 95)), 3))
    res = json.loads((run / 'results.jsonl').read_text().splitlines()[0])
    return dict(run=run.name, bag=str(bag), result=res['result'], lap=res['drive_s'],
                before_highway_s=duration(1., CP10),
                after_highway_s=duration(CP13, END),
                max_ordinary_kmh=round(max(x[2] for x in ordinary), 3),
                over60_samples=len(overs), over60_first=overs[:3],
                cruise_error_max_m=max(cruise_errors, default=None), cruise_tracking=tracking_by_section,
                collisions=res['collisions'], body_contacts=res['body_contacts'],
                full_stops=res['full_stops'], sections=stats,
                localization_resets=log.count('localization reset'))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--out')
    args = ap.parse_args()
    reports = [report(run) for run in args.runs]
    text = json.dumps(reports, indent=2)
    print(text)
    if args.out:
        Path(args.out).write_text(text + '\n')
