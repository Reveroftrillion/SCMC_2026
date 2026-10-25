#!/usr/bin/env python3
"""Practice only: behavior_node stalls -- gaps between /behavior/target messages, with the station they happened at.

    python3 practice/sim/analysis/tick_gaps.py ~/foscar_logs/bags/run_<stamp> [--min 0.15] [--from 1100 --to 1650]

behavior_node publishes at 20 Hz. A gap of 0.5 s or more means one tick ran that long: the controller drives on the
old target and the next highway tick finds a stale scan (SCAN_UNKNOWN) and drops the change it had just started
(2026-10-06: an on-demand curve fit made wait_gap 0.5 s per tick, 22 stalls up to 1.4 s, every 4->3 aborted).
Check this after any change that adds work to a tick.
"""
import argparse

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--min', type=float, default=.15, help='report gaps longer than this [s]')
    ap.add_argument('--from', dest='lo', type=float, default=-1e9)
    ap.add_argument('--to', dest='hi', type=float, default=1e9)
    a = ap.parse_args()
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/behavior/target', '/localization/s']))
    s, prev, gaps = 0., None, []
    while r.has_next():
        topic, data, stamp = r.read_next()
        t = stamp / 1e9
        if topic == '/localization/s':
            s = deserialize_message(data, get_message(types[topic])).data
            continue
        if prev is not None and t - prev > a.min and a.lo <= s <= a.hi:
            gaps.append((s, t - prev))
        prev = t
    for s, g in gaps:
        print('s%.1f  gap %.2f s' % (s, g))
    print('%d gaps over %.2f s, longest %.2f s' % (len(gaps), a.min, max((g for _, g in gaps), default=0.)))


if __name__ == '__main__':
    main()
