#!/usr/bin/env python3
"""Practice only: every highway state change of a run, with speed, lead, plan and what blocked it.

    python3 practice/sim/analysis/highway_trace.py ~/foscar_logs/bags/run_<stamp> [--from 1100 --to 1640]

One line per (state, reason, current_lane, target_lane) change on /behavior/highway. Read it next to
practice/sim/pass_records.jsonl (the planner's box view at each decision) and the run's objects_0.jsonl (truth cars)
to say whether a tuck-in was right (see CLAUDE.md, highway).
A CHANGE_STARTED followed at once by SCAN_HOLD / SCAN_UNKNOWN -> FOLLOW is a dropped change: check tick_gaps.py.
"""
import argparse
import json

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--from', dest='lo', type=float, default=1100.)
    ap.add_argument('--to', dest='hi', type=float, default=1640.)
    a = ap.parse_args()
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/behavior/highway', '/localization/s', '/localization/odometry']))
    s, kmh, last = 0., 0., None
    while r.has_next():
        topic, data, _ = r.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == '/localization/s':
            s = msg.data
            continue
        if topic == '/localization/odometry':
            kmh = msg.twist.twist.linear.x * 3.6
            continue
        if not a.lo <= s <= a.hi:
            continue
        d = json.loads(msg.data)
        key = (d.get('state'), d.get('reason'), d.get('current_lane'), d.get('target_lane'))
        if key == last:
            continue
        last = key
        p = d.get('plan') or {}
        print('s%.1f %3.0f km/h %-8s %-16s lane %s->%s | lead %s | plan %s %s front=%s rear=%s | blocked %s' % (
            s, kmh, key[0], key[1], key[2], key[3], json.dumps(d.get('lead'))[:50], p.get('kind'), p.get('mode'),
            p.get('front'), p.get('rear'), d.get('blocked_by')))


if __name__ == '__main__':
    main()
