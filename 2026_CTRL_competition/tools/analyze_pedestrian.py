"""Offline pedestrian replay and ray evidence. Never opens simulator sockets.
#!/usr/bin/env python3

Usage: source install/setup.bash; python3 tools/analyze_pedestrian.py BEHAVIOR_BAG RAW_BAG OUTPUT
Ray counts are diagnostics at one observed footprint, not permission to drive.
Assumes the measured lidar mounting (2, 0, 2.11), level roll/pitch, planar ego pose.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

from foscar_control.core import quaternion_yaw
from foscar_lidar.free_space import ray_box_evidence
from foscar_lidar.obstacle_node import ObstacleNode


def messages(directory, names):
    for db in sorted(Path(directory).glob('*.db3')):
        with sqlite3.connect(str(db)) as conn:
            topics = {i: (n, get_message(t)) for i, n, t in conn.execute('select id,name,type from topics') if n in names}
            for ts, tid, data in conn.execute('select timestamp,topic_id,data from messages order by timestamp'):
                if tid in topics:
                    n, cls = topics[tid]
                    yield ts / 1e9, n, deserialize_message(data, cls)


def pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return np.array([p.x, p.y]), quaternion_yaw(q.w, q.x, q.y, q.z)


def rot(yaw):
    return np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])


def write_csv(path, rows):
    if rows:
        with path.open('w') as out:
            writer = csv.DictWriter(out, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('behavior_bag'); parser.add_argument('raw_bag'); parser.add_argument('output')
    args = parser.parse_args()
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    records = list(messages(args.behavior_bag, {'/events', '/localization/odometry', '/localization/s', '/map/route'}))
    odom = [(t, m) for t, n, m in records if n == '/localization/odometry']
    candidates = [(t, m) for t, n, m in records if n == '/events' and m.type == 'PEDESTRIAN']
    if not odom or not candidates:
        raise ValueError('requires odometry and pedestrian observations')
    # Select an observed footprint near the ego centerline, not an assumed person location.
    anchor_t, anchor = min(candidates, key=lambda item: abs(item[1].y))
    _, anchor_odom = min(odom, key=lambda item: abs(item[0] - anchor_t))
    anchor_pos, anchor_yaw = pose(anchor_odom)
    anchor_rot = rot(anchor_yaw)
    half = np.array([max(.3, anchor.size_x / 2), max(.3, anchor.size_y / 2)])
    lower = np.r_[np.array([anchor.x, anchor.y]) - half, .3]
    upper = np.r_[np.array([anchor.x, anchor.y]) + half, 1.8]
    t0 = min(t for t, _, _ in records)
    rays, events = [], []
    rclpy.init()
    node = ObstacleNode(parameter_overrides=[Parameter('use_tf', value=False), Parameter('publish_markers', value=False), Parameter('stats_period_s', value=0.)])
    replay_t = t0

    class Capture:
        def publish(self, e):
            events.append(dict(t=replay_t-t0, type=e.type, identity=e.value,
                               s=e.s, x=e.x, y=e.y, vx=e.vx, vy=e.vy,
                               size_x=e.size_x, size_y=e.size_y))

    node.event_pub = Capture()
    try:
        stream = [(t, n, m) for t, n, m in records if n != '/events']
        stream += list(messages(args.raw_bag, {'/sensors/lidar/points'}))
        for replay_t, name, m in sorted(stream, key=lambda row: row[0]):
            if name == '/map/route': node.on_route(m)
            elif name == '/localization/odometry': node.on_odom(m)
            elif name == '/localization/s': node.ego_s = m.data
            elif name == '/sensors/lidar/points':
                node.on_cloud(m)
                cloud_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                nearest_t, o = min(odom, key=lambda item: abs(item[0] - cloud_t))
                if abs(nearest_t-cloud_t) > .15: continue
                pos, yaw = pose(o)
                pts = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3).astype(float)
                pts += [2., 0., 2.11]
                pts[:, :2] = (pts[:, :2] @ rot(yaw).T + pos - anchor_pos) @ anchor_rot
                origin = np.r_[(np.array([2., 0.]) @ rot(yaw).T + pos - anchor_pos) @ anchor_rot, 2.11]
                evidence = ray_box_evidence(origin, pts, lower, upper)
                rays.append(dict(t=replay_t-t0, **evidence))
    finally:
        node.destroy_node(); rclpy.shutdown()
    write_csv(out/'ray_evidence.csv', rays)
    write_csv(out/'replayed_events.csv', events)
    summary = dict(anchor_time=anchor_t-t0, anchor_lower=lower.tolist(), anchor_upper=upper.tolist(),
                   raw_frames=len(rays), replay_events=len(events), types=sorted({e['type'] for e in events}),
                   limitation='One observed footprint; ray evidence alone cannot certify full corridor clearance.')
    (out/'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
