#!/usr/bin/env python3
"""Practice only: lidar intensity of floor vs barricade vs everything else in the tunnel (user 2026-09-29:
braking pitch makes the floor look like an obstacle; can reflectivity tell them apart?).

    python3 practice/sim/lidar_intensity.py <run_..._points bag> --scenario practice/scenarios/X.json [--s-from 1860 --s-to 1970]

Points are put in the map with the odometry pose (lidar 2.0 m ahead of the rear axle, 1.75 m up; roll/pitch
ignored, which is exactly what braking breaks). Classes: 'barricade' within a true 1.2 m footprint +0.3 m,
'floor' at relative height < 0.25 m elsewhere on the pavement, 'raised' at >= 0.25 m elsewhere (walls,
clutter and floor lifted by pitch)."""
import argparse, json, math, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from swerve_analysis import route, to_frenet, barricades


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag'); ap.add_argument('--scenario', required=True)
    ap.add_argument('--s-from', type=float, default=1860.); ap.add_argument('--s-to', type=float, default=1975.)
    a = ap.parse_args()
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2 as pc2
    P, S = route(); obs = barricades(a.scenario, P, S)
    r = SequentialReader(); r.open(StorageOptions(uri=a.bag, storage_id='sqlite3'), ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    pose = None; acc = {'barricade': [], 'floor': [], 'raised': []}; heights = {'barricade': [], 'floor': [], 'raised': []}
    while r.has_next():
        topic, data, t = r.read_next()
        m = deserialize_message(data, get_message(types[topic]))
        if topic == '/localization/odometry':
            q = m.pose.pose.orientation
            pose = (m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            continue
        if pose is None:
            continue
        s_ego = to_frenet(P, S, pose[0], pose[1])[0]
        if not a.s_from <= s_ego <= a.s_to:
            continue
        pts = pc2.read_points_numpy(m, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)
        pts = pts[(pts[:, 0] > 0) & (pts[:, 0] < 35) & (abs(pts[:, 1]) < 8)]
        c, sn = math.cos(pose[2]), math.sin(pose[2])
        lx, ly = pts[:, 0]+2.0, pts[:, 1]
        wx, wy = pose[0]+c*lx-sn*ly, pose[1]+sn*lx+c*ly
        z = pts[:, 2]+1.75          # height above the (assumed level) road
        inb = np.zeros(len(pts), bool)
        for o in obs:
            inb |= (abs(wx-o['x']) < o['half']+.3) & (abs(wy-o['y']) < o['half']+.3)
        for name, sel in (('barricade', inb & (z > .1)), ('floor', ~inb & (z < .25)), ('raised', ~inb & (z >= .25) & (abs(pts[:, 1]) < 6))):
            acc[name].append(pts[sel, 3]); heights[name].append(z[sel])
    for name in acc:
        v = np.concatenate(acc[name]) if acc[name] else np.array([])
        h = np.concatenate(heights[name]) if heights[name] else np.array([])
        if not len(v):
            print(name, 'no points'); continue
        qs = np.percentile(v, [1, 5, 25, 50, 75, 95, 99])
        print(f"{name:9s} n={len(v):7d} zero={np.mean(v == 0)*100:5.1f}%  intensity p1/5/25/50/75/95/99 = {np.round(qs, 1).tolist()}"
              f"  height p5/50/95 = {np.round(np.percentile(h, [5, 50, 95]), 2).tolist()}")


if __name__ == '__main__':
    main()
