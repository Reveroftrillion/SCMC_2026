#!/usr/bin/env python3
"""Analyze a CLOSED native audit bag near a generated tunnel scene actor.

Location matching reports candidate observed outlines, not simulator actor ID
identity or hidden-object dimensions. Does not publish, run nodes, or alter bags.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

sys.dont_write_bytecode = True
import numpy as np
import rosbag
import rospy
from ctrl_control.ctrl_core import Route
from ctrl_planning.ctrl_frenet import to_frenet

PORT = Path(__file__).resolve().parents[1]


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [clean(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--scene-manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--termination-record', type=Path,
                        help='Explicit user-confirmed termination record for a ctrl_start run')
    args = parser.parse_args()
    termination_scope = 'external audit end.json'
    if args.termination_record:
        termination = json.loads(args.termination_record.read_text())
        if (termination.get('scope') != 'user-confirmed stopped'
                or Path(termination.get('run_dir', '')).resolve() != args.run.resolve()
                or termination.get('manual_park_confirmed') is not True):
            parser.error('Explicit matching user-confirmed stop/Manual-P record required')
        termination_scope = 'user-confirmed stop/Manual-P; no external collision audit'
    elif not (args.run / 'end.json').is_file():
        parser.error('An ended audit run or explicit user termination record is required')
    if list(args.run.rglob('*.active')):
        parser.error('Active bags must be closed before analysis')
    bags = sorted(list(args.run.glob('*.bag')) + list(args.run.glob('bags/*.bag')))
    if not bags:
        parser.error('No closed audit bags')
    manifest = json.loads(args.scene_manifest.read_text())
    station = float(manifest['station'])
    position = manifest['added_actor']['pos']
    actor_xy = np.array([position['x'], position['y']])
    route_path = PORT / 'src/ctrl_bringup/routes/global_path.txt'
    if hashlib.sha256(route_path.read_bytes()).hexdigest() != manifest['route_sha256']:
        parser.error('Route differs from the scene generation geometry')
    route = Route(np.loadtxt(route_path))
    args.output.mkdir(parents=True, exist_ok=False)
    s = None
    mission = None
    outlines, targets, poses, modes = [], [], [], []
    camera_at = None
    topics = ['/localization/s', '/localization/odometry', '/localization/mode',
              '/behavior/mission', '/behavior/target', '/perception/footprints']
    for path in bags:
        with rosbag.Bag(str(path)) as bag:
            for topic, msg, when in bag.read_messages(topics=topics):
                now = when.to_sec()
                if topic == '/localization/s':
                    s = msg.data
                elif topic == '/behavior/mission':
                    mission = json.loads(msg.data)['mission']
                if s is None or not station - 100. <= s <= station + 100.:
                    continue
                if camera_at is None and s >= station - 20.:
                    camera_at = now
                if topic == '/localization/odometry':
                    p = msg.pose.pose.position
                    projected_s, d = to_frenet(route, p.x, p.y, s)
                    poses.append(dict(t=now, s=s, projected_s=projected_s, d=d,
                                      speed_mps=msg.twist.twist.linear.x, x=p.x, y=p.y))
                elif topic == '/localization/mode':
                    if not modes or modes[-1]['mode'] != msg.data:
                        modes.append(dict(t=now, s=s, mode=msg.data))
                elif topic == '/behavior/target':
                    targets.append(dict(t=now, ego_s=s, mission=mission, state=msg.state,
                        v_max=msg.v_max, stop_s=msg.stop_s, offset=msg.lat_offset,
                        chain=list(zip(msg.chain_offset, msg.chain_start_s, msg.chain_length_m))))
                elif topic == '/perception/footprints':
                    data = json.loads(msg.data)
                    for obj in data.get('objects', []):
                        points = np.asarray(obj.get('xy', []), dtype=float)
                        if points.ndim != 2 or points.shape[1] != 2 or not len(points):
                            continue
                        distance = float(np.linalg.norm(points.mean(axis=0) - actor_xy))
                        if distance > 4. or (obj.get('height') or 0.) < .2:
                            continue
                        sd = np.array([to_frenet(route, x, y, station) for x, y in points])
                        outlines.append(dict(t=now, stamp=data['stamp'], ego_s=s, id=obj['id'],
                            kind=obj.get('kind'), height=obj.get('height'), xy=points.tolist(),
                            center_distance_to_actor_m=distance, s_min=float(sd[:, 0].min()),
                            s_max=float(sd[:, 0].max()), d_min=float(sd[:, 1].min()),
                            d_max=float(sd[:, 1].max())))
    # The camera is evidence only; copy its original compressed bytes.
    camera = None
    if camera_at is not None:
        choices = []
        for path in bags:
            with rosbag.Bag(str(path)) as bag:
                for _, msg, when in bag.read_messages(topics=['/sensors/camera/traffic/compressed'],
                        start_time=rospy.Time.from_sec(camera_at - .3),
                        end_time=rospy.Time.from_sec(camera_at + .3)):
                    choices.append((abs(when.to_sec() - camera_at), when.to_sec(), bytes(msg.data)))
        if choices:
            _, stamp, raw = min(choices, key=lambda value: value[0])
            with (args.output / 'approach-camera.jpg').open('xb') as stream:
                stream.write(raw)
            camera = dict(t=stamp, near_ego_s=station - 20., sha256=hashlib.sha256(raw).hexdigest())
    summary = dict(run=str(args.run), termination_scope=termination_scope,
        scene_sha256=manifest['generated_sha256'],
        expected_actor=manifest['added_actor'], expected_station=station,
        matched_observations=len(outlines), matched_ids=sorted({row['id'] for row in outlines}),
        first_match=None if not outlines else {k: v for k, v in outlines[0].items() if k != 'xy'},
        last_match=None if not outlines else {k: v for k, v in outlines[-1].items() if k != 'xy'},
        target_states=dict(Counter(row['state'] for row in targets)),
        avoidance_targets=sum('AVOID' in row['state'] for row in targets),
        minimum_target_offset=min((row['offset'] for row in targets), default=None),
        maximum_target_offset=max((row['offset'] for row in targets), default=None),
        minimum_measured_d=min((row['d'] for row in poses), default=None),
        maximum_measured_d=max((row['d'] for row in poses), default=None),
        localization_modes=modes, camera=camera,
        limitations=['Within 4m of intended location, observed height >=0.2m; candidate association only',
            'Footprints omit hidden body edges; not a ground-truth size or clearance measurement',
            'Prescribed scene pose and preceding recorded s; no proof of official scoring or every actor identity'])
    for name, value in [('summary.json', summary), ('outlines.json', outlines),
                        ('targets.json', targets), ('poses.json', poses)]:
        with (args.output / name).open('x') as stream:
            json.dump(clean(value), stream, indent=2, allow_nan=False)
            stream.write('\n')
    print(json.dumps(clean(summary), indent=2))


if __name__ == '__main__':
    main()
