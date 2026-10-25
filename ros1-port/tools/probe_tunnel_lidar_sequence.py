#!/usr/bin/env python3
"""Extract CLOSED bag inputs, then replay LiDAR calculations with ROS doubles.

No ROS master, publication or vehicle control. Extraction and replay run in
separate processes so native ROS classes never mix with test doubles.
"""
import argparse
import hashlib
import importlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import subprocess

sys.dont_write_bytecode = True
PORT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def extract(args):
    import numpy as np
    import rosbag
    import rospy
    from sensor_msgs import point_cloud2
    if args.bag.suffix != '.bag' or not (args.bag.parent / 'end.json').exists():
        raise ValueError('A closed audit bag and end.json are required')
    if not all(map(math.isfinite, [args.start, args.end])) or not args.start < args.end:
        raise ValueError('Finite increasing record-time interval required')
    args.output.mkdir(parents=True, exist_ok=False)
    frames, odometry = [], []
    station, mission = None, None
    with rosbag.Bag(str(args.bag)) as bag:
        for topic, msg, when in bag.read_messages(
                topics=['/localization/s', '/localization/odometry', '/behavior/mission',
                        '/sensors/lidar/points'],
                start_time=rospy.Time.from_sec(args.start - 3.),
                end_time=rospy.Time.from_sec(args.end)):
            if topic == '/localization/s':
                station = float(msg.data)
            elif topic == '/behavior/mission':
                mission = json.loads(msg.data)['mission']
            elif topic == '/localization/odometry':
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                odometry.append(dict(record_time=when.to_sec(), stamp=msg.header.stamp.to_sec(),
                    x=p.x, y=p.y, yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                    speed=msg.twist.twist.linear.x))
            elif when.to_sec() >= args.start:
                if msg.header.frame_id != 'lidar' or station is None or mission is None:
                    raise ValueError('Missing lidar-frame/preceding station/mission context')
                points = np.array(list(point_cloud2.read_points(msg,
                    field_names=['x', 'y', 'z', 'intensity'], skip_nans=True)), dtype=np.float32)
                name = 'cloud-%04d.npz' % len(frames)
                np.savez_compressed(args.output / name, points=points, stamp=msg.header.stamp.to_sec())
                frames.append(dict(file=name, sha256=digest(args.output/name),
                    record_time=when.to_sec(), stamp=msg.header.stamp.to_sec(), s=station, mission=mission))
    if not frames:
        raise ValueError('No raw frames in interval')
    save(args.output/'inputs.json', dict(bag=str(args.bag), start=args.start, end=args.end,
        frames=frames, odometry=odometry,
        limitations=['Bag record order, preceding mission/station; not native callback scheduling',
                     'No input older than the requested interval minus 3s is replayed']))
    print(json.dumps(dict(output=str(args.output), frames=len(frames), odometry=len(odometry))))


def replay(args):
    import numpy as np
    sys.path.insert(0, str(PORT/'verification'))
    import ros_stubs
    ros_stubs.install(PORT)
    for package in (PORT/'src').iterdir():
        if package.is_dir():
            sys.path.insert(0, str(package))
    source = PORT/'src/ctrl_lidar/ctrl_lidar/ctrl_obstacle_node.py'
    working_digest = digest(source)
    source_ref = 'working tree'
    model_digest = working_digest
    if args.baseline_head:
        source_ref = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=PORT.parent, text=True).strip()
        raw = subprocess.check_output(['git', 'show', source_ref+':'+str(source.relative_to(PORT.parent))], cwd=PORT.parent)
        model_digest = hashlib.sha256(raw).hexdigest()
        package = importlib.import_module('ctrl_lidar')
        name = 'ctrl_lidar.ctrl_obstacle_node'
        module = importlib.util.module_from_spec(importlib.util.spec_from_loader(name, loader=None))
        module.__file__ = '<'+source_ref+':ctrl_obstacle_node.py>'
        sys.modules[name] = module
        setattr(package, 'obstacle_node', module)
        exec(compile(raw, module.__file__, 'exec'), module.__dict__)
    from ctrl_lidar.ctrl_obstacle_node import ObstacleNode
    from ctrl_lidar.ctrl_ground import region
    from ctrl_ros1.ctrl_parameter import Parameter
    from ctrl_ros1 import ctrl_point_cloud2 as point_cloud2
    from std_msgs.msg import Header, String
    from sensor_msgs.msg import PointField
    from replay_lidar_ground import trace_calculation
    inputs = json.loads((args.input/'inputs.json').read_text())
    node = ObstacleNode(parameter_overrides=[Parameter('use_tf', value=False),
        Parameter('publish_markers', value=False),
        Parameter('link_set', value=str(PORT/'src/ctrl_bringup/maps/link_set.json'))])
    route = np.loadtxt(PORT/'src/ctrl_bringup/routes/global_path.txt')[:, :2]
    node.route_xy = route
    node.route_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(['x', 'y', 'z', 'intensity'])]
    rows, odom_index = [], 0
    try:
        for frame in inputs['frames']:
            path = args.input/frame['file']
            if digest(path) != frame['sha256']:
                raise ValueError('Raw frame changed: '+str(path))
            while (odom_index < len(inputs['odometry']) and
                   inputs['odometry'][odom_index]['record_time'] <= frame['record_time']):
                odom = inputs['odometry'][odom_index]
                node.odom_buf.append((odom['stamp'], odom['x'], odom['y'], odom['yaw']))
                node.speed_buf.append((odom['stamp'], odom['speed']))
                odom_index += 1
            with np.load(path, allow_pickle=False) as cloud:
                raw, stamp = cloud['points'].copy(), float(cloud['stamp'])
            node.ego = node.ego_pose_at(stamp)
            if node.ego is None:
                raise ValueError('No fresh pose for raw frame')
            node.ego_s = frame['s']
            node.on_mission(String(data=json.dumps(dict(mission=frame['mission']))))
            msg = point_cloud2.create_cloud(Header(frame_id='lidar'), fields, raw)
            dets, locals_ = trace_calculation(node.detect, msg, stamp)
            # Inspect the actual unfiltered clearance cloud and exact paint mask.
            base = node.raw_clearance_points
            mapped = node.to_map(base, stamp)
            near = np.linalg.norm(mapped-np.array(args.actor_xy), axis=1) < 2.
            elevated = near & (base[:, 2] > node.ground_z) & (base[:, 2] < 2.)
            paint = locals_.get('paint', np.zeros(len(raw), dtype=bool))
            plane = locals_.get('plane')
            corrected = np.zeros(len(raw), dtype=bool) if plane is None else (
                locals_['height_region'] if locals_.get('height_region') is not None else region(base))
            observed = []
            for det in dets:
                xy = node.to_map(np.array([[det['x'], det['y']]]), stamp)[0]
                if np.linalg.norm(xy-np.array(args.actor_xy)) < 3.:
                    observed.append(dict(xy=xy.tolist(), top=det['z_max'], n=det['n']))
            node.track_and_publish(dets, msg.header.stamp, stamp)
            all_published = json.loads(node.footprint_pub.values[-1].data)['objects']
            published = [p for p in all_published
                         if np.linalg.norm(np.array(p['center'])-np.array(args.actor_xy)) < 3.]
            tracks = [dict(id=tr.id, hits=tr.hits, misses=tr.misses, kind=tr.kind,
                structure=bool(tr.structure), history=list(tr.history),
                hold=getattr(tr, 'hold_fp', None) is not None,
                held_near=bool(node.tunnel_held_near(tr)),
                on_road=bool(node.observation_on_road(*tr.history[-1][1:])))
                for tr in node.tracks if tr.history and
                np.linalg.norm(np.array(tr.history[-1][1:])-np.array(args.actor_xy)) < 3.]
            rows.append(dict(s=frame['s'], stamp=stamp, record_time=frame['record_time'],
                plane=None if plane is None else plane.tolist(), pitch_deg=node.body_pitch_deg(stamp),
                elevated_near_actor=int(elevated.sum()),
                elevated_paint_removed=int((elevated & paint).sum()),
                corrected_elevated_near_actor=int((elevated & corrected).sum()),
                observed=observed, published=published, all_published=all_published, tracks=tracks))
    finally:
        node.destroy_node()
    observed = [r for r in rows if r['observed']]
    published = [r for r in rows if r['published']]
    save(args.output, dict(scope='Offline calculation/tracking with ROS doubles; no controls',
        input_manifest_sha256=digest(args.input/'inputs.json'), source_sha256=model_digest, source_ref=source_ref,
        source_unchanged=digest(source)==working_digest, frames=len(rows), observed_frames=len(observed),
        published_frames=len(published), first_observed=None if not observed else observed[0],
        first_published=None if not published else published[0], rows=rows,
        limitations=inputs['limitations']+['Parameter fallback transform [2,0,2.11]; native TF not replayed',
            'Fresh recorded mission per scan; wall-clock transport freshness is not tested',
            'Tracks warmed only from this short window; no planner or vehicle dynamics replay']))
    print(json.dumps(dict(frames=len(rows), observed_frames=len(observed), published_frames=len(published),
        first_observed_s=None if not observed else observed[0]['s'],
        first_published_s=None if not published else published[0]['s'])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest='mode', required=True)
    extract_args = modes.add_parser('extract')
    extract_args.add_argument('--bag', type=Path, required=True)
    extract_args.add_argument('--start', type=float, required=True)
    extract_args.add_argument('--end', type=float, required=True)
    extract_args.add_argument('--output', type=Path, required=True)
    extract_args.set_defaults(run=extract)
    replay_args = modes.add_parser('replay')
    replay_args.add_argument('--input', type=Path, required=True)
    replay_args.add_argument('--output', type=Path, required=True)
    replay_args.add_argument('--actor-xy', type=float, nargs=2, required=True)
    replay_args.add_argument('--baseline-head', action='store_true', help='Load only committed obstacle_node in memory')
    replay_args.set_defaults(run=replay)
    args = parser.parse_args()
    args.run(args)


if __name__ == '__main__':
    main()
