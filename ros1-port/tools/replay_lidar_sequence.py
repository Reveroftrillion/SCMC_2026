#!/usr/bin/env python3
"""Offline, ordered LiDAR/odometry replay. Starts no ROS or control transport.

Extraction uses native rosbag; replay uses explicit ROS doubles and one planner
evaluation per cloud, not the live executor's 20 Hz scheduling. Input messages
and existing evidence are never overwritten.
"""
import argparse
import importlib
import importlib.util
import json
import linecache
import math
from pathlib import Path
import subprocess
import sys
import time

sys.dont_write_bytecode = True
from replay_lidar_ground import PORT, plain, sha256, trace_calculation, write_json


def extract(args):
    import numpy as np
    import rosbag
    import rospy
    from sensor_msgs import point_cloud2
    args.output.mkdir(parents=True, exist_ok=False)
    topics = ['/localization/odometry', '/localization/s', '/behavior/mission',
              '/sensors/lidar/points', '/behavior/target', '/perception/footprints',
              '/localization/mode', '/sensors/camera/traffic/compressed']
    count, camera_at, transforms, route = 0, -math.inf, {}, None
    with rosbag.Bag(str(args.bag), 'r') as bag:
        for topic, msg, _ in bag.read_messages(topics=['/map/route', '/tf_static'],
                                              end_time=rospy.Time.from_sec(args.start)):
            if topic == '/map/route':
                route = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
            else:
                for tf in msg.transforms:
                    t, q = tf.transform.translation, tf.transform.rotation
                    transforms[tf.child_frame_id] = dict(parent=tf.header.frame_id,
                        translation=[t.x, t.y, t.z], rotation=[q.x, q.y, q.z, q.w])
        if route is None:
            raise ValueError('No preceding recorded route')
        write_json(args.output / 'metadata.json', dict(bag=str(args.bag), start=args.start,
                   end=args.end, route=route, transforms=transforms))
        with (args.output / 'events.jsonl').open('x') as stream:
            for topic, msg, when in bag.read_messages(topics=topics,
                    start_time=rospy.Time.from_sec(args.start), end_time=rospy.Time.from_sec(args.end)):
                row = dict(topic=topic, record_time=when.to_sec())
                if hasattr(msg, 'header'):
                    row['stamp'] = [msg.header.stamp.secs, msg.header.stamp.nsecs]
                if topic == '/sensors/lidar/points':
                    fields = ['x', 'y', 'z'] + (['intensity'] if any(f.name == 'intensity' for f in msg.fields) else [])
                    points = np.array(list(point_cloud2.read_points(msg, field_names=fields, skip_nans=True)), dtype=np.float32)
                    name = 'cloud-%05d.npz' % count
                    np.savez_compressed(args.output / name, points=points)
                    row.update(file=name, frame=msg.header.frame_id, fields=fields)
                    count += 1
                elif topic == '/localization/odometry':
                    p, q = msg.pose.pose.position, msg.pose.pose.orientation
                    row.update(position=[p.x, p.y, p.z], rotation=[q.x, q.y, q.z, q.w], speed=msg.twist.twist.linear.x)
                elif topic == '/sensors/camera/traffic/compressed':
                    if when.to_sec() - camera_at < 2.:
                        continue
                    camera_at = when.to_sec()
                    name = 'camera-%.6f.jpg' % camera_at
                    with (args.output / name).open('xb') as image:
                        image.write(msg.data)
                    row['file'] = name
                elif topic == '/behavior/target':
                    row['target'] = dict(state=msg.state, v_max=msg.v_max, stop_s=msg.stop_s)
                else:
                    row['data'] = msg.data
                stream.write(json.dumps(plain(row), allow_nan=False) + '\n')
    write_json(args.output / 'manifest.json', {p.name: sha256(p) for p in args.output.iterdir() if p.is_file()})
    print(json.dumps(dict(output=str(args.output), clouds=count)), flush=True)


def setup_runtime(baseline):
    sys.path.insert(0, str(PORT / 'verification'))
    import ros_stubs
    ros_stubs.install(PORT)
    for package in (PORT / 'src').iterdir():
        if package.is_dir():
            sys.path.insert(0, str(package))
    ref = 'working tree'
    sources = {f'src/ctrl_lidar/ctrl_lidar/{name}.py': sha256(
        PORT / f'src/ctrl_lidar/ctrl_lidar/{name}.py') for name in ('ground', 'obstacle_node')}
    if baseline:
        ref = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=PORT.parent, text=True).strip()
        package = importlib.import_module('ctrl_lidar')
        for name in ('ground', 'obstacle_node'):
            path = 'ros1-port/src/ctrl_lidar/ctrl_lidar/' + name + '.py'
            source = subprocess.check_output(['git', 'show', ref + ':' + path], cwd=PORT.parent)
            import hashlib
            sources[path[len('ros1-port/'):]] = hashlib.sha256(source).hexdigest()
            key = 'ctrl_lidar.' + name
            module = importlib.util.module_from_spec(importlib.util.spec_from_loader(key, loader=None))
            filename = '<' + ref + ':' + path + '>'
            module.__file__ = filename
            linecache.cache[filename] = (len(source), None, source.decode().splitlines(True), filename)
            sys.modules[key] = module
            setattr(package, name, module)
            exec(compile(source, filename, 'exec'), module.__dict__)
    return ros_stubs, ref, sources


def replay(args):
    import numpy as np
    import yaml
    ros_stubs, source_ref, sources = setup_runtime(args.baseline_head)
    from ctrl_lidar import ctrl_obstacle_node as detection
    from ctrl_ros1 import ctrl_point_cloud2 as pc2
    from sensor_msgs.msg import PointField
    from std_msgs.msg import Header, String
    from nav_msgs.msg import Odometry
    from ctrl_control.ctrl_core import Route
    from ctrl_control.ctrl_mission_config import load_mission_config
    from ctrl_planning.ctrl_mission import MissionSupervisor
    from ctrl_planning.ctrl_state import Behavior, Context
    from ctrl_planning.states.ctrl_cruise import Cruise
    from ctrl_planning.states.ctrl_avoid import Avoid
    from ctrl_planning.states.ctrl_tunnel import Tunnel
    from ctrl_planning.ctrl_lane_prediction import LaneMap
    from ctrl_planning.ctrl_tunnel_corridor import WallMemory
    from ctrl_planning.ctrl_frenet import to_frenet
    from ctrl_planning.states import ctrl_avoid as avoid_module

    args.output.mkdir(parents=True, exist_ok=False)
    meta = json.loads((args.input / 'metadata.json').read_text())
    manifest = json.loads((args.input / 'manifest.json').read_text())
    if any(sha256(args.input / name) != digest for name, digest in manifest.items()):
        raise ValueError('Extracted input manifest mismatch')
    tf = meta['transforms']['lidar']
    if tf['parent'] != 'base_link' or tf['rotation'] != [0., 0., 0., 1.]:
        raise ValueError('Replay currently requires the recorded unrotated base_link<-lidar transform')
    params = dict(use_tf=False, publish_markers=False,
                  link_set=str(PORT / 'src/ctrl_bringup/maps/link_set.json'),
                  **dict(zip(('lidar_x', 'lidar_y', 'lidar_z'), tf['translation'])),
                  **{'highway.config': str(PORT / 'src/ctrl_planning/config/highway.yaml'),
                     'highway.mission_config': str(PORT / 'src/ctrl_planning/config/missions.yaml')})
    node = detection.ObstacleNode(parameter_overrides=[ros_stubs.Parameter(k, value=v) for k, v in params.items()])
    node.track_namespace = 'offline'
    node.route_xy = np.array(meta['route'])
    node.route_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(node.route_xy, axis=0), axis=1))]
    route = Route(node.route_xy)
    road = LaneMap(json.loads((PORT / 'src/ctrl_bringup/maps/link_set.json').read_text()))
    cfg = yaml.safe_load((PORT / 'src/ctrl_planning/config/behavior.yaml').read_text())['behavior_node']['ros__parameters']
    avoid = Avoid(**{k[6:]: v for k, v in cfg.items() if k.startswith('avoid.')})
    tunnel = Tunnel(**{k[7:]: v for k, v in cfg.items() if k.startswith('tunnel.')})
    behavior = Behavior([Cruise(**{k[7:]: v for k, v in cfg.items() if k.startswith('cruise.')}), avoid, tunnel])
    mission = MissionSupervisor(load_mission_config(PORT / 'src/ctrl_planning/config/missions.yaml'))
    walls, corridor = WallMemory(), None
    clock, saved, road_failures = [0.], {}, []
    detection.perf_counter = lambda: clock[0]
    original_detect = node.detect

    def observe_detect(msg, stamp):
        dets, values = trace_calculation(original_detect, msg, stamp)
        saved.clear()
        saved.update(values)
        return dets

    node.detect = observe_detect
    original_paved = avoid_module.paved_moves

    def observe_paved(*values, **kw):
        if len(road_failures) >= 24:
            return original_paved(*values, **kw)
        result, local = trace_calculation(original_paved, *values, **kw)
        if not result and 'points' in local:
            ctx, points = local['ctx'], local['points']
            inside = ctx.road_map.contains_road(points)
            invalid = np.flatnonzero(~inside)
            if len(invalid):
                first = int(invalid[0])
                road_failures.append(dict(record_time=clock[0], d0=local['d0'], moves=local['moves'],
                    s_from=local['s_from'], s_to=local['s_to'], road_type=type(ctx.road_map).__name__,
                    invalid_count=len(invalid), first_point=points[first],
                    first_sd=to_frenet(route, *points[first], ctx.s),
                    first_body_station=float(local['ss'][first // 35]),
                    first_body_longitudinal=float(local['a'].ravel()[first % 35]),
                    first_body_lateral=float(local['w'].ravel()[first % 35])))
        return result

    avoid_module.paved_moves = observe_paved
    frames, mode, last_s, recorded_target = 0, 'GPS', None, None
    began = time.perf_counter()
    with (args.input / 'events.jsonl').open() as stream, (args.output / 'frames.jsonl').open('x') as output:
        for line in stream:
            row = json.loads(line)
            clock[0] = row['record_time']
            topic = row['topic']
            if topic == '/localization/odometry':
                msg = Odometry()
                msg.header.stamp = ros_stubs.Stamp(secs=row['stamp'][0], nsecs=row['stamp'][1])
                for axis, value in zip('xyz', row['position']):
                    setattr(msg.pose.pose.position, axis, value)
                for axis, value in zip('xyzw', row['rotation']):
                    setattr(msg.pose.pose.orientation, axis, value)
                msg.twist.twist.linear.x = row['speed']
                node.on_odom(msg)
            elif topic == '/localization/s':
                last_s = node.ego_s = row['data']
            elif topic == '/localization/mode':
                mode = row['data']
            elif topic == '/behavior/mission':
                node.on_mission(String(data=row['data']))
            elif topic == '/behavior/target':
                recorded_target = row['target']
            elif topic == '/sensors/lidar/points':
                with np.load(args.input / row['file'], allow_pickle=False) as data:
                    raw = data['points']
                fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                          for i, name in enumerate(row['fields'])]
                stamp = ros_stubs.Stamp(secs=row['stamp'][0], nsecs=row['stamp'][1])
                cloud = pc2.create_cloud(Header(stamp=stamp, frame_id=row['frame']), fields, raw)
                node.on_cloud(cloud)
                fp = json.loads(node.footprint_pub.values[-1].data)
                ego = node.ego_pose_at(stamp.to_sec())
                if ego is None or last_s is None:
                    continue
                if fp['tunnel_walls'] and walls.add(fp['tunnel_walls'], route, last_s, 0., to_frenet, road):
                    if corridor is None or walls.due(clock[0]):
                        corridor = walls.corridor(road, end_s=tunnel.s_to)
                ctx = Context(s=last_s, x=ego[0], y=ego[1], yaw=ego[2],
                    v=node.speed_buf[-1][1], now=clock[0], route=route, loc_mode=mode, road_map=road)
                mission.update(ctx)
                mission.apply_context(ctx)
                if ctx.mission_name == 'TUNNEL' and corridor is not None:
                    ctx.road_map = corridor
                ctx.occupancy_stamp = fp['stamp']
                ctx.occupancy_hulls = {o['id']: o['xy'] for o in fp['objects']}
                ctx.occupancy_heights = {o['id']: o['height'] for o in fp['objects']}
                ctx.occupancy_kinds = {o['id']: o['kind'] for o in fp['objects']}
                ctx.occupancy_speeds = {o['id']: (o['vx'], o['vy']) for o in fp['objects']}
                ctx.occupancy_cleared = fp['cleared']
                ctx.occupancy_sensor = ego[:2]
                if args.behavior:
                    target = behavior.tick(ctx, allowed_states=mission.policy.allowed_states)
                else:
                    entered = avoid.check_entry(ctx)
                    target = avoid.compute_target(ctx) if entered else None
                if avoid.bev is not None:
                    node.on_clearance_queries(String(data=json.dumps(avoid.bev.clearance_queries(ctx.now))))
                ex, ey, yaw = ego
                co, si = math.cos(yaw), math.sin(yaw)
                base = node.lidar_to_base(raw[:, :3], row['frame'], node.get_clock().now())
                world = np.c_[ex+co*base[:, 0]-si*base[:, 1], ey+si*base[:, 0]+co*base[:, 1]]
                near = np.linalg.norm(world - [args.focus_x, args.focus_y], axis=1) < 1.5
                focus = np.c_[world[near], base[near, 2], raw[near, 3]] if raw.shape[1] == 4 else []
                result = dict(record_time=clock[0], stamp=stamp.to_sec(), s=last_s, ego=ego,
                    mission=ctx.mission_name, pitch_deg=saved.get('pitch'), plane=saved.get('plane'),
                    detections=saved.get('dets'), footprints=fp, recorded_target=recorded_target,
                    target=None if target is None else dict(state=target.state, v_max=target.v_max,
                                                           stop_s=target.stop_s, chain=target.chain),
                    boxes=[] if avoid.bev is None else [vars(b) for b in avoid.bev.boxes],
                    reason=None if avoid.bev is None else avoid.bev.last_reason,
                    focus_points_map_xy_base_z_intensity=focus)
                output.write(json.dumps(plain(result), allow_nan=False)+'\n')
                for pub in node.publishers:
                    pub.values.clear()
                frames += 1
                if frames % 20 == 0:
                    print('frames=%d s=%.2f reason=%s' % (frames, last_s, result['reason']), flush=True)
    write_json(args.output / 'road-failures.json', road_failures)
    if avoid.bev is not None:
        write_json(args.output / 'last-state.json', dict(hulls=avoid.bev.field.hulls,
            region_cells=avoid.bev.field.region_cells, remembered=avoid.bev.remembered,
            sightings=avoid.bev.sightings))
    write_json(args.output / 'summary.json', dict(frames=frames, elapsed_s=time.perf_counter()-began,
        baseline_head=args.baseline_head, behavior_probe=args.behavior,
        source_ref=source_ref, source_sha256=sources,
        input_manifest_sha256=sha256(args.input/'manifest.json'),
        limitations=['ROS doubles, no native transport or executor scheduling',
                     'One evaluation per cloud; only CRUISE/AVOID/TUNNEL in behavior probe',
                     'Tracking and wall memory start at the extraction window',
                     'Recorded trajectory, not closed-loop motion after a changed plan']))
    node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    ex = sub.add_parser('extract')
    ex.add_argument('--bag', type=Path, required=True)
    ex.add_argument('--start', type=float, required=True)
    ex.add_argument('--end', type=float, required=True)
    ex.add_argument('--output', type=Path, required=True)
    ex.set_defaults(run=extract)
    run = sub.add_parser('replay')
    run.add_argument('--input', type=Path, required=True)
    run.add_argument('--output', type=Path, required=True)
    run.add_argument('--baseline-head', action='store_true')
    run.add_argument('--behavior', action='store_true', help='Merge CRUISE/AVOID/TUNNEL states in the calculation probe')
    run.add_argument('--focus-x', type=float, required=True)
    run.add_argument('--focus-y', type=float, required=True)
    run.set_defaults(run=replay)
    args = parser.parse_args()
    args.run(args)


if __name__ == '__main__':
    main()
