#!/usr/bin/env python3
"""Read-only bag extraction and offline LiDAR calculation diagnostics.

No ROS master, publisher, UDP socket or vehicle control is started. `inspect`
uses the repository's explicit ROS test doubles, not native ROS transport.
Height translations are sensitivity experiments, never calibration estimates.
"""
import argparse
import hashlib
import importlib
import importlib.util
import json
import linecache
import math
from pathlib import Path
import sys
import subprocess

sys.dont_write_bytecode = True
PORT = Path(__file__).resolve().parents[1]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def plain(value):
    import numpy as np
    if isinstance(value, np.ndarray):
        return plain(value.tolist())
    if isinstance(value, np.generic):
        return plain(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    return value


def write_json(path, value):
    with path.open('x') as stream:
        json.dump(plain(value), stream, indent=2, allow_nan=False)
        stream.write('\n')


def extract(args):
    import numpy as np
    import rosbag
    import rospy
    from sensor_msgs import point_cloud2

    args.output.mkdir(parents=True, exist_ok=False)
    topics = ['/localization/odometry', '/localization/s', '/behavior/mission',
              '/behavior/target', '/perception/footprints', '/sensors/lidar/points']
    context = {}
    with rosbag.Bag(str(args.bag), 'r') as bag:
        for topic, msg, when in bag.read_messages(
                topics=topics, start_time=rospy.Time.from_sec(args.at - 2.),
                end_time=rospy.Time.from_sec(args.at + 1.)):
            if topic == '/sensors/lidar/points':
                if when.to_sec() < args.at:
                    continue
                names = [field.name for field in msg.fields]
                fields = ['x', 'y', 'z'] + (['intensity'] if 'intensity' in names else [])
                points = np.array(list(point_cloud2.read_points(
                    msg, field_names=fields, skip_nans=True)), dtype=np.float32)
                if msg.header.frame_id != 'lidar':
                    raise ValueError('Only the recorded lidar frame is supported')
                np.savez_compressed(args.output / 'cloud.npz', points=points,
                                    stamp=msg.header.stamp.to_sec())
                context.update(bag=str(args.bag), requested_record_time=args.at,
                               cloud_record_time=when.to_sec(),
                               cloud_stamp=msg.header.stamp.to_sec(), fields=fields,
                               frame_id=msg.header.frame_id)
                write_json(args.output / 'context.json', context)
                print(json.dumps(dict(output=str(args.output), points=len(points),
                                      s=context.get('s'), mission=context.get('mission'))))
                return
            if topic == '/localization/odometry':
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                context['ego'] = [p.x, p.y, math.atan2(2 * (q.w*q.z + q.x*q.y),
                                                     1 - 2 * (q.y*q.y + q.z*q.z))]
                context['odom_stamp'] = msg.header.stamp.to_sec()
                context['speed'] = msg.twist.twist.linear.x
            elif topic == '/localization/s':
                context['s'] = msg.data
            elif topic == '/behavior/mission':
                context['mission'] = json.loads(msg.data)['mission']
            elif topic == '/behavior/target':
                context['recorded_target'] = dict(state=msg.state, v_max=msg.v_max,
                                                  stop_s=msg.stop_s)
            elif topic == '/perception/footprints':
                context['recorded_footprints'] = json.loads(msg.data)
    raise ValueError('No LiDAR frame within one second of the requested bag time')


def trace_calculation(function, *args):
    """Inspect the actual calculation without duplicating or changing it."""
    captured = {}
    code = getattr(function, '__func__', function).__code__

    def trace(frame, event, arg):
        if frame.f_code is code:
            if event == 'return':
                captured.update(frame.f_locals)
                captured['_return_location'] = dict(
                    file=frame.f_code.co_filename, line=frame.f_lineno,
                    source=''.join(linecache.getline(frame.f_code.co_filename, line)
                                   for line in range(max(1, frame.f_lineno - 2), frame.f_lineno + 1)))
            return trace
        return None

    previous = sys.gettrace()
    try:
        sys.settrace(trace)
        result = function(*args)
    finally:
        sys.settrace(previous)
    return result, captured


def inspect(args):
    import numpy as np
    sys.path.insert(0, str(PORT / 'verification'))
    import ros_stubs
    ros_stubs.install(PORT)
    for package in (PORT / 'src').iterdir():
        if package.is_dir():
            sys.path.insert(0, str(package))
    source_paths = [PORT / 'src/ctrl_lidar/ctrl_lidar/ctrl_ground.py',
                    PORT / 'src/ctrl_lidar/ctrl_lidar/ctrl_obstacle_node.py']
    source_hashes = {str(path.relative_to(PORT)): sha256(path) for path in source_paths}
    source_ref = 'working tree'
    if args.baseline_head:
        # Load the two committed calculation modules in memory. No checkout,
        # source replacement, alternate expectations or original-tree tests.
        repo = PORT.parent
        source_ref = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
        package = importlib.import_module('ctrl_lidar')
        for path in source_paths:
            relative = str(path.relative_to(repo))
            source = subprocess.check_output(['git', 'show', source_ref + ':' + relative], cwd=repo)
            source_hashes[str(path.relative_to(PORT))] = hashlib.sha256(source).hexdigest()
            name = 'ctrl_lidar.' + path.stem
            module = importlib.util.module_from_spec(importlib.util.spec_from_loader(name, loader=None))
            filename = '<' + source_ref + ':' + relative + '>'
            module.__file__ = filename
            linecache.cache[filename] = (len(source), None, source.decode().splitlines(True), filename)
            sys.modules[name] = module
            setattr(package, path.stem, module)
            exec(compile(source, filename, 'exec'), module.__dict__)
    from ctrl_lidar.ctrl_ground import estimate_ground_plane, region, relative_height
    from ctrl_lidar.ctrl_obstacle_node import ObstacleNode
    from ctrl_ros1.ctrl_parameter import Parameter
    from ctrl_ros1 import ctrl_point_cloud2 as point_cloud2
    from sensor_msgs.msg import PointField
    from std_msgs.msg import Header, String

    with np.load(args.cloud, allow_pickle=False) as data:
        raw = data['points'].copy()
        stamp = float(data['stamp'])
    if raw.ndim != 2 or raw.shape[1] not in (3, 4):
        raise ValueError('Expected Nx3 or Nx4 XYZ/intensity array')
    context = json.loads(args.context.read_text()) if args.context else {}
    output = dict(scope='Offline calculations with ROS doubles, not runtime/drive validation',
                  cloud_sha256=sha256(args.cloud), cloud=str(args.cloud), stamp=stamp,
                  source_sha256=source_hashes, source_ref=source_ref,
                  context=context, experiments=[])
    fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(('x', 'y', 'z', 'intensity')[:raw.shape[1]])]
    for shift in dict.fromkeys([0.] + args.diagnostic_z_shift):
        params = dict(use_tf=False, publish_markers=False,
                      link_set=str(PORT / 'src/ctrl_bringup/maps/link_set.json'))
        node = ObstacleNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        try:
            if context.get('ego'):
                node.ego = tuple(context['ego'])
                node.odom_buf.append((context.get('odom_stamp', stamp), *node.ego))
            node.ego_s = context.get('s')
            mission = context.get('mission', 'TUNNEL')
            node.on_mission(String(data=json.dumps(dict(mission=mission))))
            # No runtime transform is changed. Translate this experiment's copied input only.
            translated = raw.copy()
            translated[:, 2] += shift
            cloud = point_cloud2.create_cloud(Header(frame_id='lidar'), fields, translated)
            base = node.lidar_to_base(translated[:, :3], 'lidar', node.get_clock().now())
            generic_plane, locals_ = trace_calculation(estimate_ground_plane, base)
            detections, detection_locals = trace_calculation(node.detect, cloud, stamp)
            plane = detection_locals.get('plane')
            near = region(base)
            height = relative_height(base, plane)
            materials = {}
            if raw.shape[1] == 4:
                for label, low, high in [('asphalt_candidate', node.road_intensity_min, node.road_intensity_max),
                                         ('paint_candidate', node.paint_intensity_min, node.paint_intensity_max)]:
                    mask = near & (raw[:, 3] >= low) & (raw[:, 3] <= high)
                    materials[label] = dict(count=int(mask.sum()),
                        base_z_p05_p50_p95=np.percentile(base[mask, 2], [5, 50, 95]) if mask.any() else [],
                        at_or_above_paint_gate=int((mask & (height >= node.paint_max_z)).sum()))
            # Associate surviving detections with the actual DBSCAN input. No extra
            # material filter is applied; intensity is evidence, not ground truth.
            labels = detection_locals.get('labels')
            intensities = detection_locals.get('intensity')
            if labels is not None and intensities is not None:
                pts = detection_locals['pts']
                for detection in detections:
                    matching = [label for label in np.unique(labels) if label >= 0
                                and int((labels == label).sum()) == detection['n']
                                and np.linalg.norm(pts[labels == label, :2].mean(axis=0)
                                                   - [detection['x'], detection['y']]) < 1e-4]
                    if len(matching) == 1:
                        values = intensities[labels == matching[0]]
                        detection['intensity_p05_p50_p95'] = np.percentile(values, [5, 50, 95])
                        detection['paint_band_fraction'] = float(np.mean(
                            (values >= node.paint_intensity_min) & (values <= node.paint_intensity_max)))
            matches = []
            if context.get('ego') and detections:
                ex, ey, yaw = context['ego']
                c, s = math.cos(yaw), math.sin(yaw)
                centers = np.array([[ex+c*d['x']-s*d['y'], ey+s*d['x']+c*d['y']]
                                    for d in detections])
                for obj in context.get('recorded_footprints', {}).get('objects', []):
                    distances = np.linalg.norm(centers - obj['center'], axis=1)
                    index = int(distances.argmin())
                    matches.append(dict(recorded_id=obj['id'], nearest_detection=index,
                                        center_distance_m=float(distances[index])))
            support = locals_.get('support')
            output['experiments'].append(dict(
                diagnostic_only=bool(shift), input_z_translation=shift,
                plane=plane, generic_plane=generic_plane,
                ground_plane_source=('generic' if generic_plane is not None else
                                     'tunnel_material' if plane is not None else 'none'),
                last_candidate_or_refit=locals_.get('plane'),
                estimator_return_location=locals_.get('_return_location'),
                best_inlier_count=locals_.get('best_inlier_count', 0),
                support_span_xyz=None if support is None else np.ptp(support, axis=0),
                materials=materials, detection_count=len(detections), detections=detections,
                recorded_footprint_matches=matches,
                limitations=['Single frame, no tracking warmup or planner state',
                             'Context uses the last preceding bag pose, without interpolation',
                             'No speed history: pitch compensation is zero',
                             'Default obstacle launch parameters; no full launch replay',
                             'Intensity bands alone do not establish object ground truth']))
        finally:
            node.destroy_node()
    write_json(args.output, output)
    print(json.dumps(dict(output=str(args.output), experiments=[
        dict(z_shift=e['input_z_translation'], plane=plain(e['plane']),
             detections=e['detection_count']) for e in output['experiments']])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    ex = sub.add_parser('extract', help='Read native rosbag without publishing or starting ROS')
    ex.add_argument('--bag', type=Path, required=True)
    ex.add_argument('--at', type=float, required=True, help='Bag record time, seconds')
    ex.add_argument('--output', type=Path, required=True, help='New output directory')
    ex.set_defaults(run=extract)
    ins = sub.add_parser('inspect', help='Replay one NPZ cloud through existing detection code')
    ins.add_argument('--cloud', type=Path, required=True)
    ins.add_argument('--context', type=Path)
    ins.add_argument('--baseline-head', action='store_true',
                     help='Load committed ground/detection modules in memory for same-input comparison')
    ins.add_argument('--diagnostic-z-shift', type=float, action='append', default=[])
    ins.add_argument('--output', type=Path, required=True)
    ins.set_defaults(run=inspect)
    args = parser.parse_args()
    args.run(args)


if __name__ == '__main__':
    main()
