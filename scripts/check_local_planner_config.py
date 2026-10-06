#!/usr/bin/env python3
"""Preflight YAML validation without ROS. Exit 2 on ERROR; WARNING exits 0."""
import argparse
import math
from pathlib import Path
import yaml
from local_planner_tools import (DEFAULT_CONFIG, GLOBAL_MAP_OFFSET, finite_number,
                                load_document, load_path, protect_outputs, write_json)
from local_planning_core import FrenetLocalPlanner, validate_zones


def check_config(document, points=None, external_lidar_tf=False):
    findings = []
    def add(level, item, message):
        findings.append(dict(level=level, item=item, message=message))
    p = document.get('static_obstacle_avoidance_planner')
    if not isinstance(p, dict):
        add('ERROR', 'YAML', 'static_obstacle_avoidance_planner must be a mapping')
        return findings
    positive = ('vehicle_width', 'vehicle_front', 'vehicle_rear', 'wheelbase', 'road_half_width',
                'horizon', 'transition_length', 'return_length', 'sample_step', 'max_curvature',
                'max_steering_rate_per_m', 'max_heading_error', 'min_reference_segment',
                'max_reference_yaw_change', 'max_reference_curvature', 'max_lateral_accel',
                'static_speed_kmh', 'dynamic_speed_kmh', 'approach_speed_kmh', 'update_hz',
                'input_timeout', 'global_path_timeout', 'obstacle_memory_s',
                'obstacle_merge_distance', 'return_clear_time', 'return_d_tolerance',
                'return_heading_tolerance')
    nonnegative = ('safety_margin', 'switch_penalty', 'switch_hysteresis', 'approach_distance')
    for key in positive + nonnegative:
        value = p.get(key)
        ok = finite_number(value) and (value > 0 if key in positive else value >= 0)
        add('PASS' if ok else 'ERROR', key, str(value) if ok else 'requires finite numeric %s; got %r' %
            ('> 0' if key in positive else '>= 0', value))
    def numbers(*keys):
        return all(finite_number(p.get(key)) for key in keys)
    if numbers('vehicle_width', 'safety_margin', 'road_half_width'):
        half = p['vehicle_width'] / 2 + p['safety_margin']
        add('ERROR' if half >= p['road_half_width'] else 'PASS', 'footprint/corridor',
            'required half-width %.3fm; corridor %.3fm' % (half, p['road_half_width']))
    if numbers('horizon', 'transition_length', 'return_length'):
        ok = p['horizon'] >= p['transition_length'] + p['return_length']
        add('PASS' if ok else 'ERROR', 'horizon coverage', 'horizon must cover transition + return')
    if numbers('wheelbase', 'vehicle_front', 'vehicle_rear'):
        ok = p['wheelbase'] <= p['vehicle_front'] + p['vehicle_rear']
        add('PASS' if ok else 'ERROR', 'wheelbase/length', 'wheelbase must fit vehicle length')
    offsets = p.get('offsets')
    valid_offsets = isinstance(offsets, list) and bool(offsets) and all(finite_number(v) for v in offsets)
    add('PASS' if valid_offsets else 'ERROR', 'offsets', repr(offsets))
    if valid_offsets:
        if len(set(offsets)) != len(offsets):
            add('WARNING', 'offsets', 'duplicate offsets will be deduplicated by core')
        if 0 not in offsets:
            add('WARNING', 'offsets', 'core also inserts the centre candidate 0')
        if numbers('road_half_width', 'vehicle_width', 'safety_margin'):
            beyond = [v for v in offsets if abs(v) > p['road_half_width']]
            impossible = [v for v in offsets if abs(v) + p['vehicle_width']/2 + p['safety_margin'] > p['road_half_width']]
            if beyond:
                add('ERROR', 'offsets/corridor', 'offset centres outside corridor: ' + repr(beyond))
            elif impossible:
                add('WARNING', 'offsets/footprint', 'some offsets cannot fit even aligned: ' + repr(impossible))
    for key, minimum in (('reference_behind_points', 0), ('reference_ahead_points', 4)):
        ok = type(p.get(key)) is int and p[key] >= minimum
        add('PASS' if ok else 'ERROR', key, 'requires integer >= %d' % minimum)
    weights = p.get('weights')
    keys = {'obstacle', 'offset', 'curvature', 'steering', 'continuity', 'return'}
    ok = isinstance(weights, dict) and set(weights) == keys and all(finite_number(v) and v >= 0 for v in weights.values())
    add('PASS' if ok else 'ERROR', 'weights', 'requires all six finite nonnegative cost weights')
    if p.get('dynamic_policy') not in ('stop_on_obstacle', 'observe_only'):
        add('ERROR', 'dynamic_policy', 'unknown policy')
    elif p['dynamic_policy'] == 'observe_only':
        add('WARNING', 'dynamic_policy', 'observe_only does not stop for newly entered dynamic missions')
    zones = []
    for key in ('static_zones', 'dynamic_zones'):
        try:
            validate_zones(p.get(key))
        except ValueError as exc:
            add('ERROR', key, str(exc))
            continue
        if not p[key]:
            add('WARNING', key, 'empty: this mission is disabled')
        else:
            add('PASS', key, '%d zones' % len(p[key]))
        for i, z in enumerate(p[key]):
            label = '%s[%d]' % (key, i)
            if 'start' in z and points is not None and z['end'] >= len(points):
                add('ERROR', label, 'end exceeds path waypoint count %d' % len(points))
            zones.append((label, z))
    for i, (label, a) in enumerate(zones):
        for other, b in zones[i+1:]:
            if ('start' in a) == ('start' in b):
                overlap = (max(a['start'], b['start']) <= min(a['end'], b['end'])) if 'start' in a else (
                    max(a['xmin'], b['xmin']) <= min(a['xmax'], b['xmax']) and
                    max(a['ymin'], b['ymin']) <= min(a['ymax'], b['ymax']))
            elif points is None:
                add('WARNING', 'zone overlap', label + ' / ' + other + ': mixed representations need --path to compare')
                continue
            else:
                index_zone, rect = (a, b) if 'start' in a else (b, a)
                subset = points[index_zone['start']:index_zone['end']+1]
                from local_planning_state import approach_zone_distance
                overlap = any(rect['xmin'] <= x <= rect['xmax'] and rect['ymin'] <= y <= rect['ymax'] for x, y in subset)
                if not overlap and len(subset) > 1:
                    length = float(sum(math.hypot(*(v-u)) for u,v in zip(subset[:-1],subset[1:])))
                    overlap = approach_zone_distance([rect], 0, *subset[0], subset, length + 1e-6) is not None
            if overlap:
                policy = 'dynamic zones take precedence' if label.split('[')[0] != other.split('[')[0] else 'same-mission ranges overlap'
                add('WARNING', 'zone overlap', label + ' overlaps ' + other + '; ' + policy)
    viz = document.get('viz_planner')
    if not isinstance(viz, dict):
        add('ERROR', 'viz_planner', 'must be a mapping')
    else:
        extrinsics = viz.get('lidar_xyz_rpy')
        if extrinsics == []:
            add('PASS' if external_lidar_tf else 'WARNING', 'lidar_xyz_rpy',
                'external TF declared; verify live map->LiDAR at scan time' if external_lidar_tf else
                'empty: external TF provider required; active zone will HOLD without TF')
        elif not isinstance(extrinsics, list) or len(extrinsics) != 6 or not all(finite_number(v) for v in extrinsics):
            add('ERROR', 'lidar_xyz_rpy', 'requires [] or six finite numbers [x,y,z,roll,pitch,yaw] in m/rad')
        else:
            add('PASS', 'lidar_xyz_rpy', repr(extrinsics))
            if external_lidar_tf:
                add('WARNING', 'lidar_xyz_rpy', 'configured transform and external provider may publish duplicate TF')
        if not isinstance(viz.get('lidar_frame'), str) or not viz['lidar_frame'].strip():
            add('ERROR', 'lidar_frame', 'requires a nonempty frame name')
    # Same core validation used by the running planner; never bypass its limits.
    if not any(f['level'] == 'ERROR' for f in findings):
        try:
            FrenetLocalPlanner(p)
            add('PASS', 'runtime core', 'configuration accepted')
        except (ValueError, TypeError, KeyError) as exc:
            add('ERROR', 'runtime core', str(exc))
    return findings


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--path', type=Path, help='optional path for bounds/mixed-zone checks')
    parser.add_argument('--map-offset', nargs=2, type=float, default=GLOBAL_MAP_OFFSET, metavar=('X', 'Y'),
                        help='file XY to map translation; defaults match existing GlobalPathPlanner')
    parser.add_argument('--external-lidar-tf', action='store_true')
    parser.add_argument('--json', type=Path)
    args = parser.parse_args(argv)
    try:
        protect_outputs([args.json], [args.config] + ([args.path] if args.path else []))
        if not all(math.isfinite(v) for v in args.map_offset):
            raise ValueError('map-offset must be finite')
        points = load_path(args.path, args.map_offset)[0] if args.path else None
        findings = check_config(load_document(args.config), points, args.external_lidar_tf)
    except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
        print('ERROR: ' + str(exc))
        print('ERROR가 있으므로 Local Planner를 실제 주행에 사용하면 안 됩니다.')
        return 2
    for row in findings:
        print('{level}: {item}: {message}'.format(**row))
    errors = sum(f['level'] == 'ERROR' for f in findings)
    warnings = sum(f['level'] == 'WARNING' for f in findings)
    print('SUMMARY: %d ERROR, %d WARNING' % (errors, warnings))
    if errors:
        print('ERROR가 있으므로 Local Planner를 실제 주행에 사용하면 안 됩니다.')
    else:
        print('PASS: offline config checks only; live TF, measurements and MORAI validation still required.')
    if args.json:
        try:
            write_json(args.json, dict(config=str(args.config.resolve()), errors=errors, warnings=warnings, findings=findings))
        except OSError as exc:
            print('ERROR: cannot write report: ' + str(exc))
            return 2
    return 2 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
