#!/usr/bin/env python3
"""Create a new MORAI scene with one additional static tunnel box; never load it.

Clone an existing static asset in the supplied K-city scene. Preserve every
existing actor, ego, spawn, traffic-light and sensor field. Scale factors are
not physical dimensions; validate the loaded geometry with LiDAR before a trial.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys

sys.dont_write_bytecode = True
PORT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PORT / 'src/ctrl_control'))
import numpy as np
from ctrl_control.ctrl_core import Route


def ids(value):
    if isinstance(value, dict):
        if type(value.get('UNIQUEID')) is int:
            yield value['UNIQUEID']
        for child in value.values():
            yield from ids(child)
    elif isinstance(value, list):
        for child in value:
            yield from ids(child)


def build(scene, template_id, position, heading_deg, scale, actor_id=None):
    map_info = scene.get('mapInfo', {})
    if (map_info.get('mapName') != 'R_KR_PR_K-city_2025'
            or map_info.get('eastOffset') != 302595.
            or map_info.get('northOffset') != 4124145.):
        raise ValueError('Unsupported map/origin; use the current K-city sample scene')
    if len(position) != 3 or len(scale) != 3 or not all(
            math.isfinite(v) for v in (*position, heading_deg, *scale)) or min(scale) <= 0.:
        raise ValueError('Finite pose and positive scale factors required')
    matches = [obj for obj in scene.get('objectList', []) if obj.get('UNIQUEID') == template_id]
    if len(matches) != 1:
        raise ValueError('Exactly one static template id required')
    source = matches[0]
    if source.get('moveSpeed', 0.) != 0. or source.get('waypointEventList'):
        raise ValueError('Moving or waypoint-driven template is not supported')
    used = set(ids(scene))
    if actor_id is None:
        actor_id = max(used, default=0) + 1
    if type(actor_id) is not int or actor_id <= 0 or actor_id in used:
        raise ValueError('Require an unused positive integer actor id')
    path_name = 'ObstaclePath - %d' % actor_id
    if any(row.get('pathName') == path_name for row in scene.get('waypointDataList', [])):
        raise ValueError('Actor waypoint name already exists')
    actor = copy.deepcopy(source)
    actor['UNIQUEID'] = actor_id
    for field, values in [('pos', position), ('scale', scale)]:
        actor[field] = {**dict(zip('xyz', map(float, values))),
                        **{'_' + axis: '%.3f' % value for axis, value in zip('xyz', values)}}
    actor['rot'] = dict(roll='0.000', pitch='0.000', yaw='%.3f' % heading_deg)
    result = copy.deepcopy(scene)
    result['objectList'].append(actor)
    if actor.get('isWaypointMode'):
        result.setdefault('waypointDataList', []).append(
            dict(pathName=path_name, waypointData=[]))
    return result, actor


def ground_z(links, point):
    """Interpolate the closest surveyed map segment in XY; retain its Z."""
    starts = np.vstack([np.asarray(row['points'], float)[:-1] for row in links])
    ends = np.vstack([np.asarray(row['points'], float)[1:] for row in links])
    delta = ends - starts
    length2 = np.sum(delta[:, :2]**2, axis=1)
    u = np.clip(np.sum((point - starts[:, :2]) * delta[:, :2], axis=1) / np.maximum(length2, 1e-12), 0., 1.)
    projected = starts + u[:, None] * delta
    distance = np.linalg.norm(projected[:, :2] - point, axis=1)
    best = int(np.argmin(distance))
    if distance[best] > 5.:
        raise ValueError('No nearby surveyed road height')
    return float(projected[best, 2])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--template-id', type=int, default=2)
    parser.add_argument('--actor-id', type=int, required=True,
                        help='Unused id verified by native MORAI spawn/load; max existing id + 1 is not sufficient')
    parser.add_argument('--station', type=float, default=1905.)
    parser.add_argument('--offset', type=float, default=1.)
    parser.add_argument('--scale', nargs=3, type=float, default=[3., 1., 1.])
    args = parser.parse_args(argv)
    if not (math.isfinite(args.station) and 1868. <= args.station <= 1970.
            and math.isfinite(args.offset) and abs(args.offset) <= 6.):
        parser.error('Require tunnel station 1868..1970 and finite offset within 6m')
    raw = args.scene.read_bytes()
    scene = json.loads(raw)
    route_path = PORT / 'src/ctrl_bringup/routes/global_path.txt'
    map_path = PORT / 'src/ctrl_bringup/maps/link_set.json'
    route = Route(np.loadtxt(route_path))
    heading = route.heading_at(args.station)
    xy = np.asarray(route.at(args.station)) + args.offset * np.array([-math.sin(heading), math.cos(heading)])
    position = [*xy, ground_z(json.loads(map_path.read_text()), xy)]
    result, actor = build(scene, args.template_id, position, math.degrees(heading), args.scale,
                          actor_id=args.actor_id)
    args.output.mkdir(parents=True, exist_ok=False)
    generated = args.output / 'CTRL_tunnel_left_box.json'
    with generated.open('x') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    with (args.output / 'original-scene.json').open('xb') as stream:
        stream.write(raw)
    restored = copy.deepcopy(result)
    restored['objectList'].pop()
    if actor.get('isWaypointMode'):
        restored['waypointDataList'].pop()
        if 'waypointDataList' not in scene:
            del restored['waypointDataList']
    summary = dict(source=str(args.scene.resolve()), source_sha256=hashlib.sha256(raw).hexdigest(),
        generated=str(generated.resolve()), generated_sha256=hashlib.sha256(generated.read_bytes()).hexdigest(),
        station=args.station, offset=args.offset, added_actor=actor,
        unchanged_existing_scene=(restored == scene),
        added_waypoint_path=('ObstaclePath - %d' % actor['UNIQUEID'] if actor.get('isWaypointMode') else None),
        route_sha256=hashlib.sha256(route_path.read_bytes()).hexdigest(),
        map_sha256=hashlib.sha256(map_path.read_bytes()).hexdigest(),
        warnings=['Generated only, not loaded into MORAI', 'Scale factors are not measured physical dimensions',
                  'Explicit actor id still requires native load/save verification for each scene',
                  'Surveyed ground Z, object seating and collision must be checked after loading',
                  'Found file content is not independent proof of the scene currently loaded in MORAI'])
    with (args.output / 'manifest.json').open('x') as stream:
        json.dump(summary, stream, indent=2, allow_nan=False)
        stream.write('\n')
    assert args.scene.read_bytes() == raw
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
