#!/usr/bin/env python3
"""Compare synthetic tunnel decisions on the map and recorded wall evidence.

Read a CLOSED audit bag; never start ROS or publish controls. Reconstruct wall
memory only up to the first footprint scan at/after --at-s. This is a geometry
probe on prescribed poses, not replay of the recorded vehicle's decisions.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
import probe_tunnel_contingencies as probe
from ctrl_planning.ctrl_frenet import to_frenet
from ctrl_planning.ctrl_tunnel_corridor import WallMemory


def reconstruct(messages, runtime, at_s):
    """Keep only preceding TUNNEL wall evidence; never borrow a future scan."""
    memory = WallMemory()
    station = None
    mission = None
    evidence = []
    corridor = None
    for topic, msg, when in messages:
        if topic == '/localization/s':
            station = float(msg.data)
        elif topic == '/behavior/mission':
            mission = json.loads(msg.data)['mission']
        elif topic == '/perception/footprints' and mission == 'TUNNEL' and station is not None:
            data = json.loads(msg.data)
            walls = data.get('tunnel_walls', [])
            if data.get('frame') != 'map' or not data.get('scan_valid', True) or not walls:
                continue
            evidence.append(dict(record_time=when.to_sec(), stamp=data['stamp'], s=station, walls=walls))
            if memory.add(walls, runtime.route, station, 0., to_frenet, runtime.road):
                if corridor is None or memory.due(when.to_sec()):
                    corridor = memory.corridor(runtime.road, end_s=runtime.config['tunnel.s_to'])
            if station >= at_s and corridor is not None:
                # A distant query must not silently use an unrelated end-of-tunnel scan.
                if station - at_s > 3.:
                    raise ValueError('No nearby wall scan at the requested station')
                return corridor, evidence
    raise ValueError('No preceding TUNNEL wall corridor at the requested station')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--at-s', type=float, default=1880.)
    args = parser.parse_args(argv)
    if args.bag.suffix != '.bag' or not args.bag.is_file():
        parser.error('--bag must be an existing closed .bag file')
    # The original failing case remains unchanged; changing its query station
    # would compare different geometry without retaining its earlier contract.
    if args.at_s != 1880.:
        parser.error('This probe currently supports only the original s1880 fixture')
    args.output.mkdir(parents=True, exist_ok=False)
    before = probe.source_manifest()
    own = Path(__file__).resolve()
    before[str(own.relative_to(probe.PORT))] = hashlib.sha256(own.read_bytes()).hexdigest()
    runtime = probe.Runtime()
    import rosbag
    with rosbag.Bag(str(args.bag)) as bag:
        messages = bag.read_messages(topics=['/localization/s', '/behavior/mission', '/perception/footprints'])
        corridor, evidence = reconstruct(messages, runtime, args.at_s)
    cases = probe.scenarios()
    names = ['static_left_narrow', 'static_center', 'static_right']
    map_only = {name: runtime.run(cases[name]) for name in names}
    runtime.road = corridor
    walls = {name: runtime.run(cases[name]) for name in names}
    after = probe.source_manifest()
    after[str(own.relative_to(probe.PORT))] = hashlib.sha256(own.read_bytes()).hexdigest()
    unchanged = before == after
    def verdict(results):
        return {name: all(row['passed'] for row in rows) for name, rows in results.items()}
    summary = dict(bag=str(args.bag.resolve()), bag_bytes=args.bag.stat().st_size,
        at_s=args.at_s, wall_scans=len(evidence), last_scan_s=evidence[-1]['s'],
        last_record_time=evidence[-1]['record_time'], corridor_runs=len(corridor.runs),
        map_only=verdict(map_only), with_recorded_walls=verdict(walls), source_unchanged=unchanged,
        limitations=['Synthetic obstacle/ego poses, recorded walls only; no raw LiDAR or closed-loop simulation',
            'Bag record order and preceding s, not native callback timing or exact live wall rebuild phase',
            'Walls observed on one baseline lap, not proof that a changed obstacle cannot occlude them'])
    output = dict(summary=summary, sources=before, wall_evidence=evidence,
                  map_only=map_only, with_recorded_walls=walls)
    with (args.output / 'result.json').open('x') as stream:
        json.dump(probe.clean(output), stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(summary, indent=2))
    return 0 if unchanged and all(summary['with_recorded_walls'].values()) else 1


if __name__ == '__main__':
    sys.exit(main())
