#!/usr/bin/env python3
"""Probe production tunnel decisions with synthetic observations, without ROS.

Uses the checked-in map, route and mission/behavior configuration. Poses are
prescribed, not driven by the outputs. This is not a sensor or collision test.
Every invocation needs a new output directory; previous evidence is immutable.
"""
import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import sys
import time

sys.dont_write_bytecode = True
PORT = Path(__file__).resolve().parents[1]
# Resolve package:// geometry from this checkout, independently of a host ROS overlay.
os.environ['ROS_PACKAGE_PATH'] = str(PORT / 'src')
for package in ('ctrl_control', 'ctrl_planning', 'ctrl_ros1'):
    sys.path.insert(0, str(PORT / 'src' / package))

import numpy as np
import yaml

from ctrl_control.ctrl_core import Route
from ctrl_control.ctrl_mission_config import load_mission_config
from ctrl_planning.ctrl_lane_prediction import LaneMap
from ctrl_planning.ctrl_mission import MissionSupervisor
from ctrl_planning.ctrl_state import Behavior, Context, EventInfo
from ctrl_planning.states.ctrl_avoid import Avoid
from ctrl_planning.states.ctrl_cruise import Cruise
from ctrl_planning.states.ctrl_tunnel import Tunnel
from ctrl_planning.states.ctrl_yield import Yield


@dataclass(frozen=True)
class Body:
    key: str = 'lidar:barrier'
    kind: str = 'OBSTACLE'
    s: float = 1905.
    d: float = 0.
    length: float = 3.
    width: float = 2.
    height: float = 1.02
    lateral_mps: float = 0.


@dataclass(frozen=True)
class Frame:
    now: float
    bodies: tuple = ()
    expect: str = 'move'
    s: float = 1880.
    speed: float = 0.
    loc_mode: str = 'DR'
    scan_stamp: float = None
    cleared: tuple = ()
    mapped: bool = True
    expected_mission: str = 'TUNNEL'


def scenarios():
    barrier = Body()
    wall = Body(s=1890., width=20.)
    person = Body(key='lidar:person', kind='PEDESTRIAN', s=1891.,
                  length=.6, width=.6, height=1.7)
    outside = Body(key=person.key, kind=person.kind, s=person.s, d=10.,
                   length=.6, width=.6, height=1.7)
    cases = {
        'clear_dr': [Frame(1.)],
        'clear_gps': [Frame(1., loc_mode='GPS')],
        'clear_at_50kmh': [Frame(1., speed=50./3.6)],
        'gps_return_at_exit': [Frame(1., s=1989.), Frame(2., s=1992., loc_mode='GPS',
                                                   expected_mission='CRUISE')],
        'missing_map': [Frame(1., (barrier,), 'stop', mapped=False)],
        'fully_blocked_then_cleared': [
            Frame(1., (wall,), 'stop'), Frame(1.1, (wall,), 'stop'),
            Frame(1.2, cleared=({'id': wall.key, 'last_seen': 1.1},)),
            Frame(1.3),
        ],
        'late_blocker': [Frame(1.), Frame(1.1, (Body(s=1886., width=20.),), 'stop')],
        'stale_scan_then_restored': [
            Frame(1., (barrier,), 'bypass'), Frame(1.1, (barrier,), 'bypass'),
            Frame(2., (barrier,), 'stop', scan_stamp=1.1),
            Frame(2.1, cleared=({'id': barrier.key, 'last_seen': 1.1},)),
            Frame(2.2),
        ],
        'pedestrian_clear_and_reenter': [
            Frame(1., (person,), 'yield'), Frame(1.1, (person,), 'yield'),
            Frame(1.2, (outside,)), Frame(1.3, (outside,)),
            Frame(1.4, (person,), 'yield'),
        ],
        'pedestrian_during_bypass': [
            Frame(1., (barrier,), 'bypass'),
            Frame(1.1, (barrier, person), 'yield_with_curve'),
        ],
        'unknown_crossing_body': [Frame(1., (Body(
            key='lidar:unknown', kind='DYNAMIC_UNKNOWN', s=1891.,
            length=.6, width=.6, height=1.7, lateral_mps=1.5),), 'yield')],
    }
    for label, offset in [('right', -1.), ('center', 0.)]:
        cases['static_' + label] = [Frame(1., (Body(d=offset),), 'bypass')]
    # At d=+1 a 2 m body closes the mapped gap with vehicle width + margin.
    # Keep that stop case separately from a narrower, passable left-side body.
    cases['static_left_no_paved_gap'] = [Frame(1., (Body(d=1.),), 'stop')]
    cases['static_left_narrow'] = [Frame(1., (Body(d=1., width=1.),), 'bypass')]
    for kmh in (20., 50.):
        cases['sudden_person_at_%dkmh' % kmh] = [
            Frame(1., speed=kmh/3.6),
            Frame(1.1, (person,), 'yield', s=1880.+kmh/3.6*.1, speed=kmh/3.6),
        ]
    return cases


class Runtime:
    def __init__(self):
        self.route = Route(np.loadtxt(PORT / 'src/ctrl_bringup/routes/global_path.txt'))
        self.road = LaneMap(json.loads((PORT / 'src/ctrl_bringup/maps/link_set.json').read_text()))
        self.config = yaml.safe_load((PORT / 'src/ctrl_planning/config/behavior.yaml').read_text())[
            'behavior_node']['ros__parameters']
        self.missions = load_mission_config(PORT / 'src/ctrl_planning/config/missions.yaml')

    def behavior(self):
        states = []
        for cls in (Cruise, Avoid, Yield, Tunnel):
            prefix = cls.name.lower() + '.'
            states.append(cls(**{key[len(prefix):]: value for key, value in self.config.items()
                                 if key.startswith(prefix)}))
        return Behavior(states)

    def body(self, body, stamp):
        heading = self.route.heading_at(body.s)
        tangent = np.array([math.cos(heading), math.sin(heading)])
        normal = np.array([-tangent[1], tangent[0]])
        center = np.asarray(self.route.at(body.s)) + normal * body.d
        hull = tuple(tuple(center + tangent * a * body.length / 2. + normal * b * body.width / 2.)
                     for a, b in [(-1, -1), (1, -1), (1, 1), (-1, 1)])
        velocity = normal * body.lateral_mps
        event = EventInfo(type=body.kind, stamp=stamp, s=body.s, d=body.d,
                          x=float(center[0]), y=float(center[1]),
                          size_x=body.length, size_y=body.width, box_yaw=heading,
                          vx=float(velocity[0]), vy=float(velocity[1]),
                          value=body.key, footprint=hull)
        return event, hull, tuple(velocity)

    def context(self, frame):
        x, y = self.route.at(frame.s)
        ctx = Context(s=frame.s, x=x, y=y, yaw=self.route.heading_at(frame.s),
                      v=frame.speed, now=frame.now, loc_mode=frame.loc_mode,
                      route=self.route, road_map=self.road if frame.mapped else None)
        stamp = frame.now if frame.scan_stamp is None else frame.scan_stamp
        ctx.occupancy_stamp = stamp
        ctx.occupancy_sensor = (x, y)
        ctx.occupancy_hulls = {}
        ctx.occupancy_heights = {}
        ctx.occupancy_kinds = {}
        ctx.occupancy_speeds = {}
        ctx.occupancy_cleared = frame.cleared
        for body in frame.bodies:
            event, hull, velocity = self.body(body, stamp)
            if -.05 <= ctx.now - event.stamp <= self.config['event_timeout_s']:
                ctx.events.append(event)
            ctx.occupancy_hulls[body.key] = hull
            ctx.occupancy_heights[body.key] = body.height
            ctx.occupancy_kinds[body.key] = body.kind
            ctx.occupancy_speeds[body.key] = velocity
        return ctx

    def run(self, frames):
        behavior = self.behavior()
        mission = MissionSupervisor(self.missions)
        rows = []
        for frame in frames:
            ctx = self.context(frame)
            if mission.update(ctx):
                behavior.reset()
            yielding = next(state for state in behavior.states if state.name == 'YIELD')
            ctx.yield_left_key = yielding.left_key if not behavior.engaged['YIELD'] else None
            mission.apply_context(ctx)
            started = time.monotonic()
            target = mission.limit(behavior.tick(ctx, allowed_states=mission.policy.allowed_states))
            duration = time.monotonic() - started
            avoid = next(state for state in behavior.states if state.name == 'AVOID')
            yielding = next(state for state in behavior.states if state.name == 'YIELD')
            mission.observe_pedestrian_yield(active=behavior.engaged['YIELD'],
                pedestrian=yielding.target_type == 'PEDESTRIAN',
                verified_clear=yielding.pedestrian_passage_confirmed)
            checks = evaluate(frame.expect, target, behavior.last_targets)
            # Mandatory for every frame, independent of its expected behavior.
            checks['finite_nonnegative_speed_cap'] = math.isfinite(target.v_max) and target.v_max >= 0.
            checks['mission_speed_cap'] = target.v_max <= ctx.mission_speed_mps + 1e-9
            checks['mission'] = ctx.mission_name == frame.expected_mission
            gap = None
            if frame.expect == 'bypass' and not all(checks.values()) and avoid.boxes():
                gap = dict(inflated_half_width_m=avoid.path_planner(ctx).half,
                           boxes=[asdict(box) for box in avoid.boxes()],
                           paved_offsets=avoid.slalom_offsets(ctx, avoid.boxes()))
            rows.append(dict(input=asdict(frame), mission=ctx.mission_name,
                target=asdict(target), engaged=behavior.active_names(),
                state_targets={name: asdict(value) for name, value in behavior.last_targets.items()},
                bev_reason=avoid.bev.last_reason if avoid.bev else None,
                yield_reason=yielding.reason, checks=checks, passed=all(checks.values()),
                gap_diagnostic=gap, calculation_s=duration))
        return rows


def evaluate(expect, target, state_targets):
    """Behavior contracts, not snapshots of the current output values."""
    stopped = target.v_max == 0.
    moving = target.v_max > 0. and not math.isfinite(target.stop_s)
    if expect == 'stop':
        return {'stop_command': stopped}
    if expect == 'move':
        return {'movement_available': moving}
    if expect == 'bypass':
        return {'movement_available': moving, 'avoidance_curve': bool(target.chain)
                and any(abs(move[0]) > .1 for move in target.chain)}
    if expect in ('yield', 'yield_with_curve'):
        checks = {'yield_active': target.state == 'YIELD', 'stop_command': stopped,
                  'stop_position': math.isfinite(target.stop_s)}
        if expect == 'yield_with_curve':
            lateral = state_targets.get('AVOID')
            checks['retained_avoidance_curve'] = lateral is not None and bool(lateral.chain) and (
                target.chain, target.lat_offset, target.entry_s, target.entry_length_m) == (
                lateral.chain, lateral.lat_offset, lateral.entry_s, lateral.entry_length_m)
        return checks
    raise ValueError('unknown expected behavior: ' + expect)


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [clean(item) for item in value]
    return value


def source_manifest():
    paths = sorted((PORT / 'src/ctrl_planning/ctrl_planning').rglob('*.py'))
    paths += sorted((PORT / 'src/ctrl_control/ctrl_control').rglob('*.py'))
    paths += [PORT / name for name in (
        'tools/probe_tunnel_contingencies.py', 'src/ctrl_planning/config/behavior.yaml',
        'src/ctrl_planning/config/missions.yaml', 'src/ctrl_bringup/routes/global_path.txt',
        'src/ctrl_bringup/maps/link_set.json', 'src/ctrl_bringup/routes/checkpoints.txt',
        'src/ctrl_ros1/ctrl_ros1/ctrl_packages.py')]
    return {str(path.relative_to(PORT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', choices=sorted(scenarios()), action='append')
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    before = source_manifest()
    runtime = Runtime()
    cases = scenarios()
    results = {name: runtime.run(cases[name]) for name in (args.case or cases)}
    unchanged = before == source_manifest()
    summary = dict(python=platform.python_version(), source_unchanged=unchanged,
        cases=len(results), frames=sum(len(rows) for rows in results.values()),
        passed=[name for name, rows in results.items() if all(row['passed'] for row in rows)],
        failed=[name for name, rows in results.items() if not all(row['passed'] for row in rows)],
        limitations=['Synthetic events and footprints, no LiDAR detection/classification',
            'Prescribed poses, no closed-loop vehicle, braking-distance or collision certification',
            'No ROS scheduling, UDP, camera, official penalty evaluation or localization integration'])
    for name, value in [('sources.json', before), ('cases.json', results), ('summary.json', summary)]:
        with (args.output / name).open('x') as stream:
            json.dump(clean(value), stream, indent=2, allow_nan=False)
            stream.write('\n')
    print(json.dumps(summary, indent=2))
    return 0 if unchanged and not summary['failed'] else 1


if __name__ == '__main__':
    sys.exit(main())
