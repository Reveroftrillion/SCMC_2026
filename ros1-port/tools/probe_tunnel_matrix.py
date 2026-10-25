#!/usr/bin/env python3
"""Batch prescribed tunnel decisions and an independent sampled Frenet-body check.

No ROS, UDP or dynamics. A stop is counted separately from a traversable bypass.
The Frenet rectangle approximation is a diagnostic, not physical clearance proof.
"""
import argparse
from dataclasses import asdict
from itertools import product
import json
import math
from pathlib import Path
import time

import numpy as np
import probe_tunnel_contingencies as base
from ctrl_planning.ctrl_mission import MissionSupervisor
from ctrl_control.ctrl_core import FRONT_AXLE


def overlap(a, b):
    a, b = np.asarray(a), np.asarray(b)
    for polygon in (a, b):
        for delta in np.roll(polygon, -1, axis=0) - polygon:
            axis = np.array([-delta[1], delta[0]])
            if np.linalg.norm(axis) < 1e-12:
                continue
            aa, bb = a @ axis, b @ axis
            if aa.max() < bb.min()-1e-9 or bb.max() < aa.min()-1e-9:
                return False
    return True


def lateral(target, station):
    moves = list(target.chain)
    if not moves and target.entry_s is not None:
        moves = [(target.lat_offset, target.entry_s, target.entry_length_m)]
        if target.return_s is not None:
            moves.append((0., target.return_s, target.return_length_m))
    previous = 0.
    for offset, start, length in moves:
        if station < start:
            return previous, 0.
        if length > 0. and station < start+length:
            u = (station-start)/length
            return (previous+(offset-previous)*(10*u**3-15*u**4+6*u**5),
                    (offset-previous)*30*u*u*(1-u)**2/length)
        previous = offset
    return previous, 0.


def sampled_overlap(target, frame, bodies, width, front, rear):
    end = max(b.s+b.length/2.+rear for b in bodies)
    # Target.stop_s is the front-wheel station, while frame.s is the rear axle.
    # Check the entire commanded segment including its exact stopping endpoint.
    # This checks geometry only: finite stop_s does not prove timely braking.
    if math.isfinite(target.stop_s):
        end = min(end, max(frame.s, target.stop_s-FRONT_AXLE))
    for station in np.r_[np.arange(frame.s, end, .25), end]:
        d, slope = lateral(target, station)
        angle = math.atan(slope)
        tangent = np.array([math.cos(angle), math.sin(angle)])
        normal = np.array([-tangent[1], tangent[0]])
        car = np.array([np.array([station, d])+x*tangent+y*normal
                        for x, y in [(-rear, -width/2), (front, -width/2),
                                     (front, width/2), (-rear, width/2)]])
        for body in bodies:
            box = [[body.s+x*body.length/2., body.d+y*body.width/2.]
                   for x, y in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]
            if overlap(car, box):
                return dict(s=float(station), d=d, body=body.key)
    return None


def cases(dense=False):
    for station, offset, width, length, kmh in product(
            [1905., 1920.], [-1., 0., 1.], [1., 1.8], [1., 3.], [0., 20., 50.]):
        body = base.Body(s=station, d=offset, width=width, length=length)
        yield 'static-s%g-d%g-w%g-l%g-v%g' % (station, offset, width, length, kmh), 'static', [
            base.Frame(1., (body,), s=station-25., speed=kmh/3.6)]
    for station, kmh, mode in product([1870., 1880., 1920., 1960.], [0., 20., 50.], ['DR', 'GPS']):
        yield 'clear-s%g-v%g-%s' % (station, kmh, mode), 'move', [
            base.Frame(1., s=station, speed=kmh/3.6, loc_mode=mode)]
    for distance, kmh, length in product([8., 15., 25.], [0., 20., 50.], [1., 3.]):
        body = base.Body(s=1880.+distance, width=20., length=length)
        yield 'blocked-gap%g-v%g-l%g' % (distance, kmh, length), 'stop', [
            base.Frame(1., (body,), speed=kmh/3.6)]
    for distance, offset, kmh in product([10., 20.], [-1., 0., 1.], [0., 20., 50.]):
        person = base.Body(key='person', kind='PEDESTRIAN', s=1880.+distance,
                           d=offset, length=.6, width=.6, height=1.7)
        yield 'person-reenter-gap%g-d%g-v%g' % (distance, offset, kmh), 'reenter', [
            base.Frame(1., (person,), speed=kmh/3.6),
            base.Frame(1.1, (person,), speed=kmh/3.6),
            base.Frame(1.2, cleared=({'id': person.key, 'last_seen': 1.1},)),
            base.Frame(1.3, (person,), speed=kmh/3.6)]
    for offset, kmh in product([-1., 0., 1.], [0., 20., 50.]):
        body = base.Body(d=offset, width=1.)
        yield 'stale-restored-d%g-v%g' % (offset, kmh), 'stale', [
            base.Frame(1., (body,), speed=kmh/3.6),
            base.Frame(1.1, (body,), speed=kmh/3.6),
            base.Frame(2., (body,), speed=kmh/3.6, scan_stamp=1.1),
            base.Frame(2.1, (body,), speed=kmh/3.6)]
    for gap, first_d, kmh in product([12., 20., 30.], [-1., 1.], [0., 20., 50.]):
        bodies = (base.Body(key='first', d=first_d, width=1.),
                  base.Body(key='second', s=1905.+gap, d=-first_d, width=1.))
        yield 'two-gap%g-d%g-v%g' % (gap, first_d, kmh), 'static', [
            base.Frame(1., bodies, speed=kmh/3.6)]
    # Both outlines are now within 40m. Preserve the earlier far-outline cases
    # too: their whole-target overlaps must remain visible as diagnostics.
    for distance, gap, first_d, kmh in product(
            [15., 20., 25.], [6., 10., 14.], [-1., 0., 1.], [0., 20., 50.]):
        bodies = (base.Body(key='first', s=1880.+distance, d=first_d, width=1.),
                  base.Body(key='second', s=1880.+distance+gap, d=-first_d, width=1.))
        yield 'near-two-dist%g-gap%g-d%g-v%g' % (distance, gap, first_d, kmh), 'static', [
            base.Frame(1., bodies, speed=kmh/3.6)]
    for distance, offset, kmh in product([8., 15., 25.], [-1., 0., 1.], [0., 20., 50.]):
        yield 'missing-map-dist%g-d%g-v%g' % (distance, offset, kmh), 'stop', [
            base.Frame(1., (base.Body(s=1880.+distance, d=offset, width=1.),),
                       speed=kmh/3.6, mapped=False)]
    if dense:
        for station, offset, width, length, kmh in product(
                [1885., 1905., 1930., 1955.], [-2., -1., 0., 1., 2.],
                [.5, 1., 2.], [.5, 1., 3.], [0., 10., 20., 35., 50.]):
            body = base.Body(s=station, d=offset, width=width, length=length)
            yield 'dense-s%g-d%g-w%g-l%g-v%g' % (station, offset, width, length, kmh), 'static', [
                base.Frame(1., (body,), s=station-25., speed=kmh/3.6)]


def run(runtime, contract, frames):
    behavior, mission = runtime.behavior(), MissionSupervisor(runtime.missions)
    rows = []
    for index, frame in enumerate(frames):
        ctx = runtime.context(frame)
        if mission.update(ctx):
            behavior.reset()
        yielding = next(st for st in behavior.states if st.name == 'YIELD')
        ctx.yield_left_key = yielding.left_key if not behavior.engaged['YIELD'] else None
        mission.apply_context(ctx)
        target = mission.limit(behavior.tick(ctx, allowed_states=mission.policy.allowed_states))
        avoid = next(st for st in behavior.states if st.name == 'AVOID')
        checks = dict(finite_speed=math.isfinite(target.v_max) and target.v_max >= 0.,
                      mission_cap=target.v_max <= ctx.mission_speed_mps+1e-9)
        hit = None
        if contract == 'move':
            checks.update(base.evaluate('move', target, behavior.last_targets))
        elif contract == 'stop' or (contract == 'stale' and index == 2):
            checks.update(base.evaluate('stop', target, behavior.last_targets))
        elif contract == 'reenter' and index != 2:
            checks.update(base.evaluate('yield', target, behavior.last_targets))
        elif frame.bodies and target.v_max > 0.:
            hit = sampled_overlap(target, frame, frame.bodies, avoid.car_width_m,
                                  avoid.front_m, avoid.car_length_m-avoid.front_m)
            checks['sampled_frenet_body_clear'] = hit is None
        outcome = ('zero_speed_cap' if target.v_max == 0. else
                   'braking_target' if math.isfinite(target.stop_s) else
                   'bypass' if target.chain or abs(target.lat_offset)>.1 else 'moving')
        rows.append(dict(input=asdict(frame), target=asdict(target), outcome=outcome,
                         checks=checks, passed=all(checks.values()), sampled_overlap=hit,
                         bev_reason=avoid.bev.last_reason if avoid.bev else None,
                         planning_boxes=[asdict(box) for box in avoid.boxes()],
                         planning_horizon_m=avoid.horizon_m))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--dense', action='store_true', help='Add 900 station/width/length/offset/speed combinations')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before = base.source_manifest()
    before[str(Path(__file__).relative_to(base.PORT))] = base.hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    runtime = base.Runtime()
    records, started = [], time.monotonic()
    with (args.output/'cases.jsonl').open('x') as stream:
        for name, contract, frames in cases(args.dense):
            rows = run(runtime, contract, frames)
            record = dict(name=name, contract=contract, passed=all(r['passed'] for r in rows), rows=rows)
            stream.write(json.dumps(base.clean(record), allow_nan=False)+'\n')
            stream.flush()
            records.append(dict(name=name, contract=contract, passed=record['passed'],
                                outcomes=[r['outcome'] for r in rows]))
            if len(records) % 25 == 0:
                print(json.dumps(dict(done=len(records), elapsed_s=time.monotonic()-started)), flush=True)
    after = base.source_manifest()
    after[str(Path(__file__).relative_to(base.PORT))] = base.hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    result = dict(scope='Map-only synthetic calculation matrix, no raw sensors/ROS/dynamics/control',
        cases=len(records), frames=sum(len(r['outcomes']) for r in records),
        passed=sum(r['passed'] for r in records), failed=[r for r in records if not r['passed']],
        results=records, elapsed_s=time.monotonic()-started, source_unchanged=before==after,
        limitations=['Stops and bypasses are separate outcomes; stop is not a mobility pass',
            'Finite stop_s truncates at stop_s - FRONT_AXLE; no braking-distance proof',
            'Sampled Frenet rectangles approximate body orientation; whole output path checked even beyond sensing horizon',
            'Synthetic person semantics/position, no perception/classification or official scoring'])
    for name, value in [('summary.json', result), ('sources.json', before)]:
        with (args.output/name).open('x') as stream:
            json.dump(base.clean(value), stream, indent=2, allow_nan=False)
    print(json.dumps({k: v for k, v in result.items() if k not in ('results', 'failed')}), flush=True)


if __name__ == '__main__':
    main()
