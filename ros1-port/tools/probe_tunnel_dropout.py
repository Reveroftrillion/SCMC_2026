#!/usr/bin/env python3
"""Compare single-outline/dropout/reappearance sequences on prescribed poses.

No ROS, raw-sensor simulation or vehicle dynamics. A non-bypass initial target
is reported as ineligible, not counted as a successful avoidance experiment.
"""
import argparse
import hashlib
import importlib.util
from itertools import product
import json
from pathlib import Path
import sys

import probe_tunnel_matrix as matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--baseline-avoid', type=Path,
                        help='Read an immutable isolated pre-change ctrl_avoid.py; other modules unchanged')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    source = matrix.base.PORT/'src/ctrl_planning/ctrl_planning/states/ctrl_avoid.py'
    if args.baseline_avoid:
        source = args.baseline_avoid
        name = 'ctrl_planning.states._dropout_baseline'
        spec = importlib.util.spec_from_file_location(name, source)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        matrix.base.Avoid = module.Avoid
    source_digest = hashlib.sha256(source.read_bytes()).hexdigest()
    runtime = matrix.base.Runtime()
    records = []
    with (args.output/'cases.jsonl').open('x') as stream:
        for offset, width, kmh, misses, distance in product(
                [-1., 0., 1.], [1., 2.1], [0., 20., 50.], [1, 4, 8], [25., 35.]):
            body = matrix.base.Body(d=offset, width=width, length=.1, height=.42)
            start = body.s-distance
            frames = [matrix.base.Frame(1., (body,), s=start, speed=kmh/3.6)]
            for i in range(1, misses+2):
                frames.append(matrix.base.Frame(1.+i*.1, () if i <= misses else (body,),
                    s=start+kmh/3.6*i*.1, speed=kmh/3.6))
            rows = matrix.run(runtime, 'static', frames)
            initial = rows[0]['target']
            eligible = initial['v_max'] > 0. and bool(initial['chain'])
            retained = []
            if eligible:
                for row in rows[1:-1]:
                    expected = tuple(move for move in initial['chain']
                                     if move[1]+move[2] > row['input']['s'])
                    retained.append(row['target']['state'] == 'AVOID'
                                    and tuple(row['target']['chain']) == expected)
            record = dict(offset=offset, width=width, kmh=kmh, misses=misses, distance=distance,
                eligible=eligible, retained_during_dropout=None if not eligible else all(retained),
                decision_checks_passed=all(r['passed'] for r in rows), rows=rows)
            stream.write(json.dumps(matrix.base.clean(record), allow_nan=False)+'\n')
            records.append({k: v for k, v in record.items() if k != 'rows'})
    summary = dict(scope='Prescribed map-only single-outline/dropout/reappearance; no physical dynamics',
        cases=len(records), eligible=sum(r['eligible'] for r in records),
        retained=sum(r['retained_during_dropout'] is True for r in records),
        ineligible=sum(not r['eligible'] for r in records),
        failures=[r for r in records if r['eligible'] and not r['retained_during_dropout']],
        other_decision_failures=[r for r in records if not r['decision_checks_passed']],
        avoid_source_sha256=source_digest, avoid_source=str(source),
        source_unchanged=hashlib.sha256(source.read_bytes()).hexdigest() == source_digest,
        limitations=['Ego progresses at prescribed input speed, independent of output caps',
                     'No native wall corridor, raw returns, transport scheduling, braking or steering response',
                     'Retained curves may stop under guards; retention is not mobility or collision certification'])
    with (args.output/'summary.json').open('x') as stream:
        json.dump(summary, stream, indent=2, allow_nan=False)
    print(json.dumps({k: v for k, v in summary.items() if k not in ('failures', 'other_decision_failures')}))


if __name__ == '__main__':
    main()
