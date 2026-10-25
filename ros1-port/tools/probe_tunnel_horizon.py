#!/usr/bin/env python3
"""Locate the observed two-body planning boundary using independent fresh poses.

Each prescribed pose starts a fresh behavior. This does NOT replay a committed
curve, prove braking distance, or establish closed-loop safety at the boundary.
"""
import argparse
from itertools import product
import json
from pathlib import Path

import probe_tunnel_matrix as matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    sources = matrix.base.source_manifest()
    for name in ['probe_tunnel_matrix.py', 'probe_tunnel_horizon.py']:
        path = Path(__file__).with_name(name)
        sources[str(path.relative_to(matrix.base.PORT))] = matrix.base.hashlib.sha256(path.read_bytes()).hexdigest()
    runtime = matrix.base.Runtime()
    records = []
    for gap, kmh, station in product([20., 30.], [0., 20., 50.],
                                   [1880., 1885., 1890., 1895., 1900.]):
        bodies = (matrix.base.Body(key='first', s=1905., d=-1., width=1.),
                  matrix.base.Body(key='second', s=1905.+gap, d=1., width=1.))
        frame = matrix.base.Frame(1., bodies, s=station, speed=kmh/3.6)
        row = matrix.run(runtime, 'static', [frame])[0]
        records.append(dict(gap=gap, kmh=kmh, station=station, **row))
    summary = dict(scope='Fresh independent map-only poses; no committed-curve or physical replan validation',
                   cases=len(records), passed=sum(row['passed'] for row in records),
                   failed=[dict(gap=r['gap'], kmh=r['kmh'], station=r['station'],
                                hit=r['sampled_overlap']) for r in records if not r['passed']],
                   limitations=['Each fresh pose starts at d=0; temporal replanning is untested',
                                'Whole returned path checked beyond current BEV horizon',
                                'Zero speed and finite stopping targets do not prove physical braking'])
    for name, data in [('summary.json', summary), ('cases.json', records), ('sources.json', sources)]:
        with (args.output/name).open('x') as stream:
            json.dump(matrix.base.clean(data), stream, indent=2, allow_nan=False)
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
