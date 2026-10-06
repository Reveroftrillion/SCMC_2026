#!/usr/bin/env python3
"""Compare Local candidate outcomes; never chooses or writes optimal runtime parameters."""
import argparse
import copy
import itertools
import math
from pathlib import Path
import yaml
from local_planner_tools import DEFAULT_CONFIG, load_document, planner_config, finite_number, protect_outputs, write_csv
from local_planner_scenarios import scenarios, evaluate_scenario, RESULT_FIELDS

DEFAULT_GRID = {
    'safety_margin': [.1, .3, .6],
    'offsets': [[-2., 0., 2.], [-3., -2., 0., 2., 3.]],
    'transition_length': [8., 12., 16.],
    'return_length': [8., 12., 16.],
    'horizon': [20., 26., 34.],
    'road_half_width': [2., 3., 4.5],
    'max_curvature': [.06, .13, .2],
    'max_steering_rate_per_m': [.08, .25, .5],
}


def variants(config, grid, mode='one-at-a-time', max_runs=500):
    if not isinstance(grid, dict) or not grid or set(grid)-set(DEFAULT_GRID):
        raise ValueError('grid must contain supported tuning keys: ' + ', '.join(DEFAULT_GRID))
    for key, values in grid.items():
        if not isinstance(values, list) or not values:
            raise ValueError(key + ': requires a nonempty list of values')
        if key == 'offsets':
            if not all(isinstance(v, list) and v and all(finite_number(x) for x in v) for v in values):
                raise ValueError('offsets grid requires lists of finite numbers')
        elif not all(finite_number(v) for v in values):
            raise ValueError(key + ': grid values must be finite numbers')
    count = 1 + (sum(map(len, grid.values())) if mode == 'one-at-a-time' else math.prod(map(len, grid.values())))
    if count > max_runs:
        raise ValueError('%d variants exceed --max-runs %d; narrow the grid' % (count, max_runs))
    yield 'baseline', copy.deepcopy(config), ''
    if mode == 'one-at-a-time':
        for key, values in grid.items():
            for value in values:
                p = copy.deepcopy(config)
                p[key] = copy.deepcopy(value)
                yield key, p, repr(value)
    else:
        keys = list(grid)
        for combination in itertools.product(*(grid[k] for k in keys)):
            p = copy.deepcopy(config)
            p.update(zip(keys, copy.deepcopy(combination)))
            yield 'cartesian', p, repr(dict(zip(keys, combination)))


def classify(row, config, clearance_warning):
    if row['result'] == 'CONFIG_ERROR': return 'CONFIG_ERROR'
    if row['state'] == 'HOLD': return 'HOLD'
    if row['valid_candidates'] == 0: return 'ALL_REJECTED'
    flags = []
    clearance = row.get('min_clearance_m')
    if finite_number(clearance) and clearance < clearance_warning: flags.append('LOW_CLEARANCE')
    if row.get('selected_peak_curvature', 0) >= .8*config['max_curvature']: flags.append('NEAR_CURVATURE_LIMIT')
    if row.get('selected_peak_steering_rate_per_m', 0) >= .8*config['max_steering_rate_per_m']: flags.append('NEAR_STEERING_RATE_LIMIT')
    return ';'.join(flags) or 'OK'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--grid', type=Path, help='YAML mapping key -> values; offsets -> lists of offsets')
    parser.add_argument('--mode', choices=['one-at-a-time','cartesian'], default='one-at-a-time')
    parser.add_argument('--scenario', action='append', choices=[s.name for s in scenarios()])
    parser.add_argument('--max-runs', type=int, default=500, help='cap parameter variants before scenario multiplication')
    parser.add_argument('--clearance-warning', type=float, default=.25, help='report threshold in metres, not a collision rule')
    parser.add_argument('--csv', type=Path)
    args = parser.parse_args(argv)
    try:
        protect_outputs([args.csv], [args.config] + ([args.grid] if args.grid else []))
        if not math.isfinite(args.clearance_warning) or args.clearance_warning < 0 or args.max_runs < 1:
            raise ValueError('invalid clearance threshold or max-runs')
        config = planner_config(args.config)
        grid = load_document(args.grid) if args.grid else DEFAULT_GRID
        chosen = args.scenario or ['center_obstacle','all_blocked','curved_road']
        rows = []
        for variant, p, value in variants(config, grid, args.mode, args.max_runs):
            for scenario in scenarios():
                if scenario.name not in chosen: continue
                # The swept corridor must not be overwritten by a fixture's narrow corridor.
                for row in evaluate_scenario(scenario, p, check_expectations=False, apply_updates=False):
                    row.update(variant=variant, value=value, assessment=classify(row,p,args.clearance_warning))
                    row.update({k:repr(p[k]) for k in DEFAULT_GRID})
                    rows.append(row)
                    print('%s=%s | %s/%s | %s | valid=%s offset=%s clearance=%s curvature=%s rate=%s rejected=%s reason=%s' % (
                        variant,value,scenario.name,row['frame'],row['assessment'],row['valid_candidates'],
                        row.get('selected_offset',''),row.get('min_clearance_m',''),row.get('selected_peak_curvature',''),
                        row.get('selected_peak_steering_rate_per_m',''),row.get('rejections',''),row.get('reason','')))
        if args.csv:
            write_csv(args.csv, rows, ['variant','value','assessment']+list(DEFAULT_GRID)+RESULT_FIELDS)
            print('Report: ' + str(args.csv))
        print('SUMMARY: %d rows; infeasible configurations are reported, no runtime YAML modified.' % len(rows))
        return 0
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
        print('ERROR: ' + str(exc))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
