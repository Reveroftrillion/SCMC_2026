#!/usr/bin/env python3
"""Run deterministic Local core scenarios without ROS/MORAI. Exit 1 on expectation failure."""
import argparse
from pathlib import Path
import yaml
from local_planner_tools import DEFAULT_CONFIG, planner_config, protect_outputs, write_csv
from local_planner_scenarios import scenarios, evaluate_scenario, display, RESULT_FIELDS


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--scenario', action='append', choices=[s.name for s in scenarios()], help='repeat to select; default all')
    parser.add_argument('--csv', type=Path)
    args = parser.parse_args(argv)
    try:
        protect_outputs([args.csv], [args.config])
        config = planner_config(args.config)
        rows = [r for s in scenarios() if not args.scenario or s.name in args.scenario for r in evaluate_scenario(s, config)]
        display(rows)
        print('Expected states are offline harness results, not ROS/MORAI validation. inf clearance means no obstacles.')
        failures = sum(r['result'] != 'PASS' for r in rows)
        print('SUMMARY: %d scenarios, %d frames, %d failures' % (len({r['scenario'] for r in rows}), len(rows), failures))
        if args.csv:
            write_csv(args.csv, rows, RESULT_FIELDS)
            print('Report: ' + str(args.csv))
        return 1 if failures else 0
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
        print('ERROR: ' + str(exc))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
