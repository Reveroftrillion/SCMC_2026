"""Validate converted paths through the new progress and steering modules offline."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import uuid
import xml.etree.ElementTree as ET


WS = Path(__file__).resolve().parents[1]
REPO = WS.parent
PACKAGE = WS / 'src/ctrl_control'
sys.dont_write_bytecode = True
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(Path(__file__).parent))
from validate_lateral import integrate_bicycle  # noqa: E402
from ctrl_control.ctrl_lateral import LateralConfig, VehiclePose, compute_steering  # noqa: E402
from ctrl_control.ctrl_progress import ProgressConfig, ProgressTracker  # noqa: E402
from ctrl_control.ctrl_path_conversion import (  # noqa: E402
    StationedPath, build_offset_path, entry_return_moves, moves_from_chain,
)


def make_scene(case, sampling):
    origin = case['origin_m']
    base = StationedPath.from_global_route('base', 'synthetic_map', [(0, 0), (120, 0)],
                                          global_origin_m=origin)
    if case['kind'] == 'global':
        return base, lambda x: 0.0
    if case['kind'] == 'reference':
        def expected_y(x):
            return 1.5 * (1.0 - math.cos(2.0 * math.pi * x / 80.0)) if 0 <= x <= 80 else 0.0
        xs = [i * 0.25 for i in range(481)]
        route = StationedPath.from_reference('lane-reference', 'synthetic_map', xs,
                    [expected_y(x) for x in xs], [origin + 1.2 * x + 0.001 * x * x for x in xs])
        return route, expected_y
    if case['kind'] == 'preview':
        schedule = [(10.0, 12.0, case['offset_m']), (42.0, 12.0, 0.0)]
        moves = entry_return_moves(case['offset_m'], entry_start_s=origin + 10,
                                   entry_length_m=12, return_start_s=origin + 42, return_length_m=12)
    else:
        schedule = [(10.0, 10.0, 1.5), (30.0, 12.0, -1.5), (50.0, 12.0, 0.0)]
        moves = moves_from_chain([item[2] for item in schedule],
                                 [origin + item[0] for item in schedule], [item[1] for item in schedule])
    route = build_offset_path(base, moves, path_id=case['name'], initial_offset_m=0.0,
                               base_step_m=sampling['base_step_m'], max_offset_m=sampling['max_offset_m'],
                               max_points=sampling['max_points'])

    def expected_y(x):
        # Independent evaluation of the specified boundary-value polynomial,
        # using analytic x on a straight baseline rather than converted geometry.
        previous = 0.0
        for start, length, target in schedule:
            if x < start:
                return previous
            if x < start + length:
                t = (x - start) / length
                return previous + (target - previous) * (10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5)
            previous = target
        return previous
    return route, expected_y


def simulate(case, sampling, lateral, progress, dt, tail_s, trace):
    route, expected_y = make_scene(case, sampling)
    tracker = ProgressTracker(progress)
    x, y, yaw, previous = 0.0, case['initial_error_m'], 0.0, 0.0
    errors = [abs(y - expected_y(x))]
    maximum_angle = maximum_rate = roundtrip_error = 0.0
    global_maximum = route.global_stations_m[0]
    monotonic = True
    failure = None
    for index in range(round(case['duration_s'] / dt)):
        now = index * dt
        vehicle = VehiclePose(x, y, yaw, case['speed_mps'], now, 'synthetic_map')
        if not index:
            lower, upper = route.local_window(route.global_stations_m[0], route.global_stations_m[0] + 5.0)
            tracked = tracker.acquire(route.path, vehicle, now_s=now, reference_stamp_s=now,
                                      search_start_m=lower, search_end_m=upper)
        else:
            tracked = tracker.update(vehicle, now_s=now, reference_stamp_s=now)
        if not tracked.valid:
            failure = {'step': index, 'component': 'progress', 'reason': tracked.reason}
            break
        local_s = tracked.state.station_m
        global_s = route.global_at_local(local_s)
        roundtrip_error = max(roundtrip_error, abs(route.local_at_global(global_s) - local_s))
        new_global_maximum = route.global_at_local(tracked.state.max_station_m)
        monotonic = monotonic and new_global_maximum >= global_maximum
        global_maximum = new_global_maximum
        steering = compute_steering(vehicle, tracked.reference, lateral, now_s=now,
                                    dt_s=dt, previous_angle_rad=previous)
        if not steering.valid:
            failure = {'step': index, 'component': 'steering', 'reason': steering.reason}
            break
        maximum_angle = max(maximum_angle, abs(steering.angle_rad))
        maximum_rate = max(maximum_rate, abs(steering.angle_rad - previous) / dt)
        previous = steering.angle_rad
        x, y, yaw = integrate_bicycle(x, y, yaw, case['speed_mps'], previous, lateral.wheelbase_m, dt)
        errors.append(abs(y - expected_y(x)))
        trace.writerow((case['name'], (index + 1) * dt, x, y, previous, local_s, global_s,
                        global_maximum, route.global_at_local(steering.target_s_m), errors[-1]))
    tail = errors[-max(1, round(tail_s / dt)):]
    measured = dict(max_error_m=max(errors), tail_rms_m=math.sqrt(sum(x * x for x in tail) / len(tail)),
                    final_error_m=errors[-1], max_angle_rad=maximum_angle,
                    max_rate_rad_s=maximum_rate, max_station_roundtrip_error_m=roundtrip_error)
    gates = {name: measured[name] <= case[name] for name in ('max_error_m', 'tail_rms_m', 'final_error_m')}
    gates.update(completed=failure is None and len(errors) == round(case['duration_s'] / dt) + 1,
                 global_progress_monotonic=monotonic, station_roundtrip=roundtrip_error <= 1e-8,
                 angle_bound=maximum_angle <= lateral.max_angle_rad + 1e-12,
                 rate_bound=maximum_rate <= lateral.max_rate_rad_s + 1e-12)
    return dict(name=case['name'], passed=all(gates.values()), gates=gates, measured=measured,
                steps=len(errors) - 1, failure=failure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='New result directory only')
    args = parser.parse_args()
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%SZ')
    output = (args.output or WS / 'verification/.work' /
              ('independent-path-conversion-' + stamp + '-' + uuid.uuid4().hex[:8])).resolve()
    output.mkdir(parents=True, exist_ok=False)
    definition_path = Path(__file__).with_name('path_conversion_cases.json')
    definition = json.loads(definition_path.read_text(encoding='utf-8'))
    lateral_path = Path(__file__).with_name('lateral_cases.json')
    progress_path = Path(__file__).with_name('progress_cases.json')
    lateral_definition = json.loads(lateral_path.read_text(encoding='utf-8'))
    progress_definition = json.loads(progress_path.read_text(encoding='utf-8'))
    lateral = LateralConfig(**lateral_definition['config'])
    progress = ProgressConfig(**progress_definition['progress_config'])
    tests = [PACKAGE / 'test' / name for name in ('test_path_conversion.py', 'test_progress.py', 'test_lateral.py')]
    arguments = ['-q', '-p', 'no:cacheprovider', *map(str, tests), '--junitxml=' + str(output / 'unit.xml')]
    code = ('import sys; sys.path.insert(0,' + repr(str(PACKAGE)) + '); '
            'import pytest; raise SystemExit(pytest.main(' + repr(arguments) + '))')
    env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', PYTHONDONTWRITEBYTECODE='1', PYTHONUTF8='1')
    with (output / 'unit.log').open('w', encoding='utf-8') as stream:
        process = subprocess.run([sys.executable, '-X', 'utf8', '-B', '-c', code], cwd=REPO,
                                 env=env, stdout=stream, stderr=subprocess.STDOUT)
    counts = dict(tests=0, failures=0, errors=0, skipped=0)
    if (output / 'unit.xml').exists():
        for suite in ET.parse(output / 'unit.xml').getroot().iter('testsuite'):
            for key in counts:
                counts[key] += int(suite.attrib.get(key, 0))
    counts['exit_code'] = process.returncode
    print('Unit:', counts, flush=True)
    results = []
    with (output / 'trace.csv').open('w', encoding='utf-8', newline='') as stream:
        trace = csv.writer(stream)
        trace.writerow(('scenario', 'time_s', 'x_m', 'y_m', 'steering_rad', 'local_s_m',
                        'global_s_m', 'max_global_s_m', 'target_global_s_m', 'oracle_abs_error_m'))
        for case in definition['scenarios']:
            result = simulate(case, definition, lateral, progress, lateral_definition['dt_s'],
                              lateral_definition['tail_s'], trace)
            results.append(result)
            print(case['name'] + ': ' + ('PASS' if result['passed'] else 'FAIL'), flush=True)
    sources = [Path(__file__), definition_path, lateral_path, progress_path,
               Path(__file__).with_name('validate_lateral.py'), *tests,
               *[PACKAGE / 'ctrl_control' / name for name in ('ctrl_path_conversion.py', 'ctrl_progress.py', 'ctrl_lateral.py')],
               WS / 'docs/PATH_CONVERSION_CONTRACT_2026-10-25.md']
    imports = sorted(name for name in sys.modules if name.startswith(('ctrl_', 'rospy', 'rclpy')))
    expected = ['ctrl_control', 'ctrl_control.ctrl_lateral', 'ctrl_control.ctrl_path_conversion', 'ctrl_control.ctrl_progress']
    passed = (process.returncode == 0 and counts['tests'] > 0 and counts['skipped'] == 0
              and all(item['passed'] for item in results) and imports == expected)
    report = dict(schema_version=1, passed=passed, scope=definition['scope'],
                  recorded_at_utc=datetime.now(timezone.utc).isoformat(), unit=counts, scenarios=results,
                  project_or_ros_imports=imports,
                  source_sha256={p.relative_to(REPO).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                  limitations=['No ROS/catkin/MORAI execution or control packets',
                               'No road, collision, steering feasibility or sensor/actuator dynamics validation',
                               'Message lifecycle, mode conflicts and reference release not connected'])
    (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('Report:', output / 'report.json')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
