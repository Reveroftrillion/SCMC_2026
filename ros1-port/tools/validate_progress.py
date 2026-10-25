"""Offline progress + steering validation; every invocation creates new output."""

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
from validate_lateral import integrate_bicycle, make_scene  # noqa: E402
from ctrl_control.ctrl_lateral import LateralConfig, VehiclePose, compute_steering  # noqa: E402
from ctrl_control.ctrl_progress import ProgressConfig, ProgressTracker  # noqa: E402


def simulate(case, lateral, progress, dt, tail_s, trace):
    path, yaw, error = make_scene(case)
    tracker = ProgressTracker(progress)
    x, y, previous = 0.0, case['initial_error_m'], 0.0
    errors = [abs(error(x, y))]
    maximum_angle = maximum_rate = maximum_window = 0.0
    high_water = 0.0
    monotonic = True
    failure = None
    steps = round(case['duration_s'] / dt)
    for index in range(steps):
        now = index * dt
        vehicle = VehiclePose(x, y, yaw, case['speed_mps'], now, 'synthetic_map')
        if index == 0:
            tracked = tracker.acquire(path, vehicle, now_s=now, reference_stamp_s=now,
                                      search_start_m=0.0, search_end_m=path.length_m)
        else:
            tracked = tracker.update(vehicle, now_s=now, reference_stamp_s=now)
        if not tracked.valid:
            failure = {'step': index, 'component': 'progress', 'reason': tracked.reason}
            break
        monotonic = monotonic and tracked.state.max_station_m >= high_water
        high_water = tracked.state.max_station_m
        if index:
            maximum_window = max(maximum_window,
                                 tracked.reference.search_end_m - tracked.reference.search_start_m)
        steering = compute_steering(vehicle, tracked.reference, lateral, now_s=now,
                                    dt_s=dt, previous_angle_rad=previous)
        if not steering.valid:
            failure = {'step': index, 'component': 'steering', 'reason': steering.reason}
            break
        maximum_angle = max(maximum_angle, abs(steering.angle_rad))
        maximum_rate = max(maximum_rate, abs(steering.angle_rad - previous) / dt)
        previous = steering.angle_rad
        x, y, yaw = integrate_bicycle(x, y, yaw, case['speed_mps'], previous, lateral.wheelbase_m, dt)
        errors.append(abs(error(x, y)))
        trace.writerow((case['name'], (index + 1) * dt, x, y, yaw, previous,
                        tracked.state.station_m, high_water,
                        tracked.reference.search_start_m, tracked.reference.search_end_m, errors[-1]))
    tail = errors[-max(1, round(tail_s / dt)):]
    measured = dict(max_error_m=max(errors), tail_rms_m=math.sqrt(sum(x * x for x in tail) / len(tail)),
                    final_error_m=errors[-1], max_angle_rad=maximum_angle,
                    max_rate_rad_s=maximum_rate, max_update_window_m=maximum_window)
    gates = {name: measured[name] <= case[name]
             for name in ('max_error_m', 'tail_rms_m', 'final_error_m')}
    gates.update(completed=failure is None and len(errors) == steps + 1,
                 monotonic_progress=monotonic,
                 angle_bound=maximum_angle <= lateral.max_angle_rad + 1e-12,
                 rate_bound=maximum_rate <= lateral.max_rate_rad_s + 1e-12)
    return dict(name=case['name'], passed=all(gates.values()), gates=gates,
                measured=measured, steps=len(errors) - 1, failure=failure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='New directory, never an existing result folder')
    args = parser.parse_args()
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%SZ')
    output = (args.output or WS / 'verification/.work' /
              ('independent-progress-' + stamp + '-' + uuid.uuid4().hex[:8])).resolve()
    output.mkdir(parents=True, exist_ok=False)
    definition_path = Path(__file__).with_name('progress_cases.json')
    definition = json.loads(definition_path.read_text(encoding='utf-8'))
    lateral_path = definition_path.with_name(definition['lateral_cases'])
    cases = json.loads(lateral_path.read_text(encoding='utf-8'))
    lateral = LateralConfig(**cases['config'])
    progress = ProgressConfig(**definition['progress_config'])

    tests = [str(PACKAGE / 'test' / name) for name in ('test_progress.py', 'test_lateral.py')]
    arguments = ['-q', '-p', 'no:cacheprovider', *tests, '--junitxml=' + str(output / 'unit.xml')]
    code = ('import sys; sys.path.insert(0,' + repr(str(PACKAGE)) + '); '
            'import pytest; raise SystemExit(pytest.main(' + repr(arguments) + '))')
    env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', PYTHONDONTWRITEBYTECODE='1', PYTHONUTF8='1')
    with (output / 'unit.log').open('w', encoding='utf-8') as stream:
        process = subprocess.run([sys.executable, '-X', 'utf8', '-B', '-c', code],
                                 cwd=REPO, env=env, stdout=stream, stderr=subprocess.STDOUT)
    counts = dict(tests=0, failures=0, errors=0, skipped=0)
    if (output / 'unit.xml').exists():
        for suite in ET.parse(output / 'unit.xml').getroot().iter('testsuite'):
            for name in counts:
                counts[name] += int(suite.attrib.get(name, 0))
    counts['exit_code'] = process.returncode
    print('Unit:', counts, flush=True)
    results = []
    with (output / 'trace.csv').open('w', encoding='utf-8', newline='') as stream:
        trace = csv.writer(stream)
        trace.writerow(('scenario', 'time_s', 'x_m', 'y_m', 'yaw_rad', 'steering_rad',
                        'station_m', 'max_station_m', 'search_start_m', 'search_end_m', 'oracle_abs_error_m'))
        for case in cases['scenarios']:
            result = simulate(case, lateral, progress, cases['dt_s'], cases['tail_s'], trace)
            results.append(result)
            print(case['name'] + ': ' + ('PASS' if result['passed'] else 'FAIL'), flush=True)
    source_paths = [Path(__file__), definition_path, lateral_path,
                    Path(__file__).with_name('validate_lateral.py'),
                    PACKAGE / 'ctrl_control/ctrl_lateral.py', PACKAGE / 'ctrl_control/ctrl_progress.py',
                    PACKAGE / 'test/test_progress.py', PACKAGE / 'test/test_lateral.py',
                    WS / 'docs/PROGRESS_CONTRACT_2026-10-25.md']
    imports = sorted(name for name in sys.modules if name.startswith(('ctrl_', 'rospy', 'rclpy')))
    passed = (process.returncode == 0 and counts['tests'] > 0 and counts['skipped'] == 0
              and all(result['passed'] for result in results)
              and imports == ['ctrl_control', 'ctrl_control.ctrl_lateral', 'ctrl_control.ctrl_progress'])
    report = dict(schema_version=1, passed=passed, scope=definition['scope'],
                  recorded_at_utc=datetime.now(timezone.utc).isoformat(), unit=counts,
                  scenarios=results, config=definition['progress_config'], dt_s=cases['dt_s'],
                  project_or_ros_imports=imports,
                  source_sha256={path.relative_to(REPO).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
                                 for path in source_paths},
                  limitations=['Synthetic bicycle only; no actuator delay/noise/collision model',
                               'No ROS/catkin/MORAI execution or control packets',
                               'Global station/lap mapping and production route conversion remain pending'])
    (output / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print('Report:', output / 'report.json')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
