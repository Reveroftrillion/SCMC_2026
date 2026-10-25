"""Request lifecycle validation, including bounded progress and steering offline."""

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
from ctrl_control.ctrl_path_conversion import StationedPath  # noqa: E402
from ctrl_control.ctrl_path_selection import PathRequest, PathSelector, SelectionConfig  # noqa: E402


def simulate(case, definition, lateral, progress, trace):
    dt = definition['dt_s']
    base = StationedPath.from_global_route('base', 'map', [(0, 0), (60, 0)], global_origin_m=500)
    selector = PathSelector(base, SelectionConfig(**definition['selection_config']))
    tracker = ProgressTracker(progress)
    x, y, yaw, angle = 0.0, case['initial_error_m'], 0.0, 0.0
    global_progress, revision = 500.0, None
    modes, references = [], set()
    errors = [abs(y)]
    maximum_rate = maximum_angle = 0.0
    released = False
    failure = None

    def expected_y(position):
        if case['mode'] == 'reference':
            return 0.75 * (1 - math.cos(math.pi * position / 20)) if 0 <= position <= 40 else 0.0
        if position < 10 or position >= 40:
            return 0.0
        if position >= 20 and position < 30:
            return 1.5
        u = (position - (10 if position < 20 else 30)) / 10
        blend = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5
        return 1.5 * (blend if position < 20 else 1 - blend)

    xs = tuple(i * 0.25 for i in range(241))
    ys = tuple(expected_y(value) for value in xs)
    for index in range(round(case['duration_s'] / dt)):
        now = index * dt
        if index % definition['heartbeat_steps'] == 0:
            released = released or global_progress >= case['release_global_s']
            if released:
                request = PathRequest(now, 'map', reference_release=case['mode'] == 'reference')
            elif case['mode'] == 'offset':
                request = PathRequest(now, 'map', lat_offset_m=1.5,
                                      entry_s=510, entry_length_m=10, return_s=530, return_length_m=10)
            elif index == 0:
                request = PathRequest(now, 'map', reference_id='lane', reference_x=xs, reference_y=ys,
                                      reference_global_s=tuple(500 + value for value in xs))
            else:
                request = PathRequest(now, 'map')  # Explicitly keep active reference.
            result = selector.submit(request, now_s=now,
                                      progress_global_s=global_progress if released else None)
            if not result.valid:
                failure = {'step': index, 'component': 'selection', 'reason': result.reason}
                break
        selected = selector.current(now_s=now)
        if not selected.valid:
            failure = {'step': index, 'component': 'current', 'reason': selected.reason}
            break
        state = selected.state
        references.add(id(state.path))
        if not modes or modes[-1] != state.mode:
            modes.append(state.mode)
        pose = VehiclePose(x, y, yaw, case['speed_mps'], now, 'map')
        if revision != state.revision:
            lower, upper = state.path.local_window(max(500, global_progress - 2), min(560, global_progress + 2))
            tracked = tracker.acquire(state.path.path, pose, now_s=now,
                        reference_stamp_s=state.request_stamp_s, search_start_m=lower, search_end_m=upper)
        else:
            tracked = tracker.update(pose, now_s=now, reference_stamp_s=state.request_stamp_s)
        if not tracked.valid:
            failure = {'step': index, 'component': 'progress', 'reason': tracked.reason}
            break
        revision = state.revision
        global_progress = state.path.global_at_local(tracked.state.max_station_m)
        steering = compute_steering(pose, tracked.reference, lateral, now_s=now,
                                    dt_s=dt, previous_angle_rad=angle)
        if not steering.valid:
            failure = {'step': index, 'component': 'steering', 'reason': steering.reason}
            break
        maximum_rate = max(maximum_rate, abs(steering.angle_rad - angle) / dt)
        angle = steering.angle_rad
        maximum_angle = max(maximum_angle, abs(angle))
        x, y, yaw = integrate_bicycle(x, y, yaw, case['speed_mps'], angle, lateral.wheelbase_m, dt)
        errors.append(abs(y - expected_y(x)))
        trace.writerow((case['name'], (index + 1) * dt, state.mode, revision, global_progress,
                        x, y, angle, errors[-1], state.request_stamp_s))
    tail = errors[-max(1, round(definition['tail_s'] / dt)):]
    measured = dict(max_error_m=max(errors), tail_rms_m=math.sqrt(sum(e * e for e in tail) / len(tail)),
                    final_error_m=errors[-1], max_angle_rad=maximum_angle,
                    max_rate_rad_s=maximum_rate, geometry_objects=len(references))
    gates = {key: measured[key] <= case[key] for key in ('max_error_m', 'tail_rms_m', 'final_error_m')}
    gates.update(completed=failure is None and len(errors) == round(case['duration_s'] / dt) + 1,
                 modes=modes == [case['mode'], 'global'], stable_geometry=len(references) == 2,
                 angle_bound=maximum_angle <= lateral.max_angle_rad + 1e-12,
                 rate_bound=maximum_rate <= lateral.max_rate_rad_s + 1e-12)
    return dict(name=case['name'], passed=all(gates.values()), measured=measured,
                gates=gates, modes=modes, steps=len(errors) - 1, failure=failure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='A new result directory only')
    args = parser.parse_args()
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%SZ')
    output = (args.output or WS / 'verification/.work' /
              ('independent-path-selection-' + stamp + '-' + uuid.uuid4().hex[:8])).resolve()
    output.mkdir(parents=True, exist_ok=False)
    tools = Path(__file__).parent
    definition = json.loads((tools / 'path_selection_cases.json').read_text(encoding='utf-8'))
    lateral = LateralConfig(**json.loads((tools / 'lateral_cases.json').read_text(encoding='utf-8'))['config'])
    progress = ProgressConfig(**json.loads((tools / 'progress_cases.json').read_text(encoding='utf-8'))['progress_config'])
    tests = [PACKAGE / 'test' / name for name in ('test_path_selection.py', 'test_path_conversion.py', 'test_progress.py', 'test_lateral.py')]
    arguments = ['-q', '-p', 'no:cacheprovider', *map(str, tests), '--junitxml=' + str(output / 'unit.xml')]
    code = ('import sys; sys.path.insert(0,' + repr(str(PACKAGE)) + '); '
            'import pytest; raise SystemExit(pytest.main(' + repr(arguments) + '))')
    env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', PYTHONDONTWRITEBYTECODE='1', PYTHONUTF8='1')
    with (output / 'unit.log').open('w', encoding='utf-8') as stream:
        process = subprocess.run([sys.executable, '-X', 'utf8', '-B', '-c', code], cwd=REPO,
                                 env=env, stdout=stream, stderr=subprocess.STDOUT)
    counts = dict(tests=0, failures=0, errors=0, skipped=0)
    if (output / 'unit.xml').is_file():
        for suite in ET.parse(output / 'unit.xml').getroot().iter('testsuite'):
            for key in counts:
                counts[key] += int(suite.attrib.get(key, 0))
    counts['exit_code'] = process.returncode
    print('Unit:', counts, flush=True)
    results = []
    with (output / 'trace.csv').open('w', encoding='utf-8', newline='') as stream:
        trace = csv.writer(stream)
        trace.writerow(('scenario', 'time_s', 'mode', 'revision', 'global_s', 'x', 'y',
                        'steering_rad', 'oracle_abs_error_m', 'request_stamp_s'))
        for case in definition['scenarios']:
            result = simulate(case, definition, lateral, progress, trace)
            results.append(result)
            print(case['name'] + ': ' + ('PASS' if result['passed'] else 'FAIL'), flush=True)
    sources = [Path(__file__), tools / 'path_selection_cases.json', tools / 'lateral_cases.json',
               tools / 'progress_cases.json', tools / 'validate_lateral.py', *tests,
               *[PACKAGE / 'ctrl_control' / name for name in ('ctrl_path_selection.py', 'ctrl_path_conversion.py',
                                                             'ctrl_progress.py', 'ctrl_lateral.py')],
               WS / 'docs/PATH_SELECTION_CONTRACT_2026-10-25.md']
    imports = sorted(name for name in sys.modules if name.startswith(('ctrl_', 'rospy', 'rclpy')))
    expected = ['ctrl_control', 'ctrl_control.ctrl_lateral', 'ctrl_control.ctrl_path_conversion',
                'ctrl_control.ctrl_path_selection', 'ctrl_control.ctrl_progress']
    passed = (process.returncode == 0 and counts['tests'] > 0 and counts['skipped'] == 0
              and all(item['passed'] for item in results) and imports == expected)
    report = dict(schema_version=1, passed=passed, scope=definition['scope'],
                  recorded_at_utc=datetime.now(timezone.utc).isoformat(), unit=counts, scenarios=results,
                  project_or_ros_imports=imports,
                  source_sha256={p.relative_to(REPO).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                  limitations=['No production ROS wiring, control packets or MORAI driving',
                               'No sensor noise, actuator delay, collision or road constraints',
                               'Fail-closed results require longitudinal stop/handoff integration'])
    (output / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print('Report:', output / 'report.json')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
