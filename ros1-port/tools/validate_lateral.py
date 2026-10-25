"""Run the independent steering checks in a new, non-overwriting result folder.

This runs only offline tests and a synthetic bicycle model. It does not load
the production controller, open a ROS connection, or send control packets.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET


WS = Path(__file__).resolve().parents[1]
REPO = WS.parent
PACKAGE = WS / 'src/ctrl_control'
sys.dont_write_bytecode = True
sys.path.insert(0, str(PACKAGE))
from ctrl_control.ctrl_lateral import (  # noqa: E402
    LateralConfig, Polyline, TrackingReference, VehiclePose, compute_steering,
)


def make_scene(case):
    """Generate analytic test geometry and an error oracle outside the tracker."""
    travel = case['speed_mps'] * case['duration_s']
    kind = case['kind']
    if kind == 'straight':
        points = ((-20.0, 0.0), (travel + 50.0, 0.0))
        yaw = 0.0

        def error(x, y):
            return y
    elif kind == 'circle':
        radius, direction = case['radius_m'], case['direction']
        points = tuple((radius * math.sin(i * 0.01),
                        direction * radius * (1.0 - math.cos(i * 0.01)))
                       for i in range(472))
        yaw = 0.0

        def error(x, y):
            return math.hypot(x, y - direction * radius) - radius
    elif kind == 'sine':
        amplitude = case['amplitude_m']
        wave_number = 2.0 * math.pi / case['wavelength_m']
        xs = [-20.0 + i * 0.25 for i in range(math.ceil((travel + 70.0) / 0.25) + 1)]
        points = tuple((x, amplitude * math.sin(wave_number * x)) for x in xs)
        yaw = math.atan(amplitude * wave_number)

        def error(x, y):
            # Vertical error is conservative relative to perpendicular distance.
            return y - amplitude * math.sin(wave_number * x)
    else:
        raise ValueError('unsupported synthetic scene: ' + kind)
    return Polyline(case['name'], 'synthetic_map', points), yaw, error


def integrate_bicycle(x, y, yaw, speed, angle, wheelbase, dt):
    """Exact constant-curvature arc for one step, with no actuator delay/slip."""
    curvature = math.tan(angle) / wheelbase
    distance = speed * dt
    turn = curvature * distance
    # sinc form avoids cancellation near zero steering.
    half_turn = turn / 2.0
    scale = math.sin(half_turn) / half_turn if half_turn != 0.0 else 1.0
    return (x + distance * scale * math.cos(yaw + half_turn),
            y + distance * scale * math.sin(yaw + half_turn), yaw + turn)


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(len(ordered) * fraction) - 1)]


def simulate(case, settings, dt, tail_s, trace):
    line, yaw, error = make_scene(case)
    x, y = 0.0, case['initial_error_m']
    speed = case['speed_mps']
    previous = 0.0
    errors = [abs(error(x, y))]
    durations_us = []
    max_angle, max_rate = 0.0, 0.0
    angle_limits, rate_limits = 0, 0
    steps = round(case['duration_s'] / dt)
    failure = None
    for index in range(steps):
        now = index * dt
        vehicle = VehiclePose(x, y, yaw, speed, now, 'synthetic_map')
        reference = TrackingReference(line, now, 0.0, line.length_m)
        started = time.perf_counter_ns()
        result = compute_steering(vehicle, reference, settings, now_s=now,
                                  dt_s=dt, previous_angle_rad=previous)
        durations_us.append((time.perf_counter_ns() - started) / 1000.0)
        if not result.valid:
            failure = {'step': index, 'reason': result.reason}
            break
        angle = result.angle_rad
        rate = abs(angle - previous) / dt
        max_angle = max(max_angle, abs(angle))
        max_rate = max(max_rate, rate)
        angle_limits += int(result.angle_limited)
        rate_limits += int(result.rate_limited)
        x, y, yaw = integrate_bicycle(x, y, yaw, speed, angle, settings.wheelbase_m, dt)
        errors.append(abs(error(x, y)))
        trace.writerow((case['name'], (index + 1) * dt, x, y, yaw,
                        angle, result.desired_angle_rad, errors[-1], result.projection_s_m))
        previous = angle
    tail = errors[-max(1, round(tail_s / dt)):]
    measured = {
        'max_error_m': max(errors),
        'tail_rms_m': math.sqrt(sum(value * value for value in tail) / len(tail)),
        'final_error_m': errors[-1],
        'max_angle_rad': max_angle,
        'max_rate_rad_s': max_rate,
    }
    gates = {key: measured[key] <= case[key]
             for key in ('max_error_m', 'tail_rms_m', 'final_error_m')}
    gates.update(completed=failure is None and len(errors) == steps + 1,
                 angle_bound=max_angle <= settings.max_angle_rad + 1e-12,
                 rate_bound=max_rate <= settings.max_rate_rad_s + 1e-12)
    return {
        'name': case['name'], 'kind': case['kind'], 'passed': all(gates.values()),
        'gates': gates, 'measured': measured, 'failure': failure,
        'steps': len(errors) - 1, 'angle_limited_steps': angle_limits,
        'rate_limited_steps': rate_limits,
        'compute_us': {'p50': percentile(durations_us, 0.50),
                       'p95': percentile(durations_us, 0.95),
                       'p99': percentile(durations_us, 0.99),
                       'max': max(durations_us)},
    }


def unit_checks(output):
    test = PACKAGE / 'test/test_lateral.py'
    xml_path = output / 'unit.xml'
    arguments = ['-q', '-p', 'no:cacheprovider', str(test), '--junitxml=' + str(xml_path)]
    code = ('import sys; sys.path.insert(0, ' + repr(str(PACKAGE)) + '); '
            'import pytest; raise SystemExit(pytest.main(' + repr(arguments) + '))')
    environment = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
                       PYTHONDONTWRITEBYTECODE='1', PYTHONUTF8='1')
    with (output / 'unit.log').open('w', encoding='utf-8') as log:
        process = subprocess.run([sys.executable, '-B', '-c', code], cwd=REPO,
                                 env=environment, stdout=log, stderr=subprocess.STDOUT)
    counts = dict(tests=0, failures=0, errors=0, skipped=0)
    if xml_path.is_file():
        for suite in ET.parse(xml_path).getroot().iter('testsuite'):
            for key in counts:
                counts[key] += int(suite.attrib.get(key, 0))
    counts['passed'] = counts['tests'] - counts['failures'] - counts['errors'] - counts['skipped']
    counts['exit_code'] = process.returncode
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='New result directory; existing paths are refused')
    args = parser.parse_args()
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%SZ')
    output = (args.output or WS / 'verification/.work' /
              ('independent-lateral-' + stamp + '-' + uuid.uuid4().hex[:8])).resolve()
    output.mkdir(parents=True, exist_ok=False)
    case_path = Path(__file__).with_name('lateral_cases.json')
    definition = json.loads(case_path.read_text(encoding='utf-8'))
    settings = LateralConfig(**definition['config'])
    units = unit_checks(output)
    print('Unit checks:', units, flush=True)
    scenarios = []
    with (output / 'trace.csv').open('w', encoding='utf-8', newline='') as stream:
        trace = csv.writer(stream)
        trace.writerow(('scenario', 'time_s', 'rear_x_m', 'rear_y_m', 'yaw_rad',
                        'steering_rad', 'desired_steering_rad', 'oracle_abs_error_m', 'projection_s_m'))
        for case in definition['scenarios']:
            result = simulate(case, settings, definition['dt_s'], definition['tail_s'], trace)
            scenarios.append(result)
            print(case['name'] + ': ' + ('PASS' if result['passed'] else 'FAIL'), flush=True)
    sources = [Path(__file__), case_path, PACKAGE / 'ctrl_control/ctrl_lateral.py',
               PACKAGE / 'test/test_lateral.py', WS / 'docs/CONTROL_CONTRACT_2026-10-25.md']
    manifest = {str(path.relative_to(REPO)).replace('\\', '/'):
                hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    imports = sorted(name for name in sys.modules
                     if name.startswith(('ctrl_', 'rospy', 'rclpy')))
    passed = (units['exit_code'] == 0 and units['passed'] > 0
              and units['skipped'] == 0 and all(item['passed'] for item in scenarios)
              and imports == ['ctrl_control', 'ctrl_control.ctrl_lateral'])
    report = {
        'schema_version': 1, 'passed': passed,
        'scope': definition['scope'], 'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
        'python': sys.version, 'platform': platform.platform(),
        'unit': units, 'scenarios': scenarios, 'config': definition['config'],
        'dt_s': definition['dt_s'], 'tail_s': definition['tail_s'],
        'project_or_ros_imports': imports, 'source_sha256': manifest,
        'limitations': ['No ROS/catkin/MORAI execution', 'No control packets sent',
                        'No actuator delay, tire dynamics, sensor noise or collision model',
                        'No production calibration or baseline live-performance comparison'],
    }
    (output / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print('Report:', output / 'report.json', flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
