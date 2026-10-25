"""Focused Windows checks, synthetic comparison replay, and example settings export."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid
import xml.etree.ElementTree as ET

WS = Path(__file__).resolve().parents[1]
REPO = WS.parent
PACKAGE = WS / 'src/ctrl_control'
sys.path.insert(0, str(PACKAGE))
from ctrl_control.ctrl_lateral import VehiclePose
from ctrl_control.ctrl_path_conversion import StationedPath
from ctrl_control.ctrl_path_selection import PathRequest
from ctrl_control.ctrl_shadow import ShadowEngine, ShadowFrame, ShadowSettings, encode, replay


def example_settings():
    tools = WS / 'tools'
    def load(name, key):
        return json.loads((tools / name).read_text(encoding='utf-8'))[key]
    return ShadowSettings.from_dict(dict(
        lateral=load('lateral_cases.json', 'config'),
        progress=load('progress_cases.json', 'progress_config'),
        selection=load('path_selection_cases.json', 'selection_config'),
        seed_half_window_m=2., initial_step_s=.05, legacy_transition_time_s=1.5,
        legacy_return_time_s=2., legacy_min_length_m=6.))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--replay', type=Path, help='Replay an existing shadow JSONL without ROS')
    group.add_argument('--example-config', type=Path, help='Write NEW synthetic-only settings; calibrate before use')
    parser.add_argument('--output', type=Path, help='New validation directory')
    args = parser.parse_args()
    if args.replay:
        result = replay(args.replay)
        print(json.dumps(result))
        return 0 if result['passed'] else 1
    if args.example_config:
        with args.example_config.open('x', encoding='utf-8') as stream:
            json.dump(asdict(example_settings()), stream, indent=2, allow_nan=False)
            stream.write('\n')
        print('Synthetic-only example:', args.example_config)
        return 0
    output = (args.output or WS / 'verification/.work' /
              ('shadow-' + datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%SZ') + '-' + uuid.uuid4().hex[:8])).resolve()
    output.mkdir(parents=True, exist_ok=False)
    names = ['test_lateral.py', 'test_progress.py', 'test_path_conversion.py', 'test_path_selection.py',
             'test_shadow.py', 'test_shadow_follower.py', 'test_core.py', 'test_reference_wire.py', 'test_route_start.py']
    tests = [PACKAGE / 'test' / name for name in names]
    code = ('import sys; from pathlib import Path; ws=Path(' + repr(str(WS)) + '); '
            'sys.path.insert(0,str(ws / "verification")); '
            '[sys.path.insert(0,str(p)) for p in (ws / "src").iterdir() if p.is_dir()]; '
            'import ros_stubs; ros_stubs.install(ws); import pytest; '
            'raise SystemExit(pytest.main(' + repr(['-q', '-p', 'no:cacheprovider', *map(str, tests),
                                                  '--junitxml=' + str(output / 'unit.xml')]) + '))')
    env = dict(os.environ, PYTHONUTF8='1', PYTHONDONTWRITEBYTECODE='1', PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
    with (output / 'unit.log').open('w', encoding='utf-8') as stream:
        process = subprocess.run([sys.executable, '-X', 'utf8', '-B', '-c', code], cwd=REPO, env=env,
                                 stdout=stream, stderr=subprocess.STDOUT)
    counts = dict(tests=0, failures=0, errors=0, skipped=0)
    if (output / 'unit.xml').exists():
        for suite in ET.parse(output / 'unit.xml').getroot().iter('testsuite'):
            for key in counts:
                counts[key] += int(suite.attrib.get(key, 0))
    results = []
    for mode in ('global', 'legacy_offset', 'reference'):
        e = ShadowEngine(StationedPath.from_global_route('base', 'map', ((0, 0), (60, 0)), global_origin_m=500),
                         example_settings())
        path = output / (mode + '.jsonl')
        valid = 0
        with path.open('x', encoding='utf-8') as stream:
            stream.write(json.dumps(encode(e.header()), allow_nan=False) + '\n')
            for i in range(200):
                now, x = 10 + i * .05, 10 + i * .1
                kwargs = dict(lat_offset_m=.5) if mode == 'legacy_offset' else {}
                if mode == 'reference' and i == 0:
                    kwargs = dict(reference_id='line', reference_x=tuple(range(61)), reference_y=(0.,) * 61,
                                  reference_global_s=tuple(range(500, 561)))
                f = ShadowFrame(now, VehiclePose(x, .2, 0, 2, now, 'map'), 500 + x, 0,
                                PathRequest(now, 'map', **kwargs), now)
                result = e.evaluate(f)
                valid += result['valid']
                stream.write(json.dumps(encode(dict(type='frame', input=asdict(f), output=result)), allow_nan=False) + '\n')
        result = replay(path)
        result.update(mode=mode, valid_candidates=valid)
        result['passed'] = result['passed'] and valid == 200
        results.append(result)
    sources = [Path(__file__), *tests, *sorted((PACKAGE / 'ctrl_control').glob('ctrl_*.py')),
               WS / 'verification/ros_stubs.py', *[WS / 'tools' / name for name in
               ('lateral_cases.json', 'progress_cases.json', 'path_selection_cases.json')]]
    report = dict(schema_version=1, recorded_at_utc=datetime.now(timezone.utc).isoformat(),
                  passed=process.returncode == 0 and counts['tests'] > 0 and not any(counts[k] for k in
                         ('failures', 'errors', 'skipped')) and all(r['passed'] for r in results),
                  unit=dict(**counts, exit_code=process.returncode), synthetic_replay=results,
                  source_sha256={p.relative_to(REPO).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                  limitations=['ROS doubles only; no catkin, middleware, physical sensor replay or MORAI validation',
                               'Synthetic input replay checks determinism, not closed-loop vehicle performance',
                               'No vehicle command transmission; shadow default disabled'])
    (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'], unit=report['unit'], synthetic_replay=results), indent=2))
    print('Report:', output / 'report.json')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
