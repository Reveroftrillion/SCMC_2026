"""Windows 회귀 검사를 새 결과 디렉터리에서 실행하고 과거 기록을 보존한다."""
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from ctrl_names import current_relative


WS = Path(__file__).resolve().parents[1]
REPO = WS.parent


def main():
    results = WS / 'verification/.work' / ('identifier-refactor-' + datetime.now().strftime('%Y%m%d-%H%M%S'))
    results.mkdir(parents=True, exist_ok=False)
    if '--ctrl-names-only' in sys.argv:
        return subprocess.run([sys.executable,'-X','utf8',str(WS/'tools/check_ctrl_names.py'),
                               '--output',str(results)],cwd=REPO).returncode
    tracked = subprocess.check_output(['git', 'ls-files', '-z'], cwd=REPO).decode('utf-8').split('\0')
    # Resolve the reference directory rename before it has been committed too.
    tracked = [name.replace('2026foscar-competition/', '2026_CTRL_competition/', 1)
               if name.startswith('2026foscar-competition/') else name for name in tracked]
    tracked=[current_relative(name) if name.startswith('ros1-port/src/') else name for name in tracked]
    protected = [REPO / name for name in tracked if name and (
        name.startswith('2026_CTRL_competition/') or
        (name.startswith('ros1-port/') and (name.endswith('.json') or name.endswith('.md'))
         and name not in ('ros1-port/README.md',)))]
    before = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in protected}
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONUTF8='1', PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
    outcomes = {}

    def run(label, arguments):
        log = results / (label + '.log')
        with log.open('w', encoding='utf-8') as stream:
            completed = subprocess.run([sys.executable, '-X', 'utf8', *arguments], cwd=REPO,
                                       env=env, stdout=stream, stderr=subprocess.STDOUT)
        outcomes[label] = {'exit_code': completed.returncode, 'log': str(log)}
        print(label + ': exit ' + str(completed.returncode), flush=True)

    python_files = [REPO / name for name in tracked if name.startswith('ros1-port/') and name.endswith('.py')]
    python_files += [Path(__file__), WS / 'tools/check_identifier_refactor.py']
    for path in set(python_files):
        compile(path.read_bytes(), str(path), 'exec')
    print('syntax: PASS (' + str(len(set(python_files))) + ' Python files)', flush=True)
    run('identifier-ast', [str(WS / 'tools/check_identifier_refactor.py')])
    run('adapter', ['-m', 'unittest', 'discover', '-s', str(WS / 'src/ctrl_ros1/test'), '-v'])
    # Explicit sys.path supports embeddable Python's isolated ._pth configuration.
    run('packaging-unit', ['-c', 'import sys, unittest; sys.path.insert(0,' + repr(str(WS / 'tools')) +
                          '); suite=unittest.defaultTestLoader.loadTestsFromName("test_packaging"); '
                          'raise SystemExit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())'])
    run('packaging', [str(WS / 'tools/check_packaging.py')])
    run('boundary', [str(WS / 'tools/port_boundary.py')])
    run('preflight', [str(WS / 'tools/preflight.py')])

    def module_command(path, setup):
        return ['-c', 'import importlib.util; from pathlib import Path; '
                'spec=importlib.util.spec_from_file_location("isolated_check",' + repr(str(path)) + '); '
                'module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module); ' + setup]

    preservation = results / 'preservation'
    preservation.mkdir()
    run('preservation', module_command(WS / 'verification/check_preservation.py',
        'module.HERE=Path(' + repr(str(preservation)) + '); module.main()'))
    differential = results / 'differential'
    differential.mkdir()
    for filename in ('pytest_bridge.py', 'ros_stubs.py'):
        shutil.copy2(WS / 'verification' / filename, differential / filename)
    for filename in ('ctrl_names.py','ctrl_name_map.json'):
        shutil.copy2(WS/'tools'/filename,differential/filename)
    # ros_stubs resolves the production adapter relative to its own file.
    shutil.copytree(WS / 'src/ctrl_ros1', results / 'src/ctrl_ros1',
                    ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    run('differential', module_command(WS / 'verification/run_differential.py',
        'module.HERE=Path(' + repr(str(differential)) + '); raise SystemExit(module.main())'))
    model = results / 'model'
    shutil.copytree(WS / 'src', model / 'src', ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    run('model', module_command(WS / 'tools/check_model_equivalence.py',
        'module.WS=Path(' + repr(str(model)) + '); module.main()'))
    changed = [str(path.relative_to(REPO)) for path, digest in before.items()
               if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != digest]
    summary = {'scope': 'Windows static/unit/differential only; no ROS/MORAI execution',
               'python_files_compiled': len(set(python_files)), 'checks': outcomes,
               'protected_files': len(before), 'protected_file_changes': changed}
    (results / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print('Protected file changes: ' + repr(changed), flush=True)
    print('Results: ' + str(results), flush=True)
    # Known byte/boundary mismatches and model preflight remain failures in logs.
    required = ('identifier-ast', 'adapter', 'packaging-unit', 'packaging', 'differential', 'model')
    return 1 if changed or any(outcomes[label]['exit_code'] for label in required) else 0


if __name__ == '__main__':
    raise SystemExit(main())
