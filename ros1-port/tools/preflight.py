#!/usr/bin/env python3
"""Read-only checks available without Ubuntu, ROS or third-party packages."""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).resolve().parent))
from ctrl_names import legacy_text


WS = Path(__file__).resolve().parents[1]
REPOSITORY = WS.parent
SOURCE = REPOSITORY / '2026_CTRL_competition'


def main():
    failures = []
    for tree in (SOURCE / 'src', WS / 'src'):
        for path in tree.rglob('*.py'):
            if '__pycache__' in path.parts:
                continue
            try:
                ast.parse(path.read_text(encoding='utf-8'), filename=str(path), feature_version=(3, 8))
            except (SyntaxError, UnicodeError) as exc:
                failures.append(str(exc))
    packages = sorted((WS / 'src').glob('*/package.xml'))
    for package in packages:
        data = ET.parse(package).getroot()
        name = data.findtext('name')
        if data.findtext('buildtool_depend') != 'catkin':
            failures.append(name + ': missing catkin build tool')
        if not (package.parent / 'CMakeLists.txt').exists():
            failures.append(name + ': missing CMakeLists.txt')
    # Check the existing calibrations rather than weakening their runtime guard.
    for base in (SOURCE, WS):
        def resource_path(relative):return base/(legacy_text(relative) if base==SOURCE else relative)
        calibration = json.loads(resource_path('src/ctrl_camera/config/yolo_stoplines.json').read_text(encoding='utf-8'))
        route = resource_path('src/ctrl_bringup/routes/global_path.txt')
        actual = hashlib.sha256(route.read_bytes()).hexdigest()
        if actual != calibration['route_sha256']:
            failures.append(str(base) + ': traffic route calibration hash mismatch')
        model = resource_path('src/ctrl_camera/models/best.onnx')
        actual_model = hashlib.sha256(model.read_bytes()).hexdigest()
        if actual_model != '04ab171d8a46b5576891a23034000fa34cc63bf63336c8edb67b9a3fee551b8b':
            failures.append(str(base) + ': original model README hash mismatch')
        for resource in ('ctrl_planning/config/highway_kappa.npz',
                         'ctrl_bringup/maps/link_set.seams.npz',
                         'ctrl_bringup/routes/spawn_path.txt',
                         'ctrl_bringup/routes/checkpoints.txt'):
            if not resource_path('src/'+resource).is_file():
                failures.append(str(base) + ': missing resource ' + resource)
    for script in ('port_boundary.py', 'check_packaging.py'):
        result = subprocess.run([sys.executable, str(WS / 'tools' / script)], cwd=str(REPOSITORY),
                                capture_output=True, text=True, encoding='utf-8')
        if result.returncode:
            failures.append(result.stdout + result.stderr)
    if failures:
        raise SystemExit('\n'.join(failures))
    print('PASS: Python 3.8 syntax; %d catkin packages; original/port route calibration and model hashes;'
          ' required resources; exact boundary and launch contract checks' % len(packages))


if __name__ == '__main__':
    main()
