#!/usr/bin/env python3
"""Reproduce only the ROS boundary edits; never write to the reference tree."""
import argparse
from pathlib import Path
import re
import ast
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from ctrl_names import current_path, legacy_ast, current_python


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / '2026_CTRL_competition' / 'src'
DESTINATION = ROOT / 'ros1-port' / 'src'


def converted(relative, text):
    text = re.sub(r'^import rclpy$', 'import ctrl_ros1 as rclpy', text, flags=re.M)
    text = text.replace('from rclpy.', 'from ctrl_ros1.')
    text = text.replace('from ament_index_python.packages import ',
                        'from ctrl_ros1.ctrl_packages import ')
    text = text.replace('from sensor_msgs_py import ', 'from ctrl_ros1 import ')
    # Humble Quaternion.w defaults to 1; Noetic defaults to 0. Preserve Marker
    # payloads (including DELETEALL), using a generated-message subclass only.
    def marker_import(match):
        names = [name.strip() for name in match.group(1).split(',')]
        if 'Marker' not in names:
            return match.group(0)
        others = [name for name in names if name != 'Marker']
        prefix = 'from visualization_msgs.msg import ' + ', '.join(others) + '\n' if others else ''
        return prefix + 'from ctrl_ros1.ctrl_messages import Marker'
    text = re.sub(r'^from visualization_msgs\.msg import ([^\n]+)$', marker_import, text, flags=re.M)
    text = re.sub(r'\.sec\b', '.secs', text)
    text = re.sub(r'\.nanosec\b', '.nsecs', text)
    if 'from builtin_interfaces.msg import Duration' in text:
        text = text.replace('from builtin_interfaces.msg import Duration',
                            'from rospy import Duration')
        text = re.sub(r'\bDuration\(sec=', 'Duration(secs=', text)
    text = text.replace('TransformBroadcaster(self)', 'TransformBroadcaster()')
    if relative.as_posix() == 'foscar_lidar/foscar_lidar/obstacle_node.py':
        # Keep TF updates on the same executor as cloud callbacks, as in Humble.
        text = text.replace('            import tf2_ros\n',
                            '            from ctrl_ros1 import tf2_ros\n')
    if relative.as_posix() == 'foscar_planning/foscar_planning/highway_policy.py':
        text = text.replace('import hashlib\n',
                            'import hashlib\nfrom ctrl_ros1.ctrl_packages import get_package_share_directory\n')
        text = text.replace('Path(__file__).resolve().parents[1]/"config"/"highway_kappa.npz"',
                            'Path(get_package_share_directory("ctrl_planning"))/"config"/"highway_kappa.npz"')
    return current_python(text,relative)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', action='store_true', help='apply only to ros1-port/src')
    args = parser.parse_args()
    changed = []
    errors = []
    for path in sorted(SOURCE.glob('*/*/**/*.py')):
        relative = path.relative_to(SOURCE)
        # Runtime modules only. Keep original tests and launch descriptions as fixtures.
        if len(relative.parts) < 3 or relative.parts[0] != relative.parts[1]:
            continue
        original = path.read_text(encoding='utf-8')
        expected = converted(relative, original)
        target = current_path(DESTINATION / relative)
        if expected != original:
            changed.append(relative.as_posix())
        if args.write:
            # Preserve byte-identical pure Python files and all source resource files.
            if expected != original:
                target.write_text(expected, encoding='utf-8', newline='\n')
        elif not target.exists() or ast.dump(legacy_ast(target.read_text(encoding='utf-8'))) != ast.dump(legacy_ast(expected)):
            errors.append(relative.as_posix())
    print('ROS boundary files:', len(changed))
    for path in changed:
        print('  ' + path)
    if errors:
        raise SystemExit('Unexpected production changes: ' + ', '.join(errors))


if __name__ == '__main__':
    main()
