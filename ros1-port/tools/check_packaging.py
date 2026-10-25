"""Check launch contracts, topic sets and resource bytes without ROS installed."""
import ast
import hashlib
from pathlib import Path
import shlex
import sys
import xml.etree.ElementTree as ET

# The Windows embeddable interpreter deliberately omits the script directory
# from sys.path; this tool has one explicit sibling dependency.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_packaging import ORIGINAL, WS, read_launch
from ctrl_names import launch_name, package_name, legacy_text


LAUNCHES = [('ctrl_bringup', 'competition'), ('ctrl_bringup', 'low_speed'),
            ('ctrl_lidar', 'obstacle'), ('ctrl_lidar', 'drive'),
            ('ctrl_camera', 'yolo_signal'), ('ctrl_camera', 'gps_record')]


def normalized(item):
    kind = type(item).__name__
    if kind == 'LaunchConfiguration':
        return '$(arg ' + item.args[0] + ')'
    if kind == 'ParameterValue':
        return normalized(item.args[0])
    if isinstance(item, bool):
        return str(item).lower()
    return str(item).replace('\\', '/').replace(str(Path.home()).replace('\\', '/'), '$(env HOME)')


def nested(actions):
    for action in actions:
        yield action
        if type(action).__name__ == 'GroupAction':
            yield from nested(action.args[0])
        elif type(action).__name__ == 'TimerAction':
            yield from nested(action.kwargs['actions'])


def check_launches():
    node_count = argument_count = parameter_count = 0
    for package, stem in LAUNCHES:
        source = list(nested(read_launch(package, stem + '.launch.py')))
        destination = ET.parse(WS / 'src' / package / 'launch' / launch_name(stem + '.launch')).getroot()
        expected_args = {action.args[0]: normalized(action.kwargs['default_value'])
                         for action in source if type(action).__name__ == 'DeclareLaunchArgument'}
        actual_args = {element.get('name'): element.get('default') for element in destination.findall('arg')}
        assert expected_args == actual_args, (package, stem, 'argument defaults', expected_args, actual_args)
        argument_count += len(expected_args)
        elements = list(destination.iter('node'))
        for deferred in (WS / 'src' / package / 'launch').glob(launch_name(stem + '_delayed.launch')):
            elements += list(ET.parse(deferred).getroot().iter('node'))
        required = {action.args[0].kwargs.get('target_action') for action in source
                    if type(action).__name__ == 'RegisterEventHandler'
                    and type(action.args[0]).__name__ == 'OnProcessExit'}
        for action in source:
            if type(action).__name__ != 'Node':
                continue
            data = action.kwargs
            pkg = 'rviz' if data['package'] == 'rviz2' else data['package']
            executable = 'rviz' if data['executable'] == 'rviz2' else data['executable']
            matches = [element for element in elements if element.get('pkg') == pkg and element.get('type') == executable]
            assert len(matches) == 1, (package, stem, executable, 'missing or duplicate node')
            node = matches[0]
            assert node.get('required') == ('true' if action in required else None), (stem, executable, 'exit policy')
            expected_params = {}
            files = []
            for item in data.get('parameters', []):
                if isinstance(item, dict):
                    expected_params.update(item)
                else:
                    files.append(normalized(item))
            actual_params = {element.get('name').replace('/', '.'): element for element in node.findall('param')}
            assert set(actual_params) == set(expected_params) | ({'_ros2_parameter_files'} if files else set()), (stem, executable, 'parameter key set')
            for key, item in expected_params.items():
                element = actual_params[key]
                expected = normalized(item)
                if key in ('code_version', 'detections_log'):
                    assert element.get('command'), (stem, executable, key, 'launch-time metadata')
                elif key == 'route' and expected == '$(arg route)':
                    expr = element.get('value')
                    assert expr.startswith('$(eval ') and expr.endswith(')'), (stem, 'route choice guard')
                    for choice in ('auto', 'spawn', 'global'):
                        assert eval(expr[7:-1], {'arg': lambda _: choice}) == choice
                    try:
                        eval(expr[7:-1], {'arg': lambda _: 'invalid'})
                    except KeyError:
                        pass
                    else:
                        raise AssertionError('route choices were dropped')
                else:
                    assert element.get('value') == expected, (stem, executable, key, expected, element.attrib)
                value_type = item.kwargs['value_type'] if type(item).__name__ == 'ParameterValue' else type(item)
                if 'command' not in element.attrib:
                    assert element.get('type') == {bool: 'bool', int: 'int', float: 'double'}.get(value_type, 'str'), (stem, executable, key, 'parameter type')
                parameter_count += 1
            if files:
                assert len(files) == 1 and actual_params['_ros2_parameter_files'].get('value') == files[0]
            if pkg == 'tf2_ros':
                args = shlex.split(node.get('args'))
                assert args == ['$(arg', 'lidar_x)', '$(arg', 'lidar_y)', '$(arg', 'lidar_z)', '0', '0', '0', '1', 'base_link', 'lidar']
            node_count += 1
        for timer in [action for action in source if type(action).__name__ == 'TimerAction']:
            helper = next(element for element in elements if element.get('type') == 'ctrl_delay_wall')
            assert float(helper.get('args').split()[0]) == timer.kwargs['period'] == 5.0
    return argument_count, node_count, parameter_count


def check_topics():
    tree = ast.parse((ORIGINAL / 'foscar_bringup/launch/competition.launch.py').read_text(encoding='utf-8'))
    expected = {}
    for assignment in ast.walk(tree):
        if not isinstance(assignment, ast.Assign):
            continue
        names = [target.id for target in assignment.targets if isinstance(target, ast.Name)]
        if not set(names) & {'bag', 'points'} or not isinstance(assignment.value, ast.Call):
            continue
        cmd = next(keyword.value for keyword in assignment.value.keywords if keyword.arg == 'cmd')
        expected[names[0]] = [item.value for item in cmd.elts if isinstance(item, ast.Constant)
                              and isinstance(item.value, str) and item.value.startswith('/')]
    helper = ast.parse((WS / 'src/ctrl_bringup/scripts/ctrl_record_bag').read_text(encoding='utf-8'))
    actual = {target.id: ast.literal_eval(assignment.value) for assignment in helper.body if isinstance(assignment, ast.Assign)
              for target in assignment.targets if isinstance(target, ast.Name) and target.id in ('TOPICS', 'POINT_TOPICS')}
    assert expected['bag'] == actual['TOPICS']
    assert expected['points'] == actual['POINT_TOPICS']
    return len(actual['TOPICS']), len(actual['POINT_TOPICS'])


def check_resources():
    count = 0
    for package in ORIGINAL.iterdir():
        for directory in ('config', 'maps', 'routes', 'models', 'msg'):
            source = package / directory
            if not source.is_dir():
                continue
            expected_files = {file.relative_to(source) for file in source.rglob('*') if file.is_file()}
            destination = WS / 'src' / package_name(package.name) / directory
            assert expected_files == {file.relative_to(destination) for file in destination.rglob('*') if file.is_file()}, (package, directory, 'resource inventory')
            for relative in expected_files:
                assert hashlib.sha256((source / relative).read_bytes()).digest() == hashlib.sha256((destination / relative).read_bytes()).digest(), (package.name, directory, relative)
                count += 1
    return count


if __name__ == '__main__':
    arguments, nodes, parameters = check_launches()
    topics, point_topics = check_topics()
    resources = check_resources()
    print('PASS: {} launch arguments, {} node declarations, {} typed parameters; {} + {} bag topics; {} exact resource/message files'.format(arguments, nodes, parameters, topics, point_topics, resources))
