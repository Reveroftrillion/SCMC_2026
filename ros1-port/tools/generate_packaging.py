"""Generate ROS1 packaging from the preserved ROS2 launch descriptions.

This script writes only ros1-port, never the comparison source tree.
"""
from pathlib import Path
import ast
import sys
import types
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).resolve().parent))
from ctrl_names import ctrl_text, current_path, package_name, REVERSE_PACKAGES, FILES

WS = Path(__file__).resolve().parents[1]
ORIGINAL = WS.parent / '2026_CTRL_competition' / 'src'


def write_file(path, text):
    path=current_path(path)
    text=ctrl_text(text)
    # pathlib.Path.write_text(newline=...) and ElementTree.indent were added
    # after Noetic's Python 3.8. Keep generation tools usable there as well.
    with path.open('w', encoding='utf-8', newline='\n') as stream:
        stream.write(text)


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class LaunchConfiguration(Action):
    def __str__(self):
        return '$(arg ' + self.args[0] + ')'


class ParameterValue(Action):
    pass


def read_launch(package, filename):
    """Execute launch declarations against inert action classes, without ROS."""
    names = {
        'LaunchDescription', 'DeclareLaunchArgument', 'EmitEvent', 'ExecuteProcess',
        'GroupAction', 'IncludeLaunchDescription', 'RegisterEventHandler', 'TimerAction',
        'IfCondition', 'OnProcessExit', 'Shutdown', 'PythonLaunchDescriptionSource',
        'Node', 'AndSubstitution', 'NotSubstitution', 'OnShutdown', 'OpaqueFunction',
    }
    classes = {name: type(name, (Action,), {}) for name in names}
    modules = ['ament_index_python', 'ament_index_python.packages', 'launch',
               'launch.actions', 'launch.conditions', 'launch.event_handlers',
               'launch.events', 'launch.launch_description_sources',
               'launch.substitutions', 'launch_ros', 'launch_ros.actions',
               'launch_ros.parameter_descriptions']
    saved = {name: sys.modules.get(name) for name in modules}
    try:
        for name in modules:
            module = types.ModuleType(name)
            module.__dict__.update(classes)
            module.LaunchConfiguration = LaunchConfiguration
            module.ParameterValue = ParameterValue
            module.get_package_share_directory = lambda pkg: '$(find ' + pkg + ')'
            module.PackageNotFoundError = LookupError
            sys.modules[name] = module
        package=REVERSE_PACKAGES.get(package,package)
        filename={v:k for k,v in FILES.items()}.get(filename,filename)
        filename = ORIGINAL / package / 'launch' / filename
        scope = {'__file__': str(filename)}
        text = ctrl_text(filename.read_text(encoding='utf-8')).replace('    logs.mkdir(parents=True, exist_ok=True)\n', '')
        exec(compile(text, str(filename), 'exec'), scope)
        # code_version is evaluated at launch time through launch_info, not frozen here.
        if 'code_version' in scope:
            scope['code_version'] = lambda: '__CODE_VERSION__'
        # Avoid creating a literal $(find...) path from yolo's logs.mkdir.
        return scope['generate_launch_description']().args[0]
    finally:
        for name, module in saved.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module


def value(item):
    if isinstance(item, ParameterValue):
        return value(item.args[0])
    if isinstance(item, list):
        return ''.join(value(x) for x in item)
    if isinstance(item, bool):
        return str(item).lower()
    return str(item).replace('\\', '/').replace(str(Path.home()).replace('\\', '/'), '$(env HOME)')


def condition(item):
    if type(item).__name__ == 'IfCondition':
        item = item.args[0]
    kind = type(item).__name__
    if kind == 'LaunchConfiguration':
        return "arg('" + item.args[0] + "')"
    if kind == 'AndSubstitution':
        return '(' + ' and '.join(condition(x) for x in item.args) + ')'
    if kind == 'NotSubstitution':
        return '(not ' + condition(item.args[0]) + ')'
    return value(item)


def add_condition(element, item):
    if item is not None:
        if type(item.args[0]).__name__ == 'LaunchConfiguration':
            element.set('if', value(item.args[0]))
        else:
            element.set('if', '$(eval ' + condition(item) + ')')


def parameter(parent, name, item):
    kind = item.kwargs.get('value_type') if isinstance(item, ParameterValue) else type(item)
    text = value(item)
    if name == 'route' and text == '$(arg route)':
        # ROS1 XML has no choices attribute. Evaluate the same allowed values
        # while parsing the launch so invalid input still fails before startup.
        text = "$(eval {'auto':'auto','spawn':'spawn','global':'global'}[arg('route')])"
    attributes = {'name': name.replace('.', '/')}
    if text == '__CODE_VERSION__':
        attributes['command'] = 'python3 "$(find ctrl_bringup)/scripts/ctrl_launch_info" code-version'
    elif name == 'detections_log':
        attributes['command'] = 'python3 "$(find ctrl_bringup)/scripts/ctrl_launch_info" detections-log'
    else:
        attributes['value'] = text
        attributes['type'] = {bool: 'bool', int: 'int', float: 'double'}.get(kind, 'str')
    ET.SubElement(parent, 'param', attributes)


def render_node(parent, action, required):
    data = action.kwargs
    package, executable = data['package'], data['executable']
    if package == 'rviz2':
        package, executable = 'rviz', 'rviz'
    name = data.get('name', executable)
    attributes = {'pkg': package, 'type': executable, 'name': name,
                  'output': data.get('output', 'log')}
    if action in required:
        attributes['required'] = 'true'
    if 'arguments' in data:
        attributes['args'] = ' '.join('"' + value(x) + '"' for x in data['arguments'])
    if package == 'tf2_ros':
        args = data['arguments']
        attributes['args'] = ' '.join(value(x) for x in (args[1], args[3], args[5])) + ' 0 0 0 1 base_link lidar'
    node = ET.SubElement(parent, 'node', attributes)
    add_condition(node, data.get('condition'))
    files = []
    for params in data.get('parameters', []):
        if isinstance(params, dict):
            for key, item in params.items():
                parameter(node, key, item)
        else:
            files.append(value(params))
    if files:
        if len(files) != 1:
            raise ValueError('Expected one original configuration file')
        parameter(node, '_ros2_parameter_files', files[0])


def render_include(parent, action):
    path = value(action.args[0].args[0]).replace('.launch.py', '.launch')
    element = ET.SubElement(parent, 'include', {'file': path})
    add_condition(element, action.kwargs.get('condition'))
    for key, item in action.kwargs.get('launch_arguments', []):
        ET.SubElement(element, 'arg', {'name': key, 'value': value(item)})


def render(parent, action, required, package, stem, arguments):
    kind = type(action).__name__
    if kind == 'DeclareLaunchArgument':
        ET.SubElement(parent, 'arg', {'name': action.args[0], 'default': value(action.kwargs['default_value'])})
    elif kind == 'Node':
        render_node(parent, action, required)
    elif kind == 'IncludeLaunchDescription':
        render_include(parent, action)
    elif kind == 'GroupAction':
        group = ET.SubElement(parent, 'group')
        add_condition(group, action.kwargs.get('condition'))
        for child in action.args[0]:
            render(group, child, required, package, stem, arguments)
    elif kind == 'TimerAction':
        # ROS2 TimerAction uses wall time. A child roslaunch starts the entire
        # camera group after the same monotonic delay, including log creation.
        deferred = ET.Element('launch')
        for argument in arguments:
            ET.SubElement(deferred, 'arg', {'name': argument.args[0]})
        for child in action.kwargs['actions']:
            render(deferred, child, required, package, stem, arguments)
        delayed_name = stem + '_delayed.launch'
        write_xml(WS / 'src' / package / 'launch' / delayed_name, deferred)
        attrs = {'pkg': 'ctrl_bringup', 'type': 'delay_wall', 'name': stem + '_camera_delay', 'output': 'screen',
                 'args': str(action.kwargs['period']) + ' ' + package + ' ' + delayed_name + ' ' +
                 ' '.join('"' + argument.args[0] + ':=' + value(LaunchConfiguration(argument.args[0])) + '"' for argument in arguments)}
        # The delayed competition group has one camera condition, so avoid
        # starting the helper when disabled. Drive has the equivalent cameras.
        attrs['if'] = '$(arg ' + ('camera' if stem == 'competition' else 'cameras') + ')'
        ET.SubElement(parent, 'node', attrs)
    elif kind == 'ExecuteProcess':
        points = action.kwargs['condition'].args[0].args[0] == 'bag_points'
        node = ET.SubElement(parent, 'node', {'pkg': 'ctrl_bringup', 'type': 'record_bag',
            'name': 'competition_bag_points' if points else 'competition_bag', 'output': 'log',
            'args': '--bag-dir "$(arg bag_dir)"' + (' --points' if points else '')})
        add_condition(node, action.kwargs['condition'])
    elif kind == 'RegisterEventHandler':
        pass
    else:
        raise ValueError('Unhandled launch action: ' + kind)


def write_xml(path, root):
    def indent(element, level=0):
        prefix = '\n' + '  ' * level
        if len(element):
            if not element.text or not element.text.strip():
                element.text = prefix + '  '
            for child in element:
                indent(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = prefix
        if level and (not element.tail or not element.tail.strip()):
            element.tail = prefix
    indent(root)
    write_file(path, '<?xml version="1.0"?>\n' + ET.tostring(root, encoding='unicode') + '\n')


def convert_launch(package, stem):
    actions = read_launch(package, stem + '.launch.py')
    arguments = [action for action in actions if type(action).__name__ == 'DeclareLaunchArgument']
    required = set()
    for action in actions:
        if type(action).__name__ == 'RegisterEventHandler':
            handler = action.args[0]
            if type(handler).__name__ == 'OnProcessExit' and 'target_action' in handler.kwargs:
                required.add(handler.kwargs['target_action'])
    root = ET.Element('launch')
    if stem == 'competition':
        ET.SubElement(root, 'param', {'name': '/ros1_port/competition/bag_name',
            'command': 'python3 "$(find ctrl_bringup)/scripts/ctrl_launch_info" bag-name'})
    for action in actions:
        render(root, action, required, package, stem, arguments)
    write_xml(WS / 'src' / package / 'launch' / (stem + '.launch'), root)


def metadata():
    for original in ORIGINAL.iterdir():
        if not (original / 'package.xml').is_file():
            continue
        name = package_name(original.name)
        destination = WS / 'src' / name
        source_xml = ET.parse(original / 'package.xml').getroot()
        package = ET.Element('package', {'format': '2'})
        for tag in ['name', 'version', 'description', 'maintainer', 'license']:
            package.append(source_xml.find(tag))
        ET.SubElement(package, 'buildtool_depend').text = 'catkin'
        if name == 'ctrl_interfaces':
            ET.SubElement(package, 'build_depend').text = 'message_generation'
            ET.SubElement(package, 'exec_depend').text = 'message_runtime'
            ET.SubElement(package, 'depend').text = 'std_msgs'
            write_file(destination / 'CMakeLists.txt', '''cmake_minimum_required(VERSION 3.0.2)
project(ctrl_interfaces)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_message_files(FILES VehicleStatus.msg ControlCommand.msg DrivingTarget.msg Event.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
''')
        else:
            depend = {'rospy', 'ctrl_ros1'}
            obsolete = {'rclpy', 'ament_index_python', 'launch_ros', 'sensor_msgs_py', 'rviz2'}
            depend.update(element.text for element in source_xml.findall('exec_depend') if element.text not in obsolete)
            if name == 'ctrl_bringup':
                depend.update({'ctrl_camera', 'roslaunch', 'rosbag', 'rviz', 'tf2_ros'})
            if name == 'ctrl_lidar':
                depend.update({'python3-matplotlib'})
            if name == 'ctrl_planning':
                depend.update({'geometry_msgs'})
            for item in sorted(depend):
                ET.SubElement(package, 'exec_depend').text = item
            write_file(destination / 'setup.py',
                "from setuptools import setup, find_packages\nfrom catkin_pkg.python_setup import generate_distutils_setup\n\n" +
                "setup(**generate_distutils_setup(packages=find_packages(include=['" + name + "', '" + name + ".*']), package_dir={'': '.'}))\n")
            cfg = destination / 'setup.cfg'
            if cfg.is_file():
                cfg.unlink()
            scripts = []
            for text in ast.walk(ast.parse((original / 'setup.py').read_text(encoding='utf-8'))):
                if isinstance(text, ast.Constant) and isinstance(text.value, str) and ' = ' in text.value:
                    executable, target = text.value.split(' = ')
                    module, function = target.split(':')
                    scripts.append(executable)
                    folder = destination / 'scripts'
                    folder.mkdir(exist_ok=True)
                    write_file(folder / executable, '#!/usr/bin/env python3\nfrom ' + module + ' import ' + function + '\n\nif __name__ == "__main__":\n    ' + function + '()\n')
            if name == 'ctrl_bringup':
                scripts += ['delay_wall', 'launch_info', 'record_bag']
            cmake = ['cmake_minimum_required(VERSION 3.0.2)', 'project(' + name + ')',
                     'find_package(catkin REQUIRED)', 'catkin_python_setup()', 'catkin_package()']
            if scripts:
                # Noetic catkin's shebang rewrite matches env python, not
                # env python3. Configure absolute interpreter shebangs so an
                # install-space launch cannot silently select system 3.8.
                cmake += ['set(_foscar_python_sources', *['  scripts/' + item for item in scripts], ')',
                          'set(_foscar_python_configured)',
                          'file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/python_scripts")',
                          'foreach(_foscar_script IN LISTS _foscar_python_sources)',
                          '  get_filename_component(_foscar_name "${_foscar_script}" NAME)',
                          '  set(_foscar_destination "${CMAKE_CURRENT_BINARY_DIR}/python_scripts/${_foscar_name}")',
                          '  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${_foscar_script}" "${_foscar_destination}" COPYONLY)',
                          '  file(READ "${_foscar_destination}" _foscar_contents)',
                          '  string(REGEX REPLACE "^#![^\\n]*" "#!${PYTHON_EXECUTABLE}" _foscar_contents "${_foscar_contents}")',
                          '  file(WRITE "${_foscar_destination}" "${_foscar_contents}")',
                          '  list(APPEND _foscar_python_configured "${_foscar_destination}")', 'endforeach()',
                          'catkin_install_python(PROGRAMS ${_foscar_python_configured}',
                          '  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})']
            for folder in ['config', 'maps', 'routes', 'models', 'rviz']:
                if (destination / folder).is_dir():
                    cmake.append('install(DIRECTORY ' + folder + '/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/' + folder + ')')
            if (destination / 'launch').is_dir():
                cmake.append('install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch FILES_MATCHING PATTERN "*.launch")')
            if name == 'ctrl_bringup':
                # XML param-command paths must also work in a catkin install
                # space, where rospack resolves share rather than source.
                cmake.append('install(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/python_scripts/ctrl_launch_info" DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)')
            for requirements in sorted(destination.glob('requirements*.txt')):
                cmake.append('install(FILES ' + requirements.name + ' DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})')
            write_file(destination / 'CMakeLists.txt', '\n'.join(cmake) + '\n')
        write_xml(destination / 'package.xml', package)


def rviz_resources():
    """Keep visualization settings, translating ROS2 plugin/topic schema only."""
    for original in ORIGINAL.glob('*/rviz/*.rviz'):
        lines = original.read_text(encoding='utf-8').splitlines()
        output = []
        index = 0
        while index < len(lines):
            line = lines[index]
            if line.strip() == 'Topic:':
                indent = len(line) - len(line.lstrip())
                fields = {}
                index += 1
                while index < len(lines) and len(lines[index]) - len(lines[index].lstrip()) > indent:
                    key, text = lines[index].strip().split(':', 1)
                    fields[key] = text.strip()
                    index += 1
                output.append(' ' * indent + 'Topic: ' + fields['Value'])
                if 'Depth' in fields:
                    output.append(' ' * indent + 'Queue Size: ' + fields['Depth'])
                continue
            output.append(line.replace('rviz_default_plugins/', 'rviz/').replace('rviz_common/', 'rviz/'))
            index += 1
        destination = WS / 'src' / original.relative_to(ORIGINAL)
        write_file(destination, '\n'.join(output) + '\n')


if __name__ == '__main__':
    metadata()
    rviz_resources()
    for pkg, launch in [('ctrl_bringup', 'competition'), ('ctrl_bringup', 'low_speed'),
                        ('ctrl_lidar', 'obstacle'), ('ctrl_lidar', 'drive'),
                        ('ctrl_camera', 'yolo_signal'), ('ctrl_camera', 'gps_record')]:
        convert_launch(pkg, launch)
