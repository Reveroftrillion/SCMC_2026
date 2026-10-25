"""Expand the real launch graph without starting nodes or binding UDP ports."""

import runpy
from pathlib import Path

import pytest
import yaml

pytest.importorskip('launch_ros')
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions


@pytest.mark.parametrize('custom_camera_config', [False, True])
@pytest.mark.parametrize('tag', ['foggy_13h', '123', 'true', '2026-09-22'])
@pytest.mark.parametrize('drive,check,stop_seconds,stopping', [
    ('false', 'true', '60', False), ('true', 'false', '60', True),
    ('true', 'false', '0', False), ('true', 'true', '60', False),
    ('false', 'false', '60', False)])
def test_record_preserves_drive_and_camera_parameters(monkeypatch, tmp_path,
                                                     custom_camera_config, tag,
                                                     drive, check, stop_seconds, stopping):
    package = Path(__file__).resolve().parents[1]
    camera_file = package / 'config/record.yaml'
    camera_params = yaml.safe_load(camera_file.read_text())
    if custom_camera_config:
        camera_params['camera_bridge']['ros__parameters']['port'] = 19208
        camera_file = tmp_path / 'camera.yaml'
        camera_file.write_text(yaml.safe_dump(camera_params))

    context = LaunchContext()
    context.launch_configurations.update(check=check, drive=drive, view='false', tag=tag,
                                         stop_seconds=stop_seconds)
    if custom_camera_config:
        context.launch_configurations['config'] = str(camera_file)
    received = {}
    generated_files = []

    def capture_process(action, ctx):
        # Node.execute has already resolved its parameters; only process startup is replaced.
        command = [perform_substitutions(ctx, part) for part in action.cmd]
        name = Path(command[0]).name
        for arg in command:
            if arg.startswith('__node:='):
                name = arg.split(':=', 1)[1]
        params = {}
        for i, arg in enumerate(command):
            if arg != '--params-file':
                continue
            path = Path(command[i + 1])
            data = yaml.safe_load(path.read_text())
            for key in ('/**', name, '/' + name):
                params.update(data.get(key, {}).get('ros__parameters', {}))
            if path.name.startswith('launch_params_'):
                generated_files.append(path)
        received[name] = params

    monkeypatch.setattr(ExecuteProcess, 'execute', capture_process)

    def visit(entity):
        for child in entity.visit(context) or []:
            visit(child)

    try:
        module = runpy.run_path(str(package / 'launch/ctrl_record.launch.py'))
        visit(module['generate_launch_description']())
        drive_file = Path(get_package_share_directory('ctrl_bringup')) / 'config/low_speed.yaml'
        drive_params = yaml.safe_load(drive_file.read_text())
        for node in ('sensor_bridge', 'localization', 'path_follower'):
            for key, value in drive_params[node]['ros__parameters'].items():
                if node == 'path_follower' and key == 'require_target' and stopping:
                    value = True
                assert received[node].get(key) == value, (node, key, received[node])
        assert received['sensor_bridge']['drive'] is (drive == 'true')
        assert received['path_follower']['continuous'] is True
        assert received['path_follower']['speed_kmh'] == 20.0
        for node in ('camera_bridge', 'camera_bridge_semantic'):
            assert received[node] == camera_params[node]['ros__parameters']
        assert received['recorder_node']['check'] is (check == 'true')
        assert received['recorder_node']['tag'] == tag
        assert received['recorder_node']['rgb_topic'] == camera_params['recorder_node']['ros__parameters']['rgb_topic']
        assert ('collection_stop_node' in received) is stopping
        if stopping:
            assert received['collection_stop_node']['stop_seconds'] == 60.
            assert received['collection_stop_node']['route'] == 'auto'
    finally:
        for path in generated_files:
            path.unlink(missing_ok=True)


@pytest.mark.parametrize('shutting_down,code,fails', [(False, 1, True), (False, 0, True),
                                                    (True, 0, False), (True, -2, False),
                                                    (True, -15, True)])
def test_unexpected_node_exit_is_reported_to_one_shot(tmp_path, shutting_down, code, fails):
    from launch.events.process import ProcessExited
    context = LaunchContext()
    context._set_is_shutdown(shutting_down)
    status = tmp_path / 'failure.txt'
    context.launch_configurations['failure_file'] = str(status)
    module = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'launch/ctrl_record.launch.py'))
    event = ProcessExited(action=ExecuteProcess(cmd=['false']), returncode=code,
                          name='recorder_node-6', cmd=['false'], cwd=None, env=None, pid=1)
    actions = module['record_process_exit'](event, context)
    assert status.exists() == fails
    if fails:
        assert 'recorder_node-6' in status.read_text()
    assert bool(actions) == (fails and not shutting_down)
