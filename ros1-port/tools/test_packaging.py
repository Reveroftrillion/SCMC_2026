"""Process-boundary checks that do not need ROS, a simulator, or real waits."""
import importlib.util
import os
from pathlib import Path
import runpy
import signal
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

TOOLS = Path(__file__).resolve().parent
WS = TOOLS.parent
SCRIPTS = WS / 'src/ctrl_bringup/scripts'
spec = importlib.util.spec_from_file_location('target_checker', TOOLS / 'check_noetic_environment.py')
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


class ReplacedProcess(Exception):
    pass


class PackagingTests(unittest.TestCase):
    def test_camera_delay_waits_five_wall_seconds_then_execs_scoped_launch(self):
        helper = runpy.run_path(str(SCRIPTS / 'ctrl_delay_wall'))
        args = ['delay_wall', '5.0', 'ctrl_bringup', 'ctrl_competition_delayed.launch', 'camera_port:=9208', '__name:=delay', '__log:=ignored']
        with patch.object(sys, 'argv', args), patch('time.monotonic', side_effect=[100., 100., 105.]), \
                patch('time.sleep') as sleep, patch('signal.signal'), \
                patch('os.execvp', side_effect=ReplacedProcess) as execute:
            with self.assertRaises(ReplacedProcess):
                helper['main']()
        sleep.assert_called_once_with(5.)
        execute.assert_called_once_with('roslaunch', ['roslaunch', 'ctrl_bringup', 'ctrl_competition_delayed.launch', 'camera_port:=9208'])

    def test_shutdown_during_camera_delay_prevents_child_start(self):
        helper = runpy.run_path(str(SCRIPTS / 'ctrl_delay_wall'))
        handlers = {}
        def remember(number, callback):
            handlers[number] = callback
        def interrupt(_seconds):
            handlers[signal.SIGINT](signal.SIGINT, None)
        with patch.object(sys, 'argv', ['delay_wall', '5', 'ctrl_bringup', 'ctrl_competition_delayed.launch']), \
                patch('signal.signal', side_effect=remember), patch('time.monotonic', return_value=100.), \
                patch('time.sleep', side_effect=interrupt), patch('os.execvp') as execute:
            with self.assertRaises(SystemExit) as result:
                helper['main']()
        self.assertEqual(result.exception.code, 128 + signal.SIGINT)
        execute.assert_not_called()

    def test_invalid_delay_is_not_silently_accepted(self):
        helper = runpy.run_path(str(SCRIPTS / 'ctrl_delay_wall'))
        for delay in ('-1', 'nan', 'inf'):
            with self.subTest(delay=delay), patch.object(sys, 'argv', ['delay_wall', delay, 'pkg', 'file.launch']), patch('os.execvp') as execute:
                with self.assertRaises(ValueError):
                    helper['main']()
                execute.assert_not_called()

    def test_bag_groups_use_one_stem_and_preserve_native_command_arguments(self):
        helper = runpy.run_path(str(SCRIPTS / 'ctrl_record_bag'))
        fake = types.ModuleType('rosgraph')
        fake.Master = lambda _name: types.SimpleNamespace(getParam=lambda _key: 'run_20261013_093000')
        with tempfile.TemporaryDirectory(prefix='foscar_bag_test_') as directory:
            for points in (False, True):
                args = ['record_bag', '--bag-dir', directory] + (['--points'] if points else []) + ['__name:=recorder', '__log:=ignored']
                with self.subTest(points=points), patch.dict(sys.modules, {'rosgraph': fake}), patch.object(sys, 'argv', args), patch('os.execvp', side_effect=ReplacedProcess) as execute:
                    with self.assertRaises(ReplacedProcess):
                        helper['main']()
                suffix = '_points' if points else ''
                output = str(Path(directory) / ('run_20261013_093000' + suffix + '.bag'))
                topics = helper['POINT_TOPICS'] if points else helper['TOPICS']
                execute.assert_called_once_with('rosbag', ['rosbag', 'record', '-O', output] + topics)

    def test_deployment_resource_hashes_match_source_copy(self):
        detail = checker.check_resources(lambda package: str(WS / 'src' / package))
        self.assertTrue(detail.startswith('22 '))

    def test_deployment_hash_check_rejects_same_length_modified_resource(self):
        read_bytes = Path.read_bytes
        def changed(path):
            data = read_bytes(path)
            if path.as_posix().endswith('ctrl_bringup/config/low_speed.yaml'):
                return bytes([data[0] ^ 1]) + data[1:]
            return data
        with patch.object(Path, 'read_bytes', changed):
            with self.assertRaisesRegex(RuntimeError, 'low_speed.yaml: SHA256 differs'):
                checker.check_resources(lambda package: str(WS / 'src' / package))

    def test_dependency_module_shadowing_is_rejected(self):
        wrong_numpy = types.SimpleNamespace(__version__='2.0.0', __file__='wrong/site-packages/numpy')
        with patch.object(checker.metadata, 'version', return_value='1.24.4'), \
                patch.object(checker.importlib, 'import_module', return_value=wrong_numpy):
            with self.assertRaisesRegex(RuntimeError, 'metadata/module versions disagree'):
                checker.check_pin('numpy')

    def test_wrong_launch_interpreter_is_rejected_even_when_module_imports_work(self):
        with tempfile.TemporaryDirectory(prefix='foscar_python_test_') as directory:
            selected = Path(directory) / 'python310'
            wrong = Path(directory) / 'python38'
            selected.touch()
            wrong.touch()
            script = Path(directory) / 'entrypoint'
            with patch.object(sys, 'executable', str(selected)):
                script.write_text('#!' + selected.as_posix() + '\n', encoding='utf-8')
                checker.check_script_interpreter(script)
                script.write_text('#!' + wrong.as_posix() + '\n', encoding='utf-8')
                with self.assertRaisesRegex(RuntimeError, 'would run a different Python interpreter'):
                    checker.check_script_interpreter(script)
                script.write_text('#!/usr/bin/env python3\n', encoding='utf-8')
                with patch.object(checker.shutil, 'which', return_value=str(wrong)):
                    with self.assertRaisesRegex(RuntimeError, 'would run a different Python interpreter'):
                        checker.check_script_interpreter(script)


if __name__ == '__main__':
    unittest.main()
