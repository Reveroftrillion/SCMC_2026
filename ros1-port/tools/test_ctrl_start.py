"""Load competition launch configs without starting ROS nodes or sending UDP."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import roslaunch.config

WS = Path(__file__).resolve().parents[1]


class CtrlStartTests(unittest.TestCase):
    def load(self, directory, *arguments):
        config = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            str(WS / 'ctrl_start.launch'), config,
            argv=['run_dir:=' + str(directory)] + list(arguments), verbose=False)
        return config

    def test_default_drives_and_preserves_first_lap_ports_and_recorders(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary) / 'run with spaces'
            config = self.load(run)
            values = {key: value.value for key, value in config.params.items()}
            self.assertTrue(run.is_dir())
            self.assertIs(values['/sensor_bridge/drive'], True)
            for node in ('localization', 'path_follower', 'behavior_node'):
                self.assertIs(values['/' + node + '/start_at_beginning'], True)
            for name, port in [('status_port', 909), ('gps_port', 1111),
                               ('imu_port', 1112), ('ctrl_port', 9093)]:
                self.assertEqual(values['/sensor_bridge/' + name], port)
            self.assertEqual(values['/lidar_bridge/port'], 2368)
            self.assertEqual(values['/path_follower/log_dir'], str(run / 'follower'))
            names = {node.name for node in config.nodes}
            self.assertTrue({'competition_bag', 'manual_sensor_bag',
                             'competition_camera_delay'}.issubset(names))
            raw = next(node for node in config.nodes if node.name == 'manual_sensor_bag')
            self.assertIn('"' + str(run / 'raw.bag') + '"', raw.args)
            self.assertIn('camera_port:=9297', next(
                node.args for node in config.nodes if node.name == 'competition_camera_delay'))

    def test_receive_only_and_recording_switches(self):
        with tempfile.TemporaryDirectory() as temporary:
            config = self.load(Path(temporary), 'drive:=false', 'bag:=false')
            self.assertIs(config.params['/sensor_bridge/drive'].value, False)
            self.assertFalse({'competition_bag', 'manual_sensor_bag'} &
                             {node.name for node in config.nodes})
            config = self.load(Path(temporary), 'raw:=false')
            self.assertIn('competition_bag', {node.name for node in config.nodes})
            self.assertNotIn('manual_sensor_bag', {node.name for node in config.nodes})

    def test_legacy_wrapper_defaults_to_receive_only_and_calls_shared_launch(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            shutil.copy2(WS / 'docker/run_course.sh', folder / 'run_course.sh')
            (folder / 'run.sh').write_text('#!/bin/bash\nprintf "%s\\n" "$@"\n')
            (folder / 'console_filter.py').write_text('import sys\nprint(sys.stdin.read())\n')
            for args, expected in [([], 'drive:=false'), (['drive:=true'], 'drive:=true')]:
                result = subprocess.run(['bash', str(folder / 'run_course.sh')] + args,
                                        check=True, capture_output=True, text=True)
                self.assertIn('ctrl_start.launch', result.stdout)
                self.assertEqual(result.stdout.count(expected), 1)
                self.assertNotIn('drive:=false' if args else 'drive:=true', result.stdout)
                self.assertFalse(list(folder.rglob('manual.launch')))


if __name__ == '__main__':
    os.environ['ROS_PACKAGE_PATH'] = str(WS / 'src') + ':/opt/ros/noetic/share'
    unittest.main()
