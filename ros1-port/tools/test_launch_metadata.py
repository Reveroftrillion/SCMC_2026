"""Verify launch metadata reaches recorders as paths without command newlines."""
import os
from pathlib import Path
import runpy
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import roslaunch.config
import roslaunch.xmlloader

PORT = Path(__file__).resolve().parents[1]


class LaunchMetadataTests(unittest.TestCase):
    def load(self, folder):
        config = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(str(PORT / 'ctrl_start.launch'), config,
            argv=['drive:=false', 'run_dir:=' + str(folder)], verbose=False)
        return {key: value.value for key, value in config.params.items()}

    def test_command_metadata_has_no_trailing_newline(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary) / 'record with spaces'
            values = self.load(folder)
            self.assertEqual(values['/ros1_port/ctrl_start/run_dir'], str(folder))
            self.assertRegex(values['/ros1_port/competition/bag_name'], r'^run_[0-9]{8}_[0-9]{6}\Z')
            for name, value in values.items():
                if name.endswith('/code_version'):
                    self.assertNotIn('\n', value)

    def test_recorder_receives_a_normal_bag_filename(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            values = self.load(folder)
            bag_name = values['/ros1_port/competition/bag_name']
            rosgraph = types.SimpleNamespace(Master=lambda _: types.SimpleNamespace(
                getParam=lambda _: bag_name))
            recorder = runpy.run_path(str(PORT / 'src/ctrl_bringup/scripts/ctrl_record_bag'))
            with patch.dict(sys.modules, rosgraph=rosgraph), \
                 patch.object(sys, 'argv', ['record_bag', '--bag-dir', str(folder)]), \
                 patch.object(os, 'execvp') as execute:
                recorder['main']()
            command, args = execute.call_args[0]
            self.assertEqual(command, 'rosbag')
            output = args[args.index('-O') + 1]
            self.assertEqual(output, str(folder / (bag_name.strip() + '.bag')))
            self.assertNotIn('\n', output)


if __name__ == '__main__':
    os.environ['ROS_PACKAGE_PATH'] = str(PORT / 'src') + ':/opt/ros/noetic/share'
    unittest.main()
