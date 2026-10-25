"""Check camera entry-point cleanup without opening sockets or loading a model."""
import unittest
from unittest.mock import Mock, patch

from ctrl_camera import ctrl_camera_bridge as camera_bridge, ctrl_yolo_signal_node as yolo_signal_node
from ctrl_ros1.ctrl_executors import ExternalShutdownException


class CameraShutdownTests(unittest.TestCase):
    def run_main(self, module, factory, error, cleanup_error=None):
        node = Mock()
        node.destroy_node.side_effect = cleanup_error
        with patch.object(module, factory, return_value=node), \
             patch.object(module.rclpy, 'init'), \
             patch.object(module.rclpy, 'ok', return_value=True), \
             patch.object(module.rclpy, 'spin', side_effect=error), \
             patch.object(module.rclpy, 'shutdown') as shutdown:
            try:
                module.main()
            finally:
                node.destroy_node.assert_called_once_with()
                if cleanup_error is None:
                    shutdown.assert_called_once_with()

    def test_normal_stop_cleans_up_without_error(self):
        for module, factory in ((camera_bridge, 'CameraBridge'), (yolo_signal_node, 'YoloSignalNode')):
            for error in (ExternalShutdownException(), KeyboardInterrupt()):
                with self.subTest(module=module.__name__, error=type(error).__name__):
                    self.run_main(module, factory, error)

    def test_unexpected_runtime_error_is_still_reported(self):
        for module, factory in ((camera_bridge, 'CameraBridge'), (yolo_signal_node, 'YoloSignalNode')):
            with self.subTest(module=module.__name__):
                with self.assertRaisesRegex(RuntimeError, 'unexpected camera fault'):
                    self.run_main(module, factory, RuntimeError('unexpected camera fault'))

    def test_cleanup_error_is_still_reported(self):
        for module, factory in ((camera_bridge, 'CameraBridge'), (yolo_signal_node, 'YoloSignalNode')):
            with self.subTest(module=module.__name__):
                with self.assertRaisesRegex(RuntimeError, 'cleanup failed'):
                    self.run_main(module, factory, ExternalShutdownException(), RuntimeError('cleanup failed'))


if __name__ == '__main__':
    unittest.main()
