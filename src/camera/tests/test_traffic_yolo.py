#!/usr/bin/env python3
"""YOLO adapter, source timestamps and public ROS outputs without a model/master."""
import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

spec = importlib.util.spec_from_file_location('traffic_yolo', Path(__file__).resolve().parents[1]/'scripts/traffic_yolo.py')
yolo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(yolo)


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, msg): self.messages.append(msg)


class FrameTests(unittest.TestCase):
    def setUp(self):
        for name in ('loginfo_throttle', 'logerr_throttle'):
            log_patch = patch.object(rospy, name)
            log_patch.start()
            self.addCleanup(log_patch.stop)
        self.node = yolo.TrafficYoloNode.__new__(yolo.TrafficYoloNode)
        self.node.bridge = CvBridge()
        self.node.confidence, self.node.device = .4, 'cpu'
        self.node.roi = [0., 0., 1., 1.]
        self.node.timeout = self.node.max_image_age = .7
        self.node.last_result = float('-inf')
        self.node.lock, self.node.processing_lock = threading.RLock(), threading.Lock()
        self.node.tracker = yolo.SignalTracker()
        self.node.mission = yolo.CameraMission(enabled=False)
        self.node.header = CompressedImage().header
        self.node.last_input_stamp = None
        self.node.last_detection_stamp = rospy.Time(0)
        self.node.image_width = self.node.image_height = 0
        self.node.last_frame = self.node.frame_header = None
        for name in ('status_pub', 'raw_pub', 'track_pub', 'mission_pub',
                     'active_pub', 'state_pub', 'image_pub'):
            setattr(self.node, name, Publisher())
        self.boxes = []
        self.node.model = SimpleNamespace(predict=lambda *a, **kw: [SimpleNamespace(
            boxes=self.boxes, names={0:'4red',1:'4greenleft',2:'car',3:'3red',
                                    4:'4green',5:'4redleft',6:'4redyellow',
                                    7:'3redleft',8:'3redyellow',9:'4yellow'})])
        self.msg = CompressedImage()
        self.msg.format = 'jpeg'
        self.msg.data = cv2.imencode('.jpg', np.zeros((32,32,3), dtype=np.uint8))[1].tobytes()
        self.clock = 1.

    def feed(self, count=1):
        for _ in range(count):
            self.clock += .05
            self.msg.header.stamp = rospy.Time.from_sec(100+self.clock)
            self.callback()

    def callback(self):
        with patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(100+self.clock)), \
             patch.object(yolo.time, 'monotonic', side_effect=lambda: self.clock):
            self.node.image_callback(self.msg)

    def set_class(self, cls=0, confidence=.8):
        self.boxes[:] = [SimpleNamespace(cls=[cls], conf=[confidence], xyxy=[[2,2,20,20]])]

    def test_empty_detections_publish_unknown_and_image(self):
        self.feed()
        self.assertEqual(self.node.status_pub.messages[-1].data, 'UNKNOWN')
        self.assertEqual(len(self.node.image_pub.messages), 1)
        self.assertFalse(self.node.track_pub.messages[-1].bbox_valid)

    def test_filters_non_traffic_objects_and_keeps_image_header(self):
        self.msg.header.frame_id = 'camera_front'
        self.boxes.extend([SimpleNamespace(cls=[2],conf=[.99],xyxy=[[1,1,10,10]]),
                           SimpleNamespace(cls=[0],conf=[.8],xyxy=[[2,2,20,20]])])
        self.feed(3)
        self.assertEqual(self.node.status_pub.messages[-1].data, '4red')
        self.assertEqual(self.node.image_pub.messages[-1].header, self.msg.header)
        self.assertEqual(self.node.track_pub.messages[-1].detection_count, 1)

    def test_three_lamp_signal_is_displayed_and_published(self):
        self.boxes.extend([SimpleNamespace(cls=[0],conf=[.7],xyxy=[[2,2,20,20]]),
                           SimpleNamespace(cls=[3],conf=[.9],xyxy=[[1,1,10,10]])])
        with patch.object(cv2, 'putText', wraps=cv2.putText) as draw:
            self.feed(3)
        self.assertEqual(self.node.status_pub.messages[-1].data, '3red')
        self.assertTrue(any(call.args[1].startswith('3red ') for call in draw.call_args_list))

    def test_every_trained_traffic_class_is_published_after_confirmation(self):
        for cls, name in self.node.model.predict()[0].names.items():
            if name == 'car': continue
            with self.subTest(label=name):
                self.node.tracker.clear()
                self.set_class(cls)
                self.feed(3)
                self.assertEqual(self.node.status_pub.messages[-1].data, name)
                self.assertEqual(self.node.raw_pub.messages[-1].data, name)

    def test_stale_and_zero_stamp_images_cannot_publish_go(self):
        self.set_class(1)
        self.msg.header.stamp = rospy.Time.from_sec(1.)
        self.callback()
        self.assertEqual(self.node.status_pub.messages[-1].data, 'UNKNOWN')
        self.msg.header.stamp = rospy.Time(0)
        self.callback()
        self.assertFalse(self.node.track_pub.messages[-1].signal_valid)

    def test_watchdog_expires_old_go_and_updates_debug_image(self):
        self.set_class(1)
        self.feed(3)
        self.clock += 1.
        with patch.object(yolo.time, 'monotonic', return_value=self.clock):
            self.node.watchdog(None)
        self.assertEqual(self.node.status_pub.messages[-1].data, 'UNKNOWN')
        self.assertEqual(self.node.track_pub.messages[-1].state, 'STALE_IMAGE')
        self.assertFalse(self.node.track_pub.messages[-1].bbox_valid)
        self.assertEqual(self.node.image_pub.messages[-1].header, self.msg.header)

    def test_model_error_invalidates_previous_go(self):
        self.set_class(1)
        self.feed(3)
        def fail(*args, **kwargs): raise RuntimeError('inference failed')
        self.node.model.predict = fail
        self.feed()
        self.assertEqual(self.node.status_pub.messages[-1].data, 'UNKNOWN')
        self.assertEqual(self.node.track_pub.messages[-1].state, 'IMAGE_ERROR')

    def test_short_miss_holds_filtered_signal_but_preserves_original_detection_stamp(self):
        self.set_class()
        self.feed(3)
        original_stamp = self.node.track_pub.messages[-1].last_detection_stamp
        self.boxes.clear()
        self.feed()
        result = self.node.track_pub.messages[-1]
        self.assertEqual(self.node.raw_pub.messages[-1].data, 'UNKNOWN')
        self.assertEqual(self.node.status_pub.messages[-1].data, '4red')
        self.assertTrue(result.held)
        self.assertFalse(result.observed)
        self.assertEqual(result.last_detection_stamp, original_stamp)
        self.assertGreater(result.header.stamp, original_stamp)

    def test_long_miss_invalidates_previous_signal(self):
        self.set_class(1)
        self.feed(3)
        self.boxes.clear()
        self.clock += .5
        self.feed()
        self.assertEqual(self.node.status_pub.messages[-1].data, 'UNKNOWN')
        self.assertFalse(self.node.track_pub.messages[-1].bbox_valid)

    def test_one_red_observation_cancels_previous_go(self):
        self.set_class(1)
        self.feed(3)
        self.set_class(0)
        self.feed()
        self.assertEqual(self.node.status_pub.messages[-1].data, '4red')

    def test_duplicate_source_frames_reset_confirmation(self):
        self.set_class(1)
        self.feed()
        self.clock += .01
        self.callback()  # Same source stamp.
        self.assertEqual(self.node.track_pub.messages[-1].state, 'STALE_IMAGE')
        self.feed(2)
        self.assertEqual(self.node.status_pub.messages[-1].data, 'UNKNOWN')

    def test_roi_box_ratios_refer_to_full_image(self):
        self.node.roi = [.25,.25,.75,.75]
        self.boxes[:] = [SimpleNamespace(cls=[0],conf=[.8],xyxy=[[2,2,10,10]])]
        self.feed(3)
        result = self.node.track_pub.messages[-1]
        self.assertAlmostEqual(result.raw_width_ratio, .25)
        self.assertAlmostEqual(result.xmin, .3125)
        self.assertEqual(result.image_width, 32)

    def test_slow_inference_cannot_restore_previous_go(self):
        self.set_class(1)
        self.feed(3)
        original_predict = self.node.model.predict
        def slow(*args, **kwargs):
            self.clock += 1.
            return original_predict()
        self.node.model.predict = slow
        self.feed()
        self.assertEqual(self.node.status_pub.messages[-1].data, 'UNKNOWN')
        self.assertEqual(self.node.track_pub.messages[-1].state, 'STALE_IMAGE')

    def test_camera_failure_keeps_active_mission_latched(self):
        self.node.mission = yolo.CameraMission()
        self.set_class()
        self.feed(12)
        self.assertTrue(self.node.active_pub.messages[-1].data)
        self.clock += 1.
        with patch.object(yolo.time, 'monotonic', return_value=self.clock):
            self.node.watchdog(None)
        self.assertTrue(self.node.active_pub.messages[-1].data)
        self.assertEqual(self.node.mission_pub.messages[-1].phase, 'ACTIVE_STALE')


if __name__ == '__main__': unittest.main()
