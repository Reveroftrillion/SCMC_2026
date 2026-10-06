#!/usr/bin/env python3
"""Stop-line geometry, crosswalk rejection, confirmation and camera freshness."""
import importlib.util
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import yaml

SCRIPTS = Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0, str(SCRIPTS))
from stop_line_core import StopLineDetector, Confirmation, Detection

RECTANGLE = [[0., 1.], [0., 0.], [1., 0.], [1., 1.]]


def road(bands=()):
    frame = np.full((300, 400, 3), 40, dtype=np.uint8)
    for row in bands:
        cv2.rectangle(frame, (20, row), (380, row+7), (255, 255, 255), -1)
    return frame


def direction_arrow(bands=()):
    frame = road(bands)
    cv2.rectangle(frame, (190, 115), (210, 250), (255, 255, 255), -1)
    cv2.rectangle(frame, (70, 170), (330, 178), (255, 255, 255), -1)
    for triangle in ([[70, 155], [40, 174], [70, 195]],
                     [[330, 155], [360, 174], [330, 195]],
                     [[175, 125], [200, 90], [225, 125]]):
        cv2.fillPoly(frame, [np.array(triangle, dtype=np.int32)], (255, 255, 255))
    return frame


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.detector = StopLineDetector(roi=RECTANGLE)

    def test_single_transverse_line_has_image_position_but_no_guessed_distance(self):
        result, image, mask = self.detector.detect(road([210]))
        self.assertTrue(result.candidate)
        self.assertFalse(result.detected)
        self.assertAlmostEqual(result.image_y_ratio, .713, delta=.02)
        self.assertGreater(result.confidence, .85)
        self.assertFalse(result.distance_valid)
        self.assertTrue(np.isnan(result.distance_m))
        self.assertEqual(image.shape, (300, 400, 3))
        self.assertEqual(mask.shape, (320, 320))

    def test_longitudinal_lanes_are_not_stop_lines(self):
        frame = road()
        for x in (90, 310):
            cv2.rectangle(frame, (x, 10), (x+7, 290), (255, 255, 255), -1)
        self.assertFalse(self.detector.detect(frame)[0].candidate)

    def test_repeated_crosswalk_bands_are_rejected(self):
        result = self.detector.detect(road([100, 130, 160, 190]))[0]
        self.assertFalse(result.candidate)
        self.assertEqual(result.state, 'CROSSWALK_PATTERN')

    def test_isolated_stop_line_before_crosswalk_survives(self):
        result = self.detector.detect(road([70, 100, 130, 240]))[0]
        self.assertTrue(result.candidate)
        self.assertGreater(result.image_y_ratio, .78)

    def test_broad_hood_or_white_object_is_rejected(self):
        frame = road()
        cv2.rectangle(frame, (0, 160), (399, 270), (255, 255, 255), -1)
        self.assertFalse(self.detector.detect(frame)[0].candidate)

    def test_short_marking_and_yellow_paint_are_rejected(self):
        for color, right in [((255, 255, 255), 170), ((0, 255, 255), 380)]:
            frame = road()
            cv2.rectangle(frame, (20, 200), (right, 208), color, -1)
            self.assertFalse(self.detector.detect(frame)[0].candidate)

    def test_thin_roi_edge_fragment_is_not_a_stop_line(self):
        frame = np.full((320, 320, 3), 40, dtype=np.uint8)
        frame[-1, 20:301] = 255
        self.assertFalse(self.detector.detect(frame)[0].candidate)

    def test_direction_arrow_transverse_branch_is_rejected(self):
        result, _, _ = self.detector.detect(direction_arrow())
        self.assertFalse(result.candidate)
        self.assertEqual(result.state, 'ARROW_PATTERN')

    def test_stop_line_ahead_of_direction_arrow_is_preserved(self):
        result, _, _ = self.detector.detect(direction_arrow([60]))
        self.assertTrue(result.candidate)
        self.assertAlmostEqual(result.image_y_ratio, .213, delta=.02)

    def test_stop_line_joining_lane_edges_is_preserved(self):
        frame = road([210])
        for x in (30, 355):
            cv2.rectangle(frame, (x, 50), (x+7, 290), (255, 255, 255), -1)
        self.assertTrue(self.detector.detect(frame)[0].candidate)

    def test_perspective_lane_roi_finds_a_transverse_band(self):
        frame = road([200])
        detector = StopLineDetector(roi=[[.1, .95], [.4, .3], [.6, .3], [.9, .95]])
        self.assertTrue(detector.detect(frame)[0].candidate)

    def test_metric_distance_requires_explicit_ground_correspondences(self):
        detector = StopLineDetector(roi=RECTANGLE, calibration_image_points=RECTANGLE,
                                    calibration_ground_points=[[5, 2], [20, 2], [20, -2], [5, -2]])
        result = detector.detect(road([210]))[0]
        self.assertTrue(result.distance_valid)
        self.assertAlmostEqual(result.distance_m, 9.3, delta=.2)

    def test_invalid_roi_or_ground_calibration_is_rejected(self):
        with self.assertRaises(ValueError):
            StopLineDetector(roi=[[0, 0], [1, 1], [1, 0], [0, 1]])
        with self.assertRaises(ValueError):
            StopLineDetector(calibration_image_points=RECTANGLE,
                             calibration_ground_points=[[0, 0], [1, 0], [2, 0], [3, 0]])


class Camera7RegressionTests(unittest.TestCase):
    def setUp(self):
        package = SCRIPTS.parent
        settings = yaml.safe_load((package/'config/stop_line.yaml').read_text())
        options = {key: value for key, value in settings.items()
                   if not key.startswith('camera_') and key not in
                   ('confirm_window', 'confirm_hits', 'max_y_jump', 'image_timeout')}
        self.detector = StopLineDetector(**options)
        self.data = Path(__file__).resolve().parent/'data'

    def test_near_stop_line_above_bonnet_is_confirmed(self):
        frame = cv2.imread(str(self.data/'camera7_stop_line_near.jpg'))
        confirmation = Confirmation()
        for now in (1., 1.05, 1.1):
            result = confirmation.update(self.detector.detect(frame)[0], now)
        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.image_y_ratio, .583, delta=.015)

    def test_stop_line_ahead_of_arrow_is_selected(self):
        frame = cv2.imread(str(self.data/'camera7_stop_line_approach.jpg'))
        result = self.detector.detect(frame)[0]
        self.assertTrue(result.candidate)
        self.assertAlmostEqual(result.image_y_ratio, .457, delta=.02)

    def test_distant_curb_in_scene_without_stop_line_is_rejected(self):
        frame = cv2.imread(str(self.data/'camera7_road_without_stop_line.jpg'))
        self.assertFalse(self.detector.detect(frame)[0].candidate)

    def test_full_arrow_branch_is_ignored_and_actual_stop_line_is_selected(self):
        frame = cv2.imread(str(self.data/'camera7_arrow_before_stop_line.jpg'))
        result = self.detector.detect(frame)[0]
        self.assertTrue(result.candidate)
        self.assertAlmostEqual(result.image_y_ratio, .432, delta=.015)


class FrontCameraRegressionTests(unittest.TestCase):
    def setUp(self):
        settings = yaml.safe_load((SCRIPTS.parent/'config/stop_line_front.yaml').read_text())
        keys = ('roi', 'bev_size', 'white_v_min', 'white_s_max', 'min_coverage', 'min_band_height',
                'max_band_height', 'crosswalk_min_bands', 'crosswalk_max_gap', 'arrow_stem_min_length',
                'calibration_image_points', 'calibration_ground_points')
        self.detector = StopLineDetector(**{key: settings[key] for key in keys})

    def test_saved_front_road_without_line_does_not_create_a_stop(self):
        frame = cv2.imread(str(Path(__file__).parent/'data/front_road_without_stop_line.png'))
        self.assertFalse(self.detector.detect(frame)[0].candidate)

    def test_front_roi_confirms_a_horizontal_line_and_preserves_original_y(self):
        frame = np.full((720, 1280, 3), 40, dtype=np.uint8)
        cv2.rectangle(frame, (100, 460), (1180, 468), (255, 255, 255), -1)
        confirmation = Confirmation()
        for now in (1., 1.05, 1.1):
            result = confirmation.update(self.detector.detect(frame)[0], now)
        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.image_y_ratio, .644, delta=.01)
        self.assertFalse(result.distance_valid)

    def test_front_roi_rejects_a_transverse_arrow_branch(self):
        frame = np.full((720, 1280, 3), 40, dtype=np.uint8)
        cv2.rectangle(frame, (660, 414), (680, 553), (255, 255, 255), -1)
        cv2.rectangle(frame, (447, 489), (862, 497), (255, 255, 255), -1)
        result = self.detector.detect(frame)[0]
        self.assertFalse(result.candidate)
        self.assertEqual(result.state, 'ARROW_PATTERN')


class ConfirmationTests(unittest.TestCase):
    def hit(self, y=.6):
        return Detection(candidate=True, image_y_ratio=y, distance_valid=True, distance_m=5.)

    def test_three_hits_in_five_confirm_only_a_current_candidate(self):
        tracker = Confirmation()
        self.assertFalse(tracker.update(self.hit(), 1.).detected)
        self.assertFalse(tracker.update(Detection(), 1.05).detected)
        self.assertFalse(tracker.update(self.hit(), 1.10).detected)
        self.assertFalse(tracker.update(Detection(), 1.15).detected)
        self.assertTrue(tracker.update(self.hit(), 1.20).detected)
        lost = tracker.update(Detection(), 1.25)
        self.assertFalse(lost.detected)
        self.assertFalse(lost.distance_valid)

    def test_candidate_jump_does_not_reuse_another_lines_confirmation(self):
        tracker = Confirmation()
        for now in (1., 1.05, 1.1):
            tracker.update(self.hit(), now)
        changed = tracker.update(self.hit(.8), 1.15)
        self.assertFalse(changed.detected)
        self.assertFalse(changed.distance_valid)

    def test_camera_gap_or_time_reversal_clears_confirmation(self):
        for resume in (2., .9):
            tracker = Confirmation()
            for now in (1., 1.05, 1.1):
                tracker.update(self.hit(), now)
            self.assertFalse(tracker.update(self.hit(), resume).detected)


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)


class NodeTests(unittest.TestCase):
    def setUp(self):
        import stop_line
        from std_msgs.msg import Header
        self.module = stop_line
        self.node = stop_line.StopLineNode.__new__(stop_line.StopLineNode)
        self.node.lock = threading.Lock()
        self.node.enabled = True
        self.node.require_mission_active = True
        self.node.timeout = .5
        self.node.detector = StopLineDetector(roi=RECTANGLE)
        self.node.confirm = Confirmation()
        self.node.bridge = stop_line.CvBridge()
        self.node.last_stamp = None
        self.node.last_image = time.monotonic()
        self.node.header = Header()
        self.node.publisher, self.node.image_pub, self.node.mask_pub = Publisher(), Publisher(), Publisher()
        for name in ('loginfo_throttle', 'logerr_throttle'):
            logging = patch.object(stop_line.rospy, name)
            logging.start()
            self.addCleanup(logging.stop)

    def frame(self, stamp):
        message = self.module.CompressedImage()
        message.header.stamp = self.module.rospy.Time.from_sec(stamp)
        message.header.frame_id = 'camera_stop_line'
        message.format = 'jpeg'
        message.data = cv2.imencode('.jpg', road([210]))[1].tobytes()
        with patch.object(self.module.rospy.Time, 'now', return_value=self.module.rospy.Time.from_sec(stamp+.01)):
            self.node.image_callback(message)

    def test_current_frame_header_and_detection_are_published(self):
        for stamp in (1., 1.05, 1.1): self.frame(stamp)
        result = self.node.publisher.messages[-1]
        self.assertTrue(result.detected)
        self.assertFalse(result.distance_valid)
        self.assertEqual(result.header, self.node.image_pub.messages[-1].header)
        self.assertEqual(result.header.frame_id, 'camera_stop_line')

    def test_deactivation_and_reactivation_require_new_confirmations(self):
        from std_msgs.msg import Bool
        for stamp in (1., 1.05, 1.1): self.frame(stamp)
        self.node.activation_callback(Bool(data=False))
        self.assertEqual(self.node.publisher.messages[-1].state, 'DISABLED')
        self.frame(1.15)
        self.assertFalse(self.node.publisher.messages[-1].detected)
        self.node.activation_callback(Bool(data=True))
        self.frame(1.2)
        self.assertFalse(self.node.publisher.messages[-1].detected)

    def test_stale_and_duplicate_frames_cannot_confirm_detection(self):
        message = self.module.CompressedImage()
        message.header.stamp = self.module.rospy.Time.from_sec(1.)
        with patch.object(self.module.rospy.Time, 'now', return_value=self.module.rospy.Time.from_sec(2.)):
            self.node.image_callback(message)
        self.assertEqual(self.node.publisher.messages[-1].state, 'STALE_IMAGE')
        self.frame(2.)
        self.frame(2.)
        self.assertEqual(self.node.publisher.messages[-1].state, 'STALE_IMAGE')
        self.assertFalse(self.node.publisher.messages[-1].detected)

    def test_camera_watchdog_clears_a_previous_detection(self):
        for stamp in (1., 1.05, 1.1): self.frame(stamp)
        self.node.last_image = time.monotonic()-1.
        self.node.watchdog(None)
        self.assertEqual(self.node.publisher.messages[-1].state, 'STALE_IMAGE')
        self.assertFalse(self.node.publisher.messages[-1].detected)


if __name__ == '__main__':
    unittest.main()
