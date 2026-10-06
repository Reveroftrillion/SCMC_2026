#!/usr/bin/env python3
"""Fresh camera-only stop-line measurements and visual debugging."""
import copy
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Header
from simul_msgs.msg import StopLine
from stop_line_core import StopLineDetector, Confirmation, Detection


class StopLineNode:
    def __init__(self):
        self.lock = threading.Lock()
        self.enabled = rospy.get_param('~enabled', True)
        self.require_mission_active = rospy.get_param('~require_mission_active', False)
        if self.require_mission_active:
            self.enabled = False  # Wait for an explicit mission activation.
        self.timeout = float(rospy.get_param('~image_timeout', .5))
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('image_timeout must be positive')
        keys = ('roi', 'bev_size', 'white_v_min', 'white_s_max', 'min_coverage', 'min_band_height',
                'max_band_height', 'crosswalk_min_bands', 'crosswalk_max_gap', 'arrow_stem_min_length',
                'calibration_image_points', 'calibration_ground_points')
        options = {key: rospy.get_param('~'+key) for key in keys if rospy.has_param('~'+key)}
        self.detector = StopLineDetector(**options)
        self.confirm = Confirmation(rospy.get_param('~confirm_window', 5),
                                    rospy.get_param('~confirm_hits', 3),
                                    rospy.get_param('~max_y_jump', .08), self.timeout)
        self.bridge = CvBridge()
        self.last_image = float('-inf')
        self.last_stamp = None
        self.header = Header()
        self.publisher = rospy.Publisher('/stop_line/detection', StopLine, queue_size=1)
        self.image_pub = rospy.Publisher('/stop_line/image_raw', Image, queue_size=1)
        self.mask_pub = rospy.Publisher('/stop_line/mask', Image, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.1), self.watchdog)
        self.activation_sub = rospy.Subscriber('/traffic_mission_active', Bool, self.activation_callback,
                                              queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/front/image/compressed', CompressedImage,
                                         self.image_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo('Stop-line detector ready: enabled=%s, metric_calibration=%s',
                      self.enabled, self.detector.calibration_image is not None)

    def publish(self, result):
        message = StopLine()
        message.header = copy.deepcopy(self.header)
        message.enabled, message.candidate, message.detected = self.enabled, result.candidate, result.detected
        message.confidence, message.image_y_ratio = result.confidence, result.image_y_ratio
        message.distance_valid, message.distance_m, message.state = result.distance_valid, result.distance_m, result.state
        if result.endpoints:
            (message.x1, message.y1), (message.x2, message.y2) = result.endpoints
        else:
            message.x1 = message.y1 = message.x2 = message.y2 = float('nan')
        self.publisher.publish(message)

    def activation_callback(self, message):
        if not self.require_mission_active:
            return
        with self.lock:
            if self.enabled != bool(message.data):
                self.enabled = bool(message.data)
                self.confirm.clear()
                self.publish(Detection(state='WAITING_FOR_IMAGE' if self.enabled else 'DISABLED'))

    def watchdog(self, _event):
        with self.lock:
            if time.monotonic()-self.last_image > self.timeout:
                self.confirm.clear()
                self.publish(Detection(state='STALE_IMAGE' if self.enabled else 'DISABLED'))

    def image_callback(self, message):
        # Keep confirmation and mission activation atomic with frame processing.
        with self.lock:
            started = time.monotonic()
            self.header = copy.deepcopy(message.header)
            age = (rospy.Time.now()-message.header.stamp).to_sec()
            if (message.header.stamp.is_zero() or not -.1 <= age <= self.timeout or
                    (self.last_stamp is not None and message.header.stamp <= self.last_stamp)):
                self.confirm.clear()
                self.publish(Detection(state='STALE_IMAGE' if self.enabled else 'DISABLED'))
                return
            self.last_stamp = message.header.stamp
            try:
                frame = self.bridge.compressed_imgmsg_to_cv2(message, 'bgr8')
                self.last_image = started
                if self.enabled:
                    result, annotated, mask = self.detector.detect(frame)
                    result = self.confirm.update(result, started)
                else:
                    self.confirm.clear()
                    result, annotated, mask = Detection(state='DISABLED'), frame.copy(), None
                age = (rospy.Time.now()-message.header.stamp).to_sec()
                if not -.1 <= age <= self.timeout or time.monotonic()-started > self.timeout:
                    self.confirm.clear()
                    result = Detection(state='STALE_IMAGE' if self.enabled else 'DISABLED')
                self.publish(result)
                label = 'Stop line: {}'.format(result.state)
                if result.distance_valid:
                    label += ' {:.2f} m'.format(result.distance_m)
                cv2.putText(annotated, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            .7, (0, 255, 255), 2)
                image = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
                image.header = copy.deepcopy(message.header)
                self.image_pub.publish(image)
                if mask is not None:
                    mask_message = self.bridge.cv2_to_imgmsg(mask, 'mono8')
                    mask_message.header = copy.deepcopy(message.header)
                    self.mask_pub.publish(mask_message)
                rospy.loginfo_throttle(1., '[STOP LINE] state=%s coverage=%.2f image_y=%.3f distance_valid=%s',
                                       result.state, result.confidence, result.image_y_ratio, result.distance_valid)
            except (CvBridgeError, ValueError, RuntimeError, cv2.error) as exc:
                self.confirm.clear()
                self.publish(Detection(state='IMAGE_ERROR' if self.enabled else 'DISABLED'))
                rospy.logerr_throttle(2., 'Stop-line image failed: %s', exc)


def main():
    rospy.init_node('stop_line_detector')
    try:
        node = StopLineNode()
    except Exception as exc:
        rospy.logfatal('Stop-line startup failed: %s', exc)
        return 1
    rospy.spin()
    node.timer.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
