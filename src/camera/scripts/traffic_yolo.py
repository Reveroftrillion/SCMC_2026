#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO, target filtering and camera-only traffic-mission boundaries."""
import copy
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2
import rospkg
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String
from simul_msgs.msg import CameraTrafficMission, TrafficLightTrack
from traffic_camera_core import CameraMission, Observation, SignalTracker, TRAFFIC_CLASSES

MISSION_CLASSES = {'4red', '4yellow', '4greenleft'}


class TrafficYoloNode:
    def __init__(self):
        default_weights = os.path.join(rospkg.RosPack().get_path('camera'), 'models', '1027_40epoch.pt')
        weights = os.path.abspath(os.path.expanduser(rospy.get_param('~weights_path', default_weights)))
        if not os.path.isfile(weights):
            raise RuntimeError('YOLO weights not found: {}. Set weights_path / yolo_weights.'.format(weights))
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError('Install camera/requirements-yolo.txt in your Python environment: {}'.format(exc))
        self.device = rospy.get_param('~device', 'cpu')
        self.confidence = float(rospy.get_param('~confidence', .4))
        self.timeout = float(rospy.get_param('~signal_timeout', .7))
        self.max_image_age = float(rospy.get_param('~max_image_age', .7))
        self.roi = [float(value) for value in rospy.get_param('~roi', [0., 0., 1., 1.])]
        if (not 0 < self.confidence <= 1 or
                not all(math.isfinite(v) and v > 0 for v in (self.timeout, self.max_image_age))):
            raise ValueError('confidence must be in (0,1]; timeouts must be positive')
        if len(self.roi) != 4 or not (0 <= self.roi[0] < self.roi[2] <= 1 and 0 <= self.roi[1] < self.roi[3] <= 1):
            raise ValueError('roi must be normalized [xmin, ymin, xmax, ymax]')
        self.tracker = SignalTracker(**rospy.get_param('~signal_filter', {}))
        if self.tracker.hold_time >= self.timeout:
            raise ValueError('signal_filter/hold_time must be shorter than signal_timeout')
        self.mission = CameraMission(**rospy.get_param('~camera_mission', {}))
        self.model = YOLO(weights)
        self.model.to(self.device)
        names = self.model.names
        names = set(names.values() if isinstance(names, dict) else names)
        if not MISSION_CLASSES.issubset(names):
            raise ValueError('Model lacks required mission classes: {}'.format(sorted(MISSION_CLASSES-names)))
        self.bridge = CvBridge()
        self.lock, self.processing_lock = threading.RLock(), threading.Lock()
        self.last_result = float('-inf')
        self.last_input_stamp = None
        self.header = CompressedImage().header
        self.last_detection_stamp = rospy.Time(0)
        self.image_width = self.image_height = 0
        self.last_frame = self.frame_header = None
        self.status_pub = rospy.Publisher('/traffic_light_status', String, queue_size=1)
        self.raw_pub = rospy.Publisher('/traffic_light_status/raw', String, queue_size=1)
        self.track_pub = rospy.Publisher('/traffic_light/tracked', TrafficLightTrack, queue_size=1)
        self.mission_pub = rospy.Publisher('/traffic_camera_mission', CameraTrafficMission, queue_size=1)
        self.active_pub = rospy.Publisher('/traffic_mission_active', Bool, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher('/traffic_camera_mission_state', String, queue_size=1, latch=True)
        self.image_pub = rospy.Publisher('/yolo_traffic_light/image_raw', Image, queue_size=1)
        self.publish(self.tracker.snapshot(time.monotonic()), self.mission.result())
        self.timer = rospy.Timer(rospy.Duration(.1), self.watchdog)
        self.reset_sub = rospy.Subscriber('/traffic_camera_mission/reset', Bool, self.reset_callback, queue_size=1)
        self.passed_sub = rospy.Subscriber('/traffic_camera_mission/passed', Bool, self.passed_callback, queue_size=1)
        self.subscriber = rospy.Subscriber('/camera/traffic/image/compressed', CompressedImage,
                                           self.image_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo('Traffic YOLO loaded: %s, device=%s, classes=%s, mission_exit=%s',
                      weights, self.device, sorted(names), self.mission.exit_mode)

    def image_fresh(self, message):
        if message.header.stamp.is_zero(): return False
        age = (rospy.Time.now()-message.header.stamp).to_sec()
        return -.1 <= age <= self.max_image_age

    def publish(self, signal, mission, count=0):
        if signal.observed: self.last_detection_stamp = self.header.stamp
        elif not signal.track_id: self.last_detection_stamp = rospy.Time(0)
        track = TrafficLightTrack()
        track.header = copy.deepcopy(self.header)
        track.last_detection_stamp = self.last_detection_stamp
        track.track_id, track.detection_count = signal.track_id, count
        track.image_width, track.image_height = self.image_width, self.image_height
        track.raw_label, track.label, track.state = signal.raw_label, signal.label, signal.state
        track.observed, track.held = signal.observed, signal.held
        track.signal_valid, track.bbox_valid = signal.valid, signal.bbox is not None
        track.confidence = signal.confidence
        track.xmin, track.ymin, track.xmax, track.ymax = signal.bbox or (float('nan'),)*4
        track.width_ratio, track.height_ratio = signal.width, signal.height
        track.area_ratio = signal.width*signal.height
        track.raw_width_ratio = signal.raw_bbox[2]-signal.raw_bbox[0] if signal.raw_bbox else float('nan')
        track.raw_height_ratio = signal.raw_bbox[3]-signal.raw_bbox[1] if signal.raw_bbox else float('nan')
        track.detection_age, track.signal_age = signal.detection_age, signal.signal_age
        state = CameraTrafficMission()
        state.header = copy.deepcopy(self.header)
        state.mission_id, state.target_track_id = mission.mission_id, mission.target_track_id
        state.active, state.phase, state.reason = mission.active, mission.phase, mission.reason
        state.enter_threshold_width_ratio = self.mission.enter_width
        state.entry_width_ratio, state.peak_width_ratio = mission.entry_width, mission.peak_width
        state.exit_armed = mission.exit_armed
        self.raw_pub.publish(String(data=signal.raw_label))
        self.status_pub.publish(String(data=signal.label))
        self.track_pub.publish(track)
        self.mission_pub.publish(state)
        self.active_pub.publish(Bool(data=mission.active))
        self.state_pub.publish(String(data='M{}:{}'.format(mission.mission_id, mission.phase)))

    def summary_image(self, frame, header, signal, mission):
        if signal.bbox:
            height, width = frame.shape[:2]
            x0, y0, x1, y1 = (int(value*scale) for value, scale in zip(signal.bbox, (width, height, width, height)))
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 200, 255) if signal.held else (255, 200, 0), 3)
        lines = ['Raw: {}  Signal: {} [{}]'.format(signal.raw_label, signal.label, signal.state),
                 'Target width: {:.2f}%  Start >= {:.2f}%  Track: {}'.format(
                     100*signal.width, 100*self.mission.enter_width, signal.track_id),
                 'Mission M{}: {}'.format(mission.mission_id, mission.phase)]
        for index, label in enumerate(lines):
            cv2.putText(frame, label, (10, 28+index*25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
        image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        image.header = copy.deepcopy(header)
        self.image_pub.publish(image)

    def invalidate(self, now, reason='STALE_IMAGE'):
        signal, mission = self.tracker.fault(now), self.mission.fault(now)
        signal.state = reason
        self.publish(signal, mission)
        if self.last_frame is not None:
            self.summary_image(self.last_frame.copy(), self.frame_header, signal, mission)

    def watchdog(self, _event):
        with self.lock:
            if time.monotonic()-self.last_result > self.timeout:
                self.invalidate(time.monotonic())

    def reset_callback(self, message):
        if not message.data: return
        with self.lock:
            self.tracker.clear()
            self.mission.reset()
            self.last_detection_stamp = rospy.Time(0)
            self.publish(self.tracker.snapshot(time.monotonic()), self.mission.result(reason='MANUAL_RESET'))

    def passed_callback(self, message):
        if not message.data: return
        with self.lock:
            now = time.monotonic()
            self.publish(self.tracker.snapshot(now), self.mission.complete(now))

    def image_callback(self, message):
        # Inference can run slowly without blocking the freshness watchdog.
        with self.processing_lock:
            started = time.monotonic()
            with self.lock:
                self.header = copy.deepcopy(message.header)
                if (not self.image_fresh(message) or
                        (self.last_input_stamp is not None and message.header.stamp <= self.last_input_stamp)):
                    self.invalidate(started)
                    return
                self.last_input_stamp = message.header.stamp
            try:
                frame = self.bridge.compressed_imgmsg_to_cv2(message, 'bgr8')
                height, width = frame.shape[:2]
                with self.lock:
                    self.image_width, self.image_height = width, height
                    self.last_frame, self.frame_header = frame.copy(), copy.deepcopy(message.header)
                x0, y0 = int(self.roi[0]*width), int(self.roi[1]*height)
                x1, y1 = int(self.roi[2]*width), int(self.roi[3]*height)
                crop = frame[y0:y1, x0:x1]
                if not crop.size: raise ValueError('ROI is empty for this image')
                prediction = self.model.predict(crop, conf=self.confidence, verbose=False, device=self.device)[0]
                observations = []
                for box in prediction.boxes:
                    name, confidence = prediction.names[int(box.cls[0])], float(box.conf[0])
                    if name not in TRAFFIC_CLASSES or confidence < self.confidence: continue
                    bx0, by0, bx1, by1 = (float(value) for value in box.xyxy[0])
                    bounds = (max(0., min(1., (bx0+x0)/width)), max(0., min(1., (by0+y0)/height)),
                              max(0., min(1., (bx1+x0)/width)), max(0., min(1., (by1+y0)/height)))
                    observation = Observation(name, confidence, bounds)
                    if not observation.valid(): continue
                    observations.append(observation)
                    cv2.rectangle(frame, (int(bounds[0]*width), int(bounds[1]*height)),
                                  (int(bounds[2]*width), int(bounds[3]*height)), (0, 150, 0), 1)
                    cv2.putText(frame, '{} {:.2f}'.format(name, confidence),
                                (int(bounds[0]*width), max(15, int(bounds[1]*height)-10)),
                                cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 200, 0), 1)
                now = time.monotonic()
                with self.lock:
                    if not self.image_fresh(message) or now-started > self.max_image_age:
                        self.invalidate(now)
                        return
                    signal = self.tracker.update(observations, now, locked=self.mission.locked)
                    mission = self.mission.update(signal, now)
                    if mission.reset_track:
                        self.tracker.clear()
                        signal = self.tracker.snapshot(now)
                    self.last_result = now
                    self.publish(signal, mission, len(observations))
                self.summary_image(frame, message.header, signal, mission)
                rospy.loginfo_throttle(1., '[YOLO] raw=%s signal=%s state=%s width=%.4f mission=%s active=%s',
                                       signal.raw_label, signal.label, signal.state, signal.width, mission.phase, mission.active)
            except (CvBridgeError, ValueError, RuntimeError, cv2.error) as exc:
                with self.lock: self.invalidate(time.monotonic(), 'IMAGE_ERROR')
                rospy.logerr_throttle(2., 'Traffic YOLO frame failed: %s', exc)


def main():
    rospy.init_node('yolo_traffic_light_node')
    try: node = TrafficYoloNode()
    except Exception as exc:
        rospy.logfatal('Traffic YOLO startup failed: %s', exc)
        return 1
    rospy.spin()
    node.timer.shutdown()
    return 0


if __name__ == '__main__': raise SystemExit(main())
