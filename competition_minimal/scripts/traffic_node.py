#!/usr/bin/env python3
"""ROS traffic signal detector with ONNX Runtime and Ultralytics checkpoint backends."""
import ast
import math
import os
import threading
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float64
from traffic_core import LABELS, TrafficDecision


class Detector:
    def __init__(self, model_path, pt_path, confidence):
        self.model_path, self.pt_path, self.confidence = model_path, pt_path, confidence
        self.backend = None
        errors = []
        if os.path.isfile(model_path):
            try:
                self._load_onnx()
                return
            except Exception as exc:
                errors.append('ONNX: %s' % exc)
                rospy.logwarn('ONNX detector unavailable; trying PyTorch: %s', exc)
        else:
            errors.append('ONNX file missing: %s' % model_path)
        try:
            self._load_pt()
            return
        except Exception as exc:
            errors.append('PyTorch: %s' % exc)
        raise RuntimeError('; '.join(errors))

    @staticmethod
    def _names(names):
        if isinstance(names, (list, tuple)):
            names = dict(enumerate(names))
        result = {int(k): str(v) for k, v in names.items()}
        if len(result) != len(LABELS) or set(result.values()) != set(LABELS):
            raise ValueError('Unsupported traffic class metadata: %s' % names)
        return result

    def _load_onnx(self):
        import onnxruntime as ort
        self.ort = ort
        available = ort.get_available_providers()
        providers = (['CUDAExecutionProvider', 'CPUExecutionProvider']
                     if 'CUDAExecutionProvider' in available else ['CPUExecutionProvider'])
        try:
            session = ort.InferenceSession(self.model_path, providers=providers)
        except Exception as exc:
            rospy.logwarn('CUDA ONNX session failed, trying CPU: %s', exc)
            session = ort.InferenceSession(self.model_path, providers=['CPUExecutionProvider'])
        inputs = session.get_inputs()
        if len(inputs) != 1:
            raise ValueError('Expected one ONNX input')
        input_spec = inputs[0]
        shape = input_spec.shape
        if len(shape) != 4 or shape[0] != 1 or shape[1] != 3 or not all(isinstance(v, int) and v > 0 for v in shape[2:]):
            raise ValueError('Expected fixed NCHW image input, got %s' % shape)
        if input_spec.type != 'tensor(float)':
            raise ValueError('Unsupported ONNX input type %s' % input_spec.type)
        meta = session.get_modelmeta().custom_metadata_map
        self.names = self._names(ast.literal_eval(meta.get('names', '{}')))
        self.session, self.input = session, input_spec
        self.height, self.width = shape[2], shape[3]
        self.backend = 'onnx'
        rospy.loginfo('Traffic ONNX input=%s classes=%s providers=%s', shape, self.names,
                      self.session.get_providers())

    def _load_pt(self):
        if not os.path.isfile(self.pt_path):
            raise RuntimeError('Checkpoint missing: %s' % self.pt_path)
        import torch
        from ultralytics import YOLO
        model = YOLO(self.pt_path)
        self.names = self._names(model.names)
        if getattr(model, 'task', None) != 'detect':
            raise ValueError('Traffic checkpoint must be a detection model')
        self.model = model
        self.device = '0' if torch.cuda.is_available() else 'cpu'
        self.backend = 'pt'
        rospy.loginfo('Traffic PyTorch input=640 classes=%s device=%s', self.names, self.device)

    def _infer_pt(self, crop):
        try:
            results = self.model.predict(source=crop, imgsz=640, conf=self.confidence,
                                         iou=.45, device=self.device, verbose=False)
        except Exception:
            if self.device == 'cpu':
                raise
            rospy.logwarn('CUDA PyTorch inference failed; switching to CPU')
            self.device = 'cpu'
            results = self.model.predict(source=crop, imgsz=640, conf=self.confidence,
                                         iou=.45, device='cpu', verbose=False)
        if len(results) != 1 or results[0].boxes is None:
            raise ValueError('Unexpected PyTorch inference result')
        classes = results[0].boxes.cls.cpu().numpy()
        if not np.isfinite(classes).all() or any(int(i) != i or int(i) not in self.names for i in classes):
            raise ValueError('Invalid detected class IDs')
        return [self.names[int(i)] for i in classes]

    def infer(self, frame, roi):
        h, w = frame.shape[:2]
        x0, y0, x1, y1 = (int(v * scale) for v, scale in zip(roi, (w, h, w, h)))
        crop = frame[y0:y1, x0:x1]
        if crop.size == 0:
            raise ValueError('Empty signal ROI')
        if self.backend == 'pt':
            return self._infer_pt(crop)
        try:
            return self._infer_onnx(crop)
        except Exception as exc:
            rospy.logwarn('ONNX traffic inference failed; trying PyTorch: %s', exc)
            self._load_pt()
            return self._infer_pt(crop)

    def _infer_onnx(self, crop):
        ch, cw = crop.shape[:2]
        scale = min(self.width / cw, self.height / ch)
        nw, nh = max(1, int(round(cw * scale))), max(1, int(round(ch * scale)))
        left, top = (self.width - nw) // 2, (self.height - nh) // 2
        padded = np.full((self.height, self.width, 3), 114, dtype=np.uint8)
        padded[top:top+nh, left:left+nw] = cv2.resize(crop, (nw, nh))
        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
        try:
            outputs = self.session.run(None, {self.input.name: tensor})
        except Exception:
            if 'CUDAExecutionProvider' not in self.session.get_providers():
                raise
            rospy.logwarn('CUDA traffic inference failed; switching to CPU')
            self.session = self.ort.InferenceSession(self.model_path, providers=['CPUExecutionProvider'])
            outputs = self.session.run(None, {self.input.name: tensor})
        pred = np.asarray(outputs[0])
        if pred.ndim != 3 or pred.shape[0] != 1:
            raise ValueError('Unsupported YOLO output shape %s' % (pred.shape,))
        pred = pred[0]
        channels = 4 + len(LABELS)
        if pred.shape[0] == channels:
            pred = pred.T
        if pred.ndim != 2 or pred.shape[1] != channels or not np.isfinite(pred).all():
            raise ValueError('Unsupported YOLO output shape or values %s' % (pred.shape,))
        scores = pred[:, 4:]
        ids = np.argmax(scores, axis=1)
        conf = scores[np.arange(len(ids)), ids]
        selected = np.flatnonzero(conf >= self.confidence)
        boxes, class_ids, confidences = [], [], []
        for i in selected:
            cx, cy, bw, bh = pred[i, :4]
            bx0, by0 = max(0., float(cx-bw/2)), max(0., float(cy-bh/2))
            bx1, by1 = min(float(self.width), float(cx+bw/2)), min(float(self.height), float(cy+bh/2))
            if bx1 <= bx0 or by1 <= by0:
                continue
            boxes.append([bx0, by0, bx1-bx0, by1-by0])
            class_ids.append(int(ids[i]))
            confidences.append(float(conf[i]))
        if not boxes:
            return []
        keep = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence, 0.45)
        return [self.names[class_ids[int(i)]] for i in np.asarray(keep).reshape(-1)]


class Node:
    def __init__(self):
        self.lock = threading.RLock()
        self.decision = TrafficDecision(rospy.get_param('~intersections', []),
                                        rospy.get_param('~positions_verified', False),
                                        rospy.get_param('~front_offset', 3.0))
        if len(self.decision.items) != 4:
            self.decision.verified = False
        self.progress = float('nan')
        self.progress_time = 0.0
        self.frame_time = 0.0
        self.last_stamp = None
        self.detector = None
        self.stop_pub = rospy.Publisher('/competition/traffic_stop', Bool, queue_size=1)
        self.limit_pub = rospy.Publisher('/competition/traffic_speed_limit', Float64, queue_size=1)
        try:
            confidence = float(rospy.get_param('~confidence', .4))
            if not math.isfinite(confidence) or not 0 < confidence <= 1:
                raise ValueError('Invalid confidence')
            self.detector = Detector(rospy.get_param('~model_path'), rospy.get_param('~pt_path'), confidence)
        except Exception as exc:
            rospy.logerr('Traffic detector unavailable; signal stays UNKNOWN: %s', exc)
        if not self.decision.verified:
            rospy.logerr('Traffic mission distances are not verified: vehicle stop requested')
        self.progress_sub = rospy.Subscriber('/competition/progress', Float64, self.on_progress, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/traffic/image/compressed', CompressedImage, self.on_image,
                                          queue_size=1, buff_size=2**24)
        self.timer = rospy.Timer(rospy.Duration(.1), self.publish)

    def on_progress(self, msg):
        with self.lock:
            self.progress = float(msg.data)
            self.progress_time = time.monotonic()

    def on_image(self, msg):
        now = time.monotonic()
        with self.lock:
            s = self.progress
            idx = self.decision.current(s)
            if self.detector is None or idx is None:
                return
            stamp = msg.header.stamp.to_sec()
            age = (rospy.Time.now() - msg.header.stamp).to_sec()
            if stamp <= 0 or age < -.1 or age > .8 or (self.last_stamp is not None and stamp <= self.last_stamp):
                self.decision.observe(s, [], now)
                rospy.logwarn_throttle(2., 'Traffic frame rejected: stale, future, zero or non-monotonic timestamp (age=%.3f)', age)
                return
            self.last_stamp = stamp
            roi = self.decision.items[idx]['roi']
        try:
            frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                rospy.logwarn_throttle(2., 'Traffic frame JPEG decode failed; signal UNKNOWN')
                raise ValueError('JPEG decode failed')
            labels = self.detector.infer(frame, roi)
            if not labels:
                rospy.loginfo_throttle(2., 'Traffic ROI has no detections; outside-ROI objects excluded, signal UNKNOWN (ROI=%s)', roi)
            end = time.monotonic()
            with self.lock:
                if age + end-now > .8:
                    rospy.logwarn_throttle(2., 'Traffic inference result too old (%.3fs); signal UNKNOWN', age + end-now)
                    labels = []
                elif self.decision.current(self.progress) != idx:
                    rospy.logwarn_throttle(2., 'Traffic inference crossed intersection ROI boundary; signal UNKNOWN')
                    labels = []
                self.decision.observe(self.progress, labels, end)
                self.frame_time = end
            rospy.loginfo_throttle(1., 'Traffic detection: intersection=%s labels=%s', idx + 1, labels)
        except Exception as exc:
            with self.lock:
                self.decision.observe(self.progress, [], time.monotonic())
            rospy.logerr_throttle(2., 'Traffic inference failed, signal UNKNOWN: %s', exc)

    def publish(self, _event):
        now = time.monotonic()
        with self.lock:
            if now-self.progress_time > .7 or not math.isfinite(self.progress):
                stop, limit, reason = True, 0.0, 'STALE_PROGRESS'
            else:
                stop, limit, reason = self.decision.decision(self.progress, now)
        self.stop_pub.publish(Bool(data=stop))
        self.limit_pub.publish(Float64(data=limit))
        rospy.loginfo_throttle(1., 'Traffic decision: %s stop=%s speed_limit=%.2f m/s', reason, stop, limit)


if __name__ == '__main__':
    rospy.init_node('competition_traffic')
    Node()
    rospy.spin()
