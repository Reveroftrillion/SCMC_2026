import os
import sys
import types
import unittest
import importlib.util
import struct
import threading
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
# Allow these backend tests to run without ROS on the development PC.
if 'rospy' not in sys.modules:
    sys.modules['rospy'] = types.SimpleNamespace(loginfo=lambda *a: None, logwarn=lambda *a: None)
for name, message_names in [('sensor_msgs.msg', ('CompressedImage',)), ('std_msgs.msg', ('Bool', 'Float64'))]:
    if name not in sys.modules:
        sys.modules.setdefault(name.split('.')[0], types.ModuleType(name.split('.')[0]))
        module = types.ModuleType(name)
        for message_name in message_names:
            setattr(module, message_name, type(message_name, (), {}))
        sys.modules[name] = module

import numpy as np
from traffic_core import LABELS
from traffic_node import Detector
import traffic_node
from traffic_core import TrafficDecision
from udp_protocol import CameraAssembler
import cv2


class TrafficBackendTest(unittest.TestCase):
    def test_exact_model_classes_required(self):
        self.assertEqual(Detector._names(dict(enumerate(LABELS))), dict(enumerate(LABELS)))
        with self.assertRaises(ValueError):
            Detector._names(dict(enumerate(LABELS + ('other',))))
        self.assertEqual(Detector._names(dict(enumerate(reversed(LABELS)))), dict(enumerate(reversed(LABELS))))
        with self.assertRaises(ValueError):
            Detector._names({i: '4red' for i in range(len(LABELS))})

    def test_absent_onnx_uses_pt(self):
        with patch.object(Detector, '_load_pt', autospec=True) as load:
            Detector('file_that_does_not_exist.onnx', 'checkpoint.pt', .4)
            load.assert_called_once()

    def test_onnx_inference_failure_uses_pt(self):
        detector = Detector.__new__(Detector)
        detector.backend = 'onnx'
        with patch.object(detector, '_infer_onnx', side_effect=RuntimeError('bad inference')), \
                patch.object(detector, '_load_pt') as load, \
                patch.object(detector, '_infer_pt', return_value=['4red']) as infer:
            labels = detector.infer(np.zeros((20, 30, 3), dtype=np.uint8), [0., 0., 1., 1.])
            self.assertEqual(labels, ['4red'])
            load.assert_called_once()
            infer.assert_called_once()

    def test_empty_roi_is_not_green(self):
        detector = Detector.__new__(Detector)
        with self.assertRaises(ValueError):
            detector.infer(np.zeros((2, 2, 3), dtype=np.uint8), [0., 0., 0., 0.])

    def test_udp_jpeg_to_traffic_stop_and_green_release(self):
        class Pub:
            def __init__(self): self.messages=[]
            def publish(self, msg): self.messages.append(msg)
        class Stamp:
            def __init__(self, value): self.value=value
            def to_sec(self): return self.value
        class Now:
            def __sub__(self, _stamp): return types.SimpleNamespace(to_sec=lambda:0.01)
        ros = types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda:Now()),
            logwarn_throttle=lambda *a:None, loginfo_throttle=lambda *a:None, logerr_throttle=lambda *a:None)
        n=traffic_node.Node.__new__(traffic_node.Node)
        n.lock=threading.RLock(); n.progress=21.; n.progress_time=10.; n.last_stamp=None; n.frame_time=0.
        n.decision=TrafficDecision([dict(start_s=10., stop_s=25., direction='straight', roi=[0,0,1,1])],True)
        n.stop_pub, n.limit_pub = Pub(), Pub()
        labels=['4red']; n.detector=types.SimpleNamespace(infer=lambda frame,roi:labels[:])
        jpeg=cv2.imencode('.jpg',np.zeros((32,32,3),dtype=np.uint8))[1].tobytes()
        assembler=CameraAssembler()
        with patch.object(traffic_node,'rospy',ros), \
             patch.object(traffic_node,'Bool',lambda **k:types.SimpleNamespace(**k)), \
             patch.object(traffic_node,'Float64',lambda **k:types.SimpleNamespace(**k)):
            for i,now in enumerate((10.0,10.1,10.2,10.3)):
                if i==2: labels[:]=['4green']
                raw=b'MOR'+struct.pack('<4I',i,0,0,len(jpeg))+jpeg+b'EI'
                decoded=assembler.feed(raw)
                msg=types.SimpleNamespace(data=decoded,header=types.SimpleNamespace(stamp=Stamp(i+1)))
                n.progress_time=now
                with patch.object(traffic_node.time,'monotonic',return_value=now):
                    n.on_image(msg); n.publish(None)
                if i==1: self.assertTrue(n.stop_pub.messages[-1].data)
            self.assertFalse(n.stop_pub.messages[-1].data)

    @unittest.skipUnless(importlib.util.find_spec('ultralytics') and importlib.util.find_spec('torch') and
                         (Path(__file__).resolve().parents[1] / 'weights/1027_40epoch.pt').is_file(),
                         'PyTorch/Ultralytics or checkpoint unavailable')
    def test_real_checkpoint_inference_on_empty_frame(self):
        weight = Path(__file__).resolve().parents[1] / 'weights/1027_40epoch.pt'
        detector = Detector('missing.onnx', str(weight), .4)
        labels = detector.infer(np.zeros((720, 1280, 3), dtype=np.uint8), [0., 0., 1., 1.])
        self.assertIsInstance(labels, list)
        self.assertTrue(all(label in LABELS for label in labels))


if __name__ == '__main__':
    unittest.main()
