#!/usr/bin/env python3
"""Compare original/port real ONNX execution on deterministic synthetic frames.

Uses the source Detector's own provider selection and preprocessing unchanged.
No ROS transport, real camera footage, GPU timing, or driving is validated.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import onnxruntime as ort


sys.dont_write_bytecode = True
WS = Path(__file__).resolve().parents[1]
BASELINE = WS.parent / '2026_CTRL_competition'


def load(base, name):
    path = base / ('src/foscar_camera/foscar_camera/yolo_onnx.py' if base == BASELINE else 'src/ctrl_camera/ctrl_camera/ctrl_yolo_onnx.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Capture:
    def __init__(self, session):
        self.session = session
        self.raw = None

    def run(self, *args, **kwargs):
        result = self.session.run(*args, **kwargs)
        self.raw = result[0].copy()
        return result


def main():
    report = dict(python=sys.version, numpy=np.__version__, opencv=cv2.__version__,
                  onnxruntime=ort.__version__, available_providers=ort.get_available_providers(),
                  atol=0, rtol=0, scope='synthetic images, same Windows execution provider; no ROS or camera timing')
    runs = []
    for label, base in [('original', BASELINE), ('port', WS)]:
        module = load(base, label + '_yolo')
        path = base / ('src/foscar_camera/models/best.onnx' if base == BASELINE else 'src/ctrl_camera/models/best.onnx')
        run = dict(label=label, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        try:
            detector = module.Detector(path, confidence=.4, threads=4)
        except Exception as exc:
            run['initialization_error'] = dict(type=type(exc).__name__, message=str(exc))
            runs.append((run, None, module))
            continue
        run['provider'] = detector.provider
        run['input'] = [(i.name, i.shape, i.type) for i in detector.session.get_inputs()]
        run['output'] = [(i.name, i.shape, i.type) for i in detector.session.get_outputs()]
        detector.session = Capture(detector.session)
        runs.append((run, detector, module))
    report['models'] = [run for run, _, _ in runs]
    initialized = all(detector is not None for _, detector, _ in runs)
    report['initialized'] = initialized
    report['cases'] = []
    if initialized:
        rng = np.random.default_rng(20261013)
        frames = [np.zeros((240, 320, 3), np.uint8),
                  np.full((480, 640, 3), 114, np.uint8),
                  rng.integers(0, 256, (720, 1280, 3), dtype=np.uint8)]
        for index, image in enumerate(frames):
            values = []
            for run, detector, module in runs:
                detections = detector(image.copy())
                values.append((detections, detector.session.raw,
                               module.choose_signal(detections, [0., 0., 1., 1.], image.shape)))
            same_raw = np.array_equal(values[0][1], values[1][1], equal_nan=True)
            same_decoded = values[0][0] == values[1][0]
            same_signal = values[0][2] == values[1][2]
            report['cases'].append(dict(index=index, shape=list(image.shape),
                                        input_sha256=hashlib.sha256(image.tobytes()).hexdigest(),
                                        raw_equal=same_raw, decoded_equal=same_decoded,
                                        signal_equal=same_signal,
                                        max_abs_diff=float(np.max(np.abs(values[0][1] - values[1][1]))),
                                        detections=values[0][0], signal=values[0][2]))
        report['passed'] = all(c['raw_equal'] and c['decoded_equal'] and c['signal_equal'] for c in report['cases'])
    else:
        report['passed'] = False
        report['matching_baseline_error'] = report['models'][0].get('initialization_error') == report['models'][1].get('initialization_error')
    destination = WS / 'docs/model-comparison.json'
    destination.parent.mkdir(exist_ok=True)
    destination.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
