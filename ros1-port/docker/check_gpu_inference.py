#!/usr/bin/env python3
"""Require real CUDA inference, compare unchanged Detectors, retain EP profile.

Current MORAI camera frames are diagnostics only, not equivalent sensor inputs.
"""
import argparse
from collections import Counter
import gc
import hashlib
import importlib.util
import json
from pathlib import Path
import time

import cv2
import numpy as np
import onnxruntime as ort

PORT = Path(__file__).resolve().parents[1]


def load(base, name):
    relative=('src/ctrl_camera/ctrl_camera/ctrl_yolo_onnx.py' if base==PORT else 'src/foscar_camera/foscar_camera/yolo_onnx.py')
    spec = importlib.util.spec_from_file_location(name, base / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Capture:
    def __init__(self, session):
        self.session = session

    def run(self, names, inputs):
        self.inputs = inputs
        self.raw = self.session.run(names, inputs)
        return self.raw


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--image', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    image = cv2.imread(str(args.image))
    if image is None:
        raise SystemExit('Cannot decode actual MORAI sample')
    report = dict(scope='actual CUDA inference; synthetic and current non-equivalent Camera-7 frame',
                  numpy=np.__version__, onnxruntime=ort.__version__,
                  sample_shape=list(image.shape), sample_sha256=hashlib.sha256(args.image.read_bytes()).hexdigest(),
                  available_providers=ort.get_available_providers(), runs=[])
    rng = np.random.default_rng(20261013)
    frames = [np.zeros((240, 320, 3), np.uint8), np.full((480, 640, 3), 114, np.uint8),
              rng.integers(0, 256, (720, 1280, 3), dtype=np.uint8), image]
    results = []
    for label, base in [('original', PORT.parent / '2026_CTRL_competition'), ('port', PORT)]:
        module = load(base, label + '_gpu_detector')
        model = base / ('src/ctrl_camera/models/best.onnx' if base==PORT else 'src/foscar_camera/models/best.onnx')
        detector = module.Detector(model, confidence=.4, threads=4)
        if detector.provider != 'CUDAExecutionProvider':
            raise RuntimeError(label + ' silently fell back to ' + detector.provider)
        captured = Capture(detector.session)
        detector.session = captured
        cases = []
        for frame in frames:
            started = time.perf_counter()
            detections = detector(frame.copy())
            elapsed = (time.perf_counter() - started) * 1000
            cases.append((captured.raw[0].copy(), detections,
                          module.choose_signal(detections, [0., 0., 1., 1.], frame.shape)))
            print(label, frame.shape, elapsed, 'ms', flush=True)
        inputs = captured.inputs
        report['runs'].append(dict(label=label, model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                                   provider=detector.provider, detections=cases[-1][1]))
        results.append(cases)
        del detector, captured
        gc.collect()
    report['cases'] = []
    for i, (a, b) in enumerate(zip(*results)):
        report['cases'].append(dict(index=i, raw_equal=bool(np.array_equal(a[0], b[0], equal_nan=True)),
                                   max_abs_diff=float(np.max(np.abs(a[0] - b[0]))),
                                   decoded_equal=a[1] == b[1], signal_equal=a[2] == b[2]))
    options = ort.SessionOptions()
    options.enable_profiling = True
    options.profile_file_prefix = str(args.output.parent / 'cuda-profile')
    options.intra_op_num_threads = 4
    session = ort.InferenceSession(str(PORT / 'src/ctrl_camera/models/best.onnx'),
                                   sess_options=options, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
    if session.get_providers()[0] != 'CUDAExecutionProvider':
        raise RuntimeError('Profile session did not initialize CUDA')
    session.run(None, inputs)
    timings = []
    for _ in range(5):
        started = time.perf_counter()
        session.run(None, inputs)
        timings.append((time.perf_counter() - started) * 1000)
    profile_path = session.end_profiling()
    events = json.loads(Path(profile_path).read_text())
    providers = Counter(event.get('args', {}).get('provider') for event in events
                        if event.get('cat') == 'Node' and event.get('args', {}).get('provider'))
    report.update(profile_file=profile_path, executed_node_providers=dict(providers),
                  inference_ms=timings, median_ms=float(np.median(timings)))
    report['passed'] = bool(providers['CUDAExecutionProvider']) and all(
        r['raw_equal'] and r['decoded_equal'] and r['signal_equal'] for r in report['cases'])
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
