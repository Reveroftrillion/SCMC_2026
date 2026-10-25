"""Actual green + lower-road false arrow caused a 120 s STOP on 2026-10-22."""
import json
from pathlib import Path
import pytest
from ctrl_camera.ctrl_yolo_onnx import choose_signal

HERE = Path(__file__).resolve().parent
DATA = json.loads((HERE/'data/signal_roi_20261022.json').read_text())
CONFIG = json.loads((HERE.parent/'config/yolo_stoplines.json').read_text())
ROI = next(line['roi'] for line in CONFIG['lines'] if line['id']=='intersection_02')

@pytest.mark.parametrize('frame', DATA['frames'], ids=['green-first','green-confirm','red','yellow'])
def test_recorded_signal_and_lower_road_are_separated(frame):
    assert choose_signal(frame['detections'], DATA['old_roi'], DATA['image_shape']) == frame['old']
    assert choose_signal(frame['detections'], ROI, DATA['image_shape']) == frame['candidate']
