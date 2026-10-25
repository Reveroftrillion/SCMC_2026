"""Diagnostic integrity: inspect the estimator without altering its result."""
import importlib.util
import hashlib
import json
from pathlib import Path
import sys
import subprocess

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('replay_lidar_ground',
                                             Path(__file__).with_name('replay_lidar_ground.py'))
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)
sys.path.insert(0, str(replay.PORT / 'src/ctrl_lidar'))
from ctrl_lidar.ctrl_ground import estimate_ground_plane


@pytest.mark.parametrize('height', [0., .53])
def test_trace_preserves_estimator_result_and_input(height):
    x, y = np.meshgrid(np.linspace(3.1, 27.9, 40), np.linspace(-4.9, 4.9, 20))
    points = np.c_[x.ravel(), y.ravel(), (.006*x - .002*y + height).ravel()]
    before = points.copy()
    expected = estimate_ground_plane(points)
    prior_trace = sys.gettrace()
    actual, captured = replay.trace_calculation(estimate_ground_plane, points)
    assert sys.gettrace() is prior_trace
    np.testing.assert_array_equal(points, before)
    if expected is None:
        assert actual is None
        # A perfect high plane is rejected before any candidate receives support;
        # the noisy recorded frame can instead reach the final refit rejection.
        assert captured['best_inlier_count'] == 0
        assert 'return None' in captured['_return_location']['source']
    else:
        np.testing.assert_array_equal(actual, expected)


def test_trace_restores_previous_hook_after_error():
    def failing():
        raise ValueError('diagnostic failure')
    prior = sys.gettrace()
    with pytest.raises(ValueError, match='diagnostic failure'):
        replay.trace_calculation(failing)
    assert sys.gettrace() is prior


def test_report_refuses_to_overwrite_existing_evidence(tmp_path):
    output = tmp_path / 'report.json'
    replay.write_json(output, dict(value=1))
    before = output.read_bytes()
    with pytest.raises(FileExistsError):
        replay.write_json(output, dict(value=2))
    assert output.read_bytes() == before


def test_committed_baseline_uses_exact_committed_sources(tmp_path):
    cloud = tmp_path / 'cloud.npz'
    np.savez_compressed(cloud, points=np.array([[4., 0., -1.6], [5., 0., -1.6], [6., 0., -1.6]]), stamp=1.)
    original = cloud.read_bytes()
    output = tmp_path / 'baseline.json'
    subprocess.run([sys.executable, str(Path(replay.__file__)), 'inspect',
                    '--cloud', str(cloud), '--baseline-head', '--output', str(output)], check=True)
    report = json.loads(output.read_text())
    for name, digest in report['source_sha256'].items():
        source = subprocess.check_output(['git', 'show', report['source_ref'] + ':ros1-port/' + name],
                                         cwd=replay.PORT.parent)
        assert hashlib.sha256(source).hexdigest() == digest
    assert cloud.read_bytes() == original
