"""Ordered offline replay must preserve inputs and emit a merged target."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

PORT = Path(__file__).resolve().parents[1]


def test_ordered_replay_keeps_input_manifest_and_emits_targets(tmp_path):
    source = tmp_path / 'input'
    source.mkdir()
    route = np.loadtxt(PORT / 'src/ctrl_bringup/routes/global_path.txt')[:30, :2]
    meta = dict(route=route.tolist(), transforms={'lidar': dict(parent='base_link',
                translation=[2., 0., 2.11], rotation=[0., 0., 0., 1.])})
    (source / 'metadata.json').write_text(json.dumps(meta))
    np.savez_compressed(source / 'cloud.npz', points=np.array([[4., 0., -1.6]], dtype=np.float32))
    events = []
    for second in (1, 2, 3):
        events.extend([
            dict(topic='/localization/odometry', record_time=float(second), stamp=[second, 0],
                 position=[*route[10], 0.], rotation=[0., 0., 0., 1.], speed=0.),
            dict(topic='/localization/s', record_time=second+.001, data=5.),
            dict(topic='/behavior/mission', record_time=second+.002, data='{"mission":"CRUISE"}'),
            dict(topic='/sensors/lidar/points', record_time=second+.01, stamp=[second, 10000000],
                 frame='lidar', fields=['x', 'y', 'z'], file='cloud.npz')])
    (source / 'events.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in events))
    manifest = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir()}
    (source / 'manifest.json').write_text(json.dumps(manifest))
    output = tmp_path / 'result'
    command = [sys.executable, str(PORT / 'tools/replay_lidar_sequence.py'), 'replay',
               '--input', str(source), '--output', str(output), '--behavior',
               '--focus-x', '0', '--focus-y', '0']
    subprocess.run(command, check=True)
    frames = [json.loads(row) for row in (output / 'frames.jsonl').read_text().splitlines()]
    assert len(frames) == 3
    assert [row['stamp'] for row in frames] == [1.01, 2.01, 3.01]
    assert all(row['target'] is not None for row in frames)
    assert all(row['target']['v_max'] > 0 for row in frames)
    assert all(hashlib.sha256((source/name).read_bytes()).hexdigest() == digest
               for name, digest in manifest.items())
    before = (output / 'frames.jsonl').read_bytes()
    assert subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE).returncode != 0
    assert (output / 'frames.jsonl').read_bytes() == before
