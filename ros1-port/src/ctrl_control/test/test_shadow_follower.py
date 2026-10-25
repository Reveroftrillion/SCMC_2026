"""Follower boundary checks with ROS doubles; no middleware or sockets."""
import io
import json
from dataclasses import asdict
from types import SimpleNamespace

import pytest

from ctrl_control.ctrl_core import Output, Target
from ctrl_control.ctrl_follower import Follower
from ctrl_control.ctrl_shadow import replay
from ros_stubs import Capture, Logger, Stamp, Time
from test_shadow import settings


def follower(tmp_path, config=''):
    f = object.__new__(Follower)
    f.p = dict(steer_kick_deg=0, target_timeout=.5, dr_speed_kmh=0, shadow_config_file=config)
    for name in ('command_pub', 'state_pub', 'telemetry_pub', 'target_pub', 'path_pub'):
        setattr(f, name, Capture())
    f.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=100))
    f.get_logger = lambda: Logger()
    f.target, f.target_stamp, f.target_arrival = Target(), 100., 100.
    f.loc_mode, f.start, f.last_report, f.previous_state = 'GNSS', 100., 100., 'CRUISE'
    f.writer = SimpleNamespace(writerow=lambda row: None)
    f.log, f.log_path = io.StringIO(), tmp_path / 'follower.csv'
    f.controller = SimpleNamespace(s_offset=-500.)
    f.route = SimpleNamespace(points=((0., 0.), (60., 0.)))
    f.odom = SimpleNamespace(header=SimpleNamespace(stamp=Stamp(sec=100), frame_id='map'))
    f.shadow_recorder, f.shadow_request, f.shadow_request_stamp = None, None, None
    f.shadow_request_error = ''
    f.shadow_failed = False
    return f


@pytest.mark.parametrize('mode', ['disabled', 'enabled', 'missing_config', 'worker_error'])
def test_production_publication_survives_shadow_lifecycle(tmp_path, mode):
    config = tmp_path / 'config.json'
    config.write_text(json.dumps(asdict(settings())), encoding='utf-8')
    if mode == 'missing_config':
        config = tmp_path / 'missing.json'
    f = follower(tmp_path, '' if mode == 'disabled' else str(config))
    if mode == 'worker_error':
        f.shadow_recorder = SimpleNamespace(error='simulated disk failure', close=lambda: False)
    output = Output('CRUISE', 'ok', accel=.2, brake=.05, steering=.1, progress=510., target_speed=2.)
    f.publish_output(output, 100., 10., .2, 0., 2.)
    command, = f.command_pub.values
    assert (command.accel, command.brake, command.steering_rad, command.state) == (.2, .05, .1, 'CRUISE')
    assert len(f.state_pub.values) == len(f.telemetry_pub.values) == len(f.target_pub.values) == 1
    if mode == 'enabled':
        assert f.shadow_recorder is not None and not f.shadow_failed
        f.close()
        assert replay(tmp_path / 'follower.shadow.jsonl')['passed']
        row = json.loads((tmp_path / 'follower.shadow.jsonl').read_text().splitlines()[1])
        assert row['output']['valid'] and row['input']['old_angle_rad'] == .1
    elif mode == 'disabled':
        assert f.shadow_recorder is None
        f.close()
    else:
        assert f.shadow_failed
        f.publish_output(output, 100., 10., .2, 0., 2.)
        assert len(f.command_pub.values) == 2
        f.close()


def test_snapshot_uses_accepted_target_and_marks_invalid_state(tmp_path):
    config = tmp_path / 'config.json'
    config.write_text(json.dumps(asdict(settings())), encoding='utf-8')
    f = follower(tmp_path, str(config))
    f.target = Target(state='INVALID_TARGET')
    f.publish_output(Output('CRUISE', 'test', progress=510.), 100., 10., 0., 0., 2.)
    f.close()
    row = json.loads((tmp_path / 'follower.shadow.jsonl').read_text().splitlines()[1])
    assert not row['output']['valid'] and row['output']['reason'] == 'INVALID_TARGET'


def test_snapshot_error_persists_when_worker_drops_first_frame(tmp_path):
    f = follower(tmp_path, 'enabled')
    frames = []
    f.shadow_recorder = SimpleNamespace(error=None, push=frames.append)
    f.target = Target(reference=SimpleNamespace())  # Missing reference fields.
    output = Output('CRUISE', 'test', progress=510.)
    for _ in range(2):
        f.publish_output(output, 100., 10., 0., 0., 2.)
    assert not f.shadow_failed
    assert [row.input_error for row in frames] == ['target_snapshot_error'] * 2
    assert len(f.command_pub.values) == 2
