"""Observer lifecycle and deterministic replay; no vehicle connection."""
import json
import math
import threading
from dataclasses import asdict, replace
from types import SimpleNamespace

import pytest

from ctrl_control.ctrl_lateral import VehiclePose
from ctrl_control.ctrl_path_selection import PathRequest
from ctrl_control.ctrl_shadow import (
    ShadowEngine, ShadowFrame, ShadowRecorder, ShadowSettings, decode, encode,
    replay, request_from_target,
)
from test_lateral import config as lateral_config
from test_progress import config as progress_config
from test_path_selection import config as selection_config
from test_path_conversion import base


def settings():
    return ShadowSettings(lateral_config(), progress_config(), selection_config(), 2, .05, 1.5, 2, 6)


def frame(now=10., x=10., **changes):
    return replace(ShadowFrame(now, VehiclePose(x, .2, 0, 2, now, 'map'),
                              500 + x, 0, PathRequest(now, 'map'), now), **changes)


def engine():
    return ShadowEngine(base(), settings())


def test_header_and_settings_roundtrip():
    original = engine()
    restored = ShadowEngine.from_header(json.loads(json.dumps(encode(original.header()), allow_nan=False)))
    assert restored.evaluate(frame()) == original.evaluate(frame())
    assert ShadowSettings.from_dict(asdict(settings())) == settings()


@pytest.mark.parametrize('value', [0, -1, math.inf, math.nan, True])
def test_invalid_settings(value):
    with pytest.raises(ValueError):
        replace(settings(), initial_step_s=value)


def test_normal_heartbeat_uses_same_revision_and_bounded_steering():
    e = engine()
    for i in range(6):
        out = e.evaluate(frame(10 + i * .05, 10 + i * .1))
        assert out['valid'], out
        assert out['revision'] == 1
        assert abs(out['new_angle_rad']) <= .5
    assert out['new_angle_rad'] < 0


def test_stale_input_produces_no_candidate():
    e = engine()
    assert e.evaluate(frame())['valid']
    out = e.evaluate(frame(11, request=None))
    assert not out['valid'] and out['new_angle_rad'] is None


def test_input_fault_requires_newer_valid_request():
    e = engine()
    assert e.evaluate(frame(input_error='INVALID_TARGET'))['reason'] == 'INVALID_TARGET'
    assert not e.evaluate(frame())['valid']
    assert e.evaluate(frame(10.05, 10.1))['valid']


def test_nonfinite_request_blocks_then_newer_request_recovers():
    e = engine()
    assert not e.evaluate(frame(request=PathRequest(math.nan, 'map')))['valid']
    assert e.evaluate(frame(10.05, 10.1))['valid']


def test_noncruise_is_diagnostic_only():
    out = engine().evaluate(frame(comparable=False))
    assert not out['valid'] and out['reason'] == 'production_not_comparable'
    assert out['new_angle_rad'] is None


def test_legacy_offset_anchor_does_not_slide_on_heartbeat():
    e = engine()
    assert e.evaluate(frame(request=PathRequest(10, 'map', lat_offset_m=1)))['valid']
    anchor = e.selector.state.anchor
    out = e.evaluate(frame(10.05, 10.1, request=PathRequest(10.05, 'map', lat_offset_m=1)))
    assert out['valid'], out
    assert e.selector.state.anchor == anchor


def test_return_heartbeats_hold_until_transition_complete():
    e = engine()
    e.evaluate(frame(request=PathRequest(10, 'map', lat_offset_m=1)))
    assert e.evaluate(frame(10.05, 10.1))['valid']
    anchor = e.selector.state.anchor
    for i in range(2, 5):
        out = e.evaluate(frame(10 + i * .05, 10 + i * .1))
        assert out['valid'], out
        assert e.selector.state.anchor == anchor
        assert out['mode'] == 'offset'


def test_projection_rejects_bad_frame_and_out_of_route_hint():
    f = frame()
    assert not engine().evaluate(replace(f, pose=replace(f.pose, frame_id='odom')))['valid']
    assert not engine().evaluate(replace(f, global_hint_s=10000))['valid']


def test_request_snapshot_and_nonfinite_json():
    target = SimpleNamespace(reference=None, v_max=math.inf, stop_s=math.nan,
        lat_offset=1., entry_s=math.nan, entry_length_m=0., return_s=math.nan,
        return_length_m=0., chain=((1., 510., 6.),), reference_release=False)
    r = request_from_target(target, 10.)
    assert r.chain_offset == (1.,) and r.stop_global_s is None
    encoded = json.dumps(encode(asdict(r)), allow_nan=False)
    assert math.isinf(decode(json.loads(encoded))['v_max_mps'])


def test_worker_record_replay_and_tampering(tmp_path):
    path = tmp_path / 'shadow.jsonl'
    recorder = ShadowRecorder(engine(), path)
    assert recorder.push(frame())
    assert recorder.close()
    assert replay(path) == dict(frames=1, mismatches=[], passed=True)
    with pytest.raises(FileExistsError):
        ShadowRecorder(engine(), path)
    rows = path.read_text().splitlines()
    changed = json.loads(rows[1]); changed['output']['valid'] = False
    path.write_text(rows[0] + '\n' + json.dumps(changed) + '\n')
    assert replay(path)['mismatches'] == [1]


def test_slow_worker_bounds_pending_queue_and_replay(tmp_path):
    e = engine()
    entered, release = threading.Event(), threading.Event()
    evaluate = e.evaluate
    def slow(value):
        entered.set()
        assert release.wait(2)
        return evaluate(value)
    e.evaluate = slow
    recorder = ShadowRecorder(e, tmp_path / 'slow.jsonl')
    recorder.push(frame())
    assert entered.wait(2)
    try:
        for i in range(1, 11):
            recorder.push(frame(10 + i * .01, 10 + i * .02))
        assert recorder._queue.qsize() == 1 and recorder.dropped == 9
    finally:
        release.set()
        assert recorder.close()
    assert recorder.written == 2
    assert replay(tmp_path / 'slow.jsonl')['passed']


def test_worker_failure_is_reported_without_escaping_producer(tmp_path):
    e = engine()
    def fail(_):
        raise OSError('simulated write/calculation failure')
    e.evaluate = fail
    recorder = ShadowRecorder(e, tmp_path / 'failure.jsonl')
    recorder.push(frame())
    assert not recorder.close()
    assert 'OSError' in recorder.error
    assert not recorder.push(frame())
