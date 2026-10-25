"""Request lifecycle and fail-closed selection requirements, with no ROS imports."""

import math
from dataclasses import FrozenInstanceError, replace
from types import SimpleNamespace

import pytest

from ctrl_control.ctrl_path_selection import (
    PathRequest, PathSelector, SelectionConfig, TransitionAnchor,
)
from test_path_conversion import base


def config(**changes):
    values = dict(request_max_age_s=0.5, future_tolerance_s=0.02, base_step_m=0.5,
                  max_offset_m=4.0, max_points=1000, max_reference_step_m=1.0,
                  max_reference_ids=4)
    values.update(changes)
    return SelectionConfig(**values)


def selector(**changes):
    return PathSelector(base(), config(**changes))


def request(stamp=10.0, **changes):
    return replace(PathRequest(stamp, 'map'), **changes)


def reference(stamp=10.0, name='reference', **changes):
    values = dict(reference_id=name, reference_x=(0.0, 1.0, 2.0),
                  reference_y=(0.0, 0.0, 0.0), reference_global_s=(500.0, 501.0, 502.0))
    values.update(changes)
    return request(stamp, **values)


def preview(stamp=10.0, **changes):
    values = dict(lat_offset_m=2.0, entry_s=510.0, entry_length_m=10.0,
                  return_s=530.0, return_length_m=10.0)
    values.update(changes)
    return request(stamp, **values)


def submit(s, r=None, now=10.0, **kwargs):
    return s.submit(r if r is not None else request(now), now_s=now, **kwargs)


def assert_blocked(s, result, old, reason, now):
    assert not result.valid and result.state is None
    assert result.reason == reason
    assert s.state is old
    assert not s.current(now_s=now).valid


def test_global_heartbeat_reuses_geometry_and_updates_metadata_atomically():
    s = selector()
    assert not s.current(now_s=10.0).valid
    first = submit(s, request(v_max_mps=5, stop_global_s=550))
    second = submit(s, request(10.1, v_max_mps=0, stop_global_s=520), now=10.1)
    assert first.valid and second.valid
    assert first.state.path is second.state.path is s.base
    assert first.state.revision == second.state.revision == 1
    assert second.state.v_max_mps == 0 and second.state.stop_global_s == 520


def test_reference_held_until_explicit_release_with_new_speed_metadata():
    s = selector()
    initial = submit(s, reference())
    held = submit(s, request(10.1, v_max_mps=3), now=10.1)
    assert held.valid and held.state.mode == 'reference'
    assert held.state.path is initial.state.path and held.state.revision == initial.state.revision
    released = submit(s, request(10.2, reference_release=True), now=10.2)
    assert released.valid and released.state.mode == 'global'
    assert released.state.path is s.base and released.state.revision == initial.state.revision + 1


def test_reference_same_id_keeps_object_and_changed_shape_blocks_old_output():
    s = selector()
    first = submit(s, reference())
    same = submit(s, reference(10.1), now=10.1)
    assert same.state.path is first.state.path
    bad = submit(s, reference(10.2, reference_y=(0.1, 0.1, 0.1)), now=10.2)
    assert_blocked(s, bad, same.state, 'reference_id_mutation', 10.2)
    assert s.retained_reference_ids == 1
    recovered = submit(s, reference(10.3), now=10.3)
    assert recovered.valid and recovered.state.path is first.state.path


def test_reference_id_remains_immutable_after_release_and_switch():
    s = selector()
    submit(s, reference())
    submit(s, request(10.1, reference_release=True), now=10.1)
    second = submit(s, reference(10.2, name='second'), now=10.2)
    bad = submit(s, reference(10.3, reference_global_s=(600, 601, 602)), now=10.3)
    assert_blocked(s, bad, second.state, 'reference_id_mutation', 10.3)


def test_reference_registry_has_a_limit_and_does_not_evict_old_id_records():
    s = selector(max_reference_ids=1)
    first = submit(s, reference())
    bad = submit(s, reference(10.1, name='second'), now=10.1)
    assert_blocked(s, bad, first.state, 'reference_registry_full', 10.1)
    assert s.retained_reference_ids == 1
    assert submit(s, reference(10.2), now=10.2).valid


@pytest.mark.parametrize('changes', [dict(reference_release=True), dict(lat_offset_m=1),
    dict(entry_length_m=5), dict(chain_offset=(1,), chain_start_s=(510,), chain_length_m=(10,))])
def test_reference_payload_cannot_mix_with_release_or_avoidance(changes):
    s = selector()
    result = submit(s, reference(**changes))
    assert_blocked(s, result, None, 'reference_mode_conflict', 10.0)
    assert s.retained_reference_ids == 0


def test_active_reference_requires_release_before_switching_to_offset():
    s = selector()
    first = submit(s, reference())
    result = submit(s, preview(10.1), now=10.1)
    assert_blocked(s, result, first.state, 'reference_release_required', 10.1)
    released = submit(s, preview(10.2, reference_release=True), now=10.2)
    assert released.valid and released.state.mode == 'offset'


def test_chain_has_documented_precedence_over_other_avoidance_fields():
    s = selector()
    result = submit(s, preview(chain_offset=(-1, 0), chain_start_s=(510, 530), chain_length_m=(10, 10)))
    assert result.valid
    assert result.state.path.point_at_global(520)[1] == -1.0
    assert result.state.path.point_at_global(540)[1] == 0.0


def test_preview_heartbeat_reuses_path_and_does_not_restart_transition():
    s = selector()
    first = submit(s, preview())
    second = submit(s, preview(10.1, v_max_mps=4), now=10.1)
    assert second.valid and second.state.path is first.state.path
    assert second.state.revision == first.state.revision
    assert second.state.path.point_at_global(515)[1] == 1.0


def test_legacy_offset_requires_anchor_and_reuses_first_anchor():
    s = selector()
    assert not submit(s, request(lat_offset_m=2)).valid
    anchor = TransitionAnchor(510, 10, 0.5)
    first = submit(s, request(10.1, lat_offset_m=2), now=10.1, anchor=anchor)
    assert first.valid and first.state.path.point_at_global(510)[1] == 0.5
    second = submit(s, request(10.2, lat_offset_m=2), now=10.2)
    assert second.valid and second.state.path is first.state.path
    bad = submit(s, request(10.3, lat_offset_m=2), now=10.3,
                  anchor=TransitionAnchor(511, 10, 0.5))
    assert_blocked(s, bad, second.state, 'anchor_changed_for_active_offset', 10.3)


def test_changed_legacy_offset_needs_a_new_anchor():
    s = selector()
    first = submit(s, request(lat_offset_m=2), anchor=TransitionAnchor(510, 10, 0))
    bad = submit(s, request(10.1, lat_offset_m=-1), now=10.1)
    assert_blocked(s, bad, first.state, 'transition_anchor_required', 10.1)


def test_zero_packet_holds_plan_until_zero_return_is_proven_complete():
    s = selector()
    initial = submit(s, preview())
    held = submit(s, request(10.1), now=10.1)
    assert held.valid and held.state.path is initial.state.path
    bad = submit(s, request(10.2), now=10.2, progress_global_s=539.9)
    assert_blocked(s, bad, held.state, 'offset_not_finished', 10.2)
    result = submit(s, request(10.3), now=10.3, progress_global_s=540)
    assert result.valid and result.state.mode == 'global'


def test_nonzero_terminal_offset_cannot_be_released_without_a_return():
    s = selector()
    first = submit(s, preview(return_length_m=0))
    bad = submit(s, request(10.1), now=10.1, progress_global_s=550)
    assert_blocked(s, bad, first.state, 'offset_not_finished', 10.1)
    returned = submit(s, request(10.2), now=10.2, anchor=TransitionAnchor(530, 10, 2))
    assert returned.valid and returned.state.mode == 'offset'
    assert returned.state.path.point_at_global(535)[1] == 1.0


@pytest.mark.parametrize('station', [499, 561, math.nan, math.inf])
def test_invalid_offset_completion_evidence(station):
    s = selector()
    old = submit(s, preview()).state
    result = submit(s, now=10.1, progress_global_s=station)
    assert_blocked(s, result, old, 'offset_not_finished', 10.1)


@pytest.mark.parametrize('stamp,now,reason', [(10, 10.1, 'request_not_newer'),
    (9.9, 10.1, 'request_not_newer'), (10.1, 10.7, 'request_stale'),
    (11, 10.1, 'request_from_future')])
def test_discarded_packets_do_not_refresh_or_corrupt_valid_output(stamp, now, reason):
    s = selector()
    first = submit(s)
    result = submit(s, request(stamp), now=now)
    assert not result.valid and result.reason == reason
    assert s.state is first.state
    current = s.current(now_s=now)
    assert current.valid == (now - 10 <= 0.5)


def test_fresh_malformed_packet_blocks_old_output_until_strictly_newer_recovery():
    s = selector()
    old = submit(s).state
    bad = submit(s, request(10.2, lat_offset_m=math.nan), now=10.2)
    assert_blocked(s, bad, old, 'invalid_path_fields', 10.2)
    assert not submit(s, request(10.1), now=10.2).valid
    assert not s.current(now_s=10.2).valid
    assert submit(s, request(10.3), now=10.3).valid


def test_timeout_boundary_and_future_tolerance_boundary_are_inclusive():
    s = selector()
    assert submit(s, request(10.02)).valid
    assert s.current(now_s=10.52).valid
    assert not s.current(now_s=10.521).valid
    assert submit(s, request(10.6), now=10.6).valid


def test_clock_rewind_requires_explicit_reset_even_after_clock_catches_up():
    s = selector()
    submit(s)
    assert s.current(now_s=9.0).reason == 'clock_reset_required'
    assert not submit(s, request(11), now=11).valid
    s.reset()
    assert not s.current(now_s=1).valid and s.retained_reference_ids == 0
    assert submit(s, request(1), now=1).valid


@pytest.mark.parametrize('changes,reason', [
    ({'frame_id': 'base_link'}, 'frame_mismatch'),
    ({'v_max_mps': -1}, 'invalid_longitudinal_metadata'),
    ({'v_max_mps': math.nan}, 'invalid_longitudinal_metadata'),
    ({'v_max_mps': True}, 'invalid_longitudinal_metadata'),
    ({'stop_global_s': math.inf}, 'invalid_longitudinal_metadata'),
    ({'entry_length_m': -1}, 'invalid_path_fields'),
    ({'entry_length_m': 1, 'entry_s': math.nan}, 'invalid_path_fields'),
    ({'reference_release': 1}, 'invalid_path_fields'),
    ({'chain_offset': (1,), 'chain_start_s': (), 'chain_length_m': (5,)}, 'invalid_chain'),
    ({'chain_offset': (1,), 'chain_start_s': (510,), 'chain_length_m': (0,)}, 'invalid_chain'),
])
def test_common_validation_is_atomic(changes, reason):
    s = selector()
    assert_blocked(s, submit(s, request(**changes)), None, reason, 10.0)


@pytest.mark.parametrize('name,changes,reason', [
    ('base', {}, 'reserved_reference_id'),
    ('__ctrl_selection_5', {}, 'reserved_reference_id'),
    ('reference', {'reference_x': (0, 2, 4)}, 'reference_step_limit'),
    ('reference', {'reference_x': (0, 1)}, 'reference array lengths differ'),
])
def test_reference_identity_and_geometry_limits(name, changes, reason):
    s = selector()
    assert_blocked(s, submit(s, reference(name=name, **changes)), None, reason, 10.0)
    assert s.retained_reference_ids == 0


def test_point_limit_and_invalid_conversion_do_not_register_a_path():
    s = selector(max_points=2)
    assert submit(s, reference()).reason == 'reference_point_limit'
    assert s.retained_reference_ids == 0
    assert submit(s, preview(10.1), now=10.1).reason == 'sample budget exceeded'
    assert s.state is None


def test_requests_copy_mutable_array_inputs():
    xs = [0, 1, 2]
    r = reference(reference_x=xs)
    xs[1] = 50
    assert r.reference_x == (0, 1, 2)
    with pytest.raises(FrozenInstanceError):
        r.stamp_s = 20


def message():
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(secs=10, nsecs=100_000_000), frame_id='map'),
        v_max=5.0, stop_s=math.nan, lat_offset=0.0, entry_s=0.0, entry_length_m=0.0,
        return_s=0.0, return_length_m=0.0, chain_offset=[], chain_start_s=[], chain_length_m=[],
        reference_id='', reference_x=[], reference_y=[], reference_global_s=[], reference_release=False)


def test_ros1_field_snapshot_preserves_stamp_units_and_no_stop_sentinel():
    wire = message()
    r = PathRequest.from_message(wire)
    assert r.stamp_s == 10.1 and r.stop_global_s is None
    wire.v_max = 0
    assert r.v_max_mps == 5
    assert selector().submit_message(wire, now_s=10.1).valid


@pytest.mark.parametrize('seconds,reason', [(9, 'request_not_newer'), (11, 'request_from_future')])
def test_discardable_message_is_ordered_before_malformed_array_decoding(seconds, reason):
    s = selector()
    old = submit(s).state
    wire = message()
    wire.header.stamp.secs = seconds
    wire.reference_x = None
    result = s.submit_message(wire, now_s=10.1)
    assert not result.valid and result.reason == reason
    assert s.current(now_s=10.1).valid and s.state is old


def test_fresh_message_decode_failure_advances_observed_order_before_recovery():
    s = selector()
    old = submit(s).state
    wire = message()
    wire.header.stamp.nsecs = 200_000_000
    wire.reference_x = None
    assert_blocked(s, s.submit_message(wire, now_s=10.2), old, 'invalid_message', 10.2)
    assert submit(s, request(10.1), now=10.2).reason == 'request_not_newer'
    assert not s.current(now_s=10.2).valid
    assert submit(s, request(10.3), now=10.3).valid


@pytest.mark.parametrize('value', [-1, 1_000_000_000, True])
def test_malformed_ros_timestamp_blocks_output(value):
    s = selector()
    old = submit(s).state
    wire = message()
    wire.header.stamp.nsecs = value
    assert_blocked(s, s.submit_message(wire, now_s=10.1), old, 'invalid_message', 10.1)


@pytest.mark.parametrize('field,value', [('request_max_age_s', 0), ('future_tolerance_s', -1),
    ('base_step_m', 0), ('max_offset_m', math.inf), ('max_points', True),
    ('max_reference_step_m', 0), ('max_reference_ids', 0)])
def test_invalid_config(field, value):
    with pytest.raises(ValueError):
        config(**{field: value})
