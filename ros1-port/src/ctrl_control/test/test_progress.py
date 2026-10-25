"""Progress requirements: branch ownership, timing, motion and atomic updates."""

import math
from dataclasses import replace

import pytest

from ctrl_control.ctrl_lateral import Polyline, VehiclePose, compute_steering, project_position
from ctrl_control.ctrl_progress import ProgressConfig, ProgressTracker
from test_lateral import config as lateral_config


def config(**changes):
    values = dict(max_speed_mps=20.0, backtrack_m=1.0, forward_margin_m=0.3,
                  max_gap_s=0.25, max_tracking_error_m=3.0,
                  max_heading_error_rad=1.2, pose_max_age_s=0.15,
                  reference_max_age_s=0.5, future_tolerance_s=0.02,
                  end_tolerance_m=0.1)
    values.update(changes)
    return ProgressConfig(**values)


def line(points=((0.0, 0.0), (100.0, 0.0)), name='route'):
    return Polyline(name, 'map', points)


def pose(x=10.0, stamp=10.0, **changes):
    return replace(VehiclePose(x, 0.0, 0.0, 2.0, stamp, 'map'), **changes)


def acquire(tracker, path=None, vehicle=None, **changes):
    path = path if path is not None else line()
    vehicle = vehicle if vehicle is not None else pose()
    values = dict(now_s=10.0, reference_stamp_s=10.0,
                  search_start_m=0.0, search_end_m=path.length_m)
    values.update(changes)
    return tracker.acquire(path, vehicle, **values)


def update(tracker, x=10.2, now=10.1, **changes):
    values = dict(now_s=now, reference_stamp_s=now)
    values.update(changes)
    return tracker.update(pose(x, now), **values)


def assert_failure(tracker, result, old, reason):
    assert not result.valid and result.reason == reason
    assert result.state is None and result.reference is None
    assert tracker.state is old


def test_requires_explicit_initial_acquisition_and_reset():
    tracker = ProgressTracker(config())
    assert_failure(tracker, update(tracker), None, 'not_acquired')
    assert acquire(tracker).valid
    tracker.reset()
    assert tracker.state is None
    assert_failure(tracker, update(tracker), None, 'not_acquired')


def test_sequential_progress_reference_and_steering_integration():
    tracker = ProgressTracker(config())
    assert acquire(tracker).valid
    for step in range(1, 51):
        now = 10.0 + 0.1 * step
        vehicle = pose(10.0 + step * 0.2, now)
        result = tracker.update(vehicle, now_s=now, reference_stamp_s=now)
        assert result.valid
        assert result.state.station_m == pytest.approx(vehicle.x_m)
        assert result.state.max_station_m == pytest.approx(vehicle.x_m)
        assert result.reference.search_start_m <= vehicle.x_m <= result.reference.search_end_m
        output = compute_steering(vehicle, result.reference, lateral_config(), now_s=now,
                                  dt_s=0.1, previous_angle_rad=0.0)
        assert output.valid and output.angle_rad == 0.0


def test_small_stationary_noise_does_not_erase_maximum_progress():
    tracker = ProgressTracker(config())
    acquire(tracker, vehicle=pose(speed_mps=0.0))
    for index, x in enumerate([10.1, 9.9, 10.05, 9.8, 10.0]):
        result = update(tracker, x, 10.1 + index * 0.1)
        assert result.valid
        assert result.state.station_m == pytest.approx(x)
        assert result.state.max_station_m == pytest.approx(10.1)


def test_search_window_respects_motion_budget_and_high_water_backtrack():
    tracker = ProgressTracker(config())
    acquire(tracker)
    result = update(tracker)
    assert result.valid
    assert result.reference.search_start_m == 9.0
    assert result.reference.search_end_m == pytest.approx(12.3)


def test_self_crossing_acquisition_requires_an_explicit_branch():
    path = line(((-10.0, -10.0), (10.0, 10.0), (-10.0, 10.0), (10.0, -10.0)))
    tracker = ProgressTracker(config())
    vehicle = pose(0.0, yaw_rad=math.pi / 4)
    bad = acquire(tracker, path, vehicle)
    assert_failure(tracker, bad, None, 'ambiguous_projection')
    result = acquire(tracker, path, vehicle, search_end_m=path.stations_m[1])
    assert result.valid
    for step in range(1, 8):
        now = 10.0 + 0.1 * step
        result = tracker.update(pose(0.1 * step, now, y_m=0.1 * step, yaw_rad=math.pi / 4),
                                now_s=now, reference_stamp_s=now)
        assert result.valid
        assert result.state.station_m < path.stations_m[1]


def test_close_start_and_end_remain_on_seeded_start_branch():
    path = line(((0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0), (0.0, 0.05)))
    tracker = ProgressTracker(config())
    result = acquire(tracker, path, pose(0.0, y_m=0.05), search_end_m=5.0)
    assert result.valid and result.state.station_m == 0.0
    assert not result.state.completed
    result = update(tracker, 0.2)
    assert result.valid and result.state.station_m == pytest.approx(0.2)
    assert result.reference.search_end_m < 5.0


def test_excessive_backward_motion_cannot_hide_at_clipped_window_boundary():
    tracker = ProgressTracker(config())
    acquire(tracker)
    old = tracker.state
    assert_failure(tracker, update(tracker, 8.9), old, 'backtrack_exceeded')
    # A rejected sample did not move the window backwards.
    result = update(tracker, 9.1, 10.2)
    assert result.valid and result.state.max_station_m == 10.0


def test_large_position_jump_does_not_trigger_global_reacquisition():
    tracker = ProgressTracker(config())
    acquire(tracker)
    old = tracker.state
    assert_failure(tracker, update(tracker, 50.0), old, 'position_jump')
    assert update(tracker, 10.4, 10.2).valid


def test_gap_requires_explicit_reacquisition_instead_of_expanding_window():
    tracker = ProgressTracker(config())
    acquire(tracker)
    old = tracker.state
    assert_failure(tracker, update(tracker, 10.0, 10.251), old, 'reacquire_required')
    assert acquire(tracker, vehicle=pose(60.0, 11.0), now_s=11.0,
                   reference_stamp_s=11.0, search_start_m=55.0, search_end_m=65.0).valid
    assert tracker.state.station_m == 60.0


@pytest.mark.parametrize('now,stamp,reference_stamp,reason', [
    (10.0, 10.0, 10.0, 'clock_not_advancing'),
    (9.99, 9.99, 9.99, 'clock_not_advancing'),
    (10.1, 10.0, 10.1, 'pose_not_advancing'),
    (10.1, 9.99, 10.1, 'pose_not_advancing'),
    (10.1, 10.1, 9.99, 'reference_time_regressed'),
    (10.2, 10.0, 10.2, 'pose_stale'),
    (10.1, 10.121, 10.1, 'pose_from_future'),
    (10.1, 10.1, 9.5, 'reference_stale'),
    (10.1, 10.1, 10.121, 'reference_from_future'),
])
def test_time_failures_are_atomic(now, stamp, reference_stamp, reason):
    tracker = ProgressTracker(config())
    acquire(tracker)
    old = tracker.state
    result = tracker.update(pose(10.1, stamp), now_s=now, reference_stamp_s=reference_stamp)
    assert_failure(tracker, result, old, reason)


def test_equal_reference_timestamp_is_allowed_until_it_expires():
    tracker = ProgressTracker(config())
    acquire(tracker)
    assert update(tracker, reference_stamp_s=10.0).valid
    assert tracker.state.reference_stamp_s == 10.0


def test_gap_and_motion_boundaries_are_inclusive():
    tracker = ProgressTracker(config())
    acquire(tracker)
    assert update(tracker, 15.3, 10.25).valid


def test_new_reference_has_its_own_station_origin_even_with_the_same_id():
    tracker = ProgressTracker(config())
    acquire(tracker)
    replacement = line(((8.0, 0.0), (40.0, 0.0)))
    result = acquire(tracker, replacement, pose(10.0, 10.1), now_s=10.1,
                     reference_stamp_s=10.1, search_end_m=5.0)
    assert result.valid and result.state.station_m == 2.0
    assert result.state.max_station_m == 2.0
    assert result.reference.path is replacement


def test_failed_replacement_keeps_old_path_and_returns_no_reference():
    tracker = ProgressTracker(config())
    acquire(tracker)
    old = tracker.state
    result = acquire(tracker, line(((0.0, 50.0), (100.0, 50.0))), pose())
    assert_failure(tracker, result, old, 'tracking_error_exceeded')
    assert update(tracker).valid


def test_completion_needs_both_remaining_station_and_endpoint_distance():
    tracker = ProgressTracker(config())
    result = acquire(tracker, vehicle=pose(99.95, y_m=0.2))
    assert result.valid and not result.state.completed
    result = tracker.update(pose(100.0, 10.1), now_s=10.1, reference_stamp_s=10.1)
    assert result.valid and result.state.completed
    assert result.reference.search_start_m < result.reference.search_end_m
    result = update(tracker, 99.95, 10.2)
    assert result.valid and result.state.completed
    old = tracker.state
    assert_failure(tracker, update(tracker, 0.0, 10.3), old, 'position_jump')


@pytest.mark.parametrize('changes,reason', [
    ({'frame_id': 'base_link'}, 'frame_mismatch'),
    ({'speed_mps': -0.01}, 'speed_out_of_bounds'),
    ({'speed_mps': 20.01}, 'speed_out_of_bounds'),
    ({'x_m': math.nan}, 'nonfinite_input'),
    ({'yaw_rad': math.inf}, 'nonfinite_input'),
    ({'yaw_rad': math.pi}, 'heading_error_exceeded'),
    ({'y_m': 3.01}, 'tracking_error_exceeded'),
])
def test_invalid_acquisition_has_no_state(changes, reason):
    tracker = ProgressTracker(config())
    assert_failure(tracker, acquire(tracker, vehicle=pose(**changes)), None, reason)


@pytest.mark.parametrize('start,end,reason', [(-1.0, 10.0, 'invalid_search_window'),
                                             (10.0, 10.0, 'invalid_search_window'),
                                             (0.0, math.inf, 'nonfinite_input')])
def test_invalid_seed_window_is_rejected(start, end, reason):
    tracker = ProgressTracker(config())
    assert_failure(tracker, acquire(tracker, search_start_m=start, search_end_m=end), None, reason)


@pytest.mark.parametrize('field,value', [
    ('max_speed_mps', 0.0), ('backtrack_m', 0.0), ('forward_margin_m', -1.0),
    ('max_gap_s', 0.0), ('max_tracking_error_m', 0.0),
    ('max_heading_error_rad', math.pi), ('pose_max_age_s', -1.0),
    ('reference_max_age_s', 0.0), ('future_tolerance_s', -1.0),
    ('end_tolerance_m', 0.0), ('max_speed_mps', math.inf), ('backtrack_m', True),
])
def test_invalid_configuration(field, value):
    with pytest.raises(ValueError):
        config(**{field: value})


def test_public_projection_interpolates_and_does_not_apply_clock_policy():
    result = project_position(line(), pose(12.5, stamp=-1e6), 0.0, 20.0)
    assert result.valid and result.station_m == 12.5
    assert result.distance_m == 0.0


def test_projected_result_is_identical_to_steering_projection():
    vehicle = pose(12.5, y_m=0.4)
    tracker = ProgressTracker(config())
    progress = acquire(tracker, vehicle=vehicle)
    result = compute_steering(vehicle, progress.reference, lateral_config(),
                              now_s=10.0, dt_s=0.1, previous_angle_rad=0.0)
    assert result.valid
    assert result.projection_s_m == progress.state.station_m
