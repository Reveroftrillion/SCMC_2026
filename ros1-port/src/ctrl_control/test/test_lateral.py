"""Requirement/geometry checks for the new standalone steering calculation."""

import math
import random
from dataclasses import FrozenInstanceError, replace

import pytest

from ctrl_control.ctrl_lateral import (
    LateralConfig, Polyline, TrackingReference, VehiclePose, compute_steering,
)


def config(**changes):
    values = dict(wheelbase_m=2.8, lookahead_min_m=4.0, lookahead_max_m=12.0,
                  lookahead_time_s=0.6, max_angle_rad=0.5, max_rate_rad_s=0.8,
                  max_tracking_error_m=3.0, max_heading_error_rad=1.2,
                  pose_max_age_s=0.15, reference_max_age_s=0.5,
                  future_tolerance_s=0.02, max_step_s=0.2)
    values.update(changes)
    return LateralConfig(**values)


def path(points=((-20.0, 0.0), (100.0, 0.0)), path_id='straight'):
    return Polyline(path_id, 'map', points)


def pose(**changes):
    values = dict(x_m=0.0, y_m=0.0, yaw_rad=0.0, speed_mps=6.0,
                  stamp_s=10.0, frame_id='map')
    values.update(changes)
    return VehiclePose(**values)


def reference(line=None, **changes):
    line = line if line is not None else path()
    values = dict(path=line, stamp_s=10.0, search_start_m=0.0,
                  search_end_m=line.length_m)
    values.update(changes)
    return TrackingReference(**values)


def calculate(vehicle=None, ref=None, settings=None, **changes):
    arguments = dict(now_s=10.0, dt_s=0.1, previous_angle_rad=0.0)
    arguments.update(changes)
    return compute_steering(vehicle if vehicle is not None else pose(),
                            ref if ref is not None else reference(),
                            settings if settings is not None else config(), **arguments)


def assert_invalid(result, reason):
    assert not result.valid
    assert result.reason == reason
    assert result.angle_rad is None
    assert result.desired_angle_rad is None


def test_straight_center_has_zero_steering_and_segment_interpolation():
    result = calculate()
    assert result.valid
    assert result.angle_rad == 0.0
    assert result.projection_s_m == pytest.approx(20.0)
    assert result.cross_track_error_m == 0.0
    assert result.target_xy_m == pytest.approx((7.6, 0.0))
    assert result.target_s_m == pytest.approx(27.6)


@pytest.mark.parametrize('speed', [0.0, 2.0, 6.0, 12.0, 30.0])
def test_left_right_reflection_preserves_magnitude(speed):
    left = calculate(pose(y_m=0.8, speed_mps=speed))
    right = calculate(pose(y_m=-0.8, speed_mps=speed))
    assert left.valid and right.valid
    assert left.angle_rad < 0.0 < right.angle_rad
    assert left.angle_rad == pytest.approx(-right.angle_rad)
    assert left.cross_track_error_m == pytest.approx(-right.cross_track_error_m)
    assert left.desired_angle_rad == pytest.approx(-right.desired_angle_rad)


def test_rigid_transform_invariance_on_one_hundred_scenes():
    generator = random.Random(20261025)
    original_pose = pose(x_m=3.0, y_m=0.7, yaw_rad=0.07)
    original = calculate(original_pose)
    assert original.valid
    for _ in range(100):
        rotation = generator.uniform(-math.pi, math.pi)
        dx, dy = generator.uniform(-500, 500), generator.uniform(-500, 500)
        c, s = math.cos(rotation), math.sin(rotation)

        def transform(x, y):
            return c * x - s * y + dx, s * x + c * y + dy

        points = tuple(transform(*point) for point in path().points)
        x, y = transform(original_pose.x_m, original_pose.y_m)
        result = calculate(replace(original_pose, x_m=x, y_m=y,
                                   yaw_rad=original_pose.yaw_rad + rotation),
                           reference(path(points)))
        assert result.valid
        assert result.angle_rad == pytest.approx(original.angle_rad, abs=1e-12)
        assert result.desired_angle_rad == pytest.approx(original.desired_angle_rad, abs=1e-12)
        assert result.cross_track_error_m == pytest.approx(original.cross_track_error_m, abs=1e-10)


@pytest.mark.parametrize('radius,direction', [(10.0, 1), (25.0, 1), (25.0, -1), (80.0, -1)])
def test_known_circle_radius_uses_chord_not_arc_length(radius, direction):
    increment = 0.05
    points = tuple((radius * math.sin(i * increment),
                    direction * radius * (1.0 - math.cos(i * increment)))
                   for i in range(31))
    line = path(points, 'circle')
    # Target is exactly the tenth vertex, which lies on the analytic circle.
    preview = line.stations_m[10]
    settings = config(lookahead_min_m=preview, lookahead_max_m=preview,
                      lookahead_time_s=0.0, max_rate_rad_s=100.0)
    result = calculate(pose(speed_mps=0.0), reference(line), settings)
    assert result.valid
    assert result.target_xy_m == pytest.approx(points[10])
    assert result.curvature_per_m == pytest.approx(direction / radius, abs=1e-12)
    assert result.angle_rad == pytest.approx(direction * math.atan(2.8 / radius), abs=1e-12)
    assert math.hypot(*result.target_vehicle_xy_m) < result.lookahead_m


@pytest.mark.parametrize('speed,expected', [(0.0, 4.0), (2.0, 5.2), (6.0, 7.6), (100.0, 12.0)])
def test_speed_dependent_preview_is_explicitly_bounded(speed, expected):
    result = calculate(pose(speed_mps=speed))
    assert result.valid
    assert result.lookahead_m == pytest.approx(expected)


def test_short_reference_targets_endpoint_without_extrapolation():
    result = calculate(pose(x_m=0.25), reference(path(((0.0, 0.0), (1.0, 0.0)))))
    assert result.valid and result.target_at_end
    assert result.target_xy_m == (1.0, 0.0)
    assert result.target_s_m == 1.0
    assert result.angle_rad == 0.0


@pytest.mark.parametrize('x,reason', [(1.0, 'target_coincident'), (1.5, 'target_not_ahead')])
def test_end_and_passed_end_do_not_issue_a_new_steering(x, reason):
    result = calculate(pose(x_m=x), reference(path(((0.0, 0.0), (1.0, 0.0)))))
    assert_invalid(result, reason)


def test_target_that_turns_back_behind_vehicle_is_rejected():
    line = path(((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (-2.0, 1.0)))
    result = calculate(pose(speed_mps=0.0), reference(line),
                       config(lookahead_min_m=5.0, lookahead_max_m=5.0))
    assert_invalid(result, 'target_not_ahead')


def test_angle_saturation_and_rate_limit_are_distinct():
    settings = config(max_angle_rad=0.02, max_rate_rad_s=100.0)
    saturated = calculate(pose(y_m=-2.0, speed_mps=0.0), settings=settings)
    assert saturated.valid and saturated.angle_limited and not saturated.rate_limited
    assert saturated.angle_rad == 0.02
    limited = calculate(pose(y_m=-2.0, speed_mps=0.0), settings=replace(settings, max_rate_rad_s=0.1))
    assert limited.valid and limited.angle_limited and limited.rate_limited
    assert limited.angle_rad == pytest.approx(0.01)


@pytest.mark.parametrize('previous', [-0.5, -0.2, 0.0, 0.2, 0.5])
@pytest.mark.parametrize('dt', [0.001, 0.02, 0.2])
def test_every_valid_output_obeys_both_actuator_limits(previous, dt):
    settings = config()
    result = calculate(pose(y_m=-2.0), settings=settings,
                       previous_angle_rad=previous, dt_s=dt)
    assert result.valid
    assert abs(result.angle_rad) <= settings.max_angle_rad
    assert abs(result.angle_rad - previous) <= settings.max_rate_rad_s * dt + 1e-15


def test_reference_replacement_uses_new_geometry_but_keeps_rate_continuity():
    first = calculate(pose(y_m=-1.0), reference(path(path_id='first')))
    second_ref = reference(path(((-20.0, -2.0), (100.0, -2.0)), 'replacement'))
    second = calculate(pose(y_m=-1.0), second_ref, previous_angle_rad=first.angle_rad)
    assert first.valid and second.valid
    assert second.path_id == 'replacement'
    assert first.desired_angle_rad > 0.0 > second.desired_angle_rad
    assert abs(second.angle_rad - first.angle_rad) <= config().max_rate_rad_s * 0.1 + 1e-15


def test_explicit_search_window_selects_a_branch_at_a_crossing():
    line = path(((-10.0, -10.0), (10.0, 10.0), (-10.0, 10.0), (10.0, -10.0)))
    vehicle = pose(yaw_rad=math.pi / 4.0)
    assert_invalid(calculate(vehicle, reference(line)), 'ambiguous_projection')
    result = calculate(vehicle, reference(line, search_end_m=line.stations_m[1]))
    assert result.valid
    assert result.projection_s_m == pytest.approx(math.sqrt(200.0))
    assert result.angle_rad == pytest.approx(0.0, abs=1e-15)


def test_projection_is_clipped_to_explicit_search_window():
    line = path(((0.0, 0.0), (30.0, 0.0)))
    result = calculate(pose(x_m=9.0), reference(line, search_start_m=10.0, search_end_m=20.0))
    assert result.valid
    assert result.projection_s_m == 10.0
    assert result.distance_to_path_m == 1.0


@pytest.mark.parametrize('start,end', [(-1.0, 20.0), (20.0, 10.0), (10.0, 10.0), (0.0, 121.0)])
def test_invalid_progress_window_is_rejected(start, end):
    assert_invalid(calculate(ref=reference(search_start_m=start, search_end_m=end)),
                   'invalid_search_window')


@pytest.mark.parametrize('field', ['x_m', 'y_m', 'yaw_rad', 'speed_mps', 'stamp_s'])
@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_pose_has_no_angle(field, bad):
    assert_invalid(calculate(pose(**{field: bad})), 'nonfinite_input')


@pytest.mark.parametrize('argument', ['now_s', 'dt_s', 'previous_angle_rad'])
def test_nonfinite_call_parameters_have_no_angle(argument):
    assert_invalid(calculate(**{argument: float('nan')}), 'nonfinite_input')


@pytest.mark.parametrize('field', ['stamp_s', 'search_start_m', 'search_end_m'])
def test_nonfinite_reference_has_no_angle(field):
    assert_invalid(calculate(ref=reference(**{field: float('inf')})), 'nonfinite_input')


@pytest.mark.parametrize('dt', [0.0, -0.1, 0.200001])
def test_invalid_time_step_is_rejected(dt):
    assert_invalid(calculate(dt_s=dt), 'invalid_step')


@pytest.mark.parametrize('angle', [-0.500001, 0.500001])
def test_out_of_bounds_previous_command_is_not_silently_clipped(angle):
    assert_invalid(calculate(previous_angle_rad=angle), 'previous_angle_out_of_bounds')


@pytest.mark.parametrize('field,value,reason', [
    ('frame_id', 'base_link', 'frame_mismatch'),
    ('speed_mps', -0.01, 'reverse_not_supported'),
    ('y_m', 3.001, 'tracking_error_exceeded'),
    ('yaw_rad', math.pi, 'heading_error_exceeded'),
    ('stamp_s', 9.849, 'pose_stale'),
    ('stamp_s', 10.021, 'pose_from_future'),
])
def test_pose_contract_violations(field, value, reason):
    assert_invalid(calculate(pose(**{field: value})), reason)


@pytest.mark.parametrize('stamp,reason', [(9.499, 'reference_stale'), (10.021, 'reference_from_future')])
def test_reference_age_is_independent_of_pose_age(stamp, reason):
    assert_invalid(calculate(ref=reference(stamp_s=stamp)), reason)


@pytest.mark.parametrize('pose_stamp,reference_stamp', [(9.85, 9.5), (10.02, 10.02), (10.0, 10.0)])
def test_age_boundaries_are_inclusive(pose_stamp, reference_stamp):
    assert calculate(pose(stamp_s=pose_stamp), reference(stamp_s=reference_stamp)).valid


def test_geometric_error_boundaries_are_inclusive():
    assert calculate(pose(y_m=3.0)).valid
    assert calculate(pose(yaw_rad=1.2)).valid


@pytest.mark.parametrize('points', [(), ((0.0, 0.0),), ((0.0, 0.0), (0.0, 0.0)),
                                   ((0.0, 0.0), (1e-7, 0.0)),
                                   ((0.0, 0.0), (math.nan, 1.0)),
                                   ((0.0, 0.0), (math.inf, 1.0)),
                                   ((0.0, 0.0, 0.0), (1.0, 1.0)),
                                   ((0.0, 0.0), 1.0),
                                   ((-1e308, 0.0), (1e308, 0.0))])
def test_invalid_geometry_is_rejected_at_construction(points):
    with pytest.raises(ValueError):
        path(points)


@pytest.mark.parametrize('field,value', [('path_id', ''), ('frame_id', ' '), ('path_id', None)])
def test_missing_path_identity_is_rejected(field, value):
    values = dict(path_id='route', frame_id='map', points=((0.0, 0.0), (10.0, 0.0)))
    values[field] = value
    with pytest.raises(ValueError):
        Polyline(**values)


def test_path_owns_immutable_copy_and_accepts_a_single_pass_iterator():
    source = [[0.0, 0.0], [10.0, 0.0]]
    line = path(iter(source))
    source[1][1] = 50.0
    assert line.points == ((0.0, 0.0), (10.0, 0.0))
    with pytest.raises(FrozenInstanceError):
        line.frame_id = 'changed'
    assert line.point_at(2.5) == (2.5, 0.0)


@pytest.mark.parametrize('station', [-0.01, 120.01, math.nan, math.inf])
def test_path_sampling_rejects_extrapolation(station):
    with pytest.raises(ValueError):
        path().point_at(station)


@pytest.mark.parametrize('field,value', [
    ('wheelbase_m', 0.0), ('lookahead_min_m', 0.0), ('lookahead_max_m', 3.0),
    ('lookahead_time_s', -0.1), ('max_angle_rad', math.pi / 2.0),
    ('max_angle_rad', 0.0), ('max_rate_rad_s', 0.0),
    ('max_tracking_error_m', -1.0), ('max_heading_error_rad', 2.0),
    ('pose_max_age_s', 0.0), ('reference_max_age_s', 0.0),
    ('future_tolerance_s', -0.1), ('max_step_s', 0.0),
    ('wheelbase_m', math.inf), ('wheelbase_m', '2.8'), ('max_step_s', True),
])
def test_invalid_configuration_is_rejected(field, value):
    with pytest.raises(ValueError):
        config(**{field: value})


def test_calculation_is_repeatable_and_has_no_hidden_path_state():
    request = reference()
    vehicle = pose(y_m=0.8)
    first = calculate(vehicle, request)
    calculate(pose(y_m=-0.8), reference(path(path_id='other')))
    assert calculate(vehicle, request) == first


def test_finite_but_unrepresentable_geometry_returns_invalid():
    line = path(((0.0, 0.0), (10.0, 10.0)))
    result = calculate(pose(x_m=1e308, y_m=1e308), reference(line))
    assert not result.valid
    assert result.angle_rad is None
