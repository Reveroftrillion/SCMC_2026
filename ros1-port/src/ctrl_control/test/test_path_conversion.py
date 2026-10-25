"""Station-label and path-shape requirements independent of legacy code."""

import math
from dataclasses import FrozenInstanceError

import pytest

from ctrl_control.ctrl_lateral import VehiclePose, compute_steering
from ctrl_control.ctrl_path_conversion import (
    OffsetMove, StationedPath, build_offset_path, entry_return_moves,
    moves_from_chain, offset_at,
)
from ctrl_control.ctrl_progress import ProgressTracker
from test_lateral import config as lateral_config
from test_progress import config as progress_config


def base(origin=500.0):
    return StationedPath.from_global_route('base', 'map', ((0.0, 0.0), (60.0, 0.0)),
                                           global_origin_m=origin)


def offset_path(moves=(), initial=0.0, **changes):
    arguments = dict(path_id='offset', initial_offset_m=initial,
                     base_step_m=0.5, max_offset_m=4.0, max_points=1000)
    arguments.update(changes)
    return build_offset_path(base(), moves, **arguments)


@pytest.mark.parametrize('origin', [0.0, 1000.0, -5.0])
def test_general_route_origin_is_explicit(origin):
    route = base(origin)
    assert route.global_at_local(0.0) == origin
    assert route.global_at_local(17.25) == origin + 17.25
    assert route.local_at_global(origin + 60.0) == 60.0
    assert route.local_window(origin + 10, origin + 20) == (10.0, 20.0)


def test_nonuniform_global_labels_have_an_invertible_piecewise_mapping():
    route = StationedPath.from_reference('local', 'map', [0, 3, 3], [0, 0, 4], [10, 20, 50])
    assert route.path.length_m == 7.0
    assert route.global_at_local(1.5) == 15.0
    assert route.local_at_global(35.0) == 5.0
    assert route.point_at_global(35.0) == (3.0, 2.0)
    for index in range(101):
        q = route.path.length_m * index / 100
        assert route.local_at_global(route.global_at_local(q)) == pytest.approx(q)


@pytest.mark.parametrize('value', [499.9, 560.1, math.nan, math.inf])
def test_global_mapping_refuses_extrapolation(value):
    with pytest.raises(ValueError):
        base().local_at_global(value)


@pytest.mark.parametrize('value', [-0.01, 60.01, math.nan, math.inf])
def test_local_mapping_refuses_extrapolation(value):
    with pytest.raises(ValueError):
        base().global_at_local(value)


@pytest.mark.parametrize('start,end', [(520, 510), (510, 510), (499, 510), (510, 561), (math.nan, 520)])
def test_invalid_global_window(start, end):
    with pytest.raises(ValueError):
        base().local_window(start, end)


@pytest.mark.parametrize('xs,ys,labels', [
    ([], [], []), ([0], [0], [500]), ([0, 1], [0], [500, 501]),
    ([0, 1], [0, 0], [500]), ([0, 1], [0, 0], [500, 500]),
    ([0, 1], [0, 0], [501, 500]), ([0, 0], [0, 0], [500, 501]),
    ([0, math.nan], [0, 0], [500, 501]), ([0, 1], [0, 0], [500, math.inf]),
    ([0, 1], [0, 0], [-1e308, 1e308]),
])
def test_invalid_reference_arrays_are_rejected_together(xs, ys, labels):
    with pytest.raises(ValueError):
        StationedPath.from_reference('reference', 'map', xs, ys, labels)


def test_reference_arrays_are_copied_and_frozen():
    xs, ys, labels = [0, 10], [0, 0], [1000, 1020]
    route = StationedPath.from_reference('reference', 'map', xs, ys, labels)
    xs[1], ys[1], labels[1] = 50, 50, 1100
    assert route.path.points[-1] == (10.0, 0.0)
    assert route.global_stations_m[-1] == 1020.0
    with pytest.raises(FrozenInstanceError):
        route.global_stations_m = (0.0, 1.0)


def test_entry_hold_return_geometry_and_station_labels():
    moves = entry_return_moves(2.0, entry_start_s=510.0, entry_length_m=10.0,
                               return_start_s=530.0, return_length_m=10.0)
    route = offset_path(moves)
    for station, expected_y in [(500, 0), (510, 0), (515, 1), (520, 2),
                                (525, 2), (530, 2), (535, 1), (540, 0), (560, 0)]:
        assert route.point_at_global(station) == pytest.approx((station - 500.0, expected_y))
    assert route.path.length_m > 60.0
    assert route.global_at_local(route.path.length_m) == 560.0
    assert route.local_at_global(520.0) > 20.0


def test_offset_can_start_from_an_explicit_nonzero_initial_offset():
    moves = (OffsetMove(510.0, 10.0, 2.0),)
    route = offset_path(moves, initial=1.0)
    assert route.point_at_global(500.0)[1] == 1.0
    assert route.point_at_global(515.0)[1] == 1.5
    assert route.point_at_global(560.0)[1] == 2.0


def test_chain_and_entry_return_describe_the_same_explicit_schedule():
    chain = moves_from_chain([2, 0], [510, 530], [10, 10])
    preview = entry_return_moves(2, entry_start_s=510, entry_length_m=10,
                                 return_start_s=530, return_length_m=10)
    assert chain == preview
    assert offset_path(chain).path.points == offset_path(preview).path.points


def test_touching_transitions_are_allowed_and_end_derivatives_are_zero():
    moves = moves_from_chain([2, -1], [510, 520], [10, 10])
    assert offset_at(520, moves, 0.0) == 2.0
    for station in [510, 520, 530]:
        h = 1e-3
        left, middle, right = [offset_at(s, moves, 0.0) for s in [station - h, station, station + h]]
        assert abs((right - left) / (2 * h)) < 1e-5
        assert abs((right - 2 * middle + left) / h ** 2) < 1e-3


def test_reflection_changes_offset_sign_but_not_station_mapping():
    left = offset_path(entry_return_moves(2, entry_start_s=510, entry_length_m=10))
    right = offset_path(entry_return_moves(-2, entry_start_s=510, entry_length_m=10))
    for a, b in zip(left.path.points, right.path.points):
        assert a[0] == b[0] and a[1] == -b[1]
    assert left.path.stations_m == right.path.stations_m
    assert left.global_stations_m == right.global_stations_m


def test_constant_left_offset_moves_inside_an_analytic_left_circle():
    radius = 25.0
    points = tuple((radius * math.sin(i * 0.05), radius * (1 - math.cos(i * 0.05))) for i in range(41))
    route = StationedPath.from_global_route('circle', 'map', points, global_origin_m=1000.0)
    shifted = build_offset_path(route, (), path_id='inner', initial_offset_m=2.0,
                                base_step_m=0.25, max_offset_m=4.0, max_points=2000)
    for label in route.global_stations_m[1:-1]:
        x, y = shifted.point_at_global(label)
        assert math.hypot(x, y - radius) == pytest.approx(radius - 2.0, abs=1e-9)


def test_rigid_transform_preserves_offset_shape_and_global_labels():
    original = base()
    rotation, dx, dy = 1.1, 80.0, -25.0
    c, s = math.cos(rotation), math.sin(rotation)
    transform = lambda x, y: (c * x - s * y + dx, s * x + c * y + dy)
    rotated = StationedPath.from_reference('rotated', 'map',
        [transform(*p)[0] for p in original.path.points],
        [transform(*p)[1] for p in original.path.points], original.global_stations_m)
    moves = entry_return_moves(2, entry_start_s=510, entry_length_m=10)
    a = offset_path(moves)
    b = build_offset_path(rotated, moves, path_id='shifted', initial_offset_m=0,
                          base_step_m=0.5, max_offset_m=4, max_points=1000)
    for label in [500, 510, 515, 520, 540, 560]:
        assert b.point_at_global(label) == pytest.approx(transform(*a.point_at_global(label)), abs=1e-9)


@pytest.mark.parametrize('start,length,offset', [(510, 0, 1), (510, -1, 1),
                                               (math.nan, 10, 1), (510, 10, math.inf), (1e308, 1, 1)])
def test_invalid_move(start, length, offset):
    with pytest.raises(ValueError):
        OffsetMove(start, length, offset)


@pytest.mark.parametrize('offsets,starts,lengths', [([1, 0], [510], [10, 10]),
                                                 ([1, 0], [510, 515], [10, 10]),
                                                 ([1, 0], [530, 510], [5, 5])])
def test_invalid_chain_schedule(offsets, starts, lengths):
    with pytest.raises(ValueError):
        moves_from_chain(offsets, starts, lengths)


def test_incomplete_return_preview_is_not_inferred():
    with pytest.raises(ValueError):
        entry_return_moves(1, entry_start_s=510, entry_length_m=10, return_start_s=530)


@pytest.mark.parametrize('changes', [dict(base_step_m=0), dict(base_step_m=math.inf),
                                    dict(max_offset_m=0), dict(max_points=10),
                                    dict(max_points=True), dict(initial_offset_m=5),
                                    dict(initial_offset_m=math.nan), dict(path_id='base')])
def test_invalid_conversion_limits(changes):
    with pytest.raises(ValueError):
        offset_path(**changes)


@pytest.mark.parametrize('move', [OffsetMove(499, 5, 1), OffsetMove(555, 10, 1), OffsetMove(510, 10, 5)])
def test_transition_must_fit_base_and_offset_bounds(move):
    with pytest.raises(ValueError):
        offset_path((move,))


def test_reversing_geometry_and_too_large_inner_offset_are_rejected():
    back = StationedPath.from_global_route('back', 'map', [(0, 0), (10, 0), (0, 0)], global_origin_m=0)
    with pytest.raises(ValueError, match='normal'):
        build_offset_path(back, (), path_id='shifted', initial_offset_m=1,
                          base_step_m=0.5, max_offset_m=4, max_points=100)
    radius = 2.0
    points = [(radius * math.sin(i * 0.1), radius * (1 - math.cos(i * 0.1))) for i in range(20)]
    circle = StationedPath.from_global_route('small', 'map', points, global_origin_m=0)
    with pytest.raises(ValueError, match='reverses'):
        build_offset_path(circle, (), path_id='inside', initial_offset_m=3,
                          base_step_m=0.1, max_offset_m=4, max_points=1000)


@pytest.mark.parametrize('kind', ['global', 'offset', 'reference'])
def test_all_three_forms_connect_to_progress_and_steering(kind):
    if kind == 'global':
        route = base()
    elif kind == 'offset':
        route = offset_path(entry_return_moves(2, entry_start_s=510, entry_length_m=10))
    else:
        route = StationedPath.from_reference('local', 'map', [0, 10, 60], [0, 0, 0], [500, 520, 620])
    x, y = route.point_at_global(505)
    vehicle = VehiclePose(x, y, 0.0, 2.0, 10.0, 'map')
    tracker = ProgressTracker(progress_config())
    lower, upper = route.local_window(500, 510)
    result = tracker.acquire(route.path, vehicle, now_s=10, reference_stamp_s=10,
                              search_start_m=lower, search_end_m=upper)
    assert result.valid
    assert route.global_at_local(result.state.station_m) == pytest.approx(505)
    steering = compute_steering(vehicle, result.reference, lateral_config(), now_s=10,
                                dt_s=0.1, previous_angle_rad=0)
    assert steering.valid
    assert route.global_at_local(steering.projection_s_m) == pytest.approx(505)
