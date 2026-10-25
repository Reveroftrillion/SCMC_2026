"""Immutable path conversion with explicit global labels; no legacy helpers."""

import math
from bisect import bisect_right
from dataclasses import dataclass
from typing import Tuple

from .ctrl_lateral import Polyline


def _finite(*values):
    try:
        return all(not isinstance(value, bool) and math.isfinite(value) for value in values)
    except (TypeError, ValueError, OverflowError):
        return False


def _interpolate(value, labels, values):
    if not _finite(value) or not labels[0] <= value <= labels[-1]:
        raise ValueError('station outside path')
    index = min(bisect_right(labels, value) - 1, len(labels) - 2)
    fraction = (value - labels[index]) / (labels[index + 1] - labels[index])
    return values[index] + fraction * (values[index + 1] - values[index])


@dataclass(frozen=True)
class StationedPath:
    path: Polyline
    global_stations_m: Tuple[float, ...]

    def __post_init__(self):
        labels = tuple(self.global_stations_m)
        if (len(labels) != len(self.path.points) or not _finite(*labels)
                or any(b <= a for a, b in zip(labels, labels[1:]))
                or any(not math.isfinite(b - a) for a, b in zip(labels, labels[1:]))):
            raise ValueError('global labels must be finite and strictly increasing')
        object.__setattr__(self, 'global_stations_m', tuple(float(value) for value in labels))

    @classmethod
    def from_global_route(cls, path_id, frame_id, points, *, global_origin_m):
        if not _finite(global_origin_m):
            raise ValueError('finite global origin required')
        path = Polyline(path_id, frame_id, points)
        return cls(path, tuple(global_origin_m + q for q in path.stations_m))

    @classmethod
    def from_reference(cls, path_id, frame_id, xs, ys, global_stations_m):
        xs, ys, labels = tuple(xs), tuple(ys), tuple(global_stations_m)
        if len(xs) != len(ys) or len(xs) != len(labels):
            raise ValueError('reference array lengths differ')
        return cls(Polyline(path_id, frame_id, tuple(zip(xs, ys))), labels)

    def global_at_local(self, local_s_m):
        return _interpolate(local_s_m, self.path.stations_m, self.global_stations_m)

    def local_at_global(self, global_s_m):
        return _interpolate(global_s_m, self.global_stations_m, self.path.stations_m)

    def point_at_global(self, global_s_m):
        return self.path.point_at(self.local_at_global(global_s_m))

    def local_window(self, start_global_s_m, end_global_s_m):
        if (not _finite(start_global_s_m, end_global_s_m)
                or start_global_s_m >= end_global_s_m):
            raise ValueError('global window must increase')
        return self.local_at_global(start_global_s_m), self.local_at_global(end_global_s_m)


@dataclass(frozen=True)
class OffsetMove:
    start_global_s: float
    length_m: float
    target_offset_m: float

    def __post_init__(self):
        if (not _finite(self.start_global_s, self.length_m, self.target_offset_m)
                or self.length_m <= 0.0 or not math.isfinite(self.end_global_s)
                or self.end_global_s <= self.start_global_s):
            raise ValueError('invalid offset move')

    @property
    def end_global_s(self):
        return self.start_global_s + self.length_m


def _ordered_moves(moves):
    moves = tuple(moves)
    if any(b.start_global_s < a.end_global_s for a, b in zip(moves, moves[1:])):
        raise ValueError('offset moves overlap or are unordered')
    return moves


def moves_from_chain(offsets, starts, lengths):
    offsets, starts, lengths = tuple(offsets), tuple(starts), tuple(lengths)
    if not len(offsets) == len(starts) == len(lengths):
        raise ValueError('chain array lengths differ')
    return _ordered_moves(OffsetMove(start, length, offset)
                          for offset, start, length in zip(offsets, starts, lengths))


def entry_return_moves(offset_m, *, entry_start_s, entry_length_m,
                       return_start_s=None, return_length_m=None):
    moves = [OffsetMove(entry_start_s, entry_length_m, offset_m)]
    if (return_start_s is None) != (return_length_m is None):
        raise ValueError('return start and length must be supplied together')
    if return_start_s is not None:
        moves.append(OffsetMove(return_start_s, return_length_m, 0.0))
    return _ordered_moves(moves)


def offset_at(global_s_m, moves, initial_offset_m):
    """Evaluate an already validated schedule; holds the last completed offset."""
    previous = initial_offset_m
    for move in moves:
        if global_s_m < move.start_global_s:
            break
        if global_s_m < move.end_global_s:
            u = (global_s_m - move.start_global_s) / move.length_m
            blend = u ** 3 * (10.0 + u * (-15.0 + 6.0 * u))
            return previous + (move.target_offset_m - previous) * blend
        previous = move.target_offset_m
    return previous


def _tangents(path):
    segments = []
    for a, b in zip(path.points, path.points[1:]):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        segments.append(((b[0] - a[0]) / length, (b[1] - a[1]) / length))
    tangents = [segments[0]]
    for a, b in zip(segments, segments[1:]):
        x, y = a[0] + b[0], a[1] + b[1]
        length = math.hypot(x, y)
        if length <= 1e-6:
            raise ValueError('reversing base vertex has no offset normal')
        tangents.append((x / length, y / length))
    return tuple(tangents + [segments[-1]])


def _tangent_at(path, tangents, local_s):
    index = min(bisect_right(path.stations_m, local_s) - 1, len(path.points) - 2)
    u = ((local_s - path.stations_m[index])
         / (path.stations_m[index + 1] - path.stations_m[index]))
    a, b = tangents[index:index + 2]
    x, y = a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])
    length = math.hypot(x, y)
    if length <= 1e-6:
        raise ValueError('undefined offset normal')
    return x / length, y / length


def build_offset_path(base: StationedPath, moves, *, path_id,
                      initial_offset_m, base_step_m, max_offset_m, max_points):
    """Sample a bounded offset schedule and retain original global station labels.

    This checks representation/direction, not road occupancy or collision safety.
    """
    if (not _finite(initial_offset_m, base_step_m, max_offset_m)
            or base_step_m <= 1e-6 or max_offset_m <= 0.0
            or not isinstance(max_points, int) or isinstance(max_points, bool) or max_points < 2):
        raise ValueError('invalid sampling limits')
    if path_id == base.path.path_id:
        raise ValueError('offset path needs a distinct identity')
    moves = _ordered_moves(moves)
    if abs(initial_offset_m) > max_offset_m or any(abs(move.target_offset_m) > max_offset_m for move in moves):
        raise ValueError('offset exceeds supplied limit')
    if any(move.start_global_s < base.global_stations_m[0]
           or move.end_global_s > base.global_stations_m[-1] for move in moves):
        raise ValueError('move outside base path')
    ratio = base.path.length_m / base_step_m
    if not math.isfinite(ratio) or ratio > max_points or len(base.path.points) > max_points:
        raise ValueError('sample budget exceeded')
    samples = set(base.path.stations_m)
    samples.update(i * base_step_m for i in range(math.ceil(ratio)))
    samples.add(base.path.length_m)
    for move in moves:
        samples.update((base.local_at_global(move.start_global_s), base.local_at_global(move.end_global_s)))
    ordered = []
    for local_s in sorted(samples):
        if ordered and local_s - ordered[-1] <= 1e-6:
            if local_s == base.path.length_m:
                ordered[-1] = local_s
            continue
        ordered.append(local_s)
    if len(ordered) > max_points:
        raise ValueError('sample budget exceeded')
    tangents = _tangents(base.path)
    points, labels = [], []
    for local_s in ordered:
        label = base.global_at_local(local_s)
        offset = offset_at(label, moves, initial_offset_m)
        x, y = base.path.point_at(local_s)
        tx, ty = _tangent_at(base.path, tangents, local_s)
        points.append((x - offset * ty, y + offset * tx))
        labels.append(label)
    for index, (a, b) in enumerate(zip(points, points[1:])):
        tx, ty = _tangent_at(base.path, tangents, (ordered[index] + ordered[index + 1]) / 2.0)
        if (b[0] - a[0]) * tx + (b[1] - a[1]) * ty <= 0.0:
            raise ValueError('offset path reverses base direction')
    return StationedPath(Polyline(path_id, base.path.frame_id, tuple(points)), tuple(labels))
