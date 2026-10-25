"""Standalone rear-axle steering calculation; not connected to a ROS node.

Contract: docs/CONTROL_CONTRACT_2026-10-25.md. Geometry reference:
R. C. Coulter, CMU-RI-TR-92-01 (1992), sections 2-3.
All distances are metres, times seconds, and angles radians. Positive steering
turns left. This module uses only Python's standard library.
"""

import math
from bisect import bisect_right
from dataclasses import dataclass, field
from typing import Optional, Tuple


Point = Tuple[float, float]
_GEOMETRY_EPS_M = 1e-6
_DISTANCE_TIE_M = 1e-9
_TIME_ROUNDOFF_S = 1e-12


def _finite(*values):
    try:
        return all(not isinstance(value, bool) and math.isfinite(value)
                   for value in values)
    except (TypeError, ValueError, OverflowError):
        return False


@dataclass(frozen=True)
class Polyline:
    """An immutable, directed reference with its own zero-based arc length."""

    path_id: str
    frame_id: str
    points: Tuple[Point, ...]
    stations_m: Tuple[float, ...] = field(init=False, repr=False)

    def __post_init__(self):
        if not isinstance(self.path_id, str) or not self.path_id.strip():
            raise ValueError('path_id must be a nonempty string')
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise ValueError('frame_id must be a nonempty string')
        try:
            supplied_points = tuple(self.points)
            if any(len(point) != 2 or not _finite(*point) for point in supplied_points):
                raise ValueError('points must contain finite x/y pairs')
            points = tuple((float(point[0]), float(point[1])) for point in supplied_points)
        except (TypeError, IndexError, OverflowError) as exc:
            raise ValueError('points must contain finite x/y pairs') from exc
        if len(points) < 2:
            raise ValueError('at least two points are required')
        stations = [0.0]
        for start, end in zip(points, points[1:]):
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            station = stations[-1] + length
            if (not _finite(length, station) or length <= _GEOMETRY_EPS_M
                    or station <= stations[-1]):
                raise ValueError('duplicate, too short, or unrepresentable segment')
            stations.append(station)
        object.__setattr__(self, 'points', points)
        object.__setattr__(self, 'stations_m', tuple(stations))

    @property
    def length_m(self):
        return self.stations_m[-1]

    def point_at(self, station_m):
        """Interpolate within this path. Extrapolation is deliberately rejected."""
        if not _finite(station_m) or not 0.0 <= station_m <= self.length_m:
            raise ValueError('station is outside the reference')
        index = min(bisect_right(self.stations_m, station_m) - 1,
                    len(self.points) - 2)
        fraction = ((station_m - self.stations_m[index])
                    / (self.stations_m[index + 1] - self.stations_m[index]))
        start, end = self.points[index:index + 2]
        return (start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]))


@dataclass(frozen=True)
class VehiclePose:
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float
    stamp_s: float
    frame_id: str


@dataclass(frozen=True)
class TrackingReference:
    """A fresh tracking request and an explicit branch-local projection window.

    The window restricts vehicle projection, not lookahead along the path.
    stamp_s is the upstream tracking request time, not the map file's age.
    """

    path: Polyline
    stamp_s: float
    search_start_m: float
    search_end_m: float


@dataclass(frozen=True)
class LateralConfig:
    """Explicit engineering inputs; there is no production vehicle calibration."""

    wheelbase_m: float
    lookahead_min_m: float
    lookahead_max_m: float
    lookahead_time_s: float
    max_angle_rad: float
    max_rate_rad_s: float
    max_tracking_error_m: float
    max_heading_error_rad: float
    pose_max_age_s: float
    reference_max_age_s: float
    future_tolerance_s: float
    max_step_s: float

    def __post_init__(self):
        if not _finite(*vars(self).values()):
            raise ValueError('configuration must be finite numeric values')
        positive = (self.wheelbase_m, self.lookahead_min_m, self.max_rate_rad_s,
                    self.max_tracking_error_m, self.pose_max_age_s,
                    self.reference_max_age_s, self.max_step_s)
        if (any(value <= 0.0 for value in positive)
                or self.lookahead_max_m < self.lookahead_min_m
                or self.lookahead_time_s < 0.0 or self.future_tolerance_s < 0.0
                or not 0.0 < self.max_angle_rad < math.pi / 2.0
                or not 0.0 < self.max_heading_error_rad <= math.pi / 2.0):
            raise ValueError('configuration limits are inconsistent')


@dataclass(frozen=True)
class SteeringResult:
    valid: bool
    reason: str
    path_id: str
    angle_rad: Optional[float] = None
    desired_angle_rad: Optional[float] = None
    curvature_per_m: Optional[float] = None
    projection_s_m: Optional[float] = None
    distance_to_path_m: Optional[float] = None
    cross_track_error_m: Optional[float] = None
    heading_error_rad: Optional[float] = None
    target_s_m: Optional[float] = None
    target_xy_m: Optional[Point] = None
    target_vehicle_xy_m: Optional[Point] = None
    lookahead_m: Optional[float] = None
    angle_limited: bool = False
    rate_limited: bool = False
    target_at_end: bool = False


@dataclass(frozen=True)
class _Projection:
    station_m: float
    distance_m: float
    cross_track_m: float
    heading_rad: float


@dataclass(frozen=True)
class PositionProjection:
    """Public geometry result for progress management; no time/speed policy."""

    valid: bool
    reason: str
    station_m: Optional[float] = None
    distance_m: Optional[float] = None
    cross_track_m: Optional[float] = None
    heading_rad: Optional[float] = None


def project_position(path: Polyline, pose: VehiclePose,
                     search_start_m: float, search_end_m: float) -> PositionProjection:
    if not _finite(pose.x_m, pose.y_m, search_start_m, search_end_m):
        return PositionProjection(False, 'nonfinite_input')
    if pose.frame_id != path.frame_id:
        return PositionProjection(False, 'frame_mismatch')
    if not 0.0 <= search_start_m < search_end_m <= path.length_m:
        return PositionProjection(False, 'invalid_search_window')
    projection, reason = _project(path, pose, search_start_m, search_end_m)
    if projection is None:
        return PositionProjection(False, reason)
    return PositionProjection(True, 'ok', projection.station_m, projection.distance_m,
                              projection.cross_track_m, projection.heading_rad)


def _project(path, pose, lower, upper):
    """Closest point on clipped segments, with ambiguity reported to the caller."""
    first = max(0, bisect_right(path.stations_m, lower) - 1)
    candidates = []
    for index in range(first, len(path.points) - 1):
        start_s, end_s = path.stations_m[index:index + 2]
        if start_s > upper:
            break
        start, end = path.points[index:index + 2]
        length = end_s - start_s
        tx = (end[0] - start[0]) / length
        ty = (end[1] - start[1]) / length
        along = (pose.x_m - start[0]) * tx + (pose.y_m - start[1]) * ty
        if not _finite(along):
            return None, 'nonfinite_geometry'
        along = max(max(0.0, lower - start_s),
                    min(min(length, upper - start_s), along))
        x, y = start[0] + along * tx, start[1] + along * ty
        dx, dy = pose.x_m - x, pose.y_m - y
        distance = math.hypot(dx, dy)
        cross_track = tx * dy - ty * dx
        if not _finite(distance, cross_track):
            return None, 'nonfinite_geometry'
        candidates.append(_Projection(start_s + along, distance, cross_track,
                                      math.atan2(ty, tx)))
    closest = min(candidates, key=lambda item: item.distance_m)
    for item in candidates:
        if (abs(item.distance_m - closest.distance_m) <= _DISTANCE_TIE_M
                and abs(item.station_m - closest.station_m) > _GEOMETRY_EPS_M):
            return None, 'ambiguous_projection'
    return closest, 'ok'


def compute_steering(pose: VehiclePose, reference: TrackingReference,
                     config: LateralConfig, *, now_s: float, dt_s: float,
                     previous_angle_rad: float) -> SteeringResult:
    """Return a rate-limited steering angle, or an invalid result with no angle.

    The caller owns progress, clock-reset handling, longitudinal decisions, and
    the last *accepted* steering angle. No result is sent to a vehicle here.
    """
    path = reference.path

    def invalid(reason):
        return SteeringResult(False, reason, path.path_id)

    if not _finite(pose.x_m, pose.y_m, pose.yaw_rad, pose.speed_mps,
                   pose.stamp_s, reference.stamp_s, now_s, dt_s,
                   previous_angle_rad, reference.search_start_m,
                   reference.search_end_m):
        return invalid('nonfinite_input')
    if pose.frame_id != path.frame_id:
        return invalid('frame_mismatch')
    if pose.speed_mps < 0.0:
        return invalid('reverse_not_supported')
    if not 0.0 < dt_s <= config.max_step_s:
        return invalid('invalid_step')
    if abs(previous_angle_rad) > config.max_angle_rad:
        return invalid('previous_angle_out_of_bounds')
    for name, stamp, max_age in (
            ('pose', pose.stamp_s, config.pose_max_age_s),
            ('reference', reference.stamp_s, config.reference_max_age_s)):
        age = now_s - stamp
        if not math.isfinite(age):
            return invalid('nonfinite_input')
        if age < -config.future_tolerance_s - _TIME_ROUNDOFF_S:
            return invalid(name + '_from_future')
        if age > max_age + _TIME_ROUNDOFF_S:
            return invalid(name + '_stale')
    if not 0.0 <= reference.search_start_m < reference.search_end_m <= path.length_m:
        return invalid('invalid_search_window')

    projection, reason = _project(path, pose, reference.search_start_m,
                                  reference.search_end_m)
    if projection is None:
        return invalid(reason)
    if projection.distance_m > config.max_tracking_error_m:
        return invalid('tracking_error_exceeded')
    heading_error = math.atan2(math.sin(projection.heading_rad - pose.yaw_rad),
                               math.cos(projection.heading_rad - pose.yaw_rad))
    if abs(heading_error) > config.max_heading_error_rad:
        return invalid('heading_error_exceeded')

    lookahead = min(config.lookahead_max_m,
                    config.lookahead_min_m + pose.speed_mps * config.lookahead_time_s)
    target_s = min(path.length_m, projection.station_m + lookahead)
    target = path.point_at(target_s)
    dx, dy = target[0] - pose.x_m, target[1] - pose.y_m
    cosine, sine = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    forward = cosine * dx + sine * dy
    left = -sine * dx + cosine * dy
    distance = math.hypot(forward, left)
    if not _finite(forward, left, distance):
        return invalid('nonfinite_geometry')
    if distance <= _GEOMETRY_EPS_M:
        return invalid('target_coincident')
    if forward <= _GEOMETRY_EPS_M:
        return invalid('target_not_ahead')

    curvature = 2.0 * (left / distance) / distance
    desired = math.atan(config.wheelbase_m * curvature)
    bounded = max(-config.max_angle_rad, min(config.max_angle_rad, desired))
    step_limit = config.max_rate_rad_s * dt_s
    angle = max(previous_angle_rad - step_limit,
                min(previous_angle_rad + step_limit, bounded))
    if not _finite(curvature, desired, step_limit, angle):
        return invalid('nonfinite_geometry')
    return SteeringResult(
        valid=True, reason='ok', path_id=path.path_id,
        angle_rad=angle, desired_angle_rad=desired, curvature_per_m=curvature,
        projection_s_m=projection.station_m, distance_to_path_m=projection.distance_m,
        cross_track_error_m=projection.cross_track_m, heading_error_rad=heading_error,
        target_s_m=target_s, target_xy_m=target,
        target_vehicle_xy_m=(forward, left), lookahead_m=lookahead,
        angle_limited=bounded != desired, rate_limited=angle != bounded,
        target_at_end=target_s == path.length_m)
