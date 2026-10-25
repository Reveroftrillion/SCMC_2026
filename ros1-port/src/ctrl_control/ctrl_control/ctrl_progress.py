"""Explicit acquisition and bounded progress on the new standalone polyline."""

import math
from dataclasses import dataclass
from typing import Optional

from .ctrl_lateral import Polyline, TrackingReference, VehiclePose, project_position


def _finite(*values):
    try:
        return all(not isinstance(value, bool) and math.isfinite(value) for value in values)
    except (TypeError, ValueError, OverflowError):
        return False


@dataclass(frozen=True)
class ProgressConfig:
    max_speed_mps: float
    backtrack_m: float
    forward_margin_m: float
    max_gap_s: float
    max_tracking_error_m: float
    max_heading_error_rad: float
    pose_max_age_s: float
    reference_max_age_s: float
    future_tolerance_s: float
    end_tolerance_m: float

    def __post_init__(self):
        if not _finite(*vars(self).values()):
            raise ValueError('progress configuration must be finite numeric values')
        positive = (self.max_speed_mps, self.backtrack_m, self.max_gap_s,
                    self.max_tracking_error_m, self.pose_max_age_s,
                    self.reference_max_age_s, self.end_tolerance_m)
        if (any(value <= 0.0 for value in positive) or self.forward_margin_m < 0.0
                or self.future_tolerance_s < 0.0
                or not 0.0 < self.max_heading_error_rad <= math.pi / 2.0):
            raise ValueError('inconsistent progress limits')


@dataclass(frozen=True)
class ProgressState:
    path: Polyline
    station_m: float
    max_station_m: float
    pose: VehiclePose
    accepted_at_s: float
    reference_stamp_s: float
    completed: bool


@dataclass(frozen=True)
class ProgressResult:
    valid: bool
    reason: str
    state: Optional[ProgressState] = None
    reference: Optional[TrackingReference] = None


class ProgressTracker:
    def __init__(self, config: ProgressConfig):
        self.config = config
        self._state = None

    @property
    def state(self):
        """Last accepted state, including after failure; diagnostic access only."""
        return self._state

    def reset(self):
        self._state = None

    def _input_error(self, path, pose, now_s, reference_stamp_s):
        cfg = self.config
        if not _finite(pose.x_m, pose.y_m, pose.yaw_rad, pose.speed_mps,
                       pose.stamp_s, now_s, reference_stamp_s):
            return 'nonfinite_input'
        if pose.frame_id != path.frame_id:
            return 'frame_mismatch'
        if not 0.0 <= pose.speed_mps <= cfg.max_speed_mps:
            return 'speed_out_of_bounds'
        for name, stamp, maximum in (('pose', pose.stamp_s, cfg.pose_max_age_s),
                                     ('reference', reference_stamp_s, cfg.reference_max_age_s)):
            age = now_s - stamp
            if not math.isfinite(age):
                return 'nonfinite_input'
            if age < -cfg.future_tolerance_s - 1e-12:
                return name + '_from_future'
            if age > maximum + 1e-12:
                return name + '_stale'
        return None

    def _accept(self, path, pose, now_s, reference_stamp_s, lower, upper, prior):
        projection = project_position(path, pose, lower, upper)
        if not projection.valid:
            return ProgressResult(False, projection.reason)
        cfg = self.config
        if projection.distance_m > cfg.max_tracking_error_m:
            return ProgressResult(False, 'tracking_error_exceeded')
        heading = math.atan2(math.sin(projection.heading_rad - pose.yaw_rad),
                             math.cos(projection.heading_rad - pose.yaw_rad))
        if abs(heading) > cfg.max_heading_error_rad:
            return ProgressResult(False, 'heading_error_exceeded')
        # A clipped projection must not hide motion behind the backtrack boundary.
        point = path.point_at(projection.station_m)
        along = ((pose.x_m - point[0]) * math.cos(projection.heading_rad)
                 + (pose.y_m - point[1]) * math.sin(projection.heading_rad))
        if prior is not None and projection.station_m <= lower + 1e-9 and along < -1e-9:
            return ProgressResult(False, 'backtrack_exceeded')
        maximum = max(prior.max_station_m, projection.station_m) if prior else projection.station_m
        end_distance = math.hypot(pose.x_m - path.points[-1][0], pose.y_m - path.points[-1][1])
        completed = ((prior is not None and prior.completed)
                     or (path.length_m - projection.station_m <= cfg.end_tolerance_m
                         and end_distance <= cfg.end_tolerance_m))
        state = ProgressState(path, projection.station_m, maximum, pose, now_s,
                              reference_stamp_s, completed)
        reference = TrackingReference(path, reference_stamp_s, lower, upper)
        self._state = state
        return ProgressResult(True, 'ok', state, reference)

    def acquire(self, path: Polyline, pose: VehiclePose, *, now_s: float,
                reference_stamp_s: float, search_start_m: float,
                search_end_m: float) -> ProgressResult:
        """Explicit seed/replacement. Failure leaves the old accepted state intact."""
        error = self._input_error(path, pose, now_s, reference_stamp_s)
        if error:
            return ProgressResult(False, error)
        return self._accept(path, pose, now_s, reference_stamp_s,
                            search_start_m, search_end_m, None)

    def update(self, pose: VehiclePose, *, now_s: float,
               reference_stamp_s: float) -> ProgressResult:
        prior = self._state
        if prior is None:
            return ProgressResult(False, 'not_acquired')
        error = self._input_error(prior.path, pose, now_s, reference_stamp_s)
        if error:
            return ProgressResult(False, error)
        if now_s <= prior.accepted_at_s:
            return ProgressResult(False, 'clock_not_advancing')
        if pose.stamp_s <= prior.pose.stamp_s:
            return ProgressResult(False, 'pose_not_advancing')
        if reference_stamp_s < prior.reference_stamp_s:
            return ProgressResult(False, 'reference_time_regressed')
        elapsed = now_s - prior.accepted_at_s
        if elapsed > self.config.max_gap_s + 1e-12:
            return ProgressResult(False, 'reacquire_required')
        budget = self.config.max_speed_mps * elapsed + self.config.forward_margin_m
        displacement = math.hypot(pose.x_m - prior.pose.x_m, pose.y_m - prior.pose.y_m)
        if not _finite(budget, displacement):
            return ProgressResult(False, 'nonfinite_motion')
        if displacement > budget + 1e-9:
            return ProgressResult(False, 'position_jump')
        lower = max(0.0, prior.max_station_m - self.config.backtrack_m)
        upper = min(prior.path.length_m, prior.station_m + budget)
        return self._accept(prior.path, pose, now_s, reference_stamp_s, lower, upper, prior)
