"""Atomic path request selection; no ROS imports or vehicle control output."""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from .ctrl_path_conversion import (
    OffsetMove, StationedPath, build_offset_path, entry_return_moves, moves_from_chain,
)


_RESERVED = '__ctrl_selection_'


def _finite(*values):
    try:
        return all(not isinstance(value, bool) and math.isfinite(value) for value in values)
    except (TypeError, ValueError, OverflowError):
        return False


@dataclass(frozen=True)
class TransitionAnchor:
    start_global_s: float
    length_m: float
    initial_offset_m: float

    def __post_init__(self):
        OffsetMove(self.start_global_s, self.length_m, self.initial_offset_m)


@dataclass(frozen=True)
class PathRequest:
    stamp_s: float
    frame_id: str
    v_max_mps: float = math.inf
    stop_global_s: Optional[float] = None
    lat_offset_m: float = 0.0
    entry_s: float = 0.0
    entry_length_m: float = 0.0
    return_s: float = 0.0
    return_length_m: float = 0.0
    chain_offset: Tuple[float, ...] = ()
    chain_start_s: Tuple[float, ...] = ()
    chain_length_m: Tuple[float, ...] = ()
    reference_id: str = ''
    reference_x: Tuple[float, ...] = ()
    reference_y: Tuple[float, ...] = ()
    reference_global_s: Tuple[float, ...] = ()
    reference_release: bool = False

    def __post_init__(self):
        for name in ('chain_offset', 'chain_start_s', 'chain_length_m',
                     'reference_x', 'reference_y', 'reference_global_s'):
            object.__setattr__(self, name, tuple(getattr(self, name)))

    @staticmethod
    def message_stamp_s(message):
        stamp = message.header.stamp
        seconds, nanoseconds = stamp.secs, stamp.nsecs
        if (not isinstance(seconds, int) or isinstance(seconds, bool)
                or not isinstance(nanoseconds, int) or isinstance(nanoseconds, bool)
                or not 0 <= nanoseconds < 1_000_000_000):
            raise ValueError('invalid ROS1 timestamp')
        return seconds + nanoseconds * 1e-9

    @classmethod
    def from_message(cls, message):
        """Copy ROS1-style fields without importing/generated ROS message types."""
        stop = message.stop_s
        return cls(
            stamp_s=cls.message_stamp_s(message), frame_id=message.header.frame_id,
            v_max_mps=message.v_max, stop_global_s=None if math.isnan(stop) else stop,
            lat_offset_m=message.lat_offset, entry_s=message.entry_s,
            entry_length_m=message.entry_length_m, return_s=message.return_s,
            return_length_m=message.return_length_m,
            **{name: getattr(message, name) for name in (
                'chain_offset', 'chain_start_s', 'chain_length_m', 'reference_id',
                'reference_x', 'reference_y', 'reference_global_s', 'reference_release')})


@dataclass(frozen=True)
class SelectionConfig:
    request_max_age_s: float
    future_tolerance_s: float
    base_step_m: float
    max_offset_m: float
    max_points: int
    max_reference_step_m: float
    max_reference_ids: int

    def __post_init__(self):
        if (not _finite(self.request_max_age_s, self.future_tolerance_s, self.base_step_m,
                        self.max_offset_m, self.max_reference_step_m)
                or self.request_max_age_s <= 0 or self.future_tolerance_s < 0
                or self.base_step_m <= 1e-6 or self.max_offset_m <= 0
                or self.max_reference_step_m <= 1e-6
                or any(not isinstance(v, int) or isinstance(v, bool) for v in
                       (self.max_points, self.max_reference_ids))
                or self.max_points < 2 or self.max_reference_ids < 1):
            raise ValueError('invalid selection configuration')


@dataclass(frozen=True)
class SelectedPath:
    path: StationedPath
    mode: str
    revision: int
    request_stamp_s: float
    v_max_mps: float
    stop_global_s: Optional[float]
    geometry_key: tuple
    moves: tuple = ()
    anchor: Optional[TransitionAnchor] = None
    legacy_spec: Optional[tuple] = None


@dataclass(frozen=True)
class SelectionResult:
    valid: bool
    reason: str
    state: Optional[SelectedPath] = None


class PathSelector:
    def __init__(self, base: StationedPath, config: SelectionConfig):
        if base.path.path_id.startswith(_RESERVED):
            raise ValueError('reserved base path identity')
        self.base, self.config = base, config
        self.reset()

    def reset(self):
        self._state = None
        self._registry = {}
        self._last_request = -math.inf
        self._clock = None
        self._clock_fault = False
        self._blocked = None

    @property
    def state(self):
        """Last accepted state for diagnostics; use current() for valid output."""
        return self._state

    @property
    def retained_reference_ids(self):
        return len(self._registry)

    def _observe_clock(self, now_s):
        if not _finite(now_s):
            self._blocked = 'nonfinite_clock'
            return 'nonfinite_clock'
        if self._clock is not None and now_s < self._clock:
            self._clock_fault = True
        self._clock = now_s
        return 'clock_reset_required' if self._clock_fault else None

    def current(self, *, now_s):
        reason = self._observe_clock(now_s)
        if reason or self._blocked:
            return SelectionResult(False, reason or self._blocked)
        if self._state is None:
            return SelectionResult(False, 'no_request')
        if now_s - self._state.request_stamp_s > self.config.request_max_age_s + 1e-12:
            return SelectionResult(False, 'request_stale')
        return SelectionResult(True, 'ok', self._state)

    def submit_message(self, message, *, now_s, anchor=None, progress_global_s=None):
        reason = self._observe_clock(now_s)
        if reason:
            return SelectionResult(False, reason)
        try:
            stamp = PathRequest.message_stamp_s(message)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._blocked = 'invalid_message'
            return SelectionResult(False, self._blocked)
        reason = self._time_error(stamp, now_s)
        if reason:
            if reason == 'invalid_request_stamp':
                self._blocked = reason
            return SelectionResult(False, reason)
        try:
            request = PathRequest.from_message(message)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._last_request = stamp
            self._blocked = 'invalid_message'
            return SelectionResult(False, self._blocked)
        return self.submit(request, now_s=now_s, anchor=anchor, progress_global_s=progress_global_s)

    def _time_error(self, stamp, now_s):
        if not _finite(stamp):
            return 'invalid_request_stamp'
        if stamp <= self._last_request:
            return 'request_not_newer'
        age = now_s - stamp
        if age < -self.config.future_tolerance_s - 1e-12:
            return 'request_from_future'
        if age > self.config.request_max_age_s + 1e-12:
            return 'request_stale'
        return None

    def submit(self, request: PathRequest, *, now_s, anchor=None, progress_global_s=None):
        reason = self._observe_clock(now_s) or self._time_error(request.stamp_s, now_s)
        if reason:
            if reason == 'invalid_request_stamp':
                self._blocked = reason
            return SelectionResult(False, reason)
        # Fresh malformed requests are observed but must not revive old output.
        self._last_request = request.stamp_s
        try:
            self._validate(request)
            chosen, pending_id = self._choose(request, anchor, progress_global_s)
        except (TypeError, ValueError, OverflowError) as exc:
            self._blocked = str(exc) or 'invalid_request'
            return SelectionResult(False, self._blocked)
        if pending_id is not None:
            self._registry[pending_id] = chosen.path
        self._state = chosen
        self._blocked = None
        return SelectionResult(True, 'ok', chosen)

    def _validate(self, r):
        if r.frame_id != self.base.path.frame_id:
            raise ValueError('frame_mismatch')
        if (isinstance(r.v_max_mps, bool)
                or not (_finite(r.v_max_mps) or r.v_max_mps == math.inf) or r.v_max_mps < 0
                or r.stop_global_s is not None and not _finite(r.stop_global_s)):
            raise ValueError('invalid_longitudinal_metadata')
        if (not _finite(r.lat_offset_m, r.entry_length_m, r.return_length_m)
                or r.entry_length_m < 0 or r.return_length_m < 0
                or r.entry_length_m > 0 and not _finite(r.entry_s)
                or r.return_length_m > 0 and not _finite(r.return_s)
                or not isinstance(r.reference_id, str) or not isinstance(r.reference_release, bool)):
            raise ValueError('invalid_path_fields')
        if (not len(r.chain_offset) == len(r.chain_start_s) == len(r.chain_length_m)
                or not _finite(*r.chain_offset, *r.chain_start_s, *r.chain_length_m)
                or any(value <= 0 for value in r.chain_length_m)):
            raise ValueError('invalid_chain')

    def _state_for(self, r, path, mode, key, *, moves=(), anchor=None, legacy_spec=None):
        old = self._state
        same = old is not None and old.mode == mode and old.geometry_key == key
        revision = old.revision if same else (old.revision + 1 if old else 1)
        return SelectedPath(old.path if same else path, mode, revision, r.stamp_s,
                            r.v_max_mps, r.stop_global_s, key, moves, anchor, legacy_spec)

    def _hold(self, r):
        old = self._state
        return self._state_for(r, old.path, old.mode, old.geometry_key,
                               moves=old.moves, anchor=old.anchor, legacy_spec=old.legacy_spec), None

    def _choose(self, r, anchor, progress_global_s):
        old, cfg = self._state, self.config
        payload = bool(r.reference_id or r.reference_x or r.reference_y or r.reference_global_s)
        offset_request = bool(r.chain_offset or r.lat_offset_m or r.entry_length_m or r.return_length_m or anchor)
        if payload:
            if r.reference_release or offset_request:
                raise ValueError('reference_mode_conflict')
            if r.reference_id == self.base.path.path_id or r.reference_id.startswith(_RESERVED):
                raise ValueError('reserved_reference_id')
            if len(r.reference_x) > cfg.max_points:
                raise ValueError('reference_point_limit')
            candidate = StationedPath.from_reference(r.reference_id, r.frame_id,
                            r.reference_x, r.reference_y, r.reference_global_s)
            if any(b - a > cfg.max_reference_step_m + 1e-12
                   for a, b in zip(candidate.path.stations_m, candidate.path.stations_m[1:])):
                raise ValueError('reference_step_limit')
            known = self._registry.get(r.reference_id)
            if known is not None:
                if known.path.points != candidate.path.points or known.global_stations_m != candidate.global_stations_m:
                    raise ValueError('reference_id_mutation')
                candidate = known
            elif len(self._registry) >= cfg.max_reference_ids:
                raise ValueError('reference_registry_full')
            return self._state_for(r, candidate, 'reference', ('reference', r.reference_id)), (None if known else r.reference_id)
        if old is not None and old.mode == 'reference' and not r.reference_release:
            if offset_request:
                raise ValueError('reference_release_required')
            return self._hold(r)

        legacy_spec = None
        if r.chain_offset:
            moves = moves_from_chain(r.chain_offset, r.chain_start_s, r.chain_length_m)
            initial = 0.0
            anchor = None
        elif r.entry_length_m:
            moves = entry_return_moves(r.lat_offset_m, entry_start_s=r.entry_s,
                        entry_length_m=r.entry_length_m,
                        return_start_s=r.return_s if r.return_length_m else None,
                        return_length_m=r.return_length_m if r.return_length_m else None)
            initial = 0.0
            anchor = None
        elif offset_request:
            legacy_spec = (r.lat_offset_m, r.return_s if r.return_length_m else None, r.return_length_m)
            if old is not None and old.mode == 'offset' and old.legacy_spec == legacy_spec:
                if anchor is not None and anchor != old.anchor:
                    raise ValueError('anchor_changed_for_active_offset')
                return self._hold(r)
            if anchor is None:
                raise ValueError('transition_anchor_required')
            initial = anchor.initial_offset_m
            moves = entry_return_moves(r.lat_offset_m, entry_start_s=anchor.start_global_s,
                        entry_length_m=anchor.length_m,
                        return_start_s=r.return_s if r.return_length_m else None,
                        return_length_m=r.return_length_m if r.return_length_m else None)
        else:
            if old is not None and old.mode == 'offset':
                if progress_global_s is None:
                    return self._hold(r)
                if (not _finite(progress_global_s)
                        or not self.base.global_stations_m[0] <= progress_global_s <= self.base.global_stations_m[-1]
                        or progress_global_s < old.moves[-1].end_global_s
                        or old.moves[-1].target_offset_m != 0.0):
                    raise ValueError('offset_not_finished')
            return self._state_for(r, self.base, 'global', ('global',)), None

        key = ('offset', moves, initial)
        if old is not None and old.mode == 'offset' and old.geometry_key == key:
            return self._hold(r)
        revision = old.revision + 1 if old else 1
        candidate = build_offset_path(self.base, moves, path_id=_RESERVED + str(revision),
                    initial_offset_m=initial, base_step_m=cfg.base_step_m,
                    max_offset_m=cfg.max_offset_m, max_points=cfg.max_points)
        return self._state_for(r, candidate, 'offset', key, moves=moves,
                               anchor=anchor, legacy_spec=legacy_spec), None
