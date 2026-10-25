"""Opt-in comparison/replay. This module cannot publish vehicle commands."""
import json
import math
import queue
import threading
import time
from dataclasses import asdict, dataclass

from .ctrl_lateral import LateralConfig, Polyline, VehiclePose, compute_steering, project_position
from .ctrl_path_conversion import StationedPath, offset_at
from .ctrl_path_selection import PathRequest, PathSelector, SelectionConfig, TransitionAnchor
from .ctrl_progress import ProgressConfig, ProgressTracker


def encode(value):
    if isinstance(value, float) and not math.isfinite(value):
        return {'_float': 'nan' if math.isnan(value) else 'inf' if value > 0 else '-inf'}
    if isinstance(value, dict):
        return {key: encode(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [encode(item) for item in value]
    return value


def decode(value):
    if isinstance(value, dict):
        if set(value) == {'_float'}:
            return float(value['_float'])
        return {key: decode(item) for key, item in value.items()}
    if isinstance(value, list):
        return [decode(item) for item in value]
    return value


@dataclass(frozen=True)
class ShadowSettings:
    lateral: LateralConfig
    progress: ProgressConfig
    selection: SelectionConfig
    seed_half_window_m: float
    initial_step_s: float
    legacy_transition_time_s: float
    legacy_return_time_s: float
    legacy_min_length_m: float

    def __post_init__(self):
        values = (self.seed_half_window_m, self.initial_step_s, self.legacy_transition_time_s,
                  self.legacy_return_time_s, self.legacy_min_length_m)
        if (not all(math.isfinite(v) and v > 0 and not isinstance(v, bool) for v in values)
                or self.initial_step_s > self.lateral.max_step_s):
            raise ValueError('invalid shadow settings')

    @classmethod
    def from_dict(cls, data):
        data = dict(data)
        for key, kind in [('lateral', LateralConfig), ('progress', ProgressConfig), ('selection', SelectionConfig)]:
            data[key] = kind(**data[key])
        return cls(**data)


@dataclass(frozen=True)
class ShadowFrame:
    now_s: float
    pose: VehiclePose
    global_hint_s: float
    old_angle_rad: float
    request: object
    request_stamp_s: float
    comparable: bool = True
    input_error: str = ''

    @classmethod
    def from_dict(cls, data):
        data = decode(data)
        data['pose'] = VehiclePose(**data['pose'])
        if data['request'] is not None:
            data['request'] = PathRequest(**data['request'])
        return cls(**data)


def request_from_target(target, stamp_s):
    """Copy accepted follower fields without importing its legacy classes."""
    if target is None:
        return None
    ref = target.reference
    return PathRequest(stamp_s, 'map', target.v_max, None if math.isnan(target.stop_s) else target.stop_s,
        target.lat_offset, target.entry_s, target.entry_length_m, target.return_s, target.return_length_m,
        tuple(row[0] for row in target.chain), tuple(row[1] for row in target.chain), tuple(row[2] for row in target.chain),
        ref.id if ref is not None else '', tuple(float(p[0]) for p in ref.xy) if ref is not None else (),
        tuple(float(p[1]) for p in ref.xy) if ref is not None else (),
        tuple(float(s) for s in ref.global_s) if ref is not None else (), target.reference_release)


class ShadowEngine:
    def __init__(self, base, settings):
        self.base, self.settings = base, settings
        self.selector, self.tracker = PathSelector(base, settings.selection), ProgressTracker(settings.progress)
        self._seen, self._fault = -math.inf, ''
        self._revision = self._angle = self._computed_at = None

    def header(self):
        return dict(schema_version=1, type='header', settings=asdict(self.settings),
                    base=dict(path_id=self.base.path.path_id, frame_id=self.base.path.frame_id,
                              points=self.base.path.points, global_stations_m=self.base.global_stations_m))

    @classmethod
    def from_header(cls, header):
        header = decode(header)
        if header.get('schema_version') != 1 or header.get('type') != 'header':
            raise ValueError('unsupported shadow header')
        base = header['base']
        return cls(StationedPath(Polyline(base['path_id'], base['frame_id'], base['points']), base['global_stations_m']),
                   ShadowSettings.from_dict(header['settings']))

    def _window(self, route, hint):
        if not math.isfinite(hint) or not route.global_stations_m[0] <= hint <= route.global_stations_m[-1]:
            raise ValueError('global hint outside selected path')
        radius = self.settings.seed_half_window_m
        return route.local_window(max(route.global_stations_m[0], hint - radius),
                                  min(route.global_stations_m[-1], hint + radius))

    def _anchor(self, r, frame):
        if r.reference_id or r.reference_x or r.reference_release or r.chain_offset or r.entry_length_m:
            return None
        old = self.selector.state
        spec = (r.lat_offset_m, r.return_s if r.return_length_m else None, r.return_length_m)
        if old is not None and old.mode == 'reference':
            return None
        if old is not None and old.legacy_spec == spec:
            if r.lat_offset_m == 0 and not r.return_length_m:
                return old.anchor if frame.global_hint_s < old.moves[-1].end_global_s else None
            return None
        if r.lat_offset_m == 0 and r.return_length_m == 0:
            if old is None or old.mode != 'offset':
                return None
            if frame.global_hint_s >= old.moves[-1].end_global_s and old.moves[-1].target_offset_m == 0:
                return None
        if old is not None and old.mode == 'offset':
            initial = offset_at(frame.global_hint_s, old.moves, old.anchor.initial_offset_m if old.anchor else 0.0)
        else:
            lo, hi = self._window(self.base, frame.global_hint_s)
            projection = project_position(self.base.path, frame.pose, lo, hi)
            if not projection.valid:
                raise ValueError(projection.reason)
            initial = projection.cross_track_m
        duration = self.settings.legacy_return_time_s if r.lat_offset_m == 0 else self.settings.legacy_transition_time_s
        return TransitionAnchor(frame.global_hint_s, max(self.settings.legacy_min_length_m,
                                abs(frame.pose.speed_mps) * duration), initial)

    def evaluate(self, frame):
        out = dict(valid=False, reason='', old_angle_rad=frame.old_angle_rad, new_angle_rad=None,
                   delta_angle_rad=None, mode=None, revision=None)
        try:
            if frame.input_error:
                self._fault = frame.input_error
                if math.isfinite(frame.request_stamp_s):
                    self._seen = max(self._seen, frame.request_stamp_s)
            elif frame.request is not None and not math.isfinite(frame.request.stamp_s):
                raise ValueError('nonfinite request timestamp')
            elif frame.request is not None and frame.request.stamp_s > self._seen:
                self._seen = frame.request.stamp_s
                selected = self.selector.submit(frame.request, now_s=frame.now_s,
                    anchor=self._anchor(frame.request, frame), progress_global_s=frame.global_hint_s)
                self._fault = '' if selected.valid else selected.reason
            selected = self.selector.current(now_s=frame.now_s)
            if self._fault or not selected.valid:
                out['reason'] = self._fault or selected.reason
                return out
            state = selected.state
            out.update(mode=state.mode, revision=state.revision)
            if not frame.comparable:
                out['reason'] = 'production_not_comparable'
                return out
            if not math.isfinite(frame.old_angle_rad):
                raise ValueError('nonfinite production steering')
            dt = self.settings.initial_step_s if self._computed_at is None else frame.now_s - self._computed_at
            if self._revision != state.revision:
                lo, hi = self._window(state.path, frame.global_hint_s)
                progress = self.tracker.acquire(state.path.path, frame.pose, now_s=frame.now_s,
                    reference_stamp_s=state.request_stamp_s, search_start_m=lo, search_end_m=hi)
            else:
                progress = self.tracker.update(frame.pose, now_s=frame.now_s, reference_stamp_s=state.request_stamp_s)
            if not progress.valid:
                out['reason'] = progress.reason
                return out
            self._revision = state.revision
            steering = compute_steering(frame.pose, progress.reference, self.settings.lateral,
                now_s=frame.now_s, dt_s=dt, previous_angle_rad=frame.old_angle_rad if self._angle is None else self._angle)
            if not steering.valid:
                out['reason'] = steering.reason
                return out
            self._angle, self._computed_at = steering.angle_rad, frame.now_s
            out.update(valid=True, reason='ok', new_angle_rad=self._angle,
                       delta_angle_rad=self._angle - frame.old_angle_rad,
                       global_progress_s=state.path.global_at_local(progress.state.station_m))
        except (TypeError, ValueError, OverflowError, AttributeError) as exc:
            self._fault = out['reason'] = type(exc).__name__ + ': ' + str(exc)
        return out


class ShadowRecorder:
    """One pending frame, processed by a worker; no blocking put in the follower."""
    def __init__(self, engine, path):
        self.engine = engine
        self._stream = path.open('x', encoding='utf-8')
        try:
            self._stream.write(json.dumps(encode(engine.header()), allow_nan=False) + '\n')
        except Exception:
            self._stream.close()
            raise
        self._queue, self._stop = queue.Queue(maxsize=1), threading.Event()
        self.dropped = self.written = 0
        self.error = None
        self._thread = threading.Thread(target=self._run, name='ctrl-shadow', daemon=True)
        self._thread.start()

    def push(self, frame):
        if self._stop.is_set() or self.error:
            return False
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            try:
                self._queue.get_nowait()
                self.dropped += 1
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(frame)
            except queue.Full:
                self.dropped += 1
                return False
        return True

    def _run(self):
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    frame = self._queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                started = time.perf_counter()
                result = self.engine.evaluate(frame)
                row = dict(type='frame', input=asdict(frame), output=result,
                           compute_s=time.perf_counter() - started, dropped=self.dropped)
                self._stream.write(json.dumps(encode(row), allow_nan=False) + '\n')
                self.written += 1
            self._stream.flush()
        except Exception as exc:
            self.error = type(exc).__name__ + ': ' + str(exc)
        finally:
            self._stream.close()

    def close(self, timeout_s=1.0):
        self._stop.set()
        self._thread.join(timeout_s)
        return not self._thread.is_alive() and self.error is None


def replay(path):
    with path.open(encoding='utf-8') as stream:
        engine = ShadowEngine.from_header(json.loads(next(stream)))
        count, mismatches = 0, []
        for count, line in enumerate(stream, 1):
            record = json.loads(line)
            if encode(engine.evaluate(ShadowFrame.from_dict(record['input']))) != record['output']:
                mismatches.append(count)
    return dict(frames=count, mismatches=mismatches, passed=count > 0 and not mismatches)
