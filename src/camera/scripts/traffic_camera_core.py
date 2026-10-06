"""Traffic-light association, bounded signal filtering and camera mission state."""
from collections import deque
from dataclasses import dataclass
import math

TRAFFIC_CLASSES = {'4red', '4redleft', '4yellow', '4redyellow', '4greenleft', '4green',
                   '3red', '3redleft', '3redyellow'}
GO_CLASSES = {'4greenleft', '4green'}


@dataclass(frozen=True)
class Observation:
    label: str
    confidence: float
    bbox: tuple  # Normalized coordinates in the full source image.

    def valid(self):
        return (self.label in TRAFFIC_CLASSES and math.isfinite(self.confidence) and
                0 < self.confidence <= 1 and len(self.bbox) == 4 and
                all(math.isfinite(v) for v in self.bbox) and
                0 <= self.bbox[0] < self.bbox[2] <= 1 and
                0 <= self.bbox[1] < self.bbox[3] <= 1)


@dataclass
class Signal:
    raw_label: str = 'UNKNOWN'
    label: str = 'UNKNOWN'
    confidence: float = 0.
    track_id: int = 0
    observed: bool = False
    held: bool = False
    bbox: tuple = None
    raw_bbox: tuple = None
    detection_age: float = float('inf')
    signal_age: float = float('inf')
    state: str = 'NO_TARGET'

    @property
    def valid(self): return self.label != 'UNKNOWN'

    @property
    def width(self): return self.bbox[2]-self.bbox[0] if self.bbox else float('nan')

    @property
    def height(self): return self.bbox[3]-self.bbox[1] if self.bbox else float('nan')


def iou(a, b):
    intersection = max(0., min(a[2], b[2])-max(a[0], b[0])) * max(0., min(a[3], b[3])-max(a[1], b[1]))
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1])-intersection
    return intersection/union


class SignalTracker:
    def __init__(self, window=5, confirm_hits=3, hold_time=.35, track_timeout=1.,
                 vote_age=.5, bbox_alpha=.35, min_iou=.1, max_center_distance=.08,
                 max_scale_ratio=2.5, target_x=.5):
        self.window, self.confirm_hits = int(window), int(confirm_hits)
        self.hold_time, self.track_timeout, self.vote_age = float(hold_time), float(track_timeout), float(vote_age)
        self.bbox_alpha, self.min_iou = float(bbox_alpha), float(min_iou)
        self.max_center_distance, self.max_scale_ratio = float(max_center_distance), float(max_scale_ratio)
        self.target_x = float(target_x)
        values = (self.hold_time, self.track_timeout, self.vote_age, self.bbox_alpha,
                  self.min_iou, self.max_center_distance, self.max_scale_ratio, self.target_x)
        if (not all(math.isfinite(v) for v in values) or not 1 <= self.confirm_hits <= self.window <= 30 or
                not 0 < self.hold_time < self.track_timeout or not 0 < self.vote_age or
                not 0 < self.bbox_alpha <= 1 or not 0 <= self.min_iou <= 1 or
                not 0 < self.max_center_distance < .5 or self.max_scale_ratio <= 1 or not 0 <= self.target_x <= 1):
            raise ValueError('Invalid traffic signal filter settings')
        self.next_id = 0
        self.clear()

    def clear(self):
        self.track_id = 0
        self.raw_box = self.box = None
        self.last_detection = self.last_signal = float('-inf')
        self.last_update = None
        self.confidence = 0.
        self.stable_label = 'UNKNOWN'
        self.votes = deque(maxlen=self.window)
        self.image_fault = False

    def fault(self, now):
        # Retain association geometry, but invalidate colour and size on camera failure.
        self.stable_label = 'UNKNOWN'
        self.votes.clear()
        self.last_signal = float('-inf')
        self.last_update = now
        self.image_fault = True
        return self.snapshot(now)

    def snapshot(self, now, observation=None):
        detection_age, signal_age = now-self.last_detection, now-self.last_signal
        bbox_valid = self.box is not None and detection_age <= self.hold_time and not self.image_fault
        valid = self.stable_label != 'UNKNOWN' and signal_age <= self.hold_time and not self.image_fault
        observed = observation is not None
        state = ('STALE_IMAGE' if self.image_fault else
                 'OBSERVED' if observed and valid else 'CONFIRMING' if observed else
                 'HELD' if bbox_valid and valid else 'MISSING' if self.track_id else 'NO_TARGET')
        return Signal(raw_label=observation.label if observed else 'UNKNOWN',
                      label=self.stable_label if valid else 'UNKNOWN', confidence=self.confidence,
                      track_id=self.track_id, observed=observed, held=bbox_valid and not observed,
                      bbox=self.box if bbox_valid else None, raw_bbox=observation.bbox if observed else None,
                      detection_age=detection_age, signal_age=signal_age, state=state)

    def match(self, observations):
        matches = []
        for observation in observations:
            box, previous = observation.bbox, self.raw_box
            distance = math.hypot((box[0]+box[2]-previous[0]-previous[2])/2.,
                                  (box[1]+box[3]-previous[1]-previous[3])/2.)
            ratios = [(box[2]-box[0])/(previous[2]-previous[0]),
                      (box[3]-box[1])/(previous[3]-previous[1])]
            overlap = iou(box, previous)
            if (all(1/self.max_scale_ratio <= ratio <= self.max_scale_ratio for ratio in ratios) and
                    (overlap >= self.min_iou or distance <= self.max_center_distance)):
                matches.append((overlap-2*distance+.02*observation.confidence, observation))
        return max(matches, key=lambda item: item[0])[1] if matches else None

    def update(self, observations, now, locked=False):
        if self.last_update is not None and now <= self.last_update:
            return self.fault(now)
        self.last_update = now
        self.image_fault = False
        observations = [observation for observation in observations if observation.valid()]
        if self.raw_box is not None and now-self.last_detection > self.track_timeout and not locked:
            self.clear()
            self.last_update = now
        if now-self.last_detection > self.hold_time:
            self.stable_label = 'UNKNOWN'
            self.votes.clear()
        observation = self.match(observations) if self.raw_box is not None else None
        if self.raw_box is None and observations and not locked:
            observation = max(observations, key=lambda item:
                              item.confidence-.5*abs((item.bbox[0]+item.bbox[2])/2.-self.target_x))
            self.next_id += 1
            self.track_id = self.next_id
            self.box = observation.bbox
            self.stable_label = 'UNKNOWN'
            self.votes.clear()
        while self.votes and now-self.votes[0][0] > self.vote_age:
            self.votes.popleft()
        if observation is None:
            self.votes.append((now, None))
            return self.snapshot(now)
        self.raw_box = observation.bbox
        self.box = tuple(self.bbox_alpha*v+(1-self.bbox_alpha)*old
                         for v, old in zip(observation.bbox, self.box))
        self.last_detection, self.confidence = now, observation.confidence
        # A restrictive observation cancels a previous GO immediately.
        if self.stable_label in GO_CLASSES and observation.label not in GO_CLASSES:
            self.votes.clear()
            self.stable_label, self.last_signal = observation.label, now
        self.votes.append((now, observation.label))
        if sum(label == observation.label for _, label in self.votes) >= self.confirm_hits:
            self.stable_label = observation.label
        if self.stable_label == observation.label:
            self.last_signal = now
        return self.snapshot(now, observation)


@dataclass
class Mission:
    active: bool = False
    phase: str = 'IDLE'
    reason: str = 'WAITING_FOR_SIGNAL'
    mission_id: int = 0
    target_track_id: int = 0
    entry_width: float = float('nan')
    peak_width: float = float('nan')
    exit_armed: bool = False
    reset_track: bool = False


class CameraMission:
    def __init__(self, enabled=True, enter_width_ratio=.04, cancel_width_ratio=.032,
                 enter_confirm_frames=3, enter_confirm_time=.25, max_observation_gap=.35,
                 exit_mode='camera_edge', exit_edges=('top', 'left', 'right'), exit_edge_ratio=.04,
                 exit_min_growth=1.4, exit_min_peak_width=.06, exit_confirm_frames=3,
                 exit_confirm_time=.15, exit_loss_time=1., cooldown_time=3.,
                 rearm_width_ratio=.02, rearm_clear_time=1.):
        self.enabled = bool(enabled)
        self.enter_width, self.cancel_width, self.rearm_width = float(enter_width_ratio), float(cancel_width_ratio), float(rearm_width_ratio)
        self.enter_hits, self.exit_hits = int(enter_confirm_frames), int(exit_confirm_frames)
        self.enter_time, self.max_gap = float(enter_confirm_time), float(max_observation_gap)
        self.exit_mode, self.exit_edges = exit_mode, set(exit_edges)
        self.exit_edge, self.exit_growth, self.exit_peak = float(exit_edge_ratio), float(exit_min_growth), float(exit_min_peak_width)
        self.exit_time, self.exit_loss = float(exit_confirm_time), float(exit_loss_time)
        self.cooldown, self.rearm_time = float(cooldown_time), float(rearm_clear_time)
        values = (self.enter_width, self.cancel_width, self.rearm_width, self.enter_time, self.max_gap,
                  self.exit_edge, self.exit_growth, self.exit_peak, self.exit_time, self.exit_loss,
                  self.cooldown, self.rearm_time)
        if (not all(math.isfinite(v) for v in values) or
                not 0 < self.rearm_width < self.cancel_width < self.enter_width < 1 or
                not 2 <= self.enter_hits <= 30 or not 2 <= self.exit_hits <= 30 or
                not all(v > 0 for v in (self.enter_time, self.max_gap, self.exit_time, self.cooldown, self.rearm_time)) or
                not 0 < self.exit_edge < .2 or self.exit_growth <= 1 or not 0 < self.exit_peak < 1 or
                self.exit_loss <= self.max_gap or self.exit_mode not in ('camera_edge', 'external') or
                not self.exit_edges or not self.exit_edges <= {'top', 'left', 'right'}):
            raise ValueError('Invalid camera traffic mission settings')
        self.mission_id = 0
        self.reset()

    @property
    def locked(self): return self.phase in ('ACTIVE', 'COOLDOWN')

    def reset(self):
        self.phase, self.target_id = 'IDLE', 0
        self.entry_size = self.peak_size = float('nan')
        self.entry_start = self.entry_last = self.last_seen = float('-inf')
        self.entry_count = self.edge_count = 0
        self.edge_start = self.edge_last = float('-inf')
        self.exit_armed = False
        self.ended = self.clear_start = None
        self.last_update = None

    def result(self, phase=None, reason='WAITING_FOR_SIGNAL', reset_track=False):
        return Mission(active=self.enabled and self.phase == 'ACTIVE',
                       phase=phase or self.phase if self.enabled else 'DISABLED', reason=reason,
                       mission_id=self.mission_id, target_track_id=self.target_id,
                       entry_width=self.entry_size, peak_width=self.peak_size,
                       exit_armed=self.exit_armed, reset_track=reset_track)

    def fault(self, now):
        self.last_update = now
        self.edge_count, self.exit_armed = 0, False
        self.clear_start = None
        if self.phase == 'ENTERING': self.reset()
        return self.result('ACTIVE_STALE' if self.phase == 'ACTIVE' else None, 'CAMERA_NOT_FRESH')

    def complete(self, now, reason='PASS_CONFIRMED'):
        if self.phase != 'ACTIVE': return self.result(reason='IGNORED_PASS_CONFIRMATION')
        self.phase, self.ended, self.clear_start = 'COOLDOWN', now, None
        self.exit_armed = False
        return self.result(reason=reason)

    def update(self, signal, now):
        if self.last_update is not None and now <= self.last_update:
            return self.fault(now)
        self.last_update = now
        if not self.enabled: return self.result(reason='CAMERA_MISSION_DISABLED')
        if signal.state == 'STALE_IMAGE': return self.fault(now)
        if self.phase in ('IDLE', 'ENTERING'):
            if signal.observed and signal.bbox and signal.valid and signal.width >= self.enter_width:
                if signal.track_id != self.target_id or now-self.entry_last > self.max_gap:
                    self.phase, self.target_id = 'ENTERING', signal.track_id
                    self.entry_start, self.entry_count = now, 0
                    self.entry_size = self.peak_size = signal.width
                self.entry_count += 1
                self.entry_last = now
                self.peak_size = max(self.peak_size, signal.width)
                if self.entry_count >= self.enter_hits and now-self.entry_start >= self.enter_time:
                    self.phase = 'ACTIVE'
                    self.mission_id += 1
                    self.last_seen = now
                    return self.result(reason='SIZE_CONFIRMED')
                return self.result(reason='CONFIRMING_SIZE')
            if (self.phase == 'ENTERING' and
                    (now-self.entry_last > self.max_gap or
                     (signal.observed and (signal.track_id != self.target_id or signal.width < self.cancel_width)))):
                self.reset()
            return self.result(reason='WAITING_FOR_SIZE')
        if self.phase == 'COOLDOWN':
            clear = not signal.observed or (signal.bbox and signal.width < self.rearm_width)
            if clear:
                if self.clear_start is None: self.clear_start = now
            else: self.clear_start = None
            if (now-self.ended >= self.cooldown and self.clear_start is not None and
                    now-self.clear_start >= self.rearm_time):
                self.reset()
                return self.result(reason='READY_FOR_NEXT_SIGNAL', reset_track=True)
            return self.result(reason='WAITING_TO_REARM')
        if signal.observed and signal.track_id == self.target_id and signal.bbox:
            self.last_seen = now
            self.peak_size = max(self.peak_size, signal.width)
            box = signal.raw_bbox or signal.bbox
            edge = (('top' in self.exit_edges and box[1] <= self.exit_edge) or
                    ('left' in self.exit_edges and box[0] <= self.exit_edge) or
                    ('right' in self.exit_edges and box[2] >= 1-self.exit_edge))
            grown = self.peak_size >= max(self.exit_peak, self.entry_size*self.exit_growth)
            if self.exit_mode == 'camera_edge' and edge and grown:
                if now-self.edge_last > self.max_gap:
                    self.edge_start, self.edge_count = now, 0
                self.edge_count += 1
                self.edge_last = now
                self.exit_armed = self.edge_count >= self.exit_hits and now-self.edge_start >= self.exit_time
            else:
                self.edge_count, self.exit_armed = 0, False
            return self.result('EXIT_PENDING' if self.exit_armed else None, 'TARGET_OBSERVED')
        if self.exit_mode == 'camera_edge' and self.exit_armed and now-self.last_seen >= self.exit_loss:
            return self.complete(now, 'CAMERA_EDGE_EXIT')
        return self.result('EXIT_PENDING' if self.exit_armed else 'ACTIVE_LOST', 'WAITING_FOR_SAME_TARGET')
