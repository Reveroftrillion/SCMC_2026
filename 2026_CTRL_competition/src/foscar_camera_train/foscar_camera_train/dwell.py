"""ROS-free timed stops for data collection, using the stop_s front-wheel convention."""

from dataclasses import dataclass
import math


@dataclass
class Decision:
    state: str
    stop_s: float = math.nan
    v_max: float = math.inf
    remaining_s: float = 0.
    line_s: float = math.nan


class CollectionStops:
    def __init__(self, lines, stop_seconds=60., front_axle_m=3., stop_before_m=0.,
                 reach_m=.5, stopped_speed_mps=.1, max_tick_gap_s=.5):
        values = (stop_seconds, front_axle_m, stop_before_m, reach_m,
                  stopped_speed_mps, max_tick_gap_s)
        if not all(math.isfinite(v) and v >= 0 for v in values):
            raise ValueError('정차 시간·거리·속도 임계값은 유한한 0 이상의 값이어야 합니다.')
        if stop_seconds <= 0 or max_tick_gap_s <= 0:
            raise ValueError('정차 시간과 입력 유효 시간은 0보다 커야 합니다.')
        self.lines = sorted(set(float(s) for s in lines))
        if not self.lines or not all(math.isfinite(s) for s in self.lines):
            raise ValueError('수집 경로에 유효한 정지선이 없습니다.')
        self.duration, self.front, self.offset = stop_seconds, front_axle_m, stop_before_m
        self.reach, self.stopped_speed, self.max_gap = reach_m, stopped_speed_mps, max_tick_gap_s
        self.index = None
        self.since = self.last_tick = None

    def step(self, now, s, speed, healthy=True):
        if not healthy or not all(math.isfinite(v) for v in (now, s, speed)):
            self.since = self.last_tick = None
            return Decision('COLLECT_WAITING', v_max=0., remaining_s=self.duration)
        if self.last_tick is not None and not 0 <= now - self.last_tick <= self.max_gap:
            self.since = None
        self.last_tick = now
        front_s = s + self.front
        if self.index is None:
            # A run may start beyond earlier lines or on the negative-s spawn connector.
            self.index = next((i for i, line in enumerate(self.lines)
                               if line - self.offset >= front_s - self.reach), len(self.lines))
        if self.index >= len(self.lines):
            return Decision('COLLECT_CRUISE')

        line = self.lines[self.index]
        target = line - self.offset
        distance = target - front_s
        if distance < -self.reach:
            self.since = None
            # The tracker ignores stop_s behind its front axle; v_max=0 still stops it.
            return Decision('COLLECT_MISSED_STOP', target, 0., self.duration, line)
        if abs(distance) <= self.reach and abs(speed) <= self.stopped_speed:
            self.since = now if self.since is None else self.since
            remaining = max(0., self.duration - (now - self.since))
            if remaining <= 1e-9:
                self.index += 1
                self.since = None
                return Decision('COLLECT_RELEASE', line_s=line)
            return Decision('COLLECT_HOLD', target, 0., remaining, line)

        self.since = None
        # Publish even when far away; the existing tracker owns its braking ramp.
        return Decision('COLLECT_APPROACH', target, math.inf, self.duration, line)
