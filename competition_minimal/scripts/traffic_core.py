#!/usr/bin/env python3
"""Fail-closed traffic light confirmation and stopping-distance calculation."""
import math

LABELS = ('4red', '4redleft', '4yellow', '4redyellow', '4greenleft',
          '4green', '3red', '3redleft', '3redyellow')


class TrafficDecision:
    def __init__(self, intersections, positions_verified, front_offset=3.0,
                 buffer=1.0, deceleration=1.5, timeout=0.8):
        self.items = intersections
        self.verified = positions_verified is True
        self.front_offset = float(front_offset)
        self.buffer = float(buffer)
        self.deceleration = float(deceleration)
        self.timeout = float(timeout)
        self.active = None
        self.label = 'UNKNOWN'
        self.hits = 0
        self.last_seen = float('-inf')
        self.committed = set()
        try:
            if not self.items or not all(math.isfinite(x) and x > 0 for x in
                                         (self.front_offset, self.buffer, self.deceleration, self.timeout)):
                self.verified = False
            last = -1.0
            for item in self.items:
                begin, stop = float(item['start_s']), float(item['stop_s'])
                roi = item['roi']
                if not (math.isfinite(begin) and math.isfinite(stop) and 0 <= begin < stop and
                        begin > last and item['direction'] in ('straight', 'left') and
                        len(roi) == 4 and all(math.isfinite(float(x)) for x in roi) and
                        0 <= roi[0] < roi[2] <= 1 and 0 <= roi[1] < roi[3] <= 1):
                    self.verified = False
                last = stop
        except (ValueError, TypeError, KeyError):
            self.verified = False

    def current(self, s):
        if not self.verified or not math.isfinite(s):
            return None
        for i, item in enumerate(self.items):
            if float(item['start_s']) <= s <= float(item['stop_s']) + 5:
                return i
        return None

    def observe(self, s, labels, now):
        idx = self.current(s)
        if idx != self.active:
            self.active, self.hits, self.label = idx, 0, 'UNKNOWN'
            self.last_seen = float('-inf')
        if idx is None:
            return
        # More than one signal is ambiguous. A restrictive or contradictory
        # observation immediately cancels a previously confirmed GO.
        labels = list(labels)
        label = labels[0] if len(labels) == 1 and labels[0] in LABELS else 'UNKNOWN'
        if label == self.label and now > self.last_seen and now-self.last_seen <= self.timeout:
            self.hits += 1
        else:
            self.hits = 1 if label != 'UNKNOWN' else 0
        self.label, self.last_seen = label, now

    def decision(self, s, now):
        """Returns (stop, max_speed_mps, reason). Inf means no speed restriction."""
        if not self.verified or not math.isfinite(s):
            return True, 0.0, 'INVALID_MISSION_POSITIONS'
        for item in self.items:
            if item.get('verified', True) is not True:
                remaining = float(item['start_s']) - s - 5.0
                if remaining <= 0.0:
                    return True, 0.0, 'UNVERIFIED_INTERSECTION'
        # An unconfirmed crossing must never become GO merely because the
        # vehicle has overshot the detector's active window.
        for i, item in enumerate(self.items):
            if s > float(item['stop_s']) + 5.0 and i not in self.committed:
                return True, 0.0, 'UNCONFIRMED_CROSSING'
        idx = self.current(s)
        if idx is None:
            return False, float('inf'), 'OUTSIDE_INTERSECTION'
        item = self.items[idx]
        if idx != self.active:
            self.active, self.label, self.hits, self.last_seen = idx, 'UNKNOWN', 0, float('-inf')
        allowed = {'4green', '4greenleft'} if item['direction'] == 'straight' else {'4greenleft'}
        go = self.label in allowed and self.hits >= 2 and 0 <= now-self.last_seen <= self.timeout
        front_s = s + self.front_offset
        if go and front_s >= float(item['stop_s']):
            self.committed.add(idx)  # Already entered on a confirmed green.
        if idx in self.committed:
            return False, float('inf'), 'COMMITTED'
        if go:
            return False, float('inf'), 'GO ' + self.label
        distance = float(item['stop_s']) - front_s - self.buffer
        if distance <= 0:
            return True, 0.0, 'STOP ' + self.label
        return False, math.sqrt(2 * self.deceleration * distance), 'BRAKE ' + self.label
