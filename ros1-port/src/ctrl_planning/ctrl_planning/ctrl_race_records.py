"""Lap and mission stopwatch with persistent best records (no ROS).

A mission time counts only when the car drove in through its start boundary and out through
its end (mission transition 'checkpoint handoff' both ways), so practice scenarios that spawn
inside a mission never set a record. The lap starts on the first motion near the route start
and ends at the finish station.
"""
import json
import math
from datetime import datetime
from pathlib import Path

IGNORED = {'CRUISE', 'LOCALIZATION_INVALID'}


class RaceRecords:
    def __init__(self, path, finish_s, start_window_s=30., start_motion_m=.3, code=''):
        self.path = Path(path).expanduser()
        self.code = code               # branch@commit that set a record (README table)
        self.finish_s, self.start_window_s, self.start_motion_m = finish_s, start_window_s, start_motion_m
        self.best = {'lap': None, 'missions': {}}
        try:
            data = json.loads(self.path.read_text())
            if isinstance(data, dict):
                self.best['lap'] = data.get('lap')
                self.best['missions'] = dict(data.get('missions') or {})
        except (OSError, ValueError):
            pass
        self.mission = None
        self.reset_lap()

    def reset_lap(self):
        self.last_s = None
        self.rest_s = None
        self.lap_start = None
        self.lap_end = None
        self.lap_valid = False
        self.mission_start = None     # the mission name stays: only a real handoff restarts a clock
        self.last = {}
        self.new_best = set()

    # ------------------------------------------------------------------ input
    def on_s(self, s, now):
        if not math.isfinite(s):
            return
        if self.lap_start is not None and self.last_s is not None and s < self.last_s - 20.:
            self.reset_lap()          # relocated back (scenario reload): a new lap may start
        self.last_s = s
        if self.lap_start is None:
            if s > self.start_window_s:
                self.rest_s = None
                return
            if self.rest_s is None or s < self.rest_s:
                self.rest_s = s
            if s - self.rest_s >= self.start_motion_m:
                self.lap_start, self.lap_valid = now, True
            return
        if self.lap_end is None and self.lap_valid and s >= self.finish_s:
            self.lap_end = now
            self.offer('lap', now - self.lap_start)

    def on_mission(self, name, transition, now):
        if name == self.mission:
            return
        if transition != 'checkpoint handoff':
            # localization reset / relocation: nothing in progress is comparable any more
            self.mission_start = None
            if transition in ('localization reset', 'invalid localization'):
                self.lap_valid = False
            if self.lap_start is not None and transition == 'initial localization':
                self.reset_lap()
        elif self.mission not in IGNORED and self.mission is not None and self.mission_start is not None:
            elapsed = now - self.mission_start
            self.last[self.mission] = elapsed
            self.offer(self.mission, elapsed)
        self.mission = name
        self.mission_start = now if transition == 'checkpoint handoff' and name not in IGNORED else None

    # ------------------------------------------------------------------ records
    def offer(self, key, seconds):
        if not math.isfinite(seconds) or seconds <= 0:
            return False
        table = self.best['missions'] if key != 'lap' else None
        old = self.best['lap'] if key == 'lap' else table.get(key)
        if old is not None and old.get('s', math.inf) <= seconds:
            return False
        row = {'s': round(seconds, 2), 'at': datetime.now().isoformat(timespec='seconds'), 'code': self.code}
        if key == 'lap':
            self.best['lap'] = row
        else:
            table[key] = row
        self.new_best.add(key)
        self.save()
        return True

    def save(self):
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix('.tmp')
            tmp.write_text(json.dumps(self.best, indent=2, ensure_ascii=False))
            tmp.replace(self.path)
        except OSError:
            pass

    # ------------------------------------------------------------------ display
    def lap_elapsed(self, now):
        if self.lap_start is None:
            return None
        return (self.lap_end if self.lap_end is not None else now) - self.lap_start

    def mission_elapsed(self, now):
        return None if self.mission_start is None else now - self.mission_start

    def best_s(self, key):
        row = self.best['lap'] if key == 'lap' else self.best['missions'].get(key)
        return None if row is None else row.get('s')
