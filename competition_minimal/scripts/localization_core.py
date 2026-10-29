"""Fail-closed planar dead reckoning and distance-based closed-course projection."""
import bisect
import math


class Estimator:
    def __init__(self, correction_rate=1.0, max_gps_residual=20.0, max_outage=90.0, blend_time=2.0):
        self.x = self.y = self.last_time = self.last_fix = None
        self.correction_rate = correction_rate
        self.max_gps_residual = max_gps_residual
        self.max_outage = max_outage
        if not all(math.isfinite(v) and v > 0 for v in (correction_rate, max_gps_residual, max_outage, blend_time)):
            raise ValueError('localization limits must be finite and positive')
        self.blend_time = blend_time
        self.target = None

    def fix(self, x, y, now):
        if not all(math.isfinite(v) for v in (x, y, now)):
            return False
        if self.x is None:
            self.x, self.y, self.last_time, self.last_fix = x, y, now, now
            return True
        # Reject implausible returns instead of snapping to an unrelated fix.
        if math.hypot(x-self.x, y-self.y) > self.max_gps_residual:
            return False
        self.target = (x-self.x, y-self.y)
        self.last_fix = now
        return True

    def recover(self, x, y, now, max_distance=2.0):
        """Resume after a timing/outage fault only with a new nearby GPS fix."""
        if (self.x is None or not all(math.isfinite(v) for v in (x, y, now))
                or math.hypot(x-self.x, y-self.y) > max_distance):
            return False
        self.last_time = self.last_fix = now
        self.target = (x-self.x, y-self.y)
        return True

    def advance(self, now, speed_mps, yaw):
        if self.x is None or not all(math.isfinite(v) for v in (now, speed_mps, yaw)):
            return False
        dt = now - self.last_time
        if dt < 0 or dt > 0.5 or now-self.last_fix > self.max_outage:
            return False
        self.x += speed_mps * math.cos(yaw) * dt
        self.y += speed_mps * math.sin(yaw) * dt
        if self.target:
            dx, dy = self.target
            length = math.hypot(dx, dy)
            fraction = min(1.0, dt/self.blend_time, self.correction_rate*dt/length) if length else 1.0
            self.x += fraction*dx
            self.y += fraction*dy
            self.target = (dx*(1-fraction), dy*(1-fraction))
        self.last_time = now
        return True


class CourseProgress:
    """Continuous segment projection; search stays local even at the closed seam."""
    def __init__(self, points, max_error=8.0, start_window=20.0):
        if len(points) < 2 or not all(math.isfinite(v) for p in points for v in p[:2]):
            raise ValueError('invalid path')
        self.points = [(float(p[0]), float(p[1])) for p in points]
        self.s = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.s.append(self.s[-1]+math.hypot(b[0]-a[0], b[1]-a[1]))
        if self.s[-1] <= start_window:
            raise ValueError('path too short')
        self.max_error = max_error
        self.start_window = start_window
        self.progress = None
        self.previous_xy = None
        self.error = None
        self.observed_distance = 0.0
        self.initial_progress = None

    @property
    def length(self):
        return self.s[-1]

    def update(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return False
        if self.progress is None:
            lower, upper = 0.0, self.start_window
        else:
            moved = math.hypot(x-self.previous_xy[0], y-self.previous_xy[1])
            # Only observed spatial motion permits progression; no clock-based index.
            lower = max(0.0, self.progress-3.0)
            upper = min(self.length, self.progress+max(2.0, moved+2.0),
                        self.initial_progress+1.5*(self.observed_distance+moved)+2.0)
        lo = max(0, bisect.bisect_right(self.s, lower)-1)
        hi = min(len(self.points)-1, bisect.bisect_right(self.s, upper))
        best = None
        for i in range(lo, hi):
            ax, ay = self.points[i]
            bx, by = self.points[i+1]
            vx, vy = bx-ax, by-ay
            seg2 = vx*vx+vy*vy
            if seg2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((x-ax)*vx+(y-ay)*vy)/seg2))
            candidate = self.s[i]+t*math.sqrt(seg2)
            if candidate < lower-1e-9 or candidate > upper+1e-9:
                continue
            error = math.hypot(x-(ax+t*vx), y-(ay+t*vy))
            if best is None or error < best[0]:
                best = (error, candidate)
        if best is None or best[0] > self.max_error:
            return False
        self.error = best[0]
        if self.progress is None:
            self.initial_progress = best[1]
        else:
            self.observed_distance += moved
        self.progress = best[1] if self.progress is None else max(self.progress, best[1])
        self.previous_xy = (x, y)
        return True
