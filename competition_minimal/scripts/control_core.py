"""Pure pursuit and bounded PI for path coordinates in metres and speed in m/s."""
import bisect
import math


def clamp(v, lo, hi): return max(lo, min(hi, v))


def valid_zones(zones, length):
    if not isinstance(zones, dict): return False
    for name in ('static', 'pedestrian', 'roundabout', 'highway', 'tunnel'):
        try:
            pair = zones[name]
            if len(pair) != 2 or not all(math.isfinite(float(v)) for v in pair): return False
            if not 0 <= float(pair[0]) < float(pair[1]) <= length: return False
        except (KeyError, TypeError, ValueError):
            return False
    return True


def uncertainty_limit(sections, progress, length, margin=5.0, deceleration=1.5):
    """Brake before the first unverified section; never drive past it."""
    if not isinstance(sections, list) or not math.isfinite(progress): return 0.0
    starts = []
    for section in sections:
        try:
            start = float(section['start_s'])
            if not math.isfinite(start) or not 0 <= start <= length or not section.get('reason'):
                return 0.0
            starts.append(start)
        except (KeyError, TypeError, ValueError):
            return 0.0
    if not starts: return float('inf')
    remaining = min(starts) - progress - margin
    return math.sqrt(2 * deceleration * remaining) if remaining > 0 else 0.0


class Driver:
    def __init__(self, points, max_speed=40/3.6, wheelbase=3.0, steering_deg=40.0):
        if len(points) < 3: raise ValueError('path needs at least 3 points')
        self.points = [(float(p[0]), float(p[1])) for p in points]
        self.s = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.s.append(self.s[-1]+math.hypot(b[0]-a[0], b[1]-a[1]))
        if self.s[-1] < 20 or not all(math.isfinite(n) for p in self.points for n in p):
            raise ValueError('invalid path')
        self.wheelbase, self.steering_max = wheelbase, math.radians(steering_deg)
        self.max_speed = min(max_speed, 60/3.6)
        self.integral, self.steer, self.previous_target = 0.0, 0.0, 0.0
        self.finished = False

    def point(self, s):
        s = clamp(s, 0.0, self.s[-1])
        i = min(max(0, bisect.bisect_right(self.s, s)-1), len(self.points)-2)
        length = self.s[i+1]-self.s[i]
        t = (s-self.s[i])/length if length > 1e-9 else 0.0
        a, b = self.points[i], self.points[i+1]
        yaw = math.atan2(b[1]-a[1], b[0]-a[0])
        return a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t, yaw

    def reset(self):
        self.integral, self.steer, self.previous_target = 0.0, 0.0, 0.0

    def update(self, x, y, yaw, progress, speed, offset, speed_limit, stop, dt):
        if not all(math.isfinite(v) for v in (x, y, yaw, progress, speed, offset, dt)) or dt <= 0 or math.isnan(speed_limit):
            self.reset(); return 0., 1., 0.
        if (progress >= self.s[-1]-1.5 and
                math.hypot(x-self.points[-1][0], y-self.points[-1][1]) <= 2.5):
            self.finished = True
        if stop or self.finished or speed_limit <= 0.01 or progress < 0 or progress > self.s[-1]+1 or abs(offset) > 4.0:
            self.reset(); return 0., 1., 0.
        px, py, path_yaw = self.point(progress)
        lateral_error = -math.sin(path_yaw)*(x-px)+math.cos(path_yaw)*(y-py)
        if math.hypot(x-px, y-py) > 7.0:
            self.reset(); return 0., 1., 0.
        lookahead = clamp(4.0 + max(0.0, speed)*0.8, 4.0, 14.0)
        tx, ty, target_yaw = self.point(progress+lookahead)
        tx -= math.sin(target_yaw)*offset
        ty += math.cos(target_yaw)*offset
        dx, dy = tx-x, ty-y
        local_y = -math.sin(yaw)*dx + math.cos(yaw)*dy
        distance2 = max(.25, dx*dx+dy*dy)
        steering_rad = math.atan2(2*self.wheelbase*local_y, distance2)
        # Error relative to the desired lateral path, with bounded correction.
        steering_rad -= clamp(.04*(lateral_error-offset), -.12, .12)
        normalized = clamp(steering_rad / self.steering_max, -1., 1.)
        self.steer += .25*(normalized-self.steer)
        _, _, future_yaw = self.point(progress+min(20., lookahead+6.))
        turn = abs(math.atan2(math.sin(future_yaw-path_yaw), math.cos(future_yaw-path_yaw)))
        curvature = turn/max(1., min(20., lookahead+6.))
        curve_speed = math.sqrt(1.6/max(.003, curvature))
        # The finish latch brakes immediately; begin slowing beforehand so the
        # vehicle is not asked to stop from road speed at the endpoint.
        remaining = max(0., self.s[-1] - 1.5 - progress)
        finish_speed = math.sqrt(2.0 * 1.5 * remaining)
        target = max(0., min(self.max_speed, curve_speed, speed_limit, finish_speed))
        if target <= .01:
            self.reset(); return 0., 1., 0.
        error = target-speed
        self.integral = clamp(self.integral+error*min(dt, .1), -3., 3.)
        ff = min(0.0, (target-self.previous_target)/max(dt, .02)) * .03
        effort = clamp(.16*error + .025*self.integral + ff, -1., 1.)
        self.previous_target = target
        return max(0., effort), max(0., -effort), self.steer
