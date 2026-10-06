"""Small ROS-independent helpers for local mission timing and obstacle lifetime."""
import math
import numpy as np


def input_fresh(received, now, timeout, stamp=None, ros_now=None):
    if received is None or not 0 <= now - received <= timeout:
        return False
    return stamp is None or (stamp > 0 and -0.1 <= ros_now - stamp <= timeout)


class ObstacleMemory:
    def __init__(self, lifetime, merge_distance):
        if not all(math.isfinite(v) and v > 0 for v in (lifetime, merge_distance)):
            raise ValueError('obstacle memory lifetime/merge distance must be positive and finite')
        self.lifetime, self.merge_distance = lifetime, merge_distance
        self.items = []

    def circles(self, now):
        self.items = [o for o in self.items if 0 <= now - o[3] <= self.lifetime]
        return [o[:3] for o in self.items]

    def observe(self, circles, observed_at, now):
        values = np.asarray(circles, dtype=float)
        if values.size and (values.ndim != 2 or values.shape[1] != 3 or
                            not np.isfinite(values).all() or np.any(values[:, 2] < 0)):
            raise ValueError('invalid obstacle geometry')
        self.circles(now)
        for x, y, radius in circles:
            near = next((i for i, o in enumerate(self.items)
                         if math.hypot(o[0] - x, o[1] - y) < self.merge_distance), None)
            item = (float(x), float(y), float(radius), observed_at)
            if near is None:
                self.items.append(item)
            else:
                self.items[near] = item
        return self.circles(now)


def return_progress(d, heading, now, clear_since, config):
    aligned = abs(d) <= config['return_d_tolerance'] and abs(heading) <= config['return_heading_tolerance']
    if not aligned:
        return False, None
    if clear_since is None or now < clear_since:
        clear_since = now
    return now - clear_since >= config['return_clear_time'], clear_since


def approach_zone_distance(zones, index, x, y, points, margin):
    """Nearest forward zone entry along the route, in metres; never dilates a UTM box."""
    if margin <= 0 or not zones:
        return None
    route = np.vstack(([x, y], points[index + 1:]))
    distance = 0.0
    nearest = None
    for segment, (a, b) in enumerate(zip(route[:-1], route[1:])):
        length = float(np.linalg.norm(b - a))
        if distance > margin:
            break
        for z in zones:
            if 'start' in z:
                entry = distance + length if index + segment + 1 == z['start'] else None
            else:
                # Slab intersection includes entering between waypoints and touching a boundary.
                low, high = 0.0, 1.0
                for axis, lower, upper in ((0, z['xmin'], z['xmax']), (1, z['ymin'], z['ymax'])):
                    delta = b[axis] - a[axis]
                    if abs(delta) < 1e-12:
                        if not lower <= a[axis] <= upper:
                            low, high = 1.0, 0.0
                            break
                    else:
                        t1, t2 = (lower - a[axis]) / delta, (upper - a[axis]) / delta
                        low, high = max(low, min(t1, t2)), min(high, max(t1, t2))
                entry = distance + low * length if low <= high else None
            if entry is not None and 0 <= entry <= margin:
                nearest = entry if nearest is None else min(nearest, entry)
        distance += length
    return nearest
