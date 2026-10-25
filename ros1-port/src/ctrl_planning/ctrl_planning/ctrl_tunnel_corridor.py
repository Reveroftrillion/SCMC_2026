"""Measured tunnel walls as the drivable surface, with the map as fallback.

Walls are static, so faces seen ahead are remembered by route station for the
whole tunnel: the lidar looks forward only, but the avoidance sweep also checks
the body beside and behind the car.
"""
import itertools
import math
import numpy as np
from matplotlib.path import Path

_serial = itertools.count()


class WallMemory:
    def __init__(self, bin_m=1.):
        self.bin_m = bin_m
        self.clear()                      # faces: side -> station bin -> (x, y, d)

    def clear(self):
        self.faces = {'L': {}, 'R': {}}      # station bin -> median face (x, y, d)
        self.history = {'L': {}, 'R': {}}    # station bin -> recent faces
        self.built_at, self.built_reach = -math.inf, -math.inf

    def due(self, now, reach_m=5., period_s=1.):
        """Rebuild only when coverage grew by reach_m or period_s passed: static walls,
        and every rebuild invalidates the avoidance sweep cache."""
        both = set(self.faces['L']) & set(self.faces['R'])
        reach = max(both)*self.bin_m if both else -math.inf
        if reach-getattr(self, 'built_reach', -math.inf) >= reach_m or now-getattr(self, 'built_at', -math.inf) >= period_s:
            self.built_at, self.built_reach = now, reach
            return True
        return False

    def add(self, rows, route, near_local_s, s_offset, to_frenet, road_map=None, max_d_m=12., keep=9):
        """rows: [[x, y, side], ...] in map.

        Faces on the mapped pavement are not walls: a barricade's zero-intensity
        returns once put the "left wall" at d=+1.6 m and stopped the car (2026-09-26);
        the real walls stand ~2 m outside the pavement. Faces beyond max_d_m are not
        this tunnel's walls. Each station uses the median of its recent faces: keeping
        the outermost let one far return at the tunnel mouth (d=+22 m) stick for good.
        """
        parsed = []
        for row in rows:
            try:
                x, y, side = float(row[0]), float(row[1]), row[2]
            except (TypeError, ValueError, IndexError):
                continue
            if side in self.faces and math.isfinite(x) and math.isfinite(y):
                parsed.append((x, y, side))
        if parsed and road_map is not None:
            on_road = road_map.contains_road([(x, y) for x, y, _ in parsed])
            parsed = [r for r, inside in zip(parsed, on_road) if not inside]
        changed = False
        for x, y, side in parsed:
            s_local, d = to_frenet(route, x, y, near_local_s)
            if abs(d) > max_d_m:
                continue
            # Side by route offset, not by the lidar's y sign: during a swerve the car
            # yaws ~20 deg and the far right wall appears at y>0 (it was filed as the
            # left wall at d=-4 and closed the left detour, 2026-09-26).
            side = 'L' if d > 0 else 'R'
            key = int(math.floor((s_local - s_offset) / self.bin_m))
            seen = self.history[side].setdefault(key, [])
            seen.append((x, y, d))
            del seen[:-keep]
            self.faces[side][key] = sorted(seen, key=lambda f: f[2])[len(seen)//2]
            self.route, self.s_offset = route, s_offset
            changed = True
        return changed

    def wall_line(self, side, window_m=30., reject_m=.4):
        """Walls are straight and parallel to the road: per station, a robust linear
        fit of the per-bin medians within +-window_m/2. Faces more than reject_m
        (or 3 robust sigma) off the line are dropped before refitting. Stations
        between measured bins inside the window are filled by the same line."""
        keys = sorted(self.faces[side])
        if len(keys) < 3:
            return {}
        k = np.array(keys, dtype=float)
        d = np.array([self.faces[side][b][2] for b in keys])
        half = window_m/self.bin_m/2.
        out = {}
        for b in range(keys[0], keys[-1]+1):
            sel = np.abs(k-b) <= half
            if sel.sum() < 3:
                continue
            kk, dd = k[sel], d[sel]
            for _ in range(3):
                coef = np.polyfit(kk, dd, 1)
                res = dd-np.polyval(coef, kk)
                mad = 1.4826*np.median(np.abs(res-np.median(res)))
                keep = np.abs(res) <= max(reject_m, 3.*mad)
                if keep.all() or keep.sum() < 3:
                    break
                kk, dd = kk[keep], dd[keep]
            out[b] = float(np.polyval(coef, b))
        return out

    def corridor(self, road_map, clearance_m=.2, max_gap_bins=3, extend_m=40., end_s=None):
        """Corridor between the fitted wall lines. extend_m: beyond the last fitted
        station the walls continue at their last offset, up to end_s, re-measured each
        scan (without it the return curve past the sensor range was judged against the
        7 m mapped pavement and a passable barricade stopped the car)."""
        route = getattr(self, 'route', None)
        if route is None:
            return None
        from .ctrl_frenet import to_xy
        lines = {side: self.wall_line(side) for side in ('L', 'R')}
        both = sorted(set(lines['L']) & set(lines['R']))
        if not both:
            return None
        if extend_m > 0:
            last = both[-1]
            limit = last + int(extend_m/self.bin_m)
            if end_s is not None:
                limit = min(limit, int(math.floor(end_s/self.bin_m)))
            for side in ('L', 'R'):
                for b in range(last+1, limit+1):
                    lines[side][b] = lines[side][last]
            both += list(range(last+1, limit+1))
        def xy(side, b):
            return to_xy(route, (b+.5)*self.bin_m+self.s_offset, lines[side][b])
        runs, run = [], [both[0]]
        for b in both[1:]:
            if b - run[-1] <= max_gap_bins:
                run.append(b)
            else:
                runs.append(run)
                run = [b]
        runs.append(run)
        return WallCorridor(road_map, [([xy('R', b) for b in r], [xy('L', b) for b in r])
                                       for r in runs if len(r) >= 2], clearance_m)


class WallCorridor:
    """Drop-in for LaneMap.contains_road inside the tunnel.

    Where both walls were measured, a point is drivable iff it lies between the
    wall faces minus clearance_m. Elsewhere the mapped pavement decides.
    """

    def __init__(self, road_map, runs, clearance_m=.2, band_m=5.):
        self.road_map = road_map
        self.cache_key = ('walls', next(_serial))
        self.runs = []
        for right, left in runs:
            r, l = np.asarray(right, float), np.asarray(left, float)
            across = l - r
            n = across / np.maximum(np.linalg.norm(across, axis=1)[:, None], 1e-9)
            inner = Path(np.vstack([r + n*clearance_m, (l - n*clearance_m)[::-1]]))
            band = Path(np.vstack([r - n*band_m, (l + n*band_m)[::-1]]))
            self.runs.append((inner, band, r, l))

    def contains_road(self, points, seam_tolerance=.15):
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        ok = np.zeros(len(pts), dtype=bool)
        covered = np.zeros(len(pts), dtype=bool)
        for inner, band, _, _ in self.runs:
            measured = band.contains_points(pts) & ~covered
            if measured.any():
                ok[measured] = inner.contains_points(pts[measured])
                covered |= measured
        # Only unmeasured points query the map: the KD-tree lookup dominated the
        # avoidance sweep (170 ms ticks, 0.85 s target gaps -> NO_TARGET braking).
        rest = ~covered
        if rest.any() and self.road_map is not None:
            ok[rest] = self.road_map.contains_road(pts[rest], seam_tolerance)
        return ok

    def lines(self):
        """(right, left) wall polylines per measured run, for display."""
        return [(r, l) for _, _, r, l in self.runs]
