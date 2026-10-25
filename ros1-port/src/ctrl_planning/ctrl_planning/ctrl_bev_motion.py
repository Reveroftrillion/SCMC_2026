"""Planar occupied-region motion; association uses geometry, never source IDs."""
from dataclasses import dataclass, replace
import math

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import ConvexHull, QhullError

from .ctrl_highway_box_tracks import EdgeTracks


def _polygon(points):
    try:
        p = np.asarray(points, dtype=float)
    except (TypeError, ValueError):
        return None
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 3 or not np.isfinite(p).all():
        return None
    area = np.sum(p[:, 0]*np.roll(p[:, 1], -1)-p[:, 1]*np.roll(p[:, 0], -1))
    return p if abs(area) > 1e-12 else None


@dataclass(frozen=True)
class OccupancyRegion:
    polygon: tuple
    stamp: float
    velocity: tuple = None
    sigma: tuple = None
    ready: bool = False
    coasted: float = 0.
    observed_polygon: tuple = ()

    @property
    def center(self):
        return tuple(np.mean(self.polygon, axis=0))

    @property
    def covariance(self):
        """Velocity covariance, or None before motion is measured."""
        return None if self.sigma is None else ((self.sigma[0]**2, 0.), (0., self.sigma[1]**2))

    def at(self, t):
        """Mean polygon at absolute scan-clock time t; consult ranges if unready."""
        dt = max(0., float(t)-self.stamp)
        velocity = self.velocity if self.velocity is not None else (0., 0.)
        return tuple((x+velocity[0]*dt, y+velocity[1]*dt) for x, y in self.polygon)

    def displacement_range(self, t):
        """Per-axis displacement intervals from the observed polygon, including acceleration noise."""
        dt = max(0., float(t)-self.stamp)
        velocity = self.velocity if self.velocity is not None else (0., 0.)
        sigma = self.sigma if self.sigma is not None else (EdgeTracks.INIT_SPEED_SIGMA,)*2
        return tuple((v*dt-EdgeTracks.GATE_SIGMA*math.hypot(sd*dt, EdgeTracks.ACCEL_SIGMA*dt*dt/2.),
                      v*dt+EdgeTracks.GATE_SIGMA*math.hypot(sd*dt, EdgeTracks.ACCEL_SIGMA*dt*dt/2.))
                     for v, sd in zip(velocity, sigma))


class _RegionTrack:
    def __init__(self, polygon, stamp):
        self.region = OccupancyRegion(tuple(map(tuple, polygon)), stamp)
        self.axes = (EdgeTracks(), EdgeTracks())
        self._coherent = [None, None]
        for axis in self.axes:
            # Region association already happened in 2-D. Do not borrow another edge's speed.
            axis.INHERIT = False

    def observe(self, stamp, polygon, visible, sensor):
        estimates = []
        for dim, tracker in enumerate(self.axes):
            lo, hi = float(polygon[:, dim].min()), float(polygon[:, dim].max())
            rates = tracker.update(stamp, {'lo': [(lo, visible[dim][0])],
                                           'hi': [(hi, visible[dim][1])]}, float(sensor[dim]))
            known = []
            measured_known = False
            for edge_index, kind in enumerate(('lo', 'hi')):
                rate = rates[kind][0]
                edge = tracker.tracks[kind][0]
                if rate is not None:
                    measured_known |= visible[dim][edge_index]
                    tracker._predict(edge, stamp)
                    known.append((rate, float(edge['P'][1, 1])))
            if not known:
                estimates.append(None)
                continue
            if all(visible[dim]) and len(known) < 2:
                # A newly revealed opposite edge must corroborate translation.
                # The lone mature edge can be a receding visible outline.
                previous = self._coherent[dim]
                held = None
                if previous is not None:
                    measured_at, speed, sigma = previous
                    age = stamp-measured_at
                    if age <= tracker.HOLD_S:
                        held = (speed, math.hypot(sigma, tracker.ACCEL_SIGMA*age))
                estimates.append(held)
                continue
            if len(known) == 2:
                (v0, p0), (v1, p1) = known
                # Opposite ends expanding or shrinking disagree: that is shape,
                # not a translation of the occupied region.
                if abs(v0-v1) > math.sqrt(p0+p1):
                    previous = self._coherent[dim]
                    held = None
                    if previous is not None:
                        measured_at, speed, sigma = previous
                        age = stamp-measured_at
                        if age <= tracker.HOLD_S:
                            held = (speed, math.hypot(sigma, tracker.ACCEL_SIGMA*age))
                    estimates.append(held)
                    continue
                weights = np.array([1./max(p0, 1e-12), 1./max(p1, 1e-12)])
                speed = float(np.dot(weights, [v0, v1])/weights.sum())
                # Both ends share scan/pose error; averaging must not erase it.
                variance = max(p0, p1)+(v0-v1)**2/4.
                estimates.append((speed, math.sqrt(variance)))
            else:
                estimates.append((known[0][0], math.sqrt(known[0][1])))
            # A held/hidden edge is not new motion evidence. In particular,
            # repeated outline contradictions must not restart the hold timer.
            if measured_known and estimates[-1][1] <= tracker.READY_SIGMA:
                self._coherent[dim] = (stamp, *estimates[-1])
        ready = all(v is not None and v[1] <= EdgeTracks.READY_SIGMA for v in estimates)
        occupied = polygon
        if self.region.ready and not np.all(visible):
            previous = np.asarray(self.region.at(stamp))
            keep = np.ones(len(previous), dtype=bool)
            for dim in (0, 1):
                if visible[dim][0]:
                    keep &= previous[:, dim] >= polygon[:, dim].min()-1e-9
                if visible[dim][1]:
                    keep &= previous[:, dim] <= polygon[:, dim].max()+1e-9
            if keep.any():
                combined = np.vstack((polygon, previous[keep]))
                try:
                    occupied = combined[ConvexHull(combined).vertices]
                except QhullError:
                    pass
        self.region = OccupancyRegion(tuple(map(tuple, occupied)), stamp,
            tuple(v[0] for v in estimates) if ready else None,
            tuple(v[1] for v in estimates) if ready else None, ready,
            observed_polygon=tuple(map(tuple, polygon)))


class OccupancyMotion:
    """Current outlines and their 2-D edge rates, associated without semantic labels."""
    def __init__(self):
        self.stamp = -math.inf
        self.regions = []
        self._tracks = []

    @staticmethod
    def _visible_edges(polygon, polygons, sensor):
        """An extremum is measured only if a ray just beyond it reaches clear space."""
        starts = np.concatenate(polygons)
        ends = np.concatenate([np.roll(p, -1, axis=0) for p in polygons])
        segments = ends-starts
        rel = starts-sensor
        result = []
        for dim in (0, 1):
            pair = []
            for sign, extremum in ((-1., polygon[:, dim].min()), (1., polygon[:, dim].max())):
                vertices = polygon[np.abs(polygon[:, dim]-extremum) <= 1e-9].copy()
                vertices[:, dim] += sign*EdgeTracks.POS_SIGMA
                rays = vertices-sensor
                den = rays[:, None, 0]*segments[None, :, 1]-rays[:, None, 1]*segments[None, :, 0]
                valid = np.abs(den) > 1e-12
                safe_den = np.where(valid, den, 1.)
                t = (rel[None, :, 0]*segments[None, :, 1]-rel[None, :, 1]*segments[None, :, 0])/safe_den
                u = (rel[None, :, 0]*rays[:, None, 1]-rel[None, :, 1]*rays[:, None, 0])/safe_den
                blocked = np.any(valid & (t > 1e-9) & (t < 1.-1e-9) & (u >= 0.) & (u <= 1.), axis=1)
                pair.append(bool(np.any(~blocked)))
            result.append(pair)
        return result

    def update(self, stamp, hulls, sensor_xy):
        """Accept one complete scan; false means invalid, duplicate or out-of-order."""
        try:
            sensor = np.asarray(sensor_xy, dtype=float)
            if not math.isfinite(stamp) or stamp <= self.stamp or sensor.shape != (2,) or not np.isfinite(sensor).all():
                return False
        except (TypeError, ValueError):
            return False
        if not isinstance(hulls, dict):
            return False
        polygons = [p for points in hulls.values() if (p := _polygon(points)) is not None]
        self._tracks = [tr for tr in self._tracks if stamp-tr.region.stamp <= EdgeTracks.KEEP_S]
        assignments = {}
        if polygons and self._tracks:
            observed = np.array([p.mean(axis=0) for p in polygons])
            predicted = np.array([np.mean(tr.region.at(stamp), axis=0) for tr in self._tracks])
            distances = np.linalg.norm(observed[:, None]-predicted[None], axis=2)
            gates = np.array([EdgeTracks.GATE_MIN_M+
                (EdgeTracks.MAX_SPEED if tr.region.sigma is None else EdgeTracks.GATE_SIGMA*max(tr.region.sigma))
                *max(0., stamp-tr.region.stamp) for tr in self._tracks])
            cost = np.where(distances <= gates, distances, 1e12)
            rows, cols = linear_sum_assignment(cost)
            assignments = {int(i): int(j) for i, j in zip(rows, cols) if cost[i, j] < 1e12}
        tracks = []
        for i, polygon in enumerate(polygons):
            track = self._tracks[assignments[i]] if i in assignments else _RegionTrack(polygon, stamp)
            track.observe(stamp, polygon, self._visible_edges(polygon, polygons, sensor), sensor)
            tracks.append(track)
        used = set(assignments.values())
        tracks.extend(tr for i, tr in enumerate(self._tracks) if i not in used)
        self._tracks = tracks
        self.stamp = float(stamp)
        self.regions = [replace(tr.region, coasted=max(0., stamp-tr.region.stamp)) for tr in tracks]
        return True
