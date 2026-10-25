"""Sparse map-frame occupancy from complete lidar footprint scans, without object classes.

As with RingLane, non-shadowed cells inside the observed range are assumed visible.
This is a footprint visibility approximation, not raw-ray evidence of free space;
convex footprints do not expose unobserved concavities. Callers check scan freshness.
"""
import math

import numpy as np

from .avoid_planner import polygon_box

FREE, OCCUPIED, UNKNOWN = 0, 1, 2


class BevOccupancy:
    def __init__(self, resolution=.25, sight_m=40.):
        if not math.isfinite(resolution) or resolution <= 0. or not math.isfinite(sight_m) or sight_m <= 0.:
            raise ValueError('resolution and sight must be finite and positive')
        self.resolution, self.sight_m = float(resolution), float(sight_m)
        self.stamp = -math.inf
        self.sensor_xy = np.zeros(2)
        self.occupied_indices = np.empty((0, 2), dtype=np.int64)
        self.region_cells, self.hulls = {}, {}
        self._occupied, self._shadows = set(), []

    @property
    def occupied_cells(self):
        return (self.occupied_indices + .5)*self.resolution

    @staticmethod
    def _polygon(points):
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3 or not np.isfinite(points).all():
            raise ValueError('a footprint must contain at least three finite planar points')
        hull = polygon_box(points).vertices
        if len(hull) < 3:
            raise ValueError('a footprint must have positive area')
        return np.asarray(hull, dtype=float)

    def cell_polygons(self, indices):
        indices = np.asarray(indices, dtype=np.int64).reshape(-1, 2)
        return (indices[:, None, :] + np.array([[0, 0], [1, 0], [1, 1], [0, 1]]))*self.resolution

    def intersected_cells(self, polygon):
        """Integer map cells whose closed square intersects the convex footprint, including contact."""
        p = self._polygon(polygon)
        low = np.floor(np.nextafter(p.min(axis=0)/self.resolution, -np.inf)).astype(np.int64)
        high = np.floor(p.max(axis=0)/self.resolution).astype(np.int64)
        x, y = np.meshgrid(np.arange(low[0], high[0]+1), np.arange(low[1], high[1]+1), indexing='ij')
        indices = np.column_stack((x.ravel(), y.ravel()))
        centers = (indices + .5)*self.resolution
        edges = np.roll(p, -1, axis=0)-p
        axes = np.column_stack((-edges[:, 1], edges[:, 0]))
        projections = p @ axes.T
        cell_projection = centers @ axes.T
        radius = self.resolution/2.*np.abs(axes).sum(axis=1)
        separated = ((cell_projection+radius < projections.min(axis=0))
                     | (cell_projection-radius > projections.max(axis=0))).any(axis=1)
        return indices[~separated]

    def update(self, stamp, hulls, sensor_xy, heights=None, sensor_z=None, low_m=None):
        """Replace a complete scan; None denotes missing input, while {} is a received empty scan.

        With heights (key -> top above ground), the lidar height sensor_z and low_m, a body lower than the lidar casts
        its shadow only as far as a body low_m tall stays hidden behind it, (h - low_m) * r / (sensor_z - h); one no
        taller than low_m casts none. Without a height a hull shadows all the way, as before.

        Identifiers only label returned region geometry; they do not decide occupancy or motion.
        An invalid scan leaves the previous field and timestamp intact.
        """
        if hulls is None or stamp <= self.stamp:
            return False
        sensor = np.asarray(sensor_xy, dtype=float)
        if not math.isfinite(stamp) or sensor.shape != (2,) or not np.isfinite(sensor).all():
            raise ValueError('scan stamp and sensor position must be finite')
        shapes = {key: self._polygon(points) for key, points in hulls.items()}
        regions = {key: self.intersected_cells(points) for key, points in shapes.items()}
        occupied = set().union(*(set(map(tuple, cells)) for cells in regions.values()))
        shadows = []
        for key, points in shapes.items():
            top = heights.get(key) if heights is not None and sensor_z is not None and low_m is not None else None
            if top is not None and top <= low_m:
                continue
            rel = points-sensor
            edges = np.roll(rel, -1, axis=0)-rel
            crosses = edges[:, 0]*(-rel[:, 1])-edges[:, 1]*(-rel[:, 0])
            if np.all(crosses >= 0.) or np.all(crosses <= 0.):
                shadows.append(None)
                continue
            ratio = np.clip(-np.sum(rel*edges, axis=1)/np.sum(edges*edges, axis=1), 0., 1.)
            nearest = np.linalg.norm(rel+ratio[:, None]*edges, axis=1).min()
            # Extrude the actual hull along sensor rays. The far scaled hull is
            # entirely beyond the query range, including whole-cell boundaries.
            reach = self.sight_m
            if top is not None and top < sensor_z:
                reach = min(reach, (top-low_m)*nearest/(sensor_z-top))
            scale = 1.+(reach+self.resolution*math.sqrt(2.))/max(nearest,1e-9)
            shadow = self._polygon(np.r_[rel,rel*scale])
            edge = np.roll(shadow,-1,axis=0)-shadow
            axes = np.r_[np.eye(2),np.c_[-edge[:,1],edge[:,0]]]
            projection = shadow@axes.T
            radius = self.resolution/2.*np.abs(axes).sum(axis=1)
            shadows.append((axes,projection.min(axis=0)-radius,projection.max(axis=0)+radius))
        self.stamp, self.sensor_xy = float(stamp), sensor.copy()
        self.hulls, self.region_cells = shapes, regions
        self._occupied, self._shadows = occupied, shadows
        self.occupied_indices = np.asarray(sorted(occupied), dtype=np.int64).reshape(-1, 2)
        return True

    def intersected_cells_many(self, rectangles):
        """intersected_cells for n convex quadrilaterals (n, 4, 2) at once: (owner index per cell, cells)."""
        p = np.asarray(rectangles, dtype=float).reshape(-1, 4, 2)
        if not len(p):
            return np.empty(0, dtype=np.int64), np.empty((0, 2), dtype=np.int64)
        low = np.floor(np.nextafter(p.min(axis=1)/self.resolution, -np.inf)).astype(np.int64)
        high = np.floor(p.max(axis=1)/self.resolution).astype(np.int64)
        span = (high-low).max(axis=0)+1
        gx, gy = np.meshgrid(np.arange(span[0]), np.arange(span[1]), indexing='ij')
        grid = np.column_stack((gx.ravel(), gy.ravel()))
        indices = low[:, None, :]+grid[None, :, :]
        inside = np.all(indices <= high[:, None, :], axis=2)
        centers = (indices+.5)*self.resolution
        edges = np.roll(p, -1, axis=1)-p
        axes = np.stack((-edges[:, :, 1], edges[:, :, 0]), axis=2)
        projections = np.einsum('nvk,nak->nva', p, axes)
        cell_projection = np.einsum('nck,nak->nca', centers, axes)
        radius = self.resolution/2.*np.abs(axes).sum(axis=2)
        separated = ((cell_projection+radius[:, None, :] < projections.min(axis=1)[:, None, :])
                     | (cell_projection-radius[:, None, :] > projections.max(axis=1)[:, None, :])).any(axis=2)
        keep = inside & ~separated
        owner = np.nonzero(keep)[0]
        return owner, indices[keep]

    def query(self, points):
        """Cell states at map points. Range/shadow checks cover each entire square conservatively."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        result = np.full(len(points), UNKNOWN, dtype=np.uint8)
        if not math.isfinite(self.stamp) or not len(points):
            return result
        finite = np.isfinite(points).all(axis=1)
        rows = np.flatnonzero(finite)
        indices = np.floor(points[finite]/self.resolution).astype(np.int64)
        rel = (indices+.5)*self.resolution-self.sensor_xy
        distance = np.linalg.norm(rel, axis=1)
        half_diagonal = self.resolution/math.sqrt(2.)
        unknown = distance+half_diagonal > self.sight_m
        for shadow in self._shadows:
            if shadow is None:
                unknown[:] = True
                break
            axes,low,high = shadow
            projection = rel@axes.T
            unknown |= np.all((projection>=low)&(projection<=high),axis=1)
        result[rows] = np.where(unknown, UNKNOWN, FREE)
        occupied = np.fromiter((tuple(cell) in self._occupied for cell in indices), bool, count=len(indices))
        result[rows[occupied]] = OCCUPIED
        return result

    def hits_footprint(self, polygon, include_unknown=True):
        cells = self.intersected_cells(polygon)
        states = self.query((cells+.5)*self.resolution)
        return bool(np.any(states != FREE if include_unknown else states == OCCUPIED))

    def hits_bodies(self, positions, yaws, front, rear, half):
        """Whether any sampled rear-axle pose touches occupied cells; UNKNOWN is excluded.

        Exact rectangle/cell SAT follows a bounding-box filter. Batches bound temporary
        arrays independently of scan density and path length; they do not alter geometry.
        """
        positions = np.asarray(positions, dtype=float).reshape(-1, 2)
        yaws = np.asarray(yaws, dtype=float).reshape(-1)
        if len(positions) != len(yaws):
            raise ValueError('each body position requires one yaw')
        if not len(positions) or not len(self.occupied_indices):
            return False
        cells = self.occupied_cells
        cell_half = self.resolution/2.
        half_length, shift = (front+rear)/2., (front-rear)/2.
        for first in range(0, len(positions), 64):
            pos, yaw = positions[first:first+64], yaws[first:first+64]
            co, si = np.cos(yaw), np.sin(yaw)
            ac, asi = np.abs(co), np.abs(si)
            center = pos+shift*np.column_stack((co, si))
            extents = np.column_stack((half_length*ac+half*asi, half_length*asi+half*ac))
            low, high = (center-extents).min(axis=0)-cell_half, (center+extents).max(axis=0)+cell_half
            nearby = cells[np.all((cells >= low) & (cells <= high), axis=1)]
            for at in range(0, len(nearby), 512):
                delta = nearby[None, at:at+512, :]-center[:, None, :]
                dx, dy = delta[:, :, 0], delta[:, :, 1]
                overlap = np.all(np.abs(delta) <= extents[:, None, :]+cell_half, axis=2)
                radius = cell_half*(ac+asi)
                overlap &= np.abs(dx*co[:, None]+dy*si[:, None]) <= (half_length+radius)[:, None]
                overlap &= np.abs(-dx*si[:, None]+dy*co[:, None]) <= (half+radius)[:, None]
                if overlap.any():
                    return True
        return False
