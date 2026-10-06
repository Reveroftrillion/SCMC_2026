"""Frenet candidate evaluation; reuses the project's spline and quintic polynomial.

All geometry is metres in map. No vehicle actuation or ROS publishers here.
"""
from dataclasses import dataclass, field
import math
import numpy as np
from planning_geometry import CubicSpline2D, catesian_to_frenet
from interpolation.quintic_polynomials_planner import QuinticPolynomial


@dataclass
class Candidate:
    offset: float
    xy: np.ndarray
    yaw: np.ndarray
    curvature: np.ndarray
    cost: float = float('inf')
    clearance: float = float('inf')
    rejection: str = ''
    costs: dict = field(default_factory=dict)


class FrenetLocalPlanner:
    def __init__(self, config):
        self.p = config
        positive = ('sample_step', 'horizon', 'transition_length', 'return_length',
                    'vehicle_width', 'vehicle_front', 'vehicle_rear', 'wheelbase',
                    'max_curvature', 'max_steering_rate_per_m', 'road_half_width',
                    'min_reference_segment', 'max_reference_yaw_change', 'max_reference_curvature',
                    'max_heading_error')
        for key in positive:
            if not math.isfinite(config[key]) or config[key] <= 0:
                raise ValueError(key + ' must be finite and positive')
        if not config['offsets'] or not all(math.isfinite(x) for x in config['offsets']):
            raise ValueError('offsets must be a nonempty finite list')
        for key in ('safety_margin', 'switch_penalty', 'switch_hysteresis'):
            if not math.isfinite(config[key]) or config[key] < 0:
                raise ValueError(key + ' must be finite and nonnegative')
        if set(config['weights']) != {'obstacle', 'offset', 'curvature', 'steering', 'continuity', 'return'}:
            raise ValueError('invalid candidate cost weight keys')
        if any(not math.isfinite(v) or v < 0 for v in config['weights'].values()):
            raise ValueError('cost weights must be finite and nonnegative')
        for key in ('reference_behind_points', 'reference_ahead_points'):
            if type(config[key]) is not int or config[key] < (4 if key.endswith('ahead_points') else 0):
                raise ValueError(key + ' has invalid point count')
        if config['horizon'] < config['transition_length'] + config['return_length']:
            raise ValueError('horizon must cover transition_length + return_length')
        if config['wheelbase'] > config['vehicle_front'] + config['vehicle_rear']:
            raise ValueError('wheelbase cannot exceed vehicle length')
        if config['vehicle_width'] / 2 + config['safety_margin'] >= config['road_half_width']:
            raise ValueError('vehicle does not fit configured road corridor')
        if config['max_heading_error'] >= math.pi / 2 or config['max_reference_yaw_change'] > math.pi:
            raise ValueError('invalid heading/yaw limits')
        self.previous = None
        self.reference_diagnostic = {}

    def reset(self):
        self.previous = None

    def reference(self, points, index):
        # Bound spline work and disambiguate crossings with the existing waypoint.
        self.reference_diagnostic = {}
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or type(index) not in (int, np.int64, np.int32) or not 0 <= index < len(points):
            raise ValueError('invalid reference points/index')
        lo = max(0, index - self.p['reference_behind_points'])
        hi = min(len(points), index + self.p['reference_ahead_points'])
        xy = np.asarray(points[lo:hi], dtype=float)
        if len(xy) < 4 or not np.isfinite(xy).all():
            raise ValueError('reference path missing or invalid')
        # Compare to the last retained point, so densely sampled paths are thinned,
        # rather than deleting an entire run of individually short segments.
        kept = [xy[0]]
        for point in xy[1:]:
            if np.linalg.norm(point - kept[-1]) >= self.p['min_reference_segment']:
                kept.append(point)
        removed = len(xy) - len(kept)
        xy = np.asarray(kept)
        self.reference_diagnostic = {'input_points': hi - lo, 'removed_points': removed}
        if len(xy) < 4:
            raise ValueError('reference path has too few distinct points')
        segments = np.diff(xy, axis=0)
        headings = np.arctan2(segments[:, 1], segments[:, 0])
        turns = np.arctan2(np.sin(np.diff(headings)), np.cos(np.diff(headings)))
        if np.max(np.abs(turns)) > self.p['max_reference_yaw_change']:
            raise ValueError('reference yaw discontinuity')
        if np.sum(np.linalg.norm(segments, axis=1)) < self.p['transition_length'] + self.p['return_length']:
            raise ValueError('insufficient reference length')
        try:
            csp = CubicSpline2D(xy[:, 0], xy[:, 1], self.p['sample_step'])
        except (ValueError, np.linalg.LinAlgError) as exc:
            raise ValueError('reference spline failed: ' + str(exc)) from exc
        if len(csp.rx) < 4 or not all(np.isfinite(v).all() for v in
                (csp.rx, csp.ry, csp.ryaw, csp.raw_curvature, csp.derivative_speed)):
            raise ValueError('nonfinite reference spline')
        if np.min(csp.derivative_speed) < 1e-4:
            raise ValueError('degenerate reference spline')
        peak = float(np.max(csp.raw_curvature))
        self.reference_diagnostic['peak_curvature'] = peak
        if peak > self.p['max_reference_curvature']:
            raise ValueError('reference curvature spike (%.3f > %.3f)' % (peak, self.p['max_reference_curvature']))
        return csp

    def project(self, csp, x, y):
        # Existing converter seeds a local segment projection (including between samples).
        coarse_s, _ = catesian_to_frenet(x, y, csp)
        i = int(round(coarse_s / csp.interval))
        xy = np.column_stack((csp.rx, csp.ry))
        idx = np.arange(max(0, i - 2), min(len(xy) - 1, i + 3))
        a, v = xy[idx], xy[idx + 1] - xy[idx]
        t = np.clip(np.sum((np.array([x, y]) - a) * v, axis=1) /
                    np.maximum(np.sum(v * v, axis=1), 1e-12), 0, 1)
        q = a + t[:, None] * v
        best = np.argmin(np.linalg.norm(q - [x, y], axis=1))
        s = (idx[best] + t[best]) * csp.interval
        yaw = np.interp(s, np.arange(len(xy)) * csp.interval, np.unwrap(csp.ryaw))
        residual = np.array([x, y]) - q[best]
        d = -math.sin(yaw) * residual[0] + math.cos(yaw) * residual[1]
        return s, d, yaw

    def clearance(self, candidate, obstacles):
        """Swept multi-disc footprint vs bounding circles, including between samples."""
        obs = np.asarray(obstacles, dtype=float)
        if obs.size == 0:
            return float('inf')
        if obs.ndim != 2 or obs.shape[1] != 3 or not np.isfinite(obs).all() or np.any(obs[:, 2] < 0):
            raise ValueError('invalid obstacle circles')
        p = self.p
        length = p['vehicle_front'] + p['vehicle_rear']
        offsets = np.linspace(-p['vehicle_rear'], p['vehicle_front'],
                              max(2, int(math.ceil(length / p['vehicle_width'])) + 1))
        radius = math.hypot(p['vehicle_width'] / 2, (offsets[1] - offsets[0]) / 2)
        radius += p['safety_margin'] + p['sample_step'] / 2
        best = float('inf')
        heading = np.column_stack((np.cos(candidate.yaw), np.sin(candidate.yaw)))
        for offset in offsets:
            centers = candidate.xy + offset * heading
            a, v = centers[:-1], np.diff(centers, axis=0)
            delta = obs[:, None, :2] - a[None, :, :]
            t = np.clip(np.sum(delta * v[None, :, :], axis=2) /
                        np.maximum(np.sum(v * v, axis=1), 1e-12), 0, 1)
            nearest = a[None, :, :] + t[:, :, None] * v[None, :, :]
            distance = np.linalg.norm(obs[:, None, :2] - nearest, axis=2)
            best = min(best, float(np.min(distance - obs[:, None, 2] - radius)))
        return best

    def plan(self, csp, pose, obstacles, returning=False):
        p = self.p
        if np.asarray(pose).shape != (3,) or not np.isfinite(pose).all():
            raise ValueError('invalid planning pose')
        arrays = (csp.rx, csp.ry, csp.ryaw)
        if len(csp.rx) < 4 or len({len(v) for v in arrays}) != 1 or not all(np.isfinite(v).all() for v in arrays):
            raise ValueError('invalid reference samples')
        s0, d0, ref_yaw = self.project(csp, pose[0], pose[1])
        delta = math.atan2(math.sin(pose[2] - ref_yaw), math.cos(pose[2] - ref_yaw))
        if abs(delta) > p['max_heading_error']:
            raise ValueError('heading too far from reference')
        if abs(d0) + p['vehicle_width'] / 2 + p['safety_margin'] > p['road_half_width'] + 1e-6:
            raise ValueError('vehicle outside configured road corridor')
        length = min(p['horizon'], (len(csp.rx) - 1) * csp.interval - s0)
        if length < p['transition_length'] + p['return_length']:
            raise ValueError('insufficient reference for avoidance and return')
        u = np.linspace(0, length, max(5, int(math.ceil(length / p['sample_step'])) + 1))
        ss = np.arange(len(csp.rx)) * csp.interval
        ref_theta = np.interp(s0 + u, ss, np.unwrap(csp.ryaw))
        ref_x = np.interp(s0 + u, ss, csp.rx)
        ref_y = np.interp(s0 + u, ss, csp.ry)
        ref_k = np.gradient(np.unwrap(csp.ryaw), csp.interval)
        slope = (1 - np.interp(s0, ss, ref_k) * d0) * math.tan(delta)
        candidates = []
        for offset in ([0.0] if returning else sorted(set(p['offsets'] + [0.0]))):
            first = QuinticPolynomial(d0, slope, 0.0, offset, 0, 0, p['transition_length'])
            last = QuinticPolynomial(offset, 0, 0, 0, 0, 0, p['return_length'])
            d = np.full_like(u, offset)
            mask = u <= p['transition_length']
            d[mask] = first.calc_point(u[mask])
            mask = u >= length - p['return_length']
            d[mask] = last.calc_point(u[mask] - length + p['return_length'])
            xy = np.column_stack((ref_x - d * np.sin(ref_theta), ref_y + d * np.cos(ref_theta)))
            dx, dy = np.gradient(xy[:, 0], u), np.gradient(xy[:, 1], u)
            yaw = np.unwrap(np.arctan2(dy, dx))
            k = np.gradient(yaw, u) / np.maximum(np.hypot(dx, dy), 1e-6)
            candidate = Candidate(float(offset), xy, yaw, k)
            candidate.clearance = self.clearance(candidate, obstacles)
            steer = np.arctan(p['wheelbase'] * k)
            steer_rate = np.gradient(steer, u)
            relative_yaw = yaw - ref_theta
            footprint_d = [d + longitudinal * np.sin(relative_yaw) + lateral * np.cos(relative_yaw)
                           for longitudinal in (-p['vehicle_rear'], p['vehicle_front'])
                           for lateral in (-p['vehicle_width'] / 2, p['vehicle_width'] / 2)]
            road_extent = max(float(np.max(np.abs(edge))) for edge in footprint_d) + p['safety_margin']
            if not all(np.isfinite(v).all() for v in (xy, yaw, k, steer_rate)):
                candidate.rejection = 'nonfinite'
            elif road_extent > p['road_half_width'] + 1e-6:
                candidate.rejection = 'road_boundary'
            elif candidate.clearance <= 0:
                candidate.rejection = 'collision'
            elif np.max(np.abs(k)) > p['max_curvature']:
                candidate.rejection = 'curvature'
            elif np.max(np.abs(steer_rate)) > p['max_steering_rate_per_m']:
                candidate.rejection = 'steering_change'
            continuity = 0.0
            switch = 0.0
            if self.previous is not None:
                # World-coordinate continuity survives movement of the reference window.
                near = xy[u <= p['transition_length']]
                distances = np.linalg.norm(near[:, None, :] - self.previous.xy[None, :, :], axis=2)
                continuity = float(np.mean(np.min(distances, axis=1) ** 2))
                if offset * self.previous.offset < 0:
                    switch = p['switch_penalty']
            candidate.costs = {
                'obstacle': 0.0 if not math.isfinite(candidate.clearance) else 1 / max(0.05, candidate.clearance),
                'offset': float(np.mean(d * d)),
                'curvature': float(np.mean(k * k)),
                'steering': float(np.mean(steer_rate * steer_rate)),
                'continuity': continuity,
                'return': offset * offset,
            }
            candidate.cost = sum(p['weights'][key] * value for key, value in candidate.costs.items()) + switch
            candidates.append(candidate)
        valid = [c for c in candidates if not c.rejection]
        best = min(valid, key=lambda c: c.cost) if valid else None
        # Keep the old side when its newly checked candidate is almost as good.
        if best is not None and self.previous is not None:
            same = next((c for c in valid if c.offset == self.previous.offset), None)
            if same is not None and same.cost <= best.cost + p['switch_hysteresis']:
                best = same
        return best, candidates, (s0, d0, delta)


def validate_zones(zones):
    if not isinstance(zones, list):
        raise ValueError('zones must be a list')
    for z in zones:
        if not isinstance(z, dict):
            raise ValueError('zone must be a mapping')
        if set(z) == {'start', 'end'}:
            if type(z['start']) is not int or type(z['end']) is not int or not 0 <= z['start'] <= z['end']:
                raise ValueError('zone requires 0 <= start <= end waypoint indices')
        elif set(z) == {'xmin', 'xmax', 'ymin', 'ymax'}:
            if not all(type(v) in (int, float) and math.isfinite(v) for v in z.values()) or z['xmin'] >= z['xmax'] or z['ymin'] >= z['ymax']:
                raise ValueError('invalid map rectangle')
        else:
            raise ValueError('zone must be waypoint range or map rectangle')


def in_zones(zones, index, x, y):
    return any((index is not None and z['start'] <= index <= z['end']) if 'start' in z
               else z['xmin'] <= x <= z['xmax'] and z['ymin'] <= y <= z['ymax'] for z in zones)
