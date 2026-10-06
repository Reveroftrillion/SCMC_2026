"""ROS-independent spline and Frenet helpers, also re-exported by utils."""
import numpy as np
from scipy.interpolate import CubicSpline as CubicSpline1D


class CubicSpline2D:
    def __init__(self, x, y, interval=0.2):
        self.interval = interval
        dx, dy = np.diff(x), np.diff(y)
        self.rs = np.concatenate([[0], np.cumsum(np.hypot(dx, dy))])
        sx, sy = CubicSpline1D(self.rs, x), CubicSpline1D(self.rs, y)
        ss = np.arange(0, self.rs[-1], interval)
        self.rx, self.ry = sx(ss), sy(ss)
        dx, dy = sx(ss, 1), sy(ss, 1)
        self.ryaw = np.arctan2(dy, dx)
        ddx, ddy = sx(ss, 2), sy(ss, 2)
        with np.errstate(divide='ignore', invalid='ignore'):
            kappa = np.abs(dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** 1.5
        self.rkappa = np.nan_to_num(kappa, nan=0.0, posinf=0.0, neginf=0.0)
        # Local callers inspect the raw derivatives; retain utils' public behaviour.
        self.derivative_speed = np.hypot(dx, dy)
        self.raw_curvature = kappa


def catesian_to_frenet(x, y, csp):
    dist = np.hypot(csp.rx - x, csp.ry - y)
    idx = np.argmin(dist)
    dx, dy = x - csp.rx[idx], y - csp.ry[idx]
    s = idx * csp.interval
    d = np.copysign(dist[idx], np.cos(csp.ryaw[idx]) * dy - np.sin(csp.ryaw[idx]) * dx)
    return s, d
