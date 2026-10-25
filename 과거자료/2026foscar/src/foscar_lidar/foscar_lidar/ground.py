"""Conservative local road-plane estimate for ground removal during body tilt.

Only a broad, dominant, near-ground surface is accepted. Missing support falls
back to the original fixed-height filter; no plane is extrapolated past 28 m.
"""

import numpy as np


def region(points):
    return ((points[:, 0] > 3.) & (points[:, 0] < 28.)
            & (np.abs(points[:, 1]) < 5.))


def estimate_ground_plane(points):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    points = points[region(points) & (points[:, 2] > -.8) & (points[:, 2] < 1.2)]
    if len(points) < 150:
        return None
    rng = np.random.default_rng(0)
    if len(points) > 1500:
        points = points[rng.choice(len(points), 1500, replace=False)]
    design = np.c_[points[:, :2], np.ones(len(points))]
    best = None
    count = 0
    for _ in range(80):
        idx = rng.choice(len(points), 3, replace=False)
        try:
            plane = np.linalg.solve(design[idx], points[idx, 2])
        except np.linalg.LinAlgError:
            continue
        if np.linalg.norm(plane[:2]) > .18 or abs(plane[2]) > .25:
            continue
        inliers = np.abs(design @ plane - points[:, 2]) < .12
        if int(inliers.sum()) > count:
            best, count = inliers, int(inliers.sum())
    if best is None or count < max(120, .6 * len(points)):
        return None
    support = points[best]
    if np.ptp(support[:, 0]) < 8. or np.ptp(support[:, 1]) < 3.:
        return None
    plane = np.linalg.lstsq(design[best], points[best, 2], rcond=None)[0]
    if np.linalg.norm(plane[:2]) > .18 or abs(plane[2]) > .25:
        return None
    return plane


def relative_height(points, plane):
    points = np.asarray(points)
    height = points[:, 2].copy()
    if plane is not None:
        supported = region(points)
        height[supported] -= points[supported, :2] @ plane[:2] + plane[2]
    return height
