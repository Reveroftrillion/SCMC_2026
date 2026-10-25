"""Tunnel wall inner faces from zero-intensity returns (simulator wall material).

The same returns are excluded from obstacle clustering; here they become the
measured lateral limit of the tunnel (2026-09-26: walls ~11.2 m apart while the
mapped pavement is 7.0 m, so map-only checks refused a passable detour).
"""
import numpy as np


def wall_faces(pts, intensity, x_min=3., x_max=45., bin_m=1., min_h=.3, max_h=3., min_points=3):
    """[(x, y, side)] in base_link: innermost wall face per forward bin, side 'L' / 'R'.

    pts are base_link points with z already relative to the ground plane.
    A robust percentile of the face nearest the car (not the extreme point)
    rejects single stray returns. Bins with too few returns are omitted.
    """
    pts = np.asarray(pts, dtype=float).reshape(-1, 3)
    wall = ((np.asarray(intensity) == 0.) & (pts[:, 0] >= x_min) & (pts[:, 0] < x_max)
            & (pts[:, 2] > min_h) & (pts[:, 2] < max_h))
    x, y = pts[wall, 0], pts[wall, 1]
    out = []
    if not len(x):
        return out
    index = np.floor((x - x_min) / bin_m).astype(int)
    for b in np.unique(index):
        in_bin = index == b
        centre = x_min + (b + .5) * bin_m
        for side, mask, pct in (('R', in_bin & (y < 0.), 90.), ('L', in_bin & (y > 0.), 10.)):
            if mask.sum() >= min_points:
                out.append((float(centre), float(np.percentile(y[mask], pct)), side))
    return out
