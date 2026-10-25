"""Where the lidar cannot see a car body: distances ahead at which no ring meets the height band of a car.

The 16 rings are 2 deg apart. Past ~40 m at most one ring crosses a car (0.3-1.4 m above the road), and pitch moves
that ring off it: accelerating hard the nose rises ~0.5 deg (2026-10-02 same-model lap, asphalt-plane slope vs
longitudinal accel, corr 0.86: +0.49 deg median above 3 m/s2, -0.16 deg steady, -1.25 deg braking hard). Then the
-1 deg ring passes over a 1.44 m roof out to ~77 m and the -3 deg ring meets the road from ~41 m: a 40 km/h car in
the lane we were changing into was not seen from 80 m to 36 m, while we closed at 120 km/h (s1313, 00:26 lap).

Rays are traced in the vehicle frame (the frame the points are in), over the road plane the scan itself measured
(ground.estimate_ground_plane: z = a x + b y + c, x forward from the rear axle), extended ahead along x.
"""
import math

import numpy as np

RINGS_DEG = tuple(range(-15, 16, 2))


def blind_bands(plane, lidar_x, lidar_z, body_low=.3, body_high=1.4, near=10., far=100., step=.5, min_length=5.2,
                rings_deg=RINGS_DEG):
    """[(x0, x1)] stretches of base_link x where no ring's height above the road lies within [body_low, body_high],
    each at least min_length long (a shorter one cannot hold a whole car: part of it is seen). No plane: flat road."""
    a, c = (float(plane[0]), float(plane[2])) if plane is not None else (0., 0.)
    x = np.arange(near, far+1e-9, step)
    road = a*x+c
    seen = np.zeros(len(x), dtype=bool)
    for e in rings_deg:
        beam = lidar_z+(x-lidar_x)*math.tan(math.radians(e))
        h = beam-road
        seen |= (h >= body_low) & (h <= body_high)
    bands, start = [], None
    for xi, s in zip(x, seen):
        if not s and start is None:
            start = xi
        elif s and start is not None:
            if xi-start >= min_length:
                bands.append((float(start), float(xi)))
            start = None
    if start is not None and x[-1]-start >= min_length:
        bands.append((float(start), float(x[-1])))
    return bands


def pitch_deg(plane):
    """Nose-up pitch read from the road plane's slope along x (the road appears to fall away ahead)."""
    return 0. if plane is None else -math.degrees(math.atan(float(plane[0])))
