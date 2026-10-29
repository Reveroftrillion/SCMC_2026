#!/usr/bin/env python3
"""Project legacy route-distance anchors by XY, never by total-length scaling."""
import argparse
import bisect
import math
import numpy as np


ANCHORS = (('traffic1_start', 167.40), ('traffic1_stop', 202.4850),
           ('static_start', 270.40), ('traffic2_start', 270.3894),
           ('traffic2_stop', 309.8735), ('static_end', 388.40),
           ('pedestrian_start', 403.40), ('traffic3_start', 405.5593),
           ('traffic3_stop', 430.9174), ('pedestrian_end', 493.40),
           ('traffic4_start', 565.4795), ('traffic4_stop', 600.8039),
           ('roundabout_start', 800.92), ('roundabout_end', 908.49),
           ('highway_prepare', 1100.36), ('tunnel_start', 1807.83),
           ('tunnel_end', 1990.33))

# Measured reference XY supplied for this scenario. These are independent of
# the old route-distance values above. All indices reported are 0-based.
MEASURED_XY = {
    'traffic1_start': (-113.103, -290.250), 'traffic1_stop': (-96.093, -259.572),
    'traffic2_start': (-66.411, -198.680), 'traffic2_stop': (-60.042, -159.820),
    'static_anchor': (-59.858, -140.698), 'static_end': (-59.886, -81.306),
    'traffic3_start': (-59.998, -64.147), 'traffic3_stop': (-60.132, -38.789),
    'pedestrian_end': (-60.463, 23.693),
    'traffic4_start': (-51.096, 93.309), 'traffic4_stop': (-17.248, 99.205),
    'roundabout_start': (-97.843, 242.780), 'roundabout_end': (-77.295, 340.644),
    'highway_prepare': (61.921, 264.852), 'cp10': (62.0739, 256.4949),
    'cp12': (75.4142, -217.8551), 'cp13': (70.2141, -365.7485),
    'tunnel_start': (65.259, -441.315), 'tunnel_end': (-78.224, -546.018),
}
MAX_XY_RESIDUAL = 0.25


def cumulative(path):
    if len(path) < 2 or not np.isfinite(path).all(): raise ValueError('invalid path')
    return np.r_[0., np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]


def point_at(path, distances, s):
    if not 0 <= s <= distances[-1]: raise ValueError('anchor outside old path')
    i = min(len(path)-2, max(0, bisect.bisect_right(distances, s)-1))
    length = distances[i+1]-distances[i]
    if length <= 1e-9: raise ValueError('duplicate anchor segment')
    return path[i, :2] + (path[i+1, :2]-path[i, :2])*((s-distances[i])/length)


def project(path, distances, point):
    a, v = path[:-1, :2], np.diff(path[:, :2], axis=0)
    vv = np.sum(v*v, axis=1)
    valid = vv > 1e-12
    t = np.clip(np.sum((point-a)*v, axis=1)/np.where(valid, vv, 1), 0, 1)
    errors = np.linalg.norm(a+t[:, None]*v-point, axis=1)
    errors[~valid] = np.inf
    best = int(np.argmin(errors))
    s = distances[best] + t[best]*math.sqrt(vv[best])
    # A nearby, distant-along-route alternative makes projection ambiguous.
    candidates_s = distances[:-1] + t*np.sqrt(vv)
    ambiguity = np.any((errors <= errors[best]+0.5) & (np.abs(candidates_s-s) > 10))
    return s, float(errors[best]), bool(ambiguity)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('legacy_path', nargs='?', help='old path when converting old s anchors')
    ap.add_argument('current_path', nargs='?')
    ap.add_argument('--measured-xy', metavar='CURRENT_PATH',
                    help='project the measured scenario XY onto CURRENT_PATH')
    args = ap.parse_args()
    if args.measured_xy:
        path = np.loadtxt(args.measured_xy, usecols=(0, 1))
        distance = cumulative(path)
        print('Current %.3f m (%d points); max accepted XY residual %.2f m' %
              (distance[-1], len(path), MAX_XY_RESIDUAL))
        last = -1.0
        for name, xy in MEASURED_XY.items():
            s, error, ambiguous = project(path, distance, np.asarray(xy))
            index = int(np.argmin(np.linalg.norm(path[:, :2] - xy, axis=1)))
            ordered = s > last
            valid = error <= MAX_XY_RESIDUAL and not ambiguous and ordered
            print('%-20s s=%9.3f waypoint=%4d residual=%6.3fm ambiguous=%s ordered=%s %s' %
                  (name, s, index, error, ambiguous, ordered, 'OK' if valid else 'UNVERIFIED'))
            last = s
        print('Static/pedestrian start XY not supplied; do not assume their candidate boundaries verified.')
        return
    if not args.legacy_path or not args.current_path:
        ap.error('provide legacy_path and current_path, or --measured-xy CURRENT_PATH')
    old = np.loadtxt(args.legacy_path, usecols=(0, 1)); new = np.loadtxt(args.current_path, usecols=(0, 1))
    so, sn = cumulative(old), cumulative(new)
    print('Legacy %.3f m (%d points); current %.3f m (%d points)' % (so[-1], len(old), sn[-1], len(new)))
    if len(old) != 4430 or abs(so[-1]-2174.55) > 5:
        raise SystemExit('Legacy file does not match the stated 4430-point / 2174.55m reference; no output accepted.')
    for name, s in ANCHORS:
        xy = point_at(old, so, s)
        new_s, error, ambiguous = project(new, sn, xy)
        print('%-20s old=%8.3f xy=(%11.4f,%11.4f) new=%8.3f residual=%5.2f %s' %
              (name, s, xy[0], xy[1], new_s, error, 'AMBIGUOUS' if ambiguous else ''))
    print('Review residuals and crossing ambiguity; this report never enables missions automatically.')


if __name__ == '__main__': main()
