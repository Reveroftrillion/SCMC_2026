#!/usr/bin/env python3
"""Report route projection and geometry; never certify lane boundaries.

This checks a supplied XY anchor against the route and prints a second, distant
route candidate. A single centreline cannot establish which lane the anchor
refers to, physical road width, or the correctness of the installed sensor TF.
"""
import argparse
import math
import os

import numpy as np

from reproject_missions import MEASURED_XY, cumulative


def audit(path, xy, exclusion=30.0):
    path = np.asarray(path, dtype=float)[:, :2]
    cum = cumulative(path)
    q = np.asarray(xy, dtype=float)
    a, v = path[:-1], np.diff(path, axis=0)
    vv = np.einsum('ij,ij->i', v, v)
    lengths = np.sqrt(vv)
    valid = vv > 1e-12
    t = np.clip(np.einsum('ij,ij->i', q-a, v) / np.where(valid, vv, 1.0), 0, 1)
    delta = q - (a + t[:, None] * v)
    error = np.linalg.norm(delta, axis=1)
    error[~valid] = np.inf
    if not np.isfinite(error).any():
        raise ValueError('path has no nonzero segment')
    i = int(np.argmin(error))
    s = cum[i] + t[i] * lengths[i]
    cross = (v[i, 0] * delta[i, 1] - v[i, 1] * delta[i, 0]) / lengths[i]
    candidate_s = cum[:-1] + t * lengths
    separation = abs(candidate_s - s)
    if np.linalg.norm(path[-1]-path[0]) < 1.0:
        separation = np.minimum(separation, cum[-1] - separation)
    alt_error = error.copy()
    alt_error[separation < exclusion] = np.inf
    j = int(np.argmin(alt_error))
    alt = None if not math.isfinite(alt_error[j]) else (float(candidate_s[j]), float(alt_error[j]), j)

    def heading(at_s):
        j = min(len(v)-1, max(0, int(np.searchsorted(cum, at_s, side='right')-1)))
        if not valid[j]:
            return None
        return math.degrees(math.atan2(v[j, 1], v[j, 0]))
    return {'s': float(s), 'segment': i, 'distance': float(error[i]),
            'signed_lateral': float(cross), 'heading_before': heading(max(0., s-10.)),
            'heading': heading(s), 'heading_after': heading(min(float(cum[-1]), s+10.)),
            'alternative': alt}


def main():
    default = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'paths', 'global_path.txt'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path', default=default)
    args = parser.parse_args()
    path = np.loadtxt(args.path, usecols=(0, 1))
    for key in ('traffic4_start', 'traffic4_stop', 'roundabout_start', 'roundabout_end'):
        result = audit(path, MEASURED_XY[key])
        print('%s s=%.3f segment=%d residual=%.3f signed_d=%+.3f '
              'heading(-10,0,+10)=%s alternative(>=30m along route)=%s' %
              (key, result['s'], result['segment'], result['distance'], result['signed_lateral'],
               ','.join('%.1f' % h if h is not None else '?' for h in
                        (result['heading_before'], result['heading'], result['heading_after'])),
               result['alternative']))
    print('Centreline geometry alone does not verify road edges, lane assignment or TF.')


if __name__ == '__main__':
    main()
