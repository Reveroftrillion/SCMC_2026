#!/usr/bin/env python3
"""Read-only path quality and production Local Planner reference analysis."""
import argparse
import math
from pathlib import Path
import numpy as np
import yaml
from local_planner_tools import (DEFAULT_CONFIG, DEFAULT_PATH, load_path, planner_config,
                                protect_outputs, write_csv, write_json)
from local_planning_core import FrenetLocalPlanner


def analyze_path(points, config, lines=None, closure_distance=1.0, check_references=True):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('path requires at least two finite XY points')
    if not math.isfinite(closure_distance) or closure_distance < 0:
        raise ValueError('closure distance must be finite and nonnegative')
    core = FrenetLocalPlanner(config)
    lines = lines or list(range(1, len(points)+1))
    spacing = np.linalg.norm(np.diff(points, axis=0), axis=1)
    duplicate = spacing <= 1e-9
    near = (spacing > 1e-9) & (spacing < config['min_reference_segment'])
    yaw = np.arctan2(np.diff(points[:, 1]), np.diff(points[:, 0]))
    yaw_changes = {}
    previous = None
    for i in np.flatnonzero(~duplicate):
        if previous is not None:
            change = math.atan2(math.sin(yaw[i]-yaw[previous]), math.cos(yaw[i]-yaw[previous]))
            yaw_changes[int(i)] = abs(change)
        previous = i
    curvatures = {}
    for i in range(1, len(points)-1):
        a, b = points[i]-points[i-1], points[i+1]-points[i]
        chord = float(np.linalg.norm(points[i+1]-points[i-1]))
        denom = float(np.linalg.norm(a) * np.linalg.norm(b) * chord)
        # Undefined on coincident/reversing points; do not silently turn these into 0.
        curvatures[i] = None if denom <= 1e-12 else 2*abs(float(a[0]*b[1]-a[1]*b[0])) / denom
    discontinuities = [i for i, v in yaw_changes.items() if v > config['max_reference_yaw_change']]
    spikes = [i for i, v in curvatures.items() if v is not None and v > config['max_reference_curvature']]
    rows = []
    for i, length in enumerate(spacing):
        reasons = []
        if duplicate[i]: reasons.append('duplicate segment')
        elif near[i]: reasons.append('very short / near-duplicate segment')
        if i in discontinuities: reasons.append('yaw discontinuity')
        if i in spikes: reasons.append('discrete curvature spike')
        if curvatures.get(i, 0) is None: reasons.append('undefined discrete curvature')
        rows.append(dict(kind='segment', index=i, end_index=i+1, file_line=lines[i],
                         x=float(points[i,0]), y=float(points[i,1]), spacing=float(length),
                         yaw_change=yaw_changes.get(i, ''), curvature=curvatures.get(i, ''),
                         reason='; '.join(reasons)))
    risks = []
    if check_references:
        for index, (x,y) in enumerate(points):
            try:
                csp = core.reference(points, index)
                s, _, _ = core.project(csp, x, y)
                remaining = (len(csp.rx)-1)*csp.interval-s
                if remaining < config['transition_length']+config['return_length']:
                    raise ValueError('insufficient forward reference for avoidance and return')
            except (ValueError, np.linalg.LinAlgError) as exc:
                risk = dict(kind='local_reference_risk', index=index, end_index='', file_line=lines[index],
                            x=float(x), y=float(y), reason=str(exc))
                rows.append(risk)
                risks.append(dict(index=index, reason=str(exc)))
    finite_k = [k for k in curvatures.values() if k is not None]
    gap = float(np.linalg.norm(points[-1]-points[0]))
    summary = dict(waypoint_count=len(points), total_length_m=float(spacing.sum()),
                   mean_spacing_m=float(spacing.mean()), median_spacing_m=float(np.median(spacing)),
                   duplicate_point_count=len(points)-len(np.unique(points, axis=0)),
                   consecutive_duplicate_segment_count=int(duplicate.sum()),
                   near_duplicate_segment_count=int(near.sum()),
                   very_short_segment_indices=np.flatnonzero(near).tolist(),
                   duplicate_segment_indices=np.flatnonzero(duplicate).tolist(),
                   yaw_discontinuity_indices=discontinuities, curvature_spike_indices=spikes,
                   candidate_curvature_limit_exceeded_indices=[i for i,k in curvatures.items() if k is not None and k > config['max_curvature']],
                   undefined_curvature_indices=[i for i,k in curvatures.items() if k is None],
                   max_discrete_curvature_per_m=max(finite_k) if finite_k else None,
                   closure_gap_m=gap, closure_threshold_m=closure_distance, path_closed=gap <= closure_distance,
                   start=points[0].tolist(), end=points[-1].tolist(),
                   thresholds=dict(min_segment_m=config['min_reference_segment'], yaw_change_rad=config['max_reference_yaw_change'],
                                   reference_curvature_per_m=config['max_reference_curvature'], candidate_curvature_per_m=config['max_curvature']),
                   local_reference_checks=check_references, local_spline_risks=risks)
    return summary, rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path', type=Path, default=DEFAULT_PATH)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--closure-distance', type=float, default=1.0)
    parser.add_argument('--skip-reference-check', action='store_true', help='skip exhaustive runtime spline checks')
    parser.add_argument('--csv', type=Path, help='also writes <name>.summary.json')
    args = parser.parse_args(argv)
    try:
        summary_file = args.csv.with_suffix('.summary.json') if args.csv else None
        protect_outputs([args.csv, summary_file], [args.path, args.config])
        points, lines = load_path(args.path)
        summary, rows = analyze_path(points, planner_config(args.config), lines,
                                     args.closure_distance, not args.skip_reference_check)
        print('Coordinates: file XY; third column ignored, as in GlobalPathPlanner. Indices: zero-based.')
        for key,value in summary.items():
            if isinstance(value, list) and len(value) > 30:
                print('%s: %s ... (%d total; full list in CSV/JSON)' % (key, value[:30], len(value)))
            else:
                print('%s: %s' % (key, value))
        if args.csv:
            write_csv(args.csv, rows, ['kind','index','end_index','file_line','x','y','spacing','yaw_change','curvature','reason'])
            write_json(summary_file, summary)
            print('Reports: %s, %s' % (args.csv, summary_file))
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
        print('ERROR: ' + str(exc))
        return 2
    return 0  # Findings are a report, never an automatic path repair.


if __name__ == '__main__':
    raise SystemExit(main())
