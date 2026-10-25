#!/usr/bin/env python3
"""Plot estimated native paths from CLOSED, analyzed left-box audits.

Localization estimates are not simulator ground truth or swept-body clearance.
Both output files must be new; source evidence is read only.
"""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    manifest = args.output.with_suffix('.json')
    if args.output.exists() or manifest.exists():
        raise FileExistsError('Both plot and manifest must be new')
    sources = {}
    figure, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, constrained_layout=True)
    for run, analysis, label, color in [
            ('live-left-01', 'left01-analysis', 'Before: completed', '#2679b3'),
            ('live-left-02', 'left02-analysis', 'Before: collision', '#c43e3e'),
            ('live-left-03-after-dropout', 'left03-analysis', 'After: completed', '#25834d')]:
        end_path = args.batch/run/'end.json'
        poses_path = args.batch/analysis/'poses.json'
        end = json.loads(end_path.read_text())
        rows = [row for row in json.loads(poses_path.read_text())
                if 1875. <= row['s'] <= 1930. and row['t'] <= end['shutdown_at']]
        if not rows:
            raise ValueError('No preceding localized poses in '+run)
        for path in [end_path, poses_path]:
            sources[str(path)] = digest(path)
        for axis, values in [(axes[0], [r['d'] for r in rows]),
                             (axes[1], [r['speed_mps']*3.6 for r in rows])]:
            axis.plot([r['s'] for r in rows], values, label=label, color=color, linewidth=1.7)
        if end['reason'] == 'collision_packet_alarm':
            axes[0].plot(rows[-1]['s'], rows[-1]['d'], marker='x', color=color, markersize=10)
    for axis in axes:
        axis.axvline(1905., linestyle='--', color='#666666', label='Intended box station')
        axis.grid(alpha=.25)
    axes[0].set_ylabel('Estimated lateral d (m; +left)')
    axes[1].set_ylabel('Measured speed (km/h)')
    axes[1].set_xlabel('Rear-axle route station s (m)')
    axes[0].legend(loc='lower right', fontsize=9)
    figure.suptitle('Same left-box scene: estimated vehicle response\nNot ground-truth body clearance or identical sensor inputs')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('xb') as stream:
        figure.savefig(stream, format='png', dpi=150)
    plt.close(figure)
    with manifest.open('x') as stream:
        json.dump(dict(scope=__doc__, source_sha256=sources,
                       plot_sha256=digest(args.output)), stream, indent=2)
    print(str(args.output))


if __name__ == '__main__':
    main()
