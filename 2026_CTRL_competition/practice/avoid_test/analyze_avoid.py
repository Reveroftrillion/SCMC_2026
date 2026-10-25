#!/usr/bin/env python3
"""회피 CSV에서 yaw를 포함한 차체 네 모서리의 차선 이탈과 조향을 계산한다.

python3 analyze_avoid.py ~/foscar_logs/follow_*.csv --plot /tmp/avoid.png
경계는 지도 링크 폭 3.5m를 경로 중심선 양쪽에 적용한 근사치다.
실제 페인트 접촉/충돌 판정은 MORAI 결과와 함께 확인해야 한다.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from foscar_control.core import Route, wrap


def analyze(path, route, start=290., end=365., lane_half=1.75):
    with Path(path).open() as stream:
        rows = [r for r in csv.DictReader(stream)
                if r['state'] == 'CRUISE' and start <= float(r['s_m']) <= end]
    if len(rows) < 2:
        raise ValueError(f'{path}: 주행 데이터 부족')
    data = {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k not in ('state', 'reason')}
    d, lo, hi = [], [], []
    for x, y, yaw, s in zip(data['x'], data['y'], data['yaw_rad'], data['s_m']):
        px, py = route.at(s)
        heading = route.heading_at(s)
        d.append(-(x-px)*math.sin(heading)+(y-py)*math.cos(heading))
        corners = []
        # 후륜축 기준 전방 3.8m, 후방 0.835m, 반폭 0.946m.
        for along in (-.835, 3.8):
            for across in (-.946, .946):
                cx = x+along*math.cos(yaw)-across*math.sin(yaw)
                cy = y+along*math.sin(yaw)+across*math.cos(yaw)
                cs, _, ch = route.project(cx, cy, s)
                bx, by = route.at(cs)
                corners.append(-(cx-bx)*math.sin(ch)+(cy-by)*math.cos(ch))
        lo.append(min(corners)); hi.append(max(corners))
    data.update(d=np.array(d), body_lo=np.array(lo), body_hi=np.array(hi))
    mask = (data['body_lo'] < -lane_half) | (data['body_hi'] > lane_half)
    times = data['elapsed_s']
    intervals, begin = [], None
    for i, active in enumerate(mask):
        if active and begin is None:
            begin = i
        if begin is not None and (not active or i == len(mask)-1):
            finish = i
            intervals.append({'start_s': float(data['s_m'][begin]), 'end_s': float(data['s_m'][finish]),
                              'duration_s': float(times[finish]-times[max(0, begin-1)]),
                              'complete': bool(not active)})
            begin = None
    dt = np.diff(times)
    valid = dt > .001
    steer = np.degrees(data['steering_rad'])
    summary = dict(file=str(path), lane_boundary='route centre ±1.75m (map-width approximation)',
                   intervals=intervals, max_outside_s=max((i['duration_s'] for i in intervals), default=0.),
                   max_steering_deg=float(abs(steer).max()),
                   max_steering_rate_deg_s=float(np.max(abs(np.diff(steer)[valid]/dt[valid]))),
                   speed_kmh=[float(data['speed_mps'].min()*3.6), float(data['speed_mps'].max()*3.6)],
                   max_abs_heading_error_deg=float(max(abs(math.degrees(wrap(y-route.heading_at(s))))
                        for y, s in zip(data['yaw_rad'], data['s_m']))))
    return summary, data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='+', type=Path)
    parser.add_argument('--route', type=Path, default=Path(__file__).resolve().parents[2]/'src/foscar_bringup/routes/global_path.txt')
    parser.add_argument('--plot', type=Path)
    args = parser.parse_args()
    route = Route(np.loadtxt(args.route))
    results = [analyze(p, route) for p in args.csv]
    print(json.dumps([s for s, _ in results], ensure_ascii=False, indent=2))
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        for path, (_, d) in zip(args.csv, results):
            label = path.stem
            axes[0].plot(d['s_m'], d['d'], label=label)
            axes[0].fill_between(d['s_m'], d['body_lo'], d['body_hi'], alpha=.12)
            axes[1].plot(d['s_m'], d['speed_mps']*3.6, label=label)
            axes[2].plot(d['s_m'], np.degrees(d['steering_rad']), label=label)
        for bound in (-1.75, 1.75):
            axes[0].axhline(bound, color='black', ls='--', lw=.8)
        for ax, label in zip(axes, ('Lateral position / body envelope (m)', 'Speed (km/h)', 'Steering command (deg)')):
            ax.set_ylabel(label); ax.grid(alpha=.3)
        axes[0].legend(fontsize=8); axes[-1].set_xlabel('Route distance s (m)')
        fig.tight_layout(); args.plot.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.plot, dpi=150)


if __name__ == '__main__':
    main()
