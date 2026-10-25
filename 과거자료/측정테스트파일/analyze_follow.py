#!/usr/bin/env python3
"""path_follower CSV 로그 분석: 완주 시간, 횡오차, 속도 추종, 구간별 최대 횡오차.

  python3 analyze_follow.py ~/foscar_logs/follow_*.csv          # 여러 파일 비교 표
  python3 analyze_follow.py ~/foscar_logs/follow_X.csv --zones  # 50 m 구간별 횡오차·속도
"""
import argparse
import csv
import sys

import numpy as np

COLUMNS = ['elapsed_s', 'speed_mps', 's_m', 'cross_track_m', 'target_speed_mps', 'accel', 'brake', 'steering_rad']


def load(path):
    rows = [r for r in csv.DictReader(open(path)) if r['state'] == 'CRUISE']
    if not rows:
        return None, None
    data = {k: np.array([float(r[k]) for r in rows]) for k in COLUMNS}
    return data, rows[-1]['reason'] if rows[-1]['state'] == 'CRUISE' else rows[-1]['state']


def summary(path, data, final):
    v = data['speed_mps']*3.6
    e = data['cross_track_m']
    dv = (data['speed_mps']-data['target_speed_mps'])*3.6
    elapsed = data['elapsed_s'][-1]-data['elapsed_s'][0]
    return dict(file=path.split('/')[-1], cruise_min=elapsed/60, s_from=data['s_m'][0], s_to=data['s_m'][-1],
                v_max=v.max(), v_mean=v.mean(), err_max=e.max(), err_p95=np.percentile(e, 95),
                s_err_max=data['s_m'][e.argmax()], dv_rms=np.sqrt(np.mean(dv**2)), dv_max=np.abs(dv).max(),
                brake_time=np.sum(data['brake'] > 0)*0.05, final=final)


def zones(data, width):
    edges = np.arange(0, data['s_m'].max()+width, width)
    print(f'{"s-range":>14} {"v_mean":>7} {"v_min":>6} {"tgt_min":>7} {"err_max":>8} {"err_p95":>8} {"brake_s":>7}')
    for a, b in zip(edges[:-1], edges[1:]):
        m = (data['s_m'] >= a) & (data['s_m'] < b)
        if not m.any():
            continue
        e = data['cross_track_m'][m]
        flag = ' <-- >0.4' if e.max() > 0.4 else ''
        print(f'{a:6.0f}-{b:<6.0f} {data["speed_mps"][m].mean()*3.6:7.1f} {data["speed_mps"][m].min()*3.6:6.1f} '
              f'{data["target_speed_mps"][m].min()*3.6:7.1f} {e.max():8.2f} {np.percentile(e, 95):8.2f} '
              f'{np.sum(data["brake"][m] > 0)*0.05:7.1f}{flag}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+')
    ap.add_argument('--zones', action='store_true', help='구간별 표 (파일 하나)')
    ap.add_argument('--width', type=float, default=50., help='구간 폭 m')
    a = ap.parse_args()
    if a.zones:
        data, _ = load(a.files[0])
        if data is None:
            sys.exit('CRUISE 구간 없음')
        zones(data, a.width)
        return
    print(f'{"file":<34} {"min":>5} {"s":>11} {"v_max":>6} {"v_mean":>6} {"err_max":>7} {"@s":>6} {"err_p95":>7} '
          f'{"dv_rms":>6} {"dv_max":>6} {"brk_s":>6}  final')
    for path in a.files:
        data, final = load(path)
        if data is None:
            print(f'{path.split("/")[-1]:<34} (CRUISE 없음)')
            continue
        r = summary(path, data, final)
        print(f'{r["file"]:<34} {r["cruise_min"]:5.1f} {r["s_from"]:5.0f}-{r["s_to"]:<5.0f} {r["v_max"]:6.1f} '
              f'{r["v_mean"]:6.1f} {r["err_max"]:7.2f} {r["s_err_max"]:6.0f} {r["err_p95"]:7.2f} '
              f'{r["dv_rms"]:6.1f} {r["dv_max"]:6.1f} {r["brake_time"]:6.1f}  {r["final"]}')


if __name__ == '__main__':
    main()
