#!/usr/bin/env python3
"""주행 bag(/sensors/lidar/points + /localization/s) 에서 구간(s)별 지면 반사도 분리도를 비교.

    python3 lane_intensity_bag.py <bag 디렉토리> [--bins 1820,1881,1998,2060]

각 s 구간에 대해: 아스팔트 최빈값, 아스팔트 위 첫 빈 구간(임계값), 차선 점/프레임, 차선 반사도.
터널(s 1881~1998) 안에서 노면 재질이 바뀌어도 임계값이 유지되는지 보는 용도.
"""
import argparse
import glob
import sqlite3
from collections import Counter

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64

from lane_intensity_test import first_gap


def cloud_xyzi(msg):
    a = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(-1, msg.point_step // 4)
    return a[:, :4]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--bins', default='1820,1881,1998,2060')
    ap.add_argument('--lidar-z', type=float, default=2.11)
    ap.add_argument('--z-tol', type=float, default=0.25)
    ap.add_argument('--asphalt-band', type=int, default=3)
    a = ap.parse_args()
    edges = [float(v) for v in a.bins.split(',')]

    db = sqlite3.connect(glob.glob(a.bag + '/*.db3')[0])
    c = db.cursor()
    topics = {r[1]: r[0] for r in c.execute('select id,name from topics')}
    ss = [(t, deserialize_message(d, Float64).data) for t, d in
          c.execute(f"select timestamp,data from messages where topic_id={topics['/localization/s']}")]
    st = np.array([t for t, _ in ss]); sv = np.array([v for _, v in ss])
    groups = {i: [] for i in range(len(edges) - 1)}
    n = 0
    for t, d in c.execute(f"select timestamp,data from messages where topic_id={topics['/sensors/lidar/points']}"):
        s_ego = float(np.interp(t, st, sv))
        k = np.searchsorted(edges, s_ego) - 1
        if 0 <= k < len(edges) - 1:
            groups[k].append(cloud_xyzi(deserialize_message(d, PointCloud2)))
            n += 1
    print(f'라이다 프레임 {n} 개')
    print(f'{"구간 s":>14} {"프레임":>5} {"지면점/f":>8} {"아스팔트":>6} {"빈구간":>9} {"임계값":>5} {"차선점/f":>8} {"차선반사도":>10} {"기타최대":>7}')
    for k, frames in groups.items():
        if not frames:
            continue
        g = []
        for cl in frames:
            b = cl.copy(); b[:, 0] += 2.0; b[:, 2] += a.lidar_z
            m = (np.abs(b[:, 2]) < a.z_tol) & (b[:, 0] > 3) & (b[:, 0] < 45) & (np.abs(b[:, 1]) < 10)
            g.append(b[m])
        g = np.concatenate(g); inten = g[:, 3]
        mode = int(Counter(inten.astype(int)).most_common(1)[0][0])
        ahi = mode + a.asphalt_band
        gap = first_gap(inten, lo=ahi + 1)
        thr = gap[0] + gap[2] // 2 if gap else None
        lane = inten[inten >= thr] if thr else np.array([])
        other = inten[(inten > ahi) & (inten < (thr or 999))]
        print(f'{edges[k]:6.0f}~{edges[k+1]:6.0f} {len(frames):5d} {len(g)/len(frames):8.0f} {mode:6d} '
              f'{(str(gap[0])+"~"+str(gap[1])) if gap else "없음":>9} {thr if thr else "-":>5} {len(lane)/len(frames):8.1f} '
              f'{(f"{lane.min():.0f}~{lane.max():.0f}") if len(lane) else "-":>10} {other.max() if len(other) else 0:7.0f}')
        vals = Counter(inten.astype(int))
        print('     반사도 분포(상위):', ', '.join(f'{v}:{cnt}' for v, cnt in sorted(vals.items()) if cnt > len(frames)))


if __name__ == '__main__':
    main()
