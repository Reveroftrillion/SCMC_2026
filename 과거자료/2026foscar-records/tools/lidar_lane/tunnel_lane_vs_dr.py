#!/usr/bin/env python3
"""터널 bag 에서 라이다 반사도 우측 실선 거리 vs 추측항법(DR) 횡오차 비교.

    python3 tunnel_lane_vs_dr.py <bag 디렉토리> [--s-from 1870 --s-to 2010]

프레임마다 base_link 지면점 중 반사도 ≥ thr, y ∈ [-2.6, -1.0], x ∈ [5, 25] 를 직선 y = b0 + b1·x 로 피팅해
x=0 에서의 실선 거리(b0)와 선 각도를 구하고, 같은 시각 /localization/odometry 의 경로 기준 횡오차와 비교한다.
차로 폭 3.5 m 이면 실선은 -1.75 에 있어야 하므로  b0 - (-1.75 + 횡오차) 가 0 근처면 DR 이 옳다.
직선 피팅이라 곡선부(선 각도 |b1| > 2°) 는 믿지 말 것.
"""
import argparse
import glob
import math
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64

sys.path.insert(0, '/home/foscar/2026Morai/2026foscar/src/foscar_control')
from foscar_control.core import Route, quaternion_yaw  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--route', default='/home/foscar/2026Morai/2026foscar/src/foscar_bringup/routes/global_path.txt')
    ap.add_argument('--s-from', type=float, default=1870.)
    ap.add_argument('--s-to', type=float, default=2010.)
    ap.add_argument('--thr', type=float, default=62.)
    ap.add_argument('--lane-half', type=float, default=1.75)
    a = ap.parse_args()
    route = Route(np.loadtxt(a.route))
    db = sqlite3.connect(glob.glob(a.bag + '/*.db3')[0]); c = db.cursor()
    topics = {r[1]: r[0] for r in c.execute('select id,name from topics')}
    ss = [(t, deserialize_message(d, Float64).data) for t, d in c.execute(f"select timestamp,data from messages where topic_id={topics['/localization/s']}")]
    st = np.array([t for t, _ in ss]); sv = np.array([v for _, v in ss])
    od = []
    for t, d in c.execute(f"select timestamp,data from messages where topic_id={topics['/localization/odometry']}"):
        m = deserialize_message(d, Odometry); p = m.pose.pose.position; q = m.pose.pose.orientation
        od.append((t, p.x, p.y, quaternion_yaw(q.w, q.x, q.y, q.z), m.pose.covariance[0]))
    od = np.array(od)

    def signed(x, y):
        s, _, h = route.project(x, y); px, py = route.at(s)
        return -math.sin(h)*(x-px)+math.cos(h)*(y-py)

    rows = []
    for t, d in c.execute(f"select timestamp,data from messages where topic_id={topics['/sensors/lidar/points']}"):
        s = float(np.interp(t, st, sv))
        if not (a.s_from <= s <= a.s_to):
            continue
        m = deserialize_message(d, PointCloud2)
        pts = np.frombuffer(bytes(m.data), dtype=np.float32).reshape(-1, m.point_step // 4)[:, :4].copy()
        pts[:, 0] += 2.0; pts[:, 2] += 2.11
        g = pts[(np.abs(pts[:, 2]) < 0.25) & (pts[:, 0] > 5) & (pts[:, 0] < 25) & (pts[:, 1] > -2.6) & (pts[:, 1] < -1.0) & (pts[:, 3] >= a.thr)]
        if len(g) < 6:
            continue
        A = np.c_[np.ones(len(g)), g[:, 0]]; b = np.linalg.lstsq(A, g[:, 1], rcond=None)[0]
        if np.std(g[:, 1]-A@b) > 0.15:
            continue
        k = min(max(np.searchsorted(od[:, 0], t), 0), len(od)-1)
        rows.append((s, b[0], math.degrees(math.atan(b[1])), len(g), signed(od[k, 1], od[k, 2]), od[k, 4]))
    r = np.array(rows)
    print(f'{"s":>6} {"실선 y":>7} {"각도°":>6} {"점":>3} {"추정 횡오차":>9} {"차이":>6} 모드')
    for row in r[::5]:
        print(f'{row[0]:6.0f} {row[1]:7.2f} {row[2]:6.2f} {row[3]:3.0f} {row[4]:9.2f} {row[1]-(-a.lane_half+row[4]):6.2f} {"DR" if row[5] > 0 else "GPS"}')
    straight = r[np.abs(r[:, 2]) < 1.0]
    d = straight[:, 1]-(-a.lane_half+straight[:, 4])
    print(f'\n직선부(|각도|<1°) {len(straight)} 프레임: 실선 y 평균 {straight[:, 1].mean():.2f} (σ {straight[:, 1].std():.3f}),  '
          f'라이다−DR 차이 평균 {d.mean():+.2f} (σ {d.std():.3f})  → σ 가 작고 추세가 없으면 DR 횡드리프트 없음')


if __name__ == '__main__':
    main()
