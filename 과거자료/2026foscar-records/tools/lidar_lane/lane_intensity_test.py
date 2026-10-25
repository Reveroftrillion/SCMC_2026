#!/usr/bin/env python3
"""라이다 반사도(intensity)로 차선 페인트와 아스팔트가 구분되는지 정량 확인 (ROS 불필요).

    python3 lane_intensity_test.py                    # 시뮬 UDP 에서 30회전 캡처 후 분석
    python3 lane_intensity_test.py --revs 80          # 더 길게 (주행 중이면 여러 장면이 섞임)
    python3 lane_intensity_test.py --load logs/lane_0918_133207.npz   # 저장본 재분석

결과 → logs/lane_<월일_시분초>.{npz,json,png}

방법
  1. 지면 점 = base_link 기준 |z| < z_tol, 전방 x_min~x_max, 좌우 ±y_abs
  2. 반사도 히스토그램에서 아스팔트 위쪽 **첫 번째 빈 구간(폭 3 이상)** 을 찾아 차선 임계값을 자동 결정
     (아스팔트 10~11 / 잔디·연석 ~58 / 차선 페인트 64~110 사이가 비어 있어야 성립)
  3. 링(빔)별·프레임별로 방위각 순으로 훑어 임계값 이상이 연속된 '호(arc)' 를 셈
     → 거리별로 차선 하나를 몇 점이 찍는지, 호 폭이 차선 폭(15 cm)과 맞는지
  4. 카메라 영상에 투영해 육안 대조 (카메라 9202 가 켜져 있을 때)
"""
import argparse
import json
import os
import time
import warnings
from collections import Counter

import numpy as np

from vlp16_udp import Vlp16Receiver, to_base_link

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, 'logs')


# ---------------------------------------------------------------------------
# 캡처
# ---------------------------------------------------------------------------

def capture(revs, port, cam_port):
    frames = []
    rx = Vlp16Receiver(port=port, callback=lambda c: frames.append(c))
    cam = None
    try:
        from morai_cam_udp import CameraReceiver
        cam = CameraReceiver(port=cam_port)
    except OSError:
        pass
    t0 = time.time()
    while len(frames) < revs and time.time() - t0 < revs * 0.5 + 5:
        time.sleep(0.05)
    jpeg = cam.get_jpeg() if cam else None
    hz = rx.hz()
    rx.close()
    if cam:
        cam.close()
    if not frames:
        raise SystemExit(f'UDP :{port} 에서 라이다 패킷을 못 받았습니다. MORAI 센서 Connect 확인.')
    print(f'{len(frames)} 회전 캡처 ({hz:.1f} Hz)' + ('' if jpeg else '  (카메라 없음)'))
    return frames, jpeg


# ---------------------------------------------------------------------------
# 분석
# ---------------------------------------------------------------------------

def ground_mask(b, a):
    return ((np.abs(b[:, 2]) < a.z_tol) & (b[:, 0] > a.x_min) & (b[:, 0] < a.x_max)
            & (np.abs(b[:, 1]) < a.y_abs))


def first_gap(values, lo, min_width=3):
    """히스토그램(1 단위)에서 lo 위쪽으로 올라가며 처음 만나는 폭 min_width 이상 빈 구간 → (시작, 끝, 폭).

    아스팔트(10~11)·잔디(~58) 는 수천 점짜리 연속체라 그 안에 빈 칸이 없고, 그 위 첫 빈 구간이
    곧 '차선 페인트가 시작되는 자리' 다. 페인트(~96)와 화살표(110) 사이 빈 구간이 더 넓어도
    그건 차선/비차선 경계가 아니므로 '가장 넓은 구간' 이 아니라 '첫 구간' 을 쓴다.
    """
    hist = np.bincount(np.clip(values.astype(int), 0, 255), minlength=256)
    top = int(np.max(np.nonzero(hist)[0]))
    start = None
    for v in range(lo, top + 1):
        if hist[v] == 0:
            start = v if start is None else start
        elif start is not None:
            if v - start >= min_width:
                return (start, v - 1, v - start)
            start = None
    return None


def arcs_per_ring(frames_bl, a, thr):
    """프레임별·링별로 임계값 이상 연속 구간(호) 을 모은다."""
    rows = {r: dict(range=[], n_ground=[], arcs=[], lane_pts=[]) for r in range(16)}
    for g in frames_bl:
        for ring in range(16):
            p = g[g[:, 4] == ring]
            if len(p) < 5:
                continue
            p = p[np.argsort(np.arctan2(-p[:, 1], p[:, 0]))]
            above = p[:, 3] >= thr
            rr = rows[ring]
            rr['range'].append(float(np.median(np.hypot(p[:, 0], p[:, 1]))))
            rr['n_ground'].append(len(p))
            rr['lane_pts'].append(int(above.sum()))
            d = np.diff(np.concatenate([[0], above.astype(int), [0]]))
            for s, e in zip(np.where(d == 1)[0], np.where(d == -1)[0]):
                seg = p[s:e]
                width = float(np.hypot(seg[-1, 0] - seg[0, 0], seg[-1, 1] - seg[0, 1])) if e - s > 1 else 0.0
                rr['arcs'].append((e - s, width, float(seg[:, 3].mean())))
    return rows


def analyze(frames, a):
    frames_bl = []
    for c in frames:
        b = to_base_link(c, a.lidar_x, a.lidar_y, a.lidar_z)
        frames_bl.append(b[ground_mask(b, a)])
    g = np.concatenate(frames_bl)
    n = len(frames)
    inten = g[:, 3]
    print(f'지면 점 {len(g)} 개 (프레임당 {len(g) / n:.0f})')

    # 1. 아스팔트 = 최빈값 근처
    mode = int(Counter(inten.astype(int)).most_common(1)[0][0])
    asphalt_hi = mode + a.asphalt_band
    asphalt = inten[inten <= asphalt_hi]
    print(f'\n[1] 아스팔트: 최빈값 {mode}, {mode - a.asphalt_band}~{asphalt_hi} 범위 점 {len(asphalt)} 개 '
          f'({len(asphalt) / len(g) * 100:.1f}%)  값 분포: '
          + ', '.join(f'{v}:{c}' for v, c in sorted(Counter(asphalt.astype(int)).items())))

    # 2. 차선 임계값: 가장 넓은 빈 구간
    gap = first_gap(inten, lo=asphalt_hi + 1)
    if a.lane_thr > 0:
        thr = a.lane_thr
        print(f'\n[2] 차선 임계값 = {thr} (수동 지정)')
    elif gap:
        thr = gap[0] + gap[2] // 2
        print(f'\n[2] 반사도 {gap[0]}~{gap[1]} 구간에 점이 하나도 없음 (폭 {gap[2]}) → 차선 임계값 = {thr}')
    else:
        thr = a.lane_thr_fallback
        print(f'\n[2] 뚜렷한 빈 구간이 없음 → 기본 임계값 {thr} 사용 (분리가 불완전할 수 있음)')
    lane = inten[inten >= thr]
    other = inten[(inten > asphalt_hi) & (inten < thr)]
    print(f'    차선(≥{thr}): {len(lane)} 점, 프레임당 {len(lane) / n:.1f}  반사도 {lane.min() if len(lane) else 0:.0f}~{lane.max() if len(lane) else 0:.0f}, '
          f'평균 {lane.mean() if len(lane) else 0:.1f}')
    print(f'    기타(잔디·연석 등, {asphalt_hi + 1}~{thr - 1}): {len(other)} 점  반사도 최대 {other.max() if len(other) else 0:.0f}')
    contrast = (lane.mean() / max(asphalt.mean(), 1)) if len(lane) else 0
    print(f'    차선 / 아스팔트 평균 비 = {contrast:.1f} 배')
    if len(lane):
        vals = Counter(lane.astype(int))
        print('    차선 반사도 값 분포: ' + ', '.join(f'{v}:{c}' for v, c in sorted(vals.items())))

    # 3. 링별
    rows = arcs_per_ring(frames_bl, a, thr)
    print(f'\n[3] 링별 (거리별) — 임계값 {thr} 이상 연속 구간 = 차선 호(arc)')
    print(f'    {"링":>3} {"거리m":>6} {"지면점/f":>8} {"차선점/f":>8} {"호/f":>5} {"점/호":>6} {"호폭m":>6} {"반사도":>6}')
    ring_rows = []
    for ring in range(16):
        rr = rows[ring]
        if not rr['range']:
            continue
        arcs = np.array(rr['arcs']) if rr['arcs'] else np.zeros((0, 3))
        row = dict(ring=ring, range_m=round(float(np.mean(rr['range'])), 1),
                   ground_per_frame=round(float(np.mean(rr['n_ground'])), 0),
                   lane_pts_per_frame=round(float(np.mean(rr['lane_pts'])), 1),
                   arcs_per_frame=round(len(arcs) / n, 2),
                   pts_per_arc=round(float(arcs[:, 0].mean()), 1) if len(arcs) else 0,
                   arc_width_m=round(float(arcs[:, 1].mean()), 2) if len(arcs) else 0,
                   lane_intensity=round(float(arcs[:, 2].mean()), 0) if len(arcs) else 0)
        ring_rows.append(row)
        print(f"    {ring:3d} {row['range_m']:6.1f} {row['ground_per_frame']:8.0f} {row['lane_pts_per_frame']:8.1f} "
              f"{row['arcs_per_frame']:5.1f} {row['pts_per_arc']:6.1f} {row['arc_width_m']:6.2f} {row['lane_intensity']:6.0f}")

    # 4. 지면 위 물체는 얼마나 밝은가 (차선과 헷갈릴 게 있는지)
    above = []
    for c in frames:
        b = to_base_link(c, a.lidar_x, a.lidar_y, a.lidar_z)
        m = (b[:, 2] >= a.z_tol) & (b[:, 0] > 0) & (b[:, 0] < a.x_max) & (np.abs(b[:, 1]) < a.y_abs)
        above.append(b[m, 3])
    above = np.concatenate(above)
    print(f'\n[4] 지면 위 물체 점 {len(above)} 개: 반사도 p50 {np.percentile(above, 50):.0f} / p90 {np.percentile(above, 90):.0f} '
          f'/ 최대 {above.max():.0f}  → 임계값 {thr} 이상: {(above >= thr).sum()} 점')

    verdict = ('차선 페인트가 아스팔트·잔디와 완전히 분리됨 → 반사도 임계값만으로 차선 점 추출 가능'
               if gap and len(lane) / n >= 10 else
               '분리가 불완전하거나 차선 점이 너무 적음 — 장면/파라미터 확인 필요')
    print(f'\n판정: {verdict}')

    summary = dict(frames=n, ground_points=int(len(g)), asphalt_mode=mode, asphalt_max=asphalt_hi,
                   asphalt_n=int(len(asphalt)), gap=gap, lane_threshold=thr,
                   lane=dict(n=int(len(lane)), per_frame=round(len(lane) / n, 1),
                             min=float(lane.min()) if len(lane) else None, max=float(lane.max()) if len(lane) else None,
                             mean=float(lane.mean()) if len(lane) else None),
                   other=dict(n=int(len(other)), max=float(other.max()) if len(other) else None),
                   contrast=round(contrast, 1), ring_rows=ring_rows,
                   above_ground=dict(n=int(len(above)), max=float(above.max()), over_thr=int((above >= thr).sum())),
                   verdict=verdict, params=vars(a))
    return g, summary


# ---------------------------------------------------------------------------
# 그림
# ---------------------------------------------------------------------------

def plot(g, summary, frames, jpeg, path, a):
    import matplotlib
    matplotlib.use('Agg')
    warnings.filterwarnings('ignore', category=UserWarning)   # CJK 글꼴 tight_layout 경고
    matplotlib.rcParams['font.family'] = 'Noto Sans CJK JP'   # 같은 ttc, 한글 포함. KR 이름은 matplotlib 캐시에 없음
    matplotlib.rcParams['axes.unicode_minus'] = False
    import matplotlib.pyplot as plt

    thr, ahi = summary['lane_threshold'], summary['asphalt_max']
    cls_asph, cls_lane = g[:, 3] <= ahi, g[:, 3] >= thr
    cls_other = ~cls_asph & ~cls_lane

    fig = plt.figure(figsize=(20, 12))
    gs = fig.add_gridspec(2, 3, width_ratios=[1.4, 1, 1])
    fig.suptitle(f'라이다 반사도 차선 분석 — {summary["frames"]} 프레임 · 아스팔트 {summary["asphalt_mode"]} · '
                 f'차선 임계값 {thr} · 차선 {summary["lane"]["per_frame"]} 점/프레임', fontsize=14)

    ax = fig.add_subplot(gs[:, 0])
    ax.scatter(g[cls_asph, 0], g[cls_asph, 1], s=1, c='#bbbbbb', label=f'아스팔트 ≤{ahi}')
    ax.scatter(g[cls_other, 0], g[cls_other, 1], s=1, c='#7ab648', label=f'기타 {ahi + 1}~{thr - 1}')
    ax.scatter(g[cls_lane, 0], g[cls_lane, 1], s=6, c='#d62246', label=f'차선 ≥{thr}')
    ax.set_aspect('equal'); ax.set_xlabel('x 전방 (m)'); ax.set_ylabel('y 좌측+ (m)')
    ax.set_title('지면 점 분류 (BEV, 전 프레임 누적)'); ax.legend(loc='upper right', markerscale=6); ax.grid(alpha=.3)

    ax = fig.add_subplot(gs[0, 1])
    ax.hist(g[:, 3], bins=np.arange(0, 130, 1), color='#0b7fab', log=True)
    ax.axvspan(0, ahi + .5, color='#bbbbbb', alpha=.3)
    ax.axvline(thr, color='#d62246', ls='--', label=f'임계값 {thr}')
    if summary['gap']:
        ax.axvspan(summary['gap'][0] - .5, summary['gap'][1] + .5, color='#d62246', alpha=.12, label='빈 구간')
    ax.set_xlabel('반사도'); ax.set_ylabel('점 수 (log)'); ax.set_title('지면 점 반사도 히스토그램'); ax.legend()

    ax = fig.add_subplot(gs[1, 1])
    rr = summary['ring_rows']
    if rr:
        xs = [r['range_m'] for r in rr]
        ax.bar(xs, [r['lane_pts_per_frame'] for r in rr], width=1.2, color='#d62246', label='차선 점/프레임')
        for r in rr:
            ax.annotate(f"{r['pts_per_arc']:.1f}점/호\n폭{r['arc_width_m']:.2f}", (r['range_m'], r['lane_pts_per_frame']),
                        textcoords='offset points', xytext=(0, 4), ha='center', fontsize=7)
        ax.set_xlabel('링 거리 (m)'); ax.set_ylabel('점'); ax.set_title('거리별 차선 점 수'); ax.legend()

    ax = fig.add_subplot(gs[:, 2]) if jpeg is None else fig.add_subplot(gs[0, 2])
    ax.scatter(np.hypot(g[:, 0], g[:, 1]), g[:, 3], s=1, c=np.where(cls_lane, '#d62246', np.where(cls_asph, '#bbbbbb', '#7ab648')))
    ax.axhline(thr, color='#d62246', ls='--'); ax.set_xlabel('거리 (m)'); ax.set_ylabel('반사도')
    ax.set_title('반사도 vs 거리 (차선 페인트가 거리와 무관하게 일정한지)')

    if jpeg is not None:
        try:
            import cv2
            from morai_cam_udp import PinholeCamera
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            b = to_base_link(frames[-1], a.lidar_x, a.lidar_y, a.lidar_z)
            gg = b[ground_mask(b, a)]
            cam = PinholeCamera()
            for m, col, r in [(gg[:, 3] <= ahi, (200, 200, 200), 1), ((gg[:, 3] > ahi) & (gg[:, 3] < thr), (0, 200, 0), 2),
                              (gg[:, 3] >= thr, (0, 0, 255), 4)]:
                uv = cam.project(gg[m]); uv = uv[~np.isnan(uv[:, 0])]
                for u, v in uv:
                    cv2.circle(img, (int(u), int(v)), r, col, -1)
            ax = fig.add_subplot(gs[1, 2])
            ax.imshow(img[:, :, ::-1]); ax.axis('off'); ax.set_title('마지막 프레임 카메라 투영 (빨강 = 차선 점)')
        except Exception as exc:
            print('카메라 투영 생략:', exc)
    plt.tight_layout()
    fig.savefig(path, dpi=90)
    print(f'그림 저장: {path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=9302)
    ap.add_argument('--cam-port', type=int, default=9202)
    ap.add_argument('--revs', type=int, default=30, help='캡처할 회전 수')
    ap.add_argument('--load', help='저장된 npz 재분석')
    ap.add_argument('--lidar-x', type=float, default=2.0)
    ap.add_argument('--lidar-y', type=float, default=0.0)
    ap.add_argument('--lidar-z', type=float, default=2.11, help='지면 기준 장착 높이')
    ap.add_argument('--x-min', type=float, default=3.0)
    ap.add_argument('--x-max', type=float, default=45.0)
    ap.add_argument('--y-abs', type=float, default=10.0)
    ap.add_argument('--z-tol', type=float, default=0.25, help='지면으로 볼 |z| 허용치')
    ap.add_argument('--asphalt-band', type=int, default=3, help='최빈값 + 이 값까지를 아스팔트로')
    ap.add_argument('--lane-thr', type=int, default=0, help='0 = 자동(빈 구간), 아니면 이 값')
    ap.add_argument('--lane-thr-fallback', type=int, default=62)
    ap.add_argument('--no-plot', action='store_true')
    a = ap.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    jpeg = None
    if a.load:
        d = np.load(a.load)
        frames = [d[k] for k in sorted((k for k in d.files if k.startswith('rev_')), key=lambda s: int(s.split('_')[1]))]
        jpeg = bytes(d['jpeg']) if 'jpeg' in d.files else None
        stem = os.path.splitext(os.path.basename(a.load))[0]
        print(f'{a.load} 에서 {len(frames)} 프레임 로드')
    else:
        frames, jpeg = capture(a.revs, a.port, a.cam_port)
        stem = time.strftime('lane_%m%d_%H%M%S')
        extra = {'jpeg': np.frombuffer(jpeg, np.uint8)} if jpeg else {}
        np.savez_compressed(os.path.join(LOG_DIR, stem + '.npz'), **{f'rev_{i}': f for i, f in enumerate(frames)}, **extra)
        print(f'원본 저장: logs/{stem}.npz')

    g, summary = analyze(frames, a)
    with open(os.path.join(LOG_DIR, stem + '.json'), 'w') as f:
        json.dump(summary, f, ensure_ascii=False, indent=1,
                  default=lambda o: o.item() if hasattr(o, 'item') else str(o))
    if not a.no_plot:
        plot(g, summary, frames, jpeg, os.path.join(LOG_DIR, stem + '.png'), a)


if __name__ == '__main__':
    main()
