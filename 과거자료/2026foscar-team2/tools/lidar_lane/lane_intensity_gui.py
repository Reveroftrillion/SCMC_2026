#!/usr/bin/env python3
"""라이다 반사도 차선 실시간 뷰어 — 시뮬 UDP 를 직접 받아 창 하나로 보여준다 (ROS 불필요).

    python3 lane_intensity_gui.py                 # 라이다 9302 + 전방 카메라 9202
    python3 lane_intensity_gui.py --no-camera     # 카메라 창 끄기
    python3 lane_intensity_gui.py --thr 70        # 차선 임계값 초기값

창 구성
    왼쪽 큰 칸   위에서 본 지면 점 (오른쪽이 전방). 회색 = 아스팔트, 초록 = 잔디·연석 등, 빨강 = 차선
    오른쪽 위    전방 카메라 + 라이다 지면 점 투영 (빨강 = 차선으로 분류된 점)
    오른쪽 중간  지면 점 반사도 히스토그램 (log). 점선 = 현재 임계값
    오른쪽 아래  수치: Hz, 점 수, 링별 차선 점, 아스팔트 최대 / 차선 최소 (겹치는지)
    아래 슬라이더  차선 임계값 · 지면 |z| 허용치 · 라이다 장착 높이

키
    space 일시정지    c 반사도 색상맵 ↔ 분류 색 전환    s 지금 화면 + 원본 점 저장 (logs/)    q 종료

주의: 라이다 9302 / 카메라 9202 포트를 다른 프로그램(foscar_lidar lidar_bridge 등)이 이미
받고 있으면 둘 중 하나만 패킷을 받는다. 이 창을 볼 때는 그쪽을 잠시 끄거나 --port 를 바꾸세요.
"""
import argparse
import os
import time
import warnings
from collections import deque

import numpy as np

import matplotlib
matplotlib.use('Qt5Agg')
warnings.filterwarnings('ignore', category=UserWarning)
matplotlib.rcParams['font.family'] = 'Noto Sans CJK JP'   # 한글 포함 (KR 이름은 캐시에 없음)
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

from vlp16_udp import Vlp16Receiver, to_base_link

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, 'logs')

C_ASPH, C_OTHER, C_LANE = '#b8b8b8', '#7ab648', '#d62246'


class LaneGui:

    def __init__(self, a):
        self.a = a
        self.thr = float(a.thr)
        self.z_tol = float(a.z_tol)
        self.lidar_z = float(a.lidar_z)
        self.asphalt_max = int(a.asphalt_max)
        self.paused = False
        self.colormap = False
        self.last_time = None
        self.cloud = None
        self.hist_edges = np.arange(0, 131, 1)
        self.recent = deque(maxlen=30)      # 최근 프레임 반사도 히스토그램 (빈 구간 탐색용)

        self.rx = Vlp16Receiver(port=a.port)
        self.cam = None
        if not a.no_camera:
            try:
                from morai_cam_udp import CameraReceiver, PinholeCamera
                self.cam = CameraReceiver(port=a.cam_port)
                self.pin = PinholeCamera()
            except OSError as exc:
                print(f'카메라 포트 {a.cam_port} 열기 실패 ({exc}) — 카메라 없이 실행')

        self.build()

    # ------------------------------------------------------------------
    def build(self):
        self.fig = plt.figure('라이다 반사도 차선 뷰어', figsize=(17, 9.5))
        self.fig.patch.set_facecolor('#f4f4f4')
        gs = self.fig.add_gridspec(3, 2, width_ratios=[1.55, 1], height_ratios=[1.15, 0.9, 0.95],
                                   left=0.04, right=0.99, top=0.95, bottom=0.14, wspace=0.12, hspace=0.35)
        self.ax_bev = self.fig.add_subplot(gs[:, 0])
        self.ax_cam = self.fig.add_subplot(gs[0, 1])
        self.ax_hist = self.fig.add_subplot(gs[1, 1])
        self.ax_txt = self.fig.add_subplot(gs[2, 1])

        ax = self.ax_bev
        ax.set_facecolor('#ffffff')
        ax.set_xlim(0, self.a.x_max); ax.set_ylim(-self.a.y_abs, self.a.y_abs)
        ax.set_aspect('equal'); ax.grid(alpha=.25)
        ax.set_xlabel('전방 x (m)'); ax.set_ylabel('좌측 y (m)')
        ax.set_title('지면 점 — 회색 아스팔트 · 초록 잔디/연석 · 빨강 차선 (오른쪽이 전방)')
        self.sc_asph = ax.scatter([], [], s=3, c=C_ASPH, label='아스팔트')
        self.sc_other = ax.scatter([], [], s=3, c=C_OTHER, label='기타')
        self.sc_lane = ax.scatter([], [], s=18, c=C_LANE, label='차선', zorder=5)
        self.sc_map = ax.scatter([], [], s=4, c=[], cmap='inferno', vmin=0, vmax=120, visible=False)
        for yy in (-1.75, 1.75):
            ax.axhline(yy, color='#999999', ls=':', lw=0.8)
        ax.plot([0, 0], [-1, 1], color='k', lw=3)           # 자차 위치
        ax.legend(loc='upper right', markerscale=3, fontsize=9)

        ax = self.ax_cam
        ax.axis('off'); ax.set_title('전방 카메라 투영 (빨강 = 차선 점)')
        self.im = ax.imshow(np.zeros((360, 640, 3), np.uint8))
        self.cam_msg = ax.text(0.5, 0.5, '카메라 없음' if self.cam is None else '카메라 대기 중…',
                               ha='center', va='center', transform=ax.transAxes, color='#666')

        ax = self.ax_hist
        ax.set_yscale('log'); ax.set_xlim(0, 130); ax.set_ylim(0.8, 3e4)
        ax.set_xlabel('반사도'); ax.set_ylabel('점 수 (log)'); ax.set_title('지면 점 반사도 분포 (이번 프레임)')
        self.bars = ax.bar(self.hist_edges[:-1] + 0.5, np.ones(len(self.hist_edges) - 1), width=1.0, color='#0b7fab')
        self.thr_line = ax.axvline(self.thr, color=C_LANE, ls='--', lw=1.5)
        self.asph_span = ax.axvspan(0, self.asphalt_max + 0.5, color='#b8b8b8', alpha=.35)
        ax.text(2, 1.2e4, '아스팔트', fontsize=8, color='#555')

        ax = self.ax_txt
        ax.axis('off')
        self.txt = ax.text(0.0, 1.0, '수신 대기 중…', fontsize=10, va='top', ha='left',
                           transform=ax.transAxes, linespacing=1.35)

        # 슬라이더
        sx, sw = 0.10, 0.36
        self.s_thr = Slider(self.fig.add_axes([sx, 0.075, sw, 0.022]), '차선 임계값', 0, 150, valinit=self.thr, valstep=1)
        self.s_ztol = Slider(self.fig.add_axes([sx, 0.045, sw, 0.022]), '지면 |z| 허용 (m)', 0.05, 0.6, valinit=self.z_tol, valstep=0.01)
        self.s_lz = Slider(self.fig.add_axes([sx, 0.015, sw, 0.022]), '라이다 높이 (m)', 1.5, 2.6, valinit=self.lidar_z, valstep=0.01)
        self.s_thr.on_changed(lambda v: setattr(self, 'thr', float(v)))
        self.s_ztol.on_changed(lambda v: setattr(self, 'z_tol', float(v)))
        self.s_lz.on_changed(lambda v: setattr(self, 'lidar_z', float(v)))
        self.fig.text(0.52, 0.045, 'space 일시정지 · c 색상맵/분류 전환 · s 저장 · q 종료', fontsize=9, color='#555')

        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.timer = self.fig.canvas.new_timer(interval=int(1000 / self.a.fps))
        self.timer.add_callback(self.tick)
        self.timer.start()

    # ------------------------------------------------------------------
    def on_key(self, ev):
        if ev.key == ' ':
            self.paused = not self.paused
        elif ev.key == 'c':
            self.colormap = not self.colormap
            self.sc_map.set_visible(self.colormap)
            for s in (self.sc_asph, self.sc_other, self.sc_lane):
                s.set_visible(not self.colormap)
        elif ev.key == 's':
            self.save()
        elif ev.key == 'q':
            plt.close(self.fig)

    def save(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        stem = time.strftime('gui_%m%d_%H%M%S')
        self.fig.savefig(os.path.join(LOG_DIR, stem + '.png'), dpi=100)
        if self.cloud is not None:
            extra = {}
            if self.cam is not None and self.cam.get_jpeg():
                extra['jpeg'] = np.frombuffer(self.cam.get_jpeg(), np.uint8)
            np.savez_compressed(os.path.join(LOG_DIR, stem + '.npz'), rev_0=self.cloud, **extra)
        print(f'저장: logs/{stem}.png / .npz')

    # ------------------------------------------------------------------
    def classify(self, b):
        a = self.a
        m = ((np.abs(b[:, 2]) < self.z_tol) & (b[:, 0] > 0) & (b[:, 0] < a.x_max) & (np.abs(b[:, 1]) < a.y_abs))
        g = b[m]
        i = g[:, 3]
        lane = i >= self.thr
        asph = i <= self.asphalt_max
        other = ~lane & ~asph
        return g, asph, other, lane

    def tick(self):
        if self.paused:
            return
        cloud, t = self.rx.get()
        if cloud is None:
            self.txt.set_text(f'라이다 패킷 없음 (UDP :{self.a.port})\nMORAI 센서 Network Setting 의 Destination Port 와 Connect 상태 확인')
            self.fig.canvas.draw_idle()
            return
        if t == self.last_time:
            return
        self.last_time = t
        self.cloud = cloud
        t0 = time.perf_counter()

        b = to_base_link(cloud, self.a.lidar_x, self.a.lidar_y, self.lidar_z)
        g, asph, other, lane = self.classify(b)

        # BEV
        if self.colormap:
            self.sc_map.set_offsets(g[:, :2]); self.sc_map.set_array(g[:, 3])
        else:
            self.sc_asph.set_offsets(g[asph, :2] if asph.any() else np.zeros((0, 2)))
            self.sc_other.set_offsets(g[other, :2] if other.any() else np.zeros((0, 2)))
            self.sc_lane.set_offsets(g[lane, :2] if lane.any() else np.zeros((0, 2)))

        # 히스토그램
        h, _ = np.histogram(np.clip(g[:, 3], 0, 129.9), bins=self.hist_edges)
        for bar, v in zip(self.bars, h):
            bar.set_height(max(v, 0.5))
        self.thr_line.set_xdata([self.thr, self.thr])
        self.asph_span.set_xy([[0, 0], [0, 1], [self.asphalt_max + .5, 1], [self.asphalt_max + .5, 0], [0, 0]])

        # 카메라
        if self.cam is not None:
            img = self.cam.get_image()
            if img is not None:
                import cv2
                small = cv2.resize(img, (640, 360))
                for m, col, r in [(asph, (200, 200, 200), 1), (other, (60, 200, 60), 1), (lane, (0, 0, 255), 3)]:
                    if m.any():
                        uv = self.pin.project(g[m]); uv = uv[~np.isnan(uv[:, 0])] * 0.5
                        for u, v in uv:
                            cv2.circle(small, (int(u), int(v)), r, col, -1)
                self.im.set_data(small[:, :, ::-1])
                self.cam_msg.set_text('')

        # 수치
        li = g[lane, 3]; oi = g[other, 3]; ai = g[asph, 3]
        # 최근 프레임 누적 히스토그램에서 아스팔트 위쪽의 가장 넓은 빈 구간 = 임계값이 있어야 할 자리
        self.recent.append(np.bincount(np.clip(g[:, 3].astype(int), 0, 255), minlength=256))
        acc = np.sum(self.recent, axis=0)
        top = int(np.max(np.nonzero(acc)[0])) if acc.any() else 0
        runs, start = [], None                      # 빈 구간 목록 (시작, 끝, 폭)
        for v in range(self.asphalt_max + 1, top + 1):
            if acc[v] == 0:
                start = v if start is None else start
            elif start is not None:
                if v - start >= 2:
                    runs.append((start, v - 1, v - start))
                start = None
        thr_i = int(self.thr)
        inside = [r for r in runs if r[0] <= thr_i <= r[1] + 1]
        wide = [r for r in runs if r[2] >= 3]           # 아스팔트·잔디 연속체 위 첫 번째 진짜 빈 구간
        gap = inside[0] if inside else (wide[0] if wide else (runs[0] if runs else None))
        lines = [f'라이다 {self.rx.hz():4.1f} Hz   전체 {len(cloud):5d} 점   지면 {len(g):4d} 점   처리 {1e3 * (time.perf_counter() - t0):.0f} ms',
                 f'아스팔트(≤{self.asphalt_max}) {asph.sum():4d}   기타 {other.sum():4d}   차선(≥{self.thr:.0f}) {lane.sum():3d} 점',
                 '']
        if len(li):
            lines.append(f'이번 프레임  차선 반사도 {li.min():.0f}~{li.max():.0f} (평균 {li.mean():.0f})   '
                         f'기타 최대 {oi.max() if len(oi) else 0:.0f}   아스팔트 최대 {ai.max() if len(ai) else 0:.0f}')
        else:
            lines.append('이번 프레임  차선 점 없음 — 임계값을 낮추거나 장면 확인')
        if gap:
            lines.append(f'최근 {len(self.recent)}프레임  반사도 {gap[0]}~{gap[1]} 에 점 없음 (폭 {gap[2]})  '
                         + ('→ 임계값 OK, 두 집단 완전 분리' if inside else f'→ 임계값을 {gap[0]}~{gap[1] + 1} 로 옮기세요'))
        else:
            lines.append(f'최근 {len(self.recent)}프레임  아스팔트 위쪽에 빈 구간 없음 — 차선과 다른 재질이 겹침')
        lines.append('')
        lines.append('링  거리m  지면점  차선점 | 링  거리m  지면점  차선점')
        cells = []
        for ring in range(16):
            m = g[:, 4] == ring
            if m.sum() < 3:
                continue
            rr = float(np.median(np.hypot(g[m, 0], g[m, 1])))
            cells.append(f'{ring:2d} {rr:6.1f} {m.sum():6d} {int((m & lane).sum()):6d}')
        for i in range(0, len(cells), 2):
            lines.append(' | '.join(cells[i:i + 2]))
        self.txt.set_text('\n'.join(lines))
        self.frames_drawn = getattr(self, 'frames_drawn', 0) + 1
        if self.a.snapshot and self.frames_drawn >= 15:
            self.fig.savefig(self.a.snapshot, dpi=90)
            print('snapshot 저장:', self.a.snapshot)
            self.timer.stop(); plt.close(self.fig)
            return
        self.ax_bev.set_title(f'지면 점 (|z|<{self.z_tol:.2f} m, 라이다 높이 {self.lidar_z:.2f} m) — 오른쪽이 전방'
                              + ('   [일시정지]' if self.paused else ''))
        self.fig.canvas.draw_idle()

    def run(self):
        try:
            plt.show()
        finally:
            self.rx.close()
            if self.cam is not None:
                self.cam.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=9302)
    ap.add_argument('--cam-port', type=int, default=9202)
    ap.add_argument('--no-camera', action='store_true')
    ap.add_argument('--thr', type=float, default=62, help='차선 임계값 초기값')
    ap.add_argument('--asphalt-max', type=int, default=14, help='이 값 이하 = 아스팔트로 칠함')
    ap.add_argument('--z-tol', type=float, default=0.25)
    ap.add_argument('--lidar-x', type=float, default=2.0)
    ap.add_argument('--lidar-y', type=float, default=0.0)
    ap.add_argument('--lidar-z', type=float, default=2.11)
    ap.add_argument('--x-max', type=float, default=45.0)
    ap.add_argument('--y-abs', type=float, default=10.0)
    ap.add_argument('--fps', type=float, default=10.0, help='화면 갱신 상한')
    ap.add_argument('--snapshot', default='', help='이 경로에 화면 한 장 저장하고 종료 (확인용)')
    a = ap.parse_args()
    LaneGui(a).run()


if __name__ == '__main__':
    main()
