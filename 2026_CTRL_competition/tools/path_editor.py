#!/usr/bin/env python3
"""전역경로(global_path.txt) 를 마우스로 직접 고치는 편집기.

점 개수는 그대로 두고 점을 옮기기만 한다. 그래서 인덱스 ≈ s 가 유지되고 missions.yaml 등의 s 값이
크게 흔들리지 않는다 (저장 시 전체 길이 변화를 알려 준다). 저장하면 spawn_path.txt 안에 복사돼 있는
전역경로 점들도 같이 바꾼다 (연결로 점은 그대로). install 은 src 로 symlink 라 재빌드는 필요 없다.

조작
  왼쪽 드래그        경로 점을 잡아 끈다. 주변 점은 반경 R [m] (경로 길이 기준) 안에서 코사인으로 따라온다.
  [ / ]              R 을 줄이기 / 늘리기
  n                  커서 주변 R 안의 점을 가장 가까운 차선 중심선(link_set)에 붙인다 (대략 끌어다 놓고 n)
  m                  커서 주변 R 안의 점을 부드럽게 편다 (스무딩)
  u / ctrl+z         되돌리기          r   처음 불러온 경로로 되돌리기
  c                  곡률 색 표시 켜기/끄기
  p / P              다음 / 이전 체크포인트로 화면 이동 (원: 3 m 통과 반경, 화살표: heading)
                     원이 초록이면 경로가 3 m 안을 지나고, 빨강이면 벗어난 것
  오른쪽 드래그      화면 이동
  휠                 커서 기준 확대/축소 (툴바 이동/확대 모드일 때는 편집 안 됨)
  ctrl+s             저장 (routes/backup/ 에 원본 백업 후 덮어씀)
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

# 편집기 단축키와 겹치는 matplotlib 기본 키를 끈다 (p=이동, r/c=뷰 이력, ctrl+s=그림 저장 등)
for _k in ('keymap.pan', 'keymap.home', 'keymap.back', 'keymap.forward', 'keymap.save', 'keymap.xscale', 'keymap.yscale'):
    plt.rcParams[_k] = []
from matplotlib import font_manager as _fm
plt.rcParams['font.family'] = sorted({f.name for f in _fm.fontManager.ttflist
                                      if 'Nanum' in f.name or 'Sans CJK' in f.name}) + ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

ROOT = Path(__file__).resolve().parents[1]
ROUTES = ROOT / 'src/foscar_bringup/routes'
LINK_SET = ROOT / 'src/foscar_bringup/maps/link_set.json'
PICK_PX = 12          # 이 픽셀 안의 점을 잡는다
TIGHT_RADIUS_M = 8.   # 이보다 작은 회전반경은 빨간색으로 경고
CP_RADIUS_M = 3.      # 체크포인트 통과 반경 (규정)


def arc_length(p):
    return np.r_[0., np.cumsum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))]


def turn_radius(p, k=4):
    """각 점의 (i-k, i, i+k) 외접원 반경 [m]. 0.5 m 간격이면 k=4 가 약 2 m 기준선이라 cm 요철에 둔하다."""
    a, b, c = p[:-2 * k, :2], p[k:-k, :2], p[2 * k:, :2]
    ab, bc, ca = (np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1), np.linalg.norm(a - c, axis=1))
    cross = np.abs((b - a)[:, 0] * (c - a)[:, 1] - (b - a)[:, 1] * (c - a)[:, 0])
    with np.errstate(divide='ignore', invalid='ignore'):
        r = np.where(cross > 1e-9, ab * bc * ca / (2 * cross), np.inf)
    return np.r_[np.full(k, np.inf), r, np.full(k, np.inf)]


def read_checkpoints(path):
    """checkpoints.txt 의 '[이름]' 다음 'x, y, z' 줄과 yaw [deg] 줄 → (이름, x, y, yaw)."""
    out, name, xy = [], None, None
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if line.startswith('[') and line.endswith(']') and line != '[체크 포인트]':
            name, xy = line[1:-1], None
        elif name and xy is None and line.count(',') == 2:
            xy = tuple(map(float, line.split(',')[:2]))
        elif name and xy is not None and line:
            try:
                out.append((name, *xy, float(line)))
            except ValueError:
                out.append((name, *xy, float('nan')))
            name = None
    return out


def write_points(path, p):
    """원래 파일 형식을 지킨다: 줄바꿈(global_path 는 CRLF), 마지막 줄바꿈 유무, 숫자 자릿수
    (spawn_path 는 소수 12자리). 값이 안 바뀐 줄은 원래 텍스트를 그대로 둬서 diff 에 수정분만 보인다."""
    path = Path(path)
    old = path.read_bytes().decode() if path.exists() else '\n'
    eol = '\r\n' if '\r\n' in old[:200] else '\n'
    lines = old.splitlines()
    decimals = {len(t.split('.')[1]) if '.' in t else -1 for t in lines[0].split()} if lines else {-1}
    n = decimals.pop() if len(decimals) == 1 else -1
    fmt = (lambda v: f'{v:.{n}f}') if n >= 0 else (lambda v: repr(float(v)))
    same = len(lines) == len(p)
    out = [lines[k] if same and np.allclose(np.array(lines[k].split(), float), row, rtol=0, atol=1e-9)
           else ' '.join(fmt(v) for v in row) for k, row in enumerate(p)]
    path.write_bytes((eol.join(out) + (eol if old.endswith('\n') else '')).encode())


class Editor:
    def __init__(self, a):
        self.path_file, self.spawn_file = Path(a.path), Path(a.spawn)
        self.original = np.loadtxt(self.path_file)
        self.p = self.original.copy()
        self.s0 = arc_length(self.original)
        self.radius = a.radius
        self.history, self.grab, self.pan, self.color_curv, self.dirty = [], None, None, True, False
        self.links = []
        if Path(a.links).exists():
            self.links = [np.asarray(l['points'])[:, :2] for l in json.loads(Path(a.links).read_text())]

        self.fig, self.ax = plt.subplots(figsize=(11, 10))
        self.fig.canvas.manager.set_window_title(f'전역경로 편집기 — {self.path_file.name}')
        self.ax.set_aspect('equal', adjustable='datalim')   # 1:1 비율은 지키되 축은 창을 꽉 채운다
        self.fig.subplots_adjust(left=.05, right=.99, top=.98, bottom=.07)
        if self.links:
            self.ax.add_collection(LineCollection(self.links, colors='0.8', linewidths=.8, zorder=0))
        self.ax.plot(*self.original[:, :2].T, color='0.55', lw=1, ls='--', zorder=1, label='원본')
        self.line = LineCollection([], linewidths=2.2, zorder=2, cmap='RdYlGn', norm=plt.Normalize(0, 30))
        self.ax.add_collection(self.line)
        self.dots, = self.ax.plot([], [], '.', ms=3, color='tab:blue', zorder=3)
        self.window, = self.ax.plot([], [], '-', lw=6, alpha=.25, color='tab:orange', zorder=1.5)
        self.checkpoints = read_checkpoints(a.checkpoints) if Path(a.checkpoints).exists() else []
        self.cp_circles, self.cp_index = [], -1
        for name, x, y, yaw in self.checkpoints:
            c = plt.Circle((x, y), CP_RADIUS_M, alpha=.35, zorder=4)
            self.ax.add_patch(c); self.cp_circles.append(c)
            self.ax.plot(x, y, '+', color='k', ms=8, zorder=5)
            if np.isfinite(yaw):
                self.ax.annotate('', (x + 5 * np.cos(np.radians(yaw)), y + 5 * np.sin(np.radians(yaw))), (x, y),
                                 arrowprops=dict(arrowstyle='->', color='k', lw=1.2), zorder=5)
            label = name if name.startswith('START') else f'CP{name}'
            self.ax.annotate(label, (x, y), fontsize=9, weight='bold', color='tab:purple',
                             xytext=(6, 6), textcoords='offset points', zorder=6)
        for i in range(0, len(self.p), 200):   # 대략 100 m 마다 s 표시
            self.ax.annotate(f's{self.s0[i]:.0f}', self.p[i, :2], fontsize=7, color='0.4')
        self.status = self.fig.text(.01, .01, '', fontsize=9)
        self.ax.legend(loc='upper right')
        self.redraw(fit=True)
        for ev, fn in (('button_press_event', self.on_press), ('motion_notify_event', self.on_move),
                       ('button_release_event', self.on_release), ('key_press_event', self.on_key),
                       ('scroll_event', self.on_scroll)):
            self.fig.canvas.mpl_connect(ev, fn)
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        self.cursor = None

    # ---------- 그리기 ----------
    def redraw(self, fit=False):
        seg = np.stack([self.p[:-1, :2], self.p[1:, :2]], axis=1)
        self.line.set_segments(seg)
        r = turn_radius(self.p)
        if self.color_curv:
            self.line.set_array(np.minimum(r[:-1], r[1:]))
        else:
            self.line.set_array(None); self.line.set_color('tab:blue')
        self.dots.set_data(*self.p[:, :2].T)
        if fit:
            self.ax.set_xlim(self.p[:, 0].min() - 20, self.p[:, 0].max() + 20)
            self.ax.set_ylim(self.p[:, 1].min() - 20, self.p[:, 1].max() + 20)
        s = arc_length(self.p)
        moved = np.linalg.norm(self.p[:, :2] - self.original[:, :2], axis=1)
        gaps = np.diff(s)
        edited = moved > 1e-6
        tight = r[edited].min() if edited.any() else np.inf
        warn = []
        missed = []
        for (name, x, y, _), c in zip(self.checkpoints, self.cp_circles):
            ok = self.cp_distance(x, y) <= CP_RADIUS_M
            c.set_color('tab:green' if ok else 'tab:red')
            if not ok: missed.append(name)
        if missed: warn.append('⚠ 3 m 밖으로 지나는 체크포인트: ' + ', '.join(missed))
        if tight < TIGHT_RADIUS_M: warn.append(f'⚠ 수정 구간 최소 회전반경 {tight:.1f} m')
        if gaps.max() > .75: warn.append(f'⚠ 점 간격 최대 {gaps.max():.2f} m')
        self.status.set_text(
            f'R={self.radius:.0f} m | 수정된 점 {edited.sum()}개, 최대 이동 {moved.max():.2f} m | '
            f'전체 길이 {s[-1]:.1f} m (원본 대비 {s[-1] - self.s0[-1]:+.2f} m) | '
            f'{"저장 안 됨" if self.dirty else "저장됨"}\n' + '   '.join(warn))
        self.fig.canvas.draw_idle()

    def cp_distance(self, x, y):
        """체크포인트에서 경로(선분)까지 최소 거리 [m]."""
        a, b = self.p[:-1, :2], self.p[1:, :2]
        ab = b - a; q = np.array([x, y])
        t = np.clip(((q - a) * ab).sum(1) / np.maximum((ab ** 2).sum(1), 1e-12), 0, 1)
        return float(np.sqrt((((a + t[:, None] * ab) - q) ** 2).sum(1).min()))

    def goto_checkpoint(self, step):
        if not self.checkpoints:
            return
        self.cp_index = (self.cp_index + step) % len(self.checkpoints)
        name, x, y, yaw = self.checkpoints[self.cp_index]
        self.ax.set_xlim(x - 30, x + 30); self.ax.set_ylim(y - 30, y + 30)
        d = self.cp_distance(x, y)
        print(f'{name if name.startswith("START") else "CP" + name}: ({x:.2f}, {y:.2f}) yaw {yaw:.1f}° | '
              f'경로까지 {d:.2f} m {"OK" if d <= CP_RADIUS_M else "⚠ 3 m 초과"}')

    def show_window(self, i):
        if i is None:
            self.window.set_data([], [])
        else:
            m = np.abs(self.s0 - self.s0[i]) < self.radius
            self.window.set_data(*self.p[m, :2].T)

    # ---------- 편집 ----------
    def weights(self, i):
        d = np.abs(self.s0 - self.s0[i]) / self.radius
        return np.where(d < 1, .5 * (1 + np.cos(np.pi * d)), 0.)

    def nearest(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return None
        px = self.ax.transData.transform(self.p[:, :2])
        d = np.hypot(px[:, 0] - event.x, px[:, 1] - event.y)
        i = int(d.argmin())
        return i if d[i] < PICK_PX else None

    def push(self):
        self.history.append(self.p.copy())
        self.history = self.history[-100:]
        self.dirty = True

    def snap_to_lane(self, i):
        if not self.links:
            print('link_set.json 을 찾지 못해 차선 붙이기를 할 수 없습니다.'); return
        w = self.weights(i)
        idx = np.nonzero(w)[0]
        a = np.concatenate([l[:-1] for l in self.links]); b = np.concatenate([l[1:] for l in self.links])
        ab = b - a; L2 = np.maximum((ab ** 2).sum(1), 1e-12)
        self.push()
        for k in idx:
            q = self.p[k, :2]
            t = np.clip(((q - a) * ab).sum(1) / L2, 0, 1)
            proj = a + t[:, None] * ab
            j = np.argmin(((proj - q) ** 2).sum(1))
            self.p[k, :2] = q + w[k] * (proj[j] - q)
        self.relax(w, 3)   # 투영으로 생긴 cm 요철을 다듬는다

    def smooth(self, i):
        self.push()
        self.relax(self.weights(i), 20)

    def relax(self, w, iterations):
        q = self.p[:, :2].copy()
        for _ in range(iterations):
            avg = q.copy(); avg[1:-1] = (q[:-2] + q[1:-1] + q[2:]) / 3
            q = q + w[:, None] * (avg - q)
        self.p[:, :2] = q

    # ---------- 이벤트 ----------
    def editing_allowed(self):
        return not self.fig.canvas.toolbar or not self.fig.canvas.toolbar.mode

    def on_press(self, e):
        if e.button == 3 and e.inaxes == self.ax:   # 오른쪽 드래그 = 화면 이동
            self.pan = (e.x, e.y, self.ax.get_xlim(), self.ax.get_ylim(), self.ax.transData.inverted().frozen())
            return
        if e.button != 1 or not self.editing_allowed():
            return
        i = self.nearest(e)
        if i is None:
            return
        self.push()
        self.grab = (i, np.array([e.xdata, e.ydata]), self.p.copy(), self.weights(i))

    def on_move(self, e):
        if self.pan:
            x, y, xlim, ylim, inv = self.pan
            dx, dy = inv.transform((x, y)) - inv.transform((e.x, e.y))
            self.ax.set_xlim(xlim[0] + dx, xlim[1] + dx); self.ax.set_ylim(ylim[0] + dy, ylim[1] + dy)
            self.fig.canvas.draw_idle()
            return
        self.cursor = self.nearest(e) if self.grab is None else self.grab[0]
        if self.grab and e.inaxes == self.ax and e.xdata is not None:
            i, start, base, w = self.grab
            self.p[:, :2] = base[:, :2] + w[:, None] * (np.array([e.xdata, e.ydata]) - start)
            self.redraw()
        self.show_window(self.cursor)
        self.fig.canvas.draw_idle()

    def on_release(self, e):
        if e.button == 3:
            self.pan = None
        if self.grab:
            self.grab = None
            self.redraw()

    def on_scroll(self, e):
        if e.inaxes != self.ax:
            return
        f = .8 if e.button == 'up' else 1.25
        x0, x1 = self.ax.get_xlim(); y0, y1 = self.ax.get_ylim()
        self.ax.set_xlim(e.xdata + (x0 - e.xdata) * f, e.xdata + (x1 - e.xdata) * f)
        self.ax.set_ylim(e.ydata + (y0 - e.ydata) * f, e.ydata + (y1 - e.ydata) * f)
        self.fig.canvas.draw_idle()

    def on_key(self, e):
        k = e.key
        if k == '[':
            self.radius = max(1., self.radius / 1.25)
        elif k == ']':
            self.radius = min(300., self.radius * 1.25)
        elif k in ('u', 'ctrl+z') and self.history:
            self.p = self.history.pop(); self.dirty = True
        elif k == 'r':
            self.push(); self.p = self.original.copy()
        elif k == 'c':
            self.color_curv = not self.color_curv
        elif k in ('p', 'P'):
            self.goto_checkpoint(1 if k == 'p' else -1)
        elif k in ('n', 'm'):
            i = self.cursor
            if i is None:
                print('커서를 경로 위에 두고 누르세요.'); return
            self.snap_to_lane(i) if k == 'n' else self.smooth(i)
        elif k == 'ctrl+s':
            self.save()
        else:
            return
        self.show_window(self.cursor)
        self.redraw()

    def on_close(self, e):
        if self.dirty:
            print('⚠ 저장하지 않고 닫았습니다. 변경 사항은 버려졌습니다.')

    # ---------- 저장 ----------
    def save(self):
        stamp = time.strftime('%Y%m%d_%H%M%S')
        backup = self.path_file.parent / 'backup'
        backup.mkdir(exist_ok=True)
        on_disk = np.loadtxt(self.path_file)
        (backup / f'{self.path_file.stem}_{stamp}.txt').write_bytes(self.path_file.read_bytes())
        write_points(self.path_file, self.p)
        msg = [f'저장: {self.path_file}', f'백업: {backup}/*_{stamp}.txt']

        if self.spawn_file.exists():
            spawn = np.loadtxt(self.spawn_file)
            (backup / f'{self.spawn_file.stem}_{stamp}.txt').write_bytes(self.spawn_file.read_bytes())
            # spawn 경로 안에서 (저장 전) 전역경로 점과 정확히 같은 점을 찾아 새 좌표로 바꾼다
            n = 0
            for k, q in enumerate(spawn):
                d = np.abs(on_disk[:, :2] - q[:2]).sum(1)
                j = int(d.argmin())
                if d[j] < 1e-6:
                    spawn[k] = self.p[j]; n += 1
            write_points(self.spawn_file, spawn)
            meta = self.spawn_file.with_suffix('.json')
            if meta.exists():
                info = json.loads(meta.read_text())
                (backup / f'{meta.stem}_{stamp}.json').write_text(meta.read_text())
                info['total_length_m'] = float(arc_length(spawn)[-1])
                info['max_gap_m'] = float(np.diff(arc_length(spawn)).max())
                meta.write_text(json.dumps(info, indent=2, ensure_ascii=False) + '\n')
            msg.append(f'spawn 경로 {n}/{len(spawn)} 점 갱신 (나머지는 연결로)')
        self.dirty = False
        s = arc_length(self.p)
        msg.append(f'전체 길이 {s[-1]:.2f} m (처음 대비 {s[-1] - self.s0[-1]:+.2f} m)')
        print('\n'.join(msg))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--path', default=str(ROUTES / 'global_path.txt'))
    ap.add_argument('--spawn', default=str(ROUTES / 'spawn_path.txt'), help='함께 갱신할 spawn 경로 (없으면 건너뜀)')
    ap.add_argument('--checkpoints', default=str(ROUTES / 'checkpoints.txt'))
    ap.add_argument('--links', default=str(LINK_SET), help='배경 차선 중심선 (K-City link_set.json)')
    ap.add_argument('--radius', type=float, default=15., help='드래그 영향 반경 [m]')
    editor = Editor(ap.parse_args())  # 참조를 붙잡아 둬야 한다: mpl_connect 는 bound method 를 약한 참조로 들고 있다
    plt.show()
    return editor


if __name__ == '__main__':
    main()
