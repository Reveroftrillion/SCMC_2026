#!/usr/bin/env python3
"""라이다 뷰어 — 창 하나로 지금 라이다가 뭘 보고 있는지 전부 보여준다.

    ros2 launch ctrl_lidar ctrl_obstacle.launch.py      # 자동으로 같이 뜬다
    ros2 run ctrl_lidar lidar_view                 # 따로 띄우기

한 창에 세 칸이 들어간다.

    위     — 위에서 내려다본 모습. 오른쪽이 차 전방.
             회색 = ROI 밖이거나 지면으로 걸러진 점, 색 = 지면 위 점(높이별 색),
             노란 점선 = ROI, 박스 = /events 로 확정된 객체, 화살표 = 2초 뒤 예상 위치
    아래   — 옆에서 본 모습(전방 거리 × 높이). 위 칸과 가로축이 같다.
             갈색 지면선 위에 점이 딱 깔리면 **장착 높이가 맞는 것**이다.
    오른쪽 — 수신 주기, 단계별 포인트 수, 지면 높이 역산, 파라미터, 객체 목록

RViz 는 점과 박스는 보여주지만 "몇 Hz 로 들어오는지, 지면이 몇 m 로 잡히는지,
왜 안 잡히는지" 를 못 보여준다. 이 창은 그 숫자를 같이 띄우는 게 목적이다.
"""

import threading

import matplotlib
import numpy as np
import ctrl_ros1 as rclpy
from matplotlib.patches import Rectangle
from ctrl_ros1.ctrl_node import Node
from ctrl_ros1.ctrl_qos import qos_profile_sensor_data

from ctrl_interfaces.msg import Event
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from ctrl_ros1 import ctrl_point_cloud2 as pc2

COLOR = {'OBSTACLE': '#e07a00', 'PEDESTRIAN': '#d62246', 'VEHICLE': '#0b7fab', 'DYNAMIC_UNKNOWN': '#bf33ff'}
SHORT = {'OBSTACLE': '정지', 'PEDESTRIAN': '보행자', 'VEHICLE': '차량', 'DYNAMIC_UNKNOWN': '동적미확정'}


class LidarView(Node):

    def __init__(self):
        super().__init__('lidar_view')
        p = self.declare_parameter
        self.lidar_x = p('lidar_x', 2.0).value
        self.lidar_y = p('lidar_y', 0.0).value
        self.lidar_z = p('lidar_z', 2.11).value
        self.roi_x_max = p('roi_x_max', 40.0).value
        self.roi_y_abs = p('roi_y_abs', 6.0).value
        self.roi_z_min = p('roi_z_min', -0.3).value
        self.roi_z_max = p('roi_z_max', 2.5).value
        self.ground_z = p('ground_z', 0.15).value
        self.ground_slope_tol = p('ground_slope_tol', 0.01).value
        # 이 시간 넘게 새 이벤트가 안 오면 화면에서 지운다. 안 그러면 물체를 지나친 뒤에도
        # 마지막 박스가 계속 남는다 (실측: 이벤트 없는 구간이 최장 43초였다).
        self.event_hold_s = p('event_hold_s', 0.5).value
        self.range_x = p('view_range_m', 50.0).value      # 앞으로 몇 m 까지 그릴지
        self.range_y = p('view_width_m', 12.5).value      # 좌우 몇 m 까지 그릴지
        self.save_png = p('save_png', '').value           # 한 장 저장하고 종료

        self.lock = threading.Lock()
        self.cloud = None
        self.raw_z = np.empty(0, np.float32)
        self.stamps = []
        self.total = 0
        self.events = {}
        self.ego_speed = None
        self.ego_stamp = None

        self.create_subscription(PointCloud2, '/sensors/lidar/points',
                                 self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Event, '/events', self.on_event, 10)
        self.create_subscription(Odometry, '/localization/odometry',
                                 self.on_odom, qos_profile_sensor_data)

    def on_cloud(self, msg: PointCloud2) -> None:
        raw = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = raw.reshape(-1, 3) if raw.size else np.empty((0, 3), np.float32)
        with self.lock:
            # 라이다 → base_link (지면 = 0). 이후 모든 그림은 차 기준 좌표다.
            self.cloud = pts + np.array([self.lidar_x, self.lidar_y, self.lidar_z], np.float32)
            self.raw_z = pts[:, 2].copy() if pts.size else np.empty(0, np.float32)
            self.stamps = (self.stamps + [msg.header.stamp.secs
                                          + msg.header.stamp.nsecs * 1e-9])[-20:]
            self.total += 1

    def on_event(self, msg: Event) -> None:
        key = (msg.header.stamp.secs, msg.header.stamp.nsecs)
        with self.lock:
            self.events.setdefault(key, []).append(msg)
            for old in sorted(self.events)[:-1]:     # 최신 프레임만 남긴다
                del self.events[old]

    def on_odom(self, msg: Odometry) -> None:
        t = msg.twist.twist.linear
        with self.lock:
            self.ego_speed = float(np.hypot(t.x, t.y))
            self.ego_stamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9

    def snapshot(self) -> dict:
        """그리기용 스냅샷. 콜백과 그리기가 다른 스레드라 한 번에 복사해 간다."""
        with self.lock:
            if self.cloud is None:
                return {}
            hz = 0.0
            if len(self.stamps) > 1:
                span = self.stamps[-1] - self.stamps[0]
                hz = (len(self.stamps) - 1) / span if span > 1e-6 else 0.0
            evs = []
            if self.events:
                newest = max(self.events)
                age = (self.stamps[-1] if self.stamps else 0.0) - (newest[0] + newest[1]*1e-9)
                if age <= self.event_hold_s:
                    evs = self.events[newest]
                else:
                    self.events.clear()      # 오래됐으면 버린다
            return {'pts': self.cloud.copy(), 'raw_z': self.raw_z.copy(), 'hz': hz,
                    'total': self.total, 'events': list(evs), 'ego': self.ego_speed,
                    'ego_t': self.ego_stamp, 'last_t': self.stamps[-1]}


# ---------------------------------------------------------------------------
# 그리기 (메인 스레드에서만 호출)
# ---------------------------------------------------------------------------

def pick_font() -> str:
    """한글이 네모로 깨지지 않는 폰트를 고른다.

    matplotlib 3.5 는 .ttc 묶음에서 첫 글꼴만 등록해서 'Noto Sans Mono CJK' 는
    보통 못 쓴다. 그래서 정보 패널도 등폭이 아닌 CJK 폰트로 그린다 — 칸이 살짝
    어긋나도 글자가 깨지는 것보다는 낫다.
    """
    from matplotlib import font_manager as fm
    have = {f.name for f in fm.fontManager.ttflist}
    for name in ('Noto Sans Mono CJK KR', 'NanumGothicCoding', 'Noto Sans CJK KR',
                 'Noto Sans CJK JP', 'NanumGothic'):
        if name in have:
            return name
    return matplotlib.rcParams['font.family'][0]


def draw(node: LidarView, ax_top, ax_side, ax_info, font: str) -> None:
    snap = node.snapshot()
    for ax in (ax_top, ax_side, ax_info):
        ax.clear()
    ax_info.axis('off')

    if not snap:
        ax_top.text(0.5, 0.5,
                    '/sensors/lidar/points 수신 없음\n\n'
                    'lidar_bridge 가 떠 있는지,\nMORAI 센서가 Connect 상태인지 확인하세요.',
                    ha='center', va='center', fontsize=14, transform=ax_top.transAxes)
        ax_top.set_xticks([])
        ax_top.set_yticks([])
        return

    pts, evs = snap['pts'], snap['events']
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    in_roi = ((x > 0) & (x < node.roi_x_max) & (np.abs(y) < node.roi_y_abs)
              & (z > node.roi_z_min) & (z < node.roi_z_max))
    ground = np.maximum(node.roi_z_min,
                        node.ground_z + node.ground_slope_tol * np.maximum(x, 0.0))
    above = in_roi & (z > ground)

    # ---------------- 위에서 본 모습 (오른쪽이 전방) ----------------
    ax_top.scatter(x[~in_roi], y[~in_roi], s=0.5, c='#dcdcdc', linewidths=0)
    ax_top.scatter(x[in_roi & ~above], y[in_roi & ~above], s=0.8, c='#a6adb4', linewidths=0)
    if above.any():
        ax_top.scatter(x[above], y[above], s=5.0, c=z[above], cmap='viridis',
                       vmin=node.ground_z, vmax=node.roi_z_max, linewidths=0)
    ax_top.add_patch(Rectangle((0.0, -node.roi_y_abs), node.roi_x_max, 2 * node.roi_y_abs,
                               fill=False, ec='#e8a800', lw=1.5, ls='--'))
    ax_top.plot(0, 0, marker='>', ms=14, color='#2d6a4f', zorder=5)

    for i, ev in enumerate(sorted(evs, key=lambda e: e.x)[:10]):
        c = COLOR.get(ev.type, 'k')
        w, h = max(ev.size_x, 0.4), max(ev.size_y, 0.4)     # x방향 폭, y방향 폭
        ax_top.add_patch(Rectangle((ev.x - w / 2, ev.y - h / 2), w, h,
                                   fill=False, ec=c, lw=2.0, zorder=4))
        speed = float(np.hypot(ev.vx, ev.vy))
        label = SHORT.get(ev.type, ev.type)
        if speed > 0.3:                      # 정지물에 0.0 m/s 를 붙이면 화면만 지저분해진다
            label += f' {speed:.1f}'
        # 가까이 붙은 객체들의 라벨이 겹치지 않게 위/아래로 엇갈려 찍는다
        up = i % 2 == 0
        ax_top.text(ev.x, ev.y + (h / 2 + 0.4 if up else -h / 2 - 0.4), label, color=c,
                    fontsize=8, ha='center', va='bottom' if up else 'top', zorder=5)
        if speed > 0.3:
            ax_top.arrow(ev.x, ev.y, ev.vx * 2.0, ev.vy * 2.0, color=c,
                         head_width=0.5, length_includes_head=True, zorder=4)

    ax_top.set_xlim(-5, node.range_x)
    ax_top.set_ylim(-node.range_y, node.range_y)
    ax_top.set_aspect('equal', adjustable='box')
    ax_top.set_ylabel('← 우   y [m]   좌 →')
    ax_top.set_title('위에서 본 모습  (오른쪽이 전방, 노란 점선 = ROI)', fontsize=11)
    ax_top.grid(alpha=0.25)

    # ---------------- 옆에서 본 모습 ----------------
    ax_side.scatter(x[in_roi & ~above], z[in_roi & ~above], s=2.0, c='#c2c8ce', linewidths=0)
    ax_side.scatter(x[above], z[above], s=4.0, c='#2a9d8f', linewidths=0)
    ax_side.axhline(0.0, color='#8d6e63', lw=1.8)
    gx = np.linspace(0, node.roi_x_max, 50)
    ax_side.plot(gx, node.ground_z + node.ground_slope_tol * gx,
                 color='#e8a800', lw=1.2, ls='--')
    ax_side.axhline(node.roi_z_max, color='#e8a800', lw=1.0, ls=':')
    ax_side.text(node.range_x, 0.0, ' 지면', color='#8d6e63', fontsize=8, va='center')
    ax_side.text(node.range_x, node.ground_z, f' 임계 {node.ground_z:.2f}',
                 color='#b98200', fontsize=8, va='bottom')
    for ev in evs:
        ax_side.axvline(ev.x, color=COLOR.get(ev.type, 'k'), lw=0.8, alpha=0.45)
    ax_side.set_xlim(-5, node.range_x)
    ax_side.set_ylim(node.roi_z_min - 0.4, node.roi_z_max + 0.4)
    ax_side.set_xlabel('전방 x [m]')
    ax_side.set_ylabel('높이 z [m]')
    ax_side.set_title('옆에서 본 모습 — 갈색 선 위에 점이 깔리면 장착 높이가 맞는 것',
                      fontsize=10)
    ax_side.grid(alpha=0.25)

    # ---------------- 정보 패널 ----------------
    ax_info.text(0.0, 1.0, info_text(node, snap, pts, in_roi, above),
                 fontname=font, fontsize=9.5, va='top', ha='left', linespacing=1.45,
                 transform=ax_info.transAxes)


def info_text(node, snap, pts, in_roi, above) -> str:
    lines = ['■ 수신',
             f'   주기 : {snap["hz"]:.1f} Hz   (총 {snap["total"]} 회전)']

    ego, ego_t = snap['ego'], snap['ego_t']
    if ego is None:
        lines += ['   자차 : odom 없음 → 상대속도 모드',
                  '          주행 중 정지물이 VEHICLE 로',
                  '          오분류됩니다']
    elif abs(snap['last_t'] - ego_t) > 0.5:
        lines.append(f'   자차 : odom 끊김 ({snap["last_t"] - ego_t:+.1f} s)')
    else:
        lines.append(f'   자차 : {ego * 3.6:.1f} km/h   ego 보상 ON')

    lines += ['', '■ 포인트',
              f'   전체     : {len(pts)}',
              f'   ROI 안   : {int(in_roi.sum())}',
              f'   지면제거 : {int(above.sum())}  ← 클러스터링 입력']

    rx, ry = pts[:, 0] - node.lidar_x, pts[:, 1] - node.lidar_y
    road = (rx > 8.0) & (rx < 25.0) & (np.abs(ry) < 4.0)
    lines += ['', '■ 장착 위치',
              f'   설정 : x {node.lidar_x:.2f}  y {node.lidar_y:.2f}  z {node.lidar_z:.2f} m']
    if road.sum() >= 50:
        est = -float(np.median(snap['raw_z'][road]))
        if abs(est - node.lidar_z) < 0.10:
            lines.append(f'   실측 : z {est:.2f} m   ✓ 일치')
        else:
            lines += [f'   실측 : z {est:.2f} m   ✗ {abs(est - node.lidar_z):.2f} m 차이',
                      f'          lidar_z:={est:.2f} 로 실행하세요']
    else:
        lines.append('   실측 : 전방 노면 샘플 부족')

    lines += ['', '■ ROI',
              f'   전방 : 0 ~ {node.roi_x_max:.0f} m',
              f'   좌우 : ±{node.roi_y_abs:.0f} m',
              f'   높이 : {node.roi_z_min:+.1f} ~ {node.roi_z_max:.1f} m',
              f'   지면 : z > {node.ground_z:.2f} m 만 사용']

    evs = snap['events']
    lines += ['', f'■ /events   {len(evs)} 건']
    if not evs:
        lines += ['   없음', '   (3프레임 연속 확인 전이거나',
                  '    ROI 안에 물체가 없음)']
    for ev in sorted(evs, key=lambda e: e.x)[:7]:
        speed = float(np.hypot(ev.vx, ev.vy))
        where = f'  s {ev.s:.0f} m' if ev.s >= 0 else '  s 미상'
        lines += [f'   {ev.type}  x {ev.x:.1f}  y {ev.y:+.1f}{where}',
                  f'      {ev.size_x:.1f}×{ev.size_y:.1f} m   {speed:.1f} m/s'
                  f'   신뢰도 {ev.confidence:.2f}']
    return '\n'.join(lines)


def main(args=None):
    rclpy.init(args=args)
    node = LidarView()
    if node.save_png:
        matplotlib.use('Agg')
    font = pick_font()
    matplotlib.rcParams['font.family'] = font
    matplotlib.rcParams['axes.unicode_minus'] = False

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig = plt.figure('FOSCAR 2026 — LiDAR', figsize=(15.0, 9.0))
    gs = fig.add_gridspec(2, 2, width_ratios=[3.2, 1.0], height_ratios=[2.5, 1.0],
                          hspace=0.30, wspace=0.06,
                          left=0.05, right=0.99, top=0.95, bottom=0.07)
    ax_top, ax_info, ax_side = (fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[:, 1]),
                                fig.add_subplot(gs[1, 0]))

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    try:
        if node.save_png:
            # 저장 모드는 단일 스레드로. (spin 스레드와 동시에 돌리면 중복 spin 이 된다)
            for _ in range(80):                       # 첫 클라우드를 기다린다
                if node.snapshot():
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
            for _ in range(30):                       # /events 가 확정될 시간 (3초)
                rclpy.spin_once(node, timeout_sec=0.1)
            draw(node, ax_top, ax_side, ax_info, font)
            fig.savefig(node.save_png, dpi=90)
            print(f'저장: {node.save_png}')
        else:
            spin_thread.start()
            anim = FuncAnimation(fig, lambda _: draw(node, ax_top, ax_side, ax_info, font),
                                 interval=200, cache_frame_data=False)
            fig._foscar_anim = anim          # GC 로 애니메이션이 죽는 것 방지
            plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        # spin 이 빠져나오도록 먼저 shutdown 한 뒤 스레드를 거둔다.
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread.is_alive():
            spin_thread.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()
