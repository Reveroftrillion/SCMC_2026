#!/usr/bin/env python3
"""RGB + semantic 짝을 저장하는 수집 노드 (신호등 YOLO 학습용).

MORAI 자유 카메라(9208, 상향 FOV 70°)와 semantic 카메라(9210)를 **같은 pose** 로 달아
두 장을 동시에 받는다. semantic 은 라벨 생성에만 쓰고 추론 경로엔 없다 — 그래서 이 노드는
`ctrl_camera` 가 아니라 이 패키지에 있고, 대회 때는 띄우지 않는다.

짝짓기
------
`message_filters.ApproximateTimeSynchronizer` 로 `header.stamp` 가 `slop_s`(5 ms) 안에 드는
두 장을 묶는다. 이때 스탬프는 **MORAI 시뮬 스탬프**여야 한다 (`camera_bridge` 의
`stamp_source: morai`). 수신 시각으로 맞추면 조각 유실 때 한 프레임씩 밀려 짝지어지는데,
50 ms 밀리면 20 m 거리에서 9 px 오차고 그 라벨은 **조용히 틀린다** (명세서 §3-7).

저장 규칙 (명세서 §3-5)
----------------------
    주행 중   `step_m`(2 m) 이동마다        reason=moved
    정차 중   `interval_s`(2 s) 마다        reason=timed   ← 적→녹 전환을 놓치지 않으려고
    근접 필터 없음 — 정지선 간격이 45~280 m 라 걸러지는 게 없다

`/localization/s` 가 없으면 `timed` 만으로 동작한다. 정지선까지의 거리는 여기서 계산하지
않고 `s` 만 meta 에 남겨 `ctrl_extract.py` 가 오프라인으로 푼다.

받은 JPEG 는 **재압축하지 않고 그대로** 쓴다. 다시 인코딩하면 semantic 의 팔레트 색이
한 번 더 뭉개져서 마스크가 나빠진다.

check 모드
----------
본 수집 전에 반드시 먼저 돈다 (명세서 §7). 저장하지 않고 가정을 한 번에 검증한다 —
두 스트림 Hz 와 짝 성공률, |Δstamp| 분포, semantic 팔레트와 JPEG 손상 비율, 박스마다
**종횡비와 3구/4구 판정**, 그리고 RGB 위에 박스를 겹친 오버레이로 정렬을 눈으로 본다.
2900장 모으고 나서 틀린 걸 발견하면 6번 주행을 다시 해야 한다.

박스는 `extract.label_frame` 으로 뽑는다 — **라벨링과 같은 코드 경로다.** 진단이 따로
구현하면 'check 에선 4구였는데 extract 에선 3구' 같은 어긋남이 생기고, 그걸 알아챌
방법이 없다. 그래서 `min_box_px` 도 `ctrl_extract.py` 와 같은 값을 쓴다.

    ros2 launch ctrl_camera_train ctrl_record.launch.py check:=true drive:=false
"""

import csv
import os
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

import cv2
import message_filters
import numpy as np
import ctrl_ros1 as rclpy
from ctrl_ros1.ctrl_node import Node
from ctrl_ros1.ctrl_qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64

from . import ctrl_extract as ex                 # ROS 를 안 쓰는 모듈이라 노드에서 그대로 쓸 수 있다
from .ctrl_palette import TL_COLOR, aspect_kind, aspect_margin, color_distance

META_HEADER = ('idx', 'morai_sec', 'morai_nsec', 'recv_ns', 'dt_ms', 's', 'reason')


class RecorderNode(Node):
    """RGB + semantic 을 MORAI 타임스탬프로 짝지어 저장. 판단은 최소로."""

    def __init__(self):
        super().__init__('recorder_node')

        p = self.declare_parameter
        self.rgb_topic = p('rgb_topic', '/sensors/camera/tele/compressed').value
        self.sem_topic = p('sem_topic', '/sensors/camera/semantic/compressed').value
        self.s_topic = p('s_topic', '/localization/s').value
        self.tag = str(p('tag', '').value)

        self.step_m = float(p('step_m', 2.0).value)
        self.interval_s = float(p('interval_s', 2.0).value)
        self.slop_s = float(p('slop_s', 0.005).value)
        self.queue_size = int(p('queue_size', 20).value)

        self.check = bool(p('check', False).value)
        self.check_view = bool(p('check_view', True).value)
        self.check_seconds = float(p('check_seconds', 0.0).value)
        # ctrl_extract.py 와 같은 값이어야 진단에서 본 박스가 라벨에서도 나온다
        self.min_box_px = int(p('min_box_px', 12).value)

        # 출력 디렉토리 — check 모드에선 만들지 않는다 (진단은 아무것도 남기지 않는다)
        out_dir = Path(os.path.expanduser(str(p('out_dir', '~/foscar_logs/dataset').value)))
        session = datetime.now().strftime('%Y%m%d_%H%M%S')
        if self.tag:
            session += '_' + self.tag
        self.out = out_dir / session
        self.meta_file = None
        self.meta_writer = None
        if not self.check:
            (self.out / 'rgb').mkdir(parents=True, exist_ok=True)
            (self.out / 'sem').mkdir(parents=True, exist_ok=True)
            self.meta_file = (self.out / 'meta.csv').open('w', newline='')
            self.meta_writer = csv.writer(self.meta_file)
            self.meta_writer.writerow(META_HEADER)

        # --- 입력 ---
        self.rgb_sub = message_filters.Subscriber(
            self, CompressedImage, self.rgb_topic, qos_profile=qos_profile_sensor_data)
        self.sem_sub = message_filters.Subscriber(
            self, CompressedImage, self.sem_topic, qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.sem_sub], self.queue_size, self.slop_s)
        self.sync.registerCallback(self.on_pair)
        # 짝 성공률을 보려면 각 스트림이 몇 장 왔는지도 알아야 한다
        self.rgb_sub.registerCallback(self._count_rgb)
        self.sem_sub.registerCallback(self._count_sem)
        self.create_subscription(Float64, self.s_topic, self.on_s, qos_profile_sensor_data)

        # --- 상태 ---
        self.s = None
        self.idx = 0
        self.last_saved_s = None
        self.last_saved_t = None
        self.rgb_count = self.sem_count = self.pair_count = 0
        self.saved_since_report = 0
        self.dt_sum = 0.0
        self.dt_max = 0.0
        self.dt_n = 0
        self.decode_fail = 0

        # check 모드 누적 (종료할 때 요약)
        self.dt_all = []
        self.palette = Counter()
        self.palette_px = 0
        self.near_exact = Counter()   # TL_COLOR 언저리의 정확한 색 (팔레트 보정용)
        self.near_tl_px = 0
        self.total_px = 0
        self.snap_px = 0          # TL_COLOR 에 '가깝지만 정확히는 아닌' 픽셀 = JPEG 손상 지표
        self.kind_count = Counter()       # 3구 / 4구 / 보행자 / 병합
        self.cls_count = Counter()
        self.box_flag_count = Counter()
        self.box_ratios = []              # (종횡비, w, h, cls, flag)
        self.box_y = []
        self.box_cut = 0                  # 화면 아래로 잘린 박스
        self.box_total = 0
        self.view_failed = False
        self.summarised = False
        self.started = time.monotonic()

        self.create_timer(3.0, self.report)
        mode = 'check (저장 안 함)' if self.check else f'저장 → {self.out}'
        self.get_logger().info(
            f'{self.rgb_topic} + {self.sem_topic} 짝짓기 (slop {self.slop_s * 1000:.0f} ms) | {mode}')
        if not self.check and not self.tag:
            self.get_logger().warn(
                'tag 가 비어 있습니다. 조건 단위 train/val 분할이 이 값을 쓰므로 '
                'tag:=sunny_11h 처럼 넘기세요 (명세서 §5).')

    # ------------------------------------------------------------------
    # 입력
    # ------------------------------------------------------------------

    def _count_rgb(self, _msg) -> None:
        self.rgb_count += 1

    def _count_sem(self, _msg) -> None:
        self.sem_count += 1

    def on_s(self, msg: Float64) -> None:
        """현재 s 보관. localization 이 없으면 계속 None → 시간 기준으로만 저장."""
        self.s = float(msg.data)

    def on_pair(self, rgb: CompressedImage, sem: CompressedImage) -> None:
        """message_filters 콜백. 짝이 맞은 한 쌍이 들어온다."""
        self.pair_count += 1
        dt_ms = abs(stamp_ns(rgb.header.stamp) - stamp_ns(sem.header.stamp)) / 1e6
        self.dt_sum += dt_ms
        self.dt_max = max(self.dt_max, dt_ms)
        self.dt_n += 1

        if self.check:
            self.diagnose(rgb, sem, dt_ms)
            return

        save, reason = self.should_save()
        if save:
            self.save(bytes(rgb.data), bytes(sem.data), rgb.header.stamp, dt_ms, reason)

    # ------------------------------------------------------------------
    # 저장
    # ------------------------------------------------------------------

    def should_save(self):
        """(저장할까, 이유). 이유는 meta 의 reason 열."""
        now = time.monotonic()
        if self.last_saved_t is None:
            return True, 'moved' if self.s is not None else 'timed'
        if self.s is not None and self.last_saved_s is not None \
                and abs(self.s - self.last_saved_s) >= self.step_m:
            return True, 'moved'
        if now - self.last_saved_t >= self.interval_s:
            return True, 'timed'
        return False, ''

    def save(self, rgb: bytes, sem: bytes, stamp, dt_ms: float, reason: str) -> None:
        """받은 JPEG 를 그대로 쓴다 — 재압축하지 않는다. meta.csv 에 한 줄 추가."""
        name = f'{self.idx:06d}.jpg'
        (self.out / 'rgb' / name).write_bytes(rgb)
        (self.out / 'sem' / name).write_bytes(sem)
        self.meta_writer.writerow((
            self.idx, stamp.secs, stamp.nsecs, self.get_clock().now().nanoseconds,
            f'{dt_ms:.3f}', '' if self.s is None else f'{self.s:.3f}', reason))
        self.meta_file.flush()          # 런치를 Ctrl-C 로 끊어도 여기까지는 남는다

        self.idx += 1
        self.saved_since_report += 1
        self.last_saved_t = time.monotonic()
        if self.s is not None:
            self.last_saved_s = self.s

    # ------------------------------------------------------------------
    # check 모드 — 본 수집 전 반드시 먼저 (명세서 §7)
    # ------------------------------------------------------------------

    def diagnose(self, rgb_msg: CompressedImage, sem_msg: CompressedImage, dt_ms: float) -> None:
        self.dt_all.append(dt_ms)
        rgb = decode(rgb_msg)
        sem = decode(sem_msg)
        if rgb is None or sem is None:
            self.decode_fail += 1
            return

        if rgb.shape[:2] != sem.shape[:2]:
            self.get_logger().error(
                f'두 카메라 해상도가 다릅니다: RGB {rgb.shape[1]}×{rgb.shape[0]} vs '
                f'semantic {sem.shape[1]}×{sem.shape[0]}. 같아야 마스크가 RGB 에 맞습니다.',
                throttle_duration_sec=5.0)

        # 팔레트 — 16단위로 뭉쳐 **픽셀 수**를 센다 (JPEG 라 정확히 같은 값이 잘 안 나온다).
        # return_counts 없이 세면 '이 색이 들어 있던 프레임 수' 가 되어 정렬이 무의미해진다.
        px = sem.reshape(-1, 3)
        take = px[::37] // 16 * 16                        # 전수는 느리다. 소수 간격으로 훑는다
        colors, counts = np.unique(take, axis=0, return_counts=True)
        for bgr, n in zip(colors, counts):
            self.palette[tuple(int(v) for v in bgr)] += int(n)
        self.palette_px += len(take)

        # TL_COLOR 근접도 — 팔레트 추정이 맞는지, JPEG 가 얼마나 뭉갰는지
        dist = color_distance(sem, TL_COLOR)
        self.total_px += dist.size
        self.near_tl_px += int((dist <= 60).sum())
        self.snap_px += int(((dist > 10) & (dist <= 60)).sum())

        # TL_COLOR '언저리' 의 정확한 최빈색. 임계(60)보다 넓게 잡아서, 값이 조금 빗나가
        # 박스가 하나도 안 나오는 상황에서도 **진짜 팔레트 값이 드러나게** 한다.
        # 2026-09-22 에 순수 마젠타로 두었다가 색거리 75.7 로 전부 놓친 적이 있다.
        near = sem[(dist > 0) & (dist <= 160)]
        if len(near):
            hit, cnt = np.unique(near[::7], axis=0, return_counts=True)
            for bgr, n in zip(hit, cnt):
                self.near_exact[tuple(int(v) for v in bgr)] += int(n)

        # 라벨링과 **같은 코드 경로**로 박스를 뽑는다. 진단이 따로 구현하면 "check 에선
        # 4구였는데 extract 에선 3구" 같은 어긋남이 생기고, 그걸 알아챌 방법이 없다.
        boxes = ex.label_frame(rgb, sem, self.min_box_px)
        self.collect_boxes(boxes, rgb.shape[0])

        if self.check_view and not self.view_failed:
            self.show_overlay(rgb, sem, boxes, dt_ms)

        if self.check_seconds > 0 and time.monotonic() - self.started >= self.check_seconds:
            self.check_summary()
            raise SystemExit(0)

    def collect_boxes(self, boxes, height: int) -> None:
        """박스 통계 누적. 종횡비 임계와 ROI 를 여기 숫자로 정한다."""
        for b in boxes:
            kind, n_aspect, flag = aspect_kind(b.w, b.h)
            self.kind_count[label_of(kind, n_aspect)] += 1
            if flag:
                self.box_flag_count[flag] += 1
            if b.cls:
                self.cls_count[b.cls] += 1
            self.box_ratios.append((b.w / max(b.h, 1), b.w, b.h, b.cls, flag))
            self.box_y.append(b.y + b.h / 2.0)
            if b.y + b.h >= height - 1:
                self.box_cut += 1
            self.box_total += 1

    def show_overlay(self, rgb, sem, boxes, dt_ms: float) -> None:
        """semantic 박스를 RGB 위에. 정렬·크기·3구/4구 판정을 눈으로 본다.

        글자는 `77x19 r4.05 4구 red` 꼴이다. 색은 초록=판정 확실, 노랑=검수행,
        회색=램프 꺼짐(뒷면·소등). 종횡비 임계를 고칠 때 이 숫자를 그대로 쓰면 된다.
        """
        view = rgb.copy()
        for b in boxes:
            kind, n_aspect, flag = aspect_kind(b.w, b.h)
            color = ((160, 160, 160) if b.cls is None and flag == 'no_lamp'
                     else (0, 220, 255) if flag else (0, 255, 0))
            cv2.rectangle(view, (b.x, b.y), (b.x + b.w, b.y + b.h), color, 1)
            text = f'{b.w}x{b.h} r{b.w / max(b.h, 1):.2f} {label_of(kind, n_aspect)}'
            if b.cls:
                text += f' {b.cls}'
            if flag:
                text += f' [{flag}]'
            cv2.putText(view, text, (b.x, max(10, b.y - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
        cv2.putText(view, f'dt {dt_ms:.1f} ms | s {self.s} | box {len(boxes)} | q=quit s=save',
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        try:
            cv2.imshow('check: semantic over RGB', view)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as exc:                           # headless (DISPLAY 없음)
            self.view_failed = True
            self.get_logger().warn(f'오버레이 창을 못 엽니다 ({exc}). 숫자 진단만 계속합니다.')
            return
        if key == ord('q'):
            self.check_summary()
            raise SystemExit(0)
        if key == ord('s'):
            d = Path(os.path.expanduser('~/foscar_logs/check'))
            d.mkdir(parents=True, exist_ok=True)
            n = len(list(d.glob('rgb_*.png')))
            cv2.imwrite(str(d / f'rgb_{n:03d}.png'), rgb)
            cv2.imwrite(str(d / f'sem_{n:03d}.png'), sem)
            cv2.imwrite(str(d / f'overlay_{n:03d}.png'), view)
            self.get_logger().info(f'저장: {d}/*_{n:03d}.png')

    def check_summary(self) -> None:
        # q 키로 끝내면 여기서 한 번, destroy_node 에서 또 한 번 불린다. 한 번만 찍는다.
        if self.summarised:
            return
        self.summarised = True
        log = self.get_logger().info
        log('=' * 68)
        log('check 요약 (명세서 §7)')
        if self.dt_all:
            a = np.array(self.dt_all)
            log(f'  짝 {len(a)} 쌍 | |dt| 중앙 {np.median(a):.2f} ms  p95 {np.percentile(a, 95):.2f} ms '
                f' 최대 {a.max():.2f} ms')
            if np.median(a) < 1e-6:
                log('  → dt 가 0 입니다. 두 카메라가 같은 시뮬 프레임에 같은 스탬프를 찍고 있습니다.')
            else:
                log('  → dt 가 0 이 아닙니다. camera_bridge 의 stamp_source 가 morai 인지 확인하세요.')
        if self.total_px:
            near = 100 * self.near_tl_px / self.total_px
            artifact = 100 * self.snap_px / max(1, self.near_tl_px)
            log(f'  TL_COLOR{TL_COLOR} 근처 픽셀 {near:.4f} %  (그중 중간색 {artifact:.1f} % = JPEG 손상 지표)')
            if near < 1e-4:
                log('  → 신호등 색이 거의 없습니다. ctrl_palette.py 의 TL_COLOR 를 아래 상위 색으로 바꾸세요.')
        if self.palette:
            log(f'  semantic 상위 색 (BGR, 16단위로 뭉침, 표본 {self.palette_px} px):')
            for bgr, n in self.palette.most_common(10):
                log(f'      {bgr}  {100 * n / max(1, self.palette_px):6.2f} %')
        if self.near_exact:
            log(f'  TL_COLOR 언저리(색거리 160 안)의 정확한 최빈색 — 팔레트가 빗나갔는지 여기서 본다:')
            for bgr, n in self.near_exact.most_common(5):
                d = float(np.sqrt(sum((a - b) ** 2 for a, b in zip(bgr, TL_COLOR))))
                mark = '  ← 지금 값' if d < 1 else (f'  ← 색거리 {d:.1f}' if d > 60 else '')
                log(f'      BGR {bgr}  {n} px{mark}')
            best = self.near_exact.most_common(1)[0][0]
            d = float(np.sqrt(sum((a - b) ** 2 for a, b in zip(best, TL_COLOR))))
            if d > 60:
                log(f'    → ⚠ 최빈색이 TL_COLOR 에서 {d:.1f} 떨어져 있습니다 (임계 60). '
                    f'ctrl_palette.py 의 TL_COLOR 를 {best} 로 바꾸세요.')

        # --- 박스: 종횡비 임계(§9-1)와 ROI 를 이 숫자로 정한다 ---
        if self.box_total:
            log('-' * 68)
            log(f'  박스 {self.box_total} 개')
            log('    구분: ' + (', '.join(f'{k} {v}' for k, v in self.kind_count.most_common()) or '없음'))
            if self.cls_count:
                log('    클래스: ' + ', '.join(f'{k} {v}' for k, v in self.cls_count.most_common()))
            if self.box_flag_count:
                log('    검수행: ' + ', '.join(f'{k} {v}' for k, v in self.box_flag_count.most_common()))

            log('    큰 것부터 (종횡비 임계를 고칠 때 이 값을 쓴다):')
            for r, w, h, cls, flag in sorted(self.box_ratios, key=lambda t: -t[1] * t[2])[:12]:
                kind, n_aspect, _ = aspect_kind(w, h)
                log(f'      {w:3d}x{h:3d}  r {r:5.2f}  ±{aspect_margin(r, max(h, 1)):.2f}  '
                    f'{label_of(kind, n_aspect):>6}  {cls or "-":>10}  {flag or ""}')
            amb = self.box_flag_count.get('aspect_ambiguous', 0)
            if amb > 0.2 * self.box_total:
                log(f'    → aspect_ambiguous 가 {100 * amb / self.box_total:.0f} % 입니다. '
                    f'위 r 값을 보고 ctrl_palette.py 의 ASPECT_BOUNDARY 를 옮기세요.')

            y = np.array(self.box_y)
            log(f'    세로 위치: y {y.min():.0f}~{y.max():.0f}, 중앙값 {np.median(y):.0f}')
            if self.box_cut:
                log(f'    ⚠ 화면 아래로 잘린 박스 {self.box_cut}/{self.box_total} '
                    f'({100 * self.box_cut / self.box_total:.0f} %) — 카메라 pitch 를 내리세요')
            log(f'    → ROI 후보: y {max(0.0, y.min() / 720 - 0.05):.2f} ~ '
                f'{min(1.0, y.max() / 720 + 0.05):.2f} (여유 5 % 포함, 1280×720 기준)')
        log('=' * 68)

    # ------------------------------------------------------------------
    # 로그 (ctrl_camera 관례: 3 초 주기)
    # ------------------------------------------------------------------

    def report(self) -> None:
        if self.pair_count == 0:
            self.get_logger().warn(
                f'짝이 안 맞습니다 — RGB {self.rgb_count / 3.0:.1f} Hz, '
                f'semantic {self.sem_count / 3.0:.1f} Hz. '
                + ('두 브리지가 떠 있는지(9208/9210), ' if self.rgb_count == 0 or self.sem_count == 0
                   else f'slop_s({self.slop_s * 1000:.0f} ms)가 너무 빡빡한지, ')
                + 'stamp_source 가 양쪽 다 morai 인지 확인하세요.')
        else:
            rate = 100.0 * self.pair_count / max(1, min(self.rgb_count, self.sem_count))
            dt_avg = self.dt_sum / max(1, self.dt_n)
            extra = f' | 저장 {self.saved_since_report} (누적 {self.idx})' if not self.check else ''
            if self.decode_fail:
                extra += f' | 디코딩 실패 {self.decode_fail}'
            self.get_logger().info(
                f'RGB {self.rgb_count / 3.0:.1f} Hz | sem {self.sem_count / 3.0:.1f} Hz | '
                f'짝 {self.pair_count / 3.0:.1f} Hz ({rate:.0f} %) | '
                f'dt 평균 {dt_avg:.2f} 최대 {self.dt_max:.2f} ms | '
                f's {"-" if self.s is None else f"{self.s:.1f}"}' + extra)
        self.rgb_count = self.sem_count = self.pair_count = 0
        self.saved_since_report = self.decode_fail = 0
        self.dt_sum = self.dt_max = 0.0
        self.dt_n = 0

    def destroy_node(self):
        if self.check:
            self.check_summary()
        if self.meta_file is not None:
            self.meta_file.close()
            self.get_logger().info(f'{self.idx} 쌍 저장 → {self.out}')
        if self.check_view:
            cv2.destroyAllWindows()
        return super().destroy_node()


def label_of(kind: str, n_aspect) -> str:
    """('vehicle', 4) → '4구'. 로그와 오버레이가 같은 말을 쓰게 한다."""
    if kind == 'vehicle':
        return f'{n_aspect}구' if n_aspect else '차량등'
    return {'pedestrian': '보행자', 'merged': '병합'}.get(kind, kind)


def stamp_ns(stamp) -> int:
    return stamp.secs * 1_000_000_000 + stamp.nsecs


def decode(msg: CompressedImage):
    return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
