#!/usr/bin/env python3
"""수집한 RGB + semantic 짝 → YOLO 라벨. **ROS 도 torch 도 쓰지 않는다.**

    ml/bin/python -m ctrl_camera_train.ctrl_extract --session ~/foscar_logs/dataset/* --out out/

흐름 (명세서 §3-2, §3-3)
-----------------------
    semantic 팔레트 마스크 → 연결요소 = 함체 박스 (픽셀 단위로 정확)
    → 박스 안 RGB 를 HSV 로 읽어 켜진 램프 찾기
    → 램프의 정규화 x 위치를 한국 신호등 고정 순서와 대조해 클래스 확정
    → 라벨 서명으로 중복 제거 → (조건, 정지선) 단위 train/val 분할

**박스 안에는 배경이 없다.** 화면 전체를 뒤지는 `traffic_light_node` 와 달리 함체 안엔
켜진 램프 말고 채도 높은 게 없으므로 HSV 임계를 크게 낮춰도 오탐이 안 난다. 그래서
`traffic_light.yaml` 의 `s_lo=100, v_lo=95` 대신 `60, 60` 에서 출발한다.

검수행으로 보내는 것 (자동 라벨이 틀렸을 만한 것)
    no_lamp            램프가 안 켜짐 → 뒷면이거나 소등. 데이터셋에서 제외
    pos_mismatch       색과 위치가 어긋남 → 팔레트나 LAMP_X 를 의심
    aspect_ambiguous   삼색/사색 경계 → 위치 검증 기준이 흔들린다
    merged             갠트리 여러 개가 한 덩어리 → 사람이 쪼개야 한다

출력
    out/dataset/   images/{train,val}, labels/{train,val}, data.yaml
    out/review/    flag 붙은 프레임만, 박스를 그려서
    out/frames.json  Frame 목록 — ctrl_roboflow_io.py 가 이걸 읽어 업로드본을 만든다
"""

import argparse
import csv
import json
import math
import re
import sys
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .ctrl_palette import CLASSES, COLOR_TOLERANCE, LAMP_X, LAMP_X_TOLERANCE, ROI, TL_COLOR, aspect_kind, color_distance, roi_box

#: 박스 안 판정용 HSV 범위 [(lo, hi), ...]. OpenCV H 는 0~179.
#: traffic_light.yaml 의 값에서 s_lo/v_lo 를 100/95 → 60/60 으로 낮췄다 (위 독스트링).
HSV = {
    'red':    [((0, 60, 60), (12, 255, 255)), ((160, 60, 60), (179, 255, 255))],
    'yellow': [((15, 60, 60), (35, 255, 255))],
    'green':  [((40, 60, 60), (95, 255, 255))],
}

#: LAMP_X 의 칸 이름 → 그 칸에 켜지는 색. 사색등의 '좌회전' 칸은 초록색이다.
SLOT_COLOR = {'red': 'red', 'yellow': 'yellow', 'left': 'green', 'green': 'green'}

#: 라벨 서명의 크기 구간 [px]. 이 경계를 넘으면 "모델이 배울 내용"이 달라진 것으로 본다.
SIZE_EDGES = (8, 12, 20, 40)


@dataclass
class Box:
    x: int
    y: int
    w: int
    h: int
    cls: Optional[str] = None       # CLASSES 중 하나. None 이면 데이터셋에서 제외
    flag: Optional[str] = None      # 검수 사유

    @property
    def cx(self) -> float:
        return self.x + self.w / 2.0

    @property
    def cy(self) -> float:
        return self.y + self.h / 2.0


@dataclass
class Frame:
    idx: int
    rgb: str
    sem: str
    s: Optional[float]
    tag: str
    boxes: List[Box] = field(default_factory=list)

    @property
    def flagged(self) -> bool:
        return any(b.flag for b in self.boxes)

    @property
    def labelled(self) -> List[Box]:
        return [b for b in self.boxes if b.cls is not None]

    @property
    def has_merged(self) -> bool:
        return any(b.flag == 'merged' for b in self.boxes)


# ---------------------------------------------------------------------------
# semantic → 박스
# ---------------------------------------------------------------------------

def semantic_mask(sem_bgr: np.ndarray, tl_color=TL_COLOR, tol=COLOR_TOLERANCE) -> np.ndarray:
    """팔레트 색에 스냅 → 신호등 클래스 이진 마스크 (uint8 0/255).

    JPEG 아티팩트 때문에 정확히 일치하는 픽셀만 보면 안 된다 — 20 px 물체는 JPEG 블록
    3개 크기라 경계가 통째로 중간색이 된다 (명세서 §7). 색거리 임계로 판정한다.
    """
    return ((color_distance(sem_bgr, tl_color) <= tol).astype(np.uint8)) * 255


def find_housings(mask: np.ndarray, min_area_px: int = 12) -> List[Box]:
    """연결요소 → 최소 크기 필터 → Box 목록. 좌표는 원본 프레임 기준."""
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    boxes = []
    for i in range(1, n):                                   # 0 은 배경
        x, y, w, h, area = (int(v) for v in stats[i])
        if area < min_area_px:
            continue
        boxes.append(Box(x, y, w, h))
    return sorted(boxes, key=lambda b: (b.y, b.x))


def classify_kind(box: Box) -> Tuple[str, Optional[int], Optional[str]]:
    """종횡비로 'vehicle' | 'pedestrian' | 'merged'. 경계는 검수행 (palette.aspect_kind)."""
    return aspect_kind(box.w, box.h)


# ---------------------------------------------------------------------------
# 박스 안 상태 판정
# ---------------------------------------------------------------------------

def lamps_in_box(rgb: np.ndarray, box: Box, min_lamp_area_px: int = 2
                 ) -> List[Tuple[str, float, int]]:
    """박스 안 HSV → [(색, 정규화 x 위치, 넓이)].

    배경이 없으므로 traffic_light.yaml 의 임계값보다 크게 완화해서 쓴다.
    """
    h, w = rgb.shape[:2]
    x0, y0 = max(0, box.x), max(0, box.y)
    x1, y1 = min(w, box.x + box.w), min(h, box.y + box.h)
    if x1 <= x0 or y1 <= y0:
        return []
    crop = rgb[y0:y1, x0:x1]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    width = float(x1 - x0)

    lamps = []
    for color, ranges in HSV.items():
        mask = None
        for lo, hi in ranges:
            m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        n, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for i in range(1, n):
            area = int(stats[i][4])
            if area < min_lamp_area_px:
                continue
            lamps.append((color, float(centroids[i][0]) / width, area))
    return sorted(lamps, key=lambda l: -l[2])


def classify_state(lamps: Sequence[Tuple[str, float, int]], n_aspect: Optional[int]
                   ) -> Tuple[Optional[str], Optional[str]]:
    """(클래스, 검수사유).

    램프가 없으면 (None, 'no_lamp') → 뒷면·소등, 데이터셋에서 제외.
    색과 LAMP_X 위치가 어긋나면 (추정클래스, 'pos_mismatch') → 검수행.
    초록+좌회전이 동시면 'green_left'.
    """
    if not lamps:
        return None, 'no_lamp'
    if n_aspect is None:                        # merged — 사람이 쪼개기 전엔 클래스가 없다
        return None, 'merged'

    slots = LAMP_X[n_aspect]
    area_by_slot, mismatch = {}, False
    for color, nx, area in lamps:
        candidates = [s for s in slots if SLOT_COLOR[s] == color]
        if not candidates:                      # 이 등에 없는 색 (삼색등의 좌회전 등)
            mismatch = True
            continue
        best = min(candidates, key=lambda s: abs(nx - slots[s]))
        if abs(nx - slots[best]) > LAMP_X_TOLERANCE:
            mismatch = True
        area_by_slot[best] = area_by_slot.get(best, 0) + area

    if not area_by_slot:
        return None, 'pos_mismatch'
    flag = 'pos_mismatch' if mismatch else None
    if 'left' in area_by_slot and 'green' in area_by_slot:
        return 'green_left', flag
    return max(area_by_slot, key=area_by_slot.get), flag


def label_frame(rgb: np.ndarray, sem: np.ndarray, min_box_px: int = 12,
                min_lamp_area_px: int = 2, tl_color=TL_COLOR) -> List[Box]:
    """한 쌍 → 라벨이 붙은 Box 목록."""
    boxes = find_housings(semantic_mask(sem, tl_color), min_box_px)
    for box in boxes:
        kind, n_aspect, flag = classify_kind(box)
        lamps = lamps_in_box(rgb, box, min_lamp_area_px)
        if kind == 'merged':
            box.cls, box.flag = None, 'merged'
            continue
        if kind == 'pedestrian':
            # 보행자등도 램프가 꺼져 있으면 뒷면이다. 색은 안 쓰고 존재만 본다.
            box.cls = 'pedestrian' if lamps else None
            box.flag = flag or (None if lamps else 'no_lamp')
            continue
        cls, state_flag = classify_state(lamps, n_aspect)
        box.cls = cls
        box.flag = state_flag or flag
    return boxes


# ---------------------------------------------------------------------------
# 프레임 이름 — dataset/ 과 upload/ 가 같은 규약을 쓴다
# ---------------------------------------------------------------------------

NAME_RE = re.compile(r'^(?P<tag>.+)_(?P<idx>\d{6})_s(?P<s>NA|m?\d+)$')


def frame_stem(frame: 'Frame') -> str:
    """'sunny_11h_000123_s004123'. tag·idx·s 를 이름에 박는다.

    **점을 쓰지 않는다** — Roboflow 왕복에서 소수점이 확장자로 오해받을 수 있다.
    s 는 데시미터 정수, 음수(연결로 위 정지선)는 'm' 을 앞에 붙인다.
    이름에 s 가 들어 있어야 거리 구간별 평가(§6)와 train/val 누수 점검이 된다.
    """
    if frame.s is None:
        s = 'NA'
    else:
        s = ('m' if frame.s < 0 else '') + f'{int(round(abs(frame.s) * 10)):06d}'
    return f'{frame.tag or "untagged"}_{frame.idx:06d}_s{s}'


def parse_frame_stem(name: str) -> Tuple[str, int, Optional[float]]:
    """(tag, idx, s). Roboflow 가 붙인 '.rf.<hash>' 와 '_jpg' 는 떼고 읽는다."""
    stem = Path(name).name.split('.rf.')[0]
    stem = re.sub(r'\.(jpe?g|png|txt)$', '', stem, flags=re.I)
    stem = re.sub(r'_jpe?g$', '', stem, flags=re.I)
    m = NAME_RE.match(stem)
    if not m:
        raise ValueError(f'프레임 이름 규약에 안 맞습니다: {name!r}')
    raw = m.group('s')
    s = None if raw == 'NA' else (-1 if raw.startswith('m') else 1) * int(raw.lstrip('m')) / 10.0
    return m.group('tag'), int(m.group('idx')), s


# ---------------------------------------------------------------------------
# 중복 제거 · 분할
# ---------------------------------------------------------------------------

def size_bin(box: Box, edges=SIZE_EDGES) -> int:
    return int(np.searchsorted(edges, box.h))


def label_signature(boxes: Sequence[Box], grid: int = 16) -> tuple:
    """정렬된 (중심 격자화, 크기 구간, 클래스). 직전 채택 프레임과 같으면 중복."""
    return tuple(sorted(
        (int(b.cx) // grid, int(b.cy) // grid, size_bin(b), b.cls)
        for b in boxes if b.cls is not None))


def dedup(frames: Sequence[Frame], grid: int = 16) -> List[Frame]:
    """직전에 남긴 프레임과 라벨 서명이 같으면 버린다 (명세서 §3-6).

    빨간불 정차 구간은 프레임이 사실상 동일해서 통째로 날아가지만, 신호가 바뀌는
    **전환 순간은 서명이 달라지므로 남는다** — 거기가 가장 값진 데이터다.
    """
    out, last = [], object()
    for f in frames:
        sig = label_signature(f.boxes, grid)
        if sig == last:
            continue
        out.append(f)
        last = sig
    return out


def group_key(frame: Frame, lines: Sequence[float]) -> tuple:
    from .ctrl_stoplines import nearest_index
    return (frame.tag, nearest_index(frame.s, list(lines)))


def split_by_group(frames: Sequence[Frame], lines: Sequence[float], val_ratio: float = 0.2,
                   echo=print) -> Tuple[List[Frame], List[Frame]]:
    """(조건 태그, 정지선) 단위로 통째로 나눈다.

    무작위 분할 금지 — 연속 프레임이 train/val 에 쌍둥이로 들어가 mAP 가 0.99 로 나오고
    실주행에서 무너진다 (명세서 §5).

    그룹 수가 적으면 나눌 수가 없다. 세션 하나만 넣으면 그룹이 하나뿐이라 val 이 비는데,
    **train 을 비우는 것보다 val 을 비우는 편이 낫다** — 빈 train 으로 학습을 걸면 ultralytics
    가 한참 뒤에야 죽고, 그때까진 뭐가 잘못됐는지 안 보인다.
    """
    groups = {}
    for f in frames:
        groups.setdefault(group_key(f, lines), []).append(f)
    keys = sorted(groups, key=lambda k: (str(k[0]), -1 if k[1] is None else k[1]))
    if not keys:
        return [], []

    n_val = max(1, int(round(len(keys) * val_ratio))) if val_ratio > 0 else 0
    n_val = min(n_val, len(keys) - 1)              # train 을 절대 비우지 않는다
    if val_ratio > 0 and n_val == 0:
        echo(f'⚠ (조건, 정지선) 그룹이 {len(keys)} 개뿐이라 val 을 못 나눴습니다. '
             f'세션을 여러 조건으로 모으거나 --val-ratio 0 으로 명시하세요.')
    # 고르게 집는다 — 앞에서 잘라내면 특정 조건이 통째로 val 로 간다
    step = len(keys) / float(n_val) if n_val else 0
    val_keys = {keys[min(len(keys) - 1, int(i * step))] for i in range(n_val)}
    train = [f for k in keys if k not in val_keys for f in groups[k]]
    val = [f for k in keys if k in val_keys for f in groups[k]]
    return train, val


# ---------------------------------------------------------------------------
# 출력
# ---------------------------------------------------------------------------

def crop_roi(img: np.ndarray, boxes: Sequence[Box], roi=ROI) -> Tuple[np.ndarray, List[Box]]:
    """ROI 크롭 + 박스 좌표 이동. **올리는 것 = 학습하는 것.**

    ROI 밖으로 완전히 벗어난 박스는 버리고, 걸친 박스는 잘라 맞춘다.
    """
    h, w = img.shape[:2]
    x0, y0, x1, y1 = roi_box(w, h, roi)
    out = []
    for b in boxes:
        nx0, ny0 = max(b.x, x0), max(b.y, y0)
        nx1, ny1 = min(b.x + b.w, x1), min(b.y + b.h, y1)
        if nx1 <= nx0 or ny1 <= ny0:
            continue
        out.append(Box(nx0 - x0, ny0 - y0, nx1 - nx0, ny1 - ny0, b.cls, b.flag))
    return img[y0:y1, x0:x1], out


def yolo_lines(boxes: Sequence[Box], width: int, height: int, names: Sequence[str]) -> List[str]:
    """YOLO txt 한 줄씩: `class_id cx cy w h` (0~1 정규화)."""
    lines = []
    for b in boxes:
        if b.cls is None or b.cls not in names:
            continue
        lines.append(f'{names.index(b.cls)} {b.cx / width:.6f} {b.cy / height:.6f} '
                     f'{b.w / width:.6f} {b.h / height:.6f}')
    return lines


def write_yolo(train: Sequence[Frame], val: Sequence[Frame], out: Path, roi=ROI,
               names: Sequence[str] = CLASSES, skip_merged: bool = True) -> Path:
    """images/ labels/ data.yaml. 반환값은 data.yaml 경로."""
    out.mkdir(parents=True, exist_ok=True)
    for split, frames in (('train', train), ('val', val)):
        (out / 'images' / split).mkdir(parents=True, exist_ok=True)
        (out / 'labels' / split).mkdir(parents=True, exist_ok=True)
        for f in frames:
            if skip_merged and f.has_merged:
                continue                        # 병합 박스를 지우면 진짜 신호등이 배경이 된다
            img = cv2.imread(f.rgb)
            if img is None:
                continue
            img, boxes = crop_roi(img, f.boxes, roi)
            h, w = img.shape[:2]
            stem = frame_stem(f)
            cv2.imwrite(str(out / 'images' / split / f'{stem}.jpg'), img,
                        [cv2.IMWRITE_JPEG_QUALITY, 95])
            (out / 'labels' / split / f'{stem}.txt').write_text(
                '\n'.join(yolo_lines(boxes, w, h, names)) + '\n')

    data = out / 'data.yaml'
    data.write_text(
        f'path: {out.resolve()}\n'
        f'train: images/train\n'
        f'val: images/val\n'
        f'nc: {len(names)}\n'
        f'names: [{", ".join(names)}]\n')
    # imgsz 는 ROI 긴 변에 맞춘다 (명세서 §5). ctrl_train.py 가 이 파일을 읽는다.
    probe = next((f for f in list(train) + list(val)), None)
    if probe is not None:
        img = cv2.imread(probe.rgb)
        if img is not None:
            x0, y0, x1, y1 = roi_box(img.shape[1], img.shape[0], roi)
            (out / 'roi.json').write_text(json.dumps(
                {'roi': list(roi), 'crop_px': [x0, y0, x1, y1],
                 'width': x1 - x0, 'height': y1 - y0,
                 'imgsz': int(np.ceil(max(x1 - x0, y1 - y0) / 32) * 32)}, indent=2))
    return data


DRAW = {'red': (0, 0, 255), 'yellow': (0, 255, 255), 'green': (0, 255, 0),
        'green_left': (0, 200, 120), 'left': (255, 128, 0), 'pedestrian': (200, 120, 255)}


def write_review(frames: Sequence[Frame], out: Path, roi=ROI) -> int:
    """flag 붙은 것만, 박스를 그려서. 사람이 30분 안에 훑을 수 있게."""
    out.mkdir(parents=True, exist_ok=True)
    n = 0
    for f in frames:
        if not f.flagged:
            continue
        img = cv2.imread(f.rgb)
        if img is None:
            continue
        img, boxes = crop_roi(img, f.boxes, roi)
        for b in boxes:
            cv2.rectangle(img, (b.x, b.y), (b.x + b.w, b.y + b.h),
                          DRAW.get(b.cls, (160, 160, 160)), 1)
            label = f'{b.cls or "?"}'
            if b.flag:
                label += f' [{b.flag}]'
            cv2.putText(img, label, (b.x, max(10, b.y - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
        cv2.imwrite(str(out / f'{frame_stem(f)}.jpg'), img)
        n += 1
    return n


def stats(frames: Sequence[Frame], echo=print) -> dict:
    """클래스 분포, 박스 크기 분포, 조건별 수, 검수행 비율."""
    cls_count, flag_count, tag_count = {}, {}, {}
    heights = []
    for f in frames:
        tag_count[f.tag] = tag_count.get(f.tag, 0) + 1
        for b in f.boxes:
            if b.cls:
                cls_count[b.cls] = cls_count.get(b.cls, 0) + 1
                heights.append(b.h)
            if b.flag:
                flag_count[b.flag] = flag_count.get(b.flag, 0) + 1
    echo(f'프레임 {len(frames)} | 라벨 박스 {sum(cls_count.values())}')
    echo('  클래스: ' + (', '.join(f'{k} {v}' for k, v in sorted(cls_count.items())) or '없음'))
    echo('  조건:   ' + (', '.join(f'{k or "untagged"} {v}' for k, v in sorted(tag_count.items())) or '없음'))
    if heights:
        a = np.array(heights)
        edges = [0, *SIZE_EDGES, 10 ** 6]
        dist = ', '.join(f'{edges[i]}~{edges[i + 1]}px {int(((a >= edges[i]) & (a < edges[i + 1])).sum())}'
                         for i in range(len(edges) - 1))
        echo(f'  박스 높이: 중앙 {np.median(a):.0f} px | {dist}')
    n_flagged = sum(1 for f in frames if f.flagged)
    echo(f'  검수행: 프레임 {n_flagged} ({100 * n_flagged / max(1, len(frames)):.1f} %)'
         + (' | ' + ', '.join(f'{k} {v}' for k, v in sorted(flag_count.items())) if flag_count else ''))
    if flag_count.get('no_lamp'):
        echo('    no_lamp 이 많으면 뒷면 신호등이 많은 것이거나, 박스내 HSV 임계가 아직 높은 것입니다.')
    if n_flagged > 0.25 * max(1, len(frames)):
        echo('    ⚠ 검수행이 25 % 를 넘습니다. ctrl_palette.py 의 TL_COLOR 부터 의심하세요.')
    return {'classes': cls_count, 'flags': flag_count, 'tags': tag_count}


# ---------------------------------------------------------------------------
# 세션 읽기
# ---------------------------------------------------------------------------

def read_session(session: Path, min_box_px=12, min_lamp_area_px=2, tl_color=TL_COLOR,
                 limit=None, echo=print) -> List[Frame]:
    """`recorder_node` 가 만든 디렉토리 하나 → Frame 목록."""
    meta = session / 'meta.csv'
    if not meta.exists():
        echo(f'  건너뜀 (meta.csv 없음): {session}')
        return []
    # 디렉토리 이름 20260920_113000_sunny_11h 에서 조건 태그를 되찾는다
    parts = session.name.split('_')
    tag = '_'.join(parts[2:]) if len(parts) > 2 else session.name

    frames = []
    with meta.open(newline='') as f:
        for row in csv.DictReader(f):
            if limit is not None and len(frames) >= limit:
                break
            idx = int(row['idx'])
            rgb_path = session / 'rgb' / f'{idx:06d}.jpg'
            sem_path = session / 'sem' / f'{idx:06d}.jpg'
            if not rgb_path.exists() or not sem_path.exists():
                continue
            rgb = cv2.imread(str(rgb_path))
            sem = cv2.imread(str(sem_path))
            if rgb is None or sem is None:
                continue
            if rgb.shape[:2] != sem.shape[:2]:
                echo(f'  ⚠ 해상도 불일치 {idx}: RGB {rgb.shape[:2]} vs sem {sem.shape[:2]}')
                continue
            boxes = label_frame(rgb, sem, min_box_px, min_lamp_area_px, tl_color)
            frames.append(Frame(idx=idx, rgb=str(rgb_path), sem=str(sem_path),
                                s=float(row['s']) if row.get('s') else None,
                                tag=tag, boxes=boxes))
    echo(f'  {session.name}: {len(frames)} 프레임')
    return frames


def save_frames(frames: Sequence[Frame], path: Path) -> None:
    path.write_text(json.dumps([asdict(f) for f in frames], indent=1))


def load_frames(path: Path) -> List[Frame]:
    return [Frame(**{**d, 'boxes': [Box(**b) for b in d['boxes']]})
            for d in json.loads(Path(path).read_text())]


def parse_color(text: str):
    parts = [int(v) for v in text.replace(',', ' ').split()]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError('B,G,R 세 값이어야 합니다')
    return tuple(parts)


def parse_roi(text: str):
    parts = [float(v) for v in text.replace(',', ' ').split()]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError('x_min,y_min,x_max,y_max 네 값이어야 합니다')
    return tuple(parts)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--session', nargs='+', required=True, type=Path,
                    help='recorder_node 출력 디렉토리 (여러 개 가능)')
    ap.add_argument('--out', type=Path, default=Path('out'))
    ap.add_argument('--min-box-px', type=int, default=12, help='이보다 작은 semantic 덩어리는 노이즈')
    ap.add_argument('--min-lamp-px', type=int, default=2, help='박스 안에서 램프로 볼 최소 넓이')
    ap.add_argument('--tl-color', type=parse_color, default=TL_COLOR,
                    help=f'semantic 신호등 색 B,G,R (기본 {TL_COLOR})')
    ap.add_argument('--roi', type=parse_roi, default=ROI,
                    help=f'크롭 ROI 비율 x0,y0,x1,y1 (기본 {ROI} = 전체)')
    ap.add_argument('--no-roi', action='store_true', help='ROI 를 무시하고 프레임 전체를 쓴다')
    ap.add_argument('--no-dedup', action='store_true')
    ap.add_argument('--require-s', action='store_true',
                    help='위치 s 가 없거나 유한하지 않은 프레임 제외 (원본 파일은 보존)')
    ap.add_argument('--val-ratio', type=float, default=0.2)
    ap.add_argument('--limit', type=int, default=None, help='세션마다 이만큼만 (시험용)')
    ap.add_argument('--route', default='global', choices=['global', 'spawn'],
                    help='정지선 표를 만들 경로 (분할 그룹 키에 쓴다)')
    args = ap.parse_args(argv)

    roi = (0.0, 0.0, 1.0, 1.0) if args.no_roi else args.roi
    if tuple(roi) == (0.0, 0.0, 1.0, 1.0):
        print('ROI 가 프레임 전체입니다 — --check 로 신호등 y 대역을 보고 좁히면 '
              'imgsz 가 줄고 Roboflow 보정이 훨씬 쉬워집니다 (ctrl_palette.py 의 ROI).')

    print('세션 읽는 중...')
    frames = []
    for session in args.session:
        frames += read_session(session, args.min_box_px, args.min_lamp_px,
                               tuple(args.tl_color), args.limit)
    if args.require_s:
        before = len(frames)
        frames = [f for f in frames if f.s is not None and math.isfinite(f.s)]
        print(f'위치 s 누락/비정상 제외: {before - len(frames)} 장 (원본 보존)')
    if not frames:
        print('프레임이 없습니다.', file=sys.stderr)
        return 1

    print('\n--- 원본 ---')
    stats(frames)

    if not args.no_dedup:
        before = len(frames)
        frames = dedup(frames)
        print(f'\n중복 제거: {before} → {len(frames)} ({100 * (1 - len(frames) / before):.0f} % 감소)')

    from .ctrl_stoplines import load_table, stop_line_s
    lines = stop_line_s(load_table(route=args.route))
    print(f'정지선 {len(lines)} 개 ({args.route}): ' + ', '.join(f'{v:.1f}' for v in lines))

    train, val = split_by_group(frames, lines, args.val_ratio)
    print(f'\n분할 (조건, 정지선 단위): train {len(train)} / val {len(val)}')
    print('\n--- 최종 ---')
    stats(frames)

    data = write_yolo(train, val, args.out / 'dataset', roi)
    n_review = write_review(frames, args.out / 'review', roi)
    save_frames(frames, args.out / 'frames.json')
    print(f'\n데이터셋 → {data}')
    print(f'검수본  → {args.out / "review"} ({n_review} 장)')
    print(f'Frame 목록 → {args.out / "frames.json"}  (ctrl_roboflow_io.py 입력)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
