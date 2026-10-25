#!/usr/bin/env python3
"""평가. **mAP 는 참고만 한다** — 대회에서 의미 있는 건 아래 표다 (명세서 §6).

    ml/bin/python -m foscar_camera_train.evaluate --weights runs/detect/train/weights/best.pt \
        --frames out/frames.json

    RED → GREEN 오판   신호위반. 사실상 실격. **0 이어야 함**
    미검출률           unknown_hold_s=3.0 이라 3초 후 그냥 출발한다 → 위반
    GREEN → RED 오판   불필요한 정지. 시간 손해
    거리 구간별 정확도  40/30/20/10 m 로 나눠야 의미 있음

거리는 `foscar_camera.stop_lines.StopLineTable` 로 푼다 — 주행 노드들과 같은 표여야
"평가에선 30 m 인데 주행에선 25 m" 같은 어긋남이 안 생긴다 (명세서 §8).
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .extract import SIZE_EDGES, Box, Frame, load_frames, split_by_group

#: 클래스 → traffic_light_node 가 내보내는 Event value (명세서 §3-1).
#: green_left 는 GREEN 이다 — 직좌 = 직진 가능. 별도 값으로 내면 go_colors 에 안 걸려 헛정지한다.
EVENT = {'red': 'RED', 'yellow': 'YELLOW', 'green': 'GREEN', 'green_left': 'GREEN',
         'left': 'LEFT', 'pedestrian': None}

Pair = Tuple[Optional[str], Optional[str], Optional[Box], Optional[Frame]]


def iou(a: Box, b: Box) -> float:
    x0, y0 = max(a.x, b.x), max(a.y, b.y)
    x1, y1 = min(a.x + a.w, b.x + b.w), min(a.y + a.h, b.y + b.h)
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    union = a.w * a.h + b.w * b.h - inter
    return inter / union if union > 0 else 0.0


def predict_all(model, frames: Sequence[Frame], conf=0.25, imgsz=None) -> List[List[Box]]:
    """프레임마다 예측 박스 목록. 클래스 이름은 모델의 names 에서 가져온다."""
    names = model.names
    out = []
    for f in frames:
        res = model.predict(f.rgb, conf=conf, verbose=False,
                            **({'imgsz': imgsz} if imgsz else {}))[0]
        boxes = []
        for b in res.boxes:
            x0, y0, x1, y1 = (float(v) for v in b.xyxy[0])
            boxes.append(Box(int(x0), int(y0), int(x1 - x0), int(y1 - y0),
                             cls=names[int(b.cls[0])]))
        out.append(boxes)
    return out


def match(pred: Sequence[Box], gt: Sequence[Box], iou_thr=0.5,
          frame: Optional[Frame] = None) -> List[Pair]:
    """매칭 후 (정답클래스, 예측클래스, gt박스, 프레임).

    못 맞춘 정답은 (cls, None) = 미검출, 남는 예측은 (None, cls) = 오검출.
    """
    pairs: List[Pair] = []
    used = set()
    for g in gt:
        best, best_iou = None, 0.0
        for i, p in enumerate(pred):
            if i in used:
                continue
            v = iou(p, g)
            if v > best_iou:
                best, best_iou = i, v
        if best is not None and best_iou >= iou_thr:
            used.add(best)
            pairs.append((g.cls, pred[best].cls, g, frame))
        else:
            pairs.append((g.cls, None, g, frame))
    for i, p in enumerate(pred):
        if i not in used:
            pairs.append((None, p.cls, p, frame))
    return pairs


def critical_rates(pairs: Sequence[Pair], echo=print) -> dict:
    """대회에서 실제로 위험한 것들. mAP 보다 이 표를 본다."""
    real = [p for p in pairs if p[0] is not None and EVENT.get(p[0]) is not None]
    n = len(real)
    if not n:
        echo('평가할 차량등 박스가 없습니다.')
        return {}

    def ev(cls):
        return EVENT.get(cls) if cls else None

    red = [p for p in real if ev(p[0]) == 'RED']
    green = [p for p in real if ev(p[0]) == 'GREEN']
    red_as_green = sum(1 for p in red if ev(p[1]) == 'GREEN')
    green_as_red = sum(1 for p in green if ev(p[1]) == 'RED')
    missed = sum(1 for p in real if p[1] is None)
    false_pos = sum(1 for p in pairs if p[0] is None and ev(p[1]) is not None)

    out = {
        'red_as_green': red_as_green, 'n_red': len(red),
        'green_as_red': green_as_red, 'n_green': len(green),
        'missed': missed, 'n': n, 'false_pos': false_pos,
    }
    echo('--- 치명 지표 (명세서 §6) ---')
    echo(f'  RED → GREEN   {red_as_green}/{len(red)} '
         f'({100 * red_as_green / max(1, len(red)):.2f} %)   ← 신호위반. 0 이어야 함'
         + ('  ❌' if red_as_green else '  ✅'))
    echo(f'  미검출        {missed}/{n} ({100 * missed / n:.2f} %)'
         f'   ← unknown_hold_s=3.0 이면 3초 뒤 출발한다')
    echo(f'  GREEN → RED   {green_as_red}/{len(green)} '
         f'({100 * green_as_red / max(1, len(green)):.2f} %)   ← 불필요한 정지')
    echo(f'  오검출(없는데 봄) {false_pos}')
    return out


def _confusion(pairs: Sequence[Pair], bucket_of, bucket_names, echo=print, title='') -> dict:
    """구간 × (정답→예측) 표. bucket_of 가 None 을 주면 그 쌍은 뺀다."""
    table: Dict[object, Dict[Tuple[str, Optional[str]], int]] = {}
    for gt_cls, pred_cls, box, frame in pairs:
        if gt_cls is None or EVENT.get(gt_cls) is None:
            continue
        b = bucket_of(box, frame)
        if b is None:
            continue
        table.setdefault(b, {})
        key = (gt_cls, pred_cls)
        table[b][key] = table[b].get(key, 0) + 1

    echo(f'--- {title} ---')
    for b in sorted(table, key=lambda v: (v is None, v)):
        rows = table[b]
        total = sum(rows.values())
        correct = sum(v for (g, p), v in rows.items() if g == p)
        missed = sum(v for (g, p), v in rows.items() if p is None)
        echo(f'  {bucket_names(b):>12} : n {total:5d} | 정확 {100 * correct / max(1, total):5.1f} % '
             f'| 미검출 {100 * missed / max(1, total):5.1f} %')
        for (g, p), v in sorted(rows.items(), key=lambda kv: -kv[1]):
            if g != p and v:
                echo(f'                   {g} → {p or "미검출"}  {v}')
    return table


def confusion_by_size(pairs, edges=SIZE_EDGES, echo=print):
    """박스 높이 px 구간별. 작은 박스에서 무너지는 지점을 본다."""
    bins = [0, *edges, 10 ** 6]

    def bucket(box, _frame):
        return None if box is None else int(np.searchsorted(edges, box.h))

    return _confusion(pairs, bucket, lambda b: f'{bins[b]}~{bins[b + 1]}px', echo,
                      '박스 높이 구간별')


def confusion_by_distance(pairs, lines, edges=(10, 20, 30, 40), echo=print):
    """정지선까지 거리 구간별. meta 의 s 를 StopLineTable 로 푼다."""
    from .stoplines import distance_to_next
    bins = [0, *edges, 10 ** 6]

    def bucket(_box, frame):
        if frame is None:
            return None
        d = distance_to_next(frame.s, lines)
        return None if d is None else int(np.searchsorted(edges, d))

    return _confusion(pairs, bucket, lambda b: f'{bins[b]}~{bins[b + 1]}m', echo,
                      '정지선까지 거리 구간별')


def detection_range(pairs, lines, edges=(10, 20, 30, 40, 50), floor=0.9, echo=print):
    """검출률이 무너지는 거리. enter_distance_m 과 속도 전략을 이 값으로 정한다."""
    from .stoplines import distance_to_next
    hit, total = {}, {}
    for gt_cls, pred_cls, _box, frame in pairs:
        if gt_cls is None or EVENT.get(gt_cls) is None or frame is None:
            continue
        d = distance_to_next(frame.s, lines)
        if d is None:
            continue
        b = int(np.searchsorted(edges, d))
        total[b] = total.get(b, 0) + 1
        hit[b] = hit.get(b, 0) + (pred_cls is not None)

    bins = [0, *edges, 10 ** 6]
    limit = None
    echo('--- 검출 범위 ---')
    for b in sorted(total):
        rate = hit.get(b, 0) / total[b]
        echo(f'  {bins[b]:>3}~{bins[b + 1]:<4} m : 검출률 {100 * rate:5.1f} % (n {total[b]})')
        if rate >= floor:
            limit = bins[b + 1]
    if limit is not None:
        echo(f'  → 검출률 {100 * floor:.0f} % 를 지키는 한계 ≈ {limit} m')
        echo(f'     stop_line_node 의 enter_distance_m(40) 과 비교하세요. '
             f'{"여유가 있습니다." if limit >= 40 else "⚠ 40 m 를 못 덮습니다 — 속도를 낮춰야 합니다."}')
    else:
        echo(f'  → 어느 구간도 검출률 {100 * floor:.0f} % 를 못 넘었습니다.')
    return limit


def report(model, frames: Sequence[Frame], lines, conf=0.25, iou_thr=0.5,
           imgsz=None, echo=print) -> dict:
    preds = predict_all(model, frames, conf, imgsz)
    pairs: List[Pair] = []
    for f, pred in zip(frames, preds):
        pairs += match(pred, f.labelled, iou_thr, f)
    echo(f'프레임 {len(frames)} | 정답 박스 {sum(len(f.labelled) for f in frames)} '
         f'| 예측 박스 {sum(len(p) for p in preds)}\n')
    out = {'critical': critical_rates(pairs, echo)}
    echo('')
    confusion_by_size(pairs, echo=echo)
    echo('')
    confusion_by_distance(pairs, lines, echo=echo)
    echo('')
    out['range'] = detection_range(pairs, lines, echo=echo)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--weights', type=Path, required=True)
    ap.add_argument('--frames', type=Path, required=True, help='extract.py 의 frames.json')
    ap.add_argument('--all', action='store_true', help='val 뿐 아니라 전체 프레임으로')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--iou', type=float, default=0.5)
    ap.add_argument('--imgsz', type=int, default=None)
    ap.add_argument('--val-ratio', type=float, default=0.2)
    ap.add_argument('--route', default='global', choices=['global', 'spawn'])
    args = ap.parse_args(argv)

    if not args.weights.exists():
        print(f'{args.weights} 가 없습니다.', file=sys.stderr)
        return 1
    frames = load_frames(args.frames)
    from .stoplines import load_table, stop_line_s
    lines = stop_line_s(load_table(route=args.route))

    if not args.all:
        _, frames = split_by_group(frames, lines, args.val_ratio)
        print(f'val 프레임 {len(frames)} 으로 평가 (전체는 --all)')

    from ultralytics import YOLO
    report(YOLO(str(args.weights)), frames, lines, args.conf, args.iou, args.imgsz)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
