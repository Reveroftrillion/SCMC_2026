#!/usr/bin/env python3
"""Roboflow 왕복. **형식 변환은 없다 — 양쪽 다 YOLO txt 다.**

    ml/bin/python -m foscar_camera_train.roboflow_io --export out/frames.json --out out/upload --limit 50
    # → Roboflow 업로드 → 손으로 보정 → Export "YOLOv8", Resize None, Augmentation 없음
    ml/bin/python -m foscar_camera_train.roboflow_io --import ~/Downloads/x.zip --out dataset/

이 파일이 하는 일은 **왕복하면서 깨지는 것을 막는 것** 뿐이다 (명세서 §3-8).

    ① 분할을 Roboflow 에 맡기지 않는다 — 전부 train/ 으로 올리고 내려받은 뒤 다시 나눈다
    ② Preprocessing/Augmentation 을 끈 채로 Export 한다 (Resize None, Flip 없음)
    ③ 클래스 순서를 하드코딩하지 않는다 — 항상 export 의 data.yaml names 를 읽는다
    ④ 파일 이름에 tag 와 s 를 박는다 — Roboflow 가 해시를 붙여도 앞부분은 남는다
    ⑤ 전체 프레임이 아니라 ROI 크롭을 올린다 — 올리는 것 = 학습하는 것

④ 의 파일명은 **점을 쓰지 않는다.** `s412.3` 처럼 소수점을 넣으면 Roboflow 가 확장자로
착각해 자를 수 있다. 데시미터 정수로 박아 정확히 되돌린다.

    s 412.3 m  →  sunny_11h_000123_s004123.jpg
    s -12.3 m  →  sunny_11h_000123_sm000123.jpg      (연결로 위 정지선은 음수 s)
    s 없음     →  sunny_11h_000123_sNA.jpg
"""

import argparse
import re
import sys
import tempfile
import zipfile
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2

from .extract import (Box, Frame, crop_roi, dedup, frame_stem, load_frames, parse_frame_stem,
                      save_frames, split_by_group, stats, write_yolo, yolo_lines)
from .palette import CLASSES, ROI


# ---------------------------------------------------------------------------
# ④ 파일 이름 — 왕복해도 (tag, idx, s) 가 살아남아야 한다 (규약은 extract.frame_stem)
# ---------------------------------------------------------------------------

def upload_name(frame: Frame) -> str:
    """'sunny_11h_000123_s004123.jpg'.

    Roboflow export 가 '..._jpg.rf.<hash>.jpg' 로 바꿔도 앞부분은 남는다.
    §6 의 거리 구간별 평가가 s 를 쓰므로 이 연결이 끊기면 평가가 안 된다.
    """
    return frame_stem(frame) + '.jpg'


def parse_upload_name(name: str):
    """(tag, idx, s). '.rf.' 이후 해시와 '_jpg' 는 떼고 파싱."""
    return parse_frame_stem(name)


# ---------------------------------------------------------------------------
# 내보내기 — ROI 크롭 + YOLO txt, 전부 train/ 하나로
# ---------------------------------------------------------------------------

def write_upload(frames: Sequence[Frame], out: Path, roi=ROI,
                 names: Sequence[str] = CLASSES, limit: Optional[int] = None) -> Path:
    """ROI 크롭 + YOLO txt + data.yaml. 전부 train/ 하나로 — 분할은 Roboflow 에 안 맡긴다."""
    img_dir, lbl_dir = out / 'train' / 'images', out / 'train' / 'labels'
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    n = 0
    for f in frames:
        if limit is not None and n >= limit:
            break
        img = cv2.imread(f.rgb)
        if img is None:
            continue
        img, boxes = crop_roi(img, f.boxes, roi)
        h, w = img.shape[:2]
        name = upload_name(f)
        cv2.imwrite(str(img_dir / name), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        (lbl_dir / (Path(name).stem + '.txt')).write_text(
            '\n'.join(yolo_lines(boxes, w, h, names)) + '\n')
        n += 1

    (out / 'data.yaml').write_text(
        f'path: {out.resolve()}\n'
        f'train: train/images\n'
        f'val: train/images\n'
        f'nc: {len(names)}\n'
        f'names: [{", ".join(names)}]\n')
    (out / 'README.txt').write_text(
        'Roboflow 업로드 전 체크 (명세서 §3-8)\n'
        '  Version 만들 때 Preprocessing 의 Auto-Orient / Resize 를 모두 끈다 (Resize: None)\n'
        '  Augmentation 은 전부 끈다 — 특히 Flip Horizontal 은 좌회전 화살표를 뒤집어 치명적이다\n'
        '  Export 형식은 "YOLOv8"\n'
        '  분할은 건드리지 않는다 — 전부 train 으로 두고 내려받은 뒤 이쪽에서 다시 나눈다\n')
    return out


# ---------------------------------------------------------------------------
# 되받기 — ③ 클래스 순서는 항상 export 의 data.yaml 에서 읽는다
# ---------------------------------------------------------------------------

def read_names(data_yaml: Path) -> List[str]:
    """data.yaml 의 names. 리스트꼴과 인덱스 맵꼴을 모두 읽는다 (Roboflow 가 둘 다 쓴다)."""
    text = Path(data_yaml).read_text()
    m = re.search(r'^names:\s*\[(.*?)\]', text, re.M | re.S)
    if m:
        return [v.strip().strip('\'"') for v in m.group(1).split(',') if v.strip()]
    m = re.search(r'^names:\s*\n((?:\s+.+\n?)+)', text, re.M)
    if not m:
        raise ValueError(f'{data_yaml} 에서 names 를 못 읽었습니다')
    out = {}
    for line in m.group(1).splitlines():
        mm = re.match(r'\s*(?:-\s*|(\d+)\s*:\s*)(.+)', line)
        if mm:
            out[int(mm.group(1)) if mm.group(1) else len(out)] = mm.group(2).strip().strip('\'"')
    return [out[i] for i in sorted(out)]


def find_export_root(path: Path) -> Path:
    """zip 이면 풀고, data.yaml 이 있는 디렉토리를 찾는다."""
    if path.is_file() and zipfile.is_zipfile(path):
        tmp = Path(tempfile.mkdtemp(prefix='rf_export_'))
        with zipfile.ZipFile(path) as z:
            z.extractall(tmp)
        path = tmp
    if (path / 'data.yaml').exists():
        return path
    found = sorted(path.rglob('data.yaml'))
    if not found:
        raise FileNotFoundError(f'{path} 안에 data.yaml 이 없습니다')
    return found[0].parent


def import_export(zip_or_dir: Path, frames_json: Optional[Path] = None,
                  echo=print) -> List[Frame]:
    """Roboflow export 를 되읽어 Frame 목록 복원.

    - data.yaml 의 names 순서로 인덱스 해석 (CLASSES 순서를 믿지 않는다)
    - 파일명에서 (tag, idx, s) 복원
    - frames_json 을 주면 원본 semantic 경로까지 되살린다
    """
    root = find_export_root(Path(zip_or_dir))
    names = read_names(root / 'data.yaml')
    echo(f'export names ({len(names)}): {names}')
    if list(names) != list(CLASSES):
        echo(f'  ⚠ CLASSES 순서와 다릅니다 (기대 {list(CLASSES)}). '
             f'이 파일의 순서를 씁니다 — 명세서 §3-8 ③ 이 실제로 필요한 상황입니다.')

    original: Dict[Tuple[str, int], Frame] = {}
    if frames_json:
        original = {(f.tag, f.idx): f for f in load_frames(Path(frames_json))}

    frames, skipped, no_label = [], 0, 0
    for img_path in sorted(p for p in root.rglob('*') if p.suffix.lower() in ('.jpg', '.jpeg', '.png')):
        if 'labels' in img_path.parts:
            continue
        try:
            tag, idx, s = parse_upload_name(img_path.name)
        except ValueError:
            skipped += 1
            continue
        img = cv2.imread(str(img_path))
        if img is None:
            skipped += 1
            continue
        h, w = img.shape[:2]
        lbl = img_path.parent.parent / 'labels' / (img_path.stem + '.txt')
        boxes = []
        if not lbl.exists():
            no_label += 1
        else:
            for line in lbl.read_text().split('\n'):
                parts = line.split()
                if len(parts) != 5:
                    continue
                ci, cx, cy, bw, bh = int(parts[0]), *(float(v) for v in parts[1:])
                if not 0 <= ci < len(names):
                    continue
                boxes.append(Box(x=int(round((cx - bw / 2) * w)), y=int(round((cy - bh / 2) * h)),
                                 w=int(round(bw * w)), h=int(round(bh * h)), cls=names[ci]))
        src = original.get((tag, idx))
        frames.append(Frame(idx=idx, rgb=str(img_path),
                            sem=src.sem if src else '', s=s, tag=tag, boxes=boxes))
    if skipped:
        echo(f'  파일명이 규약에 안 맞거나 못 읽은 파일 {skipped} 개를 건너뛰었습니다')
    if no_label:
        # 라벨 없는 이미지는 '배경만 있는 프레임' 과 구분이 안 된다. 그대로 두면 왕복 비교에
        # '사람이 전부 삭제함' 으로 찍히고, dedup 이 빈 서명끼리 뭉쳐 한 장으로 줄여버린다.
        echo(f'  ⚠ 라벨 txt 가 없는 이미지 {no_label}/{len(frames)} 장. '
             f'export 구조가 <split>/images/*.jpg + <split>/labels/*.txt 가 맞는지 확인하세요.')
        if no_label == len(frames):
            raise ValueError(
                'export 안에서 라벨을 한 장도 못 찾았습니다. 되읽기를 멈춥니다 — '
                '이대로 진행하면 라벨이 전부 사라진 데이터셋이 만들어집니다.')
    echo(f'되읽은 프레임 {len(frames)}')
    return frames


# ---------------------------------------------------------------------------
# 왕복 전후 비교 — 사람이 뭘 고쳤는가 = 자동 라벨의 약점 목록
# ---------------------------------------------------------------------------

def iou(a: Box, b: Box) -> float:
    x0, y0 = max(a.x, b.x), max(a.y, b.y)
    x1, y1 = min(a.x + a.w, b.x + b.w), min(a.y + a.h, b.y + b.h)
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    union = a.w * a.h + b.w * b.h - inter
    return inter / union if union > 0 else 0.0


def diff_report(before: Sequence[Frame], after: Sequence[Frame], iou_thr=0.5, echo=print) -> dict:
    """왕복 전후 비교. 수정률이 높은 구간이 extract.py 를 고칠 곳이다."""
    idx_before = {(f.tag, f.idx): f for f in before}
    added = removed = kept = 0
    changes: Dict[Tuple[str, str], int] = {}
    by_size: Dict[int, List[int]] = {}
    from .extract import size_bin

    for f in after:
        src = idx_before.get((f.tag, f.idx))
        if src is None:
            added += len(f.boxes)
            continue
        src_boxes = list(src.labelled)
        used = set()
        for b in f.boxes:
            best, best_iou = None, 0.0
            for i, sb in enumerate(src_boxes):
                if i in used:
                    continue
                v = iou(b, sb)
                if v > best_iou:
                    best, best_iou = i, v
            if best is None or best_iou < iou_thr:
                added += 1
                continue
            used.add(best)
            kept += 1
            sb = src_boxes[best]
            bin_ = size_bin(sb)
            by_size.setdefault(bin_, [0, 0])
            by_size[bin_][1] += 1
            if sb.cls != b.cls:
                changes[(sb.cls, b.cls)] = changes.get((sb.cls, b.cls), 0) + 1
                by_size[bin_][0] += 1
        removed += len(src_boxes) - len(used)

    echo('--- 왕복 전후 ---')
    echo(f'  유지 {kept} | 사람이 추가 {added} | 사람이 삭제 {removed}')
    if changes:
        echo('  클래스 변경:')
        for (a, b), n in sorted(changes.items(), key=lambda kv: -kv[1]):
            echo(f'      {a} → {b}  {n}')
    if by_size:
        from .extract import SIZE_EDGES
        edges = [0, *SIZE_EDGES, 10 ** 6]
        echo('  크기 구간별 수정률 (박스 높이):')
        for bin_ in sorted(by_size):
            bad, total = by_size[bin_]
            echo(f'      {edges[bin_]}~{edges[bin_ + 1]} px : {100 * bad / max(1, total):.1f} % '
                 f'({bad}/{total})')
        echo('    수정률이 높은 구간이 extract.py 를 고칠 곳입니다.')
    return {'kept': kept, 'added': added, 'removed': removed, 'changes': changes}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--export', type=Path, metavar='FRAMES_JSON',
                   help='extract.py 가 만든 frames.json → 업로드본')
    g.add_argument('--import', dest='import_', type=Path, metavar='ZIP_OR_DIR',
                   help='Roboflow export (zip 또는 디렉토리) → dataset/')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--frames', type=Path, default=None,
                    help='--import 과 함께: 원본 frames.json (왕복 비교와 sem 경로 복원용)')
    ap.add_argument('--limit', type=int, default=None, help='--export 와 함께: 시험용 장수')
    ap.add_argument('--val-ratio', type=float, default=0.2)
    ap.add_argument('--no-dedup', action='store_true')
    ap.add_argument('--route', default='global', choices=['global', 'spawn'])
    args = ap.parse_args(argv)

    if args.export:
        frames = load_frames(args.export)
        # 업로드본은 이미 크롭된 좌표를 쓰므로 여기서 다시 크롭하지 않는다
        out = write_upload(frames, args.out, roi=(0.0, 0.0, 1.0, 1.0), limit=args.limit)
        n = len(list((out / 'train' / 'images').glob('*.jpg')))
        print(f'업로드본 {n} 장 → {out}')
        print((out / 'README.txt').read_text())
        if args.limit:
            print(f'⚠ 시험 왕복입니다. 내려받은 뒤 --import 로 되읽어 해상도·클래스 순서·박스가 '
                  f'그대로인지 확인한 다음 본 작업을 하세요 (명세서 §3-8).')
        return 0

    frames = import_export(args.import_, args.frames)
    if not frames:
        print('되읽은 프레임이 없습니다.', file=sys.stderr)
        return 1
    if args.frames:
        diff_report(load_frames(args.frames), frames)

    if not args.no_dedup:
        before = len(frames)
        frames = dedup(frames)
        print(f'중복 제거: {before} → {len(frames)}')

    from .stoplines import load_table, stop_line_s
    lines = stop_line_s(load_table(route=args.route))
    train, val = split_by_group(frames, lines, args.val_ratio)
    print(f'재분할 (조건, 정지선 단위): train {len(train)} / val {len(val)}')
    stats(frames)
    # 되읽은 이미지는 이미 ROI 크롭본이다 — 다시 자르면 안 된다
    data = write_yolo(train, val, args.out, roi=(0.0, 0.0, 1.0, 1.0))
    save_frames(frames, args.out / 'frames.json')
    print(f'데이터셋 → {data}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
