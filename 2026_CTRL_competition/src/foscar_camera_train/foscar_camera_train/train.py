#!/usr/bin/env python3
"""ultralytics 학습. **torch 가 필요하므로 venv 로 직접 돌린다** (`ros2 run` 안 거침).

    ml/bin/python -m foscar_camera_train.train --data out/dataset/data.yaml

ultralytics 기본값을 그냥 쓰면 이 문제에서 망가진다 (명세서 §5). 바꿔야 하는 것은 `AUG` 에
모여 있고, 그중 `fliplr` 이 가장 위험하다 — 좌회전 화살표가 우회전이 되고 사색등 순서
`적-황-좌-녹` 이 `녹-좌-황-적` 으로 뒤집혀 **위치 단서를 정반대로 학습한다.**
"""

import argparse
import json
import sys
from pathlib import Path

#: ultralytics 기본값에서 반드시 바꿀 것 (명세서 §5). 나머지는 기본값을 쓴다.
AUG = dict(
    fliplr=0.0,     # 좌회전 화살표가 우회전이 된다. 사색등 순서도 뒤집힌다
    hsv_h=0.0,      # 색상이 곧 클래스다
    scale=0.25,     # 기본 0.5 는 이미 작은 물체를 더 줄인다
    hsv_s=0.7,      # 안개 = 채도 저하. 유지해서 일반화에 쓴다
    hsv_v=0.4,      # 시간대(11/13/15시) 변화
)

DEFAULT_MODEL = 'yolo26s.pt'
DEFAULT_IMGSZ = 1280        # roi.json 이 없을 때. ROI 가 프레임 전체(1280×720)라는 전제
DEFAULT_BATCH = 4           # 8 GB + imgsz 1280 기준. imgsz 를 줄이면 올릴 수 있다


def read_names(data_yaml: Path):
    from .roboflow_io import read_names as _read
    return _read(Path(data_yaml))


def imgsz_from_roi(data_yaml: Path):
    """data.yaml 옆 roi.json 의 imgsz. ROI 를 축소 없이 넣는 것이 목적이다."""
    roi = Path(data_yaml).parent / 'roi.json'
    if not roi.exists():
        return None
    return int(json.loads(roi.read_text())['imgsz'])


def check_dataset(data_yaml: Path, leak_m: float = 5.0, echo=print) -> dict:
    """학습 전 점검. 클래스 분포, 박스 크기 분포, train/val 누수.

    누수는 **같은 조건에서 s 가 `leak_m` 안에 있는 프레임이 양쪽에 나뉜 것**을 본다.
    접근 중 연속 프레임은 거의 같은 그림이라, 그런 게 있으면 mAP 가 0.99 로 나오고
    실주행에서 무너진다 (명세서 §5).

    idx 가 아니라 s 로 재는 것이 중요하다 — idx 는 세션 안에서 연속이라 정지선이 바뀌는
    지점에서도 1 만큼만 차이 난다. 그건 전혀 다른 장면이지 누수가 아니다.
    """
    from .extract import parse_frame_stem

    data_yaml = Path(data_yaml)
    root = data_yaml.parent
    names = read_names(data_yaml)
    echo(f'클래스 {len(names)}: {names}')
    echo('  ↑ 이 순서가 라벨 인덱스다. CLASSES 튜플이 아니라 이 파일을 믿는다 (명세서 §3-8 ③)')

    counts, positions, no_s = {}, {'train': [], 'val': []}, 0
    for split in ('train', 'val'):
        lbl_dir, img_dir = root / 'labels' / split, root / 'images' / split
        n_img = len(list(img_dir.glob('*.jpg'))) if img_dir.exists() else 0
        n_box = 0
        for txt in sorted(lbl_dir.glob('*.txt')) if lbl_dir.exists() else []:
            try:
                tag, idx, s_val = parse_frame_stem(txt.name)
            except ValueError:
                tag, idx, s_val = txt.stem, -1, None
            if s_val is None:
                no_s += 1
            else:
                positions[split].append((tag, s_val, idx))
            for line in txt.read_text().split('\n'):
                p_ = line.split()
                if len(p_) != 5:
                    continue
                n_box += 1
                ci = int(p_[0])
                name = names[ci] if ci < len(names) else f'?{ci}'
                counts.setdefault(split, {})[name] = counts.setdefault(split, {}).get(name, 0) + 1
        echo(f'{split}: 이미지 {n_img}, 박스 {n_box}')
        echo('   ' + (', '.join(f'{k} {v}' for k, v in sorted(counts.get(split, {}).items()))
                      or '박스 없음'))

    leaks = []
    val_by_tag = {}
    for tag, s_val, idx in positions['val']:
        val_by_tag.setdefault(tag, []).append((s_val, idx))
    for tag in val_by_tag:
        val_by_tag[tag].sort()
    import bisect
    for tag, s_val, idx in positions['train']:
        cand = val_by_tag.get(tag)
        if not cand:
            continue
        j = bisect.bisect_left(cand, (s_val, -1))
        for k in (j - 1, j):
            if 0 <= k < len(cand) and abs(cand[k][0] - s_val) <= leak_m:
                leaks.append((tag, idx, s_val, cand[k][1], cand[k][0]))
    if no_s:
        echo(f'⚠ 파일명에 s 가 없는 라벨 {no_s} 개 — 누수 점검에서 빠집니다 '
             f'(수집 때 /localization/s 가 없었던 프레임입니다).')
    if leaks:
        echo(f'⚠ 누수 {len(leaks)} 쌍 — 같은 조건에서 {leak_m:.0f} m 안의 프레임이 '
             f'train/val 양쪽에 있습니다:')
        for tag, a, sa, b, sb in leaks[:10]:
            echo(f'    {tag}: train {a:06d}(s {sa:.1f}) ↔ val {b:06d}(s {sb:.1f})')
        echo('    split_by_group 의 그룹 키를 확인하세요 (s 가 비어 있으면 그룹이 안 나뉩니다).')
    else:
        echo(f'누수 점검: 같은 조건에서 {leak_m:.0f} m 안에 걸친 프레임 없음')

    missing = [n for n in names if n not in counts.get('train', {})]
    if missing:
        echo(f'⚠ train 에 한 장도 없는 클래스: {missing}')
    return {'names': names, 'counts': counts, 'leaks': len(leaks)}


def train(data_yaml, model=DEFAULT_MODEL, imgsz=None, batch=DEFAULT_BATCH, epochs=150,
          patience=30, **kw):
    """imgsz 는 ROI 긴 변(32 배수). None 이면 data.yaml 옆 roi.json 에서 읽는다.

    ROI 를 축소 없이 넣는 것이 목적이다. 8 GB 기준 imgsz 1280 이면 batch 4,
    imgsz 를 줄이면 batch 를 올린다.
    """
    from ultralytics import YOLO

    if imgsz is None:
        imgsz = imgsz_from_roi(data_yaml) or DEFAULT_IMGSZ
    args = dict(AUG)
    args.update(kw)
    print(f'학습 시작: {model} | imgsz {imgsz} | batch {batch} | epochs {epochs}')
    print(f'  증강: {args}')
    return YOLO(model).train(data=str(data_yaml), imgsz=imgsz, batch=batch,
                             epochs=epochs, patience=patience, **args)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--data', type=Path, required=True, help='data.yaml')
    ap.add_argument('--model', default=DEFAULT_MODEL)
    ap.add_argument('--imgsz', type=int, default=None, help='기본: roi.json 의 값')
    ap.add_argument('--batch', type=int, default=DEFAULT_BATCH)
    ap.add_argument('--epochs', type=int, default=150)
    ap.add_argument('--patience', type=int, default=30)
    ap.add_argument('--resume', action='store_true')
    ap.add_argument('--check-only', action='store_true', help='점검만 하고 학습은 안 한다')
    args = ap.parse_args(argv)

    if not args.data.exists():
        print(f'{args.data} 가 없습니다. extract.py 를 먼저 돌리세요.', file=sys.stderr)
        return 1
    info = check_dataset(args.data)
    if args.check_only:
        return 0
    if info['leaks']:
        print('누수 의심이 있는 채로 학습하면 mAP 를 믿을 수 없습니다. 그래도 진행합니다.')
    train(args.data, args.model, args.imgsz, args.batch, args.epochs, args.patience,
          resume=args.resume)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
