"""extract.py 의 순수 로직. 합성 이미지로 돌리므로 MORAI 도 GPU 도 필요 없다."""

import numpy as np
import pytest

from foscar_camera_train import extract as ex
from foscar_camera_train.extract import Box, Frame
from foscar_camera_train.palette import TL_COLOR, aspect_kind

from synth import four, pedestrian, scene, three


# --- semantic → 박스 -------------------------------------------------------

def test_semantic_mask_는_팔레트_색만_잡는다():
    img = np.zeros((10, 10, 3), np.uint8)
    img[2:5, 3:7] = TL_COLOR
    img[7, 7] = (0, 255, 0)                     # 다른 클래스 (초록)
    mask = ex.semantic_mask(img)
    assert mask[3, 4] == 255
    assert mask[7, 7] == 0
    assert mask.sum() // 255 == 12


def test_semantic_mask_는_jpeg_로_뭉개진_경계도_잡는다():
    """20 px 물체는 JPEG 블록 3개 크기라 경계가 중간색이 된다 (명세서 §7)."""
    img = np.zeros((10, 10, 3), np.uint8)
    img[2:5, 3:7] = TL_COLOR
    img[2, 3] = (230, 25, 235)                  # 살짝 어긋난 경계 픽셀
    assert ex.semantic_mask(img)[2, 3] == 255
    img[9, 9] = (255, 140, 255)                 # 너무 멀면 안 잡혀야 한다
    assert ex.semantic_mask(img)[9, 9] == 0


def test_find_housings_는_작은_덩어리를_거른다():
    _, sem = scene([(100, 50, 48, 14, {}, 4), (300, 60, 2, 2, {}, 4)])
    boxes = ex.find_housings(ex.semantic_mask(sem), min_area_px=12)
    assert len(boxes) == 1
    b = boxes[0]
    assert (b.x, b.y, b.w, b.h) == (100, 50, 48, 14)


# --- 종횡비 ---------------------------------------------------------------

@pytest.mark.parametrize('w,h,kind,n,flag', [
    # ↓ 2026-09-22 MORAI 실측. 이 셋이 임계값의 근거다.
    (77, 19, 'vehicle', 4, None),           # 사색등 4.05
    (87, 22, 'vehicle', 4, None),           # 사색등 3.95
    (49, 16, 'vehicle', 3, None),           # 삼색등 3.06
    (12, 26, 'pedestrian', None, None),     # 보행자등 0.46
    (154, 19, 'merged', None, 'merged'),    # 사색등 둘이 붙음 8.11
    (49, 14, 'vehicle', 4, 'aspect_ambiguous'),   # 3.50 — 경계 한가운데
])
def test_aspect_kind(w, h, kind, n, flag):
    assert aspect_kind(w, h) == (kind, n, flag)


def test_실측_3개가_전부_가른다():
    """임계값이 근거로 삼은 실측값. 튜닝을 바꿀 때 이게 먼저 깨져야 한다."""
    assert aspect_kind(77, 19)[1] == 4
    assert aspect_kind(87, 22)[1] == 4
    assert aspect_kind(49, 16)[1] == 3
    assert all(aspect_kind(w, h)[2] is None for w, h in [(77, 19), (87, 22), (49, 16)])


@pytest.mark.parametrize('w,h', [(33, 7), (17, 4)])
def test_원거리_박스는_경계가_넘어진다(w, h):
    """높이 1 px 이 종횡비를 많이 흔들어서(h=4 면 ±1.13) 고정 폭은 의미가 없다."""
    from foscar_camera_train.palette import ASPECT_MARGIN, aspect_margin
    assert aspect_margin(w / h, h) > ASPECT_MARGIN


def test_병합은_사색등_하나보다_확실히_놓아야_한다():
    """사색등 하나가 4.05 라 이론값 3.43 기준의 병합 임계 8.0 은 너무 빡빡했다."""
    assert aspect_kind(77, 19)[0] == 'vehicle'          # 하나
    assert aspect_kind(154, 19)[0] == 'merged'          # 둘이 붙음
    assert aspect_kind(33, 7)[0] == 'vehicle'           # 원거리 하나 (4.71) — 병합 아님


# --- 박스 안 상태 판정 -----------------------------------------------------

def test_lamps_in_box_는_슬롯_위치를_되돌려준다():
    rgb, sem = four(green='green')
    box = ex.find_housings(ex.semantic_mask(sem))[0]
    lamps = ex.lamps_in_box(rgb, box)
    assert len(lamps) == 1
    color, nx, area = lamps[0]
    assert color == 'green'
    assert nx == pytest.approx(0.875, abs=0.04)


@pytest.mark.parametrize('lamps,expected', [
    ({'red': 'red'}, 'red'),
    ({'yellow': 'yellow'}, 'yellow'),
    ({'green': 'green'}, 'green'),
    ({'left': 'green'}, 'left'),
    ({'left': 'green', 'green': 'green'}, 'green_left'),
])
def test_사색등_상태_판정(lamps, expected):
    rgb, sem = four(**lamps)
    boxes = ex.label_frame(rgb, sem)
    assert len(boxes) == 1
    assert boxes[0].cls == expected
    assert boxes[0].flag is None


def test_삼색등에는_좌회전_슬롯이_없다():
    rgb, sem = three(green='green')
    box = ex.label_frame(rgb, sem)[0]
    assert box.cls == 'green'
    assert box.flag is None


def test_램프가_꺼져_있으면_no_lamp_로_제외된다():
    """뒷면 신호등이거나 소등 — 데이터셋에서 빼야 한다 (명세서 §3-3)."""
    rgb, sem = four()
    box = ex.label_frame(rgb, sem)[0]
    assert box.cls is None
    assert box.flag == 'no_lamp'


def test_색과_위치가_어긋나면_pos_mismatch():
    """빨강이 초록 자리(0.875)에 켜져 있다 — 팔레트나 LAMP_X 를 의심해야 한다."""
    rgb, sem = four(green='red')
    box = ex.label_frame(rgb, sem)[0]
    assert box.flag == 'pos_mismatch'
    assert box.cls == 'red'                      # 추정 클래스는 남긴다


def test_갠트리_병합은_클래스_없이_검수행으로():
    rgb, sem = scene([(100, 50, 160, 14, {'red': 'red'}, 4)])
    box = ex.label_frame(rgb, sem)[0]
    assert box.cls is None
    assert box.flag == 'merged'


def test_보행자등은_색과_무관하게_pedestrian():
    rgb, sem = pedestrian(on='red')
    boxes = ex.label_frame(rgb, sem)
    assert [b.cls for b in boxes] == ['pedestrian']


def test_classify_state_는_램프가_없으면_no_lamp():
    assert ex.classify_state([], 4) == (None, 'no_lamp')


def test_classify_state_는_merged_에_클래스를_안_준다():
    assert ex.classify_state([('red', 0.125, 9)], None) == (None, 'merged')


# --- 중복 제거 ------------------------------------------------------------

def _frame(idx, boxes, s=None, tag='t'):
    return Frame(idx=idx, rgb='', sem='', s=s, tag=tag, boxes=boxes)


def test_dedup_은_같은_장면을_버리고_전환은_남긴다():
    """빨간불 정차 구간은 날아가되, 적→녹 전환 순간은 남아야 한다 (명세서 §3-6)."""
    red = [Box(100, 50, 48, 14, 'red')]
    green = [Box(100, 50, 48, 14, 'green')]
    frames = [_frame(i, list(red)) for i in range(5)] + [_frame(5, list(green))]
    kept = ex.dedup(frames)
    assert [f.idx for f in kept] == [0, 5]


def test_dedup_은_크기_구간이_바뀌면_남긴다():
    frames = [_frame(0, [Box(100, 50, 20, 9, 'red')]),      # 높이 9  → 구간 1
              _frame(1, [Box(100, 50, 40, 18, 'red')])]     # 높이 18 → 구간 2
    assert len(ex.dedup(frames)) == 2


def test_label_signature_는_라벨_없는_박스를_무시한다():
    assert ex.label_signature([Box(0, 0, 4, 4, None, 'no_lamp')]) == ()


# --- 분할 -----------------------------------------------------------------

def test_split_by_group_은_한_그룹을_쪼개지_않는다():
    """연속 프레임이 train/val 에 쌍둥이로 들어가면 mAP 가 부풀려진다 (명세서 §5)."""
    lines = [100.0, 200.0, 300.0, 400.0]
    frames = [_frame(i, [Box(0, 0, 48, 14, 'red')], s=float(line) + i * 0.5, tag=tag)
              for tag in ('sunny_11h', 'foggy_15h')
              for line in lines for i in range(6)]
    train, val = ex.split_by_group(frames, lines, val_ratio=0.25)

    assert train and val
    assert len(train) + len(val) == len(frames)
    keys_train = {ex.group_key(f, lines) for f in train}
    keys_val = {ex.group_key(f, lines) for f in val}
    assert not (keys_train & keys_val), '같은 (조건, 정지선) 그룹이 양쪽에 걸쳐 있다'


def test_split_by_group_은_여러_조건에서_val_을_고르게_집는다():
    lines = [100.0, 200.0, 300.0, 400.0]
    frames = [_frame(i, [Box(0, 0, 48, 14, 'red')], s=line, tag=tag)
              for tag in ('a', 'b') for i, line in enumerate(lines)]
    _, val = ex.split_by_group(frames, lines, val_ratio=0.5)
    assert {f.tag for f in val} == {'a', 'b'}


# --- 크롭 · YOLO txt -------------------------------------------------------

def test_crop_roi_는_좌표를_옮기고_밖은_버린다():
    img = np.zeros((720, 1280, 3), np.uint8)
    boxes = [Box(700, 400, 40, 12, 'red'),       # 안
             Box(10, 10, 20, 8, 'green')]        # ROI 밖
    out, moved = ex.crop_roi(img, boxes, (0.5, 0.5, 1.0, 1.0))
    assert out.shape[:2] == (360, 640)
    assert len(moved) == 1
    assert (moved[0].x, moved[0].y) == (60, 40)


def test_crop_roi_는_걸친_박스를_잘라_맞춘다():
    img = np.zeros((720, 1280, 3), np.uint8)
    out, moved = ex.crop_roi(img, [Box(630, 350, 40, 20, 'red')], (0.5, 0.5, 1.0, 1.0))
    assert (moved[0].x, moved[0].y, moved[0].w, moved[0].h) == (0, 0, 30, 10)


def test_yolo_lines_는_정규화된_중심과_크기를_쓴다():
    line = ex.yolo_lines([Box(100, 50, 40, 20, 'green')], 200, 100, ('red', 'green'))[0]
    idx, cx, cy, w, h = line.split()
    assert idx == '1'
    assert float(cx) == pytest.approx(0.6) and float(cy) == pytest.approx(0.6)
    assert float(w) == pytest.approx(0.2) and float(h) == pytest.approx(0.2)


def test_yolo_lines_는_라벨_없는_박스를_건너뛴다():
    assert ex.yolo_lines([Box(0, 0, 4, 4, None, 'no_lamp')], 100, 100, ('red',)) == []


def test_split_by_group_은_train_을_비우지_않는다():
    """세션 하나만 넣으면 그룹이 하나뿐이다. 그때 전부 val 로 가면 학습이 조용히 깨진다."""
    lines = [100.0]
    frames = [_frame(i, [Box(0, 0, 48, 14, 'red')], s=100.0, tag='sunny_11h') for i in range(5)]
    train, val = ex.split_by_group(frames, lines, val_ratio=0.2, echo=lambda *a: None)
    assert len(train) == 5 and val == []


def test_split_by_group_은_val_ratio_0_이면_전부_train():
    lines = [100.0, 200.0]
    frames = [_frame(i, [Box(0, 0, 48, 14, 'red')], s=s, tag='t')
              for i, s in enumerate([100.0, 200.0])]
    train, val = ex.split_by_group(frames, lines, val_ratio=0.0)
    assert len(train) == 2 and val == []


# --- 프레임 이름 규약 (dataset/ 과 upload/ 가 공유) --------------------------

@pytest.mark.parametrize('s', [412.3, 0.0, -12.3, None])
def test_frame_stem_왕복(s):
    f = _frame(123, [], s=s, tag='sunny_11h')
    tag, idx, back = ex.parse_frame_stem(ex.frame_stem(f))
    assert (tag, idx) == ('sunny_11h', 123)
    assert back == (None if s is None else pytest.approx(s, abs=0.05))


def test_frame_stem_에는_점이_없다():
    assert '.' not in ex.frame_stem(_frame(1, [], s=412.3, tag='t'))
