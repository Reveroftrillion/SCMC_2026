"""ctrl_roboflow_io.py — 왕복하면서 깨지는 것을 막는 부분만 시험한다 (명세서 §3-8)."""

import cv2
import numpy as np
import pytest

from ctrl_camera_train import ctrl_roboflow_io as rf
from ctrl_camera_train.ctrl_extract import Box, Frame, save_frames
from ctrl_camera_train.ctrl_palette import CLASSES


# --- ④ 파일 이름 왕복 ------------------------------------------------------

@pytest.mark.parametrize('s', [412.3, 0.0, -12.3, 1234.5, None])
def test_파일명은_tag_idx_s_를_그대로_되돌린다(s):
    f = Frame(idx=123, rgb='', sem='', s=s, tag='sunny_11h')
    tag, idx, back = rf.parse_upload_name(rf.upload_name(f))
    assert (tag, idx) == ('sunny_11h', 123)
    assert back == (None if s is None else pytest.approx(s, abs=0.05))


def test_파일명에는_점이_없다():
    """소수점을 넣으면 Roboflow 가 확장자로 착각해 자를 수 있다."""
    name = rf.upload_name(Frame(idx=1, rgb='', sem='', s=412.3, tag='sunny_11h'))
    assert name.count('.') == 1 and name.endswith('.jpg')


def test_roboflow_가_붙인_해시를_떼고_읽는다():
    mangled = 'sunny_11h_000123_s004123_jpg.rf.9ab3c7d1e5f00000.jpg'
    assert rf.parse_upload_name(mangled) == ('sunny_11h', 123, pytest.approx(412.3))


def test_밑줄이_든_태그도_살아남는다():
    f = Frame(idx=7, rgb='', sem='', s=1.0, tag='foggy_15h_rain')
    assert rf.parse_upload_name(rf.upload_name(f))[0] == 'foggy_15h_rain'


def test_규약에_안_맞는_이름은_에러():
    with pytest.raises(ValueError):
        rf.parse_upload_name('IMG_2024.jpg')


# --- ③ 클래스 순서를 하드코딩하지 않는다 -----------------------------------

def test_read_names_는_리스트꼴을_읽는다(tmp_path):
    p = tmp_path / 'data.yaml'
    p.write_text('path: .\ntrain: x\nnc: 3\nnames: [green, red, yellow]\n')
    assert rf.read_names(p) == ['green', 'red', 'yellow']


def test_read_names_는_인덱스_맵꼴도_읽는다(tmp_path):
    p = tmp_path / 'data.yaml'
    p.write_text("names:\n  0: green\n  1: red\n  2: yellow\n")
    assert rf.read_names(p) == ['green', 'red', 'yellow']


def test_read_names_는_대시_목록꼴도_읽는다(tmp_path):
    p = tmp_path / 'data.yaml'
    p.write_text("names:\n  - green\n  - red\n")
    assert rf.read_names(p) == ['green', 'red']


# --- 왕복 전체 ------------------------------------------------------------

def _make_frames(tmp_path, n=3):
    frames = []
    for i in range(n):
        img = np.zeros((100, 200, 3), np.uint8)
        path = tmp_path / f'src_{i}.jpg'
        cv2.imwrite(str(path), img)
        frames.append(Frame(idx=i, rgb=str(path), sem='', s=100.0 + i, tag='sunny_11h',
                            boxes=[Box(20, 30, 40, 12, 'red')]))
    return frames


def test_업로드본을_되읽으면_박스가_그대로다(tmp_path):
    """왕복 후 IoU 가 1.0 이어야 한다. 아니면 Resize/Auto-Orient 가 걸려 있는 것."""
    frames = _make_frames(tmp_path)
    rf.write_upload(frames, tmp_path / 'upload', roi=(0.0, 0.0, 1.0, 1.0))
    back = rf.import_export(tmp_path / 'upload', echo=lambda *a: None)

    assert len(back) == len(frames)
    for before, after in zip(frames, sorted(back, key=lambda f: f.idx)):
        assert after.tag == before.tag and after.idx == before.idx
        assert after.s == pytest.approx(before.s, abs=0.05)
        assert len(after.boxes) == 1
        assert after.boxes[0].cls == 'red'
        assert rf.iou(after.boxes[0], before.boxes[0]) > 0.9


def test_export_의_클래스_순서가_달라도_이름으로_복원한다(tmp_path):
    """§3-8 ③ — 'red=0' 을 박아두면 라벨이 통째로 어긋나는데 에러는 안 난다."""
    frames = _make_frames(tmp_path, 1)
    frames[0].boxes = [Box(20, 30, 40, 12, 'green')]
    out = tmp_path / 'upload'
    rf.write_upload(frames, out, roi=(0.0, 0.0, 1.0, 1.0))

    # Roboflow 가 알파벳순으로 다시 매긴 상황을 흉내낸다
    shuffled = sorted(CLASSES)
    (out / 'data.yaml').write_text(f'names: [{", ".join(shuffled)}]\n')
    txt = next((out / 'train' / 'labels').glob('*.txt'))
    old = txt.read_text().split()
    txt.write_text(' '.join([str(shuffled.index('green'))] + old[1:]) + '\n')

    back = rf.import_export(out, echo=lambda *a: None)
    assert back[0].boxes[0].cls == 'green'


def test_frames_json_을_주면_semantic_경로가_살아난다(tmp_path):
    frames = _make_frames(tmp_path, 1)
    frames[0].sem = '/some/sem/000000.jpg'
    save_frames(frames, tmp_path / 'frames.json')
    rf.write_upload(frames, tmp_path / 'upload', roi=(0.0, 0.0, 1.0, 1.0))
    back = rf.import_export(tmp_path / 'upload', tmp_path / 'frames.json', echo=lambda *a: None)
    assert back[0].sem == '/some/sem/000000.jpg'


def test_zip_으로_받은_export_도_읽는다(tmp_path):
    import zipfile
    frames = _make_frames(tmp_path, 2)
    rf.write_upload(frames, tmp_path / 'upload', roi=(0.0, 0.0, 1.0, 1.0))
    zip_path = tmp_path / 'export.zip'
    with zipfile.ZipFile(zip_path, 'w') as z:
        for p in (tmp_path / 'upload').rglob('*'):
            if p.is_file():
                z.write(p, p.relative_to(tmp_path / 'upload'))
    assert len(rf.import_export(zip_path, echo=lambda *a: None)) == 2


# --- 왕복 전후 비교 --------------------------------------------------------

def test_diff_report_는_사람이_고친_것을_센다():
    before = [Frame(idx=0, rgb='', sem='', s=1.0, tag='t', boxes=[
        Box(10, 10, 40, 12, 'red'), Box(100, 10, 40, 12, 'green')])]
    after = [Frame(idx=0, rgb='', sem='', s=1.0, tag='t', boxes=[
        Box(10, 10, 40, 12, 'red'),              # 그대로
        Box(100, 10, 40, 12, 'green_left'),      # 사람이 클래스를 고침
        Box(300, 10, 40, 12, 'red')])]           # 사람이 추가 (semantic 이 놓친 원거리)
    out = rf.diff_report(before, after, echo=lambda *a: None)
    assert out['kept'] == 2
    assert out['added'] == 1
    assert out['removed'] == 0
    assert out['changes'] == {('green', 'green_left'): 1}


def test_diff_report_는_사람이_지운_것도_센다():
    before = [Frame(idx=0, rgb='', sem='', s=1.0, tag='t',
                    boxes=[Box(10, 10, 40, 12, 'red'), Box(100, 10, 40, 12, 'red')])]
    after = [Frame(idx=0, rgb='', sem='', s=1.0, tag='t', boxes=[Box(10, 10, 40, 12, 'red')])]
    out = rf.diff_report(before, after, echo=lambda *a: None)
    assert out['removed'] == 1 and out['kept'] == 1


def test_라벨이_하나도_없으면_되읽기를_멈춘다(tmp_path):
    """라벨 없는 이미지는 '배경 프레임' 과 구분이 안 된다 — 그대로 두면 라벨이 통째로 사라진다."""
    frames = _make_frames(tmp_path, 2)
    out = tmp_path / 'upload'
    rf.write_upload(frames, out, roi=(0.0, 0.0, 1.0, 1.0))
    for txt in (out / 'train' / 'labels').glob('*.txt'):
        txt.unlink()
    with pytest.raises(ValueError, match='라벨을 한 장도 못 찾았습니다'):
        rf.import_export(out, echo=lambda *a: None)


def test_일부만_라벨이_없으면_경고하고_계속한다(tmp_path):
    frames = _make_frames(tmp_path, 3)
    out = tmp_path / 'upload'
    rf.write_upload(frames, out, roi=(0.0, 0.0, 1.0, 1.0))
    next(iter(sorted((out / 'train' / 'labels').glob('*.txt')))).unlink()
    msgs = []
    back = rf.import_export(out, echo=msgs.append)
    assert len(back) == 3
    assert any('라벨 txt 가 없는' in m for m in msgs)
