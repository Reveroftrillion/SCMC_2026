"""수집·라벨 양쪽이 쓰는 상수. **`--check` 로 확정할 값은 전부 여기 모여 있다.**

`recorder_node`(ROS) 와 `ctrl_extract.py`(ROS 없음) 가 같은 팔레트·같은 ROI 를 봐야 하므로
둘 다 여기서 읽는다. 어느 한쪽에만 고치면 마스크와 크롭이 어긋난다.

확정 절차는 명세서 §7 →

    ros2 launch ctrl_camera_train ctrl_record.launch.py check:=true drive:=false

가 찍는 요약을 보고 `TL_COLOR` 와 `ROI` 를 고친다. 그때까지의 기본값은 안전한 쪽 —
팔레트는 순수 마젠타, ROI 는 프레임 전체 — 이라 **값을 몰라도 파이프라인이 끝까지 돈다.**
"""

import numpy as np

# ---------------------------------------------------------------------------
# ⬇⬇ --check 결과를 보고 고칠 값은 이 블록 둘뿐이다 ⬇⬇
# ---------------------------------------------------------------------------

#: semantic 카메라의 신호등 클래스 색 (BGR). **2026-09-22 MORAI 실측값.**
#: 순수 마젠타 (255, 0, 255) 가 아니다 — 색거리가 75.7 이라 기본 임계 60 에 안 걸려
#: 박스가 하나도 안 나왔다. `--check` 요약의 'semantic 상위 색' 으로 확인할 것.
#: 하늘은 (255, 255, 0) 시안, 기둥·표지판은 올리브다.
TL_COLOR = (239, 74, 255)          # RGB 로는 (255, 74, 239)

#: 크롭 ROI — 화면 비율 (x_min, y_min, x_max, y_max). 기본은 프레임 전체.
#: 자유 카메라는 위를 보므로 신호등이 화면 중앙~중상단에 온다. `--check` 오버레이로
#: 신호등이 실제로 걸리는 y 대역을 보고 여유를 붙여 좁힌다. `imgsz` 가 여기서 나온다.
#: 좁히지 않으면 Roboflow 에 1280×720 원본을 올리게 돼 27×8 px 박스를 손으로 그려야 한다.
ROI = (0.0, 0.0, 1.0, 1.0)

# ---------------------------------------------------------------------------
# ⬆⬆ 여기까지 ⬆⬆
# ---------------------------------------------------------------------------

#: 팔레트 색으로 인정할 색거리(유클리드, 0~441). JPEG 아티팩트 때문에 정확히 일치하는
#: 픽셀만 보면 20 px 물체는 테두리가 통째로 날아간다 (명세서 §7).
COLOR_TOLERANCE = 60.0

#: 클래스. 순서를 코드에서 믿지 않는다 — 학습·평가는 항상 data.yaml 의 names 를 읽는다 (§3-8 ③).
CLASSES = ('red', 'yellow', 'green', 'green_left', 'left', 'pedestrian')

#: 켜진 램프의 정규화 x 위치. 한국 신호등은 순서가 고정이라 색과 교차검증할 수 있다.
LAMP_X = {4: {'red': .125, 'yellow': .375, 'left': .625, 'green': .875},
          3: {'red': .17,  'yellow': .50,                'green': .83}}

#: 위치 검증 허용 오차 (정규화 x). 사색등 칸 간격이 0.25 라 그 절반이 한계다.
LAMP_X_TOLERANCE = 0.12

# 함체 종횡비(w/h) 경계. **2026-09-22 MORAI 실측으로 잡았다.**
#
#     4구  77×19 = 4.05,  87×22 = 3.95
#     3구  49×16 = 3.06
#
# 명세서 §3-3 의 이론값(4구 1.2×0.35 = 3.43, 3구 0.9×0.35 = 2.57)보다 둘 다 19 % 크다.
# 다만 둘 사이 비율은 4.0/3.06 = 1.31 로 이론의 3.43/2.57 = 1.33 과 맞는다 — 모양 모델은
# 맞았고 절대 스케일만 틀렸던 것이다. 경계는 두 실측의 기하평균에 둔다.
PEDESTRIAN_MAX_ASPECT = 1.0     # 이보다 낮으면 세로로 긴 보행자등 (아직 실측 없음)
ASPECT_BOUNDARY = 3.48          # √(3.06 × 3.95). 위면 4구, 아래면 3구
MERGED_BOUNDARY = 5.73          # √(4.05 × 8.1). 4구 둘이 붙으면 ≈ 8.1 → 검수행

#: 경계 양옆으로 이만큼은 "모르겠다"(검수행)로 둔다. 실측 크기(h 16~22)에서 높이가 1 px
#: 흔들리면 종횡비가 ±0.2 움직인다. 다만 이 흔들림은 박스가 작을수록 커져서 —
#: h=7 이면 ±0.69, h=4 면 ±1.13 — 고정 폭으로는 원거리 박스에서 의미가 없다.
#: 그래서 `aspect_margin` 이 ratio/h 와 비교해 넓은 쪽을 쓴다.
ASPECT_MARGIN = 0.25


def color_distance(img_bgr: np.ndarray, color=TL_COLOR) -> np.ndarray:
    """각 픽셀과 `color` 사이의 유클리드 색거리. (H, W) float32."""
    diff = img_bgr.astype(np.float32) - np.asarray(color, np.float32)
    return np.sqrt((diff * diff).sum(axis=-1))


def roi_box(width: int, height: int, roi=ROI):
    """비율 ROI → 픽셀 (x0, y0, x1, y1). 최소 1 px 은 남긴다."""
    x0, y0, x1, y1 = roi
    px = (int(round(width * x0)), int(round(height * y0)),
          int(round(width * x1)), int(round(height * y1)))
    x0, y0, x1, y1 = px
    return x0, y0, max(x0 + 1, x1), max(y0 + 1, y1)


def aspect_margin(ratio: float, h: int) -> float:
    """경계 양옆의 '모르겠다' 폭. 박스가 작을수록 넓어진다.

    종횡비를 높이로 미분하면 d(w/h)/dh = -w/h² = -ratio/h 다. 곧 `ratio/h` 가 **높이가
    1 px 흔들릴 때 종횡비가 움직이는 양**이고, 실측과도 맞는다 (77×19 → 0.21, 49×16 →
    0.19, 33×7 → 0.67). 가까운 박스에서는 이 값이 작아지므로 `ASPECT_MARGIN` 을 바닥으로 쓴다.
    """
    return max(ASPECT_MARGIN, ratio / h)


def aspect_kind(w: int, h: int):
    """함체 종횡비 → ('vehicle'|'pedestrian'|'merged', n_aspect|None, flag|None).

    `n_aspect` 는 삼색등 3 / 사색등 4. 경계 근처는 못 정하므로 `aspect_ambiguous` 를 달아
    검수행으로 보낸다 — 여기서 조용히 틀리면 위치 검증(LAMP_X)이 통째로 어긋난다.

    **못 정할 때는 4 로 둔다.** 어느 쪽으로 틀리는 편이 나은지 따져보면:

    - 3구를 4구로 읽음 → red(.17↔.125)·green(.83↔.875)은 오차 .045 라 그대로 맞는다.
      yellow(.50↔.375)만 오차 .125 로 허용치(.12)를 넘어 검수행이 된다.
    - 4구를 3구로 읽음 → **좌회전(.625)이 3구의 green(.83)에 붙어 LEFT 가 GREEN 이 된다.**
      좌회전 전용 신호에 직진하는 것이라 가장 위험하다. 오차 .205 로 검수행에는 걸리지만,
      애초에 그쪽으로 기울일 이유가 없다.
    """
    if h <= 0:
        return 'merged', None, 'no_lamp'
    ratio = w / float(h)
    margin = aspect_margin(ratio, h)
    if ratio >= MERGED_BOUNDARY + margin:
        return 'merged', None, 'merged'
    if ratio <= PEDESTRIAN_MAX_ASPECT:
        return 'pedestrian', None, None
    if ratio >= ASPECT_BOUNDARY + margin:
        return 'vehicle', 4, None
    if ratio <= ASPECT_BOUNDARY - margin:
        return 'vehicle', 3, None
    return 'vehicle', 4, 'aspect_ambiguous'
