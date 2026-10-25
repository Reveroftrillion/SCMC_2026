"""합성 신호등 장면. MORAI 도 GPU 도 없이 라벨 로직을 시험한다."""

import cv2
import numpy as np

from foscar_camera_train.palette import LAMP_X, TL_COLOR

LAMP_BGR = {'red': (0, 0, 255), 'yellow': (0, 255, 255), 'green': (0, 255, 0)}


def scene(housings, size=(720, 1280)):
    """housings: [(x, y, w, h, {슬롯이름: 색}, ...)] → (rgb, sem).

    슬롯 이름은 LAMP_X 의 키('red'/'yellow'/'left'/'green')고, 색은 실제로 켜질 BGR 색이다.
    켜진 램프를 슬롯 위치에 그려서 위치 검증 로직을 시험할 수 있다.
    """
    h, w = size
    rgb = np.zeros((h, w, 3), np.uint8)
    sem = np.zeros((h, w, 3), np.uint8)
    for bx, by, bw, bh, lamps, n_aspect in housings:
        cv2.rectangle(sem, (bx, by), (bx + bw - 1, by + bh - 1), TL_COLOR, -1)
        cv2.rectangle(rgb, (bx, by), (bx + bw - 1, by + bh - 1), (30, 30, 30), -1)
        for slot, color in lamps.items():
            nx = LAMP_X[n_aspect][slot] if n_aspect in LAMP_X else float(slot)
            cx = int(bx + nx * bw)
            cy = int(by + bh / 2)
            cv2.circle(rgb, (cx, cy), max(1, bh // 3), LAMP_BGR[color], -1)
    return rgb, sem


def four(x=100, y=50, w=77, h=19, **lamps):
    """사색등 하나. 크기는 2026-09-22 MORAI 실측값 (77×19 = 4.05)."""
    return scene([(x, y, w, h, lamps, 4)])


def three(x=100, y=50, w=49, h=16, **lamps):
    """삼색등 하나. 크기는 2026-09-22 MORAI 실측값 (49×16 = 3.06)."""
    return scene([(x, y, w, h, lamps, 3)])


def pedestrian(x=100, y=50, w=12, h=26, **lamps):
    """보행자등 하나 (종횡비 12/26 = 0.46). 슬롯은 세로라 x 는 가운데."""
    rgb, sem = scene([(x, y, w, h, {}, 4)])
    for color in lamps.values():
        cv2.circle(rgb, (x + w // 2, y + h // 3), max(1, w // 3), LAMP_BGR[color], -1)
    return rgb, sem
