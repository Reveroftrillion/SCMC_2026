"""경로(Route) ↔ Frenet (s, d) 변환. ROS 없음.

`ctrl_control.ctrl_core.Route.project` 는 자차용이라 탐색창이 직전 s 의 -3~+8 m 뿐이고 횡거리에 부호가
없다. 여기서는 **자차 앞 100 m / 뒤 30 m** 창에서 찾고 부호(좌 +)를 붙인다. 부호 규약은
`obstacle_node.project_to_route` 와 같고 DrivingTarget.lat_offset 과도 같다.
한 바퀴 도는 경로라 같은 자리를 두 번 지나므로 창 없이 전체에서 찾으면 안 된다.
"""

import math
from typing import Tuple

import numpy as np


def to_frenet(route, x: float, y: float, near_s: float, back_m: float = 30., ahead_m: float = 100.
              ) -> Tuple[float, float]:
    """(x, y) → (s_local, d). near_s 는 자차의 local s. d 는 좌 +."""
    lo = max(0, int(np.searchsorted(route.s, near_s - back_m)) - 1)
    hi = min(len(route.delta), int(np.searchsorted(route.s, near_s + ahead_m)) + 1)
    if hi <= lo:
        lo, hi = 0, len(route.delta)
    idx = np.arange(lo, hi)
    p0 = route.points[idx]
    vec = route.delta[idx]
    length = route.lengths[idx]
    q = np.array([x, y]) - p0
    t = np.clip((q * vec).sum(axis=1) / length ** 2, 0., 1.)
    foot = p0 + t[:, None] * vec
    diff = np.array([x, y]) - foot
    dist = np.hypot(diff[:, 0], diff[:, 1])
    k = int(np.argmin(dist))
    s = float(route.s[idx[k]] + t[k] * length[k])
    cross = vec[k, 0] * diff[k, 1] - vec[k, 1] * diff[k, 0]          # 접선 × 오프셋 벡터 (z 성분)
    d = math.copysign(float(dist[k]), cross) if dist[k] > 1e-9 else 0.
    return s, d


def to_xy(route, s: float, d: float) -> Tuple[float, float]:
    """(s_local, d) → (x, y). tracker 의 lat_offset 적용식과 같다 (ctrl_core.py mission hook 2/3)."""
    x, y = route.at(s)
    h = route.heading_at(s)
    return x - math.sin(h) * d, y + math.cos(h) * d


def base_to_map(x_b: float, y_b: float, ego_x: float, ego_y: float, ego_yaw: float) -> Tuple[float, float]:
    """base_link (x 전방, y 좌) → map."""
    c, s = math.cos(ego_yaw), math.sin(ego_yaw)
    return ego_x + c * x_b - s * y_b, ego_y + s * x_b + c * y_b


def s_offset_from(route, x: float, y: float, s_global: float) -> float:
    """route local s − global s. 자차 위치를 경로 전체에 투영해 한 번 잰다.

    경로가 폐루프라 출발점에서는 s=0 과 s=끝 이 같은 자리다 — 끝쪽에 붙으면 오프셋이 한 바퀴만큼
    틀리므로 (−100, 길이−100) 범위로 되돌린다. spawn 연결로 경로의 오프셋은 연결로 길이(양수, 작다)."""
    s_local, _, _ = route.project(x, y)
    offset = s_local - s_global
    length = float(route.s[-1])
    if offset > length - 100.:
        offset -= length
    elif offset < -100.:
        offset += length
    return offset
