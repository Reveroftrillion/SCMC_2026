"""지도(link_set.json) 기반 주행 가능 영역 마스크.

왜 필요한가
    지금까지 "도로 밖인가" 를 **경로 중심선에서 몇 m** 로만 판단했다. 그런데 그 임계값이
    분류 결과에 따라 달라서(정지물 2 m / 이동물 5 m), 가드레일이 VEHICLE 로 한 번 오분류되면
    필터까지 같이 뚫렸다 (2026-09-20 분석). 지도에는 링크마다 중심선과 폭이 들어 있으므로
    "도로 위인가" 는 분류와 무관한 **기하 질문**으로 답할 수 있다.

무엇을 도로로 보는가
    link_set.json 의 **모든 링크의 합집합**. 내 차로뿐 아니라 옆 차로·반대 차로·교차로 내부까지
    전부 도로다 (교차로에는 차선 구조가 없어서 내 경로 주변 ±N m 로는 못 자른다).
    "내가 갈 길 위인가" 는 경로 투영(obstacle_node.on_route_path)이 따로 판단한다.
    이 마스크는 그 앞단에서 **도로 밖 구조물을 걷어내는** 역할만 한다.

정확도
    링크 점 간격이 0.5 m 라 점에만 붙이면 0.25 m 오차가 난다 → 0.25 m 로 리샘플해서
    오차를 0.125 m 로 줄인다 (K-City 634 링크 75k 점 → 150k 점, 2.4 MB).
    링크 폭은 전부 3.5 m 이고 갓길은 포함되지 않는다. 그래서 가드레일은 도로 경계 밖이지만
    **여유(margin)** 를 너무 키우면 다시 들어온다 — 정지물은 좁게, 이동물은 인도까지 넓게.
"""

import json
import math
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.spatial import cKDTree


class RoadMask:
    """링크 합집합까지의 '경계 밖 거리' 를 준다. 0 이하면 도로 안."""

    def __init__(self, centers: np.ndarray, half_widths: np.ndarray, sample_m: float = 0.25):
        self.tree = cKDTree(centers)
        self.half = half_widths
        self.max_half = float(half_widths.max()) if half_widths.size else 0.
        self.sample_m = sample_m
        self.workers = 4    # obstacle_node sets its own; -1 (all cores) took 11-12 cores in the tunnel

    # ------------------------------------------------------------------
    @classmethod
    def from_links(cls, links, sample_m: float = 0.25) -> 'RoadMask':
        """[{'points': [[x,y,z],...], 'width_start':…, 'width_end':…}, …] → 마스크."""
        xs, hs = [], []
        for link in links:
            pts = np.asarray(link.get('points', []), dtype=np.float64)
            if pts.shape[0] < 2:
                continue
            pts = pts[:, :2]
            width = float(link.get('width_start') or link.get('width_end') or 3.5)
            dense = _resample(pts, sample_m)
            xs.append(dense)
            hs.append(np.full(dense.shape[0], width / 2.))
        if not xs:
            raise ValueError('링크가 비어 있습니다')
        return cls(np.vstack(xs), np.concatenate(hs), sample_m)

    @classmethod
    def from_file(cls, path, sample_m: float = 0.25) -> Optional['RoadMask']:
        """link_set.json 경로 → 마스크. 파일이 없으면 None (마스크 없이 동작)."""
        p = Path(path).expanduser()
        if not p.is_file():
            return None
        with p.open(encoding='utf-8') as fh:
            links = json.load(fh)
        return cls.from_links(links, sample_m)

    # ------------------------------------------------------------------
    def outside(self, xy: np.ndarray) -> np.ndarray:
        """각 점이 도로 경계 **밖으로** 얼마나 나갔는지 [m]. 도로 안이면 0.

        중심선까지 거리 − 그 링크의 반폭. 리샘플 간격의 절반(0.125 m)만큼 과대평가될 수 있다.
        """
        xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        if xy.size == 0:
            return np.zeros(0)
        dist, idx = self.tree.query(xy, k=1, workers=self.workers)
        return np.maximum(dist - self.half[idx], 0.)

    def outside_one(self, x: float, y: float) -> float:
        """점 하나. 도로 안이면 0."""
        if not (math.isfinite(x) and math.isfinite(y)):
            return math.inf
        return float(self.outside(np.array([[x, y]]))[0])

    def on_road(self, xy: np.ndarray, margin_m: float = 0.) -> np.ndarray:
        return self.outside(xy) <= margin_m


def _resample(pts: np.ndarray, step: float) -> np.ndarray:
    """폴리라인을 등간격으로 다시 찍는다 (점-폴리라인 거리 오차를 step/2 로 묶기 위해)."""
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    acc = np.concatenate(([0.], np.cumsum(seg)))
    total = float(acc[-1])
    if total <= 0.:
        return pts
    n = max(2, int(total / step) + 1)
    want = np.linspace(0., total, n)
    return np.column_stack((np.interp(want, acc, pts[:, 0]), np.interp(want, acc, pts[:, 1])))
