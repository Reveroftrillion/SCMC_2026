"""정적 장애물 회피 — "옆으로 몇 m 비킬까" 만 정하는 Frenet-lite 플래너. ROS 없음.

입력  Frenet 박스 목록 (s 구간 × d 구간, 좌 +), 자차 s, 직전에 고른 d
후보  d = -d_max … +d_max, d_step 간격 (기본 25개). 후보마다 **일정한 오프셋으로 달리는 복도**

숫자 감각 (차선 3.5 m, 차폭 1.892 m, s=329 상자 폭 2 m 가 차선 한가운데일 때)
    필요한 |d| = 상자 반폭 1.0 + pad 0.2 + 차 반폭 0.946 + margin 0.3 = 2.45 m → 후보 2.5 m
    → 차가 통째로 옆 차선에 들어간다. 이건 물리적으로 피할 수 없다 (어느 쪽이든 실선을 밟는다).
    margin 을 0.5 로 올리면 2.65 m, d_max 가 2.5 면 **못 지나가서 선다**. 값 바꿀 때 이 계산부터.
    일반 상자 후보는 ±3 m. 지도 검증 터널 후보와 tracker 한계는 ±6 m.
검사  차폭 + 마진 만큼의 d 구간이 박스의 d 구간과 겹치고, s 구간도 앞 horizon 안에서 겹치면 탈락
비용  |d|·w_offset  +  차선 밖으로 나간 폭·w_line  +  직전과 반대쪽이면 w_switch  +  |d − 직전|·w_change
      (좌우가 똑같으면 왼쪽: w_left_bias 만큼 깎는다)
출력  가장 싼 d, 그때 낼 속도 상한, 후보 전부(시각화용). 가능한 후보가 없으면 blocked=True.

속도 상한은 만들지 않는다. 통과 불가 시 상태가 stop_s 를 요청한다.

곡선 모양(5차 다항식 등)은 여기서 만들지 않는다 — tracker 가 lat_offset 이 바뀐 s 부터
max(transition_min_m, v·transition_time_s) 길이의 smoothstep 곡선으로 옮겨 준다 (core.py mission hook 2/3,
/control/path 로 RViz 에 보인다). 나중에 필요하면 `candidates()` 만 바꾸면 된다.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class Box:
    """장애물이 차지하는 Frenet 영역 (s 는 global)."""
    s_min: float
    s_max: float
    d_min: float
    d_max: float
    vertices: tuple = ()       # Convex Frenet outline in perimeter order; empty means the AABB.

    @property
    def length(self) -> float:
        return self.s_max - self.s_min


def polygon_box(vertices, pad=0.) -> Box:
    """Enclose observed Frenet points with a convex hull and optional axis-aligned padding.

    Degenerate outlines retain their conservative AABB collision behavior.
    """
    points = sorted(set((float(s), float(d)) for s, d in vertices))
    if not points:
        raise ValueError('polygon_box requires at least one observed point')
    if pad > 0.:
        points = sorted(set((s + ds, d + dd) for s, d in points
                            for ds in (-pad, pad) for dd in (-pad, pad)))

    def cross(o, a, b):
        return (a[0] - o[0])*(b[1] - o[1]) - (a[1] - o[1])*(b[0] - o[0])

    lower, upper = [], []
    for chain, ordered in ((lower, points), (upper, reversed(points))):
        for p in ordered:
            while len(chain) >= 2 and cross(chain[-2], chain[-1], p) <= 0.:
                chain.pop()
            chain.append(p)
    hull = tuple(lower[:-1] + upper[:-1])
    return Box(min(p[0] for p in points), max(p[0] for p in points),
               min(p[1] for p in points), max(p[1] for p in points),
               hull if len(hull) >= 3 else ())


@dataclass
class Candidate:
    d: float
    feasible: bool
    cost: float
    line_over: float = 0.       # 차 바깥쪽 모서리가 차선 밖으로 나간 폭 [m]
    reason: str = ''


@dataclass
class Plan:
    d: float                    # 고른 오프셋 [m]. blocked 면 직전 값
    v_max: float                # [m/s]
    blocked: bool               # 지나갈 후보가 하나도 없음 → 상태가 stop_s 를 내야 한다
    candidates: List[Candidate] = field(default_factory=list)
    nearest_s: float = math.nan # 가장 가까운 박스 앞면 s (blocked 일 때 정지 기준)


class AvoidPlanner:
    def __init__(self, car_width_m=1.892, margin_m=0.3, lane_half_m=1.75, d_max_m=3., d_step_m=0.25,
                 horizon_m=40., behind_m=1., w_offset=1., w_line=2., w_switch=3., w_change=0.5,
                 w_left_bias=0.05):
        self.half = car_width_m / 2. + margin_m
        self.car_half = car_width_m / 2.
        self.lane_half = lane_half_m
        self.d_max, self.d_step = d_max_m, d_step_m
        self.horizon, self.behind = horizon_m, behind_m
        self.w_offset, self.w_line, self.w_switch, self.w_change = w_offset, w_line, w_switch, w_change
        self.w_left_bias = w_left_bias        # 좌우 비용이 같을 때 왼쪽 (개발계획서 §4-2 "기본 좌")

    def candidates(self) -> List[float]:
        n = int(round(self.d_max / self.d_step))
        return [round(i * self.d_step, 3) for i in range(-n, n + 1)]

    def relevant(self, boxes: List[Box], ego_s: float) -> List[Box]:
        return [b for b in boxes if b.s_max > ego_s - self.behind and b.s_min < ego_s + self.horizon]

    def line_over(self, d: float) -> float:
        return max(0., abs(d) + self.car_half - self.lane_half)

    def return_is_clear(self, boxes, d, start, length, front, rear):
        """직선 Frenet 구간의 복귀 곡선을 0.2m마다 차체 OBB로 검사한다.

        차폭에는 planner margin이 들어 있고 장애물에는 인식 pad가 들어 있다.
        회전 차체와 박스의 네 분리축을 검사한다. 곡선 구간에서는 호출자가 lead를 사용하지 않는다.
        """
        lo = start-front
        hi = max((b.s_max for b in boxes), default=start)+rear+1.
        count = max(1, int(math.ceil((hi-lo)/.2)))
        for k in range(count+1):
            s = lo+(hi-lo)*k/count
            u = min(1., max(0., (s-start)/length))
            offset = d*(1.-u**3*(10.-15.*u+6.*u*u))
            slope = -d*30.*u*u*(1.-u)**2/length
            yaw = math.atan(slope)
            co, si = math.cos(yaw), math.sin(yaw)
            car = [(s+a*co-w*si, offset+a*si+w*co)
                   for a in (-rear, front) for w in (-self.half, self.half)]
            axes = ((1., 0.), (0., 1.), (co, si), (-si, co))
            for b in boxes:
                box = [(x, y) for x in (b.s_min, b.s_max) for y in (b.d_min, b.d_max)]
                overlap = True
                for ax, ay in axes:
                    cp = [x*ax+y*ay for x, y in car]
                    bp = [x*ax+y*ay for x, y in box]
                    if max(cp) <= min(bp) or max(bp) <= min(cp):
                        overlap = False
                        break
                if overlap:
                    return False
        return True

    def plan(self, boxes: List[Box], ego_s: float, prev_d: float = 0.) -> Plan:
        boxes = self.relevant(boxes, ego_s)
        out: List[Candidate] = []
        for d in self.candidates():
            lo, hi = d - self.half, d + self.half
            hit: Optional[Box] = next((b for b in boxes if lo < b.d_max and hi > b.d_min), None)
            over = self.line_over(d)
            if hit is not None:
                out.append(Candidate(d, False, math.inf, over, f'hits s={hit.s_min:.0f}'))
                continue
            cost = abs(d) * self.w_offset + over * self.w_line + abs(d - prev_d) * self.w_change
            if prev_d and d and (d > 0) != (prev_d > 0):
                cost += self.w_switch
            if d > 0:
                cost -= self.w_left_bias
            out.append(Candidate(d, True, cost, over))
        nearest = min((b.s_min for b in boxes), default=math.nan)
        ok = [c for c in out if c.feasible]
        if not ok:
            return Plan(prev_d, 0., True, out, nearest_s=nearest)
        best = min(ok, key=lambda c: (c.cost, abs(c.d)))
        return Plan(best.d, math.inf, False, out, nearest_s=nearest)


def box_from_event(s: float, d: float, size_x: float, size_y: float, pad_m: float = 0.2) -> Box:
    """Event(중심 s·d, base_link 기준 크기) → Frenet 박스. 방향을 모르니 양쪽에 pad 를 더한다."""
    half_s = max(size_x, 0.5) / 2. + pad_m
    half_d = max(size_y, 0.5) / 2. + pad_m
    return Box(s - half_s, s + half_s, d - half_d, d + half_d)
