"""CRUISE — 항상 걸려 있는 바닥 상태. 구간별 속도 상한만 낸다 (팀 3).

규정: 전 구간 60 km/h, 예외구간(s=1119~1878, 고속도로 진입~톨게이트)만 초과 허용.
숫자는 `config/behavior.yaml` 의 `cruise.*` 에 있다. 구간은 세 리스트가 같은 길이로 짝을 이룬다.
"""

import math

from ..ctrl_state import BehaviorState, Context, Target


class Cruise(BehaviorState):
    name = 'CRUISE'
    priority = 0
    defaults = {
        'default_kmh': 60.,                 # 일반 구간 규정 상한
        # 보행자(460)·회전교차로(890) 는 감지거리(실측 ~11 m)가 짧아 20 km/h 아니면 못 선다. 1119~1878 예외구간.
        'zone_s_from': [430., 860., 1119.],
        'zone_s_to': [470., 900., 1878.],
        'zone_kmh': [20., 20., 60.],        # 60 초과는 미검증 (README §3)
    }

    def __init__(self, **params):
        super().__init__(**params)
        if not (len(self.zone_s_from) == len(self.zone_s_to) == len(self.zone_kmh)):
            raise ValueError('cruise: zone_s_from / zone_s_to / zone_kmh must have the same length')

    def check_entry(self, ctx: Context) -> bool:
        return True

    def check_exit(self, ctx: Context) -> bool:
        return False

    def limit_kmh(self, s: float) -> float:
        for a, b, kmh in zip(self.zone_s_from, self.zone_s_to, self.zone_kmh):
            if a <= s <= b:
                return float(kmh)
        return float(self.default_kmh)

    def compute_target(self, ctx: Context) -> Target:
        if ctx.mission_speed_mps is not None:
            return Target(state=self.name, v_max=ctx.mission_speed_mps)
        kmh = self.limit_kmh(ctx.s)
        return Target(state=self.name, v_max=kmh / 3.6 if kmh > 0 else math.inf)
