"""FSM 코어 — ROS 없는 순수 파이썬 (개발계획서 §2-3, §2-4).

상태 파일 하나 = `BehaviorState` 서브클래스 하나. 각 팀은 `states/<이름>.py` 만 만든다.

    class Avoid(BehaviorState):
        name = 'AVOID'
        priority = 70
        defaults = {'enter_distance_m': 30.}      # ROS 파라미터 `avoid.enter_distance_m` 로 노출됨

        def check_entry(self, ctx) -> bool: ...
        def check_exit(self, ctx) -> bool: ...
        def compute_target(self, ctx) -> Target: ...

코어(`Behavior.tick`) 는 매 틱 모든 상태를 돌려 **걸려 있는(engaged)** 상태 집합을 갱신한다.
    안 걸려 있으면 check_entry → 참이면 건다.   걸려 있으면 check_exit → 참이면 푼다.
그리고 걸린 상태 전부의 compute_target 을 합친다:
    state / lat_offset  = priority 가 가장 높은 상태 것
    stop_s              = 걸린 상태들 중 가장 가까운 것 (min)
    v_max               = 걸린 상태들 중 가장 낮은 것 (min)
이렇게 해야 "빨간불(STOP_LINE, stop_s) + 장애물(AVOID, lat_offset)" 이 14 m 차이로 겹칠 때
둘 다 살아남는다 (§2-4 주의). 상태 하나는 자기 일만 내면 되고 합치는 건 여기서 한다.

시간은 `ctx.now` (초) 만 쓴다. `time.time()` 금지 — bag 재생에서 어긋난다.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

CRUISE = 'CRUISE'


@dataclass
class EventInfo:
    """`foscar_interfaces/Event` 를 map 좌표·Frenet 으로 정리한 것. 노드가 채워서 ctx.events 에 넣는다."""
    type: str
    stamp: float                 # 수신 시각 [s] (ctx.now 와 같은 시계)
    s: float = -1.               # 경로 진행거리 (global) [m]. 모르면 -1
    d: float = math.nan          # 경로에서 옆으로 [m], 좌 +. 모르면 NaN
    x: float = 0.                # map 좌표 [m]
    y: float = 0.
    size_x: float = 0.           # 경로 방향 길이 [m] (base_link x 축 ≈ 진행 방향)
    size_y: float = 0.           # 가로 폭 [m]
    vx: float = 0.               # map 기준 속도 [m/s]
    vy: float = 0.
    value: str = ''
    confidence: float = 1.
    box_yaw: float = math.nan    # map heading of the measured size_x/size_y axes
    path_relevant: bool = True  # false: observation only, cannot enter a driving state

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)


def event_key(e: EventInfo) -> tuple:
    """Lidar identity survives movement and classification changes; legacy sources retain spatial keys."""
    if e.value.startswith('lidar:'):
        return ('track', e.value)
    return (e.type, round(e.s / 2.) if e.s >= 0 else round(e.x / 2.),
            round(e.d) if math.isfinite(e.d) else round(e.y / 2.))


@dataclass
class Context:
    s: float                     # 자차 후륜축 진행거리 (global) [m]
    x: float
    y: float
    yaw: float
    v: float                     # [m/s]
    now: float                   # [s]
    events: List[EventInfo] = field(default_factory=list)   # 만료 안 된 것만
    loc_mode: str = 'GPS'        # 'GPS' | 'DR' (/localization/mode)
    route: object = None         # foscar_control.core.Route (자차가 탄 경로, local s)
    s_offset: float = 0.         # route.s - s_offset = global s
    observations: List[EventInfo] = field(default_factory=list)  # off-path road users
    vehicle_prediction_active: bool = False  # entry gate owns future vehicle conflicts
    vehicle_following_active: bool = False  # committed local entry owns moving-lead following
    measured_clearance_ids: frozenset = field(default_factory=frozenset)
    measured_clearance_stamp: float = -math.inf
    traffic_circle: object = None  # current travel ordering during circular following

    road_map: object = None  # mapped pavement union for tunnel avoidance
    mission_name: str = ''
    mission_speed_mps: Optional[float] = None  # None: legacy standalone mode; inf: no mission cap
    mission_pedestrian: Optional[bool] = None
    ring_tuck: float = 0.        # 0..1: how close behind the ring lead we may tuck (1 while entering the ring)
    lateral_hold_s: float = math.nan  # 다음 신호 정지 자세 (global 후륜축 s). 이 전에는 회피 선회 금지

    def events_of(self, *types: str) -> List[EventInfo]:
        return [e for e in self.events if e.type in types]


@dataclass
class Target:
    """`foscar_interfaces/DrivingTarget` 의 ROS 없는 쌍둥이."""
    state: str = CRUISE
    reference: object = None
    reference_release: bool = False
    v_max: float = math.inf      # [m/s]. inf = 제한 없음
    stop_s: float = math.nan     # 앞바퀴 기준 정지 s (global). NaN = 없음
    lat_offset: float = 0.       # 좌 + [m]
    return_s: float = math.nan
    return_length_m: float = 0.  # <=0: no scheduled return (legacy target)
    entry_s: float = math.nan
    entry_length_m: float = 0.  # <=0: start the transition at the current position
    chain: tuple = ()           # ((d, global start s, length), ...): tunnel slalom, every move at once



class BehaviorState:
    name = 'BASE'
    priority = 0
    defaults: Dict[str, object] = {}     # 파라미터 이름 → 기본값. 노드가 `<name 소문자>.<키>` 로 노출

    def __init__(self, **params):
        for key, value in self.defaults.items():
            setattr(self, key, params.get(key, value))
        unknown = set(params) - set(self.defaults)
        if unknown:
            raise KeyError(f'{self.name}: unknown parameter(s) {sorted(unknown)}')

    def check_entry(self, ctx: Context) -> bool:
        return False

    def check_exit(self, ctx: Context) -> bool:
        return True

    def compute_target(self, ctx: Context) -> Target:
        return Target(state=self.name)

    def debug(self) -> dict:
        """시각화·로그용 부가 정보. 상태가 원하면 덮어쓴다."""
        return {}


class Behavior:
    """상태 집합을 돌리고 Target 을 합친다."""

    def __init__(self, states: List[BehaviorState]):
        names = [s.name for s in states]
        if len(set(names)) != len(names):
            raise ValueError(f'duplicate state names: {names}')
        self._factories = [(type(s), {k: getattr(s,k) for k in s.defaults}) for s in states]
        self.states = sorted(states, key=lambda s: -s.priority)
        self.engaged: Dict[str, bool] = {s.name: False for s in self.states}
        self.last_targets: Dict[str, Target] = {}

    def reset(self):
        """Drop all mission memory, keeping configured parameters. Never await check_exit."""
        self.states = sorted((cls(**params) for cls,params in self._factories), key=lambda s:-s.priority)
        self.engaged = {s.name: False for s in self.states}
        self.last_targets = {}

    def tick(self, ctx: Context, allowed_states=None) -> Target:
        active: List[BehaviorState] = []
        for state in self.states:
            if allowed_states is not None and state.name not in allowed_states:
                self.engaged[state.name] = False
                continue
            if self.engaged[state.name]:
                if state.check_exit(ctx):
                    self.engaged[state.name] = False
                    continue
            elif state.check_entry(ctx):
                self.engaged[state.name] = True
            else:
                continue
            active.append(state)
        self.last_targets = {s.name: s.compute_target(ctx) for s in active}
        if not active:
            return Target()
        top = self.last_targets[active[0].name]
        merged = Target(state=top.state or active[0].name, lat_offset=top.lat_offset,
                        return_s=top.return_s, return_length_m=top.return_length_m,
                        entry_s=top.entry_s, entry_length_m=top.entry_length_m, chain=top.chain)
        merged.v_max = min(t.v_max for t in self.last_targets.values())
        stops = [t.stop_s for t in self.last_targets.values() if math.isfinite(t.stop_s)]
        merged.stop_s = min(stops) if stops else math.nan
        return merged

    def active_names(self) -> List[str]:
        return [s.name for s in self.states if self.engaged[s.name]]
