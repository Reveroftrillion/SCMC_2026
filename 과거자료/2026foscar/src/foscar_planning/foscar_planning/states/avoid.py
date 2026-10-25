"""AVOID — 경로 위 정적 장애물을 옆으로 비켜 지나간다 (팀 2, 개발계획서 §4-2).

진입  OBSTACLE 이벤트가 경로 위(|d| ≤ on_path_d_m) 이고 앞으로 enter_distance_m 이내
목표  lat_offset = AvoidPlanner 가 고른 d. 회피 상태는 주행 속도 상한을 만들지 않는다.
      지나갈 틈이 없으면(blocked) 장애물 앞 stop_gap_m 에 선다 — 부딪히는 것보다 서는 게 싸다.
복귀  최초 앞면에 hidden_length_m를 적용한 상자 뒤끝 + 뒤 오버행 + tail_m를 기준으로 한다.
      직선 구간에서 회전 차체(폭 + margin)의 sweep이 통과할 때만 return_lead_m만큼 앞당긴다.
      선회 시작부터 return_s/return_length_m를 보내 Pure Pursuit가 복귀 곡선을 미리 본다.
탈출  장애물을 지난 뒤에도 예정된 복귀 곡선이 완료될 때까지 계획을 유지한다.

속도와 전이 곡선
    tracker 는 lat_offset 이 바뀐 지점부터 L = max(transition_min_m, v·transition_time_s) 동안
    s 에 대한 smoothstep 곡선으로 옮긴다 (core.py). 그래서 선회 시작 지점은
        장애물 앞면 − front_m − entry_buffer_m − L
    이다. 회피 진입/종료에 따른 속도 상한 변경은 없다.
    속도는 기존 CRUISE·신호·정지 요청과 경로 속도 프로파일이 담당한다.

장애물 기억
    라이다 ROI 가 x ≥ 0 (전방) 이라 **장애물이 옆에 오는 순간 이벤트가 사라진다.** 그때 오프셋을 풀면
    옆구리로 들이받는다. 그래서 한 번 본 장애물은 지나갈 때까지 기억한다. 아직 앞에 있는데
    memory_s 동안 안 보이면 오탐이었다고 보고 잊는다.
"""

import math
from dataclasses import dataclass
from typing import List, Optional

from ..avoid_planner import AvoidPlanner, Box, Plan, box_from_event
from ..state import BehaviorState, Context, Target
from ..mapped_avoidance import paved_offset, paved_moves
from ..avoid_planner import Plan as AvoidPlan
from .. import tunnel_slalom


@dataclass
class Remembered:
    box: Box
    last_seen: float


class Avoid(BehaviorState):
    name = 'AVOID'
    priority = 70
    defaults = {
        'enter_distance_m': 40.,     # 라이다 ROI 40 m. 진입은 일찍, 선회 시작은 아래 게이팅으로 늦춘다
        'hidden_length_m': 3.,       # 박스 최소 길이 [m] — 앞면만 보여도 이 길이가 있다고 본다 (상자 3 m)
        'tail_m': .5,                # 뒤 범퍼가 장애물 뒤를 이만큼 지나면 기억 해제
                                     # (복귀 곡선은 tracker에서 연속적으로 생성)
        'car_length_m': 4.635,       # 후륜축→뒤 범퍼 = car_length_m − front_m
        'on_path_d_m': 1.5,          # 이보다 옆이면 "경로 위" 가 아니다
        'memory_s': 3.,              # 볼 수 있는데 이 시간 동안 안 보이면 잊는다
        # **움직인 적 있는 것은 비켜가지 않는다 — 선다.** 멈춰 선 보행자는 속도가 느려
        # obstacle_node 가 OBSTACLE 로 분류하는데, 그걸 정적 장애물로 보고 옆으로 돌아 나가면
        # 사람 옆을 스치며 지나가게 된다 (2026-09-20 실차: 보행자에게 off=+3.0 요청).
        # 판단 근거는 **Event 의 순간 속도가 아니라 타입** 이다. 정적 박스도 측면이 드러나며
        # 중심점이 점프해 순간 속도가 6.9 m/s 까지 튄다 (실측: 박스 프레임의 27% 가 0.5 m/s 초과)
        # — 속도로 걸렀더니 박스를 못 피하고 들이받았다 (2026-09-20). 반면 PEDESTRIAN/VEHICLE
        # 분류는 히스테리시스 0.8/0.4 + 직진성 + 크기 검사를 거친 **지속적인** 움직임의 결과다.
        'dynamic_hold_s': 5.,        # 움직임이 관측된 자리를 이만큼 기억한다
        'dynamic_match_m': 3.,       # 같은 자리로 볼 거리 (보행자는 그 사이 몇 m 움직인다)
        'match_s_m': 2.,             # 같은 장애물로 볼 s 차이
        'match_d_m': 1.,             # 같은 장애물로 볼 d 차이
        'stop_gap_m': 3.,            # 못 지나갈 때 장애물 앞 여유 (앞바퀴 기준은 tracker 가 처리)
        'return_lead_m': 2.0,       # 차체 sweep이 통과할 때만 복귀 곡선 시작을 앞당김
        'return_min_m': 12.,         # 정지 출발에서도 복귀 곡선이 6m로 짧아지지 않게 함
        'return_time_s': 2.0,       # 미리 전달할 복귀 곡선 길이 = max(min, 진입 속도 × 시간)
        'transition_time_s': 1.5,    # tracker 의 전이 곡선 시간 (low_speed.yaml transition_time_s) 와 같아야 한다
        'transition_min_m': 6.,      # tracker 의 전이 곡선 최소 길이 (low_speed.yaml transition_min_m) 와 같아야 한다
        'front_m': 3.8,              # 후륜축 → 차 앞 범퍼
        'front_axle_m': 3.0,         # 후륜축 → 앞바퀴 (stop_s 기준, core.FRONT_AXLE)
        'entry_preview_m': 10.,     # tracker 최대 주시거리 전에 진입 곡선을 전달
        'entry_buffer_m': 1.5,       # 곡선 완료 외 추종 여유 — 곡선은 장애물 앞 범퍼 기준 이만큼 앞에서 끝난다.
                                     # 6 → 3 → 1.5 로 줄였다 (2026-09-20 실차: 사거리 진입 전에 차로 이탈).
                                     # 이 값은 순수 여유분이다. 0 이면 곡선이 끝나는 순간 범퍼가 장애물 옆이라
                                     # 추종 오차(주시거리 4~10 m 만큼 뒤처짐)를 흡수할 곳이 없다.
        'pad_m': 0.2,                # 박스 양쪽 여유 (Event 크기는 방향을 모른다)
        # When a swerve must fit a shorter room than the normal curve (tunnel: late
        # detection or a slalom re-plan; signal stop line held before the box) the curve
        # uses all the room and speed is capped so peak lateral acceleration stays at this value.
        'swerve_accel_mps2': 4.0,
        # Speed-dependent (user 2026-09-29: steer much harder when slow): swerve_accel_slow_mps2 at
        # swerve_accel_slow_kmh and below, swerve_accel_mps2 at swerve_accel_fast_kmh and above, linear
        # between. The curve itself never exceeds max_curvature_per_m (34.9 deg effective steering on a
        # 3.0 m wheelbase is 0.233 1/m; 80 % of it), whatever the speed.
        'swerve_accel_slow_mps2': 6.0,
        'swerve_accel_slow_kmh': 20.,
        'swerve_accel_fast_kmh': 60.,
        'max_curvature_per_m': 0.18,
        'swerve_accel_sharp_mps2': 7.0,     # lateral acceleration allowed at the steering-limited curvature
        'swerve_sharp_from_per_m': 0.03,    # curvature from which the sharp-curve allowance starts
        # While a swerve is in progress speed rises slowly instead of being frozen: hold_ramp_mps2,
        # growing to hold_ramp_end_mps2 as ramp_to_s (tunnel end) nears; fewer obstacles are left.
        'hold_ramp_mps2': 0.3,
        'hold_ramp_end_mps2': 1.5,
        'ramp_from_s': 1875.,
        'ramp_to_s': 2000.,
        # Static-obstacle mission (not the tunnel): no slow ramp. Speed is free up to the lateral
        # limit of the entry and return curves; the return is sized for the speed reached by
        # accelerating at this rate from the swerve start to the box (user 2026-09-29: the ramp
        # held 19-23 km/h from the signal stop line to s345).
        'return_plan_accel_mps2': 2.0,
        # Tunnel slalom (tunnel_slalom.py): shortest curve for a move between obstacle groups, and the
        # rate the car slows down at ahead of a move so it fits at swerve_accel (user 2026-09-29).
        'tunnel_curve_m': 12.,
        'slalom_decel_mps2': 3.0,
        'slalom_period_s': .5,
        'slowdown_replan': .75,        # below this fraction of the planning speed the curves are made again
        'tunnel_match_m': .6,          # a piece this close (plus the per-metre range term laterally) is that barricade
        'tunnel_match_per_m': .05,     # dead-reckoned heading error ~3 deg: lateral spread grows with range
        'tunnel_closest_pieces': 8,    # the box comes from the pieces seen closest
        'tunnel_extra_margin_m': 0.,   # 0.3 closed the right side past the first barricade (wall) and did
                                       # not help at the last one (2026-09-29)
        # The tracker's pure-pursuit lookahead (low_speed.yaml lookahead_m / _gain / _max_m, keep equal):
        # the slalom checks each curve also taken half a lookahead early (corner cutting).
        'tracker_lookahead_m': 7.,
        'tracker_lookahead_gain': 1.,
        'tracker_lookahead_max_m': 14.,
        'tunnel_piece_m': .3,          # smaller than this alone is not a new obstacle
        'tunnel_box_jitter_m': .16,    # box edge change between plans, p95 (2026-09-29, 5 runs): new curves' slack
        'tunnel_range_slack_per_m': .03,  # + this per metre of range: far boxes read narrow (edges grew ~0.03 m/m, p90,
                                          # every tunnel box 2026-09-30; 1.2 -> 3.5 m wide from 27 to 5 m on box 2)
        'tunnel_edge_quantile': .35,   # box edges from recent pieces (see remember_tunnel_piece)
        # --- AvoidPlanner ---
        'car_width_m': 1.892,
        'margin_m': 0.3,
        'lane_half_m': 1.75,
        'd_max_m': 3.,
        'd_step_m': 0.25,
        'horizon_m': 40.,
        'w_offset': 1.,
        'w_line': 2.,
        'w_switch': 3.,
        'w_change': 0.5,
        'w_left_bias': 0.05,         # 좌우 비용이 같으면 왼쪽 (기본 좌)
    }

    def __init__(self, **params):
        super().__init__(**params)
        self.planner = AvoidPlanner(
            car_width_m=self.car_width_m, margin_m=self.margin_m, lane_half_m=self.lane_half_m,
            d_max_m=self.d_max_m, d_step_m=self.d_step_m, horizon_m=self.horizon_m,
            behind_m=self.tail_m + (self.car_length_m - self.front_m),   # clear_s 와 같은 기준
            w_offset=self.w_offset, w_line=self.w_line, w_switch=self.w_switch, w_change=self.w_change,
            w_left_bias=self.w_left_bias)
        # Tunnel: the pose is dead-reckoned; the last barricade (100 m after GPS loss) was scraped in
        # the same place every run with the nominal margin (2026-09-29), so a little more there.
        self.tunnel_planner = AvoidPlanner(car_width_m=self.car_width_m, margin_m=self.margin_m + self.tunnel_extra_margin_m,
            d_max_m=6., d_step_m=self.d_step_m, horizon_m=self.horizon_m,
            behind_m=self.tail_m+self.car_length_m-self.front_m, w_line=0., w_left_bias=0.)
        self.memory: List[Remembered] = []
        self.dynamic: List[tuple] = []   # (s, d, 마지막 관측 시각) — 최근 움직임이 있던 자리
        self.prev_d = 0.
        self.entry_start_s = math.nan
        self.entry_length_m = 0.
        self.transition_end_s = None # 선회 시작 시 정해진 곡선 끝 s (tracker 와 같은 식)
        self.plan: Plan = None
        self._observed_at = None
        self.lateral_started = False
        self.return_start_s = None
        self.clearance_s = None
        self.return_length_m = 0.
        self.start_offset = 0.
        self.speed_cap = math.inf
        self.held_at_line = False
        self.slalom = None                 # tunnel_slalom.Slalom in force
        self.slalom_hit = None             # (s, shift) where the curve being driven met a box found later
        self.slalom_count = 0
        self.slalom_v = 0.
        self.slalom_at = -math.inf
        self.slalom_origin = 0.
        self.slalom_sides = {}             # group key -> side (+1 left / -1 right) it is passed on
        self.replan_at = -math.inf
        # No acceleration while a swerve is in progress (entry to the end of its return).
        # Ratchets down with the car's speed; kept across slalom re-plans, cleared on exit.
        # 2026-09-27: between two tunnel barricades the car sped up 40 -> 53 km/h while
        # returning, saw the 2nd one only 15 m ahead and hit it. The floor only lets a
        # car that stopped for a blocked side move off again.
        self.hold_v = math.inf
        self.hold_floor_mps = 5.

    # ------------------------------------------------------------------
    def on_path(self, e) -> bool:
        if not (e.s >= 0 and math.isfinite(e.d)):
            return False
        # Also admit a wide body whose edge reaches the car's own width + margin: a 3 m
        # barricade centred at d=2.3 reached d=0.8 and was ignored until the car scraped
        # it (2026-09-26 tunnel).
        half = max(e.size_y, 0.) / 2.
        nearest = 0. if e.d-half < 0. < e.d+half else min(abs(e.d-half), abs(e.d+half))
        return abs(e.d) <= self.on_path_d_m or nearest < self.car_width_m/2. + self.margin_m

    def on_course(self, e) -> bool:
        """On the committed avoidance curve (entry, held offset, return) at the body's station.

        on_path() measures from the reference line only. Once the car drives an avoidance curve, a
        body on that curve matters as much: a barricade in the lane the car had swerved into
        (d=+3.4, car at +3, its return running into it) was never a box and was hit at 57 km/h
        (2026-09-29 TunnelObstaclePractice, s1958).
        """
        if not (self.lateral_started and e.s >= 0 and math.isfinite(e.d)):
            return False
        return abs(e.d - self.offset_now(e.s)) - max(e.size_y, 0.) / 2. < self.car_width_m / 2. + self.margin_m

    def remember_tunnel_piece(self, box: Box, e, now: float, ego_s: float) -> None:
        """Tunnel: one barricade arrives as several tracks and fragments whose size flickers
        (0.1-1.7 m) and whose lateral position jumps (user 2026-09-29: telling the obstacles apart is weak).
        In the tunnel the heading is dead-reckoned: a few degrees off while swerving put one barricade
        (true d 1.86) anywhere between d 1.07 and 2.91 when seen 20-30 m ahead, and it split into three
        tracks that read as a 4 m wall (2026-09-29, the car stopped at s1913).
          - a piece matches a barricade laterally within tunnel_match_m + tunnel_match_per_m x range
          - the box is the robust extent (tunnel_edge_quantile percentiles) of the pieces seen closest,
            which are the accurate ones: nearing a barricade corrects it
          - barricades that come to overlap (same station, touching laterally) are merged
        A tiny piece alone is not new. No guess at what the obstacle is (user 2026-09-29: the barricades
        became boxes long ago; a cluster is just "something there"): no assumed hidden length, no size
        cap, no pad; the box is what the lidar saw."""
        rng = max(0., e.s - ego_s)
        def centre(b: Box):
            return (b.s_min + b.s_max) / 2., (b.d_min + b.d_max) / 2.
        cs, cd = centre(box)
        g = self.tunnel_match_m
        gd = g + self.tunnel_match_per_m * rng
        near = [m for m in self.memory
                if abs(centre(m.box)[0] - cs) <= g + (m.box.s_max - m.box.s_min) / 2.
                and abs(centre(m.box)[1] - cd) <= gd + (m.box.d_max - m.box.d_min) / 2.]
        pieces = self.__dict__.setdefault('_pieces', {})
        if not near:
            if max(e.size_x, e.size_y) < self.tunnel_piece_m:
                return
            m = Remembered(box, now)
            self.memory.append(m)
        else:
            m = min(near, key=lambda x: abs(centre(x.box)[0] - cs) + abs(centre(x.box)[1] - cd))
            m.last_seen = now
            for other in near:
                if other is not m and self.touching(other.box, m.box):
                    pieces.setdefault(id(m), []).extend(pieces.pop(id(other), []))
                    self.memory.remove(other)
        own = pieces.setdefault(id(m), [])
        own.append((rng, box, now))
        del own[:-30]
        closest = [x[1] for x in sorted(own, key=lambda x: x[0])[:self.tunnel_closest_pieces]]
        def q(values, frac):
            v = sorted(values)
            return v[min(len(v) - 1, int(frac * (len(v) - 1) + .5))]
        lo, hi = self.tunnel_edge_quantile, 1. - self.tunnel_edge_quantile
        s_min = q([b.s_min for b in closest], lo)
        m.box = Box(s_min, max(q([b.s_max for b in closest], hi), s_min),
                    q([b.d_min for b in closest], lo), q([b.d_max for b in closest], hi))

    def seen_now(self, ctx: Context, m) -> Optional[Box]:
        """What the lidar shows of a remembered obstacle right now: the union of the pieces of its latest
        scan, when that scan is recent (within a re-plan period) and the obstacle is still ahead of the front
        bumper (beside the car only a side is seen). None when it cannot be checked now."""
        if ctx.now - m.last_seen > self.slalom_period_s or ctx.s + self.front_m >= m.box.s_min:
            return None
        latest = [x[1] for x in self.__dict__.get('_pieces', {}).get(id(m), []) if x[2] >= m.last_seen - 1e-9]
        if not latest:
            return None
        return Box(min(b.s_min for b in latest), max(b.s_max for b in latest),
                   min(b.d_min for b in latest), max(b.d_max for b in latest))

    def merge_touching(self) -> None:
        """One barricade tracked twice (e.g. d -0.95..0.75 at s1887.8 and 0.67..1.68 at s1889.8) read as two
        obstacles in different places and stopped the car (2026-09-29): merge touching tracks."""
        pieces = self.__dict__.setdefault('_pieces', {})
        merged = True
        while merged:
            merged = False
            for i, a in enumerate(self.memory):
                for b in self.memory[i+1:]:
                    if self.touching(a.box, b.box):
                        own = pieces.setdefault(id(a), [])
                        own.extend(pieces.pop(id(b), []))
                        a.box = Box(min(a.box.s_min, b.box.s_min), max(a.box.s_max, b.box.s_max),
                                    min(a.box.d_min, b.box.d_min), max(a.box.d_max, b.box.d_max))
                        a.last_seen = max(a.last_seen, b.last_seen)
                        self.memory.remove(b)
                        merged = True
                        break
                if merged:
                    break

    @staticmethod
    def touching(a: Box, b: Box, gap: float = .3) -> bool:
        return (a.s_min - 1. < b.s_max and b.s_min - 1. < a.s_max
                and a.d_min - gap < b.d_max and b.d_min - gap < a.d_max)

    def recently_moved(self, e, now: float) -> bool:
        """이 자리에서 최근에 움직임이 관측됐는가 — 그렇다면 정적 장애물이 아니다."""
        return any(now - t <= self.dynamic_hold_s
                   and abs(e.s - ds) <= self.dynamic_match_m
                   and abs(e.d - dd) <= self.dynamic_match_m
                   for ds, dd, t in self.dynamic)

    def observe(self, ctx: Context) -> None:
        """이번 틱의 OBSTACLE 이벤트를 기억에 합치고, 지났거나 사라진 것을 지운다. 틱당 한 번."""
        if self._observed_at == ctx.now:
            return
        self._observed_at = ctx.now
        # 도로 이용자로 분류된 자리를 적어 둔다 (그 보행자가 멈추면 OBSTACLE 로 바뀌어 들어온다).
        # The static-obstacle mission only has a parked box. At 59 km/h with 8 Hz lidar the box's
        # track jittered into DYNAMIC_UNKNOWN, was never avoided and was hit (2026-09-27, s325).
        # There, an unclassified mover on the path is still a box to go around.
        # The tunnel has no pedestrians either (missions.yaml): a barricade tracked as PEDESTRIAN there
        # was dropped as a mover and never avoided (2026-09-29 TunnelObstaclePractice, s1961).
        static_zone = getattr(ctx, 'mission_name', '') in ('STATIC_OBSTACLE', 'TUNNEL')
        # 2026-09-28 full lap: approaching the same box slowly for a red light, it was tracked as
        # PEDESTRIAN (0.2-0.4 m/s centroid drift) and the car drove into it. Same rule as above.
        tunnel = getattr(ctx, 'mission_name', '') == 'TUNNEL'
        # Tunnel: every track is a barricade except a PEDESTRIAN, which obstacle_node there labels by shape only
        # (tall and narrow), never by tracker speed: a barricade tracked as VEHICLE was dropped as a mover and
        # pushed (2026-09-29). The pedestrian is YIELD's (2026-09-30 T1_CP14_Dyn).
        movers = ('PEDESTRIAN',) if tunnel else ('VEHICLE',) if static_zone else ('PEDESTRIAN', 'VEHICLE', 'DYNAMIC_UNKNOWN')
        for e in ctx.events_of(*movers):
            if e.s >= 0 and math.isfinite(e.d):
                self.dynamic.append((e.s, e.d, ctx.now))
        self.dynamic = [x for x in self.dynamic if ctx.now - x[2] <= self.dynamic_hold_s]
        kinds = (('OBSTACLE', 'DYNAMIC_UNKNOWN', 'VEHICLE') if tunnel
                 else ('OBSTACLE', 'DYNAMIC_UNKNOWN', 'PEDESTRIAN') if static_zone else ('OBSTACLE',))
        for e in ctx.events_of(*kinds):
            # Tunnel: every body on the pavement, since the slalom chooses a side for each of them at once.
            anywhere = ctx.mission_name == 'TUNNEL' and math.isfinite(e.d) and abs(e.d) <= self.tunnel_planner.d_max
            if not (anywhere or self.on_path(e) or self.on_course(e)) or self.recently_moved(e, ctx.now):
                continue
            box = box_from_event(e.s, e.d, e.size_x, e.size_y, 0. if tunnel else self.pad_m)
            if tunnel:
                self.remember_tunnel_piece(box, e, ctx.now, ctx.s)
                continue
            # 라이다는 앞면만 본다 → 박스 길이를 최소 hidden_length_m 로 본다 (s_min 기준).
            # 옆을 지나며 측면이 보여 size_x 가 커지면(0.3→1.7) 이미 실제 길이가 들어온 것이라
            # 더 늘리지 않는다 — 앞면에 더하기만 하면 뒤쪽이 2.5 m 이중 계산돼 복귀가 늦었다 (2026-09-20).
            # 플래너 충돌검사·기억·복귀가 전부 이 "실제 크기" 박스를 쓴다.
            for m in self.memory:
                if (abs((m.box.s_min + m.box.s_max) / 2 - e.s) <= self.match_s_m
                        and abs((m.box.d_min + m.box.d_max) / 2 - e.d) <= self.match_d_m):
                    front = min(m.box.s_min, box.s_min)
                    # 측면이 드러나도 최초 앞면에 붙인 숨은 길이를 다시 뒤로 밀지 않는다.
                    m.box = Box(front, max(m.box.s_max, box.s_max, front + self.hidden_length_m),
                                box.d_min, box.d_max)
                    m.last_seen = ctx.now
                    break
            else:
                box = Box(box.s_min, max(box.s_max, box.s_min + self.hidden_length_m), box.d_min, box.d_max)
                self.memory.append(Remembered(box, ctx.now))
        if tunnel:
            self.merge_touching()
        keep = []
        for m in self.memory:
            mid_s, mid_d = (m.box.s_min + m.box.s_max) / 2., (m.box.d_min + m.box.d_max) / 2.
            if any(ctx.now - t <= self.dynamic_hold_s
                   and abs(mid_s - ds) <= self.dynamic_match_m + self.hidden_length_m
                   and abs(mid_d - dd) <= self.dynamic_match_m
                   for ds, dd, t in self.dynamic):
                continue                                  # 움직이는 것 옆으로 돌아 나가지 않는다
            passed = ctx.s > self.clear_s(m.box)
            # 기억이 필요한 이유는 **라이다 ROI 가 전방뿐** 이라는 것 하나다: 물체가 옆에 오면
            # 안 보이는데 그때 잊으면 옆구리로 들이받는다. 그러니 기억은 **볼 수 없는 동안만**
            # 유지한다. 앞 범퍼가 아직 상자 앞면에 못 닿았으면 그 물체는 지금 보이는 자리다 —
            # 안 잡히면 진짜 없어진 것이다 (건너간 보행자).
            # 예전 조건(`ctx.s < s_min − stop_gap` 일 때만 잊음)은 장애물 3 m 앞에 멈춰 서면
            # 영원히 못 잊어서, 보행자가 건너가도 박스가 남고 차가 못 가는 교착이 났다 (2026-09-20).
            blind = ctx.s + self.front_m > m.box.s_min
            stale = ctx.now - m.last_seen > self.memory_s
            if passed or (stale and not blind):
                continue
            keep.append(m)
        self.memory = keep

    @property
    def rear_overhang_m(self) -> float:
        return self.car_length_m - self.front_m

    def clear_s(self, box: Box) -> float:
        """뒤 범퍼 + tail 여유가 완전히 빠지는 기억 해제 기준 (global rear-axle s).

        복귀 곡선은 이 위치를 기준으로 하되, 차체 sweep 통과 시 일부 앞당겨 시작한다.
        기억 삭제와 복귀 곡선 종료는 분리되어 있어 장애물이 사라져도 경로가 끊기지 않는다.
        """
        return box.s_max + self.tail_m + self.rear_overhang_m

    def boxes(self) -> List[Box]:
        return [m.box for m in self.memory]

    # ------------------------------------------------------------------
    def check_entry(self, ctx: Context) -> bool:
        if ctx.vehicle_following_active:
            return False  # committed merge keeps its route; close bodies use the braking guard
        self.observe(ctx)
        return any(0. < m.box.s_min - ctx.s <= self.enter_distance_m or m.box.s_min <= ctx.s <= m.box.s_max
                   for m in self.memory)

    def check_exit(self, ctx: Context) -> bool:
        self.observe(ctx)
        if self.memory:
            return False
        if ctx.mission_name == 'TUNNEL' and self.slalom is not None and abs(self.offset_now(ctx.s)) > .05:
            return False  # back to 0 is the slalom's own last move
        if (self.lateral_started and self.return_start_s is not None
                and ctx.s <= self.return_start_s+self.return_length_m):
            return False  # 박스가 뒤로 사라져도 미리 보낸 복귀 곡선은 끝까지 유지
        self.reset_manoeuvre()
        self.dynamic = []
        self.hold_v = math.inf
        return True

    def held_speed(self, ctx: Context) -> float:
        if ctx.mission_name == 'STATIC_OBSTACLE':
            # Lateral limit of the committed return curve only (no ramp in the box mission).
            if self.return_length_m <= 0.:
                return math.inf
            return self.swerve_speed(self.return_length_m, abs(self.prev_d))
        # Frozen at the measured speed each tick, the hold ratcheted 41 -> 29 km/h through the
        # tunnel slalom (s1912-1994) and the car then launched at pedal 0.8 on exit (TuningPoint,
        # 2026-09-28). Rise slowly instead, faster as the tunnel end nears, and never more than
        # one second of that rise above the actual speed (the 40 -> 53 km/h surge that hit the
        # second barricade stays impossible).
        v=max(ctx.v,self.hold_floor_mps)
        span=max(1e-6,self.ramp_to_s-self.ramp_from_s)
        frac=min(1.,max(0.,(ctx.s-self.ramp_from_s)/span))
        rate=self.hold_ramp_mps2+(self.hold_ramp_end_mps2-self.hold_ramp_mps2)*frac
        now=getattr(ctx,'now',0.)
        if not math.isfinite(self.hold_v):
            self.hold_v,self.hold_t=v,now
        else:
            dt=max(0.,now-getattr(self,'hold_t',now));self.hold_t=now
            self.hold_v=min(self.hold_v+rate*dt,v+rate*1.)
        return self.hold_v

    def reset_manoeuvre(self, start_offset: float = 0.) -> None:
        self.prev_d, self.plan, self.transition_end_s = 0., None, None
        self.entry_start_s, self.entry_length_m = math.nan, 0.
        self.lateral_started = False
        self.return_start_s = None
        self.clearance_s = None
        self.return_length_m = 0.
        self.start_offset = start_offset   # physical offset the next swerve starts from
        self.speed_cap = math.inf
        self.held_at_line = False          # waited for a signal stop line before this swerve
        self.slalom = None
        self.slalom_hit = None
        self.slalom_count = 0
        self.slalom_v = 0.
        self.slalom_at = -math.inf
        self.slalom_origin = start_offset
        self.slalom_sides = {}

    @staticmethod
    def slalom_key(group) -> int:
        return int(round(min(b.s_min for b in group)/3.))

    def slalom_side(self, group, offsets) -> int:
        """The side a group is passed on, chosen once: the side with more room between the box and the last
        paved offset. Re-chosen from scratch every plan, the side followed each new piece of a box seen from afar:
        right at 27 m (only its middle seen), left 3 m later (its right part appeared), then right again, and the
        car stopped in front of it (2026-09-30, 1 of 3 runs and both stopped full laps)."""
        key = self.slalom_key(group)
        if key not in self.slalom_sides and offsets:
            # Room from the box's centre, not its edges: a box seen from afar shows only its part near the path and
            # grows toward the wall; measured from that edge the wall side looked roomier (+3.5 chosen past the
            # first box, which reaches the left wall) and the car hit it (2026-09-30 run 115558).
            centre = (min(b.d_min for b in group)+max(b.d_max for b in group))/2.
            self.slalom_sides[key] = 1 if max(offsets)-centre >= centre-min(offsets) else -1
            print('SLALOM SIDE s_box=%.1f d=[%.2f, %.2f] offsets=[%.2f, %.2f] -> %+d' % (
                min(b.s_min for b in group), min(b.d_min for b in group), max(b.d_max for b in group),
                min(offsets), max(offsets), self.slalom_sides[key]), flush=True)
        return self.slalom_sides.get(key, 0)

    def offset_now(self, s: float) -> float:
        """Offset the tracker is following at s (same quintic as core.py)."""
        def w(u):
            u = min(1., max(0., u))
            return u**3*(10.-15.*u+6.*u*u)
        if not self.lateral_started:
            return self.start_offset
        if self.return_start_s is not None and s >= self.return_start_s and self.return_length_m > 0:
            return self.prev_d*(1.-w((s-self.return_start_s)/self.return_length_m))
        if self.entry_length_m > 0 and math.isfinite(self.entry_start_s):
            return self.start_offset+(self.prev_d-self.start_offset)*w((s-self.entry_start_s)/self.entry_length_m)
        return self.prev_d

    def transition_length(self, v: float) -> float:
        """tracker 가 이 속도에서 쓸 전이 곡선 길이 (core.py 와 같은 식)."""
        return max(self.transition_min_m, max(0., v) * self.transition_time_s)

    def swerve_curvature(self, length: float, shift: float) -> float:
        """Peak curvature of a quintic shift over this length."""
        return 5.77*max(shift, 1e-6)/max(length, 1e-6)**2

    def swerve_speed(self, length: float, shift: float) -> float:
        """Highest v with v^2 * curvature <= swerve accel at v (higher when slow, see defaults).
        A curve sharper than the steering allows gets a crawl (0.5 m/s); planners avoid choosing one."""
        k = self.swerve_curvature(length, shift)
        if k > self.max_curvature_per_m:
            return .5
        lo, hi = self.swerve_accel_slow_kmh/3.6, self.swerve_accel_fast_kmh/3.6
        a_lo, a_hi = self.swerve_accel_slow_mps2, self.swerve_accel_mps2
        # Sharp curves (barricades close together) are driven slower but with more lateral acceleration:
        # swerve_accel_mps2 up to curvature swerve_sharp_from_per_m, rising linearly to swerve_accel_sharp_mps2
        # at the steering limit (user 2026-09-29). The speed rule above applies too; the larger one counts.
        span = max(1e-6, self.max_curvature_per_m - self.swerve_sharp_from_per_m)
        a_k = self.swerve_accel_mps2 + (self.swerve_accel_sharp_mps2 - self.swerve_accel_mps2)*min(
            1., max(0., (k - self.swerve_sharp_from_per_m)/span))
        v_k = math.sqrt(a_k/k)
        if math.sqrt(a_lo/k) <= lo:
            return max(v_k, math.sqrt(a_lo/k))
        if math.sqrt(a_hi/k) >= hi:
            return max(v_k, math.sqrt(a_hi/k))
        m = (a_lo-a_hi)/(hi-lo)                     # accel = a_lo - m (v - lo) between lo and hi
        return max(v_k, (-m+math.sqrt(m*m+4.*k*(a_lo+m*lo)))/(2.*k))

    def planned_return_length(self, ctx: Context, d: float) -> float:
        v = ctx.v
        if ctx.mission_name == 'STATIC_OBSTACLE' and self.plan is not None and math.isfinite(self.plan.nearest_s):
            run = max(0., self.plan.nearest_s - ctx.s) + self.car_length_m
            v = math.sqrt(max(0., ctx.v)**2 + 2.*self.return_plan_accel_mps2*run)
            limit = getattr(ctx, 'mission_speed_mps', None)
            if limit is not None and math.isfinite(limit):
                v = min(v, limit)
            v = max(v, ctx.v)
        length = max(self.return_min_m, self.transition_min_m, v*self.return_time_s)
        if ctx.mission_name == 'TUNNEL':
            # Quintic peak curvature scales with |d| / L². Preserve the
            # curvature of a lane-half-width shift when using wider pavement.
            length = max(20., length) * math.sqrt(max(1., abs(d)/self.lane_half_m))
        return length

    def slalom_offsets(self, ctx: Context, group: List[Box]) -> List[float]:
        """Offsets passing the group with body + margin on mapped/walled pavement (cached)."""
        s0 = min(b.s_min for b in group) - self.front_m
        s1 = max(b.s_max for b in group) + self.rear_overhang_m
        cache = self.__dict__.setdefault('_paved', {})
        if len(cache) > 1024:
            cache.clear()
        out = []
        for c in self.tunnel_planner.plan(group, min(ctx.s, s0)).candidates:
            if not c.feasible:
                continue
            key = (getattr(ctx.road_map, 'cache_key', id(ctx.road_map)), id(ctx.route), ctx.s_offset, c.d,
                   round(s0, 1), round(s1, 1))
            if key not in cache:
                cache[key] = paved_offset(ctx, c.d, s0, s1, self.tunnel_planner.half)
            if cache[key]:
                out.append(c.d)
        return out

    def tunnel_target(self, ctx: Context) -> Target:
        """Tunnel: re-plan the whole slalom, back to 0 included, from where the car is whenever the
        obstacle picture changes (a body appears or is passed, the offset held now stops passing its
        group) and twice a second, throwing the old curve away (user 2026-09-29). The car follows the
        plan's moves one after another; no separate return is scheduled."""
        d_now = self.offset_now(ctx.s)
        # One group per barricade: every move is checked by the body sweep, so two close barricades need
        # not share an offset. Forcing one (A and B 10 m apart) left no reachable offset and the car ran
        # into B (2026-09-29).
        groups = (tunnel_slalom.group_boxes(self.boxes(), self.clear_s, self.front_m, self.entry_buffer_m,
                                            self.tunnel_curve_m, lambda g: False)
                  if self.memory else [])
        held = self.slalom
        # Slowed down well below the speed the plan was made at: every curve, the one under way included,
        # is planned again for the speed now (user 2026-09-29: curves made fast stayed long).
        slowed = held is not None and self.slalom_v > 3. and ctx.v < self.slowdown_replan*self.slalom_v
        changed = (held is None or slowed or len(self.memory) != self.slalom_count
                   or (held.clears and ctx.s > held.clears[0])
                   or (groups and all(abs(self.prev_d - d) >= .5*self.tunnel_planner.d_step
                                      for d in self.slalom_offsets(ctx, groups[0]))))
        due = ctx.now - self.slalom_at >= self.slalom_period_s
        if changed or due:
            self.slalom_at = ctx.now
            if held is not None and held.offsets and groups:
                old = self.slalom_moves(held, ctx.s)
                horizon = max(b.s_max for g in groups for b in g) + self.rear_overhang_m
                hit = tunnel_slalom.first_hit([b for g in groups for b in g], old, self.slalom_origin, ctx.s,
                                              min(horizon, ctx.s + 60.), self.front_m, self.rear_overhang_m,
                                              self.tunnel_planner.half)
                if hit is not None and (self.slalom_hit is None or hit[0] < self.slalom_hit[0]):
                    # lateral move that takes the body off that box, from the offset the curve had there
                    s_h, d_h, b = hit
                    half = self.tunnel_planner.half
                    shift = min(d_h + half - b.d_min, b.d_max - (d_h - half))
                    self.slalom_hit = (s_h, max(shift, 0.))
                    print('SLALOM HIT s=%.1f (%.1f m ahead) shift %.2f v=%.1f' % (s_h, s_h - ctx.s, shift, ctx.v*3.6),
                          flush=True)
            ongoing = None
            if (not slowed and self.lateral_started and self.entry_length_m > 0 and math.isfinite(self.entry_start_s)
                    and ctx.s < self.entry_start_s + self.entry_length_m):
                ongoing = (self.start_offset, self.prev_d, self.entry_start_s, self.entry_length_m)

            fresh_all = [self.seen_now(ctx, m) or m.box for m in self.memory]

            def drivable(length, shift, start):
                # slowing at slalom_decel_mps2 (brake 0.2, measured 3.1 m/s2) from now to the move's start
                v2 = max(ctx.v, 0.)**2 - 2.*self.slalom_decel_mps2*max(0., start - ctx.s)
                return v2 <= self.swerve_speed(length, shift)**2 + 1e-6

            def plan_over(groups_, slack, one_side=True):
                rejected = set()
                def feasible(g):
                    out = self.slalom_offsets(ctx, g)
                    if one_side:
                        side = self.slalom_side(g, out)
                        kept = [d for d in out if side == 0 or abs(d) < 1e-9 or d*side > 0]
                        out = kept or out
                    return [d for d in out if d not in rejected] if groups_ and g is groups_[0] else out
                for _ in range(8):
                    new_ = tunnel_slalom.plan(groups_, feasible, ctx.s, d_now, max(ctx.v, 0.), self.clear_s, self.front_m,
                                              self.rear_overhang_m, self.tunnel_planner.half, self.tunnel_curve_m,
                                              self.transition_time_s, self.transition_min_m, self.max_curvature_per_m,
                                              ongoing, .5*min(max(self.tracker_lookahead_m, self.tracker_lookahead_gain*ctx.v),
                                                              self.tracker_lookahead_max_m),
                                              hold_tol=.5*self.tunnel_planner.d_step, slack=slack,
                                              ongoing_boxes=fresh_all, speed_ok=drivable,
                                              range_slack=self.tunnel_range_slack_per_m)
                    moves = [(d, st, ln) for d, st, ln, sh in zip(new_.offsets, new_.starts, new_.lengths, new_.shifts)
                             if sh > 1e-6]
                    end = max([st + ln for _, st, ln in moves] + [ctx.s]) + self.front_m
                    if not moves or paved_moves(ctx, d_now, moves, 5.*math.floor(ctx.s/5.), end, self.front_m,
                                                self.rear_overhang_m, self.tunnel_planner.half):
                        break
                    if not groups_:
                        break
                    rejected.add(new_.offsets[0])
                if slack > 0 and new_.blocked_at is not None:
                    return plan_over(groups_, 0., one_side)   # no curve with slack: the tightest that still clears
                if one_side and new_.blocked_at is not None:
                    # Nothing on the chosen side any more: either side, and the side taken is the group's from now.
                    other = plan_over(groups_, slack, False)
                    for g2, d2 in zip(groups_, other.offsets):
                        if abs(d2) > 1e-9:
                            self.slalom_sides[self.slalom_key(g2)] = 1 if d2 > 0 else -1
                    return other
                if new_.blocked_at is not None and new_.blocked_at < len(groups_):
                    g = groups_[new_.blocked_at]
                    print('SLALOM BLOCKED at group %d s=%.1f d_now=%+.3f feasible=%s rejected=%s' % (
                        new_.blocked_at, ctx.s, d_now, self.slalom_offsets(ctx, g), sorted(rejected)), flush=True)
                return new_

            new = plan_over(groups, self.tunnel_box_jitter_m)
            # The curve wanted is ruled out (blocked, or only a steeper one is left): doubt the remembered boxes
            # and plan again, the same way, over what the lidar shows of them right now (user 2026-09-29: the
            # remembered extent stays larger than the obstacle). Obstacles not in view keep their box.
            if groups and self.memory:
                fresh = fresh_all
                if any(f is not m.box for f, m in zip(fresh, self.memory)):
                    fresh_groups = tunnel_slalom.group_boxes(fresh, self.clear_s, self.front_m, self.entry_buffer_m,
                                                             self.tunnel_curve_m, lambda g: False)
                    alt = plan_over(fresh_groups, self.tunnel_box_jitter_m)
                    if alt.blocked_at is None and (new.blocked_at is not None or alt.steepest < new.steepest - 1e-9):
                        print('SLALOM VERIFIED s=%.1f remembered %s -> seen now %s: blocked %s->None steepest %.4f->%.4f' % (
                            ctx.s, [(round(m.box.d_min, 2), round(m.box.d_max, 2)) for m in self.memory],
                            [(round(f.d_min, 2), round(f.d_max, 2)) for f in fresh], new.blocked_at,
                            new.steepest, alt.steepest), flush=True)
                        new, groups = alt, fresh_groups
            adopt = changed or (held is not None and new.offsets and held.offsets
                                and abs(new.offsets[0] - held.offsets[0]) > 1e-6
                                and (new.steepest < .7*held.steepest
                                     or (held.blocked_at is not None and new.blocked_at is None)))
            if adopt:
                first = self.slalom_moves(new, ctx.s)[:1]
                if not (ongoing and first and abs(first[0][1] - ongoing[2]) < 1e-6 and abs(first[0][2] - ongoing[3]) < 1e-6):
                    self.slalom_origin = d_now          # a new first move starts from the offset now
                else:
                    self.slalom_origin = ongoing[0]     # the move being driven goes on unchanged
                self.slalom, self.slalom_count = new, len(self.memory)
                self.slalom_v = max(ctx.v, 0.)
                if new.blocked_at is None:
                    # A curve around the box that turned up is adopted: its own speed limits (the late-braking
                    # approach included) apply from now. Kept, the hit cap crawled at 0.5 m/s past a box the
                    # new curve already cleared.
                    self.slalom_hit = None
                print('SLALOM s=%.1f d_now=%+.2f v=%.1f groups=%s offsets=%s starts=%s lengths=%s blocked=%s' % (
                    ctx.s, d_now, ctx.v*3.6, [[(round(b.s_min, 1), round(b.d_min, 2), round(b.d_max, 2)) for b in g2]
                                              for g2 in groups], new.offsets, [round(x, 1) for x in new.starts],
                    [round(x, 1) for x in new.lengths], new.blocked_at), flush=True)
        return self.slalom_output(ctx, groups)

    @staticmethod
    def slalom_moves(plan, s):
        """The plan's real moves (d, start, length), dropping holds."""
        return [(d, st, ln) for d, st, ln, sh in zip(plan.offsets, plan.starts, plan.lengths, plan.shifts) if sh > 1e-6]

    def approach_speed(self, ctx: Context, fit: float, dist: float) -> float:
        """Speed allowed now for a curve driveable at fit that starts dist ahead: slowing at
        slalom_decel_mps2 (brake 0.2, measured 3.1 m/s2). Already too fast for that (the curve came
        into the plan late, e.g. the second barricade seen 25 m ahead at 47 km/h): fit itself at once, so
        the tracker brakes now instead of trailing a cap it cannot follow (2026-09-29: entered the
        curve between the first two barricades at 29-38 km/h where it allowed 25, 4 of 5 runs stopped)."""
        ramp = math.sqrt(fit*fit + 2.*self.slalom_decel_mps2*max(0., dist))
        return fit if max(ctx.v, 0.) > ramp else ramp

    def slalom_output(self, ctx: Context, groups) -> Target:
        plan = self.slalom
        moves = self.slalom_moves(plan, ctx.s)
        # The move being driven: the last one already started (or about to, within the entry preview);
        # its predecessor gives the offset it starts from.
        k = None
        for i, (d, st, ln) in enumerate(moves):
            if st <= ctx.s + self.entry_preview_m and (i == 0 or ctx.s >= moves[i-1][1] + moves[i-1][2]):
                k = i
        origin = self.slalom_origin if k is None or k == 0 else moves[k-1][0]
        if k is not None:
            d1, st, ln = moves[k]
            self.start_offset, self.prev_d = origin, d1
            self.entry_start_s, self.entry_length_m = st, ln
            self.transition_end_s = st + ln
        self.lateral_started = abs(self.prev_d) > 1e-6 or abs(self.start_offset) > 1e-6
        self.return_start_s, self.return_length_m = None, 0.
        self.clearance_s = plan.clears[0] if plan.clears else None
        if groups:
            # For the side explanation (avoid_choice): the first group's candidates, marked by the slalom.
            ok = set(self.slalom_offsets(ctx, groups[0]))
            shown = self.tunnel_planner.plan(groups[0], min(ctx.s, min(b.s_min for b in groups[0])), self.prev_d)
            for c in shown.candidates:
                if c.feasible and c.d not in ok:
                    c.feasible, c.reason = False, 'outside mapped pavement or map unavailable'
            self.plan = AvoidPlan(self.prev_d, math.inf, plan.blocked_at == 0, shown.candidates, shown.nearest_s)
        if groups and plan.blocked_at == 0:           # the first group cannot be passed from here
            # Stop before it, as close as the front wheels still allow. Only a barricade the front bumper
            # has already reached is "passed" (no braking beside it): judging that by the stop point
            # (3 m short of it) released the brake and the car drove into the second barricade (2026-09-29).
            front = min(b.s_min for b in groups[0])
            passed = ctx.s + self.front_m >= front
            stop_s = max(front - self.stop_gap_m, ctx.s + self.front_axle_m + .3)
            return Target(state=self.name, lat_offset=self.prev_d, entry_s=self.entry_start_s,
                          entry_length_m=self.entry_length_m, stop_s=stop_s, v_max=math.inf if passed else 0.)
        # Every move still ahead at the swerve limit on its curve, slowing down for it in advance.
        cap = math.inf
        for d, st, ln in moves:
            if ctx.s < st + ln:
                prev = moves[moves.index((d, st, ln)) - 1][0] if moves.index((d, st, ln)) else self.slalom_origin
                cap = min(cap, self.approach_speed(ctx, self.swerve_speed(ln, abs(d - prev)), st - ctx.s))
        # A box turned up on the curve that was being driven (user 2026-09-29): no faster than a fresh
        # swerve off it within the distance left to where the body would meet it allows, the same swerve
        # model as every other curve. The closer it was found, the slower.
        if self.slalom_hit is not None:
            s_h, shift = self.slalom_hit
            if ctx.s >= s_h:
                self.slalom_hit = None
            elif shift > 1e-6:
                cap = min(cap, self.approach_speed(ctx, self.swerve_speed(max(s_h - ctx.s, self.transition_min_m), shift),
                                                   s_h - ctx.s - self.front_m))
        if self.lateral_started:
            cap = min(cap, self.held_speed(ctx))       # no speeding up mid-slalom (2026-09-27 rule)
        stop = {}
        if plan.blocked_at is not None and plan.blocked_at < len(plan.groups):
            stop = {'stop_s': min(b.s_min for b in plan.groups[plan.blocked_at]) - self.stop_gap_m}
        # The whole slalom to the tracker at once, back to 0 included (user 2026-09-29: the next curve
        # used to appear only after the previous one was done).
        chain = tuple((d, st, ln) for d, st, ln in moves if st + ln > ctx.s)
        return Target(state=self.name, lat_offset=self.prev_d, v_max=cap, entry_s=self.entry_start_s,
                      entry_length_m=self.entry_length_m, chain=chain, **stop)

    def held_target(self, **kwargs) -> Target:
        return Target(state=self.name, lat_offset=self.prev_d,
                      entry_s=self.entry_start_s, entry_length_m=self.entry_length_m,
                      return_s=self.return_start_s if self.return_start_s is not None else math.nan,
                      return_length_m=self.return_length_m, **kwargs)

    def compute_target(self, ctx: Context) -> Target:
        if ctx.mission_name == 'TUNNEL':
            return self.tunnel_target(ctx)
        if not self.memory and self.lateral_started and self.return_start_s is not None:
            return Target(state=self.name, lat_offset=self.prev_d, v_max=self.held_speed(ctx),
                          return_s=self.return_start_s, return_length_m=self.return_length_m,
                          entry_s=self.entry_start_s, entry_length_m=self.entry_length_m)
        self.plan = self.planner.plan(self.boxes(), ctx.s, self.prev_d)
        if self.plan.blocked:
            stop_s = self.plan.nearest_s - self.stop_gap_m
            # A stop point already behind the front wheels cannot be reached; braking
            # beside the obstacle only risks a deadlock. Hold the committed offset
            # instead, as the ordinary AVOID (whose stop_s is then ignored) does.
            # 2026-09-26 sim loop: v_max=0 beside a passed barricade for 0.8 s.
            passed = stop_s < ctx.s + self.front_axle_m
            return self.held_target(stop_s=stop_s, v_max=math.inf)
        hold = ctx.lateral_hold_s
        if (not self.lateral_started and ctx.mission_name != 'TUNNEL' and math.isfinite(hold)
                and ctx.s < hold < self.plan.nearest_s and abs(self.plan.d) > 1e-6):
            # 신호 정지선이 상자 앞에 있으면 선에 닿을 때까지 직진한다: 정지선에 바르게 서고
            # 카메라가 신호등을 계속 본다. 2026-09-28 풀랩: 선 앞에서 선회해 차선을 밟고 선 채
            # 신호등이 ROI 밖으로 나가 초록을 못 읽었다. 곡선은 선 이후 남은 공간에 맞추고,
            # 그 곡선을 탈 수 있는 속도로 미리 줄인다 (초록에 서지 않고 지나는 경우).
            self.held_at_line = True
            room = max(self.transition_min_m, self.plan.nearest_s-self.front_m-self.entry_buffer_m-hold)
            return Target(state=self.name, v_max=self.swerve_speed(room, abs(self.plan.d-self.start_offset)))
        if not self.lateral_started:
            # 인식 진입과 선회 시작을 분리한다. 곡선 길이가 속도에 비례하므로
            # 저속에서는 더 가까이 접근한 뒤 옆으로 움직인다.
            # 시작한 뒤 감속해도 이 조건을 다시 적용하지 않는다 (회피 중 복귀 방지).
            length = self.transition_length(ctx.v)
            start_distance = self.front_m + self.entry_buffer_m + length
            gap = self.plan.nearest_s - ctx.s
            room = gap - self.front_m - self.entry_buffer_m          # where the swerve must be complete
            tight = math.isfinite(room) and self.held_at_line and room < length
            self.lateral_started = (abs(self.plan.d) > 1e-6 and ctx.v > 0.3 and math.isfinite(gap)
                                    and (tight or gap <= start_distance+self.entry_preview_m))
            if not self.lateral_started:
                return Target(state=self.name)
            self.entry_start_s = max(ctx.s, self.plan.nearest_s-start_distance)
            self.entry_length_m = length
            if tight:
                # Use all the room from here, and cap speed to the lateral limit on it.
                self.entry_start_s = ctx.s
                self.entry_length_m = length = max(self.transition_min_m, room)
                self.speed_cap = self.swerve_speed(length, abs(self.plan.d-self.start_offset))
            self.transition_end_s = self.entry_start_s + length
            self.return_length_m = self.planned_return_length(ctx, self.plan.d)
        d = self.plan.d
        # 선회를 시작한 뒤에는 오프셋을 **줄이지 않는다** (키우는 건 허용 — 더 안전한 쪽).
        # 플래너는 매 틱 0.25 m 격자에서 다시 고르므로 접근 중 -1.75↔-1.50 을 오가고,
        # 상자가 옆에 오면 앞쪽 충돌검사에 덜 걸려 -2.0→-1.5→-1.0→-0.5 로 차 뒤가 빠지기도
        # 전에 계단식으로 줄였다 (2026-09-20 50 km/h bag). 복귀는 미리 보낸 공간 곡선으로 연속적으로 수행한다.
        if self.prev_d != 0. and d * self.prev_d > 0. and abs(d) < abs(self.prev_d):
            d = self.prev_d
        self.prev_d = d
        # 후륜축의 clearance 위치를 미리 전송: PP가 갑작스런 0 요청 없이 복귀를 미리 본다.
        self.clearance_s = max(self.clear_s(b) for b in self.boxes())
        safe_start = max(self.clearance_s, self.transition_end_s)
        proposed = max(safe_start-self.return_lead_m, self.transition_end_s)
        straight = True
        if ctx.route is not None:
            local_start = proposed+ctx.s_offset-self.front_m
            local_end = safe_start+self.return_length_m+ctx.s_offset
            nearby = [k for s, k in zip(ctx.route.s, ctx.route.curvature) if local_start <= s <= local_end]
            straight = bool(nearby) and max(nearby) < .02
        if (straight and self.planner.return_is_clear(self.boxes(), d, proposed, self.return_length_m,
                                                     self.front_m, self.rear_overhang_m)):
            self.return_start_s = proposed
        else:
            self.return_start_s = safe_start
        cap = self.speed_cap if self.transition_end_s is not None and ctx.s < self.transition_end_s else math.inf
        return Target(state=self.name, lat_offset=d, v_max=min(cap, self.held_speed(ctx)),
                      return_s=self.return_start_s, return_length_m=self.return_length_m,
                          entry_s=self.entry_start_s, entry_length_m=self.entry_length_m)

    def debug(self) -> dict:
        return {'plan': self.plan, 'boxes': self.boxes(), 'transition_end_s': self.transition_end_s}
