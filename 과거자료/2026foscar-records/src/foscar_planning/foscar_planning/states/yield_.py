"""YIELD — 움직이는 것(보행자·NPC) 앞에서 멈춰 기다린다 (팀 2, 개발계획서 §4-3).

진입  PEDESTRIAN / VEHICLE 이벤트가 앞 enter_distance_m 이내이고, 지금 경로 위(|d| ≤ cross_d_m)거나
      predict_s 초 안에 경로 위로 들어올 것 같을 때 (d + ḋ·t 가 ±cross_d_m 를 지나감)
목표  stop_s = 대상 s − stop_gap_m.
      tracker 의 stop_s 램프는 1.0 m/s² 라 20 km/h 에서 16 m 가 필요하다. 그 안에 못 서는 거리면
      v_max = 0 으로 **급제동** (tracker 가 속도 오차로 브레이크 0.5 ≈ 5 m/s²). 아니면 creep_mps 로 접근.
탈출  대상이 gone_s 동안 안 보이거나 경로에서 벗어나 멀어지고 있을 때 (clear_s 연속)
      관측된 위협이 있으면 대기시간만으로 출발하거나 재진입을 금지하지 않는다.

확인된 도로 이용자는 통로 밖으로 나가도 관측을 이어 받는다. 관측 소실은 통과 확인과 구분한다.

실측 (2026-09-19 s=460 보행자): 48 km/h 로 접근하니 11 m 앞에서 처음 보이고 0.6 s 뒤 충돌. 이 상태만으로는
못 막는다 — CRUISE 의 구간 속도(behavior.yaml, 430~470 m 20 km/h)와 같이 써야 한다.
"""

import math
from typing import Optional

from ..frenet import to_frenet
from ..state import BehaviorState, Context, EventInfo, Target, event_key
from ..traffic_position import rear_traffic


class Yield(BehaviorState):
    name = 'YIELD'
    priority = 80
    defaults = {
        'pedestrian_enabled': True,
        'pedestrian_zone_s_from': [-1.], # negative sentinel: unrestricted outside competition config
        'pedestrian_zone_s_to': [-1.],
        'enter_distance_m': 25.,
        'body_half_width_m': .946,
        'body_margin_m': .35,
        'cross_d_m': 2.5,            # 이 안이면 "경로 위" (차 반폭 0.95 + 여유)
        'predict_s': 2.,             # 이만큼 앞을 내다본다
        'stop_gap_m': 6.,            # 대상 s 앞 여유 (앞바퀴 기준은 tracker)
        'creep_mps': 3.,             # 서기 전 접근 속도 상한
        'brake_decel_mps2': 3.,      # v_max=0 으로 낼지 판단할 때 쓰는 제동감속 (보수적으로)
        'reaction_s': 0.4,
        'gone_s': 1.5,               # 대상이 안 보인 지 이만큼이면 지나간 것
        'clear_s': 0.35,             # 경로 밖으로 멀어지는 상태가 이만큼 이어지면 출발
        'deadlock_s': 8.,            # legacy config compatibility; never forces release
        'cooldown_s': 4.,            # legacy config compatibility; ignored
        'stopped_mps': 0.3,
    }

    def __init__(self, **params):
        super().__init__(**params)
        if (len(self.pedestrian_zone_s_from)!=len(self.pedestrian_zone_s_to) or
                any(not math.isfinite(a) or not math.isfinite(b) or b<a
                    for a,b in zip(self.pedestrian_zone_s_from,self.pedestrian_zone_s_to))):
            raise ValueError('invalid pedestrian mission zones')
        self.target_s = math.nan
        self.last_seen = None
        self.clear_since: Optional[float] = None
        self.reason = ''
        self.target_key = None
        self.target_type = None
        self.pedestrian_passage_confirmed = False

    # ------------------------------------------------------------------
    def lateral_rate(self, e: EventInfo, ctx: Context) -> float:
        """map 속도 → 경로 기준 횡속도 ḋ (좌 +). 경로가 없으면 0."""
        if ctx.route is None or e.s < 0:
            return 0.
        h = ctx.route.heading_at(e.s + ctx.s_offset)
        return -math.sin(h) * e.vx + math.cos(h) * e.vy

    def footprint_span(self, e, ctx):
        """(s centre, d centre, half length, half width) of the same-scan outline along the route, None without one.
        The event box is an AABB in base_link; re-boxed along the route it grew a standing deer (1.9 m, diagonal)
        1.2-1.5 m onto the road and YIELD never saw it clear of our width (user 2026-10-10)."""
        if ctx.route is None or len(e.footprint) < 3 or not math.isfinite(e.s):
            return None
        sd = [to_frenet(ctx.route, x, y, e.s + ctx.s_offset) for x, y in e.footprint]
        ss, dd = [p[0] - ctx.s_offset for p in sd], [p[1] for p in sd]
        return (max(ss)+min(ss))/2, (max(dd)+min(dd))/2, (max(ss)-min(ss))/2, (max(dd)-min(dd))/2

    def body_extents(self, e, ctx):
        """Measured box projected onto the route tangent (half length/width)."""
        span = self.footprint_span(e, ctx)
        if span is not None:
            return span[2], span[3]
        if ctx.route is None or not all(math.isfinite(v) and v > 0
                                        for v in (e.size_x,e.size_y)):
            return None
        if math.isfinite(e.box_yaw):
            angle = e.box_yaw-ctx.route.heading_at(e.s+ctx.s_offset)
            c,s = abs(math.cos(angle)),abs(math.sin(angle))
            return ((c*e.size_x+s*e.size_y)/2, (s*e.size_x+c*e.size_y)/2)
        radius = math.hypot(e.size_x,e.size_y)/2
        return radius,radius

    def corridor_d(self, e, ctx):
        span = self.footprint_span(e, ctx)
        centre_s, centre_d = (span[0], span[1]) if span is not None else (e.s, e.d)
        if ctx.avoidance_offset is None or ctx.mission_name != 'TUNNEL':
            return centre_d
        extent = self.body_extents(e, ctx)
        half_s = extent[0] if extent is not None else 0.
        lo,hi = max(ctx.s,centre_s-half_s),max(ctx.s,centre_s+half_s)
        n = max(1,math.ceil((hi-lo)/.25))
        offsets = [ctx.avoidance_offset(lo+(hi-lo)*i/n) for i in range(n+1)]
        return (centre_d-max(offsets) if centre_d>max(offsets) else
                centre_d-min(offsets) if centre_d<min(offsets) else 0.)

    def pedestrian_clear(self, e, ctx):
        d = self.corridor_d(e,ctx)
        extent = self.body_extents(e,ctx)
        return (e.type == 'PEDESTRIAN' and extent is not None
                and math.isfinite(e.d)
                and abs(d)-extent[1] > self.body_half_width_m+self.body_margin_m
                and d*self.lateral_rate(e,ctx) >= 0.)

    def pedestrian_active(self,ctx):
        if ctx.mission_pedestrian is not None:
            return self.pedestrian_enabled and ctx.mission_pedestrian
        zones=[(a,b) for a,b in zip(self.pedestrian_zone_s_from,self.pedestrian_zone_s_to) if a>=0.]
        return self.pedestrian_enabled and (not zones or any(
            a-self.enter_distance_m<=ctx.s<=b+self.enter_distance_m for a,b in zones))

    def threatens(self, e: EventInfo, ctx: Context) -> bool:
        d = self.corridor_d(e,ctx)
        if e.type == 'PEDESTRIAN' and not self.pedestrian_active(ctx):
            return False
        # Tunnel vehicles are parked and belong to static avoidance; people/animals use PEDESTRIAN.
        # No NPC vehicles appear in the pedestrian mission (only in the roundabout and highway). A person-sized
        # VEHICLE label there was already turned into PEDESTRIAN (Mission.apply_context); what is left is car-sized
        # (e.g. a static barricade) and must not stop the car through YIELD.
        if e.type == 'VEHICLE' and (ctx.mission_pedestrian or ctx.mission_name == 'TUNNEL'):
            return False
        local_static = ctx.vehicle_following_active and e.type == 'OBSTACLE'
        if (e.type not in ('PEDESTRIAN', 'VEHICLE', 'DYNAMIC_UNKNOWN') and not local_static) or e.s < 0 or not math.isfinite(e.d):
            return False
        # A fresh measured-corridor decision already controls this observation.
        # New/unseen observations and pedestrians retain this immediate guard.
        if (ctx.vehicle_following_active and e.type != 'PEDESTRIAN'
                and e.value in ctx.measured_clearance_ids
                and 0 <= ctx.now-ctx.measured_clearance_stamp <= .45
                and e.stamp <= ctx.measured_clearance_stamp):
            return False
        ahead = e.s - ctx.s
        if (ctx.vehicle_following_active and e.type != 'PEDESTRIAN'
                and rear_traffic(e, ctx, ctx.traffic_circle)):
            return False
        if (e.type in ('VEHICLE', 'DYNAMIC_UNKNOWN') and ctx.vehicle_prediction_active) or local_static:
            extent = self.body_extents(e,ctx)
            if extent is not None:
                # Keep the current braking corridor protected independently of
                # the future gap model; don't apply the old 25m / 2s heuristic.
                half_s,half_d = extent
                stopping = max(0.,ctx.v)*self.reaction_s + ctx.v**2/(2*self.brake_decel_mps2)
                if ctx.vehicle_following_active and not local_static and 0. <= ctx.now-e.stamp <= .45:
                    h = ctx.route.heading_at(e.s+ctx.s_offset)
                    forward = e.vx*math.cos(h)+e.vy*math.sin(h)
                    lateral = -e.vx*math.sin(h)+e.vy*math.cos(h)
                    if forward > .8 and abs(lateral) < .35*forward:
                        # A departing lead is not a stationary wall. Keep the
                        # body clearance and differential stopping distance;
                        # crossing/stale/stationary bodies retain the full guard.
                        stopping = (max(0.,ctx.v-forward)*self.reaction_s
                                    + max(0.,ctx.v**2-forward**2)/(2*self.brake_decel_mps2))
                return (ahead+half_s >= -.835-self.body_margin_m
                        and ahead-half_s <= 3.8+self.body_margin_m+stopping
                        and abs(d)-half_d <= self.body_half_width_m+self.body_margin_m)
        if self.pedestrian_clear(e,ctx):
            return False
        if not (-2. < ahead <= self.enter_distance_m):
            return False
        extent = self.body_extents(e,ctx) if e.type == 'PEDESTRIAN' else None
        if extent is not None and abs(d)-extent[1] <= self.body_half_width_m+self.body_margin_m:
            return True
        if abs(d) <= self.cross_d_m:
            return True
        d_future = d + self.lateral_rate(e, ctx) * self.predict_s
        # 지금 밖에 있어도 predict_s 안에 ±cross_d 구간을 지나가면 위협
        return (d > 0) != (d_future > 0) or abs(d_future) <= self.cross_d_m

    def threats(self, ctx: Context):
        return [e for e in ctx.events if self.threatens(e, ctx)]

    def moving_away(self, e: EventInfo, ctx: Context) -> bool:
        d = self.corridor_d(e,ctx)
        return abs(d) > self.cross_d_m and (d * self.lateral_rate(e, ctx)) > 0

    # ------------------------------------------------------------------
    def check_entry(self, ctx: Context) -> bool:
        th = self.threats(ctx)
        if not th:
            return False
        nearest = min(th, key=lambda e: e.s)
        self.pedestrian_passage_confirmed = False
        self.target_type = nearest.type
        self.target_s, self.last_seen = nearest.s, min(ctx.now, nearest.stamp)
        self.target_key = event_key(nearest) if nearest.value.startswith('lidar:') else None
        self.clear_since = None
        self.reason = f'{nearest.type} s={nearest.s:.0f} d={nearest.d:+.1f}'
        return True

    def check_exit(self, ctx: Context) -> bool:
        th = self.threats(ctx)
        # 걸린 뒤에는 종류를 안 가린다: 보행자가 느려지면 obstacle_node 가 OBSTACLE 로 바꿔 부르는데
        # (실측: 충돌 직후 PEDESTRIAN → OBSTACLE) 그 순간 "사라졌다" 고 출발하면 안 된다.
        th += [e for e in ctx.events if not ctx.vehicle_following_active and e.type == 'OBSTACLE' and math.isfinite(e.d)
               and abs(e.s - self.target_s) < 5. and abs(self.corridor_d(e,ctx)) <= self.cross_d_m]
        if th:
            nearest = min(th, key=lambda e: e.s)
            self.target_type = nearest.type
            self.target_s, self.last_seen = nearest.s, min(ctx.now, nearest.stamp)
            self.target_key = event_key(nearest) if nearest.value.startswith('lidar:') else None
            self.clear_since = None
        else:
            if ctx.vehicle_following_active and self.target_type == 'OBSTACLE':
                return self.release(ctx, 'local braking corridor clear')
            if self.target_type == 'PEDESTRIAN' and not self.pedestrian_active(ctx):
                return self.release(ctx, 'pedestrian yield disabled or outside mission zone')
            if ctx.vehicle_prediction_active and self.target_type in ('VEHICLE', 'DYNAMIC_UNKNOWN'):
                return self.release(ctx, 'vehicle prediction owns gap')
            # 위협은 아니지만 아직 보이는 같은 대상 (경로 밖으로 나가는 중) 인지
            seen = [e for e in ctx.events_of('PEDESTRIAN', 'VEHICLE', 'DYNAMIC_UNKNOWN') + ctx.observations
                    if e.type in ('PEDESTRIAN', 'VEHICLE', 'DYNAMIC_UNKNOWN')
                    and (event_key(e) == self.target_key if self.target_key is not None
                         else abs(e.s - self.target_s) < 5.)]
            if seen:
                # A measured body already outside our swept width and departing
                # needs no extra dwell. Missing/stale/unknown-size data retain
                # the old confirmation/loss path; another threat above wins.
                if (self.target_type == 'PEDESTRIAN'
                        and all(self.pedestrian_clear(e,ctx) and 0 <= ctx.now-e.stamp <= .25
                                and e.stamp > self.last_seen for e in seen)):
                    return self.release(ctx, 'moved away')
                observed_at = min(ctx.now, max(e.stamp for e in seen))
                self.last_seen = max(self.last_seen, observed_at)
                if all(not e.path_relevant or self.moving_away(e, ctx)
                       or self.pedestrian_clear(e,ctx) for e in seen):
                    if self.clear_since is None:
                        self.clear_since = observed_at
                    # Cached messages are not repeated evidence of departure.
                    if observed_at - self.clear_since >= self.clear_s:
                        return self.release(ctx, 'moved away')
                else:
                    self.clear_since = None
            else:
                self.clear_since = None
                if ctx.now - self.last_seen >= self.gone_s:
                    return self.release(ctx, 'gone')
        if ctx.s > self.target_s + 2.:
            return self.release(ctx, 'passed')
        return False

    def release(self, ctx: Context, why: str) -> bool:
        self.pedestrian_passage_confirmed = self.target_type == 'PEDESTRIAN' and why in ('moved away', 'passed')
        self.reason = f'released: {why}'
        self.target_s = math.nan
        return True

    def compute_target(self, ctx: Context) -> Target:
        stop_s = self.target_s - self.stop_gap_m
        gap = stop_s - ctx.s - 3.0                              # 앞바퀴 기준
        needed = ctx.v * self.reaction_s + ctx.v * ctx.v / (2. * self.brake_decel_mps2)
        if gap <= needed:
            return Target(state=self.name, v_max=0., stop_s=stop_s)      # 램프로는 못 선다 → 급제동
        return Target(state=self.name, v_max=self.creep_mps, stop_s=stop_s)

    def debug(self) -> dict:
        return {'target_s': self.target_s, 'reason': self.reason}
