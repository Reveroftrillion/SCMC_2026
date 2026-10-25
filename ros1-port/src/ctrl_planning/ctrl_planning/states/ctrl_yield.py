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
import numpy as np
from typing import Optional

from ..ctrl_frenet import to_frenet
from ..ctrl_state import BehaviorState, Context, EventInfo, Target, event_key
from ..ctrl_traffic_position import rear_traffic
from ..ctrl_mapped_avoidance import deep_on_lanes


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
        'walk_mps': .5,              # 이보다 느리면 서 있는 것 (예측 안 함; obstacle_node PED_WALK_MPS, 측정)
        'stop_gap_m': 6.,            # 대상 s 앞 여유 (앞바퀴 기준은 tracker)
        'creep_mps': 3.,             # 서기 전 접근 속도 상한
        'brake_decel_mps2': 3.,      # v_max=0 으로 낼지 판단할 때 쓰는 제동감속 (보수적으로)
        'reaction_s': 0.4,
        'gone_s': 1.5,               # 대상이 안 보인 지 이만큼이면 지나간 것
        'clear_s': 0.35,             # 경로 밖으로 멀어지는 상태가 이만큼 이어지면 출발
        'deadlock_s': 8.,            # legacy config compatibility; never forces release
        'cooldown_s': 4.,            # legacy config compatibility; ignored
        'stopped_mps': 0.3,
        'go_accel_mps2': 3.3,        # tunnel: our launch, for when we would reach a pedestrian moving off (measured
                                     # 3.2 m in 1.4 s from standstill, 2026-10-11; the sooner, the longer we wait)
        'go_margin_s': .5,           # ... and they must be off the lanes this much before that
        'fresh_s': .25,              # an observation this recent is this scan's (2 lidar scans)
        'superseded_m': 1.5,         # a newer track's outline this close to a stale one's is that body (obstacle_node
                                     # tunnel_hold_join_m)
        'lane_sliver_m': .5,         # tunnel: a pedestrian reaching deeper onto the mapped lanes is on our road
                                     # (bev_avoidance LANE_SLIVER_M: a deer standing beside them read 0.2-0.45 m on)
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
        self.coming = {}   # event key -> (last scan stamp, scans in a row moving toward the path at walk_mps)
        self.left_key = None   # the pedestrian released as gone off our way: not taken again unless it comes back
        self.outlines = {}     # event key -> (stamp, map outline, vx, vy): the last same-scan outline of each person
        self.going = {}        # event key -> (last scan stamp, scans in a row going away from our path, its d)

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
                # standing (under walk_mps) is not coming: +0.09 m/s of outline noise toward us stopped the car at
                # 32 km/h for a deer standing beside the road (2026-10-10)
                and (d*self.lateral_rate(e,ctx) >= 0. or abs(self.lateral_rate(e,ctx)) < self.walk_mps)
                and not self.on_lanes(e, ctx))

    def on_lanes(self, e, ctx):
        """Tunnel: a pedestrian's same-scan outline more than lane_sliver_m onto the mapped lanes, our road. On it they
        are YIELD's whatever their speed (user 2026-10-10: stop as soon as one comes onto our road, go once off it): a
        deer that turned and ran in was 1.85 m onto the lanes, its new track still reading 0.8 m/s, and YIELD came only
        0.45 s later at 1.4 m."""
        if e.type != 'PEDESTRIAN' or getattr(ctx, 'mission_name', '') != 'TUNNEL' or ctx.road_map is None:
            return False
        points = self.outline(e, ctx)
        return points is not None and deep_on_lanes(ctx, points, self.lane_sliver_m)

    def outline(self, e, ctx):
        """The pedestrian's map outline: this scan's, else its last one carried by its speed, else its centre."""
        points = e.footprint
        if len(points) >= 3:
            self.outlines[e.value] = (e.stamp, points, e.vx, e.vy)
            if len(self.outlines) > 64:
                self.outlines = {k: v for k, v in self.outlines.items() if ctx.now - v[0] < 1.}
        last = self.outlines.get(e.value)
        if len(points) < 3 and last is not None and 0. <= e.stamp - last[0] <= .3:
            # No same-scan outline joined to this event (it came on its own thread): the last one, carried by its
            # speed -- its centre read off the lanes while its outline was 0.5 m on them, and YIELD let go for a tick
            # and took it again (2026-10-10).
            dt = e.stamp - last[0]
            points = [(x + last[2]*dt, y + last[3]*dt) for x, y in last[1]]
        if len(points) < 3:
            # none at all: its centre, not its box (the box re-boxed along the route grows a standing deer onto the
            # road). Without it a person running across our lane was released and taken again 3 times in 0.4 s.
            if ctx.route is None or not (math.isfinite(e.s) and math.isfinite(e.d)):
                return None
            s = e.s + ctx.s_offset
            x, y = ctx.route.at(s)
            h = ctx.route.heading_at(s)
            points = [(x - math.sin(h)*e.d, y + math.cos(h)*e.d)]
        return points

    def clears_before_us(self, e, ctx, crossing=False):
        """Tunnel: going away from our way at walking speed or more, outside our width, and off the lanes go_margin_s
        before we could get there from here (user 2026-10-11: they are long off our way, and set off gently now we
        would reach that spot only after they are well off the road -- we waited 0.7-1.0 s after they left our line,
        until they were off both lanes)."""
        if e.type != 'PEDESTRIAN' or getattr(ctx, 'mission_name', '') != 'TUNNEL' or ctx.road_map is None:
            return False
        d, rate = self.corridor_d(e, ctx), self.lateral_rate(e, ctx)
        last, row, last_d = self.going.get(e.value, (-math.inf, 0, None))
        if e.stamp > last:
            away = d != 0. and d*rate > 0. and abs(rate) >= self.walk_mps
            row = row+1 if away and (last_d is None or abs(d) > abs(last_d)+1e-3) else 0
            self.going[e.value] = (e.stamp, row, d)
            if len(self.going) > 256:
                self.going = {k: v for k, v in self.going.items() if ctx.now-v[0] < 2.}
        # crossing: still coming through our way (2 scans running), let go if it is through and off the other side by
        # then -- only for a YIELD already held, never to skip one (user 2026-10-11: we brake hard on the first move,
        # so we stand well short of them; go once they are surely through, not once they have left our line)
        through = crossing and d*rate < 0. and self.coming.get(e.value, (0., 0, None))[1] >= 2
        if d == 0. or abs(rate) < self.walk_mps or (d*rate <= 0. and not through):
            return False
        extent = self.body_extents(e, ctx)
        points = self.outline(e, ctx)
        if extent is None or points is None:
            return False
        gap = max(0., e.s - ctx.s - 3.8)                      # our front bumper to it
        v, a = max(ctx.v, 0.), self.go_accel_mps2
        arrive = (math.sqrt(v*v + 2.*a*gap) - v)/a
        # Out of our width by the time we could be there: a person running across 7 m ahead of us left our line
        # 0.3 s after crossing its middle and we waited for the first scan with its edge measured 1.3 m off it
        # (user 2026-10-11: still too late). Still inside our width now, it must have gone away 2 scans running.
        d_out = d + rate*max(0., arrive - self.go_margin_s)
        edge = abs(d_out) - extent[1]
        inside = abs(d)-extent[1] <= self.body_half_width_m+self.body_margin_m
        if (edge <= self.body_half_width_m+self.body_margin_m or (through and d_out*d >= 0.)
                or (inside and row < 2 and not through)):
            return False
        if crossing:
            # a YIELD held: out of our width by the time we are there is enough, not off both lanes too -- a person
            # running across both lanes from beyond them needs 2.1 s for that and we would be there in 1.6 s, so we
            # still went only as they left our line (user 2026-10-11: go once they are surely off my way)
            return True
        h = ctx.route.heading_at(e.s + ctx.s_offset)
        step = np.array([-math.sin(h), math.cos(h)])*rate*.1
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        for k in range(int((arrive - self.go_margin_s)/.1) + 1):
            if not deep_on_lanes(ctx, pts + step*k, self.lane_sliver_m):
                return True
        return False

    def pedestrian_active(self,ctx):
        if ctx.mission_pedestrian is not None:
            return self.pedestrian_enabled and ctx.mission_pedestrian
        zones=[(a,b) for a,b in zip(self.pedestrian_zone_s_from,self.pedestrian_zone_s_to) if a>=0.]
        return self.pedestrian_enabled and (not zones or any(
            a-self.enter_distance_m<=ctx.s<=b+self.enter_distance_m for a,b in zones))

    def threatens(self, e: EventInfo, ctx: Context, exiting: bool = False) -> bool:
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
        # Released as moved away, it is not taken again while it keeps going (user 2026-10-10: once off our road, go):
        # a deer released off the lanes was taken again 0.1 s later by the centre-within-2.5 m rule against our
        # bypass 2.4 m off, and by its outline still on the far lane edge, running away (2026-10-10, 3 of 10 runs).
        # Stopping or coming back, it is judged again.
        rate = self.lateral_rate(e, ctx)
        if (self.left_key is not None and e.value.startswith('lidar:') and event_key(e) == self.left_key
                and d != 0. and d*rate > 0. and abs(rate) >= self.walk_mps):
            return False
        if self.clears_before_us(e, ctx, crossing=exiting):
            return False
        if e.type == 'PEDESTRIAN' and -2. < ahead <= self.enter_distance_m and self.on_lanes(e, ctx):
            return True
        if self.pedestrian_clear(e,ctx):
            return False
        if not (-2. < ahead <= self.enter_distance_m):
            return False
        extent = self.body_extents(e,ctx) if e.type == 'PEDESTRIAN' else None
        if extent is not None and abs(d)-extent[1] <= self.body_half_width_m+self.body_margin_m:
            return True
        # Tunnel: a person near our line but not on our road nor in our width is a threat only once coming (the
        # rows below): the crossing mission's 2.5 m rule stopped us 19 m before a standing deer 2.4 m off our line,
        # on one scan of speed noise, at the tick before the bypass curve moved our line away (2 of 16, 2026-10-11).
        # (without a measured width, no route, the 2.5 m rule still stands)
        if abs(d) <= self.cross_d_m and not (extent is not None and getattr(ctx, 'mission_name', '') == 'TUNNEL'):
            return True
        # standing still (speed noise +-0.2 m/s) predicts nothing: over 4 s it put people at the road edge across it
        # Two scans in a row: a standing deer's outline wobbles, one scan read +1.3 m/s toward the road and YIELD
        # stopped us three times before it ran (2026-10-10); running it reads 1.8, 3.2, 3.6 m/s scan after scan.
        # ... and its outline's edge on our side came nearer too, where outlines are joined (box edges, not tracker
        # speed): a standing deer read +1.7, +1.4 m/s in two scans while that edge went -1.9, -1.6, -1.8 m, and YIELD
        # stopped us before it ran (2026-10-10); running, the edge comes 0.3-0.5 m a scan.
        last, row, edge = self.coming.get(e.value, (-math.inf, 0, None))
        if e.stamp > last:
            span = self.footprint_span(e, ctx)
            # the edge on our side (corridor_d's sign: d < 0 is below our path), moving toward it
            near = None if span is None else (span[1] + span[3] if d < 0 else span[1] - span[3])
            nearer = near is None or edge is None or (near - edge)*(1. if d < 0 else -1.) > 1e-3
            row = row+1 if abs(rate) >= self.walk_mps and d*rate < 0 and nearer else 0
            self.coming[e.value] = (e.stamp, row, near)
            if len(self.coming) > 256:
                self.coming = {k: v for k, v in self.coming.items() if ctx.now-v[0] < 2.}
        if abs(rate) < self.walk_mps or row < 2:
            return False
        d_future = d + rate * self.predict_s
        # 지금 밖에 있어도 predict_s 안에 ±cross_d 구간을 지나가면 위협
        return (d > 0) != (d_future > 0) or abs(d_future) <= self.cross_d_m

    def threats(self, ctx: Context, exiting: bool = False):
        # People beside the path are observations, not path events, until they are close: a person walking onto the
        # road from 6 m out was judged only once 2.5 m in, the car already at them (user 2026-10-10: stop when seen)
        # (only those walking toward the path: one standing off it was already judged outside it)
        people = [e for e in ctx.observations if e.type == 'PEDESTRIAN' and math.isfinite(e.d)
                  and ((abs(self.lateral_rate(e, ctx)) >= self.walk_mps
                        and self.corridor_d(e, ctx)*self.lateral_rate(e, ctx) < 0) or self.on_lanes(e, ctx))]
        found = [e for e in list(ctx.events) + people if self.threatens(e, ctx, exiting)]
        if exiting:
            # A person's track lost and seen again under a new one is that one now (user 2026-10-11: the old one is
            # gone from the world): its event, kept event_timeout_s, held a deer's YIELD 1.0-1.25 s past it leaving
            # our width with its last position frozen inside it, the new track already out (2 of 10 runs).
            fresh = [e for e in list(ctx.events) + list(ctx.observations) if e.type == 'PEDESTRIAN'
                     and 0. <= ctx.now-e.stamp <= self.fresh_s]
            found = [e for e in found if ctx.now-e.stamp <= self.fresh_s or not any(
                o.value != e.value and o.stamp > e.stamp and self.same_body(e, o, ctx) for o in fresh)]
        return found

    def same_body(self, a, b, ctx) -> bool:
        """Their outlines within superseded_m of each other."""
        pa, pb = self.outline(a, ctx), self.outline(b, ctx)
        if pa is None or pb is None:
            return False
        pa, pb = np.asarray(pa, dtype=float).reshape(-1, 2), np.asarray(pb, dtype=float).reshape(-1, 2)
        return float(np.min(np.linalg.norm(pa[:, None, :]-pb[None, :, :], axis=2))) <= self.superseded_m

    def moving_away(self, e: EventInfo, ctx: Context) -> bool:
        d = self.corridor_d(e,ctx)
        # off our road first (on_lanes): released while still on it, it was taken again the next tick (2026-10-10)
        return abs(d) > self.cross_d_m and (d * self.lateral_rate(e, ctx)) > 0 and not self.on_lanes(e, ctx)

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
        th = self.threats(ctx, exiting=True)
        # 걸린 뒤에는 종류를 안 가린다: 보행자가 느려지면 obstacle_node 가 OBSTACLE 로 바꿔 부르는데
        # (실측: 충돌 직후 PEDESTRIAN → OBSTACLE) 그 순간 "사라졌다" 고 출발하면 안 된다.
        th += [e for e in ctx.events if not ctx.vehicle_following_active and e.type == 'OBSTACLE' and math.isfinite(e.d)
               and abs(e.s - self.target_s) < 5. and abs(self.corridor_d(e,ctx)) <= self.cross_d_m]
        if th:
            nearest = min(th, key=lambda e: e.s)
            # A temporarily static label still protects the pedestrian we yielded to; it is not a new identity.
            self.target_type = (self.target_type if ctx.mission_name == 'TUNNEL' and nearest.type == 'OBSTACLE'
                                else nearest.type)
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
                        and all((self.pedestrian_clear(e,ctx) or self.clears_before_us(e,ctx,crossing=True)) and 0 <= ctx.now-e.stamp <= self.fresh_s
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
        self.left_key = self.target_key if self.target_type == 'PEDESTRIAN' and why == 'moved away' else None
        self.reason = f'released: {why}'
        self.target_s = math.nan
        return True

    def compute_target(self, ctx: Context) -> Target:
        # Tunnel: a pedestrian coming onto our road stops us where we are, at once (user 2026-10-11: not at a stop
        # line before them -- that was the crossing mission's); we go again once they are off our way.
        if self.target_type == 'PEDESTRIAN' and getattr(ctx, 'mission_name', '') == 'TUNNEL':
            return Target(state=self.name, v_max=0., stop_s=ctx.s + 3.0)    # front wheels here
        stop_s = self.target_s - self.stop_gap_m
        gap = stop_s - ctx.s - 3.0                              # 앞바퀴 기준
        needed = ctx.v * self.reaction_s + ctx.v * ctx.v / (2. * self.brake_decel_mps2)
        if gap <= needed:
            return Target(state=self.name, v_max=0., stop_s=stop_s)      # 램프로는 못 선다 → 급제동
        return Target(state=self.name, v_max=self.creep_mps, stop_s=stop_s)

    def debug(self) -> dict:
        return {'target_s': self.target_s, 'reason': self.reason}
