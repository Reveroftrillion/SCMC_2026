"""시뮬 없이 도는 판단 로직 테스트.   cd src/foscar_planning && python3 -m pytest test/ -q"""
import math
import unittest

import numpy as np

from foscar_control.core import Route
from foscar_planning.avoid_planner import AvoidPlanner, Box, box_from_event
from foscar_planning.frenet import base_to_map, to_frenet, to_xy
from foscar_planning.state import Behavior, BehaviorState, Context, EventInfo, Target
from foscar_planning.states.avoid import Avoid
from foscar_planning.states.cruise import Cruise
from foscar_planning.states.tunnel import Tunnel
from foscar_planning.states.yield_ import Yield


def ctx(s=300., v=8., now=0., events=(), loc_mode='GPS'):
    return Context(s=s, x=0., y=0., yaw=0., v=v, now=now, events=list(events), loc_mode=loc_mode)


def obstacle(s, d, size_x=3., size_y=2., now=0.):
    return EventInfo(type='OBSTACLE', stamp=now, s=s, d=d, size_x=size_x, size_y=size_y)


def start_s(a, box_s, v, size_x=3.):
    """이 속도에서 선회가 시작되는 자차 s — 파라미터에서 직접 계산한다.

    시작 조건은 시간 딜레이가 아니라 거리다 (avoid.compute_target):
        장애물 앞면 − 자차 s ≤ front_m + entry_buffer_m + 전이 곡선 길이(v)
    상수를 바꿔도 테스트가 따라오도록 숫자를 박아 두지 않는다.
    """
    front = box_s - max(size_x, 0.5) / 2. - a.pad_m
    return front - (a.front_m + a.entry_buffer_m + a.transition_length(v) + a.entry_preview_m)


class PlannerTest(unittest.TestCase):
    def test_box_in_lane_centre_needs_line_crossing(self):
        """s=329 상자 2×3 m 가 차선 한가운데: 어느 쪽이든 실선을 밟아야 하고, 정답은 존재한다."""
        p = AvoidPlanner()
        plan = p.plan([box_from_event(329., 0., 3., 2.)], ego_s=310.)
        self.assertFalse(plan.blocked)
        self.assertGreaterEqual(abs(plan.d), 1. + 0.2 + p.half)      # 상자 반폭 + pad + 차 반폭 + 마진
        self.assertEqual(plan.v_max, math.inf)
        self.assertEqual(len(plan.candidates), 25)
        self.assertFalse(next(c for c in plan.candidates if c.d == 0.).feasible)

    def test_return_sweep_checks_rotated_body_against_box(self):
        p = AvoidPlanner()
        box = box_from_event(329., .8, 3., 2.)
        self.assertTrue(p.return_is_clear([box], -1.75, 330.035, 16., 3.8, .835))
        self.assertFalse(p.return_is_clear([box], -1.75, 318., 16., 3.8, .835))
        mirrored = box_from_event(329., -.8, 3., 2.)
        self.assertTrue(p.return_is_clear([mirrored], 1.75, 330.035, 16., 3.8, .835))

    def test_centred_box_defaults_to_left(self):
        plan = AvoidPlanner().plan([box_from_event(329., 0., 3., 2.)], ego_s=310.)
        self.assertGreater(plan.d, 0.)

    def test_box_offset_left_goes_right(self):
        """상자가 왼쪽으로 치우쳐 있으면 오른쪽 틈이 더 싸다."""
        plan = AvoidPlanner().plan([box_from_event(329., 0.8, 3., 2.)], ego_s=310.)
        self.assertLess(plan.d, 0.)

    def test_hysteresis_keeps_side(self):
        """직전에 왼쪽으로 정했으면 비슷한 비용의 오른쪽으로 뒤집지 않는다."""
        p = AvoidPlanner()
        box = box_from_event(329., 0., 3., 2.)
        left = p.plan([box], 310., prev_d=0.).d
        again = p.plan([box], 312., prev_d=left).d
        self.assertEqual((left > 0), (again > 0))

    def test_blocked_when_road_full(self):
        plan = AvoidPlanner().plan([Box(320., 323., -3., 3.)], ego_s=300.)
        self.assertTrue(plan.blocked)
        self.assertEqual(plan.nearest_s, 320.)

    def test_obstacle_behind_is_ignored(self):
        plan = AvoidPlanner().plan([box_from_event(290., 0., 3., 2.)], ego_s=300.)
        self.assertEqual(plan.d, 0.)

    def test_no_obstacle_goes_straight(self):
        plan = AvoidPlanner().plan([], ego_s=300.)
        self.assertEqual(plan.d, 0.)
        self.assertFalse(plan.blocked)


class AvoidStateTest(unittest.TestCase):
    def test_return_is_published_in_advance_and_retained_until_complete(self):
        a = Avoid()
        b = Behavior([Cruise(), a])
        t = b.tick(ctx(s=316., v=8., events=[obstacle(329., .8)]))
        self.assertGreater(getattr(t, 'return_length_m', 0.), 0.)
        self.assertAlmostEqual(t.return_s, 330.035)
        release, length, side = t.return_s, t.return_length_m, t.lat_offset
        t = b.tick(ctx(s=release+.1, v=8., now=1.))
        self.assertEqual(t.lat_offset, side)
        self.assertEqual(t.return_s, release)
        self.assertEqual(t.return_length_m, length)
        t = b.tick(ctx(s=release+length+.1, v=8., now=2.))
        self.assertEqual(t.state, 'CRUISE')
        self.assertEqual(t.lat_offset, 0.)

    def test_avoid_does_not_override_cruise_speed(self):
        for ego_s in (298., 312., 317., 324.):
            a = Avoid()
            c = ctx(s=ego_s, v=30./3.6, events=[obstacle(329., 0.)])
            self.assertTrue(a.check_entry(c))
            self.assertGreaterEqual(a.compute_target(c).v_max, c.v)  # 2026-09-27: AVOID holds the current speed (no acceleration) instead of none

    def test_side_view_does_not_extend_hidden_length_again(self):
        a = Avoid()
        a.check_entry(ctx(s=310., now=0., events=[obstacle(327.5, .8, size_x=.5)]))
        a.check_exit(ctx(s=326., now=1., events=[obstacle(328.5, .8, size_x=1.5)]))
        self.assertEqual(len(a.memory), 1)
        # First padded front 327.05 + minimum physical length 3 = 330.05.
        # Side measurement rear 329.45 fits inside the remembered box.
        self.assertAlmostEqual(a.boxes()[0].s_max, 330.05)
        self.assertTrue(a.check_exit(ctx(s=331.5, now=2.)))

    def test_slow_approach_waits_before_starting_lateral_motion(self):
        # 같은 상자가 30 m 앞에 있어도 저속에서는 즉시 차로를 벗어나지 않는다.
        a = Avoid()
        v = 10. / 3.6
        begin = start_s(a, 329., v)
        self.assertGreater(begin + a.entry_preview_m, 310.)            # 저속이면 10 m 대까지 붙어서 시작한다
        c = ctx(s=298., v=v, events=[obstacle(329., 0.8)])
        self.assertTrue(a.check_entry(c))
        t = a.compute_target(c)
        self.assertEqual(t.lat_offset, 0.)
        self.assertEqual(t.v_max, math.inf)
        # 시작 지점 1 m 전: 아직
        c = ctx(s=begin - 1., v=v, now=3., events=[obstacle(329., 0.8)])
        self.assertFalse(a.check_exit(c))
        self.assertEqual(a.compute_target(c).lat_offset, 0.)
        # 시작 지점: 선회 시작
        c = ctx(s=begin + 0.1, v=v, now=6., events=[obstacle(329., 0.8)])
        self.assertFalse(a.check_exit(c))
        self.assertLess(a.compute_target(c).lat_offset, 0.)

    def test_fast_approach_starts_earlier_and_does_not_cancel_when_slowing(self):
        a = Avoid()
        fast, slow = 30. / 3.6, 10. / 3.6
        begin = start_s(a, 329., fast)
        self.assertLess(begin, start_s(a, 329., slow))                 # 빠를수록 일찍 시작한다
        c = ctx(s=begin - 1., v=fast, events=[obstacle(329., 0.8)])
        self.assertTrue(a.check_entry(c))
        self.assertEqual(a.compute_target(c).lat_offset, 0.)           # 시작 지점 1 m 전: 아직
        c = ctx(s=begin + 0.1, v=fast, now=1., events=[obstacle(329., 0.8)])
        self.assertFalse(a.check_exit(c))
        self.assertLess(a.compute_target(c).lat_offset, 0.)
        # 시작한 뒤 급감속해도 취소되지 않는다 (저속 기준이면 아직 시작할 자리가 아니다)
        c = ctx(s=begin + 1., v=0.1, now=2., events=[obstacle(329., 0.8)])
        self.assertFalse(a.check_exit(c))
        self.assertLess(a.compute_target(c).lat_offset, 0.)

    def test_entry_is_announced_before_the_curve_starts(self):
        a = Avoid()
        c = ctx(s=310., v=5.5, events=[obstacle(329., 0.)])
        self.assertTrue(a.check_entry(c))
        t = a.compute_target(c)
        self.assertNotEqual(t.lat_offset, 0.)
        self.assertGreater(t.entry_s, c.s)
        self.assertAlmostEqual(t.entry_s+t.entry_length_m, 327.3-a.front_m-a.entry_buffer_m)

    def test_slow_start_keeps_a_gentle_return_even_after_acceleration(self):
        a = Avoid(margin_m=.5)
        c = ctx(s=312., v=2., events=[obstacle(329., .6, size_x=.3, size_y=1.4)])
        self.assertTrue(a.check_entry(c))
        t = a.compute_target(c)
        self.assertGreaterEqual(t.return_length_m, 12.)
        release, length = t.return_s, t.return_length_m
        c = ctx(s=320., v=6., now=1., events=c.events)
        self.assertFalse(a.check_exit(c))
        t = a.compute_target(c)
        self.assertAlmostEqual(t.return_s, release)
        self.assertEqual(t.return_length_m, length)

    def test_waiting_at_rest_does_not_start_lateral_motion(self):
        a = Avoid()
        for now in (0., 1., 2.):
            c = ctx(s=314., v=0., now=now, events=[obstacle(329., 0.8)])
            if now == 0.:
                self.assertTrue(a.check_entry(c))
            else:
                self.assertFalse(a.check_exit(c))
            self.assertEqual(a.compute_target(c).lat_offset, 0.)
            self.assertIsNone(a.transition_end_s)

    def test_enter_avoid_exit(self):
        a = Avoid()
        # 45 m 앞: 아직 안 들어감
        self.assertFalse(a.check_entry(ctx(s=284., events=[obstacle(329., 0.)])))
        # 시작 지점: 들어감, 오프셋 나옴
        begin = start_s(a, 329., 8.) + 0.1
        self.assertTrue(a.check_entry(ctx(s=begin, now=1., events=[obstacle(329., 0.)])))
        t = a.compute_target(ctx(s=begin, now=1., events=[obstacle(329., 0.)]))
        self.assertEqual(t.state, 'AVOID')
        self.assertNotEqual(t.lat_offset, 0.)
        self.assertTrue(math.isnan(t.stop_s))
        side = t.lat_offset
        # 옆에 있는 동안 이벤트가 사라져도(라이다 ROI 는 전방뿐) 기억으로 유지
        for k, s in enumerate((320., 326., 329., 331.)):
            self.assertFalse(a.check_exit(ctx(s=s, now=2. + k)))
            t = a.compute_target(ctx(s=s, now=2. + k))
            self.assertEqual((t.lat_offset > 0), (side > 0))
        # 미리 보낸 복귀 곡선이 끝나면 해제
        self.assertFalse(a.check_exit(ctx(s=340., now=9.)))
        self.assertTrue(a.check_exit(ctx(s=349., now=10.)))
        self.assertEqual(a.prev_d, 0.)

    def test_a_static_box_whose_measured_speed_spikes_is_still_avoided(self):
        """정적 박스도 측면이 드러나며 중심점이 점프해 순간 속도가 6.9 m/s 까지 튄다.
        속도로 '움직였다' 를 판단했더니 박스를 못 피하고 들이받았다 (2026-09-20 실차)."""
        a = Avoid()
        jumpy = EventInfo(type='OBSTACLE', stamp=0., s=329., d=0., size_x=1.4, size_y=1.5, vx=6.9, vy=0.)
        self.assertTrue(a.check_entry(ctx(s=300., now=0., events=[jumpy])))
        self.assertEqual(len(a.boxes()), 1)

    def test_a_pedestrian_that_stopped_is_not_avoided_but_waited_for(self):
        """멈춰 선 보행자는 OBSTACLE 로 분류돼 들어온다. 그 자리에서 최근 움직임이 관측됐으면
        정적 장애물로 취급하지 않는다 — 옆으로 스쳐 지나가면 안 되고 YIELD 가 세워야 한다.
        (2026-09-20 실차: 보행자에게 off=+3.0 을 요청했다.)"""
        a = Avoid()
        walking = EventInfo(type='PEDESTRIAN', stamp=0., s=460., d=0.5, size_x=0.6, size_y=0.6, vx=0., vy=2.5)
        stopped = obstacle(460.5, 0.6, size_x=0.6, size_y=0.6)     # 같은 자리에서 멈춘 뒤
        self.assertFalse(a.check_entry(ctx(s=440., now=0., events=[walking])))   # 움직이는 것은 AVOID 대상이 아니다
        self.assertFalse(a.check_entry(ctx(s=445., now=1., events=[stopped])))   # 멈춰도 비켜가지 않는다
        self.assertEqual(a.boxes(), [])
        # dynamic_hold_s(5 초) 가 지나고도 그 자리에 있으면 그건 진짜 정적 장애물이다
        self.assertTrue(a.check_entry(ctx(s=445., now=7., events=[stopped])))
        self.assertEqual(len(a.boxes()), 1)

    def test_something_that_starts_moving_is_dropped_from_the_avoid_memory(self):
        """정적으로 기억해 둔 것이 움직이기 시작하면 회피 대상에서 뺀다 (사람 옆을 스치지 않는다)."""
        a = Avoid()
        box = obstacle(460., 0.)
        self.assertTrue(a.check_entry(ctx(s=440., now=0., events=[box])))
        self.assertEqual(len(a.boxes()), 1)
        moving = EventInfo(type='PEDESTRIAN', stamp=1., s=460., d=0., size_x=0.6, size_y=0.6, vx=0., vy=2.5)
        self.assertTrue(a.check_exit(ctx(s=445., now=1., events=[moving])))
        self.assertEqual(a.boxes(), [])

    def test_a_pedestrian_who_left_the_corridor_is_forgotten_even_when_we_stopped_for_it(self):
        """건너간 보행자는 잊는다. 멈춰 선 보행자는 OBSTACLE 로 분류돼 AVOID 가 기억하는데,
        예전에는 '아직 3 m 이상 앞' 일 때만 잊어서 바로 앞에 서 있으면 영영 못 잊었다 (교착)."""
        a = Avoid()
        seen = obstacle(320., 0., size_x=0.6, size_y=0.6)
        self.assertTrue(a.check_entry(ctx(s=300., now=0., events=[seen])))
        self.assertEqual(len(a.boxes()), 1)
        # 6 m 앞에 멈춰 섰다 — 정면이라 잘 보이는 자리다. 보행자가 건너가 이벤트가 끊기면
        self.assertFalse(a.check_exit(ctx(s=313., v=0., now=1., events=[seen])))
        for now in (2., 3., 4.):                       # 마지막으로 본 게 now=1, memory_s = 3 초
            self.assertFalse(a.check_exit(ctx(s=313., v=0., now=now, events=[])))
        self.assertTrue(a.check_exit(ctx(s=313., v=0., now=4.1, events=[])))
        self.assertEqual(a.boxes(), [])

    def test_an_obstacle_beside_the_car_is_kept_although_the_lidar_cannot_see_it(self):
        """반대로 옆구리(사각지대)에 있는 것은 안 보여도 기억한다 — 풀면 들이받는다."""
        a = Avoid()
        seen = obstacle(320., 0.)
        a.check_entry(ctx(s=300., now=0., events=[seen]))
        box = a.boxes()[0]
        beside = box.s_min - a.front_m + 0.5          # 앞 범퍼가 상자 앞면을 막 지난 위치
        for now in (1., 2., 3., 4., 5.):
            self.assertFalse(a.check_exit(ctx(s=beside, now=now, events=[])))
        self.assertEqual(len(a.boxes()), 1)
        self.assertTrue(a.check_exit(ctx(s=a.clear_s(box) + 0.1, now=6., events=[])))

    def test_offset_never_shrinks_until_release(self):
        """플래너가 매 틱 0.25 격자에서 d 를 다시 골라 -1.75↔-1.50 을 오가거나, 상자가 옆에 오면
        계단식으로 줄이는데, 선회 시작 후에는 줄이지 않고 clear_s 에서 한 번에 0 으로 간다."""
        from foscar_planning.avoid_planner import Plan
        a = Avoid()
        seen = obstacle(329., 0.)
        a.check_entry(ctx(s=316., now=0., events=[seen]))
        first = a.compute_target(ctx(s=316., now=0., events=[seen])).lat_offset
        self.assertNotEqual(first, 0.)
        real_plan = a.planner.plan
        # 플래너가 흔들리는 상황을 흉내: 매 틱 |d| 를 0.25 씩 줄여 돌려준다
        def shaky(boxes, ego_s, prev_d=0.):
            pl = real_plan(boxes, ego_s, prev_d)
            return Plan(d=pl.d - 0.25 * (1 if pl.d > 0 else -1) if abs(pl.d) > 0.3 else pl.d,
                        v_max=pl.v_max, blocked=pl.blocked, nearest_s=pl.nearest_s)
        a.planner.plan = shaky
        for k, s in enumerate((318., 321., 324., 327., 330.)):
            self.assertFalse(a.check_exit(ctx(s=s, now=1. + k, events=[seen] if s < 327 else [])))
            t = a.compute_target(ctx(s=s, now=1. + k, events=[seen] if s < 327 else []))
            self.assertAlmostEqual(abs(t.lat_offset), abs(first), msg=f's={s}: 줄어들면 안 됨')
            self.assertEqual(t.lat_offset > 0, first > 0)
        a.planner.plan = real_plan
        # 복귀 곡선 중에는 계획을 유지하고 완료 후 해제
        self.assertFalse(a.check_exit(ctx(s=340., now=8.)))
        self.assertTrue(a.check_exit(ctx(s=349., now=9.)))
        self.assertEqual(a.prev_d, 0.)

    def test_hidden_length_memory_and_return_survive_rear_clearance(self):
        """라이다는 앞면만 본다: 3 m 상자가 size_x 0.3 으로 들어와도 박스를 최소 hidden_length_m
        길이로 보고, 그 뒤를 뒤 범퍼가 tail_m 지난 뒤에만 오프셋을 푼다. 후륜축 기준으로 풀면
        뒤 모서리가 스친다. 측면이 보여 size_x 가 이미 크면 더 늘리지 않는다(이중 계산 방지)."""
        a = Avoid()
        seen = obstacle(329., 0., size_x=0.3)                  # 앞면만 보임
        a.check_entry(ctx(s=304., now=1., events=[seen]))
        a.compute_target(ctx(s=304., now=1., events=[seen]))
        box = a.boxes()[0]
        s_min = 329. - max(0.3, 0.5) / 2. - a.pad_m
        seen_s_max = 329. + max(0.3, 0.5) / 2. + a.pad_m         # 라이다가 본 앞면 기준 끝
        self.assertAlmostEqual(box.s_max, s_min + a.hidden_length_m)   # 최소 길이로 늘어남
        clear = box.s_max + a.tail_m + a.rear_overhang_m
        self.assertAlmostEqual(clear, a.clear_s(box))
        # 후륜축이 실측 s_max 를 2 m 넘어도 뒤 범퍼는 아직 숨은 길이 옆 → 기억도 오프셋도 유지
        self.assertFalse(a.check_exit(ctx(s=seen_s_max + 2., now=5.)))
        self.assertNotEqual(a.compute_target(ctx(s=seen_s_max + 2., now=5.)).lat_offset, 0.)
        # 차체가 빠지면 기억은 비우지만 예정된 복귀 곡선은 완료까지 유지한다.
        self.assertFalse(a.check_exit(ctx(s=clear - 0.1, now=6.)))
        self.assertFalse(a.check_exit(ctx(s=clear + 0.1, now=7.)))
        self.assertEqual(a.boxes(), [])
        self.assertTrue(a.check_exit(ctx(s=a.return_start_s+a.return_length_m+.1, now=8.)))
        # tail_m 을 키우면 그만큼 늦게 푼다
        b = Avoid(tail_m=a.tail_m + 2.)
        b.check_entry(ctx(s=304., now=1., events=[seen]))
        b.compute_target(ctx(s=304., now=1., events=[seen]))
        self.assertFalse(b.check_exit(ctx(s=clear + 0.1, now=7.)))
        self.assertFalse(b.check_exit(ctx(s=clear + 2.1, now=8.)))
        self.assertTrue(b.check_exit(ctx(s=b.return_start_s+b.return_length_m+.1, now=9.)))
        # 측면이 보여 size_x 가 최소 길이보다 크면 그대로 (더 늘리지 않음)
        c = Avoid()
        wide = obstacle(329., 0., size_x=4.)
        c.check_entry(ctx(s=304., now=1., events=[wide]))
        c.compute_target(ctx(s=304., now=1., events=[wide]))
        self.assertAlmostEqual(c.boxes()[0].s_max, 329. + 2. + c.pad_m)

    def test_curve_timing_does_not_change_speed_even_on_late_detection(self):
        """전이 곡선 L = max(min, v·T) 은 선회 시작 시 고정된다. 앞면 전에 끝나면 속도를 안 깎고,
        늦은 인식에서도 회피 전용 속도 상한은 만들지 않는다."""
        a = Avoid()
        seen = obstacle(329., 0.)
        v = 30. / 3.6
        begin = start_s(a, 329., v) + 0.1
        a.check_entry(ctx(s=begin, v=v, now=0., events=[seen]))
        t = a.compute_target(ctx(s=begin, v=v, now=0., events=[seen]))
        self.assertNotEqual(t.lat_offset, 0.)
        self.assertAlmostEqual(a.transition_end_s, t.entry_s + a.transition_length(v))
        self.assertLessEqual(a.transition_end_s, a.plan.nearest_s - a.front_m)      # 앞면 전에 끝남
        self.assertGreaterEqual(t.v_max, v)                                         # → 안 깎음 (현재 속도 유지)
        # 늦게 봐도 회피 전용 감속 상한을 부활시키지 않는다.
        b = Avoid()
        b.check_entry(ctx(s=317., v=v, now=0., events=[seen]))
        t2 = b.compute_target(ctx(s=317., v=v, now=0., events=[seen]))
        self.assertGreaterEqual(t2.v_max, v)   # late detection: no avoid-only slowdown (speed is held)
        # 앞면에 닿은 뒤(gap ≤ 0)엔 늦추는 게 더 위험 → 다시 플래너 속도
        self.assertFalse(b.check_exit(ctx(s=324., v=v, now=2., events=[seen])))
        self.assertGreaterEqual(b.compute_target(ctx(s=324., v=v, now=2., events=[seen])).v_max, v)  # speed held, not cut

    def test_false_positive_ahead_is_forgotten(self):
        a = Avoid()
        self.assertTrue(a.check_entry(ctx(s=304., now=0., events=[obstacle(329., 0.)])))
        self.assertFalse(a.check_exit(ctx(s=305., now=1.)))
        self.assertTrue(a.check_exit(ctx(s=306., now=4.)))     # memory_s(3 s) 넘게 안 보임 → 잊음

    def test_off_path_obstacle_ignored(self):
        a = Avoid()
        self.assertFalse(a.check_entry(ctx(s=304., events=[obstacle(329., 2.5)])))

    def test_wide_obstacle_reaching_into_the_car_width_counts(self):
        # Centre d=2.3 is off path, but a 3 m-wide barricade reaches d=0.8.
        self.assertTrue(Avoid().check_entry(ctx(s=304., events=[obstacle(329., 2.3, size_y=3.)])))

    def test_blocked_stops_before_obstacle(self):
        a = Avoid()
        wall = obstacle(329., 0., size_x=1., size_y=7.)
        self.assertTrue(a.check_entry(ctx(s=304., events=[wall])))
        t = a.compute_target(ctx(s=304., events=[wall]))
        self.assertTrue(math.isfinite(t.stop_s))
        self.assertLess(t.stop_s, 329.)


class FrenetTest(unittest.TestCase):
    def setUp(self):
        # 반지름 50 m 원호 (반시계) — 곡선에서 부호와 왕복 변환이 맞는지
        ang = np.linspace(0., math.pi, 400)
        self.route = Route(np.c_[50. * np.cos(ang), 50. * np.sin(ang)])

    def test_round_trip_and_sign(self):
        for s, d in ((10., 1.5), (60., -2.), (120., 0.)):
            x, y = to_xy(self.route, s, d)
            s2, d2 = to_frenet(self.route, x, y, near_s=s - 5.)
            self.assertAlmostEqual(s, s2, delta=0.05)
            self.assertAlmostEqual(d, d2, delta=0.01)
        # 반시계 원호의 왼쪽 = 원 안쪽 (반지름이 줄어든다)
        x, y = to_xy(self.route, 30., 1.)
        self.assertLess(math.hypot(x, y), 50.)

    def test_base_to_map(self):
        x, y = base_to_map(10., 2., 100., 200., math.pi / 2)     # 북쪽을 보는 차: 앞 10 m 좌 2 m
        self.assertAlmostEqual(x, 98.)
        self.assertAlmostEqual(y, 210.)


class SOffsetTest(unittest.TestCase):
    def test_loop_start_is_not_a_lap_off(self):
        from foscar_planning.frenet import s_offset_from
        ang = np.linspace(0., 2 * math.pi, 2000)
        loop = Route(np.c_[100. * np.cos(ang), 100. * np.sin(ang)])       # 폐루프, 길이 ≈ 628
        x, y = loop.at(loop.s[-1] - 0.01)                                   # 출발점 = 끝점 자리
        self.assertAlmostEqual(s_offset_from(loop, x, y, 0.), 0., delta=0.1)
        self.assertAlmostEqual(s_offset_from(loop, *loop.at(50.), 20.), 30., delta=0.1)


def pedestrian(s, d, vy, now=0.):
    # 직선 동쪽 경로(heading 0)에서는 map vy 가 곧 ḋ
    return EventInfo(type='PEDESTRIAN', stamp=now, s=s, d=d, size_x=0.6, size_y=0.6, vx=0., vy=vy)


class YieldStateTest(unittest.TestCase):
    def setUp(self):
        self.route = Route(np.c_[np.linspace(0., 600., 1201), np.zeros(1201)])   # 동쪽 직선, s_offset 0

    def c(self, **kw):
        kw.setdefault('v', 5.6)
        out = ctx(**kw)
        out.route = self.route
        return out

    def test_crossing_pedestrian_stops_then_releases(self):
        y = Yield()
        # 오른쪽 인도(d=-4)에서 왼쪽으로 1.4 m/s: 2 초 뒤 d=-1.2 → 경로 위 → 위협
        self.assertTrue(y.check_entry(self.c(s=445., now=0., events=[pedestrian(460., -4., 1.4)])))
        t = y.compute_target(self.c(s=445., now=0., events=[pedestrian(460., -4., 1.4)]))
        self.assertEqual(t.state, 'YIELD')
        self.assertAlmostEqual(t.stop_s, 454.)
        # 15 m 앞 stop_s, 5.6 m/s: gap 6 m < 필요 7.5 m → 급제동
        self.assertEqual(t.v_max, 0.)
        # 지나가서 왼쪽 d=+3.5 로 멀어지는 중 → 새 관측으로 clear_s 확인 후 출발
        self.assertTrue(y.check_exit(self.c(s=452., v=0., now=6., events=[pedestrian(460., 3.5, 1.4, now=6.)])))

    def test_pedestrian_walking_along_sidewalk_is_ignored(self):
        y = Yield()
        e = EventInfo(type='PEDESTRIAN', stamp=0., s=460., d=-4., size_x=0.6, size_y=0.6, vx=1.4, vy=0.)
        self.assertFalse(y.check_entry(self.c(s=445., events=[e])))

    def test_gone_releases_but_visible_npc_never_times_out(self):
        y = Yield()
        self.assertTrue(y.check_entry(self.c(s=445., now=0., events=[pedestrian(460., 0., 0.)])))
        self.assertFalse(y.check_exit(self.c(s=448., now=0.5)))
        self.assertTrue(y.check_exit(self.c(s=448., now=2.)))            # gone_s 1.5 s 넘게 안 보임
        # Visible NPC remains a threat even after a long wait.
        npc = EventInfo(type='VEHICLE', stamp=0., s=890., d=0., size_x=4., size_y=2.)
        self.assertTrue(y.check_entry(self.c(s=880., now=10., events=[npc])))
        for k in range(1, 9):
            self.assertFalse(y.check_exit(self.c(s=881., v=0., now=10. + k, events=[npc])))
        self.assertFalse(y.check_exit(self.c(s=881., v=0., now=19.1, events=[npc])))
        self.assertFalse(y.check_exit(self.c(s=881., v=0., now=60., events=[npc])))

    def test_reclassified_as_obstacle_keeps_waiting(self):
        y = Yield()
        self.assertTrue(y.check_entry(self.c(s=445., now=0., events=[pedestrian(460., 0., 0.)])))
        still = EventInfo(type='OBSTACLE', stamp=3., s=460., d=0.3, size_x=0.5, size_y=0.5)
        self.assertFalse(y.check_exit(self.c(s=450., v=0., now=3., events=[still])))
        self.assertFalse(y.check_exit(self.c(s=450., v=0., now=6., events=[still])))

    def test_loss_timer_uses_observation_time_not_cache_expiry(self):
        y = Yield()
        e = pedestrian(460., 0., 0., now=10.)
        self.assertTrue(y.check_entry(self.c(s=445., now=10., events=[e])))
        self.assertFalse(y.check_exit(self.c(s=445., now=10.45, events=[e])))
        self.assertFalse(y.check_exit(self.c(s=445., now=11.4)))
        self.assertTrue(y.check_exit(self.c(s=445., now=11.55)))

    def test_clear_confirmation_requires_new_observations(self):
        y = Yield(clear_s=0.3)
        self.assertTrue(y.check_entry(self.c(s=445., now=1., events=[pedestrian(460., 0., 1., now=1.)])))
        e = pedestrian(460., 3.5, 1., now=2.)
        e.size_x = e.size_y = 0.  # without body geometry retain observation dwell
        self.assertFalse(y.check_exit(self.c(s=445., now=2., events=[e])))
        self.assertFalse(y.check_exit(self.c(s=445., now=2.4, events=[e])))
        e.stamp = 2.4
        self.assertTrue(y.check_exit(self.c(s=445., now=2.4, events=[e])))

    def test_far_target_creeps_instead_of_slamming(self):
        y = Yield()
        self.assertTrue(y.check_entry(self.c(s=436., v=5.6, now=0., events=[pedestrian(460., 0., 0.)])))
        t = y.compute_target(self.c(s=436., v=5.6, now=0., events=[pedestrian(460., 0., 0.)]))
        self.assertEqual(t.v_max, 3.)
        self.assertAlmostEqual(t.stop_s, 454.)


class CoreTest(unittest.TestCase):
    def test_merge_stop_s_and_lat_offset(self):
        """낮은 우선순위 상태의 stop_s 와 높은 상태의 lat_offset 이 둘 다 살아남는다 (§2-4 주의)."""
        class Stop(BehaviorState):
            name, priority = 'STOP', 60

            def check_entry(self, c):
                return True

            def check_exit(self, c):
                return False

            def compute_target(self, c):
                return Target(state=self.name, stop_s=315.)

        b = Behavior([Cruise(), Stop(), Avoid()])
        t = b.tick(ctx(s=start_s(Avoid(), 329., 8.) + 0.1, events=[obstacle(329., 0.)]))
        self.assertEqual(t.state, 'AVOID')
        self.assertEqual(t.stop_s, 315.)
        self.assertNotEqual(t.lat_offset, 0.)
        self.assertLessEqual(t.v_max, 60. / 3.6)                # CRUISE 속도와 정지 요청을 보존한다
        self.assertGreater(t.v_max, 0.)                          # 회피 중에는 현재 속도를 유지할 뿐 세우지 않는다
        self.assertEqual(b.active_names(), ['AVOID', 'STOP', 'CRUISE'])

    def test_cruise_only_never_stops(self):
        """카메라 없는 통합: 신호등 s 근처에서도 stop_s 가 절대 나오지 않는다."""
        b = Behavior([Cruise(), Avoid(), Yield(), Tunnel()])
        for s in (200., 208., 315., 436., 550., 610.):
            t = b.tick(ctx(s=s))
            self.assertTrue(math.isnan(t.stop_s))
            self.assertEqual(t.state, 'CRUISE')
            self.assertAlmostEqual(t.v_max, (20. if 430. <= s <= 470. else 60.) / 3.6)

    def test_speed_zones_and_tunnel(self):
        b = Behavior([Cruise(), Tunnel()])
        self.assertAlmostEqual(b.tick(ctx(s=1500.)).v_max, 60. / 3.6)
        t = b.tick(ctx(s=1880.))
        self.assertEqual(t.state, 'TUNNEL')
        self.assertAlmostEqual(t.v_max, 60. / 3.6)  # standalone Cruise cap only
        self.assertEqual(b.tick(ctx(s=2050., loc_mode='DR')).state, 'TUNNEL')
        self.assertEqual(b.tick(ctx(s=2050.)).state, 'CRUISE')

    def test_unknown_param_rejected(self):
        with self.assertRaises(KeyError):
            Avoid(nope=1)


if __name__ == '__main__':
    unittest.main()


def test_tracking_only_road_users_do_not_trigger_driving_states():
    e=EventInfo(type='VEHICLE',stamp=1.,s=15.,d=0.,value='lidar:test:9')
    c=Context(s=0,x=0,y=0,yaw=0,v=0,now=1.,observations=[e])
    b=Behavior([Cruise(),Avoid(),Yield()])
    assert b.tick(c).state=='CRUISE'


def test_observation_only_departure_can_release_same_pedestrian():
    y=Yield(clear_s=.3)
    p=EventInfo(type='PEDESTRIAN',stamp=1.,s=15.,d=0.,vy=1.,value='lidar:test:9')
    route=Route(np.column_stack((np.arange(101.),np.zeros(101))))
    c=Context(s=0,x=0,y=0,yaw=0,v=0,now=1.,events=[p],route=route)
    assert y.check_entry(c)
    c.events=[]
    for stamp in (2.,2.4):
        c.now=stamp
        c.observations=[EventInfo(type='PEDESTRIAN',stamp=stamp,s=15.,d=4.,vy=1.,value=p.value)]
        released=y.check_exit(c)
    assert released


def test_geometry_cleared_observation_releases_without_lateral_speed():
    y=Yield(clear_s=.3)
    p=EventInfo(type='PEDESTRIAN',stamp=1.,s=15.,d=0.,value='lidar:test:9')
    c=Context(s=0,x=0,y=0,yaw=0,v=0,now=1.,events=[p])
    assert y.check_entry(c)
    c.events=[]
    for stamp in (2.,2.4):
        c.now=stamp
        c.observations=[EventInfo(type='PEDESTRIAN',stamp=stamp,s=15.,d=2.3,
                                 value=p.value,path_relevant=False)]
        released=y.check_exit(c)
    assert released


def test_tunnel_default_and_zero_disable_cap_without_requesting_stop():
    for state in (Tunnel(),Tunnel(v_max_kmh=0.)):
        target=state.compute_target(ctx(s=1900.))
        assert math.isinf(target.v_max) and math.isnan(target.stop_s)
    assert Tunnel(v_max_kmh=15.).compute_target(ctx(s=1900.)).v_max==15./3.6
