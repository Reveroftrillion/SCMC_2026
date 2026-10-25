"""obstacle_node 의 '내 길을 막고 있는가' 판정 테스트.

기준은 **종류와 무관하게 하나**다: 지도(link_set)상 도로 안 + 내 통로 안.
예전에는 기준이 분류에 따라 달라서(정지물 2 m / 이동물 5 m) 오분류가 곧 필터 구멍이었다.
실측 2026-09-20 (장애물 미스폰 시나리오 = 발행 189건 전부 오탐): 연석이 경로거리 1.98 m 로
임계 2.0 바로 아래를 통과했고, 가드레일은 도로 밖 1.2 m 인데 '이동물 유예' 로 통과했다.
"""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
import rclpy

from foscar_lidar.obstacle_node import ObstacleNode, Track


def link(y, width=3.5, length=200.):
    xs = np.arange(0., length + 0.5, 0.5)
    return {'points': [[float(x), float(y), 0.] for x in xs], 'width_start': width, 'width_end': width}


def track(node, mx, my, kind='OBSTACLE', vx=0., vy=0., size=0.5, size_y=None):
    """자차는 원점에서 +x 를 보고 있다고 두므로 map 좌표 = base_link 좌표."""
    tr = Track(0, node.history_len)
    tr.history.append((0., mx, my))
    tr.kind, tr.vx, tr.vy = kind, vx, vy
    tr.x, tr.y = mx, my
    tr.size_x, tr.size_y = size, size if size_y is None else size_y
    node.ego = (0., 0., 0.)
    return tr


KINDS = ('OBSTACLE', 'PEDESTRIAN', 'VEHICLE')


class RoadGateTest(unittest.TestCase):
    """왕복 2차로(중심 y=0 과 y=3.5). 내 차로 중심선이 경로, 도로 끝은 y=5.25."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.tmp = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False, encoding='utf-8')
        json.dump([link(0.), link(3.5)], cls.tmp)
        cls.tmp.close()

    @classmethod
    def tearDownClass(cls):
        Path(cls.tmp.name).unlink(missing_ok=True)
        rclpy.shutdown()

    def node(self, **overrides):
        params = {'use_tf': False, 'use_route': True, 'link_set': self.tmp.name,
                  'publish_markers': False}
        params.update(overrides)
        n = ObstacleNode(parameter_overrides=[rclpy.parameter.Parameter(k, value=v)
                                              for k, v in params.items()])
        self.addCleanup(n.destroy_node)
        n.route_xy = np.column_stack((np.arange(0., 200.5, 0.5), np.zeros(401)))
        n.route_s = np.arange(0., 200.5, 0.5)
        return n

    def test_wide_static_mission_body_survives_only_when_box_fits_road(self):
        from std_msgs.msg import String
        n = self.node()
        n.on_mission(String(data=json.dumps(dict(mission='STATIC_OBSTACLE'))))
        body = track(n, 50., 1.5, size=8.1, size_y=4.3)
        self.assertTrue(n.on_route_path(body, 100, 0.))
        body.y = 3.5
        body.history.clear(); body.history.append((0., 50., 3.5))
        self.assertFalse(n.on_route_path(body, 100, 0.))
        self.assertEqual(body.gate_why, 'wide')
        n.mission_received_at -= 2.
        body.y = 1.5
        body.history.clear(); body.history.append((0., 50., 1.5))
        self.assertFalse(n.on_route_path(body, 100, 0.))

    def test_road_mask_is_loaded(self):
        self.assertIsNotNone(self.node().road)

    def test_guardrail_is_dropped_whatever_it_is_classified_as(self):
        """도로 밖(경계에서 0.5 m) 이면 정지물이든 차량이든 보행자든, 움직여 보여도 버린다."""
        n = self.node()
        rail_y = 5.25 + 0.5
        for kind, vx, vy in (('OBSTACLE', 0., 0.), ('VEHICLE', 7.2, 0.), ('PEDESTRIAN', 2.5, -1.0)):
            with self.subTest(kind=kind):
                self.assertFalse(n.on_route_path(track(n, 50., rail_y, kind, vx, vy), 100, 0.))
        self.assertEqual(n.dropped_off_road, 3)
        self.assertEqual(n.dropped_off_route, 0)          # 도로 판정이 먼저 걸렸다

    def test_kerb_line_at_the_lane_edge_is_dropped(self):
        """실측 오탐의 정체: 경로에서 1.98 m 에 늘어선 연석/가드레일 밑동 (크기 0.48 m).

        통로 반폭 1.5 m + 모서리 기준이라 1.98 − 0.24 = 1.74 > 1.5 로 걸러진다.
        """
        n = self.node()
        for kind in KINDS:
            with self.subTest(kind=kind):
                self.assertFalse(n.on_route_path(track(n, 50., 1.98, kind, 0.5, 0., size=0.48), 100, 0.))

    def test_a_wall_slice_across_the_road_is_dropped_but_a_distant_box_face_is_not(self):
        """둘 다 차량 좌표로는 '얇고 옆으로 긴' 모양이다. 경로 가로폭으로만 갈린다.

        실측 2026-09-20: 회피 박스 d폭 중앙 1.47 m, 곡선 구간 레일 조각 d폭 중앙 3.48 m.
        """
        n = self.node()
        self.assertFalse(n.on_route_path(track(n, 50., 0., size=0.3, size_y=3.5), 100, 0.))   # 벽 조각
        self.assertEqual(n.dropped_wide, 1)
        tr = track(n, 50., 0., size=0.3, size_y=1.45)                                         # 27 m 밖 박스 앞면
        self.assertTrue(n.on_route_path(tr, 100, 0.))
        self.assertAlmostEqual(tr.s, 50., places=1)

    def test_tunnel_barricade_across_the_lane_is_kept_when_fully_on_road(self):
        """2026-09-26: 3 m-wide barricades in the tunnel were dropped as walls and hit."""
        from time import perf_counter
        n = self.node()
        barricade = lambda: track(n, 30., 2.2, size=0.3, size_y=3.0)     # d 0.7..3.7, road edge 5.25
        self.assertFalse(n.on_route_path(barricade(), 100, 0.))           # outside the tunnel: unchanged
        n.mission_name, n.mission_received_at = 'TUNNEL', perf_counter()
        tr = barricade()
        self.assertTrue(n.on_route_path(tr, 100, 0.))
        self.assertAlmostEqual(tr.s, 30., places=1)
        # Reaching into the tunnel shoulder (road edge -1.75, kept-point band 2.5 m) still counts:
        # d -2.5..0.5 blocks my lane.
        self.assertTrue(n.on_route_path(track(n, 30., -1.0, size=0.3, size_y=3.0), 100, 0.))
        # A slice spanning past that band (d -4.7..2.3) is not a road obstacle.
        self.assertFalse(n.on_route_path(track(n, 30., -1.2, size=0.3, size_y=7.0), 100, 0.))
        # Stale mission ownership falls back to the ordinary rule.
        n.mission_received_at = perf_counter()-5.
        self.assertFalse(n.on_route_path(barricade(), 100, 0.))

    def test_anything_in_my_lane_passes_regardless_of_kind(self):
        n = self.node()
        for kind in KINDS:
            with self.subTest(kind=kind):
                tr = track(n, 50., 0.5, kind)
                self.assertTrue(n.on_route_path(tr, 100, 0.))
                self.assertAlmostEqual(tr.lat, 0.5, places=2)
                self.assertAlmostEqual(tr.s, 50., places=1)

    def test_a_wide_object_half_in_the_lane_counts_by_its_nearest_edge(self):
        """차 한 대가 차선을 반쯤 물고 서 있으면 중심이 통로 밖이어도 막고 있는 것이다."""
        n = self.node()
        self.assertTrue(n.on_route_path(track(n, 50., 2.2, 'VEHICLE', size=1.9), 100, 0.))
        # 같은 자리의 작은 물체(연석 조각)는 아니다
        self.assertFalse(n.on_route_path(track(n, 50., 2.2, 'OBSTACLE', size=0.4), 100, 0.))

    def test_pedestrian_walking_into_my_lane_is_caught_early(self):
        """도로 안(옆 차로)에서 내 차선으로 들어오는 중이면 예측 위치로 미리 잡는다."""
        n = self.node()
        self.assertTrue(n.on_route_path(track(n, 50., 3.0, 'PEDESTRIAN', 0., -1.0), 100, 0.))
        # 도로 안이라도 멀어지는 중이면 아직 아니다
        self.assertFalse(n.on_route_path(track(n, 50., 3.0, 'PEDESTRIAN', 0., +1.0), 100, 0.))
        # 도로 **밖** 인도에서 같은 속도로 다가와도 아직 아니다 — 도로 안에 들어와야 장애물이다
        self.assertFalse(n.on_route_path(track(n, 50., 6.0, 'PEDESTRIAN', 0., -1.0), 100, 0.))

    def test_mask_off_falls_back_to_the_corridor_test_only(self):
        """지도가 없으면 통로 판정만 남는다 — 그래서 지도가 필요하다.

        레일을 따라 흐르는 겉보기 속도(vx)는 통로로 들어오지 않아 걸러지지만,
        도로 밖에서 내 쪽으로 향하는 겉보기 속도(vy)는 지도 없이는 막을 수 없다.
        """
        n = self.node(use_road_mask=False)
        self.assertIsNone(n.road)
        self.assertFalse(n.on_route_path(track(n, 50., 5.75, 'VEHICLE', 7.2, 0.), 100, 0.))
        self.assertTrue(n.on_route_path(track(n, 50., 5.75, 'VEHICLE', 0., -2.0), 100, 0.))
        # 지도를 켜면 그 경우도 막힌다
        self.assertFalse(self.node().on_route_path(track(n, 50., 5.75, 'VEHICLE', 0., -2.0), 100, 0.))
        self.assertTrue(n.on_route_path(track(n, 50., 0.5, 'VEHICLE'), 100, 0.))

    PERSON = dict(top=1.75, long=.6, short=.42, spread=.4, mid=.85, imed=18.)   # 2026-10-04 pedestrian mission p50

    def test_confirmed_pedestrian_survives_speed_spike_and_stop(self):
        n = self.node()
        tr = track(n, 15., 0., vx=0., vy=2.5)
        tr.size_z, tr.straightness = 1.6, 1.
        tr.ped = dict(self.PERSON)
        tr.kind = n.classify(tr)
        self.assertEqual(tr.kind, 'PEDESTRIAN')
        for speed in (3.35, 0., 2.5):
            tr.vy = speed
            tr.ped = None if speed == 0. else dict(self.PERSON, top=1.1)   # seen in part
            tr.kind = n.classify(tr)
            self.assertEqual(tr.kind, 'PEDESTRIAN')

    def test_person_sized_mover_is_a_person_only_by_shape(self):
        n = self.node()
        # a low barricade seen in part: person-sized and moving on its noisy centre, but 1 m tall
        tr = track(n, 15., 0., vx=0., vy=2.)
        tr.size_z, tr.straightness = 1.0, 1.
        tr.ped = dict(top=1.0, long=.9, short=.3, spread=.2, mid=.9, imed=55.)
        for _ in range(3):
            tr.kind = n.classify(tr)
            self.assertNotEqual(tr.kind, 'PEDESTRIAN')
        # a person-shaped cluster without intensity or walking is no person, however many scans (shape alone: 4 false
        # of 2691 in the 2026-10-04 measurement; poles and a parked car's corner in the 2026-10-10 tunnel runs)
        tr = track(n, 15., 0.)
        tr.ped = dict(self.PERSON, imed=None)
        for _ in range(3):
            self.assertNotEqual(n.classify(tr), 'PEDESTRIAN')
        # with the measured intensity it is one at once
        tr = track(n, 15., 0.)
        tr.ped = dict(self.PERSON)
        self.assertEqual(n.classify(tr), 'PEDESTRIAN')

    def test_confirmed_vehicle_survives_partial_view_and_stop(self):
        n = self.node()
        tr = track(n, 15., 0., vx=4., size=4.5, size_y=1.9)
        tr.size_z, tr.straightness = 1.5, 1.
        tr.kind = n.classify(tr)
        self.assertEqual(tr.kind, 'VEHICLE')
        tr.vx = 0.
        tr.size_x = tr.size_y = 0.5  # only a small part is visible now
        self.assertEqual(n.classify(tr), 'VEHICLE')

    def test_only_previously_published_road_user_gets_departure_observation(self):
        n = self.node()
        tr = track(n, 50., 3.5, 'PEDESTRIAN', vy=1.)
        self.assertFalse(n.departure_observation(tr, 100, 0.))
        tr.published = True
        self.assertTrue(n.departure_observation(tr, 100, 0.))
        self.assertAlmostEqual(tr.s, 50.)
        tr.kind = 'OBSTACLE'
        self.assertFalse(n.departure_observation(tr, 100, 0.))

    def test_recorded_braking_frame_keeps_person_but_removes_ground_strips(self):
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header
        n = self.node(use_road_mask=False)
        points = np.load(Path(__file__).parent / 'data/braking_ground_20260922.npz')['points']
        cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='lidar'), points)
        dets = n.detect(cloud, 0.)
        # Actual pedestrian at x7.42/y0.65 must remain detectable.
        self.assertTrue(any(abs(d['x'] - 7.42) < .6 and abs(d['y'] - .65) < .6 for d in dets))
        # These are the two thin ground strips that triggered AVOID in the bag.
        for x, y in ((15.81, -1.43), (13.17, -.99)):
            self.assertFalse(any(abs(d['x'] - x) < .6 and abs(d['y'] - y) < .6 for d in dets))

    def test_braking_ground_filter_preserves_a_real_box_front(self):
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header
        from foscar_lidar.ground import estimate_ground_plane
        n = self.node(use_road_mask=False)
        raw = np.load(Path(__file__).parent / 'data/braking_ground_20260922.npz')['points']
        base = raw + np.array([2., 0., 2.11])
        plane = estimate_ground_plane(base)
        self.assertIsNotNone(plane)
        yy, zz = np.meshgrid(np.linspace(-.75, .75, 20), np.linspace(.4, 1.8, 20))
        xx = np.full(yy.size, 20.)
        front = np.c_[xx, yy.ravel(), zz.ravel() + plane[0]*xx + plane[1]*yy.ravel() + plane[2]]
        raw = np.r_[raw, front - [2., 0., 2.11]]
        cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='lidar'), raw)
        boxes = [d for d in n.detect(cloud, 0.) if abs(d['x']-20.) < .3 and abs(d['y']) < .3]
        self.assertTrue(boxes)
        self.assertGreater(boxes[0]['size_z'], 1.)
        self.assertGreater(boxes[0]['size_y'], 1.)

    def test_vehicle_observation_covers_adjacent_road_but_not_static_roadside(self):
        n = self.node()
        car = track(n, 50., 3.5, 'VEHICLE', vx=4., size=2.)
        self.assertTrue(n.vehicle_observation(car, 100, 0.))
        self.assertAlmostEqual(car.s, 50.)
        car.kind = 'OBSTACLE'
        self.assertFalse(n.vehicle_observation(car, 100, 0.))
        car.kind = 'VEHICLE'
        car.history.clear(); car.history.append((0., 50., 8.))
        self.assertFalse(n.vehicle_observation(car, 100, 0.))

    def test_point_cull_needs_odom_and_keeps_points_on_the_road(self):
        n = self.node()
        pts = np.array([[10., 0., 0.], [10., 3.5, 0.], [10., 9., 0.]], dtype=np.float32)
        self.assertEqual(n.cull_off_road(pts, 0.).shape[0], 3)     # odom 없으면 그대로
        n.odom_buf.append((0., 0., 0., 0.))
        self.assertEqual(n.cull_off_road(pts, 0.).shape[0], 2)     # 도로 밖 3.6 m 인 점만 빠진다
        self.assertEqual(n.culled_points, 1)

    def test_wide_vehicle_is_observation_only_and_cannot_bypass_path_gate(self):
        n = self.node()
        tr = track(n, 20., 3.5, 'VEHICLE', vx=4., size=1.)
        tr.published = True
        decision, observation = n.output_gates(tr, 0, 0.)
        self.assertFalse(decision)
        self.assertTrue(observation)
        # A previously emitted rail now outside the road remains observation-only.
        tr.history.clear(); tr.history.append((0.,20.,6.))
        tr.y = 6.
        decision, observation = n.output_gates(tr, 0, 0.)
        self.assertFalse(decision)
        self.assertTrue(observation)

    def test_predicted_lateral_motion_is_rotated_from_map_to_vehicle_frame(self):
        n = self.node()
        # Ego heading north. Pedestrian is 3m west and moves east toward route.
        n.route_xy = np.column_stack((np.zeros(401),np.arange(0.,200.5,.5)))
        n.road = None
        tr = track(n, -3.,20., 'PEDESTRIAN', vx=1.,size=.5)
        tr.x,tr.y = 20.,3.
        n.ego = (0.,0.,np.pi/2)
        self.assertTrue(n.on_route_path(tr,0,0.))
        tr.vx = -1.
        self.assertFalse(n.on_route_path(tr,0,0.))

    def test_structure_evidence_overrides_old_vehicle_label(self):
        n = self.node()
        tr = track(n,20.,0.,'VEHICLE',vx=4.,size=2.)
        tr.structure=True
        self.assertEqual(n.classify(tr),'OBSTACLE')


if __name__ == '__main__':
    unittest.main()
