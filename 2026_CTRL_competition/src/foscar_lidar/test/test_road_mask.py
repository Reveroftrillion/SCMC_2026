"""지도 기반 도로 마스크 테스트.

합성 링크로 기하를 확인하고, 실제 K-City link_set.json 이 있으면 그걸로도 한 번 확인한다.
"""

import math
import unittest
from pathlib import Path

import numpy as np

from foscar_lidar.road_mask import RoadMask

KCITY = Path(__file__).resolve().parents[2] / 'foscar_bringup/maps/link_set.json'


def straight(y, length=50., width=3.5, x0=0.):
    """y 고정, x 방향 직선 링크."""
    xs = np.arange(x0, x0 + length + 0.5, 0.5)
    return {'points': [[float(x), float(y), 0.] for x in xs], 'width_start': width, 'width_end': width}


class RoadMaskTest(unittest.TestCase):
    def test_inside_and_outside_a_single_lane(self):
        m = RoadMask.from_links([straight(0.)])
        # 중심 / 경계 바로 안 / 경계 바로 밖 / 한참 밖
        out = m.outside(np.array([[10., 0.], [10., 1.7], [10., 2.0], [10., 5.]]))
        self.assertAlmostEqual(out[0], 0.)
        self.assertAlmostEqual(out[1], 0.)                 # 반폭 1.75 안
        self.assertAlmostEqual(out[2], 0.25, places=2)     # 0.25 m 밖
        self.assertAlmostEqual(out[3], 3.25, places=2)
        self.assertTrue(m.on_road(np.array([[10., 1.7]]))[0])
        self.assertFalse(m.on_road(np.array([[10., 2.0]]))[0])
        self.assertTrue(m.on_road(np.array([[10., 2.0]]), margin_m=0.3)[0])

    def test_neighbouring_lanes_are_road_but_the_verge_is_not(self):
        """인접 차로는 도로다 (합집합). 그 바깥 갓길·가드레일 자리는 아니다."""
        m = RoadMask.from_links([straight(0.), straight(3.5), straight(-3.5)])
        self.assertTrue(m.on_road(np.array([[10., 3.5]]))[0])      # 옆 차로 중심
        self.assertTrue(m.on_road(np.array([[10., 5.2]]))[0])      # 옆 차로 경계 안
        self.assertFalse(m.on_road(np.array([[10., 5.4]]))[0])     # 도로 끝 밖 = 가드레일 자리
        self.assertGreater(m.outside_one(10., 7.), 1.5)

    def test_resampling_keeps_the_error_below_half_a_step(self):
        """점 간격 0.5 m 인 링크를 0.25 m 로 리샘플 → 점-폴리라인 거리 오차 ≤ 0.125 m."""
        m = RoadMask.from_links([straight(0.)], sample_m=0.25)
        # 링크 점 사이 한가운데(x=10.25)에서 옆으로 2 m: 참값 0.25 m 밖
        self.assertAlmostEqual(m.outside_one(10.25, 2.0), 0.25, delta=0.125)

    def test_off_the_map_is_far_outside_and_nan_is_rejected(self):
        m = RoadMask.from_links([straight(0.)])
        self.assertGreater(m.outside_one(10., 100.), 90.)
        self.assertEqual(m.outside_one(float('nan'), 0.), math.inf)

    def test_empty_query_and_no_links(self):
        m = RoadMask.from_links([straight(0.)])
        self.assertEqual(m.outside(np.zeros((0, 2))).size, 0)
        with self.assertRaises(ValueError):
            RoadMask.from_links([])
        self.assertIsNone(RoadMask.from_file('/does/not/exist.json'))

    @unittest.skipUnless(KCITY.is_file(), 'K-City link_set.json 이 없음')
    def test_real_kcity_map(self):
        import json
        m = RoadMask.from_file(KCITY)
        links = json.load(KCITY.open(encoding='utf-8'))
        centre = np.array(links[10]['points'])[5][:2]
        self.assertEqual(m.outside_one(*centre), 0.)               # 링크 중심은 도로
        # 링크에 수직으로 20 m 나가면 어느 링크에도 안 닿는다
        self.assertGreater(m.outside_one(centre[0] + 20., centre[1] + 20.), 5.)

    @unittest.skipUnless(KCITY.is_file(), 'K-City link_set.json 이 없음')
    def test_real_map_query_is_fast_enough_for_10hz(self):
        from time import perf_counter
        m = RoadMask.from_file(KCITY)
        pts = np.random.default_rng(0).normal(size=(4000, 2)) * 20.
        t0 = perf_counter()
        m.outside(pts)
        self.assertLess((perf_counter() - t0) * 1e3, 30.)           # 프레임 예산 100 ms 중 30 ms 미만


if __name__ == '__main__':
    unittest.main()
