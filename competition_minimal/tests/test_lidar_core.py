import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from lidar_core import (candidate_clear, choose_avoidance, ground_and_clusters,
                        profile, project, update_pedestrian_latch, follow_speed,
                        retain_occupancy, choose_highway_change, trajectory_observed)


def box(s, d, hs=.35, hd=.35):
    return dict(s=s, d=d, hs=hs, hd=hd)


class LidarCoreTest(unittest.TestCase):
    def test_quintic_transition_and_return(self):
        args = (0, 3.2, 10, 30, 10)
        self.assertEqual(profile(0, *args), 0)
        self.assertAlmostEqual(profile(10, *args), 3.2)
        self.assertAlmostEqual(profile(40, *args), 0)

    def test_clear_left_and_right_when_other_side_blocked(self):
        obstacle = box(25, 0)
        left = choose_avoidance([obstacle, box(25, -3.2)], 0)
        right = choose_avoidance([obstacle, box(25, 3.2)], 0)
        self.assertIsNotNone(left)
        self.assertIsNotNone(right)
        self.assertGreater(left[1], 0)
        self.assertLess(right[1], 0)

    def test_blocked_both_sides_or_late_transition(self):
        self.assertIsNone(choose_avoidance([box(25, 0), box(25, 3.2), box(25, -3.2)], 0))
        self.assertIsNone(choose_avoidance([box(6, 0)], 0))

    def test_return_path_is_also_checked(self):
        self.assertFalse(candidate_clear([box(38, 1.6)], 0, 3.2, 10, 30, 12))

    def test_only_remaining_path_checked(self):
        args = (0, 3.2, 10, 30, 12)
        self.assertFalse(candidate_clear([box(5, 1.6)], *args))
        self.assertTrue(candidate_clear([box(5, 1.6)], *args, from_s=20))
        self.assertFalse(candidate_clear([box(35, 1.6)], *args, from_s=20))

    def test_measured_road_width_required_for_configured_corridor(self):
        self.assertIsNone(choose_avoidance([box(25, 0)], 0, road_half_width=4.0))
        self.assertIsNotNone(choose_avoidance([box(25, 0)], 0, road_half_width=6.0))
        self.assertIsNone(choose_avoidance([box(25, 0)], 0, road_half_width=float('nan')))

    def test_bad_candidate_and_follow_inputs_are_safe(self):
        self.assertFalse(candidate_clear([box(float('nan'), 0)], 0, 3.2, 10, 30, 12))
        self.assertFalse(candidate_clear([], 0, 3.2, 10, 5, 12))
        self.assertEqual(follow_speed(8, 10), 0)
        self.assertEqual(follow_speed(12, 0), 2)
        self.assertEqual(follow_speed(float('nan'), 0), 0)

    def test_occupancy_survives_short_dropout_without_double_expansion(self):
        old = dict(id=3, wx=10, wy=0, vx=2, vy=0, time=5,
                   s=10, d=0, hs=.5, hd=.5, radius=.5)
        history, found = retain_occupancy([old], [], 5.4, (1, 0, 0))
        self.assertEqual(len(found), 1)
        self.assertTrue(found[0]['ghost'])
        self.assertAlmostEqual(found[0]['bx'], 9)
        self.assertAlmostEqual(found[0]['hs'], 1.3)
        history, found = retain_occupancy(history, [], 5.8, (1, 0, 0))
        self.assertAlmostEqual(found[0]['hs'], 2.1)
        _, found = retain_occupancy(history, [], 6.1, (1, 0, 0))
        self.assertEqual(found, [])

    def test_projection(self):
        path = np.array([[0., 0.], [10., 0.], [20., 0.]])
        self.assertEqual(project(path, np.array([0., 10., 20.]), [[6, 2]])[0], (6., 2.))

    def test_pedestrian_disappearance_never_clears(self):
        self.assertEqual(update_pedestrian_latch(9, [], 0), 9)
        p = dict(id=9, d=4, s=15, clear=0)
        self.assertEqual(update_pedestrian_latch(9, [p], 0), 9)
        self.assertEqual(update_pedestrian_latch(9, [p], 0), 9)
        self.assertIsNone(update_pedestrian_latch(9, [p], 0))

    def test_ground_support_and_cluster(self):
        ground = np.array([[x, y, -2.1] for x in range(0, 15) for y in range(-5, 6)])
        obj = np.array([[8 + .12*i, .12*j, -1.5 + .1*k]
                        for i in range(4) for j in range(4) for k in range(4)])
        result = ground_and_clusters(np.vstack([ground, obj]))
        self.assertIsNotNone(result)
        self.assertTrue(any(n >= 5 and lo[0] > 9 for lo, hi, n in result))
        self.assertIsNone(ground_and_clusters(np.zeros((0, 3))))

    def test_highway_change_obstacle_rear_traffic_and_cp12(self):
        plan = choose_highway_change([box(48, 0)], 30, 80, road_half_width=6.0)
        self.assertIsNotNone(plan)
        self.assertLessEqual(plan[3]+plan[4], 80)
        self.assertAlmostEqual(profile(plan[3]+plan[4], *plan), 0.0)
        self.assertIsNone(choose_highway_change([box(48, 0)], 30, 60, road_half_width=6.0))
        self.assertIsNone(choose_highway_change([box(48, 0)], 30, 80, road_half_width=4.0))
        # Both target lanes have a vehicle approaching from behind.
        self.assertIsNone(choose_highway_change([box(48, 0), box(20, 3.2), box(20, -3.2)],
                                                30, 80, road_half_width=6.0))
        self.assertFalse(candidate_clear([box(58, 3.2)], *plan, from_s=40))

    def test_tunnel_ground_coverage_walls_and_dropout(self):
        path = np.array([[float(x), 0.] for x in range(101)])
        cum = np.arange(101, dtype=float)
        plan = choose_avoidance([box(48, 0), box(48, -3.2)], 30, road_half_width=6.0)
        self.assertIsNotNone(plan)
        ground = np.array([[x-30, y] for x in np.arange(10., 80., .5)
                           for y in np.arange(-6., 6.1, .5)])
        self.assertTrue(trajectory_observed(ground, path, cum, (30., 0., 0.), 30, plan))
        self.assertFalse(trajectory_observed(np.empty((0, 2)), path, cum, (30., 0., 0.), 30, plan))
        # A wall occupies the proposed passing corridor even with ground returns.
        self.assertFalse(candidate_clear([box(48, 0), box(48, 3.2, hs=8, hd=.3)], *plan))
        missing = ground[~((ground[:, 0] > 5) & (ground[:, 0] < 22) & (ground[:, 1] > 1))]
        self.assertFalse(trajectory_observed(missing, path, cum, (30., 0., 0.), 30, plan))

    def test_highway_rear_ground_visibility_required(self):
        path = np.array([[float(x), 0.] for x in range(101)])
        cum = np.arange(101, dtype=float)
        plan = choose_highway_change([box(48, 0)], 30, 80, road_half_width=6.0)
        ground = np.array([[x-30, y] for x in np.arange(10., 80., .5)
                           for y in np.arange(-6., 6.1, .5)])
        self.assertTrue(trajectory_observed(ground, path, cum, (30., 0., 0.), 30, plan, rear_check=15))
        front_only = ground[ground[:, 0] >= -3]
        self.assertFalse(trajectory_observed(front_only, path, cum, (30., 0., 0.), 30, plan, rear_check=15))


if __name__ == '__main__':
    unittest.main()
