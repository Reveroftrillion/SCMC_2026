#!/usr/bin/env python3
import copy
import sys
import unittest
from pathlib import Path
import numpy as np
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from local_planning_core import Candidate, FrenetLocalPlanner, in_zones, validate_zones

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'local_planner.yaml'


class LocalPlanningTests(unittest.TestCase):
    def setUp(self):
        self.p = yaml.safe_load(CONFIG.read_text())['static_obstacle_avoidance_planner']
        self.core = FrenetLocalPlanner(self.p)
        self.points = np.column_stack((np.arange(0, 100, .5), np.zeros(200)))
        self.ref = self.core.reference(self.points, 0)

    def test_empty_zones_never_activate(self):
        self.assertFalse(in_zones([], 0, 0, 0))
        self.assertEqual(self.p['static_zones'], [])
        self.assertEqual(self.p['dynamic_zones'], [])

    def test_zone_boundaries_and_map_coordinates(self):
        z = [{'start': 10, 'end': 20}, {'xmin': 100, 'xmax': 110, 'ymin': 0, 'ymax': 5}]
        validate_zones(z)
        self.assertTrue(in_zones(z, 10, 0, 0))
        self.assertTrue(in_zones(z, 20, 0, 0))
        self.assertFalse(in_zones(z, 21, 0, 0))
        self.assertTrue(in_zones(z, None, 105, 3))
        with self.assertRaises(ValueError):
            validate_zones([{'start': 20, 'end': 10}])

    def test_clear_straight_road_prefers_center(self):
        best, candidates, _ = self.core.plan(self.ref, (0, 0, 0), [])
        self.assertIsNotNone(best)
        self.assertEqual(best.offset, 0)
        np.testing.assert_allclose(best.xy[:, 1], 0, atol=1e-7)
        self.assertTrue(np.isfinite([c.cost for c in candidates]).all())

    def test_center_obstacle_rejects_center_selects_side(self):
        best, candidates, _ = self.core.plan(self.ref, (0, 0, 0), [(12, 0, .5)])
        self.assertIsNotNone(best)
        self.assertNotEqual(best.offset, 0)
        self.assertGreater(best.clearance, 0)
        center = next(c for c in candidates if c.offset == 0)
        self.assertEqual(center.rejection, 'collision')
        self.assertAlmostEqual(best.xy[-1, 1], 0, places=6)

    def test_blocked_corridor_has_no_path(self):
        best, _, _ = self.core.plan(self.ref, (0, 0, 0), [(12, 0, 6)])
        self.assertIsNone(best)

    def test_side_preference_does_not_flap(self):
        first, _, _ = self.core.plan(self.ref, (0, 0, 0), [(12, .01, .5)])
        self.assertIsNotNone(first)
        self.core.previous = first
        second, _, _ = self.core.plan(self.ref, (.1, 0, 0), [(12, -.01, .5)])
        self.assertEqual(first.offset, second.offset)

    def test_new_obstacle_invalidates_previous_side(self):
        first, _, _ = self.core.plan(self.ref, (0, 0, 0), [(12, 0, .5)])
        self.core.previous = first
        best, _, _ = self.core.plan(self.ref, (0, 0, 0), [(12, 0, .5), (12, first.offset, .5)])
        self.assertIsNotNone(best)
        self.assertLess(best.offset * first.offset, 0)

    def test_vehicle_width_in_collision_check(self):
        c = Candidate(0, np.array([[0., 0.], [10., 0.]]), np.zeros(2), np.zeros(2))
        self.assertLess(self.core.clearance(c, [(5, 1, .1)]), 0)
        self.assertGreater(self.core.clearance(c, [(5, 4, .1)]), 0)

    def test_collision_between_path_samples(self):
        c = Candidate(0, np.array([[0., 0.], [20., 0.]]), np.zeros(2), np.zeros(2))
        self.assertLess(self.core.clearance(c, [(10, 0, .1)]), 0)

    def test_smooth_return_starts_near_current_pose(self):
        best, _, (_, d, _) = self.core.plan(self.ref, (4.1, 2.0, 0), [], returning=True)
        self.assertIsNotNone(best)
        np.testing.assert_allclose(best.xy[0], [4.1, 2], atol=.02)
        self.assertAlmostEqual(best.xy[-1, 1], 0, places=5)
        self.assertLess(abs(best.yaw[0]), .02)

    def test_curve_has_finite_curvature(self):
        a = np.linspace(0, 1.8, 200)
        points = np.column_stack((50 * np.sin(a), 50 * (1 - np.cos(a))))
        ref = self.core.reference(points, 0)
        best, _, _ = self.core.plan(ref, (0, 0, 0), [])
        self.assertIsNotNone(best)
        self.assertTrue(np.isfinite(best.curvature).all())
        self.assertAlmostEqual(float(np.median(best.curvature)), .02, delta=.003)

    def test_duplicate_waypoints_are_filtered(self):
        ref = self.core.reference(np.repeat(self.points, 2, axis=0), 0)
        best, _, _ = self.core.plan(ref, (0, 0, 0), [])
        self.assertIsNotNone(best)

    def test_end_of_reference_holds_instead_of_extrapolating(self):
        with self.assertRaises(ValueError):
            self.core.plan(self.ref, (75, 0, 0), [])

    def test_road_boundary_blocks_excessive_offset(self):
        p = copy.deepcopy(self.p)
        p['road_half_width'] = 2.5
        best, candidates, _ = FrenetLocalPlanner(p).plan(self.ref, (0, 0, 0), [(12, 0, .5)])
        self.assertIsNone(best)
        self.assertTrue(any(c.rejection == 'road_boundary' for c in candidates))

    def test_projection_uses_correct_side_and_between_samples(self):
        s, d, yaw = self.core.project(self.ref, 10.13, -2)
        self.assertAlmostEqual(s, 10.13, places=5)
        self.assertAlmostEqual(d, -2, places=5)


if __name__ == '__main__':
    unittest.main()
