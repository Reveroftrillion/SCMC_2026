import sys
import unittest
from pathlib import Path
import numpy as np
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from local_planning_state import ObstacleMemory, input_fresh, return_progress, approach_zone_distance


class LocalStateTests(unittest.TestCase):
    def setUp(self):
        self.p = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/local_planner.yaml').read_text())['static_obstacle_avoidance_planner']
        self.points = np.column_stack((np.arange(100.), np.zeros(100)))

    def test_obstacle_disappearance_reappearance_and_expiry(self):
        memory = ObstacleMemory(2., .5)
        memory.observe([(12, 0, .5)], 10., 10.)
        self.assertEqual(len(memory.observe([], 11., 11.)), 1)
        self.assertEqual(len(memory.observe([(12.1, 0, .5)], 11.5, 11.5)), 1)
        self.assertEqual(len(memory.circles(13.)), 1)
        self.assertEqual(memory.circles(13.6), [])

    def test_reusing_old_detections_does_not_refresh_memory(self):
        memory = ObstacleMemory(2., .5)
        memory.observe([(12, 0, .5)], 10., 10.)
        self.assertEqual(memory.observe([(12, 0, .5)], 10., 12.1), [])

    def test_invalid_obstacle_cannot_partially_change_memory(self):
        memory = ObstacleMemory(2., .5)
        with self.assertRaises(ValueError):
            memory.observe([(12, 0, .5), (14, 0, float('nan'))], 10., 10.)
        self.assertEqual(memory.items, [])

    def test_stale_future_zero_stamp_and_receipt(self):
        self.assertTrue(input_fresh(10., 10.1, .5, 20., 20.1))
        for received, now, stamp, ros_now in ((None, 10, 20, 20), (10, 10.6, 20, 20.1),
                (10, 10.1, 0, 20), (10, 10.1, 20, 20.6), (10, 10.1, 21, 20), (11, 10, 20, 20)):
            self.assertFalse(input_fresh(received, now, .5, stamp, ros_now))

    def test_return_requires_continuous_alignment(self):
        complete, since = return_progress(0, 0, 10., None, self.p)
        self.assertFalse(complete)
        complete, since = return_progress(.5, 0, 10.5, since, self.p)
        self.assertIsNone(since)
        complete, since = return_progress(0, 0, 11., since, self.p)
        self.assertFalse(complete)
        self.assertTrue(return_progress(0, 0, 11.9, since, self.p)[0])

    def test_index_approach_uses_metres_not_point_count(self):
        zones = [{'start': 20, 'end': 30}]
        self.assertEqual(approach_zone_distance(zones, 10, 10, 0, self.points, 15), 10.)
        self.assertIsNone(approach_zone_distance(zones, 0, 0, 0, self.points, 15))
        self.assertIsNone(approach_zone_distance(zones, 35, 35, 0, self.points, 15))
        self.assertIsNone(approach_zone_distance(zones, 10, 10, 0, self.points, 0))

    def test_rectangle_approach_intersects_between_waypoints(self):
        zones = [{'xmin': 20.2, 'xmax': 20.8, 'ymin': -1, 'ymax': 1}]
        self.assertAlmostEqual(approach_zone_distance(zones, 10, 10, 0, self.points, 15), 10.2)
        away = self.points.copy()
        away[:, 1] = 10
        self.assertIsNone(approach_zone_distance(zones, 10, 10, 10, away, 15))

    def test_curved_approach_follows_route_not_euclidean_distance(self):
        points = np.array([[0, 0], [0, 10], [10, 10], [10, 0], [20, 0]], dtype=float)
        zones = [{'xmin': 9, 'xmax': 11, 'ymin': -1, 'ymax': 1}]
        self.assertIsNone(approach_zone_distance(zones, 0, 0, 0, points, 15))
        self.assertAlmostEqual(approach_zone_distance(zones, 0, 0, 0, points, 35), 29.)
