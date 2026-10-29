import math
import pathlib
import sys
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from localization_core import CourseProgress, Estimator


class LocalizationTests(unittest.TestCase):
    def test_dr_and_gps_recovery_is_bounded(self):
        est = Estimator(correction_rate=1.0)
        self.assertTrue(est.fix(0, 0, 0))
        for t in range(1, 11):
            self.assertTrue(est.advance(t/10, 2, 0))
        self.assertAlmostEqual(est.x, 2)
        self.assertTrue(est.fix(4, 0, 1.0))
        self.assertTrue(est.advance(1.1, 2, 0))
        self.assertAlmostEqual(est.x, 2.3)
        self.assertFalse(est.fix(100, 0, 1.1))

    def test_missing_speed_updates_and_long_outage_fail(self):
        est = Estimator(max_outage=0.6)
        est.fix(0, 0, 0)
        self.assertFalse(est.advance(0.7, 2, 0))
        est.last_time = 0.4
        self.assertFalse(est.advance(0.7, 2, 0))

    def test_recover_requires_new_nearby_fix_and_blends(self):
        est = Estimator()
        est.fix(0, 0, 0)
        self.assertFalse(est.advance(1, 0, 0))
        self.assertFalse(est.recover(20, 0, 1))
        self.assertTrue(est.recover(1, 0, 1))
        self.assertEqual(est.x, 0)
        self.assertTrue(est.advance(1.1, 0, 0))
        self.assertAlmostEqual(est.x, 0.05)

    def test_closed_seam_does_not_start_at_finish(self):
        points = [(0, 0), (10, 0), (20, 0), (20, 10), (10, 10), (0, 10), (0, 0)]
        course = CourseProgress(points)
        self.assertTrue(course.update(0, 0))
        self.assertAlmostEqual(course.progress, 0)
        self.assertFalse(course.update(0, 10))
        self.assertEqual(course.progress, 0)

    def test_observed_progress_is_continuous(self):
        points = [(x, 0) for x in range(101)] + [(100, 10), (0, 10), (0, 0)]
        course = CourseProgress(points)
        for x in range(101):
            self.assertTrue(course.update(x, 0))
        self.assertAlmostEqual(course.progress, 100)
        for x in range(99, -1, -1):
            self.assertTrue(course.update(x, 10))
        for y in range(9, -1, -1):
            self.assertTrue(course.update(0, y))
        self.assertAlmostEqual(course.progress, course.length)

    def test_zero_time_does_not_advance(self):
        course = CourseProgress([(x, 0) for x in range(50)])
        for _ in range(30):
            self.assertTrue(course.update(0, 0))
        self.assertEqual(course.progress, 0)


if __name__ == '__main__':
    unittest.main()
