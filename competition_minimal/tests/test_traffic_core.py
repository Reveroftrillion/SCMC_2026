import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from traffic_core import TrafficDecision


class TrafficTest(unittest.TestCase):
    def setUp(self):
        self.items = [dict(start_s=10.0, stop_s=30.0, direction='straight', roi=[0., 0., 1., 1.]),
                      dict(start_s=50.0, stop_s=70.0, direction='left', roi=[0., 0., 1., 1.])]
        self.c = TrafficDecision(self.items, True)

    def test_red_yellow_unknown_brake_then_stop(self):
        for label in ['4red', '4yellow', 'UNKNOWN']:
            self.c.observe(20., [label], 1.)
            stop, limit, _ = self.c.decision(20., 1.)
            self.assertFalse(stop)
            self.assertGreater(limit, 0)
            stop, limit, _ = self.c.decision(26., 1.)
            self.assertTrue(stop)
            self.assertEqual(limit, 0)

    def test_confirm_go_and_invalidate_on_contradiction_or_timeout(self):
        self.c.observe(26., ['4green'], 1.)
        self.assertTrue(self.c.decision(26., 1.)[0])
        self.c.observe(26., ['4green'], 1.1)
        self.assertFalse(self.c.decision(26., 1.1)[0])
        self.c.observe(26., ['4red'], 1.2)
        self.assertTrue(self.c.decision(26., 1.2)[0])
        self.c.observe(26., ['4green'], 1.3)
        self.c.observe(26., ['4green'], 1.4)
        self.assertTrue(self.c.decision(26., 2.3)[0])

    def test_left_requires_arrow(self):
        self.c.committed.add(0)
        self.c.observe(66., ['4green'], 1.)
        self.c.observe(66., ['4green'], 1.1)
        self.assertTrue(self.c.decision(66., 1.1)[0])
        self.c.observe(66., ['4greenleft'], 1.2)
        self.c.observe(66., ['4greenleft'], 1.3)
        self.assertFalse(self.c.decision(66., 1.3)[0])

    def test_ambiguous_lights_and_bad_config_stop(self):
        self.c.observe(26., ['4green', '4red'], 1.)
        self.c.observe(26., ['4green', '4red'], 1.1)
        self.assertTrue(self.c.decision(26., 1.1)[0])
        self.c.observe(26., ['4green', '4green'], 1.2)
        self.c.observe(26., ['4green', '4green'], 1.3)
        self.assertTrue(self.c.decision(26., 1.3)[0])
        self.assertTrue(TrafficDecision(self.items, False).decision(0., 1.)[0])
        self.assertTrue(TrafficDecision([], True).decision(0., 1.)[0])
        self.assertTrue(self.c.decision(math.nan, 1.)[0])

    def test_confirmed_crossing_does_not_stop_in_intersection(self):
        self.c.observe(26., ['4green'], 1.)
        self.c.observe(26., ['4green'], 1.1)
        self.assertFalse(self.c.decision(27., 1.1)[0])
        self.c.observe(28., ['4yellow'], 1.2)
        self.assertFalse(self.c.decision(28., 1.2)[0])
        self.assertEqual(self.c.decision(36., 1.2)[2], 'OUTSIDE_INTERSECTION')

    def test_overshot_unconfirmed_signal_remains_stopped(self):
        self.assertEqual(self.c.decision(36., 1.0)[2], 'UNCONFIRMED_CROSSING')

    def test_future_unverified_signal_does_not_hide_current_red(self):
        self.items[1]['verified'] = False
        self.c.observe(26., ['4red'], 1.)
        self.assertTrue(self.c.decision(26., 1.)[0])
        self.c.committed.add(0)
        self.assertEqual(self.c.decision(46., 1.)[2], 'UNVERIFIED_INTERSECTION')


if __name__ == '__main__':
    unittest.main()
