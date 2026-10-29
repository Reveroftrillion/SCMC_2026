import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts')))
from geometry_audit import audit


class GeometryAuditTest(unittest.TestCase):
    def test_signed_projection_and_direction(self):
        route = np.column_stack((np.arange(101), np.zeros(101)))
        result = audit(route, (50., 2.))
        self.assertAlmostEqual(result['s'], 50.)
        self.assertAlmostEqual(result['signed_lateral'], 2.)
        self.assertAlmostEqual(result['heading'], 0.)
        self.assertGreaterEqual(result['alternative'][1], 30.)

    def test_detects_another_lane(self):
        outbound = np.column_stack((np.arange(51), np.zeros(51)))
        inbound = np.column_stack((np.arange(50, -1, -1), np.full(51, 3.)))
        result = audit(np.vstack((outbound, inbound)), (20., 0.2))
        self.assertAlmostEqual(result['distance'], 0.2)
        self.assertAlmostEqual(result['alternative'][1], 2.8)

    def test_rejects_degenerate_path(self):
        with self.assertRaises(ValueError):
            audit([[0, 0], [0, 0]], [1, 1])


if __name__ == '__main__':
    unittest.main()
