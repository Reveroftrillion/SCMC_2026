import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from live_status import summarize


class LiveStatusTest(unittest.TestCase):
    def test_fresh_stale_and_rate(self):
        self.assertEqual(summarize(0, None, None, 10)[0], 'MISSING')
        state, hz, age = summarize(11, 9, 10, 10.2)
        self.assertEqual(state, 'FRESH')
        self.assertAlmostEqual(hz, 10.)
        self.assertAlmostEqual(age, .2)
        self.assertEqual(summarize(11, 9, 10, 11.1)[0], 'STALE')


if __name__ == '__main__': unittest.main()
