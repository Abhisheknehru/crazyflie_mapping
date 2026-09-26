import math
import unittest

from test_runs.map_loader import LocalizationMap
from test_runs.scan_matcher import Alignment, CorrelativeScanMatcher


class ScanMatcherConstraintTests(unittest.TestCase):
    def make_map(self):
        width = 30
        occupied = [False] * (width * width)
        occupancy = [0] * (width * width)
        arena = LocalizationMap(.1, -1.5, width, occupied, occupancy)
        for y in range(width):
            index = y * width + 20
            occupied[index] = True
            occupancy[index] = 100
        return arena

    def test_search_ignores_lower_error_invalid_candidate(self):
        matcher = CorrelativeScanMatcher(self.make_map())
        points = [(1.0, y * .1) for y in range(-5, 6)]

        def only_left_half(alignment):
            return alignment.x <= .5

        result = matcher.match(
            points, Alignment(), xy_window=1.0, yaw_window=.01,
            xy_step=.1, yaw_step=.02, candidate_is_valid=only_left_half)
        self.assertLessEqual(result.alignment.x, .5)
        self.assertTrue(math.isfinite(result.mean_error))


if __name__ == '__main__':
    unittest.main()
