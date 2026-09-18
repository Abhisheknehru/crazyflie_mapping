"""Round-trip and obstacle-check tests for the flight coordinate transforms."""
import math
import unittest

from test_runs.map_loader import LocalizationMap
from test_runs.flight_frame import path_is_clear, to_estimator_frame, to_map_frame


class FrameRoundTripTests(unittest.TestCase):
    def test_round_trip_without_alignment(self):
        origin = (1.0, -2.0, 0.3, math.radians(30))
        raw = (2.5, -1.0, math.radians(75))
        map_pose = to_map_frame(*raw, origin, None)
        result = to_estimator_frame(*map_pose, origin, None)
        for expected, actual in zip(raw, result):
            self.assertAlmostEqual(expected, actual, places=9)

    def test_round_trip_with_alignment(self):
        origin = (0.2, 0.4, 0.0, math.radians(-15))
        alignment = (0.5, -0.3, math.radians(10), 3.0, 4.0, math.radians(100))
        raw = (1.7, 0.9, math.radians(50))
        map_pose = to_map_frame(*raw, origin, alignment)
        result = to_estimator_frame(*map_pose, origin, alignment)
        for expected, actual in zip(raw, result):
            self.assertAlmostEqual(expected, actual, places=9)

    def test_no_alignment_matches_local_frame(self):
        origin = (0.0, 0.0, 0.0, 0.0)
        raw_x, raw_y, raw_yaw = 1.0, 2.0, 0.5
        map_pose = to_map_frame(raw_x, raw_y, raw_yaw, origin, None)
        self.assertEqual(map_pose, (raw_x, raw_y, raw_yaw))

    def test_estimator_frame_identity_when_origin_is_zero(self):
        origin = (0.0, 0.0, 0.0, 0.0)
        raw = to_estimator_frame(1.0, 2.0, 0.5, origin, None)
        self.assertEqual(raw, (1.0, 2.0, 0.5))


def empty_map(resolution=.1, size=4.0):
    width = round(size / resolution)
    occupied = [False] * (width * width)
    return LocalizationMap(resolution, -width * resolution / 2, width, occupied)


class PathIsClearTests(unittest.TestCase):
    def test_clear_path_through_empty_map(self):
        lmap = empty_map()
        self.assertTrue(path_is_clear(lmap, -1.0, 0.0, 1.0, 0.0))

    def test_wall_directly_between_points_blocks(self):
        lmap = empty_map()
        index = lmap.cell(0.0, 0.0)
        lmap.occupied[index] = True
        self.assertFalse(path_is_clear(lmap, -1.0, 0.0, 1.0, 0.0))

    def test_wall_off_to_the_side_does_not_block(self):
        lmap = empty_map()
        index = lmap.cell(0.0, 1.5)
        lmap.occupied[index] = True
        self.assertTrue(path_is_clear(lmap, -1.0, 0.0, 1.0, 0.0))

    def test_zero_length_path_checks_current_cell(self):
        lmap = empty_map()
        index = lmap.cell(0.2, 0.2)
        lmap.occupied[index] = True
        self.assertFalse(path_is_clear(lmap, 0.2, 0.2, 0.2, 0.2))
