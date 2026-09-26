import math
import unittest

from test_runs.a_star import AStarPlanner, body_velocity
from test_runs.map_loader import LocalizationMap


class AStarPlannerTests(unittest.TestCase):
    def make_map(self, gap=True):
        width = 20
        occupied = [False] * (width * width)
        occupancy = [0] * (width * width)
        arena = LocalizationMap(.1, -1.0, width, occupied, occupancy)
        for y in range(width):
            if gap and 8 <= y <= 12:
                continue
            index = y * width + 10
            occupied[index] = True
            occupancy[index] = 100
        return arena

    def test_routes_through_gap(self):
        planner = AStarPlanner(self.make_map(), clearance=0.0)
        path = planner.plan((-.7, -.7), (.7, .7))
        self.assertEqual(path[0], (-.7, -.7))
        self.assertEqual(path[-1], (.7, .7))
        self.assertGreater(len(path), 2)
        for start, end in zip(path, path[1:]):
            self.assertTrue(start[0] == end[0] or start[1] == end[1])

    def test_reports_no_path_when_wall_is_closed(self):
        planner = AStarPlanner(self.make_map(gap=False), clearance=0.0)
        with self.assertRaisesRegex(ValueError, 'No collision-free path'):
            planner.plan((-.7, 0.0), (.7, 0.0))

    def test_rejects_goal_inside_clearance(self):
        planner = AStarPlanner(self.make_map(), clearance=.2)
        with self.assertRaisesRegex(ValueError, 'Goal'):
            planner.plan((-.7, 0.0), (-.05, .7))

    def test_body_velocity_respects_heading_and_speed_limit(self):
        forward, left, distance = body_velocity(
            (0.0, 0.0, math.pi / 2), (1.0, 0.0), max_speed=.1)
        self.assertAlmostEqual(forward, 0.0, places=6)
        self.assertAlmostEqual(left, -0.1, places=6)
        self.assertAlmostEqual(distance, 1.0)


if __name__ == '__main__':
    unittest.main()
