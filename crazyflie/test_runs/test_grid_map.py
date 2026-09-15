import unittest
from test_runs.grid_map import GridMap


class GridTests(unittest.TestCase):
    def test_wall_and_free_space(self):
        g = GridMap()
        g.ray(0, 0, 0, 1, True)
        data = g.occupancy()
        self.assertGreater(data[g.cell(1, 0)], 90)
        self.assertLess(data[g.cell(.5, 0)], 50)
        self.assertEqual(data[g.cell(-1, 0)], -1)

    def test_maximum_has_no_wall(self):
        g = GridMap()
        g.ray(0, 0, 0, 4, False)
        self.assertLess(g.occupancy()[g.cell(4, 0)], 50)

    def test_bounds_do_not_create_wall(self):
        g = GridMap(size=2)
        g.ray(0, 0, 0, 4, True)
        self.assertTrue(all(v <= 50 for v in g.occupancy()))

    def test_restart_is_empty(self):
        self.assertTrue(all(v == -1 for v in GridMap().occupancy()))
