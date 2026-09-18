"""Round-trip test: save a GridMap, reload it, check raycast agrees."""
import math
import os
import shutil
import tempfile
import unittest
from pathlib import Path

from test_runs.grid_map import GridMap
from test_runs.map_export import save_grid
from test_runs.map_loader import load_map, pick_latest_map


class MapLoaderTests(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmpdir, ignore_errors=True)

    def build_and_save(self):
        grid = GridMap(resolution=.1, size=4.0)
        grid.ray(0, 0, 0, 1.0, True)
        grid.ray(0, 0, 0, 3.9, False)
        prefix = str(Path(self.tmpdir) / 'maze')
        pgm, yaml = save_grid(grid, prefix)
        return grid, pgm, yaml

    def test_raycast_matches_saved_wall(self):
        grid, _, yaml = self.build_and_save()
        loaded = load_map(str(yaml))
        self.assertEqual(loaded.resolution, grid.resolution)
        self.assertEqual(loaded.width, grid.width)
        distance = loaded.raycast(0, 0, 0, 4.0)
        self.assertAlmostEqual(distance, 1.0, delta=.15)

    def test_open_direction_reports_grid_boundary(self):
        grid, _, yaml = self.build_and_save()
        loaded = load_map(str(yaml))
        distance = loaded.raycast(0, 0, math.pi, 4.0)
        boundary = grid.width * grid.resolution / 2
        self.assertAlmostEqual(distance, boundary, delta=.15)

    def test_occupancy_preserves_free_occupied_unknown(self):
        _, _, yaml = self.build_and_save()
        loaded = load_map(str(yaml))
        wall_index = loaded.cell(1.0, 0.0)
        free_index = loaded.cell(0.5, 0.0)
        unknown_index = loaded.cell(-1.5, 1.5)
        self.assertEqual(loaded.occupancy[wall_index], 100)
        self.assertEqual(loaded.occupancy[free_index], 0)
        self.assertEqual(loaded.occupancy[unknown_index], -1)

    def test_pick_latest_map_returns_newest_by_mtime(self):
        older = Path(self.tmpdir, 'maze_older.yaml')
        newer = Path(self.tmpdir, 'maze_newer.yaml')
        older.write_text('resolution: 0.1\norigin: [0, 0, 0]\nimage: x.pgm\n')
        newer.write_text('resolution: 0.1\norigin: [0, 0, 0]\nimage: x.pgm\n')
        older_time = 1000000000
        os.utime(older, (older_time, older_time))
        os.utime(newer, (older_time + 100, older_time + 100))
        self.assertEqual(pick_latest_map(self.tmpdir), str(newer))

    def test_missing_maps_dir_raises(self):
        with self.assertRaises(FileNotFoundError):
            pick_latest_map(str(Path(self.tmpdir) / 'nonexistent'))
