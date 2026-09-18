import tempfile
import unittest
from pathlib import Path

from test_runs.grid_map import GridMap
from test_runs.map_export import save_grid


class MapExportTests(unittest.TestCase):
    def test_save_nonempty_grid(self):
        grid = GridMap(size=1.)
        grid.ray(0., 0., 0., .2, True)
        with tempfile.TemporaryDirectory() as folder:
            pgm, yaml = save_grid(grid, str(Path(folder) / 'maze'))
            self.assertTrue(pgm.exists())
            self.assertTrue(yaml.exists())
            self.assertIn('resolution: 0.02', yaml.read_text())

    def test_empty_grid_is_not_saved(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):
                save_grid(GridMap(size=1.), str(Path(folder) / 'maze'))
