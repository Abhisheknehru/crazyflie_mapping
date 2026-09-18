"""Deterministic tests for the pure-Python Monte Carlo localizer."""
import math
import random
import unittest

from test_runs.map_loader import LocalizationMap
from test_runs.particle_filter import ParticleFilter

FACE_ANGLES = {'front': 0., 'left': math.pi / 2,
               'back': math.pi, 'right': -math.pi / 2}


def box_map(resolution=.1, size=4.0, wall_at=1.0):
    """A square arena with a single full-height wall line at x=wall_at."""
    width = round(size / resolution)
    occupied = [False] * (width * width)
    lmap = LocalizationMap(resolution, -width * resolution / 2, width, occupied)
    ix = math.floor((wall_at - lmap.origin) / resolution)
    for iy in range(width):
        occupied[iy * width + ix] = True
    return lmap


class ParticleFilterTests(unittest.TestCase):
    def test_predict_shifts_weighted_mean_along_shared_heading(self):
        # With every particle facing the same way, forward motion should
        # move the weighted mean in that shared direction, same as before.
        pf = ParticleFilter(box_map(), count=200, rng=random.Random(1))
        for p in pf.particles:
            p.yaw = 0.
        x0, y0, _ = pf.estimate()
        pf.predict(.3, .0, .0)
        x1, y1, _ = pf.estimate()
        self.assertAlmostEqual(x1 - x0, .3, delta=.1)
        self.assertAlmostEqual(y1 - y0, 0., delta=.1)

    def test_predict_steers_by_each_particles_own_yaw(self):
        # This is the actual fix: two particles with DIFFERENT heading
        # hypotheses must move in DIFFERENT map-frame directions for the
        # SAME body-frame motion command -- not one shared world delta.
        # This is what makes the motion model correct even when this
        # session's raw-pose "zero heading" doesn't match the saved map's.
        pf = ParticleFilter(box_map(), count=2, rng=random.Random(1),
                            alpha_trans=0., trans_floor=0.,
                            alpha_rot=0., rot_floor=0.)
        pf.particles[0].x, pf.particles[0].y, pf.particles[0].yaw = 0., 0., 0.
        pf.particles[1].x, pf.particles[1].y, pf.particles[1].yaw = 0., 0., math.pi / 2
        pf.predict(1.0, 0.0, 0.0)
        self.assertAlmostEqual(pf.particles[0].x, 1.0, delta=.05)
        self.assertAlmostEqual(pf.particles[0].y, 0.0, delta=.05)
        self.assertAlmostEqual(pf.particles[1].x, 0.0, delta=.05)
        self.assertAlmostEqual(pf.particles[1].y, 1.0, delta=.05)

    def test_update_favors_particles_consistent_with_reading(self):
        lmap = box_map(wall_at=1.0)
        pf = ParticleFilter(lmap, count=2, rng=random.Random(2))
        # particle 0 sits where the true front reading (1.0) matches exactly.
        pf.particles[0].x, pf.particles[0].y, pf.particles[0].yaw = 0., 0., 0.
        # particle 1 is offset, so its predicted front distance (0.5) does not.
        pf.particles[1].x, pf.particles[1].y, pf.particles[1].yaw = .5, 0., 0.
        pf.particles[0].weight = pf.particles[1].weight = .5
        pf.update({'front': 1.0}, FACE_ANGLES)
        self.assertGreater(pf.particles[0].weight, pf.particles[1].weight)

    def test_resample_converges_toward_true_pose(self):
        # An axis-aligned rectangular room, off-center around the true pose,
        # gives each of the 4 rays a genuinely different distance -- a
        # fingerprint that rules out the rotational/reflective ambiguity a
        # symmetric room or a single wall would leave unresolved.
        resolution, size = .1, 8.0
        width = round(size / resolution)
        occupied = [False] * (width * width)
        lmap = LocalizationMap(resolution, -width * resolution / 2, width, occupied)
        x_min, x_max, y_min, y_max = -3, 1, -1, 2
        for i in range(width):
            x = lmap.origin + (i + .5) * resolution
            for edge_y in (y_min, y_max):
                index = lmap.cell(x, edge_y)
                if index is not None:
                    occupied[index] = True
            y = lmap.origin + (i + .5) * resolution
            for edge_x in (x_min, x_max):
                index = lmap.cell(edge_x, y)
                if index is not None:
                    occupied[index] = True

        pf = ParticleFilter(lmap, count=300, rng=random.Random(4))
        true_x, true_y = 0.0, 0.0
        for _ in range(40):
            readings = {}
            for face, angle in FACE_ANGLES.items():
                readings[face] = lmap.raycast(true_x, true_y, angle, 4.0)
            pf.update(readings, FACE_ANGLES)
            # random_inject keeps sampling fresh candidate poses each round,
            # so the estimate can refine beyond the initial particles' spacing.
            pf.resample(random_inject=0.1)
        x, y, _ = pf.estimate()
        self.assertAlmostEqual(x, true_x, delta=.3)
        self.assertAlmostEqual(y, true_y, delta=.3)

    def test_estimate_handles_yaw_wraparound(self):
        pf = ParticleFilter(box_map(), count=2, rng=random.Random(5))
        pf.particles[0].x = pf.particles[0].y = 0.
        pf.particles[1].x = pf.particles[1].y = 0.
        pf.particles[0].yaw = math.pi - .01
        pf.particles[1].yaw = -math.pi + .01
        pf.particles[0].weight = pf.particles[1].weight = .5
        _, _, yaw = pf.estimate()
        self.assertAlmostEqual(abs(yaw), math.pi, delta=.05)

    def test_yaw_concentration_high_when_all_agree(self):
        pf = ParticleFilter(box_map(), count=50, rng=random.Random(6))
        for p in pf.particles:
            p.yaw = 0.3
        self.assertGreater(pf.yaw_concentration(), 0.99)

    def test_yaw_concentration_low_for_opposite_split(self):
        # Position can look converged while heading is still split between
        # two opposite hypotheses -- exactly the front/back symmetric-
        # corridor case that a position-only spread check would miss.
        pf = ParticleFilter(box_map(), count=2, rng=random.Random(7))
        pf.particles[0].yaw, pf.particles[0].weight = 0.0, .5
        pf.particles[1].yaw, pf.particles[1].weight = math.pi, .5
        self.assertLess(pf.yaw_concentration(), 0.05)

    def test_yaw_concentration_low_when_uniformly_spread(self):
        pf = ParticleFilter(box_map(), count=8, rng=random.Random(8))
        for i, p in enumerate(pf.particles):
            p.yaw = -math.pi + i * (2 * math.pi / len(pf.particles))
            p.weight = 1.0 / len(pf.particles)
        self.assertLess(pf.yaw_concentration(), 0.05)
