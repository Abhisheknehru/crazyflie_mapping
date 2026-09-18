"""Monte Carlo localization against a LocalizationMap. No ROS, no numpy."""
from dataclasses import dataclass
import math
import random


@dataclass
class Particle:
    x: float
    y: float
    yaw: float
    weight: float


def _gaussian_noise(rng, std):
    return rng.gauss(0.0, std) if std > 0 else 0.0


class ParticleFilter:
    def __init__(self, localization_map, count=800, rng=None,
                 alpha_trans=0.1, trans_floor=0.005,
                 alpha_rot=0.1, rot_floor=0.01):
        self.map = localization_map
        self.rng = rng if rng is not None else random.Random()
        self.alpha_trans = alpha_trans
        self.trans_floor = trans_floor
        self.alpha_rot = alpha_rot
        self.rot_floor = rot_floor
        self.particles = [self._random_particle() for _ in range(count)]
        for p in self.particles:
            p.weight = 1.0 / count

    def _random_particle(self):
        half = self.map.width * self.map.resolution / 2
        for _ in range(1000):
            x = self.rng.uniform(-half, half)
            y = self.rng.uniform(-half, half)
            index = self.map.cell(x, y)
            if index is not None and not self.map.occupied[index]:
                yaw = self.rng.uniform(-math.pi, math.pi)
                return Particle(x, y, yaw, 1.0)
        raise ValueError('Could not find free space to seed a particle')

    def predict(self, forward, lateral, dyaw):
        """Body-frame motion (forward/lateral relative to heading, plus a
        turn) applied per-particle using EACH particle's own yaw hypothesis
        -- not a single shared world-frame delta. This is what makes MCL
        correct without knowing the offset between wherever this session's
        raw pose calls "zero heading" and the saved map's own frame: every
        particle steers the same physical motion by its own guess of which
        way it's actually facing.
        """
        distance = math.hypot(forward, lateral)
        trans_std = self.alpha_trans * distance + self.trans_floor
        rot_std = self.alpha_rot * abs(dyaw) + self.rot_floor
        for p in self.particles:
            c, s = math.cos(p.yaw), math.sin(p.yaw)
            dx = c * forward - s * lateral
            dy = s * forward + c * lateral
            p.x += dx + _gaussian_noise(self.rng, trans_std)
            p.y += dy + _gaussian_noise(self.rng, trans_std)
            new_yaw = p.yaw + dyaw + _gaussian_noise(self.rng, rot_std)
            p.yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))

    def update(self, readings, sensor_angles, max_range=4.0,
               sigma_base=0.05, sigma_slope=0.05):
        total = 0.0
        for p in self.particles:
            likelihood = 1.0
            for face, distance in readings.items():
                if distance is None or distance >= max_range - 1e-5:
                    continue
                angle = p.yaw + sensor_angles[face]
                predicted = self.map.raycast(p.x, p.y, angle, max_range)
                sigma = sigma_base + sigma_slope * distance
                error = distance - predicted
                likelihood *= math.exp(-(error * error) / (2 * sigma * sigma))
            p.weight *= likelihood
            total += p.weight
        if total > 0:
            for p in self.particles:
                p.weight /= total
        else:
            uniform = 1.0 / len(self.particles)
            for p in self.particles:
                p.weight = uniform

    def yaw_concentration(self):
        """1.0 = every particle agrees on heading; ~0 = spread out or split
        between opposing hypotheses (e.g. a front/back symmetric corridor)
        even if position has converged. Mean resultant length of the
        weighted yaw distribution -- the standard circular-statistics
        measure for exactly this."""
        total = sum(p.weight for p in self.particles) or 1.0
        sin_sum = sum(math.sin(p.yaw) * p.weight for p in self.particles)
        cos_sum = sum(math.cos(p.yaw) * p.weight for p in self.particles)
        return math.hypot(sin_sum, cos_sum) / total

    def effective_sample_size(self):
        """Diagnostic only (not used to gate resampling -- see resample())."""
        total = sum(p.weight for p in self.particles) or 1.0
        return 1.0 / sum((p.weight / total) ** 2 for p in self.particles)

    def resample(self, random_inject=0.02):
        # Always resampling (rather than gating on effective sample size)
        # is deliberate here: skipping it once weights look uniform risks
        # freezing the particle set onto whatever the last resample
        # produced -- if that happened to be a slightly-wrong cluster, a
        # filter that isn't moving (or moving little) may never revisit it,
        # since resampling is also the only place fresh random_inject
        # candidates get introduced. Tested and confirmed this can
        # permanently lock onto the wrong pose in a static scene.
        count = len(self.particles)
        inject_count = max(0, round(count * random_inject))
        keep_count = count - inject_count
        step = 1.0 / keep_count
        start = self.rng.uniform(0, step)
        cumulative = self.particles[0].weight
        index = 0
        resampled = []
        for i in range(keep_count):
            target = start + i * step
            while cumulative < target and index < count - 1:
                index += 1
                cumulative += self.particles[index].weight
            source = self.particles[index]
            resampled.append(Particle(source.x, source.y, source.yaw, 1.0 / count))
        for _ in range(inject_count):
            fresh = self._random_particle()
            fresh.weight = 1.0 / count
            resampled.append(fresh)
        self.particles = resampled

    def estimate(self):
        total = sum(p.weight for p in self.particles) or 1.0
        x = sum(p.x * p.weight for p in self.particles) / total
        y = sum(p.y * p.weight for p in self.particles) / total
        sin_sum = sum(math.sin(p.yaw) * p.weight for p in self.particles)
        cos_sum = sum(math.cos(p.yaw) * p.weight for p in self.particles)
        yaw = math.atan2(sin_sum, cos_sum)
        return x, y, yaw
