"""Small occupancy grid for planar range-ray mapping."""
import math


class GridMap:
    def __init__(self, resolution=0.02, size=10.0):
        if not (math.isfinite(resolution) and math.isfinite(size) and
                0.01 <= resolution <= 0.5 and resolution <= size <= 50):
            raise ValueError('Invalid grid resolution or size')
        self.resolution = resolution
        self.width = math.ceil(size / resolution)
        if self.width ** 2 > 1000000:
            raise ValueError('Grid exceeds one million cells')
        self.origin = -self.width * resolution / 2
        self.values = [0] * (self.width ** 2)
        self.seen = set()

    def cell(self, x, y):
        ix = math.floor((x - self.origin) / self.resolution)
        iy = math.floor((y - self.origin) / self.resolution)
        if 0 <= ix < self.width and 0 <= iy < self.width:
            return iy * self.width + ix
        return None

    def ray(self, x, y, angle, distance, hit, hit_weight=4):
        if not all(math.isfinite(v) for v in (x, y, angle, distance)) or distance <= 0:
            return
        if self.cell(x, y) is None:
            return
        c, s = math.cos(angle), math.sin(angle)
        endpoint = self.cell(x + distance * c, y + distance * s)
        cells = set()
        steps = max(1, math.ceil(distance / (self.resolution / 2)))
        for n in range(steps + 1):
            d = distance * n / steps
            index = self.cell(x + d * c, y + d * s)
            if index is None:
                break
            cells.add(index)
        for index in cells:
            increment = hit_weight if hit and index == endpoint else -1
            self.values[index] = max(-10, min(10, self.values[index] + increment))
            self.seen.add(index)

    def occupancy(self):
        return [(-1 if i not in self.seen else
                 round(100 / (1 + math.exp(-v)))) for i, v in enumerate(self.values)]

    def clear(self):
        self.values = [0] * (self.width ** 2)
        self.seen.clear()
