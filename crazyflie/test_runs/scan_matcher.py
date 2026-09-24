"""Small correlative scan matcher for sparse Multi-ranger measurements.

This module contains no ROS code, which makes the matching math easy to test.
Points are stored in the Crazyflie's local odometry frame.  An Alignment maps
those points into the fixed saved-map frame.
"""
from dataclasses import dataclass
import heapq
import math


@dataclass
class Alignment:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class MatchResult:
    alignment: Alignment
    mean_error: float
    point_count: int


def transform_point(point, alignment):
    """Transform one (x, y) point from odometry into the map frame."""
    c, s = math.cos(alignment.yaw), math.sin(alignment.yaw)
    x, y = point
    return (alignment.x + c * x - s * y,
            alignment.y + s * x + c * y)


class DistanceField:
    """Distance from every map cell to its nearest occupied cell."""

    def __init__(self, localization_map):
        self.map = localization_map
        self.distances = [math.inf] * (localization_map.width ** 2)
        queue = []
        for index, occupied in enumerate(localization_map.occupied):
            if occupied:
                self.distances[index] = 0.0
                heapq.heappush(queue, (0.0, index))
        if not queue:
            raise ValueError('Cannot scan-match against a map with no walls')
        self._fill(queue)

    def _fill(self, queue):
        width = self.map.width
        straight = self.map.resolution
        diagonal = straight * math.sqrt(2.0)
        neighbours = ((-1, -1, diagonal), (0, -1, straight),
                      (1, -1, diagonal), (-1, 0, straight),
                      (1, 0, straight), (-1, 1, diagonal),
                      (0, 1, straight), (1, 1, diagonal))
        while queue:
            distance, index = heapq.heappop(queue)
            if distance != self.distances[index]:
                continue
            x, y = index % width, index // width
            for dx, dy, cost in neighbours:
                nx, ny = x + dx, y + dy
                if not (0 <= nx < width and 0 <= ny < width):
                    continue
                neighbour = ny * width + nx
                candidate = distance + cost
                if candidate < self.distances[neighbour]:
                    self.distances[neighbour] = candidate
                    heapq.heappush(queue, (candidate, neighbour))

    def distance(self, x, y):
        index = self.map.cell(x, y)
        if index is None:
            return 2.0  # Strong, finite penalty for leaving the map.
        return self.distances[index]


class CorrelativeScanMatcher:
    def __init__(self, localization_map):
        self.field = DistanceField(localization_map)

    def mean_error(self, points, alignment):
        if not points:
            return math.inf
        error = 0.0
        for point in points:
            x, y = transform_point(point, alignment)
            # Cap the contribution so one bad range return cannot dominate.
            error += min(1.0, self.field.distance(x, y))
        return error / len(points)

    def match(self, points, initial, xy_window=.35, yaw_window=.26,
              xy_step=.05, yaw_step=.052):
        """Coarse search followed by a smaller refinement search."""
        if not points:
            return MatchResult(initial, math.inf, 0)

        coarse = self._search(points, initial, xy_window, yaw_window,
                              xy_step, yaw_step)
        refined = self._search(points, coarse.alignment,
                               xy_step, yaw_step,
                               xy_step / 5.0, yaw_step / 5.0)
        return refined

    def _search(self, points, centre, xy_window, yaw_window,
                xy_step, yaw_step):
        best = Alignment(centre.x, centre.y, centre.yaw)
        best_error = self.mean_error(points, best)
        for dx in _steps(xy_window, xy_step):
            for dy in _steps(xy_window, xy_step):
                for dyaw in _steps(yaw_window, yaw_step):
                    candidate = Alignment(
                        centre.x + dx, centre.y + dy,
                        _wrap(centre.yaw + dyaw))
                    error = self.mean_error(points, candidate)
                    if error < best_error:
                        best, best_error = candidate, error
        return MatchResult(best, best_error, len(points))


def _steps(window, step):
    count = max(0, math.ceil(window / step))
    return (index * step for index in range(-count, count + 1))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))
