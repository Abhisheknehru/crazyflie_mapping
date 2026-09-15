"""Fixed-heading, cardinal-motion exploration; independent of ROS and flight IO."""
import math


class ReactivePolicy:
    """Follow right-side boundaries without rotating the vehicle.

    Directions are body front, left, back, right. This assumes heading is held
    by the flight controller. It is a local heuristic, not complete exploration.
    """

    def __init__(self, speed=0.10, clearance=0.30, wall_distance=0.45,
                 opening=0.75):
        if not all(math.isfinite(v) for v in
                   (speed, clearance, wall_distance, opening)):
            raise ValueError('Parameters must be finite')
        if not (0 < speed <= 0.3 and 0 < clearance < wall_distance < opening):
            raise ValueError('Require 0 < speed <= .3 and clearance < wall_distance < opening')
        self.speed = speed
        self.clearance = clearance
        self.wall_distance = wall_distance
        self.opening = opening
        self.direction = 0
        self.saw_right_wall = False

    def step(self, ranges):
        """Return body vx, vy (m/s) and a diagnostic state; invalid input stops."""
        names = ('front', 'left', 'back', 'right')
        if any(n not in ranges or not math.isfinite(ranges[n]) or ranges[n] <= 0
               for n in names):
            return 0.0, 0.0, 'INVALID_RANGE'
        distances = [ranges[n] for n in names]
        if min(distances) <= self.clearance:
            return 0.0, 0.0, 'CLEARANCE_STOP'
        forward = self.direction
        right = (forward - 1) % 4
        if distances[right] < self.opening - 0.10:
            self.saw_right_wall = True
        if self.saw_right_wall and distances[right] > self.opening:
            self.direction = right
            self.saw_right_wall = False
            return 0.0, 0.0, 'RIGHT_OPENING'
        if distances[forward] < self.wall_distance:
            # Right-hand preference at a blocked corridor, including dead ends.
            candidates = (right, (forward + 1) % 4, (forward + 2) % 4)
            for candidate in candidates:
                if distances[candidate] > self.wall_distance + 0.10:
                    self.direction = candidate
                    self.saw_right_wall = False
                    return 0.0, 0.0, 'CHANGE_DIRECTION'
            return 0.0, 0.0, 'BLOCKED'
        axes = ((1, 0), (0, 1), (-1, 0), (0, -1))
        fx, fy = axes[forward]
        rx, ry = axes[right]
        correction = 0.0
        if distances[right] < self.opening:
            correction = max(-self.speed / 2, min(self.speed / 2,
                             0.4 * (distances[right] - self.wall_distance)))
        vx = self.speed * fx + correction * rx
        vy = self.speed * fy + correction * ry
        # Do not let lateral correction enter a nearby wall.
        for value, direction in ((vx, 0 if vx >= 0 else 2),
                                 (vy, 1 if vy >= 0 else 3)):
            if value and distances[direction] < self.wall_distance:
                if direction in (0, 2):
                    vx = 0.0
                else:
                    vy = 0.0
        return vx, vy, 'FOLLOW'
