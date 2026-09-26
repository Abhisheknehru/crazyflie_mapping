"""A* path planning on the saved Crazyflie occupancy map."""
import heapq
import math


def body_velocity(current_pose, target, gain=0.8, max_speed=.10):
    """Map-frame position error converted to body-frame velocity."""
    x, y, yaw = current_pose
    dx, dy = target[0] - x, target[1] - y
    distance = math.hypot(dx, dy)
    if distance == 0:
        return 0.0, 0.0, 0.0
    speed = min(max_speed, gain * distance)
    map_vx, map_vy = speed * dx / distance, speed * dy / distance
    c, s = math.cos(yaw), math.sin(yaw)
    forward = c * map_vx + s * map_vy
    left = -s * map_vx + c * map_vy
    return forward, left, distance


class AStarPlanner:
    """Plan in known free space after inflating walls for drone clearance."""

    def __init__(self, localization_map, clearance=.10,
                 unknown_is_blocked=True, orthogonal_paths=True):
        self.map = localization_map
        self.width = localization_map.width
        self.clearance = clearance
        self.orthogonal_paths = orthogonal_paths
        blocked = [
            occupied or (unknown_is_blocked and occupancy < 0)
            for occupied, occupancy in zip(
                localization_map.occupied, localization_map.occupancy)
        ]
        self.blocked = self._inflate(blocked)

    def _inflate(self, blocked):
        radius = math.ceil(self.clearance / self.map.resolution)
        inflated = list(blocked)
        occupied_cells = [
            (index % self.width, index // self.width)
            for index, value in enumerate(blocked) if value
        ]
        for x, y in occupied_cells:
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if math.hypot(dx, dy) > radius:
                        continue
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.width and 0 <= ny < self.width:
                        inflated[ny * self.width + nx] = True
        return inflated

    def world_to_cell(self, x, y):
        index = self.map.cell(x, y)
        if index is None:
            return None
        return index % self.width, index // self.width

    def cell_to_world(self, cell):
        x, y = cell
        half = self.map.resolution / 2.0
        return (self.map.origin + x * self.map.resolution + half,
                self.map.origin + y * self.map.resolution + half)

    def is_free(self, cell):
        if cell is None:
            return False
        x, y = cell
        return (0 <= x < self.width and 0 <= y < self.width and
                not self.blocked[y * self.width + x])

    def plan(self, start_world, goal_world):
        """Return simplified map-frame waypoints, or raise ValueError."""
        start = self.world_to_cell(*start_world)
        goal = self.world_to_cell(*goal_world)
        if not self.is_free(start):
            raise ValueError('Start is outside known clear space')
        if not self.is_free(goal):
            raise ValueError('Goal is outside known clear space')
        cells = self._a_star(start, goal)
        if not cells:
            raise ValueError('No collision-free path to the goal')
        cells = self._simplify(cells)
        points = [self.cell_to_world(cell) for cell in cells]
        if not self.orthogonal_paths:
            points[0] = tuple(start_world)
            points[-1] = tuple(goal_world)
            return points
        return self._orthogonal_endpoints(points, start_world, goal_world)

    @staticmethod
    def _orthogonal_endpoints(cell_points, start, goal):
        """Connect exact poses to cell centres without diagonal segments."""
        result = [tuple(start)]

        def append(point):
            point = tuple(point)
            if point != result[-1]:
                result.append(point)

        if len(cell_points) == 1:
            append((goal[0], start[1]))
            append(goal)
            return result

        first, second = cell_points[0], cell_points[1]
        if first[0] != second[0]:
            append((second[0], start[1]))
        else:
            append((start[0], second[1]))
        append(second)
        for point in cell_points[2:]:
            append(point)

        last = result[-1]
        append((goal[0], last[1]))
        append(goal)
        return result

    def _a_star(self, start, goal):
        queue = [(0.0, 0.0, start)]
        parent = {start: None}
        cost = {start: 0.0}
        while queue:
            _, current_cost, current = heapq.heappop(queue)
            if current_cost != cost.get(current):
                continue
            if current == goal:
                return self._reconstruct(parent, goal)
            for neighbour, step_cost in self._neighbours(current):
                new_cost = current_cost + step_cost
                if new_cost >= cost.get(neighbour, math.inf):
                    continue
                cost[neighbour] = new_cost
                parent[neighbour] = current
                heuristic = math.hypot(
                    goal[0] - neighbour[0], goal[1] - neighbour[1])
                heapq.heappush(
                    queue, (new_cost + heuristic, new_cost, neighbour))
        return []

    def _neighbours(self, cell):
        x, y = cell
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if not self.orthogonal_paths:
            directions += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        for dx, dy in directions:
            neighbour = (x + dx, y + dy)
            if not self.is_free(neighbour):
                continue
            if dx and dy:
                # Do not squeeze diagonally through the corner of two walls.
                if not (self.is_free((x + dx, y)) and
                        self.is_free((x, y + dy))):
                    continue
            yield neighbour, math.hypot(dx, dy)

    def _simplify(self, cells):
        if len(cells) <= 2:
            return cells
        result = [cells[0]]
        anchor = 0
        while anchor < len(cells) - 1:
            furthest = anchor + 1
            for candidate in range(anchor + 2, len(cells)):
                if not self._line_is_free(cells[anchor], cells[candidate]):
                    break
                furthest = candidate
            result.append(cells[furthest])
            anchor = furthest
        return result

    def _line_is_free(self, start, goal):
        x0, y0 = start
        x1, y1 = goal
        if self.orthogonal_paths and x0 != x1 and y0 != y1:
            return False
        steps = max(abs(x1 - x0), abs(y1 - y0))
        if steps == 0:
            return self.is_free(start)
        previous = start
        for step in range(steps + 1):
            fraction = step / steps
            cell = (round(x0 + fraction * (x1 - x0)),
                    round(y0 + fraction * (y1 - y0)))
            if not self.is_free(cell):
                return False
            if cell[0] != previous[0] and cell[1] != previous[1]:
                if not (self.is_free((cell[0], previous[1])) and
                        self.is_free((previous[0], cell[1]))):
                    return False
            previous = cell
        return True

    @staticmethod
    def _reconstruct(parent, goal):
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = parent[current]
        return list(reversed(path))
