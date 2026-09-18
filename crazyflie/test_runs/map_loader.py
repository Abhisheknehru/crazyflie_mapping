"""Reload a GridMap saved by map_export.save_grid, for localization."""
import glob
import math
import os


def _parse_yaml(path):
    fields = {}
    with open(path) as stream:
        for line in stream:
            key, _, value = line.partition(':')
            key, value = key.strip(), value.strip()
            if key == 'resolution':
                fields['resolution'] = float(value)
            elif key == 'origin':
                fields['origin'] = float(value.strip('[]').split(',')[0])
            elif key == 'image':
                fields['image'] = value
    if 'resolution' not in fields or 'origin' not in fields or 'image' not in fields:
        raise ValueError(f'Incomplete map yaml: {path}')
    return fields


def _read_pgm(path):
    with open(path, 'rb') as stream:
        magic = stream.readline().strip()
        if magic != b'P5':
            raise ValueError(f'Not a raw PGM (P5): {path}')
        width, height = (int(v) for v in stream.readline().split())
        max_value = int(stream.readline())
        if max_value != 255:
            raise ValueError(f'Unsupported PGM maxval {max_value}: {path}')
        pixels = stream.read(width * height)
        if len(pixels) != width * height:
            raise ValueError(f'Truncated PGM data: {path}')
        return width, height, pixels


class LocalizationMap:
    def __init__(self, resolution, origin, width, occupied, occupancy=None):
        self.resolution = resolution
        self.origin = origin
        self.width = width
        self.occupied = occupied
        # -1 unknown, 0 free, 100 occupied -- same convention as
        # GridMap.occupancy()/nav_msgs OccupancyGrid. Optional: raycast()/
        # path_is_clear() only need `occupied`, this is for visualization.
        self.occupancy = occupancy if occupancy is not None else [
            100 if o else -1 for o in occupied]

    def cell(self, x, y):
        ix = math.floor((x - self.origin) / self.resolution)
        iy = math.floor((y - self.origin) / self.resolution)
        if 0 <= ix < self.width and 0 <= iy < self.width:
            return iy * self.width + ix
        return None

    def raycast(self, x, y, angle, max_range):
        c, s = math.cos(angle), math.sin(angle)
        steps = max(1, math.ceil(max_range / self.resolution))
        for n in range(steps + 1):
            d = max_range * n / steps
            index = self.cell(x + d * c, y + d * s)
            if index is None:
                return d
            if self.occupied[index]:
                return d
        return max_range


def load_map(yaml_path):
    fields = _parse_yaml(yaml_path)
    pgm_path = os.path.join(os.path.dirname(yaml_path), fields['image'])
    width, height, pixels = _read_pgm(pgm_path)
    if width != height:
        raise ValueError(f'Map must be square: {pgm_path}')
    occupied = [False] * (width * height)
    occupancy = [-1] * (width * height)
    for r in range(height):
        iy = height - 1 - r
        for c in range(width):
            pixel = pixels[r * width + c]
            index = iy * width + c
            if pixel == 0:
                occupied[index] = True
                occupancy[index] = 100
            elif pixel == 254:
                occupancy[index] = 0
    return LocalizationMap(fields['resolution'], fields['origin'], width,
                           occupied, occupancy)


def pick_latest_map(maps_dir='maps'):
    candidates = glob.glob(os.path.join(maps_dir, 'maze_*.yaml'))
    if not candidates:
        raise FileNotFoundError(f'No maze_*.yaml maps found in {maps_dir}')
    return max(candidates, key=os.path.getmtime)
