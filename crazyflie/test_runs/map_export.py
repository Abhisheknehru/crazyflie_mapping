"""Save a GridMap as ROS-compatible PGM and YAML files."""
from datetime import datetime
from pathlib import Path


def save_grid(grid, prefix):
    values = grid.occupancy()
    if not grid.seen:
        raise ValueError('map is empty')
    base = Path(prefix).expanduser().resolve()
    base.parent.mkdir(parents=True, exist_ok=True)
    suffix = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    base = base.with_name(f'{base.name}_{suffix}')
    pgm = base.with_suffix('.pgm')
    yaml = base.with_suffix('.yaml')

    pixels = bytearray()
    for y in range(grid.width - 1, -1, -1):
        for x in range(grid.width):
            value = values[y * grid.width + x]
            pixels.append(0 if value >= 65 else
                          254 if 0 <= value <= 25 else 205)
    with pgm.open('xb') as stream:
        stream.write(f'P5\n{grid.width} {grid.width}\n255\n'.encode('ascii'))
        stream.write(pixels)
    try:
        with yaml.open('x') as stream:
            stream.write(
                f'image: {pgm.name}\n'
                'mode: trinary\n'
                f'resolution: {grid.resolution}\n'
                f'origin: [{grid.origin}, {grid.origin}, 0.0]\n'
                'negate: 0\n'
                'occupied_thresh: 0.65\n'
                'free_thresh: 0.196\n')
    except OSError:
        pgm.unlink(missing_ok=True)
        raise
    return pgm, yaml
