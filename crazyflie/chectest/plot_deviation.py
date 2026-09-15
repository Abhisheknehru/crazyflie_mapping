#!/usr/bin/env python3

import argparse
import csv
import json
import math
import socket
import struct
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


COMBINED_FORMAT = '<IHHHHffff'
COMBINED_SIZE = struct.calcsize(COMBINED_FORMAT)
LEGACY_FORMAT = '<IHHHHf'
LEGACY_SIZE = struct.calcsize(LEGACY_FORMAT)


def to_metres(value):
    """Convert UDP range values to metres.

    The ESP bridge usually sends millimetres. JSON packets may already be in
    metres, so values under 10 are treated as metres.
    """
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        return None
    if value > 10.0:
        value /= 1000.0
    return value


def parse_packet(data):
    """Return front, back, left, right in metres, or None for bad packets."""
    front = back = left = right = None

    try:
        payload = data.decode('utf-8').strip()
    except UnicodeDecodeError:
        payload = None

    if payload and (payload.startswith('{') or ',' in payload):
        try:
            if payload.startswith('{'):
                ranges = json.loads(payload)
                front = ranges.get('front')
                back = ranges.get('back')
                left = ranges.get('left')
                right = ranges.get('right')
            else:
                fields = [float(value) for value in payload.split(',')]
                if len(fields) < 4:
                    return None
                front, back, left, right = fields[:4]
        except (ValueError, json.JSONDecodeError):
            return None
    elif len(data) >= COMBINED_SIZE:
        _, front, right, back, left, *_pose = struct.unpack(
            COMBINED_FORMAT, data[:COMBINED_SIZE]
        )
    elif len(data) >= LEGACY_SIZE:
        _, front, right, back, left, _hz = struct.unpack(
            LEGACY_FORMAT, data[:LEGACY_SIZE]
        )
    else:
        return None

    readings = tuple(to_metres(value) for value in (front, back, left, right))
    if any(value is None for value in readings):
        return None
    return readings


def collect_udp_data(host, port, duration, sample_time):
    front_data = []
    back_data = []
    left_data = []
    right_data = []
    timestamps = []

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(sample_time)
    sock.bind((host, port))

    print(f'Listening for UDP data on {host}:{port}')
    print('Keep the drone completely stationary...')
    print('Collecting data...')

    start = time.time()
    while time.time() - start < duration:
        try:
            data, _addr = sock.recvfrom(1024)
        except socket.timeout:
            continue

        parsed = parse_packet(data)
        if parsed is None:
            continue

        front, back, left, right = parsed
        timestamps.append(time.time() - start)
        front_data.append(front)
        back_data.append(back)
        left_data.append(left)
        right_data.append(right)

    sock.close()
    return timestamps, front_data, back_data, left_data, right_data


def write_csv(path, timestamps, front, back, left, right):
    with path.open('w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['time_s', 'front_m', 'back_m', 'left_m', 'right_m'])
        writer.writerows(zip(timestamps, front, back, left, right))


def plot_sensor(name, timestamps, data, show_plot=True, output_dir=None):
    data = np.array(data, dtype=float)
    timestamps = np.array(timestamps, dtype=float)

    mean = np.mean(data)
    std = np.std(data)

    plt.figure(figsize=(12, 4))
    plt.plot(timestamps, data, label=name)
    plt.axhline(mean, linestyle='--', linewidth=2, label=f'Mean = {mean:.3f}')
    plt.fill_between(timestamps, mean - std, mean + std, alpha=0.25, label='+/-1 sigma')

    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.title(f'{name} Noise Analysis')
    plt.grid(True)
    plt.legend()

    if output_dir is not None:
        output_path = output_dir / f'{name.lower().replace(" ", "_")}.png'
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f'Saved plot: {output_path}')

    if show_plot:
        plt.show()
    else:
        plt.close()

    print(f'\n{name}')
    print('---------------------------')
    print(f'Samples          : {len(data)}')
    print(f'Mean             : {mean:.5f} m')
    print(f'Std Deviation    : {std:.5f} m')
    print(f'Minimum          : {np.min(data):.5f} m')
    print(f'Maximum          : {np.max(data):.5f} m')
    print(f'Peak-to-Peak     : {np.ptp(data):.5f} m')
    print(f'3 sigma Threshold: {3 * std:.5f} m')


def main():
    parser = argparse.ArgumentParser(
        description='Collect UDP range data and plot sensor deviation.'
    )
    parser.add_argument('--host', default='0.0.0.0', help='UDP bind address')
    parser.add_argument('--port', type=int, default=5005, help='UDP port')
    parser.add_argument('--duration', type=float, default=60.0, help='Collection duration in seconds')
    parser.add_argument('--sample-time', type=float, default=0.05, help='Socket timeout in seconds')
    parser.add_argument('--csv', default='deviation_data.csv', help='CSV output path')
    parser.add_argument('--save-plots', action='store_true', help='Save plots as PNG files')
    parser.add_argument('--no-show', action='store_true', help='Do not open plot windows')
    args = parser.parse_args()

    timestamps, front, back, left, right = collect_udp_data(
        args.host, args.port, args.duration, args.sample_time
    )

    if not timestamps:
        print('No valid UDP range packets received.')
        return

    csv_path = Path(args.csv)
    write_csv(csv_path, timestamps, front, back, left, right)
    print(f'\nSaved data: {csv_path}')

    output_dir = csv_path.parent if args.save_plots else None
    if output_dir is not None:
        output_dir.mkdir(parents=True, exist_ok=True)

    show_plot = not args.no_show
    plot_sensor('Front Sensor', timestamps, front, show_plot, output_dir)
    plot_sensor('Back Sensor', timestamps, back, show_plot, output_dir)
    plot_sensor('Left Sensor', timestamps, left, show_plot, output_dir)
    plot_sensor('Right Sensor', timestamps, right, show_plot, output_dir)


if __name__ == '__main__':
    main()
