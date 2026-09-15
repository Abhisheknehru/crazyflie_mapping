#!/usr/bin/env python3

"""Range-only geometry analyzer for multiranger events.

This module does not estimate pose. It only turns front/back/left/right range
changes into symbolic geometry events that a pose constraint engine can use.
"""

import math
from dataclasses import dataclass


NONE = 'NONE'
WALL_APPEAR = 'WALL_APPEAR'
WALL_DISAPPEAR = 'WALL_DISAPPEAR'
OPENING = 'OPENING'
CORRIDOR = 'CORRIDOR'
CORNER = 'CORNER'
DEAD_END = 'DEAD_END'
OPEN_SPACE = 'OPEN_SPACE'
UNKNOWN = 'UNKNOWN'

FACES = ('front', 'back', 'left', 'right')


@dataclass
class RangeSnapshot:
    front: float = math.inf
    back: float = math.inf
    left: float = math.inf
    right: float = math.inf

    def get(self, face):
        return getattr(self, face)

    def as_dict(self):
        return {
            'front': self.front,
            'back': self.back,
            'left': self.left,
            'right': self.right,
        }


@dataclass
class GeometryEvent:
    event: str = NONE
    face: str = ''
    state: str = UNKNOWN
    change: float = 0.0

    @property
    def label(self):
        if self.event == NONE:
            return self.state
        if self.face:
            return f'{self.face.upper()}_{self.event}'
        return self.event


class GeometryAnalyzer:
    """Detect geometry events from multiranger measurements only."""

    def __init__(self, wall_range=1.5, change_floor=0.20):
        self.wall_range = float(wall_range)
        self.change_floor = float(change_floor)
        self.ranges = RangeSnapshot()
        self.previous = {}

    def update(self, face, distance):
        if face not in FACES:
            raise ValueError(f'unknown range face: {face}')

        distance = self.clean_range(distance)
        previous = self.previous.get(face)
        setattr(self.ranges, face, distance)
        self.previous[face] = distance

        event = NONE
        change = 0.0
        if previous is not None:
            change = distance - previous
            if abs(change) >= self.change_floor:
                if self.wall_present(distance) and not self.wall_present(previous):
                    event = WALL_APPEAR
                elif not self.wall_present(distance) and self.wall_present(previous):
                    event = OPENING
                elif change > 0.0:
                    event = WALL_DISAPPEAR
                else:
                    event = WALL_APPEAR

        return GeometryEvent(
            event=event,
            face=face if event != NONE else '',
            state=self.current_state(),
            change=change,
        )

    def snapshot(self):
        return self.ranges

    def current_state(self):
        present = {
            face for face in FACES
            if self.wall_present(self.ranges.get(face))
        }
        if not present:
            return OPEN_SPACE
        if {'front', 'left', 'right'}.issubset(present):
            return DEAD_END
        if {'back', 'left', 'right'}.issubset(present):
            return DEAD_END
        if {'left', 'right'}.issubset(present):
            return CORRIDOR
        if {'front', 'back'}.issubset(present):
            return CORRIDOR
        if len(present) >= 2:
            return CORNER
        return UNKNOWN

    def wall_present(self, distance):
        return (
            distance is not None
            and math.isfinite(distance)
            and 0.0 < distance <= self.wall_range
        )

    @staticmethod
    def clean_range(distance):
        if distance is None:
            return math.inf
        distance = float(distance)
        if not math.isfinite(distance) or distance <= 0.0:
            return math.inf
        return distance
