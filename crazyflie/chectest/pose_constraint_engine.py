#!/usr/bin/env python3

"""Rule-based pose constraint engine.

Input is optical-flow predicted motion plus a geometry event. Output is a
corrected motion increment. This module does not read sensors and does not map.
"""

from dataclasses import dataclass

try:
    from chectest.geometry_analyzer import NONE
except ImportError:
    from geometry_analyzer import NONE


FORWARD_X = 'FORWARD_X'
FORWARD_Y = 'FORWARD_Y'
STATIONARY = 'STATIONARY'
AMBIGUOUS = 'AMBIGUOUS'


@dataclass
class ConstraintResult:
    dx: float
    dy: float
    motion_direction: str
    constraint: str = NONE
    applied: bool = False


class PoseConstraintEngine:
    """Simple geometry constraints over optical-flow increments."""

    def __init__(self, alpha=0.0, dominance_margin=0.0):
        self.alpha = float(alpha)
        self.dominance_margin = float(dominance_margin)

    def correct(self, dx, dy, geometry_event):
        direction = self.dominant_motion(dx, dy)
        event = getattr(geometry_event, 'event', NONE)
        face = getattr(geometry_event, 'face', '')

        if event == NONE:
            return ConstraintResult(dx, dy, direction)

        if direction == FORWARD_X and face in ('left', 'right'):
            return ConstraintResult(
                dx=dx,
                dy=self.alpha * dy,
                motion_direction=direction,
                constraint=f'CONSTRAIN_Y_FROM_{face.upper()}_{event}',
                applied=True,
            )

        if direction == FORWARD_Y and face in ('front', 'back'):
            return ConstraintResult(
                dx=self.alpha * dx,
                dy=dy,
                motion_direction=direction,
                constraint=f'CONSTRAIN_X_FROM_{face.upper()}_{event}',
                applied=True,
            )

        return ConstraintResult(dx, dy, direction)

    def dominant_motion(self, dx, dy):
        ax = abs(dx)
        ay = abs(dy)
        if ax <= 0.0 and ay <= 0.0:
            return STATIONARY
        if ax >= ay + self.dominance_margin:
            return FORWARD_X
        if ay >= ax + self.dominance_margin:
            return FORWARD_Y
        return AMBIGUOUS
