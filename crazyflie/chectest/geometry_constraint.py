#!/usr/bin/env python3
"""Geometry Constraint Module (RIGHT wall only).

Transforms an optical-flow motion increment into a corrected one when a
detected geometry event is consistent with forward motion. It does not estimate
pose or write the occupancy map; the integrator owns the trajectory and the
mapper owns map updates.

Decision rule per increment:

    1. Read optical-flow increment (dx, dy).
    2. Determine dominant motion direction.
    3. Detect a geometry event from the wall reading.
    4. Ask: can this wall change be explained solely by forward motion?
    5. If yes, mark the event as a map-update candidate and constrain only the
       lateral pose component:

           dx_corrected = dx
           dy_corrected = alpha * dy

    6. Otherwise, trust the optical-flow motion normally.

Two ways to decide how long a forward-explained event remains active:

  * window == 0  -> CONTINUOUS: active while that wall remains present.
  * window >  0  -> EDGE-TRIGGERED: active for N samples after the event.

`alpha` (Phase 8) and `window` (edge/Priority 6) are the experiment knobs.
Left / front / back constraints are intentionally NOT implemented yet.
"""

NONE = 'NONE'
RIGHT_WALL_APPEAR = 'RIGHT_WALL_APPEAR'
FORWARD = 'FORWARD'
LATERAL = 'LATERAL'
STATIONARY = 'STATIONARY'
AMBIGUOUS = 'AMBIGUOUS'


class GeometryConstraint:

    def __init__(self, alpha=0.0, window=0, along_x_margin=0.0):
        self.alpha = float(alpha)            # 0 = freeze dy, 1 = leave dy alone
        self.window = int(window)            # 0 = continuous, >0 = edge + N samples
        self.along_x_margin = float(along_x_margin)
        self.timer = 0
        self.applied = 0                     # increments actually corrected
        self.map_updates = 0                 # forward-explained wall events
        self.forward_explained_active = False

    @staticmethod
    def is_along_x(dx, dy, margin=0.0):
        return abs(dx) >= abs(dy) + margin

    @staticmethod
    def dominant_motion(dx, dy, margin=0.0):
        ax = abs(dx)
        ay = abs(dy)
        if ax <= 0.0 and ay <= 0.0:
            return STATIONARY
        if ax >= ay + margin:
            return FORWARD
        if ay >= ax + margin:
            return LATERAL
        return AMBIGUOUS

    def can_explain_wall_change_with_forward_motion(self, dx, dy, reading):
        event = getattr(reading, 'event', NONE)
        if event != RIGHT_WALL_APPEAR:
            return False
        return self.dominant_motion(dx, dy, self.along_x_margin) == FORWARD

    def correct(self, dx, dy, reading):
        """reading: a WallReading (has .event edge and .present state)."""
        event = getattr(reading, 'event', NONE)
        present = getattr(reading, 'present', False)

        if event == RIGHT_WALL_APPEAR:
            self.forward_explained_active = (
                self.can_explain_wall_change_with_forward_motion(dx, dy, reading)
            )
            if self.forward_explained_active:
                self.map_updates += 1
                if self.window > 0:
                    self.timer = self.window     # (re)arm on valid edge only
            else:
                self.timer = 0
        elif not present:
            self.forward_explained_active = False
            self.timer = 0

        if self.window > 0:
            active = self.timer > 0
            if self.timer > 0:
                self.timer -= 1
        else:
            active = present and self.forward_explained_active

        if active and self.is_along_x(dx, dy, self.along_x_margin):
            self.applied += 1
            return dx, self.alpha * dy
        return dx, dy
