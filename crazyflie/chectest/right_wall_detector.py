#!/usr/bin/env python3
"""Online right-wall detector -- adaptive, magnitude-independent, edge-aware.

The wall change has NO fixed size, so detection is RELATIVE: a wall is present
while the right range sits below its recent open-space reference by more than
`change_floor` (just above sensor jitter); any drop magnitude past that floor
counts. The reference is tracked over a short window while no wall is present
and frozen while one is.

update(right_m) returns a WallReading with:
    event   -- RIGHT_WALL_APPEAR on the rising EDGE only (one sample), else NONE
    present -- True for as long as the wall is present (the STATE)
    change  -- current drop below the open reference (m); basis for confidence

Exposing both the edge and the state lets the constraint module choose how to
use them (fire-once + window, or continuous) without changing the detector.
Non-finite / non-positive readings are treated as "no wall in range".
"""
import math
from collections import deque, namedtuple

try:                                            # installed as a ROS package
    from chectest.geometry_constraint import RIGHT_WALL_APPEAR, NONE
except ImportError:                             # run as a plain script
    from geometry_constraint import RIGHT_WALL_APPEAR, NONE


WallReading = namedtuple('WallReading', ['event', 'present', 'change'])


class RightWallDetector:

    def __init__(self, change_floor=0.20, window=8, max_range=4.0):
        # change_floor: smallest sustained drop (m) counted as real motion.
        # window:       samples used to set the open-space reference.
        # max_range:    a no-return reading means "open space this far away",
        #               so an open->near transition registers as a real drop.
        self.change_floor = float(change_floor)
        self.max_range = float(max_range)
        self.win = deque(maxlen=int(window))
        self.present = False
        self.present_prev = False
        self.ref = None        # open-space reference distance (m)

    @staticmethod
    def _valid(r):
        return r is not None and math.isfinite(r) and r > 0.0

    def update(self, right):
        # No return -> treat as open space at max range (NOT a reset), so a
        # wall coming in from "nothing on the right" is seen as an appearance.
        if not self._valid(right):
            right = self.max_range

        self.win.append(right)
        change = 0.0

        if not self.present:
            self.ref = max(self.win)
            change = self.ref - right
            if change > self.change_floor:
                self.present = True            # any drop past the floor = appear
        else:
            change = (self.ref - right) if self.ref is not None else 0.0
            if self.ref is not None and right >= self.ref - self.change_floor:
                self.present = False           # climbed back -> wall gone
                self.ref = right
                self.win.clear()
                self.win.append(right)

        edge = (RIGHT_WALL_APPEAR
                if self.present and not self.present_prev else NONE)
        self.present_prev = self.present
        return WallReading(edge, self.present, max(change, 0.0))
