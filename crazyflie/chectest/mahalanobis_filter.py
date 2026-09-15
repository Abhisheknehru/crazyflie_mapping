#!/usr/bin/env python3
"""Mahalanobis outlier rejection for optical-flow motion increments.

Sits in front of the integrator as a preprocessing seam (like the geometry
constraint): it takes a per-frame increment ``(dx, dy)`` -- the optical-flow
velocity times dt that the pose estimate moved this sample -- and decides
whether it is statistically consistent with recent motion before it is added
to ``x`` and ``y``.

Instead of asking "is this increment large?" (a fixed threshold that also kills
fast-but-real motion), it asks "is this increment unlikely given how the drone
has been moving?":

    d2 = (z - mu)^T  Sigma^-1  (z - mu)

where ``mu`` and ``Sigma`` are the sample mean and 2x2 covariance of the last
``window`` accepted increments and ``z = (dx, dy)``.  ``d2`` is compared to a
chi-square threshold with 2 degrees of freedom:

    5.99 -> 95% kept     9.21 -> 99% kept (default)     13.8 -> 99.9% kept

A spike like vx jumping from 0.02 to 0.28 lands many sigma out and is rejected;
ordinary motion -- even a genuine acceleration the covariance has seen before --
passes.  Rejected samples are NOT folded into the running statistics, so one bad
frame cannot poison the model.

When a sample is rejected the integrator still needs *something* to advance by.
``fallback`` chooses it:

  * 'mean'  -> the recent mean increment (assume the drone kept moving as it
               was for this one dropped frame).  Best for brief dropouts.
  * 'zero'  -> no motion this frame.  Conservative; under-integrates through a
               sustained glitch.
  * 'hold'  -> repeat the last accepted increment.

Two guards keep the test well-posed:

  * ``min_samples``: until the window holds this many points the covariance is
    not trustworthy, so everything is accepted (and learned from).
  * ``var_floor``: a minimum per-axis variance.  When the drone is still the
    increments are near-constant and Sigma collapses toward zero, which would
    flag the first real motion as a huge outlier.  Flooring the variance sets
    the smallest "normal" noise scale and prevents that over-rejection.

Pure-Python (2x2 closed form) to match the other real-time modules; no numpy.
"""

from collections import deque


class MahalanobisFilter:

    def __init__(self, window=40, threshold=9.21, min_samples=10,
                 var_floor=1e-4, fallback='mean'):
        self.window = int(window)
        self.threshold = float(threshold)      # chi-square, 2 DOF
        self.min_samples = int(min_samples)
        self.var_floor = float(var_floor)      # min variance per axis
        if fallback not in ('mean', 'zero', 'hold'):
            raise ValueError(f'unknown fallback {fallback!r}')
        self.fallback = fallback

        self.buf = deque(maxlen=self.window)   # accepted (dx, dy)
        self.last = (0.0, 0.0)                 # last accepted increment
        self.seen = 0                          # total increments offered
        self.rejected = 0                      # total increments rejected

    def _stats(self):
        """Return (mean_x, mean_y, sxx, sxy, syy) over the window."""
        n = len(self.buf)
        mx = sum(p[0] for p in self.buf) / n
        my = sum(p[1] for p in self.buf) / n
        sxx = sxy = syy = 0.0
        for dx, dy in self.buf:
            ex, ey = dx - mx, dy - my
            sxx += ex * ex
            sxy += ex * ey
            syy += ey * ey
        # Sample covariance (n-1); n >= min_samples >= 2 here.
        denom = n - 1
        sxx /= denom
        sxy /= denom
        syy /= denom
        # Floor the on-diagonal variance so a near-still drone (Sigma -> 0)
        # does not make the next real motion look infinitely unlikely.
        sxx = max(sxx, self.var_floor)
        syy = max(syy, self.var_floor)
        return mx, my, sxx, sxy, syy

    def distance2(self, dx, dy):
        """Squared Mahalanobis distance of (dx, dy); None during warm-up."""
        if len(self.buf) < self.min_samples:
            return None
        mx, my, sxx, sxy, syy = self._stats()
        det = sxx * syy - sxy * sxy
        if det <= 0.0:
            # Degenerate even after flooring (perfectly collinear window);
            # fall back to axis-independent (diagonal) distance.
            ex, ey = dx - mx, dy - my
            return ex * ex / sxx + ey * ey / syy
        ex, ey = dx - mx, dy - my
        # (z-mu)^T Sigma^-1 (z-mu) with the 2x2 inverse written out.
        return (syy * ex * ex - 2.0 * sxy * ex * ey + sxx * ey * ey) / det

    def filter(self, dx, dy):
        """Vet one increment.

        Returns ``(dx_out, dy_out, accepted, d2)`` where ``d2`` is the squared
        Mahalanobis distance (None while warming up) and ``accepted`` says
        whether the raw increment passed.
        """
        self.seen += 1
        d2 = self.distance2(dx, dy)

        if d2 is None or d2 <= self.threshold:
            self.buf.append((dx, dy))
            self.last = (dx, dy)
            return dx, dy, True, d2

        # Outlier: reject, do NOT learn from it, substitute the fallback.
        self.rejected += 1
        if self.fallback == 'mean':
            mx = sum(p[0] for p in self.buf) / len(self.buf)
            my = sum(p[1] for p in self.buf) / len(self.buf)
            return mx, my, False, d2
        if self.fallback == 'hold':
            return self.last[0], self.last[1], False, d2
        return 0.0, 0.0, False, d2     # 'zero'
