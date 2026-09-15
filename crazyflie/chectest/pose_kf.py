#!/usr/bin/env python3
"""Constant-velocity Kalman filter fusing flow, stateEstimate, and wall events.

State is ``x = [px, py, vx, vy]`` (world frame, meters / meters-per-second).
This module has no sensor or ROS I/O -- same convention as
``mahalanobis_filter.py`` and ``geometry_analyzer.py`` -- so it can be driven
identically from a live node or from an offline CSV replay.

Three independent measurement sources feed the same filter, each with its own
tunable noise so the fused estimate weighs them rather than picking one:

  1. ``update_position``  -- onboard ``stateEstimate.x/y`` (position).
  2. ``update_velocity``  -- world-frame velocity from :class:`FlowVelocityEstimator`
     (post Mahalanobis gate), i.e. dead-reckoned motion.
  3. ``update_wall_velocity`` -- a *local* constraint: when
     ``geometry_analyzer`` reports a wall event on a face, the velocity
     component perpendicular to that wall is asserted to be near zero. This
     never touches absolute position, so it stays valid with no prior map.

No numpy: matrices are small (4x4 state, 2x2 or 1x1 measurements) and are
inverted in closed form, matching the rest of the real-time chectest modules.
"""

FRONT_BACK = 'front', 'back'
LEFT_RIGHT = 'left', 'right'


class PoseKF:
    """Predict/update Kalman filter over ``[px, py, vx, vy]``."""

    def __init__(self, q_pos=1e-4, q_vel=2e-2,
                 r_pos=(0.02 ** 2, 0.02 ** 2),
                 r_vel=(0.05 ** 2, 0.05 ** 2),
                 r_wall=0.05 ** 2,
                 p0=1.0):
        self.x = [0.0, 0.0, 0.0, 0.0]          # px, py, vx, vy
        self.P = [[p0 if i == j else 0.0 for j in range(4)] for i in range(4)]
        self.q_pos = float(q_pos)              # process noise, position (per second)
        self.q_vel = float(q_vel)              # process noise, velocity (per second)
        self.r_pos = tuple(r_pos)              # stateEstimate measurement noise (x, y)
        self.r_vel = tuple(r_vel)              # flow velocity measurement noise (vx, vy)
        self.r_wall = float(r_wall)            # wall-constraint measurement noise

    # ----- predict: constant-velocity transition -----------------------
    def predict(self, dt):
        if dt <= 0.0:
            return
        px, py, vx, vy = self.x
        self.x = [px + vx * dt, py + vy * dt, vx, vy]

        P = self.P
        # F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]; P' = F P F^T + Q,
        # written out directly since F only couples position with its own
        # velocity column.
        p00, p01, p02, p03 = P[0]
        p10, p11, p12, p13 = P[1]
        p20, p21, p22, p23 = P[2]
        p30, p31, p32, p33 = P[3]

        n00 = p00 + dt * (p02 + p20) + dt * dt * p22
        n01 = p01 + dt * (p03 + p21) + dt * dt * p23
        n02 = p02 + dt * p22
        n03 = p03 + dt * p23
        n11 = p11 + dt * (p13 + p31) + dt * dt * p33
        n12 = p12 + dt * p32
        n13 = p13 + dt * p33
        n22 = p22
        n23 = p23
        n33 = p33

        self.P = [
            [n00, n01, n02, n03],
            [n01, n11, n12, n13],
            [n02, n12, n22, n23],
            [n03, n13, n23, n33],
        ]
        self.P[0][0] += self.q_pos * dt
        self.P[1][1] += self.q_pos * dt
        self.P[2][2] += self.q_vel * dt
        self.P[3][3] += self.q_vel * dt

    # ----- measurement updates -------------------------------------------
    def update_position(self, x_meas, y_meas, r_pos=None):
        self._update_pair(0, 1, x_meas, y_meas, r_pos or self.r_pos)

    def update_velocity(self, vx_meas, vy_meas, r_vel=None):
        self._update_pair(2, 3, vx_meas, vy_meas, r_vel or self.r_vel)

    def update_wall_velocity(self, face, r_wall=None):
        """Assert near-zero velocity into a wall detected on ``face``.

        front/back constrain vx (index 2); left/right constrain vy (index 3).
        Purely local -- no absolute position or map assumption.
        """
        if face in FRONT_BACK:
            self._update_scalar(2, 0.0, r_wall or self.r_wall)
        elif face in LEFT_RIGHT:
            self._update_scalar(3, 0.0, r_wall or self.r_wall)
        else:
            raise ValueError(f'unknown face: {face!r}')

    # ----- shared 2x2 / 1x1 update math -----------------------------------
    def _update_pair(self, i, j, z_i, z_j, r):
        P = self.P
        r_i, r_j = r
        s00 = P[i][i] + r_i
        s01 = P[i][j]
        s11 = P[j][j] + r_j
        det = s00 * s11 - s01 * s01
        if det <= 0.0:
            return  # degenerate covariance; skip rather than divide by ~0

        inv00, inv01, inv11 = s11 / det, -s01 / det, s00 / det
        y_i = z_i - self.x[i]
        y_j = z_j - self.x[j]
        k_i = inv00 * y_i + inv01 * y_j
        k_j = inv01 * y_i + inv11 * y_j

        # K columns for state indices i, j: K[:, 0] = P[:, i]*inv00 + P[:, j]*inv01
        gain_col_i = [P[row][i] * inv00 + P[row][j] * inv01 for row in range(4)]
        gain_col_j = [P[row][i] * inv01 + P[row][j] * inv11 for row in range(4)]

        for row in range(4):
            self.x[row] += gain_col_i[row] * y_i + gain_col_j[row] * y_j

        # P' = (I - K H) P; H selects rows i, j.
        new_P = [row[:] for row in P]
        for row in range(4):
            for col in range(4):
                new_P[row][col] = (
                    P[row][col]
                    - gain_col_i[row] * P[i][col]
                    - gain_col_j[row] * P[j][col]
                )
        self.P = new_P

    def _update_scalar(self, i, z_i, r):
        P = self.P
        s = P[i][i] + r
        if s <= 0.0:
            return
        y = z_i - self.x[i]
        gain = [P[row][i] / s for row in range(4)]

        for row in range(4):
            self.x[row] += gain[row] * y

        new_P = [row[:] for row in P]
        for row in range(4):
            for col in range(4):
                new_P[row][col] = P[row][col] - gain[row] * P[i][col]
        self.P = new_P


# --------------------------------------------------------------------------
# Incremental optical-flow -> world-frame velocity, matching the offline math
# in test_runs/flow_integration.py:estimate_optical_flow_position exactly,
# but stepped one sample at a time so a live node (or replay) can call it
# per-message instead of over a whole recorded array.
# --------------------------------------------------------------------------
import math

FLOW_RESOLUTION = 0.10       # Raw PMW3901 counts are 10x motion pixels.
N_PIXELS = 35.0               # Effective sensor width used by the firmware.
THETA_PIX = 0.71674           # Effective field-of-view ground angle [rad].


class FlowVelocityEstimator:
    """Per-sample port of flow_integration.py's velocity model.

    Call :meth:`step` once per incoming sensor sample; it needs the previous
    sample's attitude to finite-difference angular rates, so the first call
    after construction (or after :meth:`reset`) returns ``None``.
    """

    def __init__(self):
        self._prev_attitude_time = None  # (roll, pitch, yaw, t) in radians/seconds

    def reset(self):
        self._prev_attitude_time = None

    def step(self, t, roll_deg, pitch_deg, yaw_deg,
              flow_delta_x, flow_delta_y, range_down_mm, flow_frame_dt):
        """Return ``(vx_world, vy_world)`` or ``None`` if not yet available.

        ``None`` also comes back when the down-range reading is invalid
        (no return, or out of the sensor's valid span), matching
        flow_integration.py's "hold position" handling.
        """
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        if self._prev_attitude_time is None:
            self._prev_attitude_time = (roll, pitch, yaw, t)
            return None

        prev_roll, prev_pitch, prev_yaw, prev_t = self._prev_attitude_time
        dt = t - prev_t
        self._prev_attitude_time = (roll, pitch, yaw, t)
        if dt <= 0.0:
            return None

        roll_rate = _wrapped_rate_deg(roll_deg, math.degrees(prev_roll), dt)
        pitch_rate = _wrapped_rate_deg(pitch_deg, math.degrees(prev_pitch), dt)
        yaw_rate = _wrapped_rate_deg(yaw_deg, math.degrees(prev_yaw), dt)

        phi, theta = roll, pitch
        omega_x = roll_rate - yaw_rate * math.sin(theta)
        omega_y = (
            pitch_rate * math.cos(phi)
            + yaw_rate * math.sin(phi) * math.cos(theta)
        )

        # Firmware mounting convention:
        # body flow X = -raw sensor deltaY, body flow Y = -raw sensor deltaX.
        body_flow_x = -flow_delta_y * FLOW_RESOLUTION
        body_flow_y = -flow_delta_x * FLOW_RESOLUTION

        distance_m = range_down_mm / 1000.0
        if distance_m <= 0.0 or distance_m >= 8.0:
            return None

        flow_rate_x = body_flow_x * THETA_PIX / (N_PIXELS * flow_frame_dt)
        flow_rate_y = body_flow_y * THETA_PIX / (N_PIXELS * flow_frame_dt)

        velocity_x_body = distance_m * (flow_rate_x + omega_y)
        velocity_y_body = distance_m * (flow_rate_y - omega_x)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_world = cos_yaw * velocity_x_body - sin_yaw * velocity_y_body
        vy_world = sin_yaw * velocity_x_body + cos_yaw * velocity_y_body
        return vx_world, vy_world


def _wrapped_rate_deg(angle_deg, prev_angle_deg, dt):
    """Finite-difference an angle (degrees) handling the +/-180 wrap."""
    difference = angle_deg - prev_angle_deg
    difference = (difference + 180.0) % 360.0 - 180.0
    return math.radians(difference) / dt
