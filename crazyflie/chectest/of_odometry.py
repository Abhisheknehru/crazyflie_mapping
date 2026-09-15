#!/usr/bin/env python3
"""Optical-flow prediction plus multiranger geometry constraints.

This node owns localization. It predicts motion from optical flow, recognizes
range-derived geometry events, applies simple rule-based pose constraints, then
publishes the corrected pose for the mapper.
"""
import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range

from chectest.geometry_constraint import GeometryConstraint, NONE
from chectest.mahalanobis_filter import MahalanobisFilter
from chectest.right_wall_detector import RightWallDetector, WallReading


def yaw_from_quat(x, y, z, w):
    """Yaw (rad) from a full ZYX quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def roll_pitch_from_quat(x, y, z, w):
    """Roll and pitch (rad) from a full ZYX quaternion."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


def quat_from_euler(roll, pitch, yaw):
    """Right-handed ZYX roll/pitch/yaw as (x, y, z, w)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class OFOdometry(Node):

    def __init__(self):
        super().__init__('of_odometry')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('record_path', '')
        self.declare_parameter('frame', 'map')
        # Phase 4: toggle the right-wall constraint. False reproduces the
        # Phase 1 baseline. Either way the recorded raw increments are the
        # same (this node only observes /odom; it never commands the drone).
        self.declare_parameter('enable_constraint', True)
        self.declare_parameter('range_front_topic', '/crazyflie/range_front')
        self.declare_parameter('range_back_topic', '/crazyflie/range_back')
        self.declare_parameter('range_left_topic', '/crazyflie/range_left')
        self.declare_parameter('range_right_topic', '/crazyflie/range_right')
        # Smallest range change (m) that counts as geometry, not jitter.
        # The change magnitude itself is unconstrained.
        self.declare_parameter('change_floor', 0.20)
        # Samples used for the detector's open-space reference, and the range a
        # no-return reading stands in for.
        self.declare_parameter('detector_window', 8)
        self.declare_parameter('detector_max_range', 4.0)
        # Constraint mechanism knobs:
        #   alpha: 0.0 = full axis constraint, 1.0 = no correction.
        #   constraint_window: 0 = hold while the wall is present, N = N samples.
        self.declare_parameter('alpha', 0.4)
        self.declare_parameter('dominance_margin', 0.0)
        self.declare_parameter('constraint_window', 0)

        # Mahalanobis outlier rejection on the raw optical-flow increment,
        # applied BEFORE the geometry constraint and the integrator. Rejects
        # statistically improbable (dx, dy) spikes instead of using a fixed
        # magnitude threshold. Disable to reproduce the Phase 1 baseline.
        self.declare_parameter('enable_mahalanobis', True)
        self.declare_parameter('maha_window', 40)       # samples of history
        self.declare_parameter('maha_threshold', 5.99)  # chi-square, 2 DOF, 99%
        self.declare_parameter('maha_min_samples', 10)  # warm-up before testing
        self.declare_parameter('maha_var_floor', 1e-4)  # min variance per axis
        self.declare_parameter('maha_fallback', 'mean')  # mean | zero | hold

        self.frame = self.get_parameter('frame').value
        record_path = self.get_parameter('record_path').value
        self.enable_constraint = bool(
            self.get_parameter('enable_constraint').value
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/of/pose', 10)
        self.path_pub = self.create_publisher(Path, '/of/path', 10)
        self.path = Path()
        self.path.header.frame_id = self.frame

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.on_odom, 10,
        )

        # Geometry analyzer recognizes events only; constraint engine corrects
        # pose only. The mapper remains a downstream consumer of /of/pose.
        self.detector = RightWallDetector(
            change_floor=float(self.get_parameter('change_floor').value),
            window=int(self.get_parameter('detector_window').value),
            max_range=float(self.get_parameter('detector_max_range').value),
        )
        self.constraint = GeometryConstraint(
            alpha=float(self.get_parameter('alpha').value),
            window=int(self.get_parameter('constraint_window').value),
            along_x_margin=float(
                self.get_parameter('dominance_margin').value
            ),
        )
        self.enable_mahalanobis = bool(
            self.get_parameter('enable_mahalanobis').value
        )
        self.maha = MahalanobisFilter(
            window=int(self.get_parameter('maha_window').value),
            threshold=float(self.get_parameter('maha_threshold').value),
            min_samples=int(self.get_parameter('maha_min_samples').value),
            var_floor=float(self.get_parameter('maha_var_floor').value),
            fallback=str(self.get_parameter('maha_fallback').value),
        )
        self.ranges = {face: float('nan') for face in
                       ('front', 'back', 'left', 'right')}
        self.reading = WallReading(NONE, False, 0.0)
        self.emit_event = NONE
        self.current_motion_direction = 'STATIONARY'
        self.current_constraint = NONE
        self.current_constraint_applied = False
        for face in ('front', 'back', 'left', 'right'):
            self.create_subscription(
                Range,
                self.get_parameter(f'range_{face}_topic').value,
                self.make_range_callback(face),
                10,
            )

        # Integrator state -- the estimated pose, accumulated from increments.
        self.tilt = (0.0, 0.0)
        self.x_pred = self.y_pred = self.yaw_pred = 0.0
        self.x = self.y = self.yaw = 0.0
        self.prev = None      # previous raw (x, y, yaw)
        self.origin = None    # first raw pose -> defines the zero
        self.samples = 0

        self.csv = None
        if record_path:
            parent = os.path.dirname(record_path)
            if parent:
                os.makedirs(parent, exist_ok=True)   # create runs/ if missing
            self.csv = open(record_path, 'w', newline='')
            self.csv.write(
                't,raw_x,raw_y,raw_yaw,'
                'pred_x,pred_y,pred_yaw,'
                'corr_x,corr_y,corr_yaw,'
                'front,back,left,right,'
                'dx_raw,dy_raw,dx_corr,dy_corr,'
                'geometry_event,geometry_state,applied_constraint,'
                'motion_direction,constraint_applied\n'
            )
            self.get_logger().info(f'Recording trajectory to {record_path}')

        self.get_logger().info(
            'OF odometry up. Geometry constraint '
            f'{"ENABLED" if self.enable_constraint else "DISABLED (baseline)"}.'
        )

    def make_range_callback(self, face):
        def callback(msg):
            self.ranges[face] = float(msg.range)
            if face != 'right':
                return
            reading = self.detector.update(float(msg.range))
            # Latch the rising edge: the detector reports it for one sample, but
            # the increment that should consume it arrives on the odom topic.
            if reading.event == NONE and self.reading.event != NONE:
                reading = reading._replace(event=self.reading.event)
            self.reading = reading
            if reading.event != NONE:
                self.get_logger().info(
                    f'GEOMETRY EVENT: {reading.event} '
                    f'(present={reading.present}, change={reading.change:+.3f} m)'
                )
        return callback

    # ----- Mahalanobis outlier-rejection seam (runs before the constraint) --
    def reject_outliers(self, dx, dy):
        if not self.enable_mahalanobis:
            return dx, dy
        dxf, dyf, accepted, d2 = self.maha.filter(dx, dy)
        if not accepted:
            self.get_logger().warn(
                f'Outlier increment rejected: dx={dx:.4f} dy={dy:.4f} '
                f'(d2={d2:.1f} > {self.maha.threshold:.1f}); '
                f'using dx={dxf:.4f} dy={dyf:.4f}'
            )
        return dxf, dyf

    # ----- Geometry Constraint Module seam (Phase 4: right wall only) -----
    def apply_constraint(self, dx, dy):
        margin = self.constraint.along_x_margin
        self.current_motion_direction = self.constraint.dominant_motion(dx, dy, margin)
        self.emit_event = self.reading.event
        if not self.enable_constraint:
            self.reading = self.reading._replace(event=NONE)
            return dx, dy
        before = self.constraint.applied
        dxc, dyc = self.constraint.correct(dx, dy, self.reading)
        self.current_constraint_applied = self.constraint.applied > before
        self.current_constraint = (self.reading.event if self.reading.event != NONE
                                   else self.current_constraint)
        # The edge is consumed; the wall's presence state persists on its own.
        self.reading = self.reading._replace(event=NONE)
        return dxc, dyc

    def on_odom(self, msg):
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        ryaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        # Tilt is not corrected here, only carried through, so the mapper can
        # reject readings taken while the ranger beams were pointing off-plane.
        self.tilt = roll_pitch_from_quat(q.x, q.y, q.z, q.w)

        if self.origin is None:
            self.origin = (rx, ry, ryaw)
            self.prev = (rx, ry, ryaw)
            self.get_logger().info(
                f'Origin set: x={rx:.3f} y={ry:.3f} '
                f'yaw={math.degrees(ryaw):.1f} deg'
            )
            self._emit(msg.header.stamp, rx, ry, ryaw, 0.0, 0.0, 0.0, 0.0)
            return

        # World-frame motion increment from the fused optical-flow pose.
        dx = rx - self.prev[0]
        dy = ry - self.prev[1]
        dyaw = ryaw - self.prev[2]
        self.prev = (rx, ry, ryaw)

        # Vet the raw increment for spikes, then apply the geometry constraint.
        # dx/dy below stay the unmodified raw increments for offline replay.
        dxf, dyf = self.reject_outliers(dx, dy)
        self.x_pred += dxf
        self.y_pred += dyf
        self.yaw_pred += dyaw
        dxc, dyc = self.apply_constraint(dxf, dyf)
        self.x += dxc
        self.y += dyc
        self.yaw += dyaw

        self._emit(msg.header.stamp, rx, ry, ryaw, dx, dy, dxc, dyc)

    def _emit(self, stamp, rx, ry, ryaw, dx_raw, dy_raw, dx_corr, dy_corr):
        self.samples += 1
        ox, oy, oyaw = self.origin
        raw_x = rx - ox
        raw_y = ry - oy
        raw_yaw = ryaw - oyaw

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = quat_from_euler(
            self.tilt[0], self.tilt[1], self.yaw)
        self.pose_pub.publish(pose)

        self.get_logger().info(
            f'Normal pose: x={raw_x:.3f} y={raw_y:.3f} '
            f'Corrected pose: x={self.x:.3f} y={self.y:.3f}'
        )

        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

        if self.csv:
            t = stamp.sec + stamp.nanosec * 1e-9
            ranges = self.ranges
            # Raw pose is logged relative to the origin so it lines up with the
            # estimate. dx_raw/dy_raw are the unmodified increments, so the run
            # can be replayed offline with the constraint off vs on.
            self.csv.write(
                f'{t:.4f},{raw_x:.4f},{raw_y:.4f},{raw_yaw:.4f},'
                f'{self.x_pred:.4f},{self.y_pred:.4f},{self.yaw_pred:.4f},'
                f'{self.x:.4f},{self.y:.4f},{self.yaw:.4f},'
                f'{ranges["front"]:.4f},{ranges["back"]:.4f},'
                f'{ranges["left"]:.4f},{ranges["right"]:.4f},'
                f'{dx_raw:.4f},{dy_raw:.4f},{dx_corr:.4f},{dy_corr:.4f},'
                f'{self.emit_event},'
                f'{"RIGHT_WALL" if self.reading.present else "NONE"},'
                f'{self.current_constraint},'
                f'{self.current_motion_direction},'
                f'{int(self.current_constraint_applied)}\n'
            )
            self.csv.flush()
            self.emit_event = NONE
            self.current_constraint = NONE
            self.current_constraint_applied = False

    def destroy_node(self):
        if self.enable_mahalanobis and self.maha.seen:
            pct = 100.0 * self.maha.rejected / self.maha.seen
            self.get_logger().info(
                f'Mahalanobis: rejected {self.maha.rejected}/{self.maha.seen} '
                f'increments ({pct:.1f}%).'
            )
        if self.csv:
            self.csv.close()
            self.get_logger().info(f'Saved {self.samples} trajectory samples.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OFOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
