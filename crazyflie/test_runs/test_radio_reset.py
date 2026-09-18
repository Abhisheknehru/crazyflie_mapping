"""Exercise bridge reset and frame math without ROS or radio hardware."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock


def method(name, **extra):
    tree = ast.parse(Path(__file__).with_name('radio_bridge.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    fn = next(n for n in tree.body + cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    helpers = [n for n in tree.body if isinstance(n, ast.FunctionDef)
               and n.name == 'align_planar_pose']
    scope = {'math': math, **extra}
    exec(compile(ast.Module(body=helpers + [fn], type_ignores=[]), '<bridge>', 'exec'), scope)
    return scope[name]


def odometry():
    return NS(header=NS(), pose=NS(pose=NS(position=NS(), orientation=NS())))


class ResetTests(unittest.TestCase):
    def test_reset_sequence_and_timeout(self):
        bridge = NS(reset_phase='SELECT_ESTIMATOR', cf=NS(param=Mock()),
                    get_logger=lambda: Mock(), start_logs=Mock(), settle_timeout=30.)
        advance = method('advance_reset')
        advance(bridge, 0.)
        advance(bridge, .5)
        advance(bridge, .6)
        self.assertEqual(bridge.cf.param.set_value.call_args_list,
                         [unittest.mock.call('stabilizer.estimator', '2'),
                          unittest.mock.call('kalman.resetEstimation', '1'),
                          unittest.mock.call('kalman.resetEstimation', '0')])
        bridge.start_logs.assert_called_once()
        self.assertEqual(bridge.reset_phase, 'SETTLING')
        self.assertAlmostEqual(bridge.reset_deadline, 30.6)

    def test_new_origin_and_heading_on_each_instance(self):
        publish = method('publish_pose', Odometry=odometry, PoseStamped=lambda: NS())
        for initial_x in (5., 20.):
            bridge = NS(origin=None, map_alignment=None, odom_pub=Mock(), pose_pub=Mock())
            data = {'stateEstimate.x': initial_x, 'stateEstimate.y': 3.,
                    'stateEstimate.z': .2, 'stateEstimate.yaw': 90.}
            publish(bridge, data, None)
            pose = bridge.pose_pub.publish.call_args.args[0].pose
            self.assertEqual((pose.position.x, pose.position.y, pose.position.z), (0., 0., 0.))
            self.assertEqual((pose.orientation.z, pose.orientation.w), (0., 1.))
            data['stateEstimate.y'] += 1.
            publish(bridge, data, None)
            pose = bridge.pose_pub.publish.call_args.args[0].pose
            self.assertAlmostEqual(pose.position.x, 1.)
            self.assertAlmostEqual(pose.position.y, 0.)


class RangeTests(unittest.TestCase):
    def test_clamp_bounds_and_conversion(self):
        convert = method('range_metres')
        for raw, expected in ((10, .02), (20, .02), (450, .45),
                              (4000, 4.), (4100, 4.), (8000, 4.)):
            self.assertEqual(convert(raw), expected)

    def test_invalid_is_finite_stop_distance(self):
        convert = method('range_metres')
        for raw in (0, -1, 32767, 65535, math.nan, math.inf, -math.inf, None):
            self.assertEqual(convert(raw), .02)

    def test_configured_limits(self):
        convert = method('range_metres')
        self.assertEqual(convert(3000, .05, 2.), 2.)
        for minimum, maximum in ((0., 4.), (.02, 5.), (1., 1.), (.02, math.nan)):
            with self.assertRaises(ValueError):
                convert(1000, minimum, maximum)


class StartupDiagnosticsTests(unittest.TestCase):
    def bridge(self):
        return NS(reset_phase='SETTLING', logs_started_at=0.,
                  telemetry_timeout=3., last_data={'pose': 29., 'ranges': 29., 'variance': 29.},
                  reset_deadline=30., settle_timeout=30., next_settle_report=0.,
                  settling_report=lambda: 'diagnostics', get_logger=lambda: Mock())

    def test_missing_stream_is_distinguished(self):
        b = self.bridge()
        b.last_data.pop('variance')
        with self.assertRaisesRegex(RuntimeError, 'Missing/stale variance'):
            method('check_startup')(b, 29.)

    def test_unsettled_timeout_with_live_streams(self):
        with self.assertRaisesRegex(RuntimeError, 'did not settle within 30 seconds'):
            method('check_startup')(self.bridge(), 30.)

    def test_ready_does_not_time_out(self):
        b = self.bridge()
        b.reset_phase = 'READY'
        method('check_startup')(b, 40.)

    def test_periodic_report(self):
        b = self.bridge()
        method('check_startup')(b, 29.)
        self.assertEqual(b.next_settle_report, 34.)
