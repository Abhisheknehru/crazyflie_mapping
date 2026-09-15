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
    scope = {'math': math, **extra}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), '<bridge>', 'exec'), scope)
    return scope[name]


def odometry():
    return NS(header=NS(), pose=NS(pose=NS(position=NS(), orientation=NS())))


class ResetTests(unittest.TestCase):
    def test_reset_sequence_and_timeout(self):
        bridge = NS(reset_phase='SELECT_ESTIMATOR', cf=NS(param=Mock()),
                    get_logger=lambda: Mock(), start_logs=Mock())
        advance = method('advance_reset')
        advance(bridge, 0.)
        advance(bridge, .5)
        advance(bridge, .6)
        self.assertEqual(bridge.cf.param.set_value.call_args_list,
                         [unittest.mock.call('stabilizer.estimator', '2'),
                          unittest.mock.call('kalman.resetEstimation', '1'),
                          unittest.mock.call('kalman.resetEstimation', '0')])
        bridge.start_logs.assert_called_once()
        with self.assertRaises(RuntimeError):
            advance(bridge, 31.)

    def test_new_origin_and_heading_on_each_instance(self):
        publish = method('publish_pose', Odometry=odometry, PoseStamped=lambda: NS())
        for initial_x in (5., 20.):
            bridge = NS(origin=None, odom_pub=Mock(), pose_pub=Mock())
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
