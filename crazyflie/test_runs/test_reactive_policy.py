import math
import unittest
from test_runs.reactive_policy import ReactivePolicy


def ranges(front=2., left=2., back=2., right=.45):
    return dict(front=front, left=left, back=back, right=right)


class PolicyTests(unittest.TestCase):
    def test_corridor(self):
        self.assertEqual(ReactivePolicy().step(ranges()), (.1, 0., 'FOLLOW'))

    def test_invalid_and_close_stop(self):
        for value in (math.nan, math.inf, -math.inf, 0., -.1, .2):
            self.assertEqual(ReactivePolicy().step(ranges(front=value))[:2], (0., 0.))
        self.assertEqual(ReactivePolicy().step({})[:2], (0., 0.))

    def test_dead_end_reverses(self):
        p = ReactivePolicy()
        self.assertEqual(p.step(ranges(front=.4, left=.4, right=.4))[2], 'CHANGE_DIRECTION')
        self.assertEqual(p.direction, 2)
        self.assertLess(p.step(ranges(front=.4, left=.45, right=.4))[0], 0)

    def test_right_opening_requires_observed_wall(self):
        p = ReactivePolicy()
        p.step(ranges(right=2.))
        self.assertEqual(p.direction, 0)
        p.step(ranges())
        self.assertEqual(p.step(ranges(right=2.))[2], 'RIGHT_OPENING')
        self.assertEqual(p.direction, 3)
        self.assertLess(p.step(ranges())[1], 0)

    def test_lateral_correction_does_not_enter_close_wall(self):
        vx, vy, _ = ReactivePolicy().step(ranges(left=.4, right=.35))
        self.assertGreater(vx, 0)
        self.assertEqual(vy, 0)

    def test_invalid_configuration(self):
        for kwargs in ({'speed': -1}, {'speed': math.nan}, {'opening': .2}):
            with self.assertRaises(ValueError):
                ReactivePolicy(**kwargs)


if __name__ == '__main__':
    unittest.main()
