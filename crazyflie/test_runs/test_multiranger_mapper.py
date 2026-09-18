"""Exercise the per-face trust hysteresis without a live rclpy node."""
import unittest

from test_runs.multiranger_mapper import MultiRangerMapper


def node(trust_near=.3, trust_far=.4):
    class Node:
        pass
    n = Node()
    n.trust_near = trust_near
    n.trust_far = trust_far
    n.face_trusted = {face: False
                      for face in ('front', 'left', 'back', 'right')}
    return n


class TrustHysteresisTests(unittest.TestCase):
    def test_enters_trusted_below_near(self):
        n = node()
        self.assertFalse(MultiRangerMapper.update_trust(n, 'front', .5))
        self.assertTrue(MultiRangerMapper.update_trust(n, 'front', .2))

    def test_stays_trusted_in_hysteresis_band(self):
        n = node()
        MultiRangerMapper.update_trust(n, 'front', .2)
        self.assertTrue(MultiRangerMapper.update_trust(n, 'front', .35))

    def test_exits_only_above_far(self):
        n = node()
        MultiRangerMapper.update_trust(n, 'front', .2)
        self.assertTrue(MultiRangerMapper.update_trust(n, 'front', .4))
        self.assertFalse(MultiRangerMapper.update_trust(n, 'front', .41))

    def test_faces_are_independent(self):
        n = node()
        MultiRangerMapper.update_trust(n, 'front', .2)
        self.assertFalse(MultiRangerMapper.update_trust(n, 'back', .5))
        self.assertTrue(MultiRangerMapper.update_trust(n, 'front', .35))
