'''With the support wrist anchor the MTC cable guard's planning margin is
~mtc_cable_support_max_stretch (-0.07) instead of ~mtc_cable_max_stretch, behind the careful flag
~mtc_cable_support_margin (off in code); the tip anchor keeps its margin. No ROS master: rospy.get_param is patched.'''
import unittest
from types import SimpleNamespace
from unittest import mock

import grasplan.mtc_pick_place as mpp


def guard(anchor):
    g = object.__new__(mpp.CableGuard)
    g.cfg = SimpleNamespace(wrist_anchor=anchor)
    return g


class TestSupportMargin(unittest.TestCase):
    def margin(self, anchor, params):
        g = guard(anchor)
        with mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: params.get(name, default)):
            g.update_params()
        return g.max_stretch

    def test_off_by_default_the_old_margin(self):
        self.assertEqual(self.margin('support', {}), mpp.DEFAULT_CABLE_MAX_STRETCH)

    def test_on_with_the_support_anchor(self):
        self.assertEqual(self.margin('support', {'~mtc_cable_support_margin': True}), -0.07)
        self.assertEqual(self.margin('support', {'~mtc_cable_support_margin': True,
                                                 '~mtc_cable_support_max_stretch': -0.08}), -0.08)

    def test_the_tip_anchor_keeps_its_margin(self):
        for anchor in ('carrier_tip', 'tube_tip'):
            self.assertEqual(self.margin(anchor, {'~mtc_cable_support_margin': True}), mpp.DEFAULT_CABLE_MAX_STRETCH)

    def test_an_explicit_margin_is_kept_when_off(self):
        self.assertEqual(self.margin('support', {'~mtc_cable_max_stretch': -0.1}), -0.1)


if __name__ == '__main__':
    unittest.main()
