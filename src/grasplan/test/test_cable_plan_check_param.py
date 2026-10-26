'''The MTC cable guard is switched by ~mtc_cable_plan_check; the old name ~mtc_cable_constraints still works when the
new one is not set. No ROS master: rospy.get_param is patched, the guard's model is a stub.'''
import unittest
from unittest import mock

import grasplan.mtc_pick_place as mpp


class StubGuard:
    def update_params(self):
        pass


def guard_after_goal(params):
    planner = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
    planner.cable_guard = None
    planner._cable_guard_cache = StubGuard()
    planner.eef_link = 'mobipick/gripper_tcp'
    with mock.patch.object(mpp.rospy, 'get_param', side_effect=lambda name, default=None: params.get(name, default)):
        planner.update_cable_guard()
    return planner.cable_guard


class TestCablePlanCheckParam(unittest.TestCase):
    def test_new_name_switches_the_guard(self):
        self.assertIsNotNone(guard_after_goal({'~mtc_cable_plan_check': True}))
        self.assertIsNone(guard_after_goal({'~mtc_cable_plan_check': False}))

    def test_old_name_still_works_without_the_new_one(self):
        self.assertIsNotNone(guard_after_goal({'~mtc_cable_constraints': True}))
        self.assertIsNone(guard_after_goal({}))

    def test_new_name_wins_over_the_old_one(self):
        self.assertIsNone(guard_after_goal({'~mtc_cable_plan_check': False, '~mtc_cable_constraints': True}))


if __name__ == '__main__':
    unittest.main()
