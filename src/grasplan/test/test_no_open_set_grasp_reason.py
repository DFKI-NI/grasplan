'''When no open-set grasp is left and only the top-down fallback is tried, the warning and the failure text say which
step left none: the grasp generator, the fingertip clearance above the support, or the IK pre-filter (out of the arm's
reach). Sim 2026-11-08: a banana and a tennis ball 1.2 m from the arm base logged "no open-set grasp ... keeps the
fingertips clear of table_1" although 24 of 26 and 42 of 54 grasps kept them clear; IK found none. Stubbed (no ROS
master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import grasplan.pick as pick

PLANNING_FAILED = 'the pick failed: PLANNING_FAILED (-1): no motion plan was found'


class TestReason(unittest.TestCase):
    def test_nothing_offered(self):
        self.assertIn('the grasp generator offered none', pick.no_open_set_grasp_reason('banana_1_1', 'table_1', 0, 0))

    def test_none_clear_of_the_table(self):
        text = pick.no_open_set_grasp_reason('banana_1_1', 'table_1', 26, 0)
        self.assertIn('keeps the fingertips clear of table_1', text)
        self.assertNotIn('IK', text)

    def test_clear_but_out_of_reach(self):
        text = pick.no_open_set_grasp_reason('tennis_ball_1_1', 'table_1', 54, 42)
        self.assertIn('none of the 42 open-set grasps of tennis_ball_1_1 that keep the fingertips clear of table_1', text)
        self.assertIn('IK solution', text)
        self.assertIn("out of the arm's reach", text)


class TestFailureText(unittest.TestCase):
    def tools(self, why, mtc=None):
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc = mtc
        p.pick_failure_reason = ''
        p.topdown_fallback_reason = why
        return p

    def test_moveit_pickup_failure_names_why_only_the_fallback_was_tried(self):
        why = pick.no_open_set_grasp_reason('banana_1_1', 'table_1', 26, 24)
        p = self.tools(why)
        with mock.patch.object(pick.rospy, 'logerr'):
            p.fail(PLANNING_FAILED)
            p.add_fallback_reason()
        self.assertTrue(p.failure_text().startswith(PLANNING_FAILED))
        self.assertIn('only the top-down fallback grasps were tried', p.failure_text())
        self.assertIn("out of the arm's reach", p.failure_text())

    def test_mtc_reason_is_kept(self):
        p = self.tools('none of the 3 ...', mtc=NS(failure_reason='MTC found no plan for grasp topdown_0'))
        with mock.patch.object(pick.rospy, 'logerr'):
            p.add_fallback_reason()
        self.assertTrue(p.failure_text().startswith('MTC found no plan for grasp topdown_0 (only the top-down'))

    def test_no_fallback_no_change(self):
        p = self.tools('')
        with mock.patch.object(pick.rospy, 'logerr'):
            p.fail(PLANNING_FAILED)
            p.add_fallback_reason()
        self.assertEqual(p.failure_text(), PLANNING_FAILED)


if __name__ == '__main__':
    unittest.main()
