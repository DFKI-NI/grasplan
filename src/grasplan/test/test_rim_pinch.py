'''Rim pinch check of the careful flag ~grasp_standing_tilt_band (off in code): after the close a finger gap below 0.7 of
the object's width across the pads is a pinch and fails the grasp before the attach; a closing axis tilted beyond 25 deg
is not checked (the box length along it is no width). Values from a sim table of 31 picks. Fake MTC (no ROS master);
needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import grasplan.mtc_pick_place as mpp
from grasplan.tools.pad_gap import is_pinch, pad_gap, pinch_checkable, theta_for_gap


class TestPinch(unittest.TestCase):
    def test_table_gaps_on_a_75_mm_can(self):
        for gap in (0.061, 0.066, 0.070, 0.072):          # straight grasps: 0.81-0.96 of the width
            self.assertFalse(is_pinch(gap, 0.075), gap)
        for gap in (0.027, 0.035, 0.045, 0.052):          # rim pinches: 0.36-0.71 (0.052 / 0.075 = 0.69)
            self.assertTrue(is_pinch(gap, 0.075), gap)

    def test_no_width_and_a_fully_closed_gripper_are_no_verdict(self):
        self.assertFalse(is_pinch(0.03, None))
        self.assertFalse(is_pinch(0.03, 0.0))
        self.assertFalse(is_pinch(0.003, 0.075))   # closed on nothing is another failure

    def test_a_tilted_closing_axis_is_not_checked(self):
        self.assertTrue(pinch_checkable(None))
        self.assertTrue(pinch_checkable(0.0))
        self.assertTrue(pinch_checkable(25.0))
        self.assertFalse(pinch_checkable(53.0))
        self.assertTrue(pinch_checkable(53.0, 60.0))

    def test_the_ratio_is_a_parameter(self):
        self.assertTrue(is_pinch(0.060, 0.075, 0.9))
        self.assertFalse(is_pinch(0.060, 0.075, 0.7))


def posture(width):
    return JointTrajectory(joint_names=['mobipick/gripper_finger_joint'],
                           points=[JointTrajectoryPoint(positions=[width], effort=[50.0])])


class Goal:
    target_name = 'cic_pringles_1'


class FakeGrasp:
    id = 'g1'
    pre_grasp_posture = posture(0.14)
    pre_grasp_approach = None


class TestPinchCheck(unittest.TestCase):
    def setUp(self):
        self.params = {'~grasp_standing_tilt_band': True}
        patcher = mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        for name in ('announce_trajectory_start',):
            p = mock.patch.object(mpp, name, lambda *a: None)
            p.start()
            self.addCleanup(p.stop)

    def mtc(self, gap, info=None):
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.failure_reason = ''
        m.regrasp_info = {'g1': {'width': 0.075, 'width_open': 0.082, 'width_box': 0.075, 'disagree': False,
                                 'max_depth': 0.03, 'reason': 'cap', 'open_set': True, 'confidence': 'low'}}
        if info:
            m.regrasp_info['g1'].update(info)
        m.finger_angle = lambda: theta_for_gap(gap)
        m.is_preempt_requested = lambda: False
        m.command_gripper = lambda p: MoveItErrorCodes.SUCCESS
        m.closed_on_object = lambda p: True
        m.attach = lambda *a: MoveItErrorCodes.SUCCESS
        m.execute_arm = lambda t: MoveItErrorCodes.SUCCESS
        m.regrasp_after_overclose = mock.Mock(return_value=False)
        sub = NS(trajectory=NS(joint_trajectory=NS(points=[1])))
        m.chosen_solution = mock.Mock()
        m.chosen_solution.toMsg.return_value.sub_trajectory = [sub, sub, sub]
        return m

    def run_execute(self, m):
        return m.execute(None, [mpp.GRIPPER, mpp.ATTACH, mpp.ARM], [], posture(0.0), attach=('can', []),
                         check_grasp=True, regrasp=m.regrasp_request(Goal(), FakeGrasp()))

    def test_a_rim_pinch_fails_the_grasp_before_the_attach_with_the_reason(self):
        m = self.mtc(0.040)
        self.assertEqual(self.run_execute(m), MoveItErrorCodes.FAILURE)
        self.assertIn('pinch (gap ', m.failure_reason)
        self.assertEqual(m.pick_progress['attached'], False)

    def test_a_good_tilted_grasp_is_not_rejected_by_the_box_length_along_its_axis(self):
        # 2026-10-03 sim: tilt 53 deg, finger gap 59 mm on a 66 mm can, width_box 111 mm (ratio 0.53) was a false pinch
        info = {'tilt_deg': 53.0, 'width_box': 0.111, 'width': 0.111}
        m = self.mtc(0.059, info)
        with mock.patch.object(mpp.rospy, 'loginfo') as log:
            self.assertEqual(self.run_execute(m), MoveItErrorCodes.SUCCESS)
        self.assertEqual(m.failure_reason, '')
        self.assertTrue(any('pinch check skipped' in c[0][0] and 'tilted 53 deg' in c[0][0] for c in log.call_args_list))

    def test_a_tilted_grasp_is_checked_again_with_a_higher_limit(self):
        self.params['~grasp_pinch_max_tilt_deg'] = 60.0
        m = self.mtc(0.059, {'tilt_deg': 53.0, 'width_box': 0.111, 'width': 0.111})
        self.assertEqual(self.run_execute(m), MoveItErrorCodes.FAILURE)

    def test_a_straight_close_goes_on_to_attach_and_lift(self):
        m = self.mtc(0.068)
        self.assertEqual(self.run_execute(m), MoveItErrorCodes.SUCCESS)
        self.assertEqual(m.failure_reason, '')
        self.assertEqual(m.pick_progress, {'closed': True, 'attached': True, 'lifted': True})

    def test_the_flag_alone_does_not_turn_on_the_regrasp_rule(self):
        m = self.mtc(0.068)
        self.run_execute(m)
        m.regrasp_after_overclose.assert_not_called()

    def test_with_the_regrasp_flag_too_both_checks_run(self):
        self.params['~grasp_regrasp_on_overclose'] = True
        m = self.mtc(0.068)
        self.run_execute(m)
        m.regrasp_after_overclose.assert_called_once()

    def test_off_in_code_no_request_no_check(self):
        self.params = {}
        m = self.mtc(0.040)
        self.assertIsNone(m.regrasp_request(Goal(), FakeGrasp()))
        self.assertEqual(self.run_execute(m), MoveItErrorCodes.SUCCESS)

    def test_closed_set_and_widthless_grasps_are_not_checked(self):
        for info in ({'open_set': False}, {'width_box': None, 'width': None, 'width_open': 0.082}):
            m = self.mtc(0.040, info)
            self.assertEqual(self.run_execute(m), MoveItErrorCodes.SUCCESS, info)

    def test_the_pad_gap_table_agrees_with_the_inverse(self):
        self.assertAlmostEqual(pad_gap(theta_for_gap(0.040)), 0.040, delta=0.002)


if __name__ == '__main__':
    unittest.main()
