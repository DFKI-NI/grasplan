'''re-grasp after over-closing (careful flag ~grasp_regrasp_on_overclose, off in code). After the close the finger
angle is compared with the one flat pads would show on the PERCEIVED width; closed more than the threshold past it, the
gripper opens, moves deeper along the grasp axis by the excess and closes again before the lift. Fake MTC, gripper and
joint state (no ROS master); needs the Mobipick image for the module imports.'''
import math
import unittest
from unittest import mock

from moveit_msgs.msg import GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import grasplan.mtc_pick_place as mpp
from grasplan.tools.pad_gap import (box_extent_along, box_max_x_in_tcp, overclose, pad_gap, pad_tip_x, regrasp_max_depth,
                                    theta_for_gap)


def posture(width):
    p = JointTrajectory(joint_names=['mobipick/gripper_finger_joint'])
    p.points.append(JointTrajectoryPoint(positions=[width], effort=[50.0]))
    return p


class Grasp:
    def __init__(self, gid):
        self.id = gid
        self.pre_grasp_posture = posture(0.14)
        self.pre_grasp_approach = GripperTranslation()


class Goal:
    target_name = 'cic_pringles_1'


class TestPadGap(unittest.TestCase):
    def test_pad_gap_matches_the_urdf_table(self):
        self.assertAlmostEqual(pad_gap(0.0), 0.1395, places=4)
        self.assertAlmostEqual(pad_gap(0.473) * 1000, 55.8, delta=0.7)   # a real close, run 3 off_1
        self.assertAlmostEqual(pad_gap(1.0), pad_gap(0.755))              # clamped to the stroke

    def test_overclose_is_the_width_minus_the_pad_gap(self):
        self.assertAlmostEqual(overclose(0.473, 0.075) * 1000, 19.2, delta=0.7)
        self.assertLess(overclose(0.35, 0.075), 0.0)   # pads (79 mm) still wider than the object

    def test_theta_for_gap_inverts_the_table_at_the_reference_points(self):
        for width, theta in ((0.100, 0.238), (0.075, 0.373), (0.060, 0.452), (0.045, 0.528), (0.035, 0.579),
                             (0.020, 0.654)):
            self.assertAlmostEqual(theta_for_gap(width), theta, delta=0.01)
        self.assertEqual(theta_for_gap(0.2), 0.0)
        self.assertAlmostEqual(pad_tip_x(0.0), 0.0122)

    def test_box_extent_along_an_axis(self):
        identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        self.assertAlmostEqual(box_extent_along([0.1, 0.2, 0.3], identity, [0, 1, 0]), 0.2)
        turned = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]   # box turned 90 deg about z: its x side now lies along y
        self.assertAlmostEqual(box_extent_along([0.1, 0.2, 0.3], turned, [0, 1, 0]), 0.1)
        s = math.sqrt(0.5)
        self.assertAlmostEqual(box_extent_along([0.1, 0.2, 0.3], [[s, -s, 0], [s, s, 0], [0, 0, 1]], [0, 1, 0]), (0.1 + 0.2) * s)


class TestMaxDepth(unittest.TestCase):
    def test_can_centred_at_the_tcp_is_capped(self):
        # a 250 mm tall can on its side axis: the object reaches far past the tips along the approach
        identity = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        beyond = box_max_x_in_tcp([0.25, 0.075, 0.075], identity) - pad_tip_x(theta_for_gap(0.075))
        self.assertEqual(regrasp_max_depth(beyond, 0.03), (0.03, 'cap'))

    def test_banana_end_grasp_has_nothing_beyond_the_tips(self):
        identity = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        beyond = box_max_x_in_tcp([0.03, 0.054, 0.03], identity) - pad_tip_x(theta_for_gap(0.054))   # ends at x +15 mm
        self.assertEqual(regrasp_max_depth(beyond, 0.03), (0.0, 'extent'))

    def test_backed_off_grasp_gets_no_depth(self):
        self.assertEqual(regrasp_max_depth(0.1, 0.03, backed_off=True), (0.0, 'backoff'))

    def test_table_room_caps_a_low_object(self):
        self.assertEqual(regrasp_max_depth(0.1, 0.03, table_room=0.012), (0.012, 'table'))
        self.assertEqual(regrasp_max_depth(0.1, 0.03, table_room=0.003), (0.0, 'table'))

    def test_extent_caps_between(self):
        depth, reason = regrasp_max_depth(0.02, 0.03)
        self.assertEqual((round(depth, 3), reason), (0.02, 'extent'))


class TestRegrasp(unittest.TestCase):
    params = {}

    def param(self, name, default=None):
        return self.params.get(name, default)

    def setUp(self):
        self.params = {'~grasp_regrasp_on_overclose': True}
        for stub in ('get_param', 'sleep'):
            patcher = mock.patch.object(mpp.rospy, stub, self.param if stub == 'get_param' else (lambda *a: None))
            patcher.start()
            self.addCleanup(patcher.stop)

    def mtc(self, angles, deeper=True, info=None):
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.gripper_links = ['mobipick/gripper_base']
        m.regrasp_info = {'g1': {'width': 0.075, 'width_open': 0.078, 'width_box': 0.075, 'disagree': False,
                                 'max_depth': 0.03, 'reason': 'cap', 'open_set': True}}
        if info:
            m.regrasp_info['g1'].update(info)
        m.angles = list(angles)
        m.finger_angle = lambda: m.angles.pop(0)
        m.events = []
        m.command_gripper = lambda p: m.events.append(('gripper', p.points[0].positions[0])) or MoveItErrorCodes.SUCCESS
        m.move_deeper = lambda request, depth: m.events.append(('deeper', round(depth, 4))) or deeper
        return m

    def fire(self, m):
        return m.regrasp_after_overclose(m.regrasp_request(Goal(), Grasp('g1')), posture(0.0))

    def test_off_by_default_in_code(self):
        self.params = {}
        self.assertIsNone(self.mtc([]).regrasp_request(Goal(), Grasp('g1')))

    def test_low_confidence_shape_does_not_fire(self):
        theta = theta_for_gap(0.075 - 0.025)
        m = self.mtc([theta], info={'source': 'box', 'confidence': 'low'})
        self.assertFalse(self.fire(m))
        self.assertEqual(m.events, [])
        m = self.mtc([theta, theta], info={'source': 'box-footprint', 'confidence': 'medium'})
        self.assertTrue(self.fire(m))

    def test_opening_only_request_does_not_fire(self):
        # no box, no pose: the opening is an upper bound with a 10-30 mm margin, the over-close rule would fire on almost
        # every such grasp
        theta = 0.40
        m = self.mtc([theta], info={'source': 'opening', 'confidence': 'low', 'width_box': None})
        self.assertFalse(self.fire(m))
        self.assertEqual(m.events, [])

    def test_log_line_names_source_and_confidence(self):
        with mock.patch.object(mpp.rospy, 'loginfo') as log:
            m = self.mtc([0.40], info={'source': 'box-footprint', 'confidence': 'medium'})
            self.fire(m)
        self.assertIn('src box-footprint conf medium', log.call_args[0][0])

    def test_no_width_no_check(self):
        self.assertIsNone(self.mtc([], info={'width': None, 'width_open': None, 'width_box': None}).regrasp_request(
            Goal(), Grasp('g1')))
        self.assertIsNone(self.mtc([]).regrasp_request(Goal(), Grasp('unknown')))

    def test_flat_close_does_not_fire(self):
        m = self.mtc([0.40])   # pads 70 mm for a 75 mm object
        self.assertFalse(self.fire(m))
        self.assertEqual(m.events, [])

    def test_decision_at_19_20_21_mm(self):
        for excess_mm, fires in ((19.0, False), (20.0, True), (21.0, True)):
            theta = theta_for_gap(0.075 - excess_mm / 1000.0)
            m = self.mtc([theta, theta])
            self.assertEqual(self.fire(m), fires, excess_mm)

    def test_pinch_opens_goes_deeper_by_the_excess_and_closes_again_once(self):
        m = self.mtc([0.50, 0.42])   # 24 mm over; after the regrasp 5 mm short
        self.assertTrue(self.fire(m))
        self.assertEqual([e[0] for e in m.events], ['gripper', 'deeper', 'gripper'])
        self.assertEqual(m.events[0][1], 0.14)                     # opened with the pre grasp posture
        self.assertAlmostEqual(m.events[1][1], 0.0241, places=3)   # the excess
        self.assertEqual(m.events[2][1], 0.0)                      # closed again with the close posture
        self.assertEqual(m.angles, [])                             # exactly two angle reads: no second attempt

    def test_depth_is_the_excess_capped_by_the_grasp_max_depth(self):
        m = self.mtc([0.65, 0.5], info={'max_depth': 0.012, 'reason': 'table'})   # 54 mm over
        self.assertTrue(self.fire(m))
        self.assertEqual(m.events[1], ('deeper', 0.012))

    def test_widths_that_disagree_do_not_fire(self):
        # T2: AnyGrasp opening 94-100 mm for a can the box says is 70 mm: the opening carries a margin
        m = self.mtc([0.50], info={'width': 0.070, 'width_open': 0.100, 'width_box': 0.070, 'disagree': True})
        self.assertFalse(self.fire(m))
        self.assertEqual(m.events, [])

    def test_shallow_by_design_is_skipped(self):
        for reason in ('extent', 'table', 'backoff'):
            m = self.mtc([0.50], info={'max_depth': 0.003, 'reason': reason})
            self.assertFalse(self.fire(m))
            self.assertEqual(m.events, [])

    def test_closed_set_is_measured_only(self):
        m = self.mtc([0.50], info={'open_set': False})
        self.assertFalse(self.fire(m))
        self.assertEqual(m.events, [])

    def test_log_only_measures_both_widths_and_never_moves(self):
        self.params = {'~grasp_regrasp_log_only': True}   # the flag itself is off
        m = self.mtc([0.50], info={'width_extent': 0.081})
        request = m.regrasp_request(Goal(), Grasp('g1'))
        self.assertTrue(request['log_only'])
        with mock.patch.object(mpp.rospy, 'loginfo') as log:
            self.assertFalse(m.regrasp_after_overclose(request, posture(0.0)))
        line = log.call_args[0][0]
        self.assertIn('width_open 78 mm width_box 75 mm', line)
        self.assertIn('excess_open', line)
        self.assertIn('excess_box', line)
        self.assertIn('extent_y 81 mm', line)   # the whole box extent along the finger axis, next to the stall gap
        self.assertIn('excess_extent', line)
        self.assertEqual(m.events, [])

    def test_failed_deeper_move_still_closes_again(self):
        m = self.mtc([0.50, 0.50], deeper=False)
        self.assertTrue(self.fire(m))
        self.assertEqual([e[0] for e in m.events], ['gripper', 'deeper', 'gripper'])

    def test_fully_closed_is_not_a_pinch(self):
        m = self.mtc([0.755])   # nothing between the fingers: handled by the closed-on-nothing check
        self.assertFalse(self.fire(m))

    def test_threshold_param(self):
        self.params['~grasp_regrasp_overclose'] = 0.030
        m = self.mtc([0.50])   # 24 mm over < 30
        self.assertFalse(self.fire(m))

    def test_no_joint_state_does_nothing(self):
        m = self.mtc([None])
        self.assertFalse(self.fire(m))
        self.assertEqual(m.events, [])


class FakeSub:
    pass


class TestRegraspTail(unittest.TestCase):
    """a fired re-grasp replaces the rest of the solution (attach, lift) by a tail planned from the arm's new state"""

    def setUp(self):
        patcher = mock.patch.object(mpp, 'announce_trajectory_start', lambda *a: None)
        patcher.start()
        self.addCleanup(patcher.stop)
        patcher = mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: default)
        patcher.start()
        self.addCleanup(patcher.stop)

    def mtc(self, fires):
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        subs = [FakeSub() for _ in range(3)]
        for sub in subs:
            sub.trajectory = mock.Mock()
            sub.trajectory.joint_trajectory.points = [1]
        m.chosen_solution = mock.Mock()
        m.chosen_solution.toMsg.return_value.sub_trajectory = subs
        m.events = []
        m.is_preempt_requested = lambda: False
        m.command_gripper = lambda p: MoveItErrorCodes.SUCCESS
        m.closed_on_object = lambda p: True
        m.regrasp_after_overclose = lambda request, p: fires
        m.execute_tail = lambda attach: m.events.append('tail') or MoveItErrorCodes.SUCCESS
        m.attach = lambda *a: m.events.append('attach') or MoveItErrorCodes.SUCCESS
        m.execute_arm = lambda t: m.events.append('arm') or MoveItErrorCodes.SUCCESS
        return m

    def run_execute(self, m):
        return m.execute(None, [mpp.GRIPPER, mpp.ATTACH, mpp.ARM], [], posture(0.0), attach=('can', []),
                         check_grasp=True, regrasp={'object': 'can'})

    def test_a_fired_regrasp_replaces_the_attach_and_lift(self):
        m = self.mtc(fires=True)
        self.assertEqual(self.run_execute(m), MoveItErrorCodes.SUCCESS)
        self.assertEqual(m.events, ['tail'])

    def test_no_firing_leaves_the_steps_untouched(self):
        m = self.mtc(fires=False)
        self.assertEqual(self.run_execute(m), MoveItErrorCodes.SUCCESS)
        self.assertEqual(m.events, ['attach', 'arm'])


class TestOpenAfterFailedLift(unittest.TestCase):
    """Sim test T2: the lift after the close failed (-4) with the gripper still closed on the can; the arm then retracted and
    carried the can away. A pick that failed after the close and before the lift opens the gripper where it stands."""

    def setUp(self):
        patcher = mock.patch.object(mpp.rospy, 'logwarn')
        patcher.start()
        self.addCleanup(patcher.stop)

    def mtc(self, progress):
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.pick_progress = progress
        m.events = []
        m.command_gripper = lambda p: m.events.append(('open', p.points[0].positions[0])) or MoveItErrorCodes.SUCCESS
        m.detach = lambda name: m.events.append(('detach', name)) or MoveItErrorCodes.SUCCESS
        return m

    def test_closed_not_lifted_opens_the_gripper_and_detaches_an_attached_object(self):
        m = self.mtc({'closed': True, 'attached': True, 'lifted': False})
        m.open_after_failed_grasp('can', posture(0.14))
        self.assertEqual(m.events, [('detach', 'can'), ('open', 0.14)])

    def test_closed_not_attached_just_opens(self):
        m = self.mtc({'closed': True, 'attached': False, 'lifted': False})
        m.open_after_failed_grasp('can', posture(0.14))
        self.assertEqual(m.events, [('open', 0.14)])

    def test_lifted_object_is_not_dropped(self):
        m = self.mtc({'closed': True, 'attached': True, 'lifted': True})
        m.open_after_failed_grasp('can', posture(0.14))
        self.assertEqual(m.events, [])

    def test_a_failure_before_the_close_changes_nothing(self):
        m = self.mtc({'closed': False, 'attached': False, 'lifted': False})
        m.open_after_failed_grasp('can', posture(0.14))
        self.assertEqual(m.events, [])


class TestPickProgress(unittest.TestCase):
    def setUp(self):
        for target, name, value in ((mpp, 'announce_trajectory_start', lambda *a: None),
                                    (mpp.rospy, 'get_param', lambda name, default=None: default)):
            patcher = mock.patch.object(target, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)

    def mtc(self, arm_code):
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        subs = []
        for _ in range(3):
            sub = FakeSub()
            sub.trajectory = mock.Mock()
            sub.trajectory.joint_trajectory.points = [1]
            subs.append(sub)
        m.chosen_solution = mock.Mock()
        m.chosen_solution.toMsg.return_value.sub_trajectory = subs
        m.is_preempt_requested = lambda: False
        m.command_gripper = lambda p: MoveItErrorCodes.SUCCESS
        m.closed_on_object = lambda p: True
        m.attach = lambda *a: MoveItErrorCodes.SUCCESS
        m.execute_arm = lambda t: arm_code
        return m

    def run_execute(self, m):
        return m.execute(None, [mpp.GRIPPER, mpp.ATTACH, mpp.ARM], [], posture(0.0), attach=('can', []),
                         check_grasp=True)

    def test_a_refused_lift_leaves_closed_and_attached_but_not_lifted(self):
        m = self.mtc(MoveItErrorCodes.CONTROL_FAILED)
        self.assertEqual(self.run_execute(m), MoveItErrorCodes.CONTROL_FAILED)
        self.assertEqual(m.pick_progress, {'closed': True, 'attached': True, 'lifted': False})

    def test_a_good_lift_is_lifted(self):
        m = self.mtc(MoveItErrorCodes.SUCCESS)
        self.assertEqual(self.run_execute(m), MoveItErrorCodes.SUCCESS)
        self.assertEqual(m.pick_progress, {'closed': True, 'attached': True, 'lifted': True})


if __name__ == '__main__':
    unittest.main()
