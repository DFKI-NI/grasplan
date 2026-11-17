'''~mtc_grasp_check_early (safe, off in code): the check after closing goes on at the first True of the gripper fact
once the gripper's action result said stalled, instead of always waiting ~mtc_grasp_check_delay; the delay stays the
cap, a False never comes earlier than without the flag, and the other checks (after the lift, before placing, after
closing fully) keep the full wait. An open-set pick that reads the finger angle next (pinch check, re-grasp after
over-closing) first waits the rest of the delay. Fake clock, fact and gripper (no ROS master); needs the Mobipick image
for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import grasplan.mtc_pick_place as mpp

SUCCESS = MoveItErrorCodes.SUCCESS


def posture(width, effort=50.0):
    p = JointTrajectory(joint_names=['mobipick/gripper_finger_joint'])
    p.points.append(JointTrajectoryPoint(positions=[width], effort=[effort]))
    return p


class Clock:
    '''ROS time of the test: rospy.sleep advances it, rospy.get_time reads it'''
    def __init__(self):
        self.now, self.sleeps = 100.0, []

    def sleep(self, duration):
        self.sleeps.append(duration)
        self.now += duration

    def get_time(self):
        return self.now


class TimedFact:
    '''gripper_has_object turns True at `true_from` s after the clock's start (None: never); counts its readings'''
    def __init__(self, clock, true_from):
        self.clock, self.start, self.true_from, self.reads = clock, clock.now, true_from, 0

    def generate_facts(self):
        self.reads += 1
        held = self.true_from is not None and self.clock.now - self.start >= self.true_from - 1e-9
        return ['gripper_has_object'] if held else []


class EarlyHoldCheck(unittest.TestCase):
    def setUp(self):
        self.params = {'~mtc_grasp_check_early': True}
        self.clock = Clock()
        for name, value in (('get_param', lambda name, default=None: self.params.get(name, default)),
                            ('sleep', self.clock.sleep), ('get_time', self.clock.get_time)):
            patcher = mock.patch.object(mpp.rospy, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)

    def mtc(self, true_from, stalled=True):
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.gripper_fact = TimedFact(self.clock, true_from)
        m.gripper_stalled = stalled
        return m

    def waited(self):
        return self.clock.now - 100.0

    def test_stalled_and_held_goes_on_at_once(self):
        m = self.mtc(0.0)
        self.assertTrue(m.gripper_holds_object('after closing'))
        self.assertLess(self.waited(), 0.2)
        self.assertEqual(m.gripper_fact.reads, 1)

    def test_stalled_and_held_from_0_3_s_goes_on_at_0_3_s(self):
        m = self.mtc(0.3)
        self.assertTrue(m.gripper_holds_object('after closing'))
        self.assertAlmostEqual(self.waited(), 0.3, delta=mpp.GRASP_CHECK_POLL_S + 1e-6)
        self.assertLess(self.waited(), 1.0)
        self.assertTrue(all(s <= mpp.GRASP_CHECK_POLL_S + 1e-9 for s in self.clock.sleeps))

    def test_stalled_and_never_held_is_false_only_at_the_cap(self):
        self.params['~mtc_grasp_check_delay'] = 0.8
        m = self.mtc(None)
        self.assertFalse(m.gripper_holds_object('after closing'))
        self.assertAlmostEqual(self.waited(), 0.8, places=6)   # never earlier, never later than without the flag
        self.assertGreater(m.gripper_fact.reads, 2)

    def test_a_close_that_reached_its_goal_waits_the_full_delay(self):
        m = self.mtc(0.0, stalled=False)
        self.assertTrue(m.gripper_holds_object('after closing'))
        self.assertEqual(self.clock.sleeps, [1.0])
        self.assertEqual(m.gripper_fact.reads, 1)

    def test_flag_off_waits_the_full_delay(self):
        self.params = {}
        m = self.mtc(0.0)
        self.assertTrue(m.gripper_holds_object('after closing'))
        self.assertEqual(self.clock.sleeps, [1.0])
        self.assertIsNone(m.hold_check_waited)

    def test_the_other_checks_keep_the_full_wait(self):
        for when, delay in (('after the lift', None), ('before placing', 0.2), ('after closing fully', None)):
            self.clock.sleeps = []
            m = self.mtc(0.0)
            self.assertTrue(m.gripper_holds_object(when, delay=delay) if delay else m.gripper_holds_object(when))
            self.assertEqual(self.clock.sleeps, [delay or 1.0], when)

    def test_the_check_switched_off_asks_nothing(self):
        self.params['~mtc_check_grasp'] = False
        m = self.mtc(None)
        self.assertTrue(m.gripper_holds_object('after closing'))
        self.assertEqual((m.gripper_fact.reads, self.clock.sleeps), (0, []))

    def test_settle_waits_the_rest_of_the_delay_after_an_early_true(self):
        m = self.mtc(0.3)
        m.gripper_holds_object('after closing')
        m.settle_before_finger_reading()
        self.assertAlmostEqual(self.waited(), 1.0, places=6)
        self.assertIsNone(m.hold_check_waited)
        before = self.clock.now
        m.settle_before_finger_reading()   # once only
        self.assertEqual(self.clock.now, before)

    def test_settle_adds_nothing_after_a_full_wait(self):
        self.params = {}
        m = self.mtc(0.0)
        m.gripper_holds_object('after closing')
        m.settle_before_finger_reading()
        self.assertEqual(self.clock.sleeps, [1.0])


class FakeGripperClient:
    def __init__(self, state, result):
        self.state, self.result = state, result

    def wait_for_server(self, timeout):
        return True

    def send_goal(self, goal):
        pass

    def wait_for_result(self, timeout):
        return True

    def get_result(self):
        return self.result

    def get_state(self):
        return self.state


class GripperStall(unittest.TestCase):
    def setUp(self):
        for name in ('loginfo', 'logwarn'):
            patcher = mock.patch.object(mpp.rospy, name, lambda *a, **k: None)
            patcher.start()
            self.addCleanup(patcher.stop)

    def command(self, state, result):
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.gripper_client, m.gripper_action_timeout = FakeGripperClient(state, result), 5.0
        m.gripper_stalled = True   # left over from an earlier command
        self.assertEqual(m.command_gripper(posture(0.0)), SUCCESS)
        return m.gripper_stalled

    def test_the_last_result_decides(self):
        from actionlib_msgs.msg import GoalStatus
        self.assertTrue(self.command(GoalStatus.ABORTED, NS(stalled=True, position=0.04)))
        self.assertFalse(self.command(GoalStatus.SUCCEEDED, NS(stalled=False, position=0.0)))
        self.assertFalse(self.command(GoalStatus.ABORTED, None))


class ExecuteClose(unittest.TestCase):
    '''the closing step of execute: an open-set pick with the pinch check reads the finger angle only after the full
    delay, a closed-set pick goes on at the early True'''
    def setUp(self):
        self.params = {'~mtc_grasp_check_early': True}
        self.clock = Clock()
        for name, value in (('get_param', lambda name, default=None: self.params.get(name, default)),
                            ('sleep', self.clock.sleep), ('get_time', self.clock.get_time)):
            patcher = mock.patch.object(mpp.rospy, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        for name in ('announce_trajectory_start',):
            patcher = mock.patch.object(mpp, name, lambda *a, **k: None)
            patcher.start()
            self.addCleanup(patcher.stop)

    def run_close(self, regrasp):
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.gripper_fact = TimedFact(self.clock, 0.2)
        none = NS(trajectory=NS(joint_trajectory=NS(points=[])))
        m.chosen_solution = NS(toMsg=lambda: NS(sub_trajectory=[none]))
        m.leaf_stage_names = lambda task: ['close']
        m.is_preempt_requested = lambda: False

        def command_gripper(p):
            m.gripper_stalled = True
            return SUCCESS
        m.command_gripper = command_gripper
        self.read_at = []
        m.pinched = lambda request: self.read_at.append(self.clock.now - 100.0) or False
        code = m.execute(object(), [mpp.GRIPPER], [], posture(0.0), check_grasp=True, regrasp=regrasp)
        self.assertEqual(code, SUCCESS)
        return m

    def test_open_set_pinch_check_reads_after_the_full_delay(self):
        self.run_close({'object': 'can', 'grasp_id': 'g1', 'open_set': True, 'pinch_check': True, 'regrasp': False,
                        'log_only': False})
        self.assertEqual(len(self.read_at), 1)
        self.assertAlmostEqual(self.read_at[0], 1.0, places=6)

    def test_closed_set_goes_on_at_the_early_true(self):
        self.run_close(None)
        self.assertLess(self.clock.now - 100.0, 0.3)
        self.assertEqual(self.read_at, [])


if __name__ == '__main__':
    unittest.main()
