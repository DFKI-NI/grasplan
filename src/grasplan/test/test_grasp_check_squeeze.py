'''#188 real 2026-09-28: the gripper held the bleach bottle twice but both picks failed as "closed on nothing": the fingers
reached the per-object close width (bleach 0.045 m) without stalling on the soft bottle, so the Robotiq reported
"reached the requested position" (gOBJ 3) like a close on air. With ~mtc_grasp_check_squeeze such a grasp closes on to
fully closed and checks again: an object stalls the fingers (held), air lets them close (still a miss). The params are
read at every check, so ~mtc_check_grasp can be switched off without a restart. Fake MTC and gripper (no ROS master);
needs the Mobipick image for the module imports.'''
import unittest
from unittest import mock

from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import grasplan.mtc_pick_place as mpp


def posture(width, effort=50.0):
    p = JointTrajectory(joint_names=['mobipick/gripper_finger_joint'])
    p.points.append(JointTrajectoryPoint(positions=[width], effort=[effort]))
    return p


class FakeGripperFact:
    '''answers of gripper_has_object, one per check'''
    def __init__(self, answers):
        self.answers = list(answers)

    def generate_facts(self):
        return ['gripper_has_object'] if self.answers.pop(0) else []


class TestGraspCheckSqueeze(unittest.TestCase):
    params = {}

    def param(self, name, default=None):   # no master
        return self.params.get(name, default)

    def setUp(self):
        self.params = {'/use_sim_time': True}
        for target, stub in ((mpp.rospy, 'get_param'), (mpp.rospy, 'sleep')):
            patcher = mock.patch.object(target, stub, self.param if stub == 'get_param' else (lambda *a: None))
            patcher.start()
            self.addCleanup(patcher.stop)

    def mtc(self, answers):
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.gripper_fact = FakeGripperFact(answers)
        m.commands = []
        m.command_gripper = lambda p: m.commands.append(p) or MoveItErrorCodes.SUCCESS
        return m

    def test_held_at_the_first_check_does_not_squeeze(self):
        m = self.mtc([True])
        self.assertTrue(m.closed_on_object(posture(0.045)))
        self.assertEqual(m.commands, [])

    def test_preset_width_reached_with_the_object_squeezes_and_holds(self):
        m = self.mtc([False, True])
        self.assertTrue(m.closed_on_object(posture(0.045, effort=40.0)))
        self.assertEqual(len(m.commands), 1)
        self.assertEqual(list(m.commands[0].points[0].positions), [0.0])
        self.assertEqual(list(m.commands[0].points[0].effort), [40.0])

    def test_preset_width_on_air_is_still_a_miss(self):
        m = self.mtc([False, False])
        self.assertFalse(m.closed_on_object(posture(0.045)))
        self.assertEqual(len(m.commands), 1)

    def test_fully_closed_command_does_not_squeeze(self):
        for width in (0.0, 0.0005):   # default gripper_close, klt
            m = self.mtc([False])
            self.assertFalse(m.closed_on_object(posture(width)))
            self.assertEqual(m.commands, [])

    def test_real_robot_default_keeps_the_old_check(self):
        self.params = {'/use_sim_time': False}
        m = self.mtc([False])
        self.assertFalse(m.closed_on_object(posture(0.045)))
        self.assertEqual(m.commands, [])

    def test_squeeze_param_overrides_the_default(self):
        self.params = {'/use_sim_time': False, '~mtc_grasp_check_squeeze': True}
        m = self.mtc([False, True])
        self.assertTrue(m.closed_on_object(posture(0.045)))
        self.params = {'/use_sim_time': True, '~mtc_grasp_check_squeeze': False}
        m = self.mtc([False])
        self.assertFalse(m.closed_on_object(posture(0.045)))
        self.assertEqual(m.commands, [])

    def test_check_switched_off_per_goal(self):
        self.params = {'~mtc_check_grasp': False}
        m = self.mtc([])   # the fact is not asked at all
        self.assertTrue(m.closed_on_object(posture(0.045)))
        self.assertTrue(m.gripper_holds_object('after the lift'))
        self.assertEqual(m.commands, [])

    def test_no_fact_no_check(self):
        m = self.mtc([])
        m.gripper_fact = None
        self.assertTrue(m.closed_on_object(posture(0.045)))


if __name__ == '__main__':
    unittest.main()
