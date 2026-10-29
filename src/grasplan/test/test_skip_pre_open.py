'''The pick's gripper pre-open is skipped when the finger joint already reads open (safe flag
~skip_pre_open_when_open, off in code). Mocks, no ROS master; needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace
from unittest import mock

import grasplan.mtc_pick_place as mpp
import grasplan.pick as pick
from grasplan.tools.pad_gap import reads_open

OPEN, CLOSED = 0.048, 0.755   # the finger joint in the sim before the plan, open and closed


class Client:
    def __init__(self):
        self.goals = []
        self.state = pick.GoalStatus.SUCCEEDED

    def wait_for_server(self, timeout):
        return True

    def send_goal(self, goal):
        self.goals.append(goal)

    def wait_for_result(self, timeout):
        return True

    def get_state(self):
        return self.state

    def get_goal_status_text(self):
        return ''


class TestReadsOpen(unittest.TestCase):
    def test_the_measured_open_and_closed_fingers(self):
        self.assertTrue(reads_open(OPEN, 0.14))
        self.assertTrue(reads_open(0.0, 0.14))
        self.assertFalse(reads_open(CLOSED, 0.14))
        self.assertFalse(reads_open(None, 0.14))   # no reading: open as before

    def test_a_partly_closed_gripper_is_not_open(self):
        self.assertFalse(reads_open(0.15, 0.14))   # ~115 mm: 25 mm short of open
        self.assertTrue(reads_open(0.08, 0.14, tolerance=0.015))   # ~127 mm
        self.assertFalse(reads_open(0.08, 0.14, tolerance=0.005))

    def test_a_smaller_configured_open_width(self):
        self.assertTrue(reads_open(0.25, 0.10))   # 98 mm of a 100 mm open
        self.assertFalse(reads_open(0.25, 0.14))


class TestSkipPreOpen(unittest.TestCase):
    def setUp(self):
        self.p = pick.PickTools.__new__(pick.PickTools)
        self.p.gripper_action_client = Client()
        self.p.gripper_action_timeout = 2.0
        self.p.gripper_action_name = '/mobipick/gripper_hw'
        self.p.grasp_planner = SimpleNamespace(gripper_open_distance=[0.14], gripper_joint_efforts=[100.0])
        self.params = {}
        self.reads = []
        patches = [mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default)),
                   mock.patch.object(mpp, '_current_joint', self.joint),
                   mock.patch.object(pick, 'robot_prefix', lambda: 'mobipick')]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)

    def joint(self, name, timeout=2.0):
        self.reads.append((name, timeout))
        return self.value

    def goals(self):
        return len(self.p.gripper_action_client.goals)

    def test_flag_off_opens_without_reading_the_joint(self):
        self.value = OPEN
        self.assertTrue(self.p.pre_open_gripper())
        self.assertEqual((self.goals(), self.reads), (1, []))

    def test_open_fingers_skip_the_goal(self):
        self.params['~skip_pre_open_when_open'] = True
        self.value = OPEN
        self.assertTrue(self.p.pre_open_gripper())
        self.assertEqual(self.goals(), 0)
        self.assertEqual(self.reads, [('mobipick/gripper_finger_joint', 1.0)])

    def test_closed_fingers_or_no_reading_open_as_before(self):
        self.params['~skip_pre_open_when_open'] = True
        for value in (CLOSED, None, 0.2):
            self.value = value
            self.assertTrue(self.p.pre_open_gripper())
        self.assertEqual(self.goals(), 3)

    def test_the_configured_joint_and_tolerance(self):
        self.params.update({'~skip_pre_open_when_open': True, '~skip_pre_open_finger_joint': 'r2/finger',
                            '~skip_pre_open_tolerance': 0.005})
        self.value = OPEN   # 132 mm: 8 mm short of open, more than 5 mm
        self.p.pre_open_gripper()
        self.assertEqual((self.goals(), self.reads[0][0]), (1, 'r2/finger'))


if __name__ == '__main__':
    unittest.main()
