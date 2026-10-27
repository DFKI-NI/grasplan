'''The gripper is opened with the configured effort: the Robotiq GripperCommand servers (real and sim) refuse a goal
whose max_effort is outside 30..100, and with the unset 0 the gripper stayed half closed before an open-set pick
(sim 2026-10-27, the fingertips inside the gelatin box at every top grasp). Mocks for the action client (no ROS
master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import grasplan.pick as pick


class FakeGripperServer:
    '''accepts a goal like robotiq_2f_gripper_action_server: max_effort in 30..100, else aborted'''

    def __init__(self):
        self.goals = []
        self.state = None

    def wait_for_server(self, timeout):
        return True

    def send_goal(self, goal):
        self.goals.append(goal)
        self.state = pick.GoalStatus.SUCCEEDED if 30.0 <= goal.command.max_effort <= 100.0 else pick.GoalStatus.ABORTED

    def wait_for_result(self, timeout):
        return True

    def get_state(self):
        return self.state

    def get_goal_status_text(self):
        return '' if self.state == pick.GoalStatus.SUCCEEDED else 'Goal gripper effort out of range'


class TestOpenGripperEffort(unittest.TestCase):
    def make(self, efforts):
        p = pick.PickTools.__new__(pick.PickTools)
        p.gripper_action_client = FakeGripperServer()
        p.gripper_action_name = '/mobipick/gripper_hw'
        p.gripper_action_timeout = 1.0
        p.grasp_planner = NS(gripper_open_distance=[0.14], gripper_joint_efforts=efforts)
        return p

    def test_open_sends_the_configured_effort(self):
        p = self.make([100.0])
        self.assertTrue(p.open_gripper())
        goal = p.gripper_action_client.goals[-1]
        self.assertAlmostEqual(goal.command.position, 0.14)
        self.assertAlmostEqual(goal.command.max_effort, 100.0)

    def test_scalar_effort(self):
        p = self.make(50.0)
        self.assertTrue(p.open_gripper())
        self.assertAlmostEqual(p.gripper_action_client.goals[-1].command.max_effort, 50.0)

    def test_refused_goal_says_why(self):
        p = self.make([0.0])
        with mock.patch.object(pick.rospy, 'logerr') as logerr:
            self.assertFalse(p.open_gripper())
        self.assertIn('Goal gripper effort out of range', logerr.call_args[0][0])


if __name__ == '__main__':
    unittest.main()
