'''#151 pick speed: with the cable guard MTC plans one solution and does not replan a rejected one for more (the
replan returned copies of the rejected path); ~mtc_speed_changes false restores the old search for all at once; the
IK pin cost term takes MTC's (SubTrajectory, comment) call and reads the IK state from the solution message.
Fake task and guard (no ROS master, no robot model); needs the Mobipick image for the module imports.'''
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from moveit_task_constructor_msgs.msg import Solution, SubTrajectory

import grasplan.mtc_pick_place as mpp


class FakeSolution:
    def __init__(self, name, cost=1.0):
        self.name, self.cost = name, cost

    def toMsg(self):
        return self.name


class FakeTask:
    def __init__(self, results):
        self.results, self.calls, self.solutions, self.published = results, [], [], None

    def init(self):
        pass

    def plan(self, count):
        self.calls.append(count)
        self.solutions = self.results[min(len(self.calls), len(self.results)) - 1][:count]
        return bool(self.solutions)

    def publish(self, solution):
        self.published = solution


class FakeGuard:
    max_stretch = -0.115

    def __init__(self, passing):
        self.passing, self.worst_q_rad = set(passing), None

    def check(self, name):
        self.worst_q_rad = {'w3': name}
        return (name in self.passing), (-0.2 if name in self.passing else -0.05), 'wrist', {}


def planner(guard, solutions=4):
    m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
    m.cable_guard, m.cable_solutions = guard, solutions
    m.where_in_task = lambda task, guard: ''
    m.describe_failures = lambda task: 'none'
    return m


class TestPlanFirst(unittest.TestCase):
    def setUp(self):
        patcher = mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: default)   # no master
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_without_guard_one_solution(self):
        task = FakeTask([[FakeSolution('a'), FakeSolution('b')]])
        self.assertTrue(planner(None).plan(task, 'grasp g'))
        self.assertEqual(task.calls, [1])

    def test_first_solution_passes_no_second_search(self):
        task = FakeTask([[FakeSolution('a')]])
        m = planner(FakeGuard(['a']))
        self.assertTrue(m.plan(task, 'grasp g'))
        self.assertEqual(task.calls, [1])
        self.assertEqual(task.published.name, 'a')

    def test_rejected_first_solution_is_not_replanned(self):
        task = FakeTask([[FakeSolution('a')], [FakeSolution('a'), FakeSolution('b'), FakeSolution('c')]])
        m = planner(FakeGuard(['b']))
        self.assertFalse(m.plan(task, 'grasp g'))
        self.assertEqual(task.calls, [1])
        self.assertIsNone(m.chosen_solution)
        self.assertIsNone(task.published)
        self.assertEqual(m.guard_rejected_q, {'w3': 'a'})   # the caller turns the grasp from this state

    def test_all_rejected_keeps_the_least_bad_state(self):
        task = FakeTask([[FakeSolution('a'), FakeSolution('b')]])
        with mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: False if 'speed' in name else default):
            m = planner(FakeGuard([]))
            self.assertFalse(m.plan(task, 'grasp g'))
        self.assertEqual(task.calls, [4])
        self.assertIsNotNone(m.guard_rejected_q)

    def test_one_cable_solution_plans_once(self):
        task = FakeTask([[FakeSolution('a')]])
        self.assertFalse(planner(FakeGuard([]), solutions=1).plan(task, 'grasp g'))
        self.assertEqual(task.calls, [1])

    def test_speed_switch_off_restores_the_old_search(self):
        task = FakeTask([[FakeSolution('a'), FakeSolution('b')]])
        with mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: False if 'speed' in name else default):
            self.assertTrue(planner(FakeGuard(['b'])).plan(task, 'grasp g'))
        self.assertEqual(task.calls, [4])

    def test_not_feasible_stops_at_once(self):
        task = FakeTask([[]])
        self.assertFalse(planner(FakeGuard([])).plan(task, 'grasp g'))
        self.assertEqual(task.calls, [1])


ARM = ['mobipick/ur5_shoulder_pan_joint', 'mobipick/ur5_wrist_3_joint']


def ik_message(start, ik):
    msg = Solution()
    msg.start_scene.robot_state.joint_state.name = ARM + ['mobipick/gripper_finger_joint']
    msg.start_scene.robot_state.joint_state.position = list(start) + [0.1]
    sub = SubTrajectory()
    sub.scene_diff.robot_state.joint_state.name = ARM
    sub.scene_diff.robot_state.joint_state.position = list(ik)
    msg.sub_trajectory.append(sub)
    return msg


class FakeSub:
    def __init__(self, msg):
        self.msg, self.failed = msg, None

    def toMsg(self):
        return self.msg

    def markAsFailure(self, comment):
        self.failed = comment


class TestPinCost(unittest.TestCase):
    def setUp(self):
        self.cost = mpp.pinned_ik_cost(dict(zip(ARM, [-1.5, 3.0])), math.radians(20.0))

    def test_called_like_mtc_with_a_comment(self):
        sub = FakeSub(ik_message([0.0, 0.0], [-1.5, 3.05]))
        self.assertAlmostEqual(self.cost(sub, ''), 0.05)
        self.assertIsNone(sub.failed)

    def test_the_ik_state_comes_from_the_scene_diff_not_the_start(self):
        sub = FakeSub(ik_message([-1.5, 3.0], [-1.5, 1.0]))        # start matches, the IK state does not
        self.assertEqual(self.cost(sub, ''), float('inf'))
        self.assertIn('not the ranked IK branch', sub.failed)

    def test_no_diff_falls_back_to_the_start_scene(self):
        msg = ik_message([-1.5, 3.1], [])
        msg.sub_trajectory[0].scene_diff.robot_state.joint_state.name = []
        self.assertAlmostEqual(self.cost(FakeSub(msg), ''), 0.1)

    def test_unreadable_solution_means_no_pin(self):
        sub = NS(toMsg=mock.Mock(side_effect=RuntimeError('no scene')))
        self.assertEqual(self.cost(sub, ''), 0.0)


if __name__ == '__main__':
    unittest.main()
