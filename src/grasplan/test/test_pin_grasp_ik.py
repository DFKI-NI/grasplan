'''#151: MTC's grasp-pose IK is pinned to the IK solution pick.py's cable ranking judged (mtc.grasp_ik_states).
Stubbed MTC stages / fake solutions (no robot model, no master); needs the Mobipick image for the imports.'''
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from moveit_task_constructor_msgs.msg import Solution
from moveit_task_constructor_msgs.msg import SubTrajectory as MtcSubTrajectory
from sensor_msgs.msg import JointState

import grasplan.mtc_pick_place as mpp
from test_pick_contact_order import Stage, build

ARM = [f'mobipick/ur5_{j}_joint' for j in ('shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3')]
RANKED = [-1.5, -1.3, 1.6, -1.0, 1.4, 4.2]


class FakeSub:
    """an MTC ComputeIK solution: its message carries the IK state in the scene diff (sub.end.scene has no Python
    conversion in MTC 0.1.3)"""
    def __init__(self, positions, names=ARM):
        self.msg = Solution()
        self.msg.start_scene.robot_state.joint_state.name = list(names) + ['mobipick/gripper_finger_joint']
        self.msg.start_scene.robot_state.joint_state.position = [0.0] * (len(names) + 1)
        diff = MtcSubTrajectory()
        diff.scene_diff.robot_state.joint_state.name = list(names)
        diff.scene_diff.robot_state.joint_state.position = list(positions)
        self.msg.sub_trajectory.append(diff)
        self.failed = None

    def toMsg(self):
        return self.msg

    def markAsFailure(self, comment):
        self.failed = comment


class TestPinnedIkCost(unittest.TestCase):
    def setUp(self):
        self.cost = mpp.pinned_ik_cost(dict(zip(ARM, RANKED)), math.radians(20.0))

    def test_the_ranked_branch_is_cheapest(self):
        near, far = FakeSub([v + 0.05 for v in RANKED]), FakeSub([v + 0.2 for v in RANKED])
        self.assertLess(self.cost(near, ''), self.cost(far, ''))
        self.assertIsNone(near.failed)

    def test_another_branch_fails(self):
        other = FakeSub(RANKED[:4] + [RANKED[4] - 1.0, RANKED[5]])   # wrist_2 57 deg off (night19b's branch)
        self.assertEqual(self.cost(other, ''), float('inf'))
        self.assertIn('not the ranked IK branch', other.failed)

    def test_no_solution_message_means_no_pin(self):
        self.assertEqual(self.cost(NS(toMsg=None), ''), 0.0)


class TestMakePickTaskPin(unittest.TestCase):
    def test_cost_term_only_with_a_stored_state(self):
        costs = []
        Stage.setCostTerm = lambda self, fn: costs.append(fn)
        try:
            state = JointState(name=ARM, position=RANKED)
            with mock.patch.object(mpp.MtcPickPlace, 'grasp_ik_states', {'g': state}, create=True):
                build()
            self.assertEqual(len(costs), 1)
            with mock.patch.object(mpp.MtcPickPlace, 'grasp_ik_states', {'other': state}, create=True):
                build()
            self.assertEqual(len(costs), 1)                 # grasp 'g' has no stored state: plain ComputeIK
            with mock.patch.object(mpp.MtcPickPlace, 'grasp_ik_states', {'g': state}, create=True), \
                    mock.patch.object(mpp, 'speed_changes_on', lambda: False):
                build()
            self.assertEqual(len(costs), 1)                 # ~mtc_speed_changes false: no pin
        finally:
            del Stage.setCostTerm


if __name__ == '__main__':
    unittest.main()
