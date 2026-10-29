'''pick.py picks the IK branch with the most cable slack (~mtc_cable_path_branches under ~mtc_cable_path_shaping):
compute_ik seeded near each UR branch, the branches ranked by the cable grid. Fake compute_ik and grid; no ROS master
(Mobipick image for the module imports).'''
import copy
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIKRequest
from sensor_msgs.msg import JointState

import grasplan.mtc_pick_place as mpp
from grasplan.tools import cable_path as cp

PREFIX = 'mobipick/'
J = cp.UR_JOINTS
NAMES = [PREFIX + f'ur5_{j}_joint' for j in J]
SLACK_W2 = math.radians(-90.0)
FIRST = [0.0, -1.5, 1.5, -1.5, math.radians(90.0), 2.0]     # wrist_2 +90: stretched in the fake grid
OTHER = [0.0, -1.5, 1.5, -1.5, SLACK_W2, 2.0]                # wrist_2 -90: slack


def grid():
    '''stretch depends on wrist_2 only: slack below 0, stretched above'''
    a = np.radians(np.arange(-180, 181, 10.0))
    return cp.CableGrid([cp.SpanGrid('wrist', ['wrist_2'], [a], np.where(a < 0, -0.3, 0.1))])


class FakeIK:
    '''returns OTHER when seeded with a negative wrist_2, else FIRST'''
    def __init__(self):
        self.calls = 0

    def __call__(self, request):
        self.calls += 1
        js = request.ik_request.robot_state.joint_state
        w2 = dict(zip(js.name, js.position))[PREFIX + 'ur5_wrist_2_joint']
        solution = JointState(name=NAMES, position=list(OTHER if w2 < 0 else FIRST))
        return NS(error_code=NS(val=MoveItErrorCodes.SUCCESS), solution=NS(joint_state=solution))


class TestLeastStretchBranch(unittest.TestCase):
    def setUp(self):
        self.params = {'~mtc_cable_path_shaping': True}
        patcher = mock.patch.object(mpp.rospy, 'get_param', lambda n, d=None: self.params.get(n, d))
        patcher.start()
        self.addCleanup(patcher.stop)
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.update_cable_guard = lambda: None
        m.cable_shaper = NS(ready=lambda: grid(), limits={j: (-math.pi, math.pi) for j in J})
        m.cable_guard = NS(max_stretch=-0.115)
        self.p = m
        self.request = GetPositionIKRequest()
        self.request.ik_request.robot_state.joint_state = JointState(name=NAMES, position=list(FIRST))
        self.start = JointState(name=NAMES, position=[0.0, -1.5, 1.5, -1.5, math.radians(-60.0), 2.0])

    def choose(self):
        ik = FakeIK()
        chosen = self.p.least_stretch_branch(ik, copy.deepcopy(self.request), JointState(name=NAMES, position=list(FIRST)),
                                             self.start, 'g1')
        return dict(zip(chosen.name, chosen.position)), ik

    def test_the_slack_branch_replaces_the_first(self):
        chosen, ik = self.choose()
        self.assertAlmostEqual(chosen[PREFIX + 'ur5_wrist_2_joint'], SLACK_W2)
        self.assertEqual(ik.calls, 7)   # one call per other branch seed

    def test_off_keeps_compute_iks_branch(self):
        self.params['~mtc_cable_path_branches'] = False
        chosen, ik = self.choose()
        self.assertAlmostEqual(chosen[PREFIX + 'ur5_wrist_2_joint'], FIRST[4])
        self.assertEqual(ik.calls, 0)

    def test_ranked_branches_are_kept_for_the_fall_through(self):
        self.choose()
        branches = self.p.grasp_ik_branches['g1']
        self.assertEqual(len(branches), 2)
        self.assertAlmostEqual(dict(zip(branches[1].name, branches[1].position))[PREFIX + 'ur5_wrist_2_joint'], FIRST[4])

    def test_closed_set_grasp_gets_a_pinned_branch(self):
        self.p.arm_group, self.p.eef_link = 'arm', PREFIX + 'gripper_tcp'
        ik = FakeIK()
        ik.wait_for_service = lambda t: None
        with mock.patch.object(mpp.rospy, 'wait_for_message', lambda *a, **k: self.start), \
                mock.patch.object(mpp.rospy, 'ServiceProxy', lambda *a, **k: ik):
            self.p.ensure_shaping_ik(NS(id='handcoded_1', grasp_pose='pose'))
        pinned = dict(zip(self.p.grasp_ik_states['handcoded_1'].name, self.p.grasp_ik_states['handcoded_1'].position))
        self.assertAlmostEqual(pinned[PREFIX + 'ur5_wrist_2_joint'], SLACK_W2)

    def test_pinned_grasp_is_left_alone(self):
        self.p.grasp_ik_states = {'g1': 'pinned by the pre-filter'}
        self.p.ensure_shaping_ik(NS(id='g1', grasp_pose='pose'))
        self.assertEqual(self.p.grasp_ik_states['g1'], 'pinned by the pre-filter')

    def test_no_grid_keeps_compute_iks_branch(self):
        self.p.cable_shaper = NS(ready=lambda: None, limits={})
        chosen, ik = self.choose()
        self.assertAlmostEqual(chosen[PREFIX + 'ur5_wrist_2_joint'], FIRST[4])


if __name__ == '__main__':
    unittest.main()
