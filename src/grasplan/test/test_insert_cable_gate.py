'''#219: insert pose gate ranks cable-safe IK solutions and delays held-back MTC offers.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
from moveit_msgs.msg import MoveItErrorCodes

import grasplan.insert as insert

W3 = 'mobipick/ur5_wrist_3_joint'


class InsertCableGate(unittest.TestCase):
    def setUp(self):
        self.params = {}
        patcher = mock.patch.object(insert.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.tool = insert.InsertTools.__new__(insert.InsertTools)
        self.tool.tcp_frame = 'mobipick/gripper_tcp'
        self.tool.insert_action_server = NS(is_preempt_requested=lambda: False)
        self.tool.action_client_helper = object()

    def prepare_ik(self):
        start = NS(joint_state=NS(name=[W3], position=[2.0]))
        model = NS(evaluate=lambda q: NS(max_stretch=-0.2 if q[W3] <= 2.0 else -0.05))
        guard = NS(model=model, max_stretch=-0.115)
        mtc = NS(cable_guard=guard, executed=False, update_cable_guard=mock.Mock())
        self.tool.place = NS(robot=NS(get_current_state=lambda: start), mtc=mtc, group_name='arm',
                             global_reference_frame='map')
        ik = mock.Mock()
        ik.wait_for_service = mock.Mock()
        ik.side_effect = [
            NS(error_code=NS(val=MoveItErrorCodes.SUCCESS), solution=NS(joint_state=NS(name=[W3], position=[w3])))
            for w3 in (2.1, 1.8)
        ]
        self.tool.compute_ik_srv = ik
        bodies = [np.eye(4), np.eye(4)]
        bodies[0][0, 3] = 1
        bodies[1][0, 3] = 2
        return bodies, mtc

    def test_gate_promotes_predicted_pass_over_shorter_wrist_turn(self):
        self.params['~insert_cable_gate'] = True
        bodies, mtc = self.prepare_ik()
        ranked = self.tool.sort_by_wrist_3_change(bodies, np.eye(4))
        self.assertEqual([int(body[0, 3]) for body in ranked], [2, 1])
        self.assertEqual(self.tool.insert_gate_passing, 1)
        mtc.update_cable_guard.assert_called_once()

    def test_code_default_off_preserves_wrist_order(self):
        bodies, mtc = self.prepare_ik()
        ranked = self.tool.sort_by_wrist_3_change(bodies, np.eye(4))
        self.assertEqual([int(body[0, 3]) for body in ranked], [1, 2])
        self.assertEqual(self.tool.insert_gate_passing, 0)
        mtc.update_cable_guard.assert_not_called()

    def test_held_back_group_runs_only_after_predicted_passes_fail(self):
        calls = []
        codes = [MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.SUCCESS]
        def plan(goal, *_args):
            calls.append(list(goal.place_locations))
            return NS(error_code=NS(val=codes.pop(0)))
        self.tool.place = NS(mtc=NS(executed=False), run_place_goal=plan)
        self.tool.insert_gate_passing = 1
        result = self.tool.run_ranked_insert_goal(NS(place_locations=['safe', 'held1', 'held2']))
        self.assertEqual(result.error_code.val, MoveItErrorCodes.SUCCESS)
        self.assertEqual(calls, [['safe'], ['held1', 'held2']])

    def test_successful_first_group_never_offers_held_back_poses(self):
        calls = []
        def plan(goal, *_args):
            calls.append(list(goal.place_locations))
            return NS(error_code=NS(val=MoveItErrorCodes.SUCCESS))
        self.tool.place = NS(mtc=NS(executed=False), run_place_goal=plan)
        self.tool.insert_gate_passing = 1
        self.tool.run_ranked_insert_goal(NS(place_locations=['safe', 'held']))
        self.assertEqual(calls, [['safe']])


if __name__ == '__main__':
    unittest.main()
