'''#198 (d) (Oscar 2026-09-28: fewer arm motions): with ~open_set_pick_from_current_state an open-set pick ranks its
grasps from the current arm state and moves to the start pose (transport) only when the first grasp is not predicted to
pass the cable gate or would turn wrist_3 more than ~open_set_pick_from_current_max_wrist_3_deg; when every grasp then
fails before the arm moved, it tries once more from the start pose. Default on in the sim, off on the real robot.
Mocks (no ROS master); needs the Mobipick image for the module imports.'''
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from moveit_msgs.msg import MoveItErrorCodes

import grasplan.pick as pick
from test_open_set_view_and_start import object_pose

W3 = 'mobipick/ur5_wrist_3_joint'


def joints(w3_deg):
    return NS(name=['mobipick/ur5_shoulder_pan_joint', W3], position=[0.0, math.radians(w3_deg)])


class Settle(unittest.TestCase):
    def setUp(self):
        self.params = {}
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.moves, self.ranks = [], []
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc = NS(cable_guard=object(), failure_reason='')
        p.deferred_start_pose, p.stayed_for_pick, p.pick_start_failed = 'transport', '', ''
        p.cable_gate_passing = 1
        p.move_ok = True
        p.move_arm_to_posture = lambda name: self.moves.append(name) or p.move_ok
        p.robot = NS(get_current_state=lambda: NS(joint_state=joints(180.0)))
        p.rank_by_predicted_cable_stretch = lambda g, s, start: self.ranks.append(start) or list(g)
        self.p = p
        self.g1, self.g2 = NS(id='anygrasp_1'), NS(id='anygrasp_2')

    def settle(self, w3_now=150.0, w3_goal=170.0, ranked=None, reachable=None):
        reachable = [self.g1, self.g2] if reachable is None else reachable
        ranked = list(reachable) if ranked is None else ranked
        solutions = [joints(w3_goal) for _ in reachable]
        return self.p.settle_pick_start(ranked, reachable, solutions, [], joints(w3_now))

    def test_stays_when_the_first_grasp_passes_and_turns_little(self):
        self.assertEqual(self.settle(), [self.g1, self.g2])
        self.assertEqual((self.moves, self.p.stayed_for_pick, self.p.deferred_start_pose), ([], 'transport', ''))

    def test_moves_when_nothing_passes_the_cable_gate(self):
        self.p.cable_gate_passing = 0
        self.settle()
        self.assertEqual(self.moves, ['transport'])
        self.assertEqual(len(self.ranks), 1)   # ranked again from the start pose
        self.assertEqual(self.p.stayed_for_pick, '')

    def test_cable_guard_off_only_the_swing_counts(self):
        self.p.mtc = None
        self.p.cable_gate_passing = 0
        self.settle()
        self.assertEqual(self.moves, [])

    def test_moves_for_a_big_wrist_3_swing(self):
        self.settle(w3_now=272.0, w3_goal=133.0)   # real T1: the swing from the upside-down grasp view
        self.assertEqual(self.moves, ['transport'])
        self.params['~open_set_pick_from_current_max_wrist_3_deg'] = 150.0
        self.p.deferred_start_pose, self.moves[:] = 'transport', []
        self.settle(w3_now=272.0, w3_goal=133.0)
        self.assertEqual(self.moves, [])

    def test_moves_when_the_first_grasp_has_only_a_colliding_solution(self):
        self.settle(ranked=[NS(id='anygrasp_9'), self.g1])
        self.assertEqual(self.moves, ['transport'])

    def test_failed_move_stops(self):
        self.p.move_ok = False
        self.assertEqual(self.settle(w3_now=272.0, w3_goal=133.0), [])
        self.assertEqual(self.p.pick_start_failed, 'transport')

    def test_nothing_reachable_leaves_it_to_the_planning_step(self):
        self.assertEqual(self.settle(reachable=[], ranked=[]), [])
        self.assertEqual((self.moves, self.p.deferred_start_pose), ([], 'transport'))


class PickFlow(unittest.TestCase):
    '''pick_object with stubs: the open-set path from the candidates to the planning'''
    def setUp(self):
        self.params = {}
        self.calls = []
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        p = pick.PickTools.__new__(pick.PickTools)
        p.detach_all_objects_flag = False
        p.scene = mock.Mock(get_attached_objects=lambda: {}, get_known_object_names=lambda: [])
        p.perceive_object = False
        p.clear_planning_scene = False
        p.add_custom_boxes_to_ps = lambda boxes: None
        p.planning_scene_boxes = []
        p.make_object_pose_and_add_objs_to_planning_scene = lambda *a, **k: (object_pose(), [0.07, 0.07, 0.11], 1)
        p.mtc = None
        p.obj_pose_pub = mock.Mock()
        p.pregrasp_posture_required = False
        p.pick_action_server = mock.Mock(is_preempt_requested=lambda: False)
        p.move_arm_to_posture = lambda name: self.calls.append(('move', name)) or True
        p.topdown_fallback_grasps = lambda *a: self.calls.append(('topdown', None)) or []
        p.open_gripper = lambda: self.calls.append(('open', None)) or True
        self.grasp = NS(id='anygrasp_1', grasp_quality=0.7)
        p.grasp_planner = mock.Mock(make_grasps_msgs_from_candidates=lambda *a: [self.grasp])
        p.top_grasps_for_low_objects = lambda g, w, pose, box, hint: (g, w, False)
        p.raise_low_grasps = lambda g, surface, w, sink=False: g
        p.robot = mock.Mock()
        p.clear_octomap_flag = False
        p.list_of_disentangle_objects = []
        p.external_grasp_batch_size = 0
        p.cable_gate_passing = 0
        p.results = [MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.PLANNING_FAILED]
        p._pick_with_action = lambda *a: self.calls.append(('plan', None)) or p.results.pop(0)

        def reachable(grasps):
            self.calls.append(('rank', None))
            if p.deferred_start_pose:   # what settle_pick_start does when the current state suits
                p.stayed_for_pick, p.deferred_start_pose = p.deferred_start_pose, ''
            return grasps
        p.reachable_external_grasps = reachable
        self.p = p

    def pick(self):
        return self.p.pick_object('pringles_can_1', 'table_2', 'side_grasp', [],
                                  external_grasp_candidates=[NS(width=0.07)])

    def test_real_robot_default_moves_first(self):
        self.pick()
        self.assertEqual(self.calls[:3], [('move', 'transport'), ('open', None), ('rank', None)])
        self.assertEqual(self.calls.count(('plan', None)), 1)   # no retry without the switch

    def test_sim_default_plans_from_here_then_retries_from_transport(self):
        self.params['/use_sim_time'] = True
        self.assertFalse(self.pick())
        self.assertEqual(self.calls, [('open', None), ('rank', None), ('plan', None), ('move', 'transport'),
                                      ('rank', None), ('plan', None), ('topdown', None)])

    def test_success_from_here_needs_no_transport(self):
        self.params['~open_set_pick_from_current_state'] = True
        self.p.results = [MoveItErrorCodes.SUCCESS]
        self.p.ensure_attached = lambda name: None
        self.p.pose_selector_delete_srv = lambda **k: None
        self.p.clear_mesh_markers = lambda **k: None
        self.p.pick_grasps_marker_array_pub = self.p.pose_selector_objects_marker_array_pub = None
        self.assertTrue(self.pick())
        self.assertNotIn(('move', 'transport'), self.calls)

    def test_retry_switch_off(self):
        self.params.update({'~open_set_pick_from_current_state': True, '~open_set_pick_from_current_state_retry': False})
        self.pick()
        self.assertEqual(self.calls, [('open', None), ('rank', None), ('plan', None), ('topdown', None)])


class SeedAndBoundedStay(PickFlow):
    def test_ik_seed_is_the_start_pose_while_its_move_waits(self):
        p = self.p
        p.robot = NS(arm=NS(get_named_target_values=lambda name: {W3: 3.0}))
        state = NS(joint_state=NS(name=['mobipick/ur5_shoulder_pan_joint', W3], position=[0.5, 1.0]))
        p.deferred_start_pose = 'transport'
        seed = p.ik_seed_state(state)
        self.assertEqual(list(seed.joint_state.position), [0.5, 3.0])
        self.assertEqual(list(state.joint_state.position), [0.5, 1.0])   # the current state is not changed
        p.deferred_start_pose = ''
        self.assertIs(p.ik_seed_state(state), state)

    def test_stay_offers_only_the_predicted_pass_grasps_then_transport_without_new_ik(self):
        self.params['/use_sim_time'] = True
        p = self.p
        g1, g2 = NS(id='anygrasp_1', grasp_quality=0.9), NS(id='anygrasp_2', grasp_quality=0.8)
        p.grasp_planner = mock.Mock(make_grasps_msgs_from_candidates=lambda *a: [g1, g2])
        offered = []
        p._pick_with_action = lambda obj, grasps, surface: offered.append([g.id for g in grasps]) or p.results.pop(0)
        p.robot = NS(get_current_state=lambda: NS(joint_state=joints(180.0)),
                     arm=NS(get_end_effector_link=lambda: 'mobipick/gripper_tcp'))
        p.external_grasp_max_attempts = 0

        def reachable(grasps):
            self.calls.append(('rank', None))
            p.ik_prefilter = ([g1, g2], [joints(170.0), joints(170.0)], [])
            p.stayed_for_pick, p.deferred_start_pose = p.deferred_start_pose, ''
            p.cable_gate_passing = 1
            return [g1, g2]
        p.reachable_external_grasps = reachable
        ranks = []
        def rank(r, s, start):   # as from transport: nothing predicted to pass, no gate
            ranks.append(start)
            p.cable_gate_passing = 0
            return list(r)
        p.rank_by_predicted_cable_stretch = rank
        self.assertFalse(self.pick())
        self.assertEqual(offered, [['anygrasp_1'], ['anygrasp_1', 'anygrasp_2']])
        self.assertEqual(self.calls.count(('rank', None)), 1)            # no second IK pre-filter
        self.assertEqual(len(ranks), 1)                                     # ranked again from the start pose
        self.assertIn(('move', 'transport'), self.calls)


if __name__ == '__main__':
    unittest.main()
