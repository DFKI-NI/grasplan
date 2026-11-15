'''~extra_grasp_sets (default off, careful): a second, optional grasp list of a class
(~handcoded_grasp_planner_extra_transforms) is offered only after the whole pick with the stored grasps failed before the
arm moved, through the same pipeline, with ids after the stored ones. The stored list never changes with the flag.
Extras never make a class known. The soup extras of mobipick_pick_n_place
(the radial ring turned 90 deg about the approach axis) close along the can axis. Mocks for rosparam (no ROS master);
needs the Mobipick image for the module imports.'''
import os
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import yaml

import grasplan.grasp_planner.handcoded_grasp_planner as hgp
from geometry_msgs.msg import PoseStamped
from grasplan.tools.topdown_grasps import quaternion_to_matrix

STORED = {'soup': {'grasp_poses': [
    {'translation': [0.0, 0.0, 0.0069], 'rotation': [0.5, 0.5, -0.5, 0.5]},
    {'translation': [0.0, 0.0283, 0.0], 'rotation': [0.5, -0.5, 0.5, -0.5]}]}}
EXTRA = {'soup': {'grasp_poses': [{'translation': [0.0, 0.0, 0.0069], 'rotation': [0.7071068, 0.0, -0.7071068, 0.0]}]},
         'pear': {'grasp_poses': [{'translation': [0.0, 0.0, 0.0], 'rotation': [0.0, 0.0, 0.0, 1.0]}]}}


def soup_files():
    '''the stored and the extra soup grasps of mobipick_pick_n_place, None when the package is not found'''
    try:
        import rospkg
        base = os.path.join(rospkg.RosPack().get_path('mobipick_pick_n_place'), 'config', 'grasplan', 'object_grasps')
    except Exception:
        return None
    paths = [os.path.join(base, f'handcoded_grasp_planner_soup{s}.yaml') for s in ('', '_extra')]
    return paths if all(os.path.exists(p) for p in paths) else None


class TestExtraGraspSets(unittest.TestCase):
    def planner(self, on, extra=EXTRA):
        params = {'~handcoded_grasp_planner_transforms': STORED, '~handcoded_grasp_planner_extra_transforms': extra,
                  '~extra_grasp_sets': on}
        patcher = mock.patch.object(hgp.rospy, 'get_param', lambda name, default=None: params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.params = params
        with mock.patch.object(hgp.rospy, 'sleep'), mock.patch.object(hgp.rospy, 'loginfo'):
            p = hgp.HandcodedGraspPlanner(call_parent_constructor=False)

        def make_grasps_msgs(name, pose, eef, grasp_type):   # the core's wrapper: one grasp per generated pose
            return [NS(id=f'grasp_{i}', pose=q) for i, q in enumerate(self.poses(p, name))]

        p.make_grasps_msgs = make_grasps_msgs
        return p

    def poses(self, planner, name='soup_1'):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.w = 1.0
        with mock.patch.object(hgp.rospy, 'loginfo'), mock.patch.object(hgp.rospy.Time, 'now', return_value=0):
            return planner.gen_end_effector_grasp_poses(name, pose, 'side_grasp').poses

    def test_the_stored_list_does_not_change_with_the_flag(self):
        for on in (False, True):
            p = self.planner(on)
            self.assertEqual(p.transforms_of('soup'), STORED['soup']['grasp_poses'])
            self.assertEqual(len(self.poses(p)), 2)

    def test_on_the_extras_come_separately_with_ids_after_the_stored_ones(self):
        p = self.planner(True)
        extra = p.extra_grasps_msgs('soup_1', None, 'tcp', 'side_grasp')
        self.assertEqual([g.id for g in extra], ['grasp_2'])
        self.assertEqual(len(self.poses(p)), 2)   # the switch back to the stored list after the call

    def test_off_there_are_no_extras(self):
        self.assertEqual(self.planner(False).extra_grasps_msgs('soup_1', None, 'tcp', 'side_grasp'), [])

    def test_the_flag_is_read_on_every_call(self):
        p = self.planner(False)
        self.params['~extra_grasp_sets'] = True
        self.assertEqual(len(p.extra_grasps_msgs('soup_1', None, 'tcp', 'side_grasp')), 1)

    def test_extras_never_make_a_class_known(self):
        p = self.planner(True)
        self.assertFalse(p.supports_object('pear_1'))
        self.assertEqual(p.extra_grasps_msgs('pear_1', None, 'tcp', 'side_grasp'), [])
        self.assertTrue(p.supports_object('soup_1'))

    def test_no_extras_for_the_class_or_none_loaded(self):
        for extra in ({}, {'soup': {}}):
            planner = self.planner(True, extra=extra)
            with mock.patch.object(hgp.rospy, 'logwarn') as warn:
                self.assertEqual(planner.extra_grasps_msgs('soup_1', None, 'tcp', 'side_grasp'), [])
            # the flag is on but nothing is loaded for the class: said, not silent (the launch file loads the extras)
            self.assertEqual(warn.call_count, 1)
            self.assertIn('no extra grasps are loaded for soup', warn.call_args[0][0])

    def test_flag_off_or_unknown_class_says_nothing(self):
        for on, name in ((False, 'soup_1'), (True, 'unknown_1')):
            planner = self.planner(on, extra={})   # each planner patches the params: make it right before its call
            with mock.patch.object(hgp.rospy, 'logwarn') as warn:
                self.assertEqual(planner.extra_grasps_msgs(name, None, 'tcp', 'side_grasp'), [])
            self.assertFalse(warn.called)


class TestPickOffersExtrasLast(unittest.TestCase):
    '''the pick offers the extras only after the whole stored pipeline failed before the arm moved'''

    def node(self, executed=False, cancel=False):
        import grasplan.pick as pick
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc = NS(executed=executed, pick_stage_executed=executed)
        p.detoured = ''
        p.pick_action_server = NS(is_preempt_requested=lambda: cancel)
        p.robot = NS(arm=NS(get_end_effector_link=lambda: 'tcp'))
        p.grasp_planner = NS(extra_grasps_msgs=lambda *a: [NS(id='grasp_32'), NS(id='grasp_33')])
        p.note_regrasp_info = lambda *a: None
        p.offered = []
        p.offer_grasps = lambda obj, grasps, support, external: p.offered.append(('old way', [g.id for g in grasps])) or 1
        p.offer_single_plan = lambda obj, grasps, support, single: p.offered.append(('single', [g.id for g in grasps])) or 1
        self.pick = pick
        return p

    def offer(self, p, result, single=None):
        from grasplan.tools.common import objectToPick
        with mock.patch.object(self.pick.rospy, 'logwarn'):
            return p.offer_extra_grasps(objectToPick('soup_1'), None, None, 'table_2', 'side_grasp', single, result)

    def test_after_a_planning_failure_the_extras_go_through_the_same_pipeline(self):
        p = self.node()
        failed = self.pick.MoveItErrorCodes.PLANNING_FAILED
        self.assertEqual(self.offer(p, failed), 1)
        self.assertEqual(p.offered, [('old way', ['grasp_32', 'grasp_33'])])
        p = self.node()
        self.offer(p, failed, single=(['untangle'], 'transport'))
        self.assertEqual(p.offered, [('single', ['grasp_32', 'grasp_33'])])

    def test_no_extras_after_a_success_an_executed_or_cancelled_pick_or_no_server(self):
        p = self.node()
        codes = self.pick.MoveItErrorCodes
        for result in (codes.SUCCESS, None):
            self.assertEqual(self.offer(p, result), result)
        p = self.node(executed=True)
        self.assertEqual(self.offer(p, codes.PLANNING_FAILED), codes.PLANNING_FAILED)
        p2 = self.node(cancel=True)
        self.offer(p2, codes.PLANNING_FAILED)
        self.assertEqual(p.offered + p2.offered, [])

    def test_a_planner_without_extras_changes_nothing(self):
        p = self.node()
        p.grasp_planner = NS(extra_grasps_msgs=lambda *a: [])
        codes = self.pick.MoveItErrorCodes
        self.assertEqual(self.offer(p, codes.PLANNING_FAILED), codes.PLANNING_FAILED)
        self.assertEqual(p.offered, [])

    def test_the_pick_calls_it_only_for_a_closed_set_pick_after_its_pipeline(self):
        import inspect
        import grasplan.pick as pick
        source = inspect.getsource(pick.PickTools.pick_object)
        at = source.index('self.offer_extra_grasps(')
        self.assertIn('if external_grasp_candidates is None:', source[at - 120:at])
        self.assertLess(source.index('result = self.offer_single_plan(object_to_pick, grasps'), at)
        self.assertLess(source.index('topdown_fallback_grasps(object_to_pick, object_pose, bounding_box'), at)


class TestThroughTheSinglePlan(unittest.TestCase):
    '''the real offer_single_plan and offer_extra_grasps with a scripted MTC backend: extras after pre-pick
    repositioning are allowed, after any executed pick stage of the goal they are not'''

    def node(self, script):
        import grasplan.pick as pick
        self.pick = pick
        codes = pick.MoveItErrorCodes
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc = NS(executed=False, pick_stage_executed=False, single_plan=None, untangle_leg_refused='',
                   untangle_failure='', end_move_done=False, end_move_failure='', failure_reason='')
        p.detoured = ''
        p.pick_action_server = NS(is_preempt_requested=lambda: False)
        p.robot = NS(arm=NS(get_end_effector_link=lambda: 'tcp'))
        p.grasp_planner = NS(extra_grasps_msgs=lambda *a: [NS(id='grasp_32')])
        p.note_regrasp_info = lambda *a: None
        p.events = []
        p.move_arm_to_posture = lambda pose: p.events.append(('untangle', pose)) or True
        script = list(script)

        def offer_grasps(obj, grasps, support, external):
            # one scripted pickup: (executed, pick stage executed, untangle failure, result)
            executed, stage, untangle, result = script.pop(0)
            p.mtc.executed = executed   # a pickup resets executed at its start, the latch it never resets
            p.mtc.pick_stage_executed = p.mtc.pick_stage_executed or stage
            p.mtc.untangle_failure = untangle
            p.events.append(('single' if p.mtc.single_plan else 'old way', [g.id for g in grasps]))
            return getattr(codes, result)

        p.offer_grasps = offer_grasps
        return p

    def run_pick(self, p):
        from grasplan.tools.common import objectToPick
        single = (['untangle_1'], 'transport')
        stored = [NS(id='grasp_0')]
        with mock.patch.object(self.pick.rospy, 'logwarn'), mock.patch.object(self.pick.rospy, 'loginfo'):
            result = p.offer_single_plan(objectToPick('soup_1'), stored, 'table_2', single)
            return p.offer_extra_grasps(objectToPick('soup_1'), None, None, 'table_2', 'side_grasp', single, result)

    def test_untangle_move_then_no_plan_offers_the_extras(self):
        p = self.node([(False, False, '', 'PLANNING_FAILED'), (False, False, '', 'PLANNING_FAILED'),
                       (False, False, '', 'SUCCESS')])
        self.assertEqual(self.run_pick(p), self.pick.MoveItErrorCodes.SUCCESS)
        self.assertEqual(p.events, [('single', ['grasp_0']), ('untangle', 'untangle_1'), ('old way', ['grasp_0']),
                                    ('single', ['grasp_32'])])

    def test_a_failed_untangle_leg_of_the_single_plan_follows_its_pre_open_so_no_extras(self):
        # the single plan sends the gripper pre-open before its untangle legs (MtcPickPlace.execute): the latch is set
        # when a leg fails, so the old way's stored grasps run (from the untangle poses) and no extras after them
        p = self.node([(True, True, 'the untangle leg to untangle_1 failed', 'CONTROL_FAILED'),
                       (False, False, '', 'PLANNING_FAILED')])
        self.run_pick(p)
        self.assertEqual(p.events, [('single', ['grasp_0']), ('untangle', 'untangle_1'), ('old way', ['grasp_0'])])

    def test_an_executed_approach_that_fails_offers_no_extras(self):
        p = self.node([(True, True, '', 'CONTROL_FAILED')])
        self.assertEqual(self.run_pick(p), self.pick.MoveItErrorCodes.CONTROL_FAILED)
        self.assertEqual(p.events, [('single', ['grasp_0'])])

    def test_a_pick_stage_of_the_goal_rules_the_extras_out_also_after_a_later_pickup_resets_executed(self):
        # the single plan ran into its pick stages and failed there as an untangle failure would not; then the old
        # way could plan nothing: executed is False again, the latch still holds
        p = self.node([(True, True, 'the untangle leg to untangle_1 failed', 'CONTROL_FAILED'),
                       (False, False, '', 'PLANNING_FAILED')])
        self.run_pick(p)
        self.assertNotIn(('single', ['grasp_32']), p.events)
        self.assertNotIn(('old way', ['grasp_32']), p.events)

    def test_the_mtc_executor_sets_the_latch_for_pick_motions_and_every_gripper_command(self):
        import inspect
        import grasplan.mtc_pick_place as mpp
        source = inspect.getsource(mpp.MtcPickPlace.execute)
        at = source.index('self.pick_stage_executed = True')
        self.assertIn('if role != UNTANGLE:', source[at - 300:at])
        self.assertLess(at, source.index('code = self.execute_arm(sub.trajectory)'))
        gripper = source.index('elif role == GRIPPER:')
        latch = source.index('self.pick_stage_executed = True', gripper)
        self.assertLess(latch, source.index('code = self.command_gripper(posture)', gripper))
        self.assertNotIn('if check_grasp and not postures:\n                    self.pick_stage_executed', source)
        self.assertNotIn('pick_stage_executed = False', inspect.getsource(mpp.MtcPickPlace.pickup))


class TestRealExecutorGripperFailure(unittest.TestCase):
    '''the real pick pipeline (offer_grasps / offer_single_plan -> _pick_with_mtc -> MtcPickPlace.pickup -> execute) with
    a planned stored grasp whose first executed step, the gripper pre-open, fails: no gripper server (-4, CONTROL_FAILED)
    or a timeout (-6, TIMED_OUT). The latch is set, so no extra grasp is offered; only the planning, the gripper and the
    arm controller are scripted.'''

    def node(self, gripper_codes, arm_codes=(), roles=None):
        import grasplan.mtc_pick_place as mpp
        import grasplan.pick as pick
        self.mpp, self.pick, codes = mpp, pick, pick.MoveItErrorCodes
        roles = roles or [mpp.ARM, mpp.GRIPPER, mpp.ATTACH, mpp.ARM]
        self.commands = []
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.pick_stage_executed, m.start_move, m.single_plan = False, None, None
        m.untangle_leg_refused, m.untangle_failure, m.end_move_done, m.end_move_failure = '', '', False, ''
        m.gripper_links, m.cable_guard = ['finger'], None
        m.is_preempt_requested = lambda: False
        m.update_cable_guard = lambda: None
        m.make_pick_task = lambda goal, grasp: ('task', list(m.single_plan and [mpp.UNTANGLE] or []) + roles)
        m.plan = lambda task, what: True
        points = NS(trajectory=NS(joint_trajectory=NS(points=[1])))
        m.chosen_solution = NS(toMsg=lambda: NS(sub_trajectory=[points] * (len(roles) + (1 if m.single_plan else 0))))
        m.leaf_stage_names = lambda task: []
        m.regrasp_request = lambda goal, grasp: None
        m.execution_text = lambda code: f'error {code}'
        m.open_after_failed_grasp = lambda *a: self.commands.append('open after the failure')
        m.release_empty_grasp = lambda *a: self.commands.append('release')
        gripper, arm = list(gripper_codes), list(arm_codes)
        def next_code(script):   # the last scripted code repeats
            return getattr(codes, script.pop(0) if len(script) > 1 else script[0])
        m.command_gripper = lambda posture: self.commands.append('gripper') or next_code(gripper)
        m.execute_arm = lambda trajectory: self.commands.append('arm') or next_code(arm)
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc, p.detoured, p.pick_note = m, '', ''
        p.arm_group_name, p.planning_time, p.external_grasp_batch_size, p.cable_gate_passing = 'arm', 5.0, 0, 0
        p.cable_replay_detour = lambda result: None
        p.pick_action_server = NS(is_preempt_requested=lambda: False)
        p.robot = NS(arm=NS(get_end_effector_link=lambda: 'tcp'))
        p.extras = []
        p.grasp_planner = NS(extra_grasps_msgs=lambda *a: p.extras.append('asked') or self.grasps('grasp_32'))
        p.note_regrasp_info = lambda *a: None
        p.move_arm_to_posture = lambda pose: self.commands.append(('untangle', pose)) or True
        return p

    def grasps(self, ident='grasp_0'):
        return [NS(id=ident, grasp_quality=1.0, pre_grasp_posture='open', grasp_posture='close')]

    def run_pick(self, p, single=None):
        from grasplan.tools.common import objectToPick
        target = objectToPick('soup_1')
        params = {'~extra_grasp_sets': True}
        with mock.patch.object(self.mpp, 'announce_trajectory_start'), \
                mock.patch.object(self.mpp, 'plan_transport_during_pick', return_value=False), \
                mock.patch.object(self.mpp.rospy, 'get_param', lambda n, d=None: params.get(n, d)), \
                mock.patch.object(self.pick.rospy, 'get_param', lambda n, d=None: params.get(n, d)), \
                mock.patch.object(self.mpp.rospy, 'logwarn'), mock.patch.object(self.mpp.rospy, 'logerr'), \
                mock.patch.object(self.mpp.rospy, 'loginfo'), mock.patch.object(self.pick.rospy, 'logwarn'), \
                mock.patch.object(self.pick.rospy, 'loginfo'):
            if single is None:
                result = p.offer_grasps(target, self.grasps(), 'table_2', False)
            else:
                result = p.offer_single_plan(target, self.grasps(), 'table_2', single)
            return result, p.offer_extra_grasps(target, None, None, 'table_2', 'side_grasp', single, result)

    def test_a_failed_pre_open_without_a_gripper_server_offers_no_extras(self):
        p = self.node(['CONTROL_FAILED'])
        result, after = self.run_pick(p)
        self.assertEqual(result, self.pick.MoveItErrorCodes.CONTROL_FAILED)
        self.assertEqual(after, result)
        self.assertTrue(p.mtc.pick_stage_executed)
        self.assertEqual(self.commands[0], 'gripper')
        self.assertNotIn('arm', self.commands)
        self.assertEqual(p.extras, [])

    def test_a_pre_open_timeout_in_the_single_plan_offers_no_extras(self):
        p = self.node(['TIMED_OUT'])
        result, after = self.run_pick(p, single=([], 'transport'))
        self.assertEqual((result, after), (self.pick.MoveItErrorCodes.TIMED_OUT,) * 2)
        self.assertTrue(p.mtc.pick_stage_executed)
        self.assertEqual(p.extras, [])

    def test_without_a_planned_grasp_the_extras_still_come(self):
        p = self.node([])
        p.mtc.plan = lambda task, what: False
        p.mtc.next_branch_plan = lambda goal, grasp, what: (None, None, False)
        p.mtc.turned_grasp_plan = lambda goal, grasp: (None, None, None)
        p.mtc.log_failed_branch = lambda grasp: None
        p.mtc.unplanned_breakdown = lambda: ''
        result, after = self.run_pick(p)
        self.assertEqual(result, self.pick.MoveItErrorCodes.PLANNING_FAILED)
        self.assertEqual(p.extras, ['asked'])
        self.assertFalse(p.mtc.pick_stage_executed)
        self.assertEqual(self.commands, [])


@unittest.skipIf(soup_files() is None, 'mobipick_pick_n_place soup grasps not found')
class TestSoupEndToEndRing(unittest.TestCase):
    def setUp(self):
        stored_path, extra_path = soup_files()
        self.stored = yaml.safe_load(open(stored_path))['soup']['grasp_poses']
        self.extra = yaml.safe_load(open(extra_path))['soup']['grasp_poses']

    def test_sixteen_end_to_end_grasps_closing_along_the_can_axis(self):
        self.assertEqual(len(self.extra), 16)
        axis = np.array([0.0, 1.0, 0.0])   # the can axis in its object frame
        for g in self.extra:
            m = quaternion_to_matrix(g['rotation'])
            self.assertAlmostEqual(abs(m[:, 1] @ axis), 1.0, places=5)   # closing along the axis
            self.assertAlmostEqual(m[:, 0] @ axis, 0.0, places=5)        # approach across it

    def test_the_ring_keeps_the_stored_backoff_and_the_eight_directions_twice(self):
        backoff = {round(float(np.linalg.norm(g['translation'])), 4) for g in self.extra}
        self.assertEqual(backoff, {0.0069})
        approaches = [tuple(np.round(quaternion_to_matrix(g['rotation'])[:, 0], 3)) for g in self.extra]
        self.assertEqual(len(set(approaches)), 8)
        self.assertTrue(all(approaches.count(a) == 2 for a in set(approaches)))

    def test_the_stored_list_is_unchanged(self):
        self.assertEqual(len(self.stored), 32)


if __name__ == '__main__':
    unittest.main()
