'''~known_pick_open_set_fallback (default off, careful): a known object whose stored grasps were all refused in planning
(MTC ended at "none of the N grasps could be planned") before any pick stage of the goal ran gets ONE second try in the
same goal with the open-set path (grasp view, open-set grasp finder, measured box). Every other failure keeps today's
text; the flag off keeps today's path. The goal-scoped latch mtc.pick_stage_executed (set before every gripper command
and every pick motion of a solution, never reset by a pickup) rules the second try out once anything of the pick ran.
Mocks for rosparam (no ROS master); needs the Mobipick image for the module imports.'''
import inspect
import unittest
from types import SimpleNamespace as NS
from unittest import mock


def params(**values):
    return lambda name, default=None: values.get(name, default)


class Node:
    '''a PickTools without __init__: the first attempt and the open-set path are scripted'''

    @staticmethod
    def make(first=False, exhausted=32, stage=False, check='', retract='', cancel=False, attached=None,
             second=(True, 'Grasplan picked open-set object soup_1', False), mtc=True, anygrasp_execution=False,
             breakdown='16 failed at grasp/compute ik (wrist clamp vs table_2)'):
        import grasplan.pick as pick
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc = NS(failure_reason='', executed=False, pick_stage_executed=False, planning_exhausted=None,
                   planning_breakdown='', transport_move='left over', transport_done=None) if mtc else None
        p.anygrasp_handles_execution = anygrasp_execution
        p.pick_action_server = NS(is_preempt_requested=lambda: cancel, results=[],
                                  set_succeeded=lambda r, t: p.pick_action_server.results.append(('succeeded', t)),
                                  set_aborted=lambda r, t: p.pick_action_server.results.append(('aborted', t)),
                                  set_preempted=lambda r=None, t='': p.pick_action_server.results.append(('preempted', t)))
        p.scene = NS(get_attached_objects=lambda names=None: dict(attached or {}))
        p.grasp_planner = NS(supports_object=lambda name: name.rsplit('_', 1)[0] == 'soup')
        p.grasp_type = 'side_grasp'
        p.calls = []

        def pick_object(name, support, grasp_type, ignore, **kw):   # the closed-set first attempt
            p.calls.append(('pick_object', name))
            p.pick_failure_reason, p.retract_failure = check, retract
            p.stored_grasp_count, p.pick_target_name = 32, name
            if p.mtc is not None:
                p.mtc.pick_stage_executed = stage
                p.mtc.planning_exhausted = None if first else (p.mtc.pick_goal_token, p.record_target or name, exhausted)
                p.mtc.planning_breakdown = breakdown if exhausted else ''
                p.mtc.failure_reason = '' if first else f'none of the {exhausted} grasps could be planned: {breakdown}'
            return first

        def unknown(goal):
            p.calls.append(('second try', goal.object_name, p.in_open_set_fallback))
            return second

        p.pick_object = pick_object
        p.pick_unknown_object_with_grasplan = unknown
        p.record_target = ''
        p.anygrasp_server_timeout, p.known_server_up = 1.0, True
        p.known_target_generate_client = lambda: NS(wait_for_server=lambda t: p.known_server_up)
        p.known_target_generate_action_name = lambda: '/mobipick/generate_grasps_known_target'
        p.transport_after_pick = lambda success: p.calls.append(('transport_after_pick', success))
        return pick, p

    @staticmethod
    def goal(name='soup_1'):
        return NS(object_name=name, support_surface_name='table_2', ignore_object_list=[])

    @staticmethod
    def run(pick, p, flag=True, name='soup_1'):
        with mock.patch.object(pick.rospy, 'get_param', params(**{'~known_pick_open_set_fallback': flag})), \
                mock.patch.object(pick.rospy, 'logwarn'), mock.patch.object(pick.rospy, 'loginfo'), \
                mock.patch.object(pick.rospy, 'logerr'), mock.patch.object(pick.rospy, 'is_shutdown', return_value=False):
            pick.PickTools.pick_obj_action_callback(p, Node.goal(name))
        return p.pick_action_server.results


class TestTrigger(unittest.TestCase):
    def test_flag_off_keeps_todays_path_and_text(self):
        pick, p = Node.make()
        results = Node.run(pick, p, flag=False)
        self.assertEqual([c[0] for c in p.calls], ['pick_object', 'transport_after_pick'])
        self.assertEqual(results, [('aborted', 'Grasplan pick failed: none of the 32 grasps could be planned: '
                                               '16 failed at grasp/compute ik (wrist clamp vs table_2)')])

    def test_all_stored_grasps_refused_gets_one_second_try_with_the_same_name(self):
        pick, p = Node.make()
        results = Node.run(pick, p)
        self.assertIn(('second try', 'soup_1', True), p.calls)
        self.assertFalse(p.in_open_set_fallback)   # only while it runs
        self.assertEqual([c[0] for c in p.calls], ['pick_object', 'second try', 'transport_after_pick'])
        self.assertEqual(results[0][0], 'succeeded')
        self.assertEqual(results[0][1], 'Grasplan pick succeeded with the grasp finder for unknown objects after all 32 '
                                        'stored grasps of soup_1 were refused (16 failed at grasp/compute ik (wrist '
                                        'clamp vs table_2))')
        self.assertIsNone(p.mtc.transport_move)   # nothing of the first attempt reaches transport_after_pick

    def test_a_miss_names_both_attempts(self):
        pick, p = Node.make(second=(False, 'Grasplan failed to pick open-set object soup_1: no grasp', False))
        results = Node.run(pick, p)
        self.assertEqual(results, [('aborted', 'Grasplan pick failed: all 32 stored grasps of soup_1 were refused (16 '
                                               'failed at grasp/compute ik (wrist clamp vs table_2)); the second try '
                                               'with the grasp finder for unknown objects failed too: Grasplan failed '
                                               'to pick open-set object soup_1: no grasp')])

    def test_a_cancel_during_the_second_try_preempts(self):
        pick, p = Node.make(second=(False, 'AnyGrasp generation was cancelled', True))
        results = Node.run(pick, p)
        self.assertEqual(results, [('preempted', 'AnyGrasp generation was cancelled')])
        self.assertNotIn('transport_after_pick', [c[0] for c in p.calls])

    def test_every_negative_condition_keeps_todays_text(self):
        cases = {
            'success': dict(first=True),
            'other MTC end or zero grasps': dict(exhausted=0),
            'pick stage ran': dict(stage=True),
            'check before MTC': dict(check='cannot pick soup_1: the gripper already holds x'),
            'retract failed': dict(retract='; retracting failed'),
            'cancel': dict(cancel=True),
            'attached': dict(attached={'relay_1': 1}),
            'no MTC': dict(mtc=False),
            'AnyGrasp executes': dict(anygrasp_execution=True),
        }
        for label, kw in cases.items():
            with self.subTest(label):
                pick, p = Node.make(**kw)
                if not kw.get('mtc', True):
                    p.failure_text = lambda: ''
                Node.run(pick, p)
                self.assertNotIn('second try', [c[0] for c in p.calls])

    def test_a_class_only_goal_takes_the_resolved_instance(self):
        pick, p = Node.make()
        orig = p.pick_object

        def resolved(name, *a, **kw):   # pick_object resolves "soup" to its first instance
            result = orig('soup_1', *a, **kw)
            return result
        p.pick_object = resolved
        results = Node.run(pick, p, name='soup')
        self.assertIn(('second try', 'soup_1', True), p.calls)
        self.assertIn('stored grasps of soup_1 were refused', results[0][1])

    def test_a_record_of_another_target_or_goal_gives_no_second_try(self):
        pick, p = Node.make()
        p.record_target = 'soup_2'
        Node.run(pick, p)
        self.assertNotIn('second try', [c[0] for c in p.calls])
        pick, p = Node.make()
        p.pick_goal_token = 5
        p.mtc.planning_exhausted = (5, 'soup_1', 32)   # left from the goal before; this goal's first attempt succeeds
        p.pick_object = lambda *a, **kw: p.calls.append(('pick_object',)) or False
        p.failure_text = lambda: 'executing grasp grasp_3 failed'
        Node.run(pick, p)   # the callback starts goal 6 and clears the record
        self.assertIsNone(p.mtc.planning_exhausted)
        self.assertNotIn('second try', [c[0] for c in p.calls])

    def test_the_refusal_names_the_failed_condition(self):
        pick, p = Node.make(stage=True)
        with mock.patch.object(pick.rospy, 'get_param', params(**{'~known_pick_open_set_fallback': True})), \
                mock.patch.object(pick.rospy, 'loginfo') as info, mock.patch.object(pick.rospy, 'logwarn'), \
                mock.patch.object(pick.rospy, 'is_shutdown', return_value=False):
            pick.PickTools.pick_obj_action_callback(p, Node.goal())
        self.assertTrue(any('not offered: a pick stage of this goal ran' in str(c) for c in info.call_args_list))

    def test_a_server_without_the_known_target_action_gets_no_second_try(self):
        pick, p = Node.make()
        p.known_server_up = False
        with mock.patch.object(pick.rospy, 'get_param', params(**{'~known_pick_open_set_fallback': True})), \
                mock.patch.object(pick.rospy, 'loginfo') as info, mock.patch.object(pick.rospy, 'logwarn'), \
                mock.patch.object(pick.rospy, 'is_shutdown', return_value=False):
            pick.PickTools.pick_obj_action_callback(p, Node.goal())
        self.assertNotIn('second try', [c[0] for c in p.calls])
        self.assertTrue(any('no known-target action (/mobipick/generate_grasps_known_target is not available)' in str(c)
                            for c in info.call_args_list))
        self.assertTrue(p.pick_action_server.results[0][1].startswith('Grasplan pick failed: none of the 32 grasps'))

    def test_one_second_try_per_goal(self):
        pick, p = Node.make()
        p.open_set_fallback_done = True
        with mock.patch.object(pick.rospy, 'get_param', params(**{'~known_pick_open_set_fallback': True})), \
                mock.patch.object(pick.rospy, 'loginfo'), mock.patch.object(pick.rospy, 'is_shutdown', return_value=False):
            self.assertEqual(p.open_set_fallback_refusal(False), 'this goal already had its second try')
        # the callback starts each goal fresh
        Node.run(pick, p)
        self.assertEqual([c[0] for c in p.calls].count('second try'), 1)

    def test_an_unknown_object_takes_the_open_set_path_as_before(self):
        pick, p = Node.make(second=(True, 'Grasplan picked open-set object banana_1', False))
        p.grasp_planner = NS(supports_object=lambda name: False)
        results = Node.run(pick, p, name='banana_1')
        self.assertEqual(p.calls, [('second try', 'banana_1', False)])
        self.assertEqual(results, [('succeeded', 'Grasplan picked open-set object banana_1')])


class TestInsideTheSecondTry(unittest.TestCase):
    def node(self):
        import grasplan.pick as pick
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc = NS(pick_stage_executed=False)
        p.list_of_disentangle_objects, p.poses_to_go_before_pick = ['mustard'], ['untangle_cable_guide_1_right']
        p.picked = []
        p.pick_object = lambda *a, **kw: p.picked.append(a[0]) or True
        p.failure_text = lambda: ''
        p.grasp_type = 'side_grasp'
        return pick, p

    def test_an_unknown_object_keeps_its_top_down_fallback(self):
        pick, p = self.node()
        p.in_open_set_fallback = False
        with mock.patch.object(pick.rospy, 'get_param', params(**{'~open_set_topdown_fallback': True})), \
                mock.patch.object(pick.rospy, 'logwarn'):
            success, _, _ = pick.PickTools.pick_topdown_without_anygrasp(p, Node.goal('banana_1'), 'no grasp')
        self.assertTrue(success)
        self.assertEqual(p.picked, ['banana_1'])

    def test_no_untangle_poses_in_the_second_try(self):
        pick, p = self.node()
        with mock.patch.object(pick.cable_detour, 'cable_aware', return_value=False), \
                mock.patch.object(pick.rospy, 'loginfo'):
            p.in_open_set_fallback = False
            self.assertEqual(p.untangle_poses_before_pick('mustard'), ['untangle_cable_guide_1_right'])
            p.in_open_set_fallback = True
            self.assertEqual(p.untangle_poses_before_pick('mustard'), [])


def box(cls, inst, x, y=0.0, z=0.8, size=(0.07, 0.07, 0.1), q=(0.0, 0.0, 0.0, 1.0)):
    return NS(class_id=cls, instance_id=inst, pose=NS(position=NS(x=x, y=y, z=z), orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3])),
              size=NS(x=size[0], y=size[1], z=size[2]))


class TestKnownTargetGate(unittest.TestCase):
    '''all 8 corners of the measured box must lie inside the known entry's box grown by the margin, in the known box
    frame (fail closed); a position and extent check, not identity'''

    def node(self, entries, frame='map', matrix=None):
        import grasplan.pick as pick
        p = pick.PickTools.__new__(pick.PickTools)
        p.global_reference_frame = 'map'
        p.pose_selector_get_all_poses_srv = lambda: NS(poses=NS(header=NS(frame_id=frame), objects=entries))
        p.frame_matrix = lambda target, source: matrix
        return pick, p

    def gate(self, p, pick, detected, **param):
        self.logged = []
        with mock.patch.object(pick.rospy, 'get_param', params(**param)), \
                mock.patch.object(pick.rospy, 'loginfo', side_effect=self.logged.append):
            return p.known_target_gate('soup_1', detected)

    def detected(self, x, y=0.0, frame='map', size=(0.07, 0.07, 0.1), q=(0.0, 0.0, 0.0, 1.0)):
        d = box('soup', 1, x, y, size=size, q=q)
        d.header = NS(frame_id=frame)
        return d

    def test_inside_the_box_and_margin_passes_and_the_spill_is_logged(self):
        pick, p = self.node([box('klt', 1, 0.2), box('soup', 1, 1.0)])
        self.assertEqual(self.gate(p, pick, self.detected(1.02)), '')   # reaches 2 cm beyond along x: allowed 3
        self.assertIn('reaches 20 / 0 / 0 mm beyond the box of soup_1 along its x / y / z (allowed 30 mm', self.logged[-1])
        self.assertIn('measured box 70 x 70 x 100 mm, yaw 0 deg, padding 0 mm, unknown points, depth completed: unknown',
                      self.logged[-1])
        self.assertIn('inside', self.logged[-1])
        d = self.detected(1.02)
        d.box_padding, d.shape = 0.01, NS(valid=True, points=812, height_reliable=False)
        self.gate(p, pick, d)
        self.assertIn('padding 10 mm, 812 points, depth completed: yes (height from completed depth)', self.logged[-1])

    def test_a_neighbour_is_refused(self):
        pick, p = self.node([box('soup', 1, 1.0), box('soup', 2, 1.09)])   # two cans side by side
        why = self.gate(p, pick, self.detected(1.09))
        self.assertIn('reaches 9.0 cm beyond where soup_1 was detected', why)
        self.assertIn('not picked', why)
        self.assertIn('outside', self.logged[-1])
        self.assertIn('beyond', self.gate(p, pick, self.detected(1.02), **{'~known_pick_fallback_target_margin': 0.0}))

    def test_a_mask_merged_with_a_wall_piece_is_refused_although_its_centre_is_inside(self):
        # the can (7 cm) merged with 12 cm of klt wall beside it: one 19 cm box whose centre is 6 cm off, inside the old
        # centre rule's 3.5 + 3 cm
        pick, p = self.node([box('soup', 1, 1.0)])
        self.assertNotEqual(self.gate(p, pick, self.detected(1.06, size=(0.19, 0.07, 0.1))), '')

    def test_the_known_object_turned_in_its_measured_box_passes(self):
        import math
        s = math.sin(math.pi / 8)
        c = math.cos(math.pi / 8)   # turned 45 deg about z: 68 x 102 mm reaches 60 mm from its centre in x and y
        pick, p = self.node([box('soup', 1, 1.0, size=(0.068, 0.102, 0.068))])
        self.assertEqual(self.gate(p, pick, self.detected(1.0, size=(0.068, 0.102, 0.068), q=(0.0, 0.0, s, c))), '')

    def test_a_small_object_inside_the_volume_passes_the_documented_limit(self):
        pick, p = self.node([box('soup', 1, 1.0)])
        self.assertEqual(self.gate(p, pick, self.detected(1.04, size=(0.02, 0.02, 0.02))), '')   # no identity proof

    def test_the_box_orientation_counts(self):
        import math
        s = math.sin(math.pi / 4)   # the long axis (x, 0.3 m) turned onto map y
        pick, p = self.node([box('soup', 1, 1.0, size=(0.3, 0.07, 0.07), q=(0.0, 0.0, s, s))])
        self.assertEqual(self.gate(p, pick, self.detected(1.0, 0.1)), '')
        self.assertNotEqual(self.gate(p, pick, self.detected(1.15, 0.0)), '')

    def test_a_box_without_a_valid_size_refuses(self):
        pick, p = self.node([box('soup', 1, 1.0)])
        self.assertIn('a box has no valid size', self.gate(p, pick, self.detected(1.0, size=(0.0, 0.07, 0.1))))

    def test_missing_entry_service_or_transform_refuses(self):
        pick, p = self.node([box('soup', 2, 1.0)])
        self.assertIn('no longer in the pose selector', self.gate(p, pick, self.detected(1.0)))
        pick, p = self.node([box('soup', 1, 1.0)], matrix=None)
        self.assertIn('no transform from camera to map', self.gate(p, pick, self.detected(1.0, frame='camera')))

        def down():
            raise pick.rospy.ServiceException('gone')
        p.pose_selector_get_all_poses_srv = down
        self.assertIn('pose selector: gone', self.gate(p, pick, self.detected(1.0)))

    def test_tf_lost_after_the_exact_instance_lookup_is_refused(self):
        pick, p = self.node([box('soup', 1, 1.0)])   # the entry is found, then the transform is gone
        calls = []
        p.frame_matrix = lambda target, source: calls.append((target, source)) or None
        why = self.gate(p, pick, self.detected(1.0, frame='camera'))
        self.assertEqual(calls, [('map', 'camera')])
        self.assertIn('could not check the grasp finder', why)
        self.assertIn('no transform from camera to map', why)

    def test_another_frame_goes_through_tf(self):
        import numpy as np
        shift = np.eye(4)
        shift[0, 3] = 1.0   # camera origin 1 m along map x
        pick, p = self.node([box('soup', 1, 1.0)], matrix=shift)
        self.assertEqual(self.gate(p, pick, self.detected(0.0, frame='camera')), '')


class TestSecondTryOpenSetPath(unittest.TestCase):
    '''inside pick_unknown_object_with_grasplan: rechecks before the view and before the goal, known_target, the gate'''

    def node(self, cancel_at=None, attached_at=None, gate='', result=None):
        import grasplan.pick as pick
        p = pick.PickTools.__new__(pick.PickTools)
        p.in_open_set_fallback = True
        p.anygrasp_generate_action_name, p.anygrasp_server_timeout, p.anygrasp_result_timeout = 'generate', 1.0, 0
        p.grasp_planner, p.grasp_type = NS(), 'side_grasp'
        p.events = []
        p.pick_action_server = NS(is_preempt_requested=lambda: cancel_at is not None and cancel_at in p.events)
        p.scene = NS(get_attached_objects=lambda names=None: {'x': 1} if attached_at in p.events else {})
        p.move_to_anygrasp_view = lambda name: p.events.append('view') or (True, '')
        p.stopped_during_grasp_view = lambda: False
        detection = NS(object=NS(class_id='soup', instance_id=1, header=NS(frame_id='map')), grasps=['g'])

        def client(which):
            return NS(wait_for_server=lambda t: True, send_goal=lambda g: p.events.append(('goal', g, which)),
                      wait_for_result=lambda t: True, get_state=lambda: pick.GoalStatus.SUCCEEDED, cancel_goal=lambda: None,
                      get_result=lambda: result or NS(success=True, detection=detection),
                      get_goal_status_text=lambda: 'ok')
        p.anygrasp_generate_action_client = client('open-set')
        p.known_target_generate_client = lambda: client('known target')
        p.known_target_generate_action_name = lambda: 'generate_known_target'
        p.known_target_gate = lambda name, obj: p.events.append('gate') or gate
        p.pick_object = lambda *a, **kw: p.events.append(('pick_object', a[0])) or True
        p.failure_text = lambda: ''
        return pick, p

    def run_try(self, pick, p):
        with mock.patch.object(pick.rospy, 'get_param', params()), mock.patch.object(pick.rospy, 'loginfo'), \
                mock.patch.object(pick.rospy, 'logerr'), mock.patch.object(pick.rospy, 'resolve_name', lambda n: n), \
                mock.patch.object(pick.rospy, 'is_shutdown', return_value=False), mock.patch.object(pick, 'TIMER'):
            return p.pick_unknown_object_with_grasplan(Node.goal())

    def test_the_goal_goes_to_the_known_target_action_and_the_gate_runs_before_the_pick(self):
        pick, p = self.node()
        self.assertEqual(self.run_try(pick, p), (True, 'Grasplan picked open-set object soup_1', False))
        goal, which = [e[1:] for e in p.events if isinstance(e, tuple) and e[0] == 'goal'][0]
        self.assertEqual(which, 'known target')
        self.assertEqual(goal.object_name, 'soup_1')
        self.assertEqual(goal.__slots__, ['object_name'])   # the message is unchanged
        names = [e if isinstance(e, str) else e[0] for e in p.events]
        self.assertEqual(names, ['view', 'goal', 'gate', 'pick_object'])

    def test_an_unknown_object_keeps_the_old_action(self):
        pick, p = self.node()
        p.in_open_set_fallback = False
        self.run_try(pick, p)
        which = [e[2] for e in p.events if isinstance(e, tuple) and e[0] == 'goal'][0]
        self.assertEqual(which, 'open-set')
        self.assertNotIn('gate', p.events)

    def test_a_refused_gate_picks_nothing(self):
        pick, p = self.node(gate='the grasp finder\'s object is 9.0 cm from where soup_1 was detected')
        self.assertEqual(self.run_try(pick, p), (False, 'the grasp finder\'s object is 9.0 cm from where soup_1 was '
                                                        'detected', False))
        self.assertNotIn('pick_object', [e[0] for e in p.events if isinstance(e, tuple)])

    def test_a_cancel_or_an_attached_object_after_the_view_sends_no_goal(self):
        pick, p = self.node(cancel_at='view')
        success, text, preempted = self.run_try(pick, p)
        self.assertEqual((success, preempted), (False, True))
        self.assertIn('before the grasp finder was asked', text)
        self.assertEqual(p.events, ['view'])
        pick, p = self.node(attached_at='view')
        success, text, preempted = self.run_try(pick, p)
        self.assertEqual((success, preempted), (False, False))
        self.assertIn('the gripper holds x', text)
        self.assertEqual(p.events, ['view'])

    def test_a_cancel_before_the_view_moves_nothing(self):
        import grasplan.pick as pick
        pick_, p = self.node()
        p.pick_action_server = NS(is_preempt_requested=lambda: True)
        success, text, preempted = self.run_try(pick_, p)
        self.assertTrue(preempted)
        self.assertEqual(p.events, [])

    def test_a_failed_generation_ends_on_the_top_down_of_the_known_entry(self):
        pick, p = self.node(result=NS(success=False, detection=None))
        p.pick_topdown_without_anygrasp = lambda goal, message: (p.events.append(('topdown', goal.object_name))
                                                                 or (False, message, False))
        success, text, _ = self.run_try(pick, p)
        self.assertIn(('topdown', 'soup_1'), p.events)   # the pose selector box of soup_1, left as DOPE wrote it


class TestHeldMode(unittest.TestCase):
    '''the mode of the attached object: written by the second try's attach, cleared by a normal attach, never touched
    with the flag off'''

    def run_note(self, flag, in_fallback, store):
        import grasplan.pick as pick
        p = pick.PickTools.__new__(pick.PickTools)
        p.in_open_set_fallback = in_fallback
        values = dict(store)
        values['~known_pick_open_set_fallback'] = flag
        writes = []
        with mock.patch.object(pick.rospy, 'get_param', lambda n, d=None: values.get(n, d)), \
                mock.patch.object(pick.rospy, 'set_param', lambda n, v: writes.append(('set', n, v)) or store.update({n: v})), \
                mock.patch.object(pick.rospy, 'has_param', lambda n: n in store), \
                mock.patch.object(pick.rospy, 'delete_param', lambda n: writes.append(('delete', n)) or store.pop(n)), \
                mock.patch.object(pick.rospy, 'loginfo'):
            p.note_held_mode('soup_1')
        return writes

    def test_the_second_try_writes_the_held_name(self):
        store = {}
        self.assertEqual(self.run_note(True, True, store), [('set', '~open_set_fallback_held', 'soup_1')])

    def test_a_normal_attach_clears_it(self):
        store = {'~open_set_fallback_held': 'soup_1'}
        self.assertEqual(self.run_note(True, False, store), [('delete', '~open_set_fallback_held')])
        self.assertEqual(self.run_note(True, False, store), [])

    def test_flag_off_writes_and_deletes_nothing(self):
        for fallback in (True, False):
            self.assertEqual(self.run_note(False, fallback, {'~open_set_fallback_held': 'soup_1'}), [])

    def test_the_goal_start_keeps_the_mode(self):
        import grasplan.pick as pick
        source = inspect.getsource(pick.PickTools.pick_obj_action_callback)
        self.assertNotIn('open_set_fallback_held', source)   # a refused next pick while held keeps it

    def test_a_cancel_after_the_attach_keeps_the_mode(self):
        import grasplan.pick as pick
        source = inspect.getsource(pick.PickTools.pick_object)
        attach = source.index('self.ensure_attached(object_to_pick.get_object_class_and_id_as_string())')
        self.assertLess(attach, source.index('self.note_held_mode('))
        self.assertLess(source.index('self.note_held_mode('), source.index('return True', attach))


class TestMtcMarksExhaustedPlanning(unittest.TestCase):
    '''MtcPickPlace.pickup says by an attribute, not by its failure text, that it ended with every grasp refused in
    planning; the latch (pick_stage_executed) is set before every pick motion and gripper command'''

    def mtc(self):
        import grasplan.mtc_pick_place as mpp
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.pick_stage_executed, m.start_move, m.single_plan = False, None, None
        m.update_cable_guard = lambda: None
        m.is_preempt_requested = lambda: False
        m.make_pick_task = lambda goal, grasp: ('task', [mpp.ARM, mpp.GRIPPER])
        m.plan = lambda task, what: False
        m.next_branch_plan = lambda goal, grasp, what: (None, None, False)
        m.turned_grasp_plan = lambda goal, grasp: (None, None, None)
        m.log_failed_branch = lambda grasp: m.unplanned.append('failed at grasp/compute ik')
        m.gripper_links = ['finger']
        m.pick_goal_token = 7
        return mpp, m

    def test_no_plan_sets_the_count_and_the_breakdown(self):
        mpp, m = self.mtc()
        goal = NS(possible_grasps=[NS(id='grasp_0'), NS(id='grasp_1')], target_name='soup_1')
        with mock.patch.object(mpp.rospy, 'logerr'), mock.patch.object(mpp.rospy, 'logwarn'), \
                mock.patch.object(mpp.rospy, 'loginfo'), mock.patch.object(mpp.rospy, 'get_param', params()):
            m.pickup(goal)
        self.assertEqual(m.planning_exhausted, (7, 'soup_1', 2))
        self.assertTrue(m.failure_reason.startswith('none of the 2 grasps could be planned'))
        self.assertEqual(m.planning_breakdown, m.failure_reason.split(': ', 1)[1] if ': ' in m.failure_reason else '')
        self.assertFalse(m.pick_stage_executed)

    def test_each_pickup_starts_with_no_mark(self):
        import grasplan.mtc_pick_place as mpp
        source = inspect.getsource(mpp.MtcPickPlace.pickup)
        start = source.index("self.planning_exhausted, self.planning_breakdown = None, ''")
        self.assertLess(start, source.index('for index, grasp in enumerate(goal.possible_grasps)'))
        self.assertNotIn('pick_stage_executed = False', source)

    def test_the_executor_sets_the_latch_for_pick_motions_and_every_gripper_command(self):
        import grasplan.mtc_pick_place as mpp
        source = inspect.getsource(mpp.MtcPickPlace.execute)
        at = source.index('self.pick_stage_executed = True')
        self.assertIn('if role != UNTANGLE:', source[at - 300:at])
        self.assertLess(at, source.index('code = self.execute_arm(sub.trajectory)'))
        gripper = source.index('elif role == GRIPPER:')
        latch = source.index('self.pick_stage_executed = True', gripper)
        self.assertLess(latch, source.index('code = self.command_gripper(posture)', gripper))

    def test_the_pick_goal_resets_latch_and_mark(self):
        import grasplan.pick as pick
        source = inspect.getsource(pick.PickTools.pick_obj_action_callback)
        self.assertIn('self.mtc.pick_stage_executed = False', source)
        self.assertIn("self.mtc.planning_exhausted, self.mtc.planning_breakdown = None, ''", source)
        self.assertNotIn('failure_reason.startswith', inspect.getsource(pick.PickTools.open_set_fallback_refusal))


class TestRealExecutorGripperFailure(unittest.TestCase):
    '''the real closed-set pipeline (offer_grasps -> _pick_with_mtc -> MtcPickPlace.pickup -> execute) with a planned
    stored grasp whose first executed step, the gripper pre-open, fails: the latch is set, so no second try; without a
    planned grasp the second try is offered'''

    def node(self, gripper_codes, plan=True):
        import grasplan.mtc_pick_place as mpp
        import grasplan.pick as pick
        self.mpp, self.pick, codes = mpp, pick, pick.MoveItErrorCodes
        roles = [mpp.ARM, mpp.GRIPPER, mpp.ATTACH, mpp.ARM]
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.pick_stage_executed, m.start_move, m.single_plan = False, None, None
        m.planning_exhausted, m.planning_breakdown, m.pick_goal_token = None, '', 3
        m.untangle_leg_refused, m.untangle_failure, m.end_move_done, m.end_move_failure = '', '', False, ''
        m.gripper_links, m.cable_guard = ['finger'], None
        m.is_preempt_requested = lambda: False
        m.update_cable_guard = lambda: None
        m.make_pick_task = lambda goal, grasp: ('task', roles)
        m.plan = lambda task, what: plan
        m.next_branch_plan = lambda goal, grasp, what: (None, None, False)
        m.turned_grasp_plan = lambda goal, grasp: (None, None, None)
        m.log_failed_branch = lambda grasp: None
        m.unplanned_breakdown = lambda: ''
        points = NS(trajectory=NS(joint_trajectory=NS(points=[1])))
        m.chosen_solution = NS(toMsg=lambda: NS(sub_trajectory=[points] * len(roles)))
        m.leaf_stage_names = lambda task: []
        m.regrasp_request = lambda goal, grasp: None
        m.execution_text = lambda code: f'error {code}'
        m.open_after_failed_grasp = lambda *a: None
        m.release_empty_grasp = lambda *a: None
        gripper = list(gripper_codes)
        m.command_gripper = lambda posture: getattr(codes, gripper[0])
        m.execute_arm = lambda trajectory: codes.SUCCESS
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc, p.detoured, p.pick_note, p.pick_failure_reason, p.retract_failure = m, '', '', '', ''
        p.arm_group_name, p.planning_time, p.external_grasp_batch_size, p.cable_gate_passing = 'arm', 5.0, 0, 0
        p.open_set_fallback_done, p.anygrasp_handles_execution = False, False
        p.pick_goal_token, p.pick_target_name = 3, 'soup_1'
        p.cable_replay_detour = lambda result: None
        p.pick_action_server = NS(is_preempt_requested=lambda: False)
        p.scene = NS(get_attached_objects=lambda names=None: {})
        p.robot = NS(arm=NS(get_end_effector_link=lambda: 'tcp'))
        return p

    def run_pick(self, p):
        from grasplan.tools.common import objectToPick
        grasps = [NS(id='grasp_0', grasp_quality=1.0, pre_grasp_posture='open', grasp_posture='close')]
        with mock.patch.object(self.mpp, 'announce_trajectory_start'), \
                mock.patch.object(self.mpp, 'plan_transport_during_pick', return_value=False), \
                mock.patch.object(self.mpp.rospy, 'get_param', params()), \
                mock.patch.object(self.pick.rospy, 'get_param', params()), \
                mock.patch.object(self.mpp.rospy, 'logwarn'), mock.patch.object(self.mpp.rospy, 'logerr'), \
                mock.patch.object(self.mpp.rospy, 'loginfo'), mock.patch.object(self.pick.rospy, 'logwarn'), \
                mock.patch.object(self.pick.rospy, 'loginfo'), \
                mock.patch.object(self.pick.rospy, 'is_shutdown', return_value=False):
            p.offer_grasps(objectToPick('soup_1'), grasps, 'table_2', False)
            return p.open_set_fallback_refusal(False)

    def test_a_failed_pre_open_rules_the_second_try_out(self):
        for code in ('CONTROL_FAILED', 'TIMED_OUT'):
            with self.subTest(code):
                p = self.node([code])
                self.assertEqual(self.run_pick(p), 'the pick did not end with every stored grasp refused in planning')
                self.assertTrue(p.mtc.pick_stage_executed)

    def test_without_a_planned_grasp_the_second_try_is_offered(self):
        p = self.node([], plan=False)
        self.assertEqual(self.run_pick(p), '')
        self.assertEqual(p.mtc.planning_exhausted, (3, 'soup_1', 1))
        self.assertFalse(p.mtc.pick_stage_executed)


if __name__ == '__main__':
    unittest.main()
