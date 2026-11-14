'''~drop_unknown_copy_of_target (default off, careful): a closed-set pick leaves out of its planning scene a pose selector
entry without stored grasps that may be the target itself, stored a second time by the search for unknown objects
(each centre inside the other box, at least half of the smaller volume shared, volumes within a factor two); the entry
stays out of the planning scene until the next goal builds it from the pose selector, and the pose selector keeps it. Off it changes
nothing. Numbers of the soup can and its copy from the sim test with the lift tolerance off (standing can, roll 90 deg).
Mocks for the scene, the pose selector and rosparam (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from test_false_boxes_and_hints import I, TABLE
from test_pick_target_log import picker

import grasplan.pick as pick
from grasplan.tools.common import objectToPick
from grasplan.tools.topdown_grasps import same_object_copy

STANDING = (0.7071068, 0.0, 0.0, 0.7071068)   # the can axis (object y) up
SOUP_SIZE = (0.06766, 0.10186, 0.06771)
SOUP = ((0.0, 0.0, 0.77), STANDING, SOUP_SIZE)
COPY_SIZE = (0.073, 0.036, 0.115)
COPY = ((-0.013, 0.0, 0.785), I, COPY_SIZE)       # the unknown-object box of the same can, 1.3 cm off, 1.5 cm up
KNOWN = {'soup', 'klt', 'box'}


def same(target, candidate, target_known=True, candidate_known=False, same_frame=True, container=False):
    return same_object_copy(target, candidate, target_known, candidate_known, same_frame, container)[0]


class TestSameObjectCopy(unittest.TestCase):
    def test_the_copy_of_the_standing_can_is_one(self):
        is_copy, shared = same_object_copy(SOUP, COPY, True, False, True)
        self.assertTrue(is_copy)
        self.assertAlmostEqual(shared, 0.638, places=3)

    def test_without_the_cans_orientation_it_would_be_missed(self):
        # the gates need the real orientation: unrotated the shared part is 0.462 (a false negative, kept)
        self.assertFalse(same(((0.0, 0.0, 0.77), I, SOUP_SIZE), COPY))

    def test_small_contents_of_a_bin_are_kept(self):
        bin_box = ((0.0, 0.0, 0.8), I, (0.4, 0.3, 0.15))
        self.assertFalse(same(bin_box, ((0.0, 0.0, 0.8), I, (0.25, 0.2, 0.12))))   # volume ratio 0.33

    def test_large_contents_of_a_bin_pass_the_geometry_but_not_a_container_target(self):
        # geometry cannot prove identity: distinct contents filling most of a box pass every geometric gate
        bin_box, contents = ((0.0, 0.0, 0.8), I, (0.4, 0.3, 0.15)), ((0.0, 0.0, 0.8), I, (0.32, 0.25, 0.14))
        self.assertTrue(same(bin_box, contents))
        self.assertFalse(same(bin_box, contents, container=True))

    def test_neighbours_are_kept(self):
        touching = ((0.068, 0.0, 0.77), I, (0.08, 0.08, 0.115))           # a second can, 6.8 cm away, padded box
        tilted = ((0.03, 0.0, 0.79), (0.0, 0.3826834, 0.0, 0.9238795), (0.2, 0.04, 0.04))   # 45 deg, beside it
        self.assertFalse(same(SOUP, touching))
        self.assertFalse(same(SOUP, tilted))

    def test_known_classes_other_frames_and_unknown_targets_are_kept(self):
        self.assertFalse(same(SOUP, COPY, candidate_known=True))   # false known-object boxes are not this rule's
        self.assertFalse(same(SOUP, COPY, same_frame=False))
        self.assertFalse(same(SOUP, COPY, target_known=False))


def entry(name, box):
    (x, y, z), (qx, qy, qz, qw), (sx, sy, sz) = box
    class_id, instance_id = name.rsplit('_', 1)
    return NS(class_id=class_id, instance_id=int(instance_id),
              pose=NS(position=NS(x=x, y=y, z=z), orientation=NS(x=qx, y=qy, z=qz, w=qw)),
              size=NS(x=sx, y=sy, z=sz))


class TestPickScene(unittest.TestCase):
    def setUp(self):
        self.params = {'~drop_unknown_copy_of_target': True}
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)

    def node(self, objects):
        p = picker(objects)
        p.grasp_planner = NS(supports_object=lambda name: name.rsplit('_', 1)[0] in KNOWN)
        p.dropped_copies = []
        return p

    def setup_scene(self, p, target='soup_1', **kwargs):
        with mock.patch.object(pick.rospy, 'loginfo'), mock.patch.object(pick.rospy, 'logwarn') as warn:
            p.make_object_pose_and_add_objs_to_planning_scene(objectToPick(target), **kwargs)
        return [c[0][0] for c in warn.call_args_list]

    def test_the_copy_is_left_out_and_said(self):
        p = self.node([entry('soup_can_1', COPY), entry('soup_1', SOUP), entry('box_1', ((0.3, 0.0, 0.77), I, (0.1,) * 3))])
        said = self.setup_scene(p)
        self.assertNotIn('soup_can_1', p.added)
        self.assertIn('soup_1', p.added)
        self.assertIn('box_1', p.added)
        self.assertEqual(list(p.dropped_copies), ['soup_can_1'])
        self.assertTrue(any(s.startswith('soup_can_1: may be the same object as target soup_1') and '64 %' in s
                            for s in said), said)

    def test_any_instance_of_the_class_finds_its_copy(self):
        p = self.node([entry('soup_1', SOUP), entry('soup_can_1', COPY)])
        self.setup_scene(p, target='soup')
        self.assertNotIn('soup_can_1', p.added)

    def test_off_is_as_before(self):
        self.params['~drop_unknown_copy_of_target'] = False
        p = self.node([entry('soup_can_1', COPY), entry('soup_1', SOUP)])
        self.setup_scene(p)
        self.assertIn('soup_can_1', p.added)
        self.assertEqual(p.dropped_copies, [])

    def test_an_open_set_pick_is_not_this_rule(self):
        from test_pick_target_log import detected_object
        p = self.node([entry('soup_1', SOUP)])
        p.complete_one_sided_box = lambda name, pose, size: None
        target = detected_object('soup_can', 1, 'map', COPY[0], COPY_SIZE)
        self.setup_scene(p, target='soup_can_1', external_object=target)
        self.assertEqual(p.dropped_copies, [])


class TestGoalEnd(unittest.TestCase):
    '''nothing is put back by the pick goal, whatever its end: the left-out entry stays out of the planning scene until
    the next goal builds the scene from the pose selector (as the open-set path does with boxes on its target)'''

    def node(self, outcome):
        p = picker([entry('soup_can_1', COPY), entry('soup_1', SOUP)])
        p.grasp_planner = NS(supports_object=lambda name: name.rsplit('_', 1)[0] in KNOWN)
        p.grasp_type = 'side_grasp'
        p.mtc = None
        p.transport_after_pick = lambda success: None
        p.failure_text = lambda: 'no grasp'
        self.results, self.cancel = [], False
        p.pick_action_server = NS(is_preempt_requested=lambda: self.cancel,
                                  set_succeeded=lambda result, text='': self.results.append(('ok', text)),
                                  set_aborted=lambda result, text='': self.results.append(('failed', text)),
                                  set_preempted=lambda *a: self.results.append(('cancelled', '')))

        def pick_object(*args):
            p.make_object_pose_and_add_objs_to_planning_scene(objectToPick('soup_1'))
            if outcome == 'cancel after attach':
                p.added['soup_1 attached'] = True
                self.cancel = True
            if outcome == 'raise':
                raise RuntimeError('planning died')
            return outcome == 'ok'

        p.pick_object = pick_object
        return p

    def run_goal(self, outcome, flag=True):
        p = self.node(outcome)
        params = {'~drop_unknown_copy_of_target': flag}
        goal = NS(object_name='soup_1', support_surface_name='table_2', ignore_object_list=[])
        with mock.patch.object(pick.rospy, 'loginfo'), mock.patch.object(pick.rospy, 'logwarn'), \
                mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: params.get(name, default)):
            try:
                p.pick_obj_action_callback(goal)
            except RuntimeError:
                pass
        return p

    def test_after_success_it_stays_out_and_the_result_names_it(self):
        p = self.run_goal('ok')
        self.assertNotIn('soup_can_1', p.added)
        self.assertEqual(self.results[0][0], 'ok')
        self.assertIn('a second entry soup_can_1 at the same place was left out of the obstacle map for this pick',
                      self.results[0][1])

    def test_after_a_failure_it_stays_out_and_the_targets_own_box_is_in_the_scene(self):
        p = self.run_goal('failed')
        self.assertNotIn('soup_can_1', p.added)
        self.assertIn('soup_1', p.added)
        self.assertEqual(self.results[0][0], 'failed')
        self.assertNotIn('second entry', self.results[0][1])

    def test_after_a_cancel_after_the_attach_nothing_is_put_back(self):
        p = self.run_goal('cancel after attach')
        self.assertNotIn('soup_can_1', p.added)
        self.assertEqual(self.results[0][0], 'cancelled')

    def test_after_an_error_it_stays_out(self):
        self.assertNotIn('soup_can_1', self.run_goal('raise').added)

    def test_the_next_goal_with_the_flag_off_adds_it_again(self):
        p = self.run_goal('failed')
        params = {'~drop_unknown_copy_of_target': False}
        with mock.patch.object(pick.rospy, 'loginfo'), mock.patch.object(pick.rospy, 'logwarn'), \
                mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: params.get(name, default)):
            p.pick_obj_action_callback(NS(object_name='soup_1', support_surface_name='table_2', ignore_object_list=[]))
        self.assertIn('soup_can_1', p.added)
        self.assertNotIn('second entry', self.results[-1][1])

if __name__ == '__main__':
    unittest.main()
