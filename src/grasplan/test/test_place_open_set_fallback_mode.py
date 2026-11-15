'''An object the pick node's second try picked with the open-set grasp finder (~known_pick_open_set_fallback on, its
name in ~open_set_fallback_held, still attached) is placed as an open-set object although its class is known: no
untangle detour, the open-set sampling options, the box markers and no resting roll of its class. The place goal drops
the note once the object is let go. With the flag off a left-over note does nothing and nothing is deleted. Mocks for
rosparam (no ROS master); needs the Mobipick image for the module imports.'''
import math
import unittest
from unittest import mock

import grasplan.place as place
from grasplan.tools import support_plane_tools as spt

CATALOG = '/mobipick/pick_object_node/handcoded_grasp_planner_transforms'
FLAG = '/mobipick/pick_object_node/known_pick_open_set_fallback'
HELD = '/mobipick/pick_object_node/open_set_fallback_held'
DETOUR = ['untangle_cable_guide_1_right', 'untangle_cable_guide_2_right']


class Fixture(unittest.TestCase):
    def setUp(self):
        self.params = {CATALOG: {'soup': {}, 'multimeter': {}}, FLAG: True, HELD: 'soup_1'}
        self.deleted = []
        for target, fake in (('get_param', lambda name, default=None: self.params.get(name, default)),
                             ('delete_param', lambda name: self.deleted.append(name) or self.params.pop(name))):
            patcher = mock.patch.object(place.rospy, target, fake)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.attached = {'soup_1'}
        self.p = place.PlaceTools.__new__(place.PlaceTools)
        self.p.scene = mock.Mock(get_attached_objects=lambda names=None: {n: 1 for n in self.attached
                                                                          if names is None or n in names})
        self.p.open_set_place_yaw_steps, self.p.open_set_place_clearance = 4, 0.02
        self.moves = []
        self.p.move_arm_to_posture = lambda name: self.moves.append(name) or True


class TestMode(Fixture):
    def test_the_held_second_try_object_is_open_set(self):
        self.assertTrue(self.p.picked_by_open_set_fallback('soup_1'))
        self.assertTrue(self.p.picked_open_set('soup_1'))

    def test_another_object_of_the_class_or_one_not_attached_is_not(self):
        self.assertFalse(self.p.picked_by_open_set_fallback('soup_2'))
        self.attached = {'soup_2'}
        self.assertFalse(self.p.picked_by_open_set_fallback('soup_1'))

    def test_flag_off_a_left_over_note_does_nothing(self):
        self.params[FLAG] = False
        self.assertFalse(self.p.picked_by_open_set_fallback('soup_1'))
        self.assertFalse(self.p.picked_open_set('soup_1'))
        self.attached = set()
        self.p.forget_open_set_fallback_held()
        self.assertEqual(self.deleted, [])

    def test_no_untangle_detour_for_it(self):
        with mock.patch.object(place.rospy, 'loginfo'):
            self.p.go_before_place('soup_1', DETOUR)
        self.assertEqual(self.moves, ['transport'])

    def test_a_normally_picked_soup_keeps_the_detour(self):
        self.params[HELD] = ''
        with mock.patch.object(place.cable_detour, 'cable_aware', return_value=False), \
                mock.patch.object(place.rospy, 'loginfo'):
            self.p.go_before_place('soup_1', DETOUR)
        self.assertEqual(self.moves, DETOUR)

    def test_open_set_sampling_options(self):
        with mock.patch.object(place.rospy, 'loginfo'):
            self.assertEqual(self.p.place_sampling_options('soup', 'table_1', open_set=True)['yaw_range'], 2.0 * math.pi)
        self.assertEqual(self.p.place_sampling_options('soup', 'table_1'), {})


class TestForget(Fixture):
    def test_let_go_drops_the_note(self):
        self.attached = set()
        self.p.forget_open_set_fallback_held()
        self.assertEqual(self.deleted, [HELD])

    def test_still_held_keeps_it(self):
        self.p.forget_open_set_fallback_held()
        self.assertEqual(self.deleted, [])

    def test_the_place_goal_forgets_before_it_ends(self):
        import inspect
        source = inspect.getsource(place.PlaceTools.place_obj_action_callback)
        self.assertLess(source.index('self.forget_open_set_fallback_held()'),
                        source.index('self.place_action_server.set_succeeded'))


class TestNoRestingRoll(unittest.TestCase):
    def test_open_set_drops_the_class_roll(self):
        source = spt.gen_place_poses_from_plane.__code__.co_varnames
        self.assertIn('open_set', source)
        self.assertEqual(spt.resting_roll('soup'), -math.pi / 2.0)   # the class roll that a second-try object skips


if __name__ == '__main__':
    unittest.main()
