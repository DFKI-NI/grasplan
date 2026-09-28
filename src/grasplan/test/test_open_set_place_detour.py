'''#151 (Oscar, 2026-09-28, real Pringles can shaken out of the gripper): place and insert of an object picked open-set
(its class is not in the pick node's handcoded grasp catalog) skip the closed-set untangle_cable_guide detour and move
to transport instead; closed-set objects keep the detour. Mocks for rosparam and the arm (no ROS master); needs the
Mobipick image for the module imports.'''
import unittest
from unittest import mock

import grasplan.insert as insert
import grasplan.place as place

CATALOG = '/mobipick/pick_object_node/handcoded_grasp_planner_transforms'
DETOUR = ['untangle_cable_guide_1_right', 'untangle_cable_guide_2_right']


class DetourFixture(unittest.TestCase):
    def setUp(self):
        self.params = {CATALOG: {'multimeter': {}, 'klt': {}, 'soup': {}}}
        self.moves = []
        for module in (place, insert):
            patcher = mock.patch.object(module.rospy, 'get_param',
                                        lambda name, default=None: self.params.get(name, default))
            patcher.start()
            self.addCleanup(patcher.stop)
        self.p = place.PlaceTools.__new__(place.PlaceTools)
        self.p.move_arm_to_posture = lambda name: self.moves.append(name) or True


class TestPlaceDetour(DetourFixture):
    def test_open_set_object_goes_via_transport(self):
        self.p.go_before_place('pringles_1', DETOUR)
        self.assertEqual(self.moves, ['transport'])

    def test_tomato_soup_can_is_open_set_although_soup_is_known(self):
        self.p.go_before_place('tomato_soup_can_1', DETOUR)
        self.assertEqual(self.moves, ['transport'])

    def test_closed_set_object_keeps_the_detour(self):
        self.p.go_before_place('multimeter_1', DETOUR)
        self.assertEqual(self.moves, DETOUR)

    def test_switch_off_keeps_the_detour_for_all(self):
        self.params['~open_set_skip_untangle_detour'] = False
        self.p.go_before_place('pringles_1', DETOUR)
        self.assertEqual(self.moves, DETOUR)

    def test_unreadable_catalog_keeps_the_detour(self):
        del self.params[CATALOG]
        self.p.go_before_place('pringles_1', DETOUR)
        self.assertEqual(self.moves, DETOUR)

    def test_empty_start_pose_means_no_move(self):
        self.params['~open_set_place_start_pose'] = ''
        self.p.go_before_place('pringles_1', DETOUR)
        self.assertEqual(self.moves, [])


class TestInsertUsesIt(DetourFixture):
    def test_insert_of_an_open_set_object_skips_the_detour(self):
        i = insert.InsertTools.__new__(insert.InsertTools)
        i.place = self.p
        self.p.scene = mock.Mock(get_attached_objects=lambda *a: {'pringles_1': None})
        self.p.place_pose_selector_clear_srv = lambda: None
        self.p.place_action_client = mock.Mock()
        i.disentangle_required = True
        i.poses_to_go_before_insert = list(DETOUR)
        i.get_support_object_pose = lambda support: None   # stop right after the detour
        self.assertFalse(i.insert_object('klt_1'))
        self.assertEqual(self.moves, ['transport'])

    def test_insert_of_a_closed_set_object_keeps_it(self):
        i = insert.InsertTools.__new__(insert.InsertTools)
        i.place = self.p
        self.p.scene = mock.Mock(get_attached_objects=lambda *a: {'multimeter_1': None})
        self.p.place_pose_selector_clear_srv = lambda: None
        self.p.place_action_client = mock.Mock()
        i.disentangle_required = True
        i.poses_to_go_before_insert = list(DETOUR)
        i.get_support_object_pose = lambda support: None
        i.insert_object('klt_1')
        self.assertEqual(self.moves, DETOUR)


if __name__ == '__main__':
    unittest.main()
