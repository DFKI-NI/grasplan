'''The one switch for the two exclusive cable modes (~cable_detour_mode, careful, hack in the code): in hack mode a closed-set
pick, place and insert go through the fixed untangle poses as always; in cable_aware mode none of them visits an untangle
pose, the cable replay detour is out as well, and a motion the cable guard refused every plan for says so in its failure
text. Open-set objects are unchanged in both modes. Mocks for rosparam and the arm (no ROS master); needs the Mobipick
image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import grasplan.cable_detour as cable_detour
import grasplan.insert as insert
import grasplan.pick as pick
import grasplan.place as place

CATALOG = '/mobipick/pick_object_node/handcoded_grasp_planner_transforms'
DETOUR = ['untangle_cable_guide_1_right', 'untangle_cable_guide_2_right']
REFUSED = 'none of the 3 grasps could be planned: 3 refused by the cable guard (wrist span)'


class Fixture(unittest.TestCase):
    def setUp(self):
        self.params = {CATALOG: {'multimeter': {}, 'klt': {}}}
        self.moves = []
        patcher = mock.patch.object(cable_detour.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)

    def mode(self, name):
        self.params['~cable_detour_mode'] = name

    def pick_tools(self):
        p = pick.PickTools.__new__(pick.PickTools)
        p.list_of_disentangle_objects, p.poses_to_go_before_pick = ['multimeter', 'relay'], list(DETOUR)
        p.mtc = self.guarded_mtc()
        return p

    @staticmethod
    def guarded_mtc():
        return NS(update_cable_guard=lambda: None, cable_guard=object())

    def place_tools(self):
        p = place.PlaceTools.__new__(place.PlaceTools)
        p.move_arm_to_posture = lambda name: self.moves.append(name) or True
        p.mtc = self.guarded_mtc()
        return p


class TestMode(Fixture):
    def test_default_is_hack(self):
        self.assertEqual(cable_detour.cable_detour_mode(), 'hack')
        self.assertFalse(cable_detour.requested_cable_aware())
        self.assertFalse(cable_detour.cable_aware(self.guarded_mtc()))

    def test_cable_aware_is_read(self):
        self.mode('cable_aware')
        self.assertTrue(cable_detour.requested_cable_aware())
        self.assertTrue(cable_detour.cable_aware(self.guarded_mtc()))

    def test_an_unknown_value_counts_as_hack(self):
        self.mode('cable-aware')
        self.assertEqual(cable_detour.cable_detour_mode(), 'hack')


class TestPick(Fixture):
    def test_hack_keeps_the_untangle_poses_of_a_listed_object(self):
        self.assertEqual(self.pick_tools().untangle_poses_before_pick('multimeter'), DETOUR)

    def test_cable_aware_has_no_untangle_pose_for_a_closed_set_object(self):
        self.mode('cable_aware')
        self.assertEqual(self.pick_tools().untangle_poses_before_pick('multimeter'), [])
        self.assertEqual(self.pick_tools().untangle_poses_before_pick('relay'), [])

    def test_an_open_set_object_has_none_in_both_modes(self):
        for name in ('hack', 'cable_aware'):
            self.mode(name)
            self.assertEqual(self.pick_tools().untangle_poses_before_pick('apple'), [])

    def test_cable_aware_without_a_cable_guard_keeps_the_poses_and_warns(self):
        self.mode('cable_aware')
        for mtc in (None, NS(update_cable_guard=lambda: None, cable_guard=None)):
            p = self.pick_tools()
            p.mtc = mtc
            with mock.patch.object(cable_detour.rospy, 'logwarn') as warn:
                self.assertEqual(p.untangle_poses_before_pick('multimeter'), DETOUR)
            self.assertIn('no MTC cable guard', warn.call_args[0][0])

    def test_cable_aware_never_takes_the_replay_detour(self):
        self.params['~mtc_cable_replay_detour'] = True
        self.mode('cable_aware')
        p = self.pick_tools()
        p.mtc = NS(executed=False, cable_guard=object(), chain_rejections=2)
        p.pick_action_server = NS(is_preempt_requested=lambda: False)
        p.open_set_pick = False
        self.assertIsNone(p.cable_replay_detour(-1))


class TestPlaceAndInsert(Fixture):
    def test_hack_visits_the_poses_before_place_and_insert(self):
        self.place_tools().go_before_place('multimeter_1', DETOUR)   # the insert node calls the same method
        self.assertEqual(self.moves, DETOUR)

    def test_cable_aware_visits_no_pose_before_place_and_insert(self):
        self.mode('cable_aware')
        self.place_tools().go_before_place('multimeter_1', DETOUR)
        self.assertEqual(self.moves, [])

    def test_cable_aware_without_a_cable_guard_keeps_the_poses_and_warns(self):
        self.mode('cable_aware')
        for mtc in (None, NS(update_cable_guard=lambda: None, cable_guard=None)):
            self.moves.clear()
            p = self.place_tools()
            p.mtc = mtc
            with mock.patch.object(cable_detour.rospy, 'logwarn') as warn:
                p.go_before_place('klt_1', DETOUR)   # the insert node calls the same method
            self.assertEqual(self.moves, DETOUR)
            self.assertIn('no MTC cable guard', warn.call_args[0][0])

    def test_an_object_picked_open_set_goes_via_transport_in_both_modes(self):
        for name in ('hack', 'cable_aware'):
            self.moves.clear()
            self.mode(name)
            self.place_tools().go_before_place('pringles_1', DETOUR)
            self.assertEqual(self.moves, ['transport'])

    def test_the_place_detour_default_does_not_follow_the_sim_clock(self):
        self.params['/use_sim_time'] = True
        p = self.place_tools()
        p.mtc = NS(update_cable_guard=lambda: None, cable_guard=object(), cable_rejections=0, executed=False)
        p.reset_detour()
        p.go_before_place('multimeter_1', DETOUR)
        self.assertEqual(self.moves, DETOUR)


class TestFailureText(Fixture):
    def pick_text(self):
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc = NS(failure_reason=REFUSED)
        return p.failure_text()

    def place_text(self):
        p = place.PlaceTools.__new__(place.PlaceTools)
        p.mtc = NS(failure_reason=REFUSED)
        return p.failure_text()

    def insert_text(self):
        i = insert.InsertTools.__new__(insert.InsertTools)
        i.place = NS(mtc=NS(failure_reason=REFUSED))
        return i.failure_text()

    def test_hack_leaves_the_reason_as_it_is(self):
        self.assertEqual((self.pick_text(), self.place_text(), self.insert_text()), (REFUSED,) * 3)

    def test_cable_aware_adds_why_no_untangle_pose_was_used_to_all_three_skills(self):
        self.mode('cable_aware')
        for text in (self.pick_text(), self.place_text(), self.insert_text()):
            self.assertTrue(text.startswith(REFUSED), text)
            self.assertIn('cable-aware mode', text)
            self.assertIn('no untangle pose', text)
            self.assertIn('refused 3 plan(s)', text)

    def test_without_a_count_the_sentence_says_the_plans(self):
        self.mode('cable_aware')
        self.assertIn('refused the plans', cable_detour.explain_refusal('all grasps refused by the cable guard'))

    def test_cable_aware_leaves_other_failures_alone(self):
        self.mode('cable_aware')
        self.assertEqual(cable_detour.explain_refusal('the gripper closed on nothing'), 'the gripper closed on nothing')
        self.assertEqual(cable_detour.explain_refusal(''), '')


if __name__ == '__main__':
    unittest.main()
