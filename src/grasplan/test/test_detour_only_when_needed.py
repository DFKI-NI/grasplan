'''#198 (e) (Oscar 2026-09-28: fewer arm motions): with ~untangle_detour_only_when_needed a closed-set place or insert
leaves the untangle_cable_guide detour out of its first try; the second try takes it (once) only when the first failed
before the arm moved and the MTC cable guard rejected a plan. Default on in the sim, off on the real robot. Mocks for
rosparam, the arm and MTC (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import grasplan.insert as insert
import grasplan.place as place

CATALOG = '/mobipick/pick_object_node/handcoded_grasp_planner_transforms'
DETOUR = ['untangle_cable_guide_1_right', 'untangle_cable_guide_2_right']


class Fixture(unittest.TestCase):
    def setUp(self):
        self.params = {CATALOG: {'multimeter': {}, 'klt': {}}, '/use_sim_time': True}
        self.moves = []
        for module in (place, insert):
            patcher = mock.patch.object(module.rospy, 'get_param',
                                        lambda name, default=None: self.params.get(name, default))
            patcher.start()
            self.addCleanup(patcher.stop)
        p = place.PlaceTools.__new__(place.PlaceTools)
        p.move_arm_to_posture = lambda name: self.moves.append(name) or True
        p.mtc = NS(cable_guard=object(), update_cable_guard=lambda: None, cable_rejections=0, executed=False,
                   failure_reason='')
        self.p = p


class TestGoBeforePlace(Fixture):
    def test_first_try_leaves_the_detour_out(self):
        self.p.reset_detour()
        self.p.go_before_place('multimeter_1', DETOUR)
        self.assertEqual(self.moves, [])
        self.assertEqual(self.p.detour_state, 'deferred')

    def test_real_robot_default_keeps_the_detour(self):
        self.params['/use_sim_time'] = False
        self.p.reset_detour()
        self.p.go_before_place('multimeter_1', DETOUR)
        self.p.go_before_place('multimeter_1', DETOUR)   # a second batch of the same try: as before
        self.assertEqual(self.moves, DETOUR + DETOUR)

    def test_no_cable_guard_keeps_the_detour(self):
        self.p.mtc.cable_guard = None
        self.p.reset_detour()
        self.p.go_before_place('multimeter_1', DETOUR)
        self.assertEqual(self.moves, DETOUR)

    def test_rejection_forces_it_once(self):
        self.p.reset_detour()
        self.p.go_before_place('multimeter_1', DETOUR)
        self.p.mtc.cable_rejections = 2
        self.assertTrue(self.p.detour_needed_after_first_try())
        self.p.go_before_place('multimeter_1', DETOUR)
        self.p.go_before_place('multimeter_1', DETOUR)   # next batch: not again
        self.assertEqual(self.moves, DETOUR)

    def test_no_rejection_no_detour(self):
        self.p.reset_detour()
        self.p.go_before_place('multimeter_1', DETOUR)
        self.assertFalse(self.p.detour_needed_after_first_try())
        self.assertEqual(self.moves, [])

    def test_arm_moved_no_detour(self):
        self.p.reset_detour()
        self.p.go_before_place('multimeter_1', DETOUR)
        self.p.mtc.cable_rejections, self.p.mtc.executed = 1, True
        self.assertFalse(self.p.detour_needed_after_first_try())

    def test_reset_clears_the_count(self):
        self.p.mtc.cable_rejections = 3
        self.p.reset_detour()
        self.assertEqual((self.p.detour_state, self.p.mtc.cable_rejections), ('', 0))


class TestPlaceGoal(Fixture):
    '''place_obj_action_callback: the tries and the detour flag they pass to place_object'''
    def setUp(self):
        super().setUp()
        self.tries = []
        self.p.disentangle_without_observe = False
        self.p.max_batch_size = 20
        self.p.place_action_server = mock.Mock(is_preempt_requested=lambda: False)
        self.p.retract_after_failure = lambda what: None
        self.outcomes = []

        def place_object(surface, observe_before_place, number_of_poses, max_batch_size, override_disentangle_dont_doit,
                         override_observe_before_place_dont_doit):
            self.tries.append(override_disentangle_dont_doit)
            if not override_disentangle_dont_doit:
                self.p.go_before_place('multimeter_1', DETOUR)
            ok, rejections = self.outcomes.pop(0)
            self.p.mtc.cable_rejections += rejections
            return ok
        self.p.place_object = place_object

    def goal(self):
        self.p.place_obj_action_callback(NS(support_surface_name='table_2', observe_before_place=True))

    def test_guard_rejection_then_detour_then_success(self):
        self.outcomes = [(False, 1), (True, 0)]
        self.goal()
        self.assertEqual(self.tries, [False, False])   # detour asked for in both tries ...
        self.assertEqual(self.moves, DETOUR)           # ... left out in the first, taken in the second

    def test_other_failure_retries_without_the_detour(self):
        self.outcomes = [(False, 0), (True, 0)]
        self.goal()
        self.assertEqual(self.tries, [False, True])
        self.assertEqual(self.moves, [])

    def test_real_robot_default_as_before(self):
        self.params['/use_sim_time'] = False
        self.outcomes = [(False, 1), (True, 0)]
        self.goal()
        self.assertEqual(self.tries, [False, True])
        self.assertEqual(self.moves, DETOUR)


if __name__ == '__main__':
    unittest.main()
