'''#192 real 2026-09-28, Pringles can on table_2: (1) a false DOPE screwdriver_1 box floating ~12 cm above the table and
reaching 1.7 % into the can box (under the 20 % overlap rule) held the can at the lift start: all 12 grasps failed at
the lift. A box that reaches into the target at all and floats more than ~drop_floating_boxes_at_target above its
support now stays out of the planning scene (default on in the sim, off on the real robot until tested). (2) the
top-down fallback put the gripper coupling inside the 23.5 cm can box: a straight-down grasp whose TCP would sit more
than ~open_set_topdown_max_depth below the object top is skipped. Mocks for the scene and rosparam (no ROS master);
needs the Mobipick image for the module imports.'''
import unittest

from test_false_boxes_and_hints import I, TABLE, Params, stamped
from test_topdown_grasps import pad_tip

from grasplan.tools.topdown_grasps import height_above_support_top, topdown_grasps

PRINGLES = ((0.0, 0.0, 0.8375), I, (0.075, 0.075, 0.235))            # standing on the table (top 0.72)
SCREWDRIVER = ((0.05, 0.0, 0.866), I, (0.034, 0.245, 0.034))         # bottom 0.849: 12.9 cm above the table
NEIGHBOUR = ((0.07, 0.0, 0.775), I, (0.073, 0.073, 0.11))            # a real can beside it, box 0.2 cm into the Pringles


class TestFloatHeight(unittest.TestCase):
    def test_heights(self):
        self.assertAlmostEqual(height_above_support_top(*SCREWDRIVER, TABLE), 0.849 - 0.72, places=4)
        self.assertEqual(height_above_support_top(*PRINGLES, TABLE), 0.0)
        self.assertEqual(height_above_support_top(*NEIGHBOUR, TABLE), 0.0)   # 1 cm into the table: resting
        self.assertIsNone(height_above_support_top((3.0, 0.0, 0.9), I, (0.1, 0.1, 0.1), TABLE))   # off the table


class TestFloatingBoxAtTheTarget(Params):
    def setUp(self):
        super().setUp()
        self.params['/use_sim_time'] = True
        self.target = (stamped(*PRINGLES[:2]), list(PRINGLES[2]))

    def drops(self, box, name='screwdriver_1'):
        return self.p.overlaps_target(name, stamped(*box[:2]), list(box[2]), self.target, 'pringles_1', {'map': TABLE})

    def test_the_floating_screwdriver_is_left_out_in_the_sim(self):
        self.assertTrue(self.drops(SCREWDRIVER))

    def test_real_robot_default_keeps_it(self):
        self.params['/use_sim_time'] = False
        self.assertFalse(self.drops(SCREWDRIVER))
        self.params['~drop_floating_boxes_at_target'] = 0.03
        self.assertTrue(self.drops(SCREWDRIVER))

    def test_a_resting_neighbour_that_touches_stays(self):
        self.assertFalse(self.drops(NEIGHBOUR, 'soup_1'))

    def test_a_floating_box_beside_the_target_stays(self):
        beside = ((0.2, 0.0, 0.866), I, (0.034, 0.245, 0.034))
        self.assertFalse(self.drops(beside))

    def test_switch_off_and_the_overlap_rule_still_works(self):
        self.params['~drop_floating_boxes_at_target'] = 0.0
        self.assertFalse(self.drops(SCREWDRIVER))
        self.params['~drop_boxes_overlapping_target'] = 0.0
        self.assertFalse(self.drops(PRINGLES, 'false_1'))
        self.params['~drop_boxes_overlapping_target'] = 0.2
        self.assertTrue(self.drops(PRINGLES, 'false_1'))

    def test_without_supports_nothing_floats(self):
        self.assertFalse(self.p.overlaps_target('screwdriver_1', stamped(*SCREWDRIVER[:2]), list(SCREWDRIVER[2]),
                                                self.target, 'pringles_1', []))


class TestTallObjectTopDown(unittest.TestCase):
    def grasps(self, box, max_depth):
        return topdown_grasps(*box, 0.72, pad_tip, min_fingertip_height=0.015, max_width=0.10, max_depth=max_depth)

    def test_pringles_gets_no_topdown_grasp(self):
        grasps, skipped = self.grasps(PRINGLES, 0.12)
        self.assertEqual(grasps, [])
        self.assertTrue(skipped and all('gripper body' in reason for reason in skipped))

    def test_old_behaviour_without_the_limit(self):
        grasps, _ = self.grasps(PRINGLES, 0.0)
        self.assertEqual(len(grasps), 4)

    def test_a_soup_can_keeps_its_grasps(self):
        soup = ((0.0, 0.0, 0.7755), I, (0.068, 0.068, 0.111))
        grasps, _ = self.grasps(soup, 0.12)
        self.assertEqual(len(grasps), 4)


if __name__ == '__main__':
    unittest.main()
