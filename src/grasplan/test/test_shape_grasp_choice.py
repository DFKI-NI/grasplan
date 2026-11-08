'''choose_grasp_rule: the grasp-type rule from shape features, on synthetic clouds and on feature dicts.'''
import unittest

from grasplan import shape_features as sf
from grasplan import shape_grasp_choice as sgc
from grasplan.test.test_shape_features import (
    box_two_faces, cylinder_front, lying_cylinder, noise, sphere_front)


def rule(cloud, completed_depth=False):
    return sgc.choose_grasp_rule(sf.shape_features(noise(cloud), 0.0, completed_depth=completed_depth))[0]


class ChooseGraspRuleTest(unittest.TestCase):
    def test_a_standing_can_gets_side_grasps(self):
        self.assertEqual(rule(cylinder_front(0.0375, 0.24)), sgc.SIDE_STANDING)

    def test_a_small_ball_is_top_only_a_bigger_ball_keeps_side_grasps(self):
        self.assertEqual(rule(sphere_front(0.02)), sgc.TOP_ONLY)          # 4 cm
        self.assertEqual(rule(sphere_front(0.035)), sgc.SIDE_STANDING)    # 7 cm

    def test_a_low_or_flat_object_is_top_only(self):
        self.assertEqual(rule(box_two_faces((0.06, 0.10, 0.02))), sgc.TOP_ONLY)

    def test_a_lying_can_and_a_long_bar_are_grasped_across_the_short_axis(self):
        self.assertEqual(rule(lying_cylinder(0.0375, 0.2)), sgc.SIDE_SHORT_AXIS)
        self.assertEqual(rule(box_two_faces((0.05, 0.25, 0.07))), sgc.SIDE_SHORT_AXIS)

    def test_a_standing_box_is_a_side_grasp_when_it_fits_across(self):
        self.assertEqual(rule(box_two_faces((0.05, 0.06, 0.15))), sgc.SIDE_STANDING)

    def test_a_standing_cylinder_wider_than_the_gripper_is_auto(self):
        self.assertEqual(rule(cylinder_front(0.08, 0.2)), sgc.AUTO)

    def test_unreliable_features_fall_back_to_auto_with_the_reason(self):
        features = sf.shape_features(noise(cylinder_front(0.03, 0.1, n=60)), 0.0)
        result, why = sgc.choose_grasp_rule(features)
        self.assertEqual(result, sgc.AUTO)
        self.assertIn('not reliable', why)

    def test_a_filled_in_height_does_not_make_a_small_object_low(self):
        # completed depth: the height is not trusted, so the low-object rule does not fire from it; the outline still can
        features = {'height': 0.2, 'height_reliable': False, 'outline_reliable': True, 'shape': 'sphere',
                    'diameter': 0.04, 'flat': None, 'tall': None, 'graspable_across': True, 'footprint_min': 0.04}
        self.assertEqual(sgc.choose_grasp_rule(features)[0], sgc.TOP_ONLY)   # small by its measured outline
        features['diameter'] = 0.07
        self.assertEqual(sgc.choose_grasp_rule(features)[0], sgc.AUTO)

    def test_limits_are_parameters(self):
        features = sf.shape_features(noise(cylinder_front(0.0375, 0.24)), 0.0)
        self.assertEqual(sgc.choose_grasp_rule(features, low_max_height=0.3)[0], sgc.TOP_ONLY)
        self.assertEqual(sgc.choose_grasp_rule(features, side_min_height=0.3)[0], sgc.AUTO)


if __name__ == '__main__':
    unittest.main()
