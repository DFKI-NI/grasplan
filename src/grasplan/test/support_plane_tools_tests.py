import math
import unittest

import tf

from grasplan.tools.support_plane_tools import (
    OBJECT_HEIGHTS,
    UPRIGHT_ROLL_CLASSES,
    compute_object_height_for_insertion,
    mesh_resting_height,
    object_height,
    resting_roll,
)


class TestComputeObjectHeightForInsertion(unittest.TestCase):
    def test_known_objects_use_table(self):
        expected = OBJECT_HEIGHTS['klt'] / 2.0 + OBJECT_HEIGHTS['multimeter'] / 2.0 + 0.02
        self.assertAlmostEqual(compute_object_height_for_insertion('multimeter', 'klt'), expected)

    def test_open_set_object_uses_measured_height(self):
        expected = OBJECT_HEIGHTS['klt'] / 2.0 + 0.189 / 2.0 + 0.02
        self.assertAlmostEqual(
            compute_object_height_for_insertion('sugar_box', 'klt', object_tbi_height=0.189), expected
        )

    def test_measured_heights_take_precedence(self):
        self.assertAlmostEqual(
            compute_object_height_for_insertion('multimeter', 'klt', object_tbi_height=0.1, support_obj_height=0.2),
            0.1 + 0.05 + 0.02,
        )

    def test_unknown_object_without_height_raises(self):
        with self.assertRaises(ValueError):
            compute_object_height_for_insertion('sugar_box', 'klt')
        with self.assertRaises(ValueError):
            compute_object_height_for_insertion('multimeter', 'crate', object_tbi_height=0.1)


class TestMeshRestingHeight(unittest.TestCase):
    '''the closed-set resting heights read from the mesh boxes of pbr_objects (no ROS master needed)'''

    DIMENSIONS = {
        'power_drill_with_grip': [17.9966002702713, 22.05359935760498, 8.121799677610397],
        'klt': [14.699999809265137, 19.700000762939453, 29.700000762939453],
        'multimeter': [17.949999868869781, 8.748800307512283, 4.206399992108345],
        'relay': [5.7500001043081284, 4.5099999755620956, 10.436400026082993],
        'screwdriver': [24.500000476837158, 3.4412000328302383, 3.4412000328302383],
        'bleach': [10.243300437927246, 25.058000564575195, 6.769899845123291],
        'hot_glue_gun': [16.4364, 17.7876, 5.2392],
    }
    TRANSFORMS = {'klt': [[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]]}

    def height(self, object_class):
        return mesh_resting_height(object_class, self.DIMENSIONS, self.TRANSFORMS)

    def test_classes_whose_table_value_is_the_box_extent_match_it(self):
        for object_class in ['power_drill_with_grip', 'klt', 'multimeter', 'relay', 'screwdriver']:
            self.assertAlmostEqual(self.height(object_class), OBJECT_HEIGHTS[object_class], places=6, msg=object_class)

    def test_hand_typed_values_differ_by_at_most_two_centimetres(self):
        # known: bleach 0.2506 vs 0.25, glue gun 0.1779 vs 0.2
        self.assertAlmostEqual(self.height('bleach'), 0.25058, places=4)
        self.assertAlmostEqual(self.height('hot_glue_gun'), 0.177876, places=5)
        for object_class in ['bleach', 'hot_glue_gun']:
            self.assertLess(abs(self.height(object_class) - OBJECT_HEIGHTS[object_class]), 0.025)

    def test_class_without_box_gives_none(self):
        self.assertIsNone(self.height('sugar_box'))

    def test_roll_classes_are_the_old_list(self):
        self.assertEqual(
            sorted(UPRIGHT_ROLL_CLASSES), sorted(['power_drill_with_grip', 'hot_glue_gun', 'bleach', 'mustard', 'soup', 'meat'])
        )
        self.assertAlmostEqual(resting_roll('bleach'), -math.pi / 2.0)
        self.assertEqual(resting_roll('klt'), 0.0)

    def test_object_height_flag_off_is_the_table(self):
        for object_class, value in OBJECT_HEIGHTS.items():
            self.assertEqual(object_height(object_class), value)
            self.assertEqual(object_height(object_class, use_mesh_heights=False), value)

    def test_object_height_unknown_class_raises_in_both_modes(self):
        for flag in (False, True):
            with self.assertRaises(KeyError):
                object_height('sugar_box', flag)

    def test_insert_pose_orientation_unchanged_by_refactor(self):
        from geometry_msgs.msg import PoseStamped
        from grasplan.tools.support_plane_tools import gen_insert_poses_from_obj

        support = PoseStamped()
        support.pose.orientation.w = 1.0
        for object_class, roll in [('bleach', -math.pi / 2.0), ('multimeter', 0.0)]:
            poses = gen_insert_poses_from_obj(object_class, support, 0.1).objects
            self.assertEqual(len(poses), 8)
            q = poses[0].pose.orientation
            expected = tf.transformations.quaternion_from_euler(roll, 0.0, 0.0)
            self.assertAlmostEqual(q.x, expected[0])
            self.assertAlmostEqual(q.w, expected[3])
