import unittest
from grasplan.tools.support_plane_tools import compute_object_height_for_insertion, OBJECT_HEIGHTS


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
