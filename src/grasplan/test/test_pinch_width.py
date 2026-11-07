'''The width the rim pinch check judges a finger gap against: the perceived box without the detector's padding, and the
box side nearest the closing axis when the axis is near one, since the closing pads turn a free box flat between them. The
numbers are a 45 x 93 x 176 mm box in the sim, perceived 53 x 102 x 179 mm with a padding of 5 mm per side. Pure geometry.'''
import math
import unittest

import numpy as np

from grasplan.tools.pad_footprint import pinch_width, width_across_pads
from grasplan.tools.pad_gap import PINCH_GAP_RATIO, is_pinch

PERCEIVED = [0.053, 0.102, 0.179]
PADDING = 0.005
GAP_ON_THE_THIN_SIDE = 0.040   # the sim's finger gap after the close, three runs 39-41 mm


def box_in_tcp(off_deg, shift_x=0.03):
    '''the box with its thin side (axis 0) off_deg from the closing axis (TCP y), turned about its tall axis (TCP z), its
    wide axis along the approach'''
    a = math.radians(off_deg)
    pose = np.eye(4)
    pose[:3, :3] = [[-math.sin(a), math.cos(a), 0.0], [math.cos(a), math.sin(a), 0.0], [0.0, 0.0, 1.0]]
    pose[0, 3] = shift_x
    return pose


class TestPinchWidth(unittest.TestCase):
    def test_a_grasp_near_the_thin_side_is_no_pinch(self):
        for off in (0.0, 5.0, 9.0, 12.0, 17.0, 20.0):
            self.assertFalse(is_pinch(GAP_ON_THE_THIN_SIDE, pinch_width(PERCEIVED, box_in_tcp(off), 0.09, PADDING)), off)

    def test_the_footprint_alone_called_it_a_pinch(self):
        width = width_across_pads(PERCEIVED, box_in_tcp(9.0), 0.09)[0]
        self.assertTrue(is_pinch(GAP_ON_THE_THIN_SIDE, width), width)

    def test_near_a_side_the_width_is_that_side_without_the_padding(self):
        self.assertAlmostEqual(pinch_width(PERCEIVED, box_in_tcp(9.0), 0.09, PADDING), 0.043)

    def test_a_padding_of_a_centimetre_gives_the_same_verdict(self):
        padded = [side + 0.010 for side in PERCEIVED]
        self.assertFalse(is_pinch(GAP_ON_THE_THIN_SIDE, pinch_width(padded, box_in_tcp(9.0), 0.09, 0.010)))

    def test_an_unknown_padding_still_counts_the_side(self):
        self.assertAlmostEqual(pinch_width(PERCEIVED, box_in_tcp(9.0), 0.09), 0.053)

    def test_a_jaw_across_the_diagonal_stays_a_pinch(self):
        width = pinch_width(PERCEIVED, box_in_tcp(45.0), 0.09, PADDING)
        self.assertGreater(width, 0.043 / math.cos(math.radians(45.0)) - 0.001)
        self.assertTrue(is_pinch(0.040, width))

    def test_a_gap_far_below_the_side_stays_a_pinch(self):
        width = pinch_width(PERCEIVED, box_in_tcp(5.0), 0.09, PADDING)
        self.assertTrue(is_pinch(PINCH_GAP_RATIO * width - 0.002, width))

    def test_a_round_object_keeps_its_rim_pinch(self):
        can = [0.095, 0.095, 0.25]   # 75 mm can, padding 10 mm
        self.assertAlmostEqual(pinch_width(can, box_in_tcp(0.0), 0.08, 0.010), 0.075)
        self.assertTrue(is_pinch(0.045, 0.075))

    def test_the_angle_limit_is_an_argument(self):
        self.assertAlmostEqual(pinch_width(PERCEIVED, box_in_tcp(9.0), 0.09, PADDING, side_max_off_deg=20.0), 0.043)
        self.assertGreater(pinch_width(PERCEIVED, box_in_tcp(9.0), 0.09, PADDING, side_max_off_deg=5.0), 0.043)


if __name__ == '__main__':
    unittest.main()
