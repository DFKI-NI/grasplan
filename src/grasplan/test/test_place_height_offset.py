'''The clearance between a placed object's bottom and the support: known objects keep the sampler's default (1 mm)
unless ~place_height_offset sets another; an open-set object is released ~open_set_place_clearance above the support
with yaws around the full circle, as before. Mocks rosparam (no ROS master); needs the Mobipick image for the imports.'''
import math
import unittest
from unittest import mock

import grasplan.place as place
from grasplan.tools.support_plane_tools import OBJECT_HEIGHTS

KNOWN = sorted(OBJECT_HEIGHTS)[0]


class TestSamplingOptions(unittest.TestCase):
    def setUp(self):
        self.params = {}
        patcher = mock.patch.object(place.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.p = place.PlaceTools.__new__(place.PlaceTools)
        self.p.open_set_place_yaw_steps, self.p.open_set_place_clearance = 4, 0.02

    def test_known_object_default_is_no_option_at_all(self):
        self.assertEqual(self.p.place_sampling_options(KNOWN, 'table_1'), {})

    def test_known_object_offset_parameter(self):
        self.params['~place_height_offset'] = 0.01
        self.assertEqual(self.p.place_sampling_options(KNOWN, 'table_1'), {'height_offset': 0.01})

    def test_open_set_object_is_unchanged_and_ignores_the_known_offset(self):
        self.params['~place_height_offset'] = 0.01
        self.assertEqual(
            self.p.place_sampling_options('banana_not_in_table_xyz', 'table_1'),
            {'yaw_range': 2.0 * math.pi, 'yaws': [0.0, math.pi / 2, math.pi, 3 * math.pi / 2], 'height_offset': 0.02},
        )


if __name__ == '__main__':
    unittest.main()
