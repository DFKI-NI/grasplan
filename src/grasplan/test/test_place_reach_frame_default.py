'''The reach frame the place poses are ordered around defaults to <robot>/<~arm_name>_base_link: ur5_base_link on
Mobipick as before, the other arm's base link on another robot; ~place_reach_frame still overrides it. Mocks rosparam
(no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from unittest import mock

import grasplan.place as place


class TestReachFrame(unittest.TestCase):
    def frame(self, params):
        with mock.patch.object(place.rospy, 'get_param', lambda name, default=None: params.get(name, default)):
            return place.PlaceTools.__new__(place.PlaceTools).place_reach_frame()

    def test_default_is_the_old_frame(self):
        self.assertEqual(self.frame({}), 'mobipick/ur5_base_link')

    def test_arm_name_decides_the_default(self):
        self.assertEqual(self.frame({'~arm_name': 'arm', '~robot_prefix': 'tiago'}), 'tiago/arm_base_link')

    def test_parameter_overrides(self):
        self.assertEqual(self.frame({'~arm_name': 'arm', '~place_reach_frame': 'base_footprint'}), 'base_footprint')


if __name__ == '__main__':
    unittest.main()
