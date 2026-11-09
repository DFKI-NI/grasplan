'''The arm joints among all joints of a joint state: 'ur5_' on a Mobipick, ~arm_joint_tag on another arm. No code outside
the cable model, whose joint names are its own, spells the UR5 name. Mocks rosparam (no ROS master).'''
import os
import unittest
from unittest import mock

from grasplan.tools import common

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')


class TestArmJointTag(unittest.TestCase):
    def setUp(self):
        common.arm_joint_tag.cache_clear()
        self.addCleanup(common.arm_joint_tag.cache_clear)

    def test_default_is_the_ur5(self):
        with mock.patch('rospy.get_param', lambda name, default=None: default):
            self.assertEqual(common.arm_joint_tag(), 'ur5_')

    def test_parameter(self):
        with mock.patch('rospy.get_param', lambda name, default=None: 'arm_'):
            self.assertEqual(common.arm_joint_tag(), 'arm_')

    def test_read_once(self):
        with mock.patch('rospy.get_param', return_value='arm_') as get_param:
            common.arm_joint_tag()
            common.arm_joint_tag()
        self.assertEqual(get_param.call_count, 1)


class TestNoFilterSpellsTheName(unittest.TestCase):
    def test_sources(self):
        for name in ['pick.py', 'mtc_pick_place.py']:
            with open(os.path.join(ROOT, name)) as f:
                self.assertNotIn("if 'ur5_' in n", f.read(), name)


if __name__ == '__main__':
    unittest.main()
