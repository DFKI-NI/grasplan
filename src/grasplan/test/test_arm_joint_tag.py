'''The arm joints among all joints of a joint state: 'ur5_' on a Mobipick, ~arm_joint_tag on another arm. No code outside
the cable model, whose joint names are its own, spells the UR5 name. Mocks rosparam (no ROS master).'''
import os
import unittest
from types import SimpleNamespace
from unittest import mock

import grasplan.mtc_pick_place as mpp
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

    def test_empty_is_a_readable_error(self):
        for empty in ('', None):
            with mock.patch('rospy.get_param', lambda name, default=None: empty):
                with self.assertRaises(ValueError) as caught:
                    common.arm_joint_tag()
            self.assertIn('~arm_joint_tag', str(caught.exception))

    def test_an_empty_tag_is_not_kept(self):
        with mock.patch('rospy.get_param', lambda name, default=None: ''):
            with self.assertRaises(ValueError):
                common.arm_joint_tag()
        with mock.patch('rospy.get_param', lambda name, default=None: 'arm_'):
            self.assertEqual(common.arm_joint_tag(), 'arm_')


class TestArmJointsOfAJointState(unittest.TestCase):
    '''one call site: the joints of a joint state that count as the arm's'''
    NAMES = ['torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'gripper_left_finger_joint', 'ur5_elbow_joint']

    def arm(self, tag):
        common.arm_joint_tag.cache_clear()
        self.addCleanup(common.arm_joint_tag.cache_clear)
        msg = SimpleNamespace(name=self.NAMES, position=[0.1, 0.2, 0.3, 0.4, 0.5])
        with mock.patch('rospy.get_param', lambda name, default=None: tag or default), \
                mock.patch.object(mpp.rospy, 'wait_for_message', lambda *args, **kwargs: msg):
            return mpp._current_arm_q()

    def test_default_tag(self):
        self.assertEqual(self.arm(None), {'ur5_elbow_joint': 0.5})

    def test_another_arm(self):
        self.assertEqual(self.arm('arm_'), {'arm_1_joint': 0.2, 'arm_2_joint': 0.3})


class TestNoFilterSpellsTheName(unittest.TestCase):
    def test_sources(self):
        for name in ['pick.py', 'mtc_pick_place.py']:
            with open(os.path.join(ROOT, name)) as f:
                self.assertNotIn("if 'ur5_' in n", f.read(), name)


if __name__ == '__main__':
    unittest.main()
