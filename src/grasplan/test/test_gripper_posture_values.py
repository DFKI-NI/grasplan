'''The gripper posture of a grasp and of a place location takes one position per gripper joint: a number for a gripper
with one joint (Mobipick, the message is the one built before), a list for a gripper with several. Mocks rosparam (no
ROS master); needs the Mobipick image for the module imports.'''
import unittest
from unittest import mock

import grasplan.grasp_planning_core as core
import grasplan.place as place
from grasplan.tools.common import as_position_list


class TestAsPositionList(unittest.TestCase):
    def test_number_and_list(self):
        self.assertEqual(as_position_list(0.1), [0.1])
        self.assertEqual(as_position_list([0.1]), [0.1])
        self.assertEqual(as_position_list((0.04, 0.04)), [0.04, 0.04])
        self.assertEqual(as_position_list(1), [1.0])


class TestGraspPostures(unittest.TestCase):
    def posture(self, value, per_object=None, object_class=None):
        c = core.GraspPlanningCore.__new__(core.GraspPlanningCore)
        c.gripper_joint_names = ['gripper_joint']
        c.gripper_joint_efforts = [10.0]
        with mock.patch.object(core.rospy, 'Duration', lambda secs: secs):
            return c.make_gripper_trajectory(value, per_object, object_class=object_class).points[0].positions

    def test_default_message_is_the_one_built_before(self):
        self.assertEqual(self.posture([0.0]), [0.0])
        self.assertEqual(self.posture([0.4], {'multimeter': 0.3}, 'box'), [0.4])
        self.assertEqual(self.posture([0.4], {'multimeter': 0.3}, 'multimeter'), [0.3])

    def test_scalar_and_list_values(self):
        self.assertEqual(self.posture(0.4), [0.4])
        self.assertEqual(self.posture([0.04, 0.04]), [0.04, 0.04])
        self.assertEqual(self.posture([0.04, 0.04], {'relay': [0.02, 0.02]}, 'relay'), [0.02, 0.02])


class TestPlacePostures(unittest.TestCase):
    def posture(self, value):
        p = place.PlaceTools.__new__(place.PlaceTools)
        p.gripper_joint_names = ['gripper_joint']
        p.gripper_joint_efforts = [10.0]
        with mock.patch.object(place.rospy, 'Duration', lambda secs: secs):
            return p.make_gripper_trajectory_msg(value).points[0].positions

    def test_default_message_is_the_one_built_before(self):
        self.assertEqual(self.posture(0.12), [0.12])

    def test_list_value(self):
        self.assertEqual(self.posture([0.05, 0.05]), [0.05, 0.05])


if __name__ == '__main__':
    unittest.main()
