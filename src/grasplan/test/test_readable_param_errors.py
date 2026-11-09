'''A wrong gripper posture, place translation or height offset parameter fails with a message that names the parameter
and the value, not a raw TypeError. Mocks rosparam (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from unittest import mock

import grasplan.grasp_planning_core as core
import grasplan.place as place
from grasplan.tools.common import as_position_list, gripper_positions


class TestPositions(unittest.TestCase):
    def test_not_a_number(self):
        for value in ('open', ['a', 'b'], None, {'x': 1}):
            with self.assertRaises(ValueError) as caught:
                as_position_list(value, '~gripper_release_distance')
            self.assertIn('~gripper_release_distance', str(caught.exception))

    def test_wrong_length(self):
        with self.assertRaises(ValueError) as caught:
            gripper_positions([0.04], ['left', 'right'], 'the gripper posture')
        for word in ('the gripper posture', 'left', 'right', '0.04'):
            self.assertIn(word, str(caught.exception))

    def test_right_length_passes(self):
        self.assertEqual(gripper_positions(0.1, ['one']), [0.1])
        self.assertEqual(gripper_positions((0.04, 0.04), ['left', 'right']), [0.04, 0.04])


class TestPostureBuilders(unittest.TestCase):
    def test_grasp_posture_names_the_object(self):
        planner = core.GraspPlanningCore.__new__(core.GraspPlanningCore)
        planner.gripper_joint_names = ['left', 'right']
        planner.gripper_joint_efforts = [1.0, 1.0]
        with self.assertRaises(ValueError) as caught:
            planner.make_gripper_trajectory(0.04, {'relay': 0.03}, object_class='relay')
        self.assertIn('relay', str(caught.exception))

    def test_release_posture_names_the_parameter(self):
        node = place.PlaceTools.__new__(place.PlaceTools)
        node.gripper_joint_names = ['left', 'right']
        node.gripper_joint_efforts = [1.0, 1.0]
        with self.assertRaises(ValueError) as caught:
            node.make_gripper_trajectory_msg(0.12)
        self.assertIn('~gripper_release_distance', str(caught.exception))


class TestPlaceParameters(unittest.TestCase):
    DEFAULTS = {'frame_id': 'base', 'axis': [0.0, 0.0, -1.0], 'desired': 0.2, 'min_dist': 0.1}

    def translation(self, given):
        node = place.PlaceTools.__new__(place.PlaceTools)
        with mock.patch.object(place.rospy, 'get_param', lambda name, default=None: given):
            return node.make_gripper_translation_from_param('~pre_place_approach', self.DEFAULTS)

    def test_not_a_dictionary(self):
        with self.assertRaises(ValueError) as caught:
            self.translation(0.2)
        self.assertIn('~pre_place_approach', str(caught.exception))

    def test_bad_values(self):
        for given in ({'axis': [0.0, 1.0]}, {'axis': ['x', 'y', 'z']}, {'axis': 1.0}, {'desired': 'far'}):
            with self.assertRaises(ValueError) as caught:
                self.translation(given)
            self.assertIn('~pre_place_approach', str(caught.exception))

    def test_good_values_build_the_message(self):
        message = self.translation({'desired': 0.3})
        self.assertAlmostEqual(message.desired_distance, 0.3)
        self.assertAlmostEqual(message.direction.vector.z, -1.0)

    def test_height_offset(self):
        node = place.PlaceTools.__new__(place.PlaceTools)
        known = next(iter(place.OBJECT_HEIGHTS))
        with mock.patch.object(place.rospy, 'get_param', lambda name, default=None: 'high'):
            with self.assertRaises(ValueError) as caught:
                node.place_sampling_options(known, 'table')
        self.assertIn('~place_height_offset', str(caught.exception))


if __name__ == '__main__':
    unittest.main()
