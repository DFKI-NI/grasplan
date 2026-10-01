'''#226: the pick node's AnyGrasp action and grasp view service names default to its own robot (a second Mobipick
asked robot 1's AnyGrasp and view service before); a single robot keeps the old names. Stubbed (no ROS master).'''
import unittest

from grasplan.pick import anygrasp_default_names


class TestAnygraspDefaultNames(unittest.TestCase):
    def test_single_robot_keeps_the_old_names(self):
        self.assertEqual(anygrasp_default_names('mobipick'), {
            'grasp': '/mobipick/grasp_object', 'generate': '/mobipick/generate_grasps', 'view': '/mobipick/grasp_view'})

    def test_second_robot_gets_its_own_names(self):
        self.assertEqual(anygrasp_default_names('mobipick2'), {
            'grasp': '/mobipick2/grasp_object', 'generate': '/mobipick2/generate_grasps',
            'view': '/mobipick2/grasp_view'})


if __name__ == '__main__':
    unittest.main()
