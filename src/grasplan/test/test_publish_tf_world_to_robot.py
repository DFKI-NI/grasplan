'''publish_tf_world_to_robot: the Gazebo model and the base frame follow the robot, as before on a Mobipick. The script
has no .py ending, so it is loaded by path. Mocks rospy and tf (no ROS master).'''
import importlib.machinery
import importlib.util
import os
import unittest
from unittest import mock

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'scripts', 'publish_tf_world_to_robot')


def load_script():
    loader = importlib.machinery.SourceFileLoader('publish_tf_world_to_robot', SCRIPT)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class TestNames(unittest.TestCase):
    def names(self, namespace, params=None):
        params = params or {}
        script = load_script()
        with mock.patch('rospy.get_param', lambda name, default=None: params.get(name, default)), \
                mock.patch('rospy.get_namespace', return_value=namespace), \
                mock.patch.object(script.rospy, 'Subscriber'), mock.patch.object(script.rospy, 'Rate'), \
                mock.patch.object(script.tf, 'TransformBroadcaster'):
            node = script.PublishTfWorldToRobot()
        return node.robot_model_name, node.robot_base_frame

    def test_mobipick_as_before(self):
        self.assertEqual(self.names('/mobipick/'), ('mobipick', 'mobipick/base_link'))
        self.assertEqual(self.names('/'), ('mobipick', 'mobipick/base_link'))

    def test_second_robot(self):
        self.assertEqual(self.names('/mobipick2/'), ('mobipick2', 'mobipick2/base_link'))

    def test_robot_without_a_prefix_needs_its_model_name(self):
        # no prefix: the frame is bare; the model name still falls back to mobipick, so such a robot sets it
        self.assertEqual(self.names('/', {'~robot_prefix': '/'}), ('mobipick', 'base_link'))
        self.assertEqual(self.names('/', {'~robot_prefix': '/', '~robot_model_name': 'tiago'}), ('tiago', 'base_link'))

    def test_parameters_win(self):
        self.assertEqual(self.names('/mobipick/', {'~robot_base_frame': 'base_footprint'})[1], 'base_footprint')


if __name__ == '__main__':
    unittest.main()
