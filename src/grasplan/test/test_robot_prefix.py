"""Grasplan's per-robot defaults follow the node namespace (mobipick for a single robot, mobipick2 ...)."""
import unittest
from unittest import mock

from grasplan.tools.common import robot_frame, robot_prefix, robot_topic


class TestRobotPrefix(unittest.TestCase):
    def _prefix(self, namespace, param=''):
        with mock.patch('rospy.get_namespace', return_value=namespace), \
                mock.patch('rospy.get_param', side_effect=lambda name, default=None: param or default):
            return robot_prefix()

    def test_namespace_decides(self):
        self.assertEqual(self._prefix('/mobipick/'), 'mobipick')
        self.assertEqual(self._prefix('/mobipick2/'), 'mobipick2')

    def test_root_namespace_keeps_mobipick(self):
        self.assertEqual(self._prefix('/'), 'mobipick')

    def test_param_wins(self):
        self.assertEqual(self._prefix('/mobipick/', param='/robot_b/'), 'robot_b')

    def test_slash_means_a_robot_without_a_prefix(self):
        self.assertEqual(self._prefix('/', param='/'), '')
        self.assertEqual(self._prefix('/mobipick/', param='/'), '')


class TestRobotNames(unittest.TestCase):
    def _names(self, namespace, param=''):
        with mock.patch('rospy.get_namespace', return_value=namespace), \
                mock.patch('rospy.get_param', side_effect=lambda name, default=None: param or default):
            return robot_frame('gripper_tcp'), robot_topic('gripper_hw'), robot_frame('')

    def test_the_mobipick_names_stay(self):
        self.assertEqual(self._names('/mobipick/'), ('mobipick/gripper_tcp', '/mobipick/gripper_hw', 'mobipick/'))
        self.assertEqual(self._names('/'), ('mobipick/gripper_tcp', '/mobipick/gripper_hw', 'mobipick/'))

    def test_a_second_robot_gets_its_own(self):
        self.assertEqual(self._names('/mobipick2/')[:2], ('mobipick2/gripper_tcp', '/mobipick2/gripper_hw'))

    def test_no_prefix_gives_bare_names(self):
        self.assertEqual(self._names('/', param='/'), ('gripper_tcp', '/gripper_hw', ''))


if __name__ == '__main__':
    unittest.main()
