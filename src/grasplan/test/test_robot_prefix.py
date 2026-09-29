"""#226: grasplan's per-robot defaults follow the node namespace (mobipick for a single robot, mobipick2 ...)."""
import unittest
from unittest import mock

from grasplan.tools.common import robot_prefix


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


if __name__ == '__main__':
    unittest.main()
