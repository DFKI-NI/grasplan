'''ActionClientHelper builds the controller cancel topics from the robot namespace; a robot without a namespace has them
at the root. Mocks the canceller (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from unittest import mock

import grasplan.tools.action_client_helper as helper


class TestCancellerTopics(unittest.TestCase):
    def topics(self, ns):
        with mock.patch.object(helper, 'TrajectoryCanceller') as canceller:
            helper.ActionClientHelper(ns, mock.Mock(), controller_names=['arm', 'gripper'])
        return [call.args[0] for call in canceller.call_args_list]

    def test_namespace_gives_namespaced_topics(self):
        self.assertEqual(
            self.topics('mobipick'),
            ['/mobipick/arm_controller/follow_joint_trajectory/', '/mobipick/gripper_controller/follow_joint_trajectory/'],
        )

    def test_slashes_around_the_namespace_are_ignored(self):
        self.assertEqual(self.topics('/mobipick/')[0], '/mobipick/arm_controller/follow_joint_trajectory/')

    def test_empty_namespace_gives_root_topics(self):
        for ns in ('', '/'):
            self.assertEqual(
                self.topics(ns),
                ['/arm_controller/follow_joint_trajectory/', '/gripper_controller/follow_joint_trajectory/'],
            )


if __name__ == '__main__':
    unittest.main()
