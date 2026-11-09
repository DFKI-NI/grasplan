'''The ~use_path_constraints orientation constraint names the end-effector link and the reach frame of this robot, not
the link hand_ee_link, which no robot here has. Mocks rospy (no ROS master); needs the image for the module imports.'''
import unittest
from types import SimpleNamespace
from unittest import mock

import grasplan.place as place


class TestConstraintFrames(unittest.TestCase):
    def test_names_come_from_the_robot(self):
        node = place.PlaceTools.__new__(place.PlaceTools)
        node.group_name = 'arm'
        node.robot = mock.Mock()
        node.robot.get_group.return_value = SimpleNamespace(get_end_effector_link=lambda: 'mobipick/gripper_tcp')
        node.tf_listener = mock.Mock()
        node.tf_listener.lookupTransform.return_value = ([0, 0, 0], [0, 0, 0, 1])
        with mock.patch.object(place.rospy, 'get_param', lambda name, default=None: default), \
                mock.patch.object(place.rospy.Time, 'now', lambda: 0), \
                mock.patch.object(place.rospy, 'Duration', lambda secs: secs):
            constraint = node.make_constraints_msg().orientation_constraints[0]
        self.assertEqual(constraint.link_name, 'mobipick/gripper_tcp')
        self.assertEqual(constraint.header.frame_id, 'mobipick/ur5_base_link')
        node.tf_listener.lookupTransform.assert_called_once_with('mobipick/ur5_base_link', 'mobipick/gripper_tcp', 0)


if __name__ == '__main__':
    unittest.main()
