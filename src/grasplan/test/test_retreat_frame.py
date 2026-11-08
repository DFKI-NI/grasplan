'''The lift after a grasp goes up in the robot's own odom frame: robot 2 of a multi-robot sim lifts in mobipick2/odom,
not in the first robot's frame; a frame set in the config wins. Fake parameter server (no ROS master); needs the
Mobipick image for the module imports.'''
import unittest
from unittest import mock

import grasplan.grasp_planning_core as core

NO_DEFAULT = object()
PARAMS = {
    '~gripper_joint_names': ['gripper_finger_joint'],
    '~gripper_close': [0.7],
    '~gripper_open': [0.0],
    '~gripper_joint_efforts': [1.0],
    '~pre_grasp_approach/min_dist': 0.08,
    '~pre_grasp_approach/desired': 0.25,
    '~pre_grasp_approach/axis': [1.0, 0.0, 0.0],
    '~post_grasp_retreat/min_dist': 0.1,
    '~post_grasp_retreat/desired': 0.15,
    '~post_grasp_retreat/axis': [0.0, 0.0, 1.0],
}


def planner(namespace, extra=None):
    params = dict(PARAMS, **(extra or {}))

    def get_param(name, default=NO_DEFAULT):
        if name in params:
            return params[name]
        if default is NO_DEFAULT:
            raise KeyError(name)
        return default

    with mock.patch.object(core.rospy, 'get_param', get_param), mock.patch.object(
        core.rospy, 'get_namespace', lambda: namespace
    ), mock.patch.object(core.rospy, 'Publisher'):
        return core.GraspPlanningCore()


class TestRetreatFrame(unittest.TestCase):
    def test_robot_1_lifts_in_its_odom_frame(self):
        self.assertEqual(planner('/mobipick/').post_grasp_retreat_frame_id, 'mobipick/odom')

    def test_robot_2_lifts_in_its_own_odom_frame(self):
        self.assertEqual(planner('/mobipick2/').post_grasp_retreat_frame_id, 'mobipick2/odom')

    def test_a_node_outside_a_robot_namespace_keeps_the_first_robot(self):
        self.assertEqual(planner('/').post_grasp_retreat_frame_id, 'mobipick/odom')

    def test_a_configured_frame_wins(self):
        p = planner('/mobipick2/', {'~post_grasp_retreat/frame_id': 'mobipick2/base_link'})
        self.assertEqual(p.post_grasp_retreat_frame_id, 'mobipick2/base_link')


if __name__ == '__main__':
    unittest.main()
