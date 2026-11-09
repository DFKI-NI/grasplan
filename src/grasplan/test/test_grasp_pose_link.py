'''~grasp_pose_link: taught grasps that refer to another link of the gripper than the arm's end-effector link are
converted to the end-effector link's pose; ~pre_grasp_approach/frame_id names the frame of the approach axis. Both off
('') the grasps and the approach are the ones built before. A fake tf listener, no ROS master; needs the Mobipick image
for the module imports.'''
import math
import unittest
from unittest import mock

from geometry_msgs.msg import Point, Pose, Quaternion
from tf import transformations as tr

import grasplan.grasp_planning_core as core


class FakeListener:
    '''the grasp link sits 10 cm along z of the end-effector link, turned 90 degrees about its y axis'''

    def __init__(self):
        self.lookups = 0

    def waitForTransform(self, *args):
        pass

    def lookupTransform(self, target, source, time):
        self.lookups += 1
        return (0.0, 0.0, 0.1), tuple(tr.quaternion_about_axis(math.pi / 2, (0, 1, 0)))


def make_core(grasp_pose_link='', approach_frame=''):
    c = core.GraspPlanningCore.__new__(core.GraspPlanningCore)
    c.grasp_pose_link = grasp_pose_link
    c.pre_grasp_approach_frame_id = approach_frame
    c._tf_listener = FakeListener()
    c._grasp_link_offset = {}
    return c


POSE = Pose(position=Point(1.0, 2.0, 3.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))


class TestGraspPoseLink(unittest.TestCase):
    def test_not_configured_the_pose_is_returned_as_it_is(self):
        c = make_core()
        self.assertIs(c.grasp_link_to_end_effector(POSE, 'tool'), POSE)
        self.assertEqual(c._tf_listener.lookups, 0)

    def test_grasp_link_equal_to_the_end_effector_link_is_identity(self):
        self.assertIs(make_core('tool').grasp_link_to_end_effector(POSE, 'tool'), POSE)

    def test_converted_pose_puts_the_grasp_link_at_the_taught_pose(self):
        c = make_core('fingers')
        out = c.grasp_link_to_end_effector(POSE, 'tool')
        t_world_ee = tr.concatenate_matrices(
            tr.translation_matrix([out.position.x, out.position.y, out.position.z]),
            tr.quaternion_matrix([out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w]))
        t_ee_link = tr.concatenate_matrices(tr.translation_matrix((0.0, 0.0, 0.1)),
                                            tr.quaternion_matrix(tr.quaternion_about_axis(math.pi / 2, (0, 1, 0))))
        t_world_link = t_world_ee.dot(t_ee_link)
        self.assertAlmostEqual(abs(t_world_link - tr.translation_matrix((1.0, 2.0, 3.0))).max(), 0.0, places=6)

    def test_the_offset_is_looked_up_once(self):
        c = make_core('fingers')
        c.grasp_link_to_end_effector(POSE, 'tool')
        c.grasp_link_to_end_effector(POSE, 'tool')
        self.assertEqual(c._tf_listener.lookups, 1)

    def test_a_missing_transform_is_a_readable_error(self):
        class NoTransform(FakeListener):
            def waitForTransform(self, *args):
                raise core.tf.Exception('frame does not exist')
        planner = core.GraspPlanningCore.__new__(core.GraspPlanningCore)
        planner.grasp_pose_link = 'grasp_link'
        planner._grasp_link_offset = {}
        planner._tf_listener = NoTransform.__new__(NoTransform)
        with mock.patch.object(core.rospy, 'Time', lambda secs: secs), \
                mock.patch.object(core.rospy, 'Duration', lambda secs: secs):
            with self.assertRaises(RuntimeError) as caught:
                planner.grasp_link_to_end_effector(None, 'ee_link')
        self.assertIn('~grasp_pose_link grasp_link', str(caught.exception))
        self.assertIn('ee_link', str(caught.exception))
        self.assertIn('frame does not exist', str(caught.exception))


class TestApproachFrame(unittest.TestCase):
    def frame(self, approach_frame):
        c = make_core(approach_frame=approach_frame)
        c.grasp_quality, c.max_contact_force = 1.0, 1.0
        c.gripper_joint_names, c.gripper_joint_efforts = ['gripper_joint'], [10.0]
        c.gripper_open_distance, c.gripper_close_distance = [0.0], [0.4]
        c.distance_gripper_open_per_obj = c.distance_gripper_close_per_obj = None
        c.pre_grasp_approach_min_dist, c.pre_grasp_approach_desired, c.pre_grasp_approach_axis = 0.05, 0.1, [0.0, 0.0, 1.0]
        c.post_grasp_retreat_frame_id, c.post_grasp_retreat_min_dist = 'base_link', 0.05
        c.post_grasp_retreat_desired, c.post_grasp_retreat_axis = 0.1, [0.0, 0.0, 1.0]
        with mock.patch.object(core.rospy, 'Duration', lambda secs: secs):
            return c.make_grasp_template('box_1', 'tool').pre_grasp_approach.direction.header.frame_id

    def test_default_is_the_end_effector_link(self):
        self.assertEqual(self.frame(''), 'tool')

    def test_parameter_names_the_frame(self):
        self.assertEqual(self.frame('fingers'), 'fingers')


if __name__ == '__main__':
    unittest.main()
