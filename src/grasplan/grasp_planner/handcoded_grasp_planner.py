# Copyright (c) 2024 DFKI GmbH
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import copy
import numpy as np

import rospy
import tf

from grasplan.grasp_planning_core import GraspPlanningCore
from geometry_msgs.msg import Pose, PoseArray

from grasplan.tools.common import separate_object_class_from_id


class HandcodedGraspPlanner(GraspPlanningCore):
    '''
    Implement concrete methods out of GraspPlanningCore class
    A handcoded grasp planner:
    1) receive object pose
    2) query from paramters fixed transforms wrt object
    3) sample around it in roll, pitch, yaw angles as needed
    '''

    def __init__(self, call_parent_constructor=True):
        if call_parent_constructor:
            super().__init__()

        # get transforms as a dictionary
        self.grasp_poses = rospy.get_param('~handcoded_grasp_planner_transforms', [])
        # optional second list per object class, same format, appended only with ~extra_grasp_sets (read per call)
        self.extra_grasp_poses = rospy.get_param('~handcoded_grasp_planner_extra_transforms', {})

        rospy.sleep(0.5)  # give some time for publisher to register
        rospy.loginfo('handcoded grasp planner object was created')

    def supports_object(self, object_name):
        object_class = separate_object_class_from_id(object_name)[0]
        return object_class in self.grasp_poses

    def transforms_of(self, object_class, extras=False):
        '''
        The stored grasp transforms of a class, or with extras=True its list in ~handcoded_grasp_planner_extra_transforms
        ([] when it has none). The stored list is what every pick plans with; the extras are offered only by
        extra_grasps_msgs, after the whole pick with the stored grasps failed before the arm moved.
        '''
        if not extras:
            return list(self.grasp_poses[object_class]['grasp_poses'])
        return list(((getattr(self, 'extra_grasp_poses', None) or {}).get(object_class) or {}).get('grasp_poses') or [])

    def extra_grasps_msgs(self, object_name, object_pose, end_effector_frame, grasp_type):
        '''
        ~extra_grasp_sets (careful, default off, read on every call): the moveit grasps of the class's extra list, with
        the ids after the stored ones (grasp_32 ... for 32 stored grasps), so logs and results tell them apart; [] when
        the flag is off or the class has no extras. Extras never make a class supported.
        '''
        object_class = separate_object_class_from_id(object_name)[0]
        if not rospy.get_param('~extra_grasp_sets', False) or object_class not in self.grasp_poses \
                or not self.transforms_of(object_class, extras=True):
            return []
        self._extras = True
        try:
            grasps = self.make_grasps_msgs(object_name, object_pose, end_effector_frame, grasp_type)
        finally:
            self._extras = False
        stored = len(self.transforms_of(object_class))
        for i, grasp in enumerate(grasps):
            grasp.id = f'grasp_{stored + i}'
        return grasps

    def gen_end_effector_grasp_poses(self, object_name, object_pose, grasp_type):
        '''
        receive object pose, generate multiple poses around it
        '''
        # tf object to world
        rot = []
        rot.append(object_pose.pose.orientation.x)
        rot.append(object_pose.pose.orientation.y)
        rot.append(object_pose.pose.orientation.z)
        rot.append(object_pose.pose.orientation.w)
        euler_rot = tf.transformations.euler_from_quaternion(rot)
        tf_object_to_world = tf.transformations.euler_matrix(euler_rot[0], euler_rot[1], euler_rot[2])
        tf_object_to_world[0][3] = object_pose.pose.position.x  # x
        tf_object_to_world[1][3] = object_pose.pose.position.y  # y
        tf_object_to_world[2][3] = object_pose.pose.position.z  # z

        tf_pose = Pose()
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = object_pose.header.frame_id
        pose_array_msg.header.stamp = rospy.Time.now()

        # get object class from anchored object
        object_class = separate_object_class_from_id(object_name)[0]

        # transform all poses from object reference frame to world reference frame
        if object_class not in self.grasp_poses:
            rospy.logerr(
                f'object "{object_class}" not found in dictionary, have you included in'
                ' handcoded_grasp_planner_transforms parameter?'
            )
            return pose_array_msg

        for transform in self.transforms_of(object_class, getattr(self, '_extras', False)):
            # tf gripper to object
            rot = transform['rotation']
            euler_rot = tf.transformations.euler_from_quaternion(rot)
            tf_gripper_to_object = tf.transformations.euler_matrix(euler_rot[0], euler_rot[1], euler_rot[2])
            tf_gripper_to_object[0][3] = transform['translation'][0]  # x
            tf_gripper_to_object[1][3] = transform['translation'][1]  # y
            tf_gripper_to_object[2][3] = transform['translation'][2]  # z

            # convert rotation matrix to position and quaternion orientation
            gripper_pose_wrt_world = np.dot(tf_object_to_world, tf_gripper_to_object)
            proll, ppitch, pyaw = tf.transformations.euler_from_matrix(gripper_pose_wrt_world)
            position = (gripper_pose_wrt_world[0][3], gripper_pose_wrt_world[1][3], gripper_pose_wrt_world[2][3])
            q_orientation = tf.transformations.quaternion_from_euler(proll, ppitch, pyaw)

            # pack elements into pose
            tf_pose.position.x = position[0]
            tf_pose.position.y = position[1]
            tf_pose.position.z = position[2]
            tf_pose.orientation.x = q_orientation[0]
            tf_pose.orientation.y = q_orientation[1]
            tf_pose.orientation.z = q_orientation[2]
            tf_pose.orientation.w = q_orientation[3]

            # append to pose array msg
            pose_array_msg.poses.append(copy.deepcopy(tf_pose))
        return pose_array_msg
