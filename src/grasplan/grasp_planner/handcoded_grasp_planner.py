#!/usr/bin/env python3

import copy
import numpy as np

import rospy
import tf

from grasplan.grasp_planning_core import GraspPlanningCore
from geometry_msgs.msg import Pose, PoseArray

from grasplan.common_grasp_tools import separate_object_class_from_id

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

        rospy.sleep(0.5) # give some time for publisher to register
        rospy.loginfo('handcoded grasp planner object was created')

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
        tf_object_to_world[0][3] = object_pose.pose.position.x # x
        tf_object_to_world[1][3] = object_pose.pose.position.y # y
        tf_object_to_world[2][3] = object_pose.pose.position.z # z

        tf_pose = Pose()
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = object_pose.header.frame_id
        pose_array_msg.header.stamp = rospy.Time.now()

        # get object class from anchored object
        object_class = separate_object_class_from_id(object_name)[0]

        # transform all poses from object reference frame to world reference frame
        if not object_class in self.grasp_poses:
            rospy.logerr(f'object "{object_class}" not found in dictionary, have you included in handcoded_grasp_planner_transforms parameter?')
            return pose_array_msg

        for transform in self.grasp_poses[object_class]['grasp_poses']:
            # tf gripper to object
            rot = transform['rotation']
            euler_rot = tf.transformations.euler_from_quaternion(rot)
            tf_gripper_to_object = tf.transformations.euler_matrix(euler_rot[0], euler_rot[1], euler_rot[2])
            tf_gripper_to_object[0][3] = transform['translation'][0] # x
            tf_gripper_to_object[1][3] = transform['translation'][1] # y
            tf_gripper_to_object[2][3] = transform['translation'][2] # z

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
