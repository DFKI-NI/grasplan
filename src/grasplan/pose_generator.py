#!/usr/bin/env python3

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
import rospy
import tf
import numpy as np

from geometry_msgs.msg import PoseArray, Pose, PoseStamped


class PoseGenerator:
    '''
    Receive a pose and generate multiple ones with different orientations
    '''

    def __init__(self):

        # parameters
        self.spherical_sampling_params = rospy.get_param('~spherical_sampling')

        self.pose_array_pub = rospy.Publisher('~poses', PoseArray, queue_size=50)

        # give some time for pose generator publisher to register in the ros network
        rospy.sleep(0.5)

    def modify_list_start_from_center(self, some_list):
        '''
        modify the list order such that starts from the middle element and then goes one left one right recursively
        example:
        some_list = [5, 3, 1, 2, 4]
        new_list = [1, 2, 3, 4, 5]
        '''
        new_list = []
        middle_index = int(len(some_list) / 2)
        new_list.append(some_list[middle_index])
        for i in range(1, 1 + middle_index):
            if middle_index + i < len(some_list):
                new_list.append(some_list[middle_index + i])
            if middle_index - i < len(some_list):
                new_list.append(some_list[middle_index - i])
        return new_list

    def generate_angles(self, start=0.0, end=2.0, step=0.1):
        '''
        create a list of samples
        example:
        start = 0.0, end = 5.0, step = 1.0
        result = [0.0, 1.0, 2.0 , 3.0, 4.0 , 5.0]
        NOTE: we have added a modifier at the end of this function to alternate the order of the list
        starting from the middle element, so for this example the list will return
        result = [3.0, 4.0, 2.0, 5.0, 1.0, 0.0]
        '''
        my_list = []
        if abs(start - end) < 0.000001:  # if start == end
            return [start]
        assert start < end
        temp_number = start
        my_list.append(start)
        while temp_number < end:
            temp_number += step
            my_list.append(temp_number)
        # remove last element (is slightly greater than the end value)
        my_list.pop()
        my_list.append(end)
        return self.modify_list_start_from_center(my_list)

    def spherical_sampling(self, grasp_type, original_pose, offset_vector):
        # offset to object tf
        tf_offset_to_object = tf.transformations.euler_matrix(0, 0, 0)
        tf_offset_to_object[0][3] = offset_vector[0]  # x
        tf_offset_to_object[1][3] = offset_vector[1]  # y
        tf_offset_to_object[2][3] = offset_vector[2]  # z

        # object to world tf
        rot = []
        rot.append(original_pose.pose.orientation.x)
        rot.append(original_pose.pose.orientation.y)
        rot.append(original_pose.pose.orientation.z)
        rot.append(original_pose.pose.orientation.w)
        euler_rot = tf.transformations.euler_from_quaternion(rot)
        tf_object_to_world = tf.transformations.euler_matrix(euler_rot[0], euler_rot[1], euler_rot[2])
        tf_object_to_world[0][3] = original_pose.pose.position.x  # x
        tf_object_to_world[1][3] = original_pose.pose.position.y  # y
        tf_object_to_world[2][3] = original_pose.pose.position.z  # z

        # prepare pose array msg
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = original_pose.header.frame_id
        pose_array_msg.header.stamp = rospy.Time.now()
        tf_pose = Pose()

        # configure sampling params for a specific object
        roll_step = self.spherical_sampling_params[grasp_type]['roll_step']
        pitch_step = self.spherical_sampling_params[grasp_type]['pitch_step']
        yaw_step = self.spherical_sampling_params[grasp_type]['yaw_step']
        roll_start = self.spherical_sampling_params[grasp_type]['roll_start']
        roll_end = self.spherical_sampling_params[grasp_type]['roll_end']
        pitch_start = self.spherical_sampling_params[grasp_type]['pitch_start']
        pitch_end = self.spherical_sampling_params[grasp_type]['pitch_end']
        yaw_start = self.spherical_sampling_params[grasp_type]['yaw_start']
        yaw_end = self.spherical_sampling_params[grasp_type]['yaw_end']

        # sample multiple rotations
        for roll in self.generate_angles(start=roll_start, end=roll_end, step=roll_step):
            for pitch in self.generate_angles(start=pitch_start, end=pitch_end, step=pitch_step):
                for yaw in self.generate_angles(start=yaw_start, end=yaw_end, step=yaw_step):
                    # apply rotations to identity matrix
                    nm = tf.transformations.euler_matrix(roll, pitch, yaw)  # rpy

                    # transform back to world reference frame
                    processed_pose_in_world_rf_m = np.dot(tf_object_to_world, np.dot(tf_offset_to_object, nm))
                    proll, ppitch, pyaw = tf.transformations.euler_from_matrix(processed_pose_in_world_rf_m)
                    position = (
                        processed_pose_in_world_rf_m[0][3],
                        processed_pose_in_world_rf_m[1][3],
                        processed_pose_in_world_rf_m[2][3],
                    )
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

    def publish_pose_array_msg(self, pose_array_msg):
        self.pose_array_pub.publish(pose_array_msg)


if __name__ == '__main__':
    # example of how to use this node, in practice it is used as a library
    rospy.init_node('pose_generator_node', anonymous=False)
    pose_generator = PoseGenerator()
    # make a random pose
    original_pose = PoseStamped()
    # ----
    original_pose.header.frame_id = 'world'
    some_position = [1.0, 0, 0]
    some_orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    offset_vector = [0, 1.0, 0]
    # ----
    original_pose.pose.position.x = some_position[0]
    original_pose.pose.position.y = some_position[1]
    original_pose.pose.position.z = some_position[2]
    original_pose.pose.orientation.x = some_orientation[0]
    original_pose.pose.orientation.y = some_orientation[1]
    original_pose.pose.orientation.z = some_orientation[2]
    original_pose.pose.orientation.w = some_orientation[3]
    pose_array_msg = pose_generator.spherical_sampling(original_pose, offset_vector)
    pose_generator.publish_pose_array_msg(pose_array_msg)
