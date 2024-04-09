#!/usr/bin/python3

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

'''
Load grasp configurations from yaml file and display them on rviz.
'''

import os
import tf
import rospy
import std_msgs
import geometry_msgs
from geometry_msgs.msg import PoseStamped, PoseArray
from visualization_msgs.msg import Marker
from grasplan.grasp_planner.handcoded_grasp_planner import HandcodedGraspPlanner


class GraspVisualizer:
    def __init__(self):
        # parameters
        self.object_name = rospy.get_param('~object_name', 'multimeter')
        self.object_pkg = rospy.get_param('~object_pkg', 'mobipick_gazebo')
        self.global_reference_frame = rospy.get_param('~global_reference_frame', 'object')
        transform_linear_x = rospy.get_param('~transform_linear_x', 0.0)
        transform_linear_y = rospy.get_param('~transform_linear_y', 0.0)
        transform_linear_z = rospy.get_param('~transform_linear_z', 0.0)
        self.transform_linear = [transform_linear_x, transform_linear_y, transform_linear_z]
        transform_angular_roll = rospy.get_param('~transform_angular_roll', 0.0)
        transform_angular_pitch = rospy.get_param('~transform_angular_pitch', 0.0)
        transform_angular_yaw = rospy.get_param('~transform_angular_yaw', 0.0)
        self.transform_angular = [transform_angular_roll, transform_angular_pitch, transform_angular_yaw]
        rospy.Subscriber('~update_object_mesh', std_msgs.msg.String, self.UpdateObjectMeshCB)
        # Publishers
        self.object_mesh_publisher = rospy.Publisher('object_mesh', Marker, queue_size=1, latch=True)
        self.pose_array_pub = rospy.Publisher('~grasp_poses', PoseArray, queue_size=50, latch=True)
        # use helper function to convert gripper poses to pose array
        self.handcoded_grasp_planner_obj = HandcodedGraspPlanner(call_parent_constructor=False)
        rospy.sleep(0.5)
        rospy.loginfo('grasp visualizer node started')

    def UpdateObjectMeshCB(self, msg):
        self.update_mesh(object_name=msg.data, object_pkg=self.object_pkg)

    def make_mesh_marker_msg(self, mesh_path, position=[0, 0, 0], orientation=[0, 0, 0, 1], mesh_scale=[1, 1, 1]):
        mesh_marker_msg = Marker()
        # mesh_marker_msg.lifetime = rospy.Duration(3.0)
        mesh_marker_msg.ns = 'object'
        mesh_marker_msg.header.frame_id = self.global_reference_frame
        mesh_marker_msg.type = Marker.MESH_RESOURCE
        mesh_marker_msg.pose.position.x = position[0]
        mesh_marker_msg.pose.position.y = position[1]
        mesh_marker_msg.pose.position.z = position[2]
        mesh_marker_msg.pose.orientation.x = orientation[0]
        mesh_marker_msg.pose.orientation.y = orientation[1]
        mesh_marker_msg.pose.orientation.z = orientation[2]
        mesh_marker_msg.pose.orientation.w = orientation[3]
        mesh_marker_msg.mesh_use_embedded_materials = True
        mesh_marker_msg.scale = geometry_msgs.msg.Vector3(mesh_scale[0], mesh_scale[1], mesh_scale[2])
        # set rgba to 0 to allow mesh_use_embedded_materials to work
        mesh_marker_msg.color = std_msgs.msg.ColorRGBA(0, 0, 0, 0)
        mesh_marker_msg.mesh_resource = mesh_path
        return mesh_marker_msg

    def publish_grasps_as_pose_array(self):
        object_name = self.object_name
        object_pose = PoseStamped()
        object_pose.header.frame_id = self.global_reference_frame
        object_pose.pose.position.x = 0.0
        object_pose.pose.position.y = 0.0
        object_pose.pose.position.z = 0.0
        object_pose.pose.orientation.x = 0.0
        object_pose.pose.orientation.y = 0.0
        object_pose.pose.orientation.z = 0.0
        object_pose.pose.orientation.w = 1.0
        grasp_type = ''  # not implemented yet, so it can be any value
        pose_array_msg = self.handcoded_grasp_planner_obj.gen_end_effector_grasp_poses(
            object_name, object_pose, grasp_type
        )
        self.pose_array_pub.publish(pose_array_msg)

    def update_mesh(self, object_name='multimeter', object_pkg='mobipick_gazebo'):
        mesh_accepted_formats = ['.dae', '.obj']
        mesh_path = None
        for mesh_accepted_format in mesh_accepted_formats:
            mesh_path = f'package://{object_pkg}/meshes/{object_name}/{object_name}{mesh_accepted_format}'
            if os.path.exists(mesh_path):
                continue
        if mesh_path is None:
            rospy.logerr('failed to update mesh')
            return
        angular_q = tf.transformations.quaternion_from_euler(
            self.transform_angular[0], self.transform_angular[1], self.transform_angular[2]
        )
        marker_msg = self.make_mesh_marker_msg(mesh_path, position=self.transform_linear, orientation=angular_q)
        rospy.loginfo(f'publishing mesh:{mesh_path}')
        self.object_mesh_publisher.publish(marker_msg)

    def start_grasp_visualizer(self):
        self.update_mesh(object_name=self.object_name, object_pkg=self.object_pkg)
        # visualize grasps
        self.publish_grasps_as_pose_array()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('grasp_visualizer', anonymous=False)
    grasp_visualizer = GraspVisualizer()
    grasp_visualizer.start_grasp_visualizer()
