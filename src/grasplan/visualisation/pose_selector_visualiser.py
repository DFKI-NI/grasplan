#!/usr/bin/python3

import numpy as np

import tf
import rospy
import std_msgs

from tf.transformations import quaternion_multiply
from geometry_msgs.msg import PoseStamped, PoseArray, Vector3, Pose
from visualization_msgs.msg import Marker, MarkerArray
from pose_selector.srv import ClassQuery

'''
Query poses from pose selector and publish them as markers in rviz for visualisation purposes
'''

class PoseSelectorVisualiser:
    def __init__(self, wait_for_pose_selector_srv=True):
        self.color = rospy.get_param('~object_color_rgba', [0,0,0,0])
        self.objects_of_interest = rospy.get_param('~objects_of_interest', ['multimeter', 'klt', 'power_drill_with_grip', 'screwdriver', 'relay'])
        self.object_pkg = rospy.get_param('~object_pkg', 'mobipick_gazebo')
        self.objects_mesh_publisher = rospy.Publisher('pose_selector_objects', MarkerArray, queue_size=1, latch=True)
        pose_selector_query_srv_name = rospy.get_param('~pose_selector_query_srv_name', '/pose_selector_class_query')
        rospy.loginfo(f'waiting for pose selector service: {pose_selector_query_srv_name}')
        if wait_for_pose_selector_srv:
            rospy.wait_for_service(pose_selector_query_srv_name, 2.0)
        self.pose_selector_class_query_srv = rospy.ServiceProxy(pose_selector_query_srv_name, ClassQuery)
        rospy.loginfo('found pose selector services')
        rospy.sleep(0.5)
        rospy.loginfo('pose selector visualiser node started')

    def make_mesh_marker_msg(self, mesh_path, mesh_pose, mesh_scale=[1,1,1], id=1, color=[0,0,0,0]):
        assert isinstance(mesh_pose, PoseStamped)
        mesh_marker_msg = Marker()
        mesh_marker_msg.id = id
        # mesh_marker_msg.lifetime = rospy.Duration(3.0)
        mesh_marker_msg.ns = 'object'
        mesh_marker_msg.header.frame_id = mesh_pose.header.frame_id
        mesh_marker_msg.pose = mesh_pose.pose
        mesh_marker_msg.type = Marker.MESH_RESOURCE
        mesh_marker_msg.mesh_use_embedded_materials = True
        mesh_marker_msg.scale = Vector3(mesh_scale[0], mesh_scale[1], mesh_scale[2])
        # set rgba to 0 to allow mesh_use_embedded_materials to work
        mesh_marker_msg.color = std_msgs.msg.ColorRGBA(*color)
        mesh_marker_msg.mesh_resource = mesh_path
        return mesh_marker_msg

    def rotate_pose(self, pose_stamped, roll=0.0, pitch=0.0, yaw=0.0):
        q_orig = np.array([pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,\
                           pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w])
        q_rot = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        q_new = quaternion_multiply(q_rot, q_orig)
        pose_stamped.pose.orientation.x = q_new[0]
        pose_stamped.pose.orientation.y = q_new[1]
        pose_stamped.pose.orientation.z = q_new[2]
        pose_stamped.pose.orientation.w = q_new[3]
        return pose_stamped

    def make_obj_marker_msg(self, object_name, mesh_pose, id=1):
        assert isinstance(object_name, str)
        assert isinstance(mesh_pose, PoseStamped)
        if object_name == 'power_drill_with_grip':
            mesh_pose = self.rotate_pose(mesh_pose, roll=3.1415) # HACK
        mesh_path = f'package://{self.object_pkg}/meshes/{object_name}.dae'
        marker_msg = self.make_mesh_marker_msg(mesh_path, mesh_pose, id=id, color=self.color)
        return marker_msg

    def update_object_poses(self):
        marker_array_msg = MarkerArray()
        id = 0
        for object_of_interest in self.objects_of_interest:
            # query pose selector
            resp = self.pose_selector_class_query_srv(object_of_interest)
            if len(resp.poses) > 0:
                for obj_pose in resp.poses:
                    rospy.logdebug(f'obj found: {obj_pose}')
                    pose_stamped_msg = PoseStamped()
                    pose_stamped_msg.header.frame_id = 'map'
                    pose_stamped_msg.pose.position = obj_pose.pose.position
                    pose_stamped_msg.pose.orientation = obj_pose.pose.orientation
                    marker_array_msg.markers.append(self.make_obj_marker_msg(obj_pose.class_id, pose_stamped_msg, id=id))
                    id += 1
            else:
                rospy.logdebug(f'Object of class {object_of_interest} is not in pose selector')
        if len(marker_array_msg.markers) > 0:
            self.objects_mesh_publisher.publish(marker_array_msg)

    def start_pose_selector_visualiser(self):
        while not rospy.is_shutdown():
            self.update_object_poses()
            rospy.sleep(1.0)

if __name__ == '__main__':
    rospy.init_node('grasp_visualiser', anonymous=False)
    grasp_visualiser = PoseSelectorVisualiser(wait_for_pose_selector_srv=True)
    grasp_visualiser.start_pose_selector_visualiser()
