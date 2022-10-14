#!/usr/bin/python3

import numpy as np

import tf
import rospy
import std_msgs

from tf.transformations import quaternion_multiply
from geometry_msgs.msg import PoseStamped, PoseArray, Vector3, Pose
from visualization_msgs.msg import Marker, MarkerArray
from pose_selector.srv import GetPoses

'''
Query poses from pose selector and publish them as markers in rviz for visualisation purposes
'''

class PoseSelectorVisualiser:
    def __init__(self, wait_for_pose_selector_srv=True):
        self.color = rospy.get_param('~object_color_rgba', [0,0,0,0])
        self.object_pkg = rospy.get_param('~object_pkg', 'mobipick_gazebo')
        self.objects_mesh_publisher = rospy.Publisher('pose_selector_objects', MarkerArray, queue_size=1, latch=True)
        self.global_reference_frame = rospy.get_param('~global_reference_frame', 'map')
        pose_selector_get_all_poses_srv_name = rospy.get_param('~pose_selector_get_all_poses_srv_name', '/pose_selector_get_all')
        rospy.loginfo(f'waiting for pose selector service: {pose_selector_get_all_poses_srv_name}')
        if wait_for_pose_selector_srv:
            rospy.wait_for_service(pose_selector_get_all_poses_srv_name, 2.0)
        self.pose_selector_get_all_poses_srv = rospy.ServiceProxy(pose_selector_get_all_poses_srv_name, GetPoses)
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

    def make_box_marker_msg(self, box_pose, box_scale=[1,1,1], id=1, color=[0,0,0,0]):
        box_marker_msg = Marker()
        box_marker_msg.id = id
        box_marker_msg.ns = 'object'
        box_marker_msg.header.frame_id = box_pose.header.frame_id
        box_marker_msg.pose = box_pose.pose
        box_marker_msg.type = Marker.CUBE
        box_marker_msg.scale = Vector3(box_scale[0], box_scale[1], box_scale[2])
        box_marker_msg.color = std_msgs.msg.ColorRGBA(*color)
        return box_marker_msg

    def make_obj_marker_msg(self, object_name, mesh_pose, id=1):
        assert isinstance(object_name, str)
        assert isinstance(mesh_pose, PoseStamped)
        if 'insole' in object_name:
            marker_msg = self.make_box_marker_msg(mesh_pose, box_scale=[0.08, 0.21, 0.03], id=id, color=self.color)
        elif 'bag' in object_name:
            mesh_path = f'package://{self.object_pkg}/models/{object_name}_insole/{object_name}_insole.stl'
            marker_msg = self.make_mesh_marker_msg(mesh_path, mesh_pose, id=id, color=self.color, mesh_scale=[0.8, 0.6, 1.0])
        else:
            mesh_path = f'package://{self.object_pkg}/models/{object_name}/{object_name}.stl'
            marker_msg = self.make_mesh_marker_msg(mesh_path, mesh_pose, id=id, color=self.color)
        return marker_msg

    def update_object_poses(self):
        marker_array_msg = MarkerArray()
        id = 0
        # query pose selector
        resp = self.pose_selector_get_all_poses_srv()
        if len(resp.poses.objects) > 0:
            for obj_pose in resp.poses.objects:
                rospy.logdebug(f'obj found: {obj_pose}')
                pose_stamped_msg = PoseStamped()
                pose_stamped_msg.header.frame_id = self.global_reference_frame
                pose_stamped_msg.pose.position = obj_pose.pose.position
                pose_stamped_msg.pose.orientation = obj_pose.pose.orientation
                marker_array_msg.markers.append(self.make_obj_marker_msg(obj_pose.class_id, pose_stamped_msg, id=id))
                id += 1
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
