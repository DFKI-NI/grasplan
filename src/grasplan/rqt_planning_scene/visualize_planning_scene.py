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

'''
reads from yaml a list of boxes and publishes them as a msgs/MarkerArray to the grasplan_planning_scene topic
'''

import rospy
import rospkg
import tf

import os
import sys
import pathlib
import yaml
import copy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped


class PlanningSceneVizSettings:
    # default settings
    text_on = True
    ignore_set = set()
    ignore_all_but = []
    topic = 'grasplan_planning_scene'
    colors = {}
    marker_ns = topic
    transparency = 1.0
    yaml_path_to_read = ''
    yaml_path_to_write = ''
    publish_tf = True


class PlanningSceneViz:
    def __init__(self, settings, load_boxes_from_yaml=True):
        self.settings = settings
        self.marker_array_pub = rospy.Publisher(self.settings.topic, MarkerArray, queue_size=1, latch=True)
        self.reset(load_boxes_from_yaml)
        self.br = None
        if self.settings.publish_tf:
            rospy.loginfo('tf boxes broadcast is enabled')
            self.br = tf.TransformBroadcaster()
            self.listener = tf.TransformListener()
        rospy.loginfo('visualize planning scene node initialized')

    def reset(self, load_boxes_from_yaml=True):
        if not self.validate_settings(self.settings):
            rospy.logerr('invalid settings, terminating node')
            rospy.signal_shutdown('invalid settings, shutting down...')
        self.box_list_dictionary = {}

        self.marker_id_count = 0
        self.all_boxes_names = []

        if load_boxes_from_yaml:
            self.load_boxes_from_yaml(self.settings.yaml_path_to_read)

        self.wait_for_subscribers()

        # delete old data if any
        self.delete_all_markers()

    def validate_settings(self, settings):
        if settings.transparency > 1.0 or settings.transparency < 0.0:
            rospy.logerr('transparency setting has to be between 0.0 and 1.0')
            return False
        return True

    def update_settings(self, settings):
        self.settings = settings

    def get_box_values(self, scene_name):
        for box in self.box_list_dictionary:
            if box['scene_name'] == scene_name:
                return box

    def reset_scene_name(self, scene_name):
        for box in self.box_list_dictionary:
            if box['scene_name'] == scene_name:
                for box_bkp in self.box_list_dictionary_bkp:
                    if box_bkp['scene_name'] == scene_name:
                        self.modify_box(
                            scene_name,
                            modify_box_position_x=True,
                            box_position_x=box_bkp['box_position_x'],
                            modify_box_position_y=True,
                            box_position_y=box_bkp['box_position_y'],
                            modify_box_position_z=True,
                            box_position_z=box_bkp['box_position_z'],
                            modify_box_orientation_x=True,
                            box_orientation_x=box_bkp['box_orientation_x'],
                            modify_box_orientation_y=True,
                            box_orientation_y=box_bkp['box_orientation_y'],
                            modify_box_orientation_z=True,
                            box_orientation_z=box_bkp['box_orientation_z'],
                            modify_box_orientation_w=True,
                            box_orientation_w=box_bkp['box_orientation_w'],
                            modify_box_x_dimension=True,
                            box_x_dimension=box_bkp['box_x_dimension'],
                            modify_box_y_dimension=True,
                            box_y_dimension=box_bkp['box_y_dimension'],
                            modify_box_z_dimension=True,
                            box_z_dimension=box_bkp['box_z_dimension'],
                        )
                        return
        rospy.logerr(f'could not found scene name: {scene_name}')

    def modify_box(
        self,
        scene_name,
        modify_box_position_x=False,
        box_position_x=0.0,
        modify_box_position_y=False,
        box_position_y=0.0,
        modify_box_position_z=False,
        box_position_z=0.0,
        modify_box_orientation_x=False,
        box_orientation_x=0.0,
        modify_box_orientation_y=False,
        box_orientation_y=0.0,
        modify_box_orientation_z=False,
        box_orientation_z=0.0,
        modify_box_orientation_w=False,
        box_orientation_w=1.0,
        modify_box_x_dimension=False,
        box_x_dimension=1.0,
        modify_box_y_dimension=False,
        box_y_dimension=1.0,
        modify_box_z_dimension=False,
        box_z_dimension=1.0,
        modify_frame_id=False,
        new_frame_id='world',
    ):
        found = False
        for box in self.box_list_dictionary:
            if box['scene_name'] == scene_name:
                found = True
                if modify_box_position_x:
                    box['box_position_x'] = box_position_x
                if modify_box_position_y:
                    box['box_position_y'] = box_position_y
                if modify_box_position_z:
                    box['box_position_z'] = box_position_z
                if modify_box_orientation_x:
                    box['box_orientation_x'] = box_orientation_x
                if modify_box_orientation_y:
                    box['box_orientation_y'] = box_orientation_y
                if modify_box_orientation_z:
                    box['box_orientation_z'] = box_orientation_z
                if modify_box_orientation_w:
                    box['box_orientation_w'] = box_orientation_w
                if modify_box_x_dimension:
                    box['box_x_dimension'] = box_x_dimension
                if modify_box_y_dimension:
                    box['box_y_dimension'] = box_y_dimension
                if modify_box_z_dimension:
                    box['box_z_dimension'] = box_z_dimension
                if modify_frame_id:
                    box['frame_id'] = new_frame_id
        self.publish_boxes()
        if not found:
            rospy.logerr(f'cannot modify box, scene_name : {scene_name} not found')

    def load_boxes_from_yaml(self, yaml_path: str) -> None:

        yaml_path = pathlib.Path(yaml_path)
        if not yaml_path.exists() or not yaml_path.is_file():
            raise FileNotFoundError(f'yaml file does not exist: {yaml_path}')

        with open(yaml_path, 'r') as file:
            yaml_content = file.read()

        self.box_list_dictionary = yaml.safe_load(yaml_content)['planning_scene_boxes']
        self.box_list_dictionary_bkp = copy.deepcopy(self.box_list_dictionary)
        # construct a list of all boxes
        self.all_boxes_names = []
        for box in self.box_list_dictionary:
            self.all_boxes_names.append(box['scene_name'])
        rospy.loginfo(f'loaded {len(self.all_boxes_names)} planning scene boxes')

    def get_all_boxes_names(self):
        return self.all_boxes_names

    def write_boxes_to_yaml(self, yaml_path: str) -> None:
        yaml_path = pathlib.Path(yaml_path)
        if not yaml_path.exists() or not yaml_path.is_file():
            raise FileNotFoundError(f'yaml file does not exist: {yaml_path}')

        with open(yaml_path, 'w') as yaml_file:
            master_dictionary = {'planning_scene_boxes': self.box_list_dictionary}
            yaml.dump(master_dictionary, yaml_file, default_flow_style=False)
            rospy.loginfo(f'writing boxes to yaml file: {yaml_path}')

    def transform_to_frame(self, object_frame, target_frame):
        # get object transform
        input_pose = PoseStamped()
        for box in self.box_list_dictionary:
            if object_frame == box['scene_name']:
                input_pose.header.frame_id = box['frame_id']
                input_pose.pose.position.x = box['box_position_x']
                input_pose.pose.position.y = box['box_position_y']
                input_pose.pose.position.z = box['box_position_z']
                input_pose.pose.orientation.x = box['box_orientation_x']
                input_pose.pose.orientation.y = box['box_orientation_y']
                input_pose.pose.orientation.z = box['box_orientation_z']
                input_pose.pose.orientation.w = box['box_orientation_w']
                # Wait for the transform to be available
                self.listener.waitForTransform(
                    target_frame, input_pose.header.frame_id, rospy.Time(), rospy.Duration(1.0)
                )
                transformed_pose = self.listener.transformPose(target_frame, input_pose)
                return transformed_pose

    def change_boxes_ref_frame(self, target_frame_dic):
        '''
        express the boxes in a custom target reference frame
        e.g.:
        target_frame_dic = {'table_2':'table_3'} # express table_2 in table_3 reference frame
        '''
        for key in target_frame_dic:
            for box in self.box_list_dictionary:
                if key == box['scene_name']:
                    rospy.loginfo(f'expressing {key} in {target_frame_dic[key]} ref frame')
                    # perform the transformation
                    new_pose = self.transform_to_frame(key, target_frame_dic[key])
                    # x = new_pose.pose.position.x
                    # y = new_pose.pose.position.y
                    # z = new_pose.pose.position.z
                    # qx = new_pose.pose.orientation.x
                    # qy = new_pose.pose.orientation.y
                    # qz = new_pose.pose.orientation.z
                    # qw = new_pose.pose.orientation.w
                    # next line is useful for debugging purposes
                    # self.broadcast_tf(x, y, z, qx, qy, qz, qw, target_frame_dic[key], f'{key}_debug')
                    self.modify_box(
                        box['scene_name'],
                        modify_box_position_x=True,
                        box_position_x=round(float(new_pose.pose.position.x), 3),
                        modify_box_position_y=True,
                        box_position_y=round(float(new_pose.pose.position.y), 3),
                        modify_box_position_z=True,
                        box_position_z=round(float(new_pose.pose.position.z), 3),
                        modify_box_orientation_x=True,
                        box_orientation_x=round(float(new_pose.pose.orientation.x), 4),
                        modify_box_orientation_y=True,
                        box_orientation_y=round(float(new_pose.pose.orientation.y), 4),
                        modify_box_orientation_z=True,
                        box_orientation_z=round(float(new_pose.pose.orientation.z), 4),
                        modify_box_orientation_w=True,
                        box_orientation_w=round(float(new_pose.pose.orientation.w), 4),
                        modify_frame_id=True,
                        new_frame_id=new_pose.header.frame_id,
                    )

    def symbolic_to_rgb_color(self, symbolic_color):
        if symbolic_color == 'green':
            return [0.0, 1.0, 0.0]
        if symbolic_color == 'blue':
            return [0.0, 0.0, 1.0]
        if symbolic_color == 'red':
            return [1.0, 0.0, 0.0]
        if symbolic_color == 'purple':
            return [1.0, 0.0, 1.0]
        if symbolic_color == 'orange':
            return [1.0, 0.65, 0.0]
        rospy.logwarn(f'color {symbolic_color} not supported, will use default green color')
        rospy.logwarn('supported colors are: green, blue, red, purple, orange')
        return [0.0, 1.0, 0.0]

    def make_text_marker(self, marker, text):
        '''
        edits an existing marker overwritting the text field
        '''
        marker.type = Marker.TEXT_VIEW_FACING
        marker.id = self.marker_id_count
        self.marker_id_count += 1
        marker.text = text
        marker.pose.position.z += marker.scale.z / 2.0 + 0.1
        marker.scale.z = 0.2  # text size
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        return marker

    def make_marker(
        self,
        scene_name,
        frame_id,
        origin_x,
        origin_y,
        origin_z,
        orientation_x,
        orientation_y,
        orientation_z,
        orientation_w,
        scale_x,
        scale_y,
        scale_z,
    ):
        rgb_color_list = self.symbolic_to_rgb_color('green')  # default
        if scene_name in self.settings.colors:
            rgb_color_list = self.symbolic_to_rgb_color(self.settings.colors[scene_name])
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = self.settings.marker_ns
        marker.id = self.marker_id_count
        self.marker_id_count += 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = origin_x
        marker.pose.position.y = origin_y
        marker.pose.position.z = origin_z
        marker.pose.orientation.x = orientation_x
        marker.pose.orientation.y = orientation_y
        marker.pose.orientation.z = orientation_z
        marker.pose.orientation.w = orientation_w
        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z
        marker.color.a = self.settings.transparency
        marker.color.r = rgb_color_list[0]
        marker.color.g = rgb_color_list[1]
        marker.color.b = rgb_color_list[2]
        return marker

    def delete_all_markers(self):
        self.marker_id_count = 0
        marker_array_msg = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.ns = self.settings.marker_ns
        marker.action = Marker.DELETEALL
        marker_array_msg.markers.append(marker)
        self.marker_array_pub.publish(marker_array_msg)
        rospy.sleep(0.01)

    def wait_for_subscribers(self):
        # check if there are subscribers, otherwise wait
        rate = rospy.Rate(5)
        count = 0
        while not rospy.is_shutdown():
            if self.marker_array_pub.get_num_connections() > 0:
                rospy.loginfo('publisher is registered and has subscribers.')
                break
            else:
                rospy.logdebug(f'waiting for subscribers to connect... attempt={count}')
                count += 1
            rate.sleep()
        rospy.loginfo('subscriber is detected, continue')

    def broadcast_tf(self, x, y, z, qx, qy, qz, qw, parent_frame_id, child_frame_id):
        if self.br:
            self.br.sendTransform((x, y, z), (qx, qy, qz, qw), rospy.Time.now(), child_frame_id, parent_frame_id)
        else:
            rospy.logerr('cannot broadcast to tf')

    def publish_tf(self):
        '''
        overrides self.settings.publish_tf
        does not take into account ignore_set
        '''
        for box in self.box_list_dictionary:
            self.broadcast_tf(
                box['box_position_x'],
                box['box_position_y'],
                box['box_position_z'],
                box['box_orientation_x'],
                box['box_orientation_y'],
                box['box_orientation_z'],
                box['box_orientation_w'],
                box['frame_id'],
                box['scene_name'],
            )

    def publish_boxes(self):
        self.delete_all_markers()
        marker_array_msg = MarkerArray()

        ignore_set = self.settings.ignore_set
        if self.settings.ignore_all_but != []:
            # automatically build ignore set
            # ignore_set = all_boxes - self.settings.ignore_all_but
            ignore_set = set([item for item in self.all_boxes_names if item not in self.settings.ignore_all_but])

        for box in self.box_list_dictionary:
            if box['scene_name'] in ignore_set:
                continue
            marker = self.make_marker(
                box['scene_name'],
                box['frame_id'],
                box['box_position_x'],
                box['box_position_y'],
                box['box_position_z'],
                box['box_orientation_x'],
                box['box_orientation_y'],
                box['box_orientation_z'],
                box['box_orientation_w'],
                box['box_x_dimension'],
                box['box_y_dimension'],
                box['box_z_dimension'],
            )
            marker_array_msg.markers.append(marker)
            if self.settings.text_on:
                text_marker = self.make_text_marker(copy.deepcopy(marker), box['scene_name'])
                marker_array_msg.markers.append(text_marker)
            if self.settings.publish_tf:
                self.broadcast_tf(
                    box['box_position_x'],
                    box['box_position_y'],
                    box['box_position_z'],
                    box['box_orientation_x'],
                    box['box_orientation_y'],
                    box['box_orientation_z'],
                    box['box_orientation_w'],
                    box['frame_id'],
                    box['scene_name'],
                )

        self.marker_array_pub.publish(marker_array_msg)


if __name__ == '__main__':
    rospy.init_node('planning_scene_publisher', anonymous=False)
    if len(sys.argv) > 1:
        yaml_path = sys.argv[1]
    else:
        package_path = rospkg.RosPack().get_path('grasplan')
        yaml_path = package_path + '/config/examples/planning_scene.yaml'
    if os.path.exists(yaml_path):
        # yaml file path exists
        settings = PlanningSceneVizSettings()
        # a list of planning scene boxes to ignore (not draw)
        settings.ignore_set = set(rospy.get_param('~ignore_set', []))
        # has precedence over ignore_set, nothing else but boxes on this list will be draw
        settings.ignore_all_but = rospy.get_param('~ignore_all_but', [])
        settings.colors = rospy.get_param('~colors', {})
        settings.transparency = rospy.get_param('~transparency', 0.8)
        settings.yaml_path_to_read = yaml_path
        settings.yaml_path_to_write = rospkg.RosPack().get_path('grasplan') + '/config/output_planning_scene.yaml'
        psv = PlanningSceneViz(settings)
        psv.publish_boxes()
        rospy.sleep(0.2)
    else:
        print('yaml file does not exist:', yaml_path)
