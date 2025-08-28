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
example on how to pick an object using grasplan and moveit
'''

import sys
import copy
import importlib
import traceback
import yaml
import numpy as np
from typing import List

import rclpy
import rclcpp
from rclpy.node import Node
from rclpy.action import ActionServer
from moveit.task_constructor import core, stages
from moveit.core.robot_model import RobotModel
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
import tf_transformations
from std_msgs.msg import String, Header
from std_srvs.srv import Empty, SetBool
#from pose_selector.srv import ClassQuery, PoseDelete, GetPoses
from object_pose_msgs.msg import ObjectPose, ObjectList
from geometry_msgs.msg import PoseStamped, Pose, Vector3Stamped, Vector3
#from grasplan.tools.moveit_errors import print_moveit_error
#from moveit_msgs.msg import MoveItErrorCodes, PickupAction, PickupGoal
from grasplan_msgs.action import PickObject
from grasplan_core.tools.common import objectToPick
#from grasplan.tools.action_client_helper import ActionClientHelper
from visualization_msgs.msg import Marker, MarkerArray


class PickTools(Node):
    def __init__(self):
        super().__init__('pick_object_node')
        self.logger = self.get_logger()
        self.logger.info('Initializing pick object node...')

        rclcpp.init()
        self.node = rclcpp.Node("mtc_pick_object_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('global_reference_frame', 'map'),
                ('detach_all_objects', False),
                ('arm_group_name', 'arm'),
                ('gripper_group_name', 'gripper'),
                ('arm_goal_tolerance', 0.01),
                ('planning_time', 20.0),
                ('planning_scene_config_file', ''),
                ('pregrasp_posture_required', False),
                ('pregrasp_posture', 'home'),
                ('clear_planning_scene', True),
                ('clear_octomap', False),
                ('poses_to_go_before_pick', []),
                ('list_of_disentangle_objects', []),
                ('perceive_object', True),
                ('arm_pose_with_objs_in_fov', 'observe100cm_right'),
                ('import_file', 'grasp_planner.simple_pregrasp_planner'),
                ('import_class', 'SimpleGraspPlanner'),
            ]
        )

        (self.global_reference_frame,
         self.detach_all_objects_flag,
         self.arm_group_name,
         gripper_group_name,
         arm_goal_tolerance,
         self.planning_time,
         planning_scene_config_file,
         self.pregrasp_posture_required,
         self.pregrasp_posture,
         self.clear_planning_scene,
         self.clear_octomap_flag,
         self.poses_to_go_before_pick,
         self.list_of_disentangle_objects,
         self.perceive_object,
         self.arm_pose_with_objs_in_fov,
         import_file,
         import_class,
         ) = self.get_parameters([
                    'global_reference_frame',
                    'detach_all_objects',
                    'arm_group_name',
                    'gripper_group_name',
                    'arm_goal_tolerance',
                    'planning_time',
                    'planning_scene_config_file',
                    'pregrasp_posture_required',
                    'pregrasp_posture',
                    'clear_planning_scene',
                    'clear_octomap',
                    'poses_to_go_before_pick',
                    'list_of_disentangle_objects',
                    'perceive_object',
                    'arm_pose_with_objs_in_fov',
                    'import_file',
                    'import_class',
                    ])
        # TODO: include octomap

        # load planning scene boxes from yaml file
        with open(planning_scene_config_file.value) as f:
            try:
                planning_scene_config_content = yaml.safe_load(f)
                self.planning_scene_boxes = planning_scene_config_content.get('planning_scene_boxes', [])
                planning_scene_topic = planning_scene_config_content.get('planning_scene_topic', '/planning_scene')
            except yaml.YAMLError as exc:
                self.logger.error(exc)

        self.planning_scene_pub = self.create_publisher(PlanningScene, planning_scene_topic, 10)
        

        # to be able to transform PoseStamped later in the code
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # import grasp planner and make object out of it
        #self.grasp_planner = getattr(importlib.import_module(import_file), import_class)()

        # service clients
        # pose_selector_activate_srv_name = rospy.get_param('~pose_selector_activate_srv_name', '/pose_selector_activate')
        # pose_selector_class_query_srv_name = rospy.get_param(
        #     '~pose_selector_class_query_srv_name', '/pose_selector_class_query'
        # )
        # pose_selector_get_all_poses_srv_name = rospy.get_param(
        #     '~pose_selector_get_all_poses_srv_name', '/pose_selector_get_all'
        # )
        # pose_selector_delete_srv_name = rospy.get_param('~pose_selector_delete_srv_name', '/pose_selector_delete')
        # self.logger.info(
        #     f'waiting for pose selector services: {pose_selector_activate_srv_name},'
        #     f' {pose_selector_class_query_srv_name}, {pose_selector_get_all_poses_srv_name},'
        #     f' {pose_selector_delete_srv_name}'
        # )

        # if wait_for_service fails, it will throw a
        # rospy.exceptions.ROSException, and the node will exit (as long as
        # this happens before moveit_commander.roscpp_initialize()).
        # rospy.wait_for_service(pose_selector_activate_srv_name, 30.0)
        # rospy.wait_for_service(pose_selector_class_query_srv_name, 30.0)
        # rospy.wait_for_service(pose_selector_get_all_poses_srv_name, 30.0)
        # rospy.wait_for_service(pose_selector_delete_srv_name, 30.0)
        # self.activate_pose_selector_srv = rospy.ServiceProxy(pose_selector_activate_srv_name, SetBool)
        # self.pose_selector_class_query_srv = rospy.ServiceProxy(pose_selector_class_query_srv_name, ClassQuery)
        # self.pose_selector_get_all_poses_srv = rospy.ServiceProxy(pose_selector_get_all_poses_srv_name, GetPoses)
        # self.pose_selector_delete_srv = rospy.ServiceProxy(pose_selector_delete_srv_name, PoseDelete)
        # self.logger.info('found pose selector services')

        try:
            self.logger.info('waiting for move_group action server')

            
            # with self.planning_scene_monitor.read_write() as scene:
            #     scene.remove_all_collision_objects()

            #self.gripper = getattr(self.robot, gripper_group_name)
            #self.robot.arm.set_planning_time(self.planning_time)
            #self.robot.arm.set_goal_tolerance(arm_goal_tolerance)
            
            self.logger.info('found move_group action server')
        except RuntimeError:
            # moveit_commander.roscpp_initialize overwrites the signal handler,
            # so if a RuntimeError occurs here, we have to manually call
            # signal_shutdown() in order for the node to properly exit.
             self.logger.fatal(
                'grasplan pick server could not connect to Moveit in time, exiting! \n' + traceback.format_exc()
            )
             self.node.destroy_node()
             self.destroy_node()
             rclcpp.shutdown()
             rclpy.shutdown()

        self.add_custom_boxes_to_ps()

        # to publish object pose for debugging purposes
        self.obj_pose_pub = self.create_publisher(PoseStamped, 'obj_pose', 1)

        # publishers
        self.event_out_pub = self.create_publisher(String, 'event_out', 1)
        self.trigger_perception_pub = self.create_publisher(String, '/object_recognition/event_in', 1)
        self.pick_grasps_marker_array_pub = self.create_publisher(MarkerArray, '/gripper', 1)
        self.pose_selector_objects_marker_array_pub = self.create_publisher(
            MarkerArray, '/pose_selector_objects', 1
        )

        # subscribers
        self.grasp_type = 'side_grasp'  # only used for simple_pregrasp_planner at the moment
        self.create_subscription(String, 'grasp_type', self.graspTypeCB, 10)

        # offer action lib server
        self.pick_action_server = ActionServer(
            self,
            PickObject,
            'pick_object',
            self.pick_obj_action_callback
        )

        # prepare fallback option because moveit pickup action server ignores preemption requests
        # create joint controller cancellers for both arm and gripper
        ns = self.get_namespace().strip('/')  # programatically get robot namespace
        #self.action_client_helper = ActionClientHelper(ns, self.pick_action_server, controller_names=['arm', 'gripper'])

        self.logger.info('pick node ready!')

    def pick_obj_action_callback(self, goal):
        result = PickObject.Result()
        # Explicitly check for cancellation at the start
        if goal.is_cancel_requested:
            self.logger.warn("Goal canceled at the start.")
            goal.canceled()
            result.success = False
            return result

        # Process the goal
        success = self.pick_object(
            goal.request.object_name, goal.request.support_surface_name, self.grasp_type, goal.request.ignore_object_list
        )

        if goal.is_cancel_requested:
            self.logger.warn("Goal canceled during processing.")
            goal.canceled()
            result.success = False
            return result

        # Handle the goal result
        if success:
            self.logger.info("Pick goal completed successfully.")
            goal.succeed()
            result.success = True
            return result
        else:
            self.logger.warn("Pick goal failed to complete.")
            goal.abort()
            result.success = False
            return result

    def graspTypeCB(self, msg):
        self.grasp_type = msg.data

    def transform_pose(self, pose, target_reference_frame):
        '''
        transform a pose from any reference frame into the target reference frame
        '''
        try:
            return self.tf_buffer.transform(pose, target_reference_frame, timeout=rclpy.duration.Duration(seconds=3.0))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.logger.error(f'Could not transform pose from {pose.header.frame_id} to {target_reference_frame}: {e}')
            return None
        
    def modify_collision_object(self, object_name: str, frame: str, pose: Pose, dimension: List[float], operation: int):
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimension

        box_object = CollisionObject()
        box_object.header.frame_id = frame
        box_object.id = object_name
        box_object.primitives.append(box)
        box_object.primitive_poses.append(pose)
        box_object.operation = operation

        return box_object

    def make_object_pose_and_add_objs_to_planning_scene(self, object_to_pick, ignore_object_list=[]):
        '''
        ignore_object_list: if an object is inside another one, you can add it to the ignore_object_list and it will
                            not be added to the planning scene, but it will rather be removed from the planning scene
        '''
        assert isinstance(object_to_pick, objectToPick)
        # TODO query pose selector
        # resp = self.pose_selector_class_query_srv(object_to_pick.obj_class)
        # if len(resp.poses) == 0:
        #     self.logger.error(
        #         f'Object of class {object_to_pick.obj_class} was not perceived, therefore its pose is not available and'
        #         ' cannot be picked'
        #     )
        #     return None, None, None
        # at least one object of the same class as the object we want to pick was perceived, continue
        object_to_pick_id = object_to_pick.id
        object_to_pick_pose = None
        object_to_pick_bounding_box = None
        object_found = False
        # TODO query pose selector
        # #########################
        # HARDCODED MULTIMETER POSE
        # #########################
        # resp = self.pose_selector_get_all_poses_srv()
        # create fake resp
        resp = ObjectList()
        multimeter_pose = ObjectPose()
        multimeter_pose.class_id = 'multimeter'
        multimeter_pose.instance_id = 1
        multimeter_pose.pose.position.x = 19.6179596796357
        multimeter_pose.pose.position.y = 14.024590623111768
        multimeter_pose.pose.position.z = 0.7383802814715025
        multimeter_pose.pose.orientation.x = -0.00021102828020717012
        multimeter_pose.pose.orientation.y = 0.013604530772689454
        multimeter_pose.pose.orientation.z = 0.19386154258750168
        multimeter_pose.pose.orientation.w = 0.9809345414017768
        multimeter_pose.size.x = 0.17949999868869781
        multimeter_pose.size.y = 0.08748800307512283
        multimeter_pose.size.z = 0.04206399992108345
        
        resp.objects = [multimeter_pose,]

        if len(resp.objects) > 0:
            for pose_selector_object in resp.objects:
                # object name
                object_name = pose_selector_object.class_id + '_' + str(pose_selector_object.instance_id)
                # object pose
                pose_stamped_msg = PoseStamped()
                pose_stamped_msg.header.frame_id = self.global_reference_frame.value
                pose_stamped_msg.pose.position = pose_selector_object.pose.position
                pose_stamped_msg.pose.orientation = pose_selector_object.pose.orientation
                # bounding box
                object_bounding_box = []
                object_bounding_box.append(pose_selector_object.size.x)
                object_bounding_box.append(pose_selector_object.size.y)
                object_bounding_box.append(pose_selector_object.size.z)
                if object_to_pick.any_obj_id and object_to_pick.obj_class == pose_selector_object.class_id:
                    object_to_pick_pose = copy.deepcopy(pose_stamped_msg)
                    object_to_pick_bounding_box = copy.deepcopy(object_bounding_box)
                    object_to_pick_id = copy.deepcopy(pose_selector_object.instance_id)
                    object_found = True
                    self.logger.info(
                        f'found an instance of the object class you want to pick in pose selector: {object_name}'
                    )
                elif object_to_pick.get_object_class_and_id_as_string() == object_name:
                    object_to_pick_pose = copy.deepcopy(pose_stamped_msg)
                    object_to_pick_bounding_box = copy.deepcopy(object_bounding_box)
                    object_to_pick_id = copy.deepcopy(pose_selector_object.instance_id)
                    object_found = True
                    self.logger.info(f'found specific object to be picked in pose selector: {object_name}')
                # TODO planning scene exceptions
                # if object_name in ignore_object_list:
                #     # check if object is already in the planning scene, if so, remove it
                #     if object_name in self.scene.get_known_object_names():
                #         self.scene.remove_world_object(object_name)
                # else:
                #     self.logger.info(f'adding object {object_name} to planning scene')
                #     # add all perceived objects to planning scene (one at at time)
                #     scene = PlanningScene()
                #     scene.is_diff = True
                #     col_object = self.modify_collision_object(
                #         object_name,
                #         object_to_pick_pose.header.frame_id,
                #         copy.deepcopy(object_to_pick_pose.pose),
                #         copy.deepcopy(object_bounding_box),
                #         CollisionObject.ADD
                #     )
                #     scene.world.collision_objects.append(col_object)
                #     self.planning_scene_pub.publish(scene)
                    

        if not object_found:
            self.logger.error(
                'the specific object you want to pick was not found:'
                f' {object_to_pick.get_object_class_and_id_as_string()}'
            )
            return None, None, None
        return (
            self.transform_pose(object_to_pick_pose, self.global_reference_frame.value),
            object_to_pick_bounding_box,
            object_to_pick_id,
        )

    # def clean_scene(self):
    #     '''
    #     iterate over all object in the scene, delete them from the scene
    #     '''
    #     for item in self.scene.get_known_object_names():
    #         self.scene.remove_world_object(item)

    # def clear_octomap(self, octomap_srv_name='clear_octomap'):
    #     '''
    #     call service to clear octomap
    #     '''
    #     self.logger.warn('Clearing octomap')
    #     rospy.ServiceProxy(octomap_srv_name, Empty)()

    # def move_arm_to_posture(self, arm_posture_name):
    #     '''
    #     use moveit commander to send the arm to a predefined arm configuration
    #     defined in srdf
    #     '''
    #     self.logger.info(f'moving arm to {arm_posture_name}')
    #     self.robot.arm.set_named_target(arm_posture_name)
    #     # attempt to move it 2 times, (sometimes fails with only 1 time)
    #     if not self.robot.arm.go():
    #         self.logger.warn(f'failed to move arm to posture: {arm_posture_name}, will retry one more time in 1 sec')
    #         rospy.sleep(1.0)
    #         self.robot.arm.go()

    # def move_gripper_to_posture(self, gripper_posture_name):
    #     '''
    #     WARNING, THIS FUNCTION SHOULD NOT BE USED!

    #     The gripper configurations in the SRDF are in radians, but the actual
    #     gripper action server (/mobipick/gripper_hw, type
    #     control_msgs/GripperCommand) expects values in meters. This means that
    #     when this function is used to send the gripper to the "opened"
    #     position, it will close and vice versa. Use the GripperCommand ypoint is in collision!action
    #     server directly instead.

    #     use moveit commander to send the gripper to a predefined configuration
    #     defined in srdf
    #     '''
    #     self.robot.gripper.set_named_target(gripper_posture_name)
    #     self.robot.gripper.go()

    # def detach_all_objects(self):
    #     for attached_object in self.scene.get_attached_objects().keys():
    #         self.gripper.detach_object(name=attached_object)

    # def clear_mesh_markers(self, namespace, publisher):
    #     marker_array_msg = MarkerArray()
    #     marker = Marker()
    #     marker.id = 0
    #     marker.ns = namespace
    #     marker.action = Marker.DELETEALL
    #     marker_array_msg.markers.append(marker)
    #     publisher.publish(marker_array_msg)


    def add_custom_boxes_to_ps(self):
        scene = PlanningScene()
        scene.is_diff = True

        for planning_scene_box in self.planning_scene_boxes:
            pose = Pose()
            pose.position.x = planning_scene_box['box_position_x']
            pose.position.y = planning_scene_box['box_position_y']
            pose.position.z = planning_scene_box['box_position_z']
            pose.orientation.x = planning_scene_box['box_orientation_x']
            pose.orientation.y = planning_scene_box['box_orientation_y']
            pose.orientation.z = planning_scene_box['box_orientation_z']
            pose.orientation.w = planning_scene_box['box_orientation_w']

            col_object = self.modify_collision_object(planning_scene_box['scene_name'], 
                                                      planning_scene_box['frame_id'],
                                                      pose,
                                                      [planning_scene_box['box_x_dimension'],
                                                       planning_scene_box['box_y_dimension'],
                                                       planning_scene_box['box_z_dimension']],
                                                      CollisionObject.ADD)

            scene.world.collision_objects.append(col_object)

        self.planning_scene_pub.publish(scene)

    def pick_object(self, object_name_as_string, support_surface_name, grasp_type, ignore_object_list=[]):
        '''
        1) move arm to a position where the attached camera can see the scene (octomap will be populated)
        2) clear octomap
        3) add table and object to be grasped to planning scene
        4) generate a list of grasp configurations
        5) call pick moveit functionality
        '''
        self.logger.info(f'Attempting to pick object: {object_name_as_string}')
        if len(ignore_object_list) > 0:
            self.logger.warn(f'the following objects: {ignore_object_list} will be ignored from the planning scene')

        # if not self.detach_all_objects_flag and len(self.scene.get_attached_objects().keys()) > 0:
        #     self.logger.error('cannot pick object, another object is currently attached to the gripper already')
        #     return False

        object_to_pick = objectToPick(object_name_as_string)

        jointspace = core.JointInterpolationPlanner()
        pipeline_planner = core.PipelinePlanner(self.node)

        mtc_task = core.Task()
        mtc_task.name = f"pick {object_name_as_string}"
        mtc_task.loadRobotModel(self.node)
        mtc_task.add(stages.CurrentState("current state"))

        # # detach (all) object if any from the gripper
        # if self.detach_all_objects_flag:
        #     self.detach_all_objects()

        # # ::::::::: perceive object to be picked (optional, read from parameter server if this is required)

        # if self.perceive_object:
        #     # send arm to a pose where objects are inside fov
        #     self.move_arm_to_posture(self.arm_pose_with_objs_in_fov)
        #     # populate pose selector with pose information
        #     self.activate_pose_selector_srv(True)
        #     # wait until pose selector gets updates
        #     rospy.sleep(4.0)
        #     # deactivate pose selector detections
        #     self.activate_pose_selector_srv(False)

        # # ::::::::: setup planning scene
        # self.logger.info('setup planning scene')

        # # remove all objects from the planning scene if needed
        # if self.clear_planning_scene:
        #     self.logger.warn('Clearing planning scene')
        #     self.clean_scene()

        # # add a list of custom boxes defined by the user to the planning scene
        # self.add_custom_boxes_to_ps(self.planning_scene_boxes)

        # # add all perceived objects of interest to planning scene and return the pose, bb, and id of the
        # # object to be picked
        object_pose, bounding_box, id = self.make_object_pose_and_add_objs_to_planning_scene(
            object_to_pick, ignore_object_list=ignore_object_list
        )

        if object_pose is None:
            return False

        # # this condition is when user only specified object class but no id, then we assign the first available id
        if id is not None:
            object_to_pick.set_id(id)

        #self.obj_pose_pub.publish(object_pose)  # publish object pose for visualization purposes

        # print objects that were added to the planning scene
        #self.logger.info(f'planning scene objects: {self.scene.get_known_object_names()}')

        # # check if user cancelled action
        # if self.pick_action_server.is_preempt_requested():
        #     self.logger.warn(
        #         f'grasplan {self.pick_action_server.action_server.ns} ' 'action server goal cancel request received'
        #     )
        #     return False

        # open gripper
        open_gripper_stage = stages.MoveTo("open gripper", jointspace)
        open_gripper_stage.group = "gripper"
        open_gripper_stage.setGoal("open")
        mtc_task.add(open_gripper_stage)

        # add object to planning scene
        col_object = self.modify_collision_object(object_name_as_string,
                                                  object_pose.header.frame_id,
                                                  object_pose.pose,
                                                  bounding_box,
                                                  CollisionObject.ADD)

        self.logger.info(f"Robot Model: {mtc_task.getRobotModel()}")

        modifyPlanningScene = stages.ModifyPlanningScene(f"add {object_name_as_string} to planning scene")
        modifyPlanningScene.addObject(col_object)
        modifyPlanningScene.allowCollisions([object_name_as_string,],
                                            ["gripper_left_robotiq_fingertip_65mm", "gripper_right_robotiq_fingertip_65mm", support_surface_name],
                                            True)
        mtc_task.add(modifyPlanningScene)

        # ::::::::: pick
        self.logger.info('picking object now')

        # generate a list of moveit grasp messages, poses are also published for visualization purposes
        # grasps = self.grasp_planner.make_grasps_msgs(
        #     object_to_pick.get_object_class_and_id_as_string(),
        #     object_pose,
        #     self.robot.arm.get_end_effector_link(),
        #     grasp_type,
        # )

        # disentangle poses
        disentangle_stage_1 = stages.MoveTo("disentangle pose 1", jointspace)
        disentangle_stage_1.group = "arm"
        disentangle_stage_1.setGoal("untangle_cable_guide_1_right")
        mtc_task.add(disentangle_stage_1)

        disentangle_stage_2 = stages.MoveTo("disentangle pose 2", jointspace)
        disentangle_stage_2.group = "arm"
        disentangle_stage_2.setGoal("untangle_cable_guide_2_right")
        mtc_task.add(disentangle_stage_2)

        alternatives = core.Alternatives("Alternatives")

        with open("src/mobipick/mobipick_pick_n_place/config/grasplan/object_grasps/handcoded_grasp_planner_multimeter.yaml") as f:
            try:
                multimeter_grasp_poses = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                self.logger.error(exc)

        rot = []
        rot.append(object_pose.pose.orientation.x)
        rot.append(object_pose.pose.orientation.y)
        rot.append(object_pose.pose.orientation.z)
        rot.append(object_pose.pose.orientation.w)
        euler_rot = tf_transformations.euler_from_quaternion(rot)
        tf_object_to_world = tf_transformations.euler_matrix(euler_rot[0], euler_rot[1], euler_rot[2])
        tf_object_to_world[0][3] = object_pose.pose.position.x  # x
        tf_object_to_world[1][3] = object_pose.pose.position.y  # y
        tf_object_to_world[2][3] = object_pose.pose.position.z  # z

        tf_pose = PoseStamped()
        pose_array = []

        # transform all poses from object reference frame to world reference frame
        if object_to_pick.obj_class not in multimeter_grasp_poses:
            self.logger.error(
                f'object "{object_to_pick.obj_class}" not found in dictionary, have you included in'
                ' handcoded_grasp_planner_transforms parameter?'
            )
            return pose_array

        i = 1
        for transform in multimeter_grasp_poses[object_to_pick.obj_class]['grasp_poses']:
            # tf gripper to object
            rot = transform['rotation']
            euler_rot = tf_transformations.euler_from_quaternion(rot)
            tf_gripper_to_object = tf_transformations.euler_matrix(euler_rot[0], euler_rot[1], euler_rot[2])
            tf_gripper_to_object[0][3] = transform['translation'][0]  # x
            tf_gripper_to_object[1][3] = transform['translation'][1]  # y
            tf_gripper_to_object[2][3] = transform['translation'][2]  # z

            # convert rotation matrix to position and quaternion orientation
            gripper_pose_wrt_world = np.dot(tf_object_to_world, tf_gripper_to_object)
            proll, ppitch, pyaw = tf_transformations.euler_from_matrix(gripper_pose_wrt_world)
            position = (gripper_pose_wrt_world[0][3], gripper_pose_wrt_world[1][3], gripper_pose_wrt_world[2][3])
            q_orientation = tf_transformations.quaternion_from_euler(proll, ppitch, pyaw)

            # pack elements into pose
            tf_pose.pose.position.x = position[0]
            tf_pose.pose.position.y = position[1]
            tf_pose.pose.position.z = position[2]
            tf_pose.pose.orientation.x = q_orientation[0]
            tf_pose.pose.orientation.y = q_orientation[1]
            tf_pose.pose.orientation.z = q_orientation[2]
            tf_pose.pose.orientation.w = q_orientation[3]
            tf_pose.header.frame_id = object_pose.header.frame_id
            tf_pose.header.stamp = rclpy.time.Time().to_msg()

            
            move = stages.MoveTo(f'move to pick {object_name_as_string}, pose {i}', pipeline_planner)
            move.group = "arm"
            move.setGoal(tf_pose)
            alternatives.insert(move)
            i += 1
        # Add the alternatives stage to the task hierarchy

        mtc_task.add(alternatives)

        # close gripper
        close_gripper_stage = stages.MoveTo("close gripper", jointspace)
        close_gripper_stage.group = "gripper"
        #close_gripper_stage.setGoal("closed")
        close_gripper_stage.setGoal({"gripper_finger_joint": 0.4})
        mtc_task.add(close_gripper_stage)

        attach_object = stages.ModifyPlanningScene(f"attach {object_name_as_string} to gripper")
        attach_object.attachObject(object_name_as_string, "gripper_tcp")
        #mtc_task.add(attach_object)

        # move up
        cartesian = core.CartesianPath()
        move = stages.MoveRelative("z +0.2", cartesian)
        move.group = "arm"
        header = Header(frame_id="map")
        move.setDirection(Vector3Stamped(header=header, vector=Vector3(x=0.0, y=0.0, z=0.2)))
        mtc_task.add(move)

        # # clear octomap from the planning scene if needed
        # if self.clear_octomap_flag:
        #     self.clear_octomap()

        # # go to intermediate arm poses if needed to disentangle arm cable
        # if object_to_pick.obj_class in self.list_of_disentangle_objects:
        #     for arm_pose in self.poses_to_go_before_pick:
        #         self.logger.info(f'going to intermediate arm pose {arm_pose} to disentangle cable')
        #         self.move_arm_to_posture(arm_pose)

        # # check if user cancelled action
        # if self.pick_action_server.is_preempt_requested():
        #     self.logger.warn(
        #         f'grasplan {self.pick_action_server.action_server.ns} ' 'action server goal cancel request received'
        #     )
        #     return False

        # # try to pick object with moveit
        # # result = self._pick_with_moveit_commander(object_to_pick, grasps, support_surface_name)
        # result = self._pick_with_action(object_to_pick, grasps, support_surface_name)
        # # handle moveit pick result
        # if result == MoveItErrorCodes.SUCCESS:
        #     # remove picked object pose from pose selector
        #     self.logger.info(
        #         'removing picked object from pose selector due to succesfull execution (as reported by moveit)'
        #     )
        #     self.pose_selector_delete_srv(class_id=object_to_pick.obj_class, instance_id=object_to_pick.id)
        #     self.logger.info(f'Successfully picked object : {object_to_pick.get_object_class_and_id_as_string()}')
        #     # clear possible grasps shown as mesh in rviz
        #     self.clear_mesh_markers(namespace='grasp_poses', publisher=self.pick_grasps_marker_array_pub)
        #     self.clear_mesh_markers(namespace='object', publisher=self.pose_selector_objects_marker_array_pub)
        #     return True
        # else:
        #     self.logger.error('grasp failed')
        #     if result:  # if result is None it means moveit action server was not found within 2 secs
        #         print_moveit_error(result)  # only print moveit error if result is different than None
        if mtc_task.plan():
            mtc_task.publish(mtc_task.solutions[0])
            mtc_task.execute(mtc_task.solutions[0])
            return True
        #import time
        #time.sleep(100)
        return False

    # def _pick_with_moveit_commander(self, object_to_pick, grasps, support_surface_name):
    #     self.robot.arm.set_support_surface_name(support_surface_name)
    #     result = self.robot.arm.pick(object_to_pick.get_object_class_and_id_as_string(), grasps)
    #     return result

    # def _pick_with_action(self, object_to_pick, grasps, support_surface_name):
    #     """
    #     Picks the object using the action client directly, bypassing the moveit_commander.
    #     This is so we can set the support_surface_name without also setting allow_gripper_support_collision to "true",
    #     otherwise there will be collisions.
    #     """
    #     result = None
    #     PICK_OBJECT_SERVER_NAME = 'pickup'

    #     action_client = actionlib.SimpleActionClient(PICK_OBJECT_SERVER_NAME, PickupAction)
    #     if action_client.wait_for_server(timeout=rospy.Duration.from_sec(2.0)):
    #         self.logger.info(f'found {rospy.resolve_name(PICK_OBJECT_SERVER_NAME)} action server')
    #         goal = PickupGoal()
    #         goal.target_name = object_to_pick.get_object_class_and_id_as_string()
    #         goal.group_name = self.arm_group_name
    #         goal.possible_grasps = grasps
    #         goal.support_surface_name = support_surface_name
    #         goal.allowed_planning_time = self.planning_time
    #         goal.planning_options.planning_scene_diff.is_diff = True
    #         goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    #         goal.planning_options.replan_delay = 2.0

    #         self.logger.info(
    #             f'sending pick {object_to_pick.get_object_class_and_id_as_string()} goal '
    #             f'to {rospy.resolve_name(PICK_OBJECT_SERVER_NAME)} action server'
    #         )
    #         self.logger.info(f'waiting for result from {rospy.resolve_name(PICK_OBJECT_SERVER_NAME)} action server')
    #         self.action_client_helper.send_goal_to_rogue_server_and_wait(goal, action_client, patience_timeout=0.1)
    #         result = action_client.get_result().error_code.val  # get moveit error code
    #     else:
    #         self.logger.error(f'action server {PICK_OBJECT_SERVER_NAME} was not found within allocated time')
    #     return result

    # def start_pick_node(self):
    #     # wait for trigger via topic or action lib
    #     self.logger.info('ready to receive pick requests')
    #     rospy.spin()
    #     # shutdown moveit cpp interface before exit
    #     moveit_commander.roscpp_shutdown()


def main(args=None):
    rclpy.init()
    pick = PickTools()
    rclpy.spin(pick)


if __name__ == '__main__':
    main()
