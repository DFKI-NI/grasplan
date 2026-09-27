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
import time
import traceback

import rospy
import actionlib
import moveit_commander

from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from tf import TransformListener
import tf.transformations
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
from pose_selector.srv import ClassQuery, PoseDelete, GetPoses
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from grasplan.tools.moveit_errors import print_moveit_error
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    MoveItErrorCodes,
    PickupAction,
    PickupGoal,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from grasplan.msg import (
    GenerateGraspsAction,
    GenerateGraspsGoal,
    GraspCandidate,
    PickObjectAction,
    PickObjectGoal,
    PickObjectResult,
)
from grasplan.srv import ViewObject
from grasplan.tools.common import objectToPick, connect_move_groups, roscpp_initialize_named
from grasplan.tools.action_client_helper import ActionClientHelper
from grasplan.tools.gripper_envelope import fingertip_envelope, lowest_point_offset
from grasplan.tools.topdown_grasps import clamp_box_to_support, topdown_grasps
from visualization_msgs.msg import Marker, MarkerArray

ACTION_STATES = {value: name for name, value in vars(GoalStatus).items() if name.isupper() and isinstance(value, int)}

# the name MoveIt gives the octomap inside the planning scene (planning_scene::PlanningScene::OCTOMAP_NS)
OCTOMAP_COLLISION_NAME = '<octomap>'


class PickTools:
    def __init__(self):

        # parameters
        self.global_reference_frame = rospy.get_param('~global_reference_frame', 'map')
        self.detach_all_objects_flag = rospy.get_param('~detach_all_objects', False)
        self.arm_group_name = rospy.get_param('~arm_group_name', 'arm')
        gripper_group_name = rospy.get_param('~gripper_group_name', 'gripper')
        self.gripper_group_name = gripper_group_name
        self._fingertip_envelope = None  # lazily, see fingertip_envelope()
        arm_goal_tolerance = rospy.get_param('~arm_goal_tolerance', 0.01)
        self.planning_time = rospy.get_param('~planning_time', 20.0)
        self.pregrasp_posture_required = rospy.get_param('~pregrasp_posture_required', False)
        self.pregrasp_posture = rospy.get_param('~pregrasp_posture', 'home')
        self.planning_scene_boxes = rospy.get_param('~planning_scene_boxes', [])
        # raise the bottom of perceived object boxes that reach into a table of the planning scene (noisy depth) to its
        # top, so the object does not collide with its support, e.g. during the lift (#148)
        self.clamp_object_boxes_to_support = rospy.get_param('~clamp_object_boxes_to_support', False)
        # MoveIt's pickup plans the grasps it is given in parallel and executes the first one that
        # planned, not the best scored one; external (AnyGrasp) candidates arrive best first, so they
        # are offered in batches of this size to keep the score order (0 = all at once)
        self.external_grasp_batch_size = int(rospy.get_param('~external_grasp_batch_size', 3))
        self.clear_planning_scene = rospy.get_param('~clear_planning_scene', True)
        self.clear_octomap_flag = rospy.get_param('~clear_octomap', False)
        # MoveIt's pick pipeline lets the gripper touch the target object and the support
        # surface, but never the octomap, so the voxels measured on the target itself make
        # every grasp fail at the 'approach & translate' stage. These two parameters allow
        # those contacts for the duration of a single pick.
        self.allow_octomap_contact = rospy.get_param('~allow_octomap_contact_during_pick', True)
        self.octomap_touch_links = rospy.get_param('~octomap_touch_links', [])
        self.poses_to_go_before_pick = rospy.get_param('~poses_to_go_before_pick', [])
        self.list_of_disentangle_objects = rospy.get_param('~list_of_disentangle_objects', [])
        # if true the arm is moved to a pose where objects are inside fov and pose selector is triggered
        # to accept obj pose updates
        self.perceive_object = rospy.get_param('~perceive_object', True)
        # the arm pose where the objects are inside the fov (used to move the arm to perceive objs right after)
        self.arm_pose_with_objs_in_fov = rospy.get_param('~arm_pose_with_objs_in_fov', 'observe100cm_right')
        # configure the desired grasp planner to use
        import_file = rospy.get_param('~import_file', 'grasp_planner.simple_pregrasp_planner')
        import_class = rospy.get_param('~import_class', 'SimpleGraspPlanner')
        self.anygrasp_action_name = rospy.get_param('~anygrasp_action_name', '/mobipick/grasp_object')
        self.anygrasp_generate_action_name = rospy.get_param(
            '~anygrasp_generate_action_name', '/mobipick/generate_grasps'
        )
        self.anygrasp_handles_execution = rospy.get_param('~anygrasp_handles_execution', False)
        self.anygrasp_server_timeout = rospy.get_param('~anygrasp_server_timeout', 2.0)
        self.anygrasp_result_timeout = rospy.get_param('~anygrasp_result_timeout', 300.0)
        self.anygrasp_arm_pose = rospy.get_param('~anygrasp_arm_pose', 'anygrasp')
        # a grasplan/ViewObject service that moves the camera to a view of the object that suits AnyGrasp (e.g. the
        # whole object within the depth range); when it is missing or fails the arm goes to anygrasp_arm_pose
        self.anygrasp_view_service = rospy.get_param('~anygrasp_view_service', '/mobipick/grasp_view')
        self.gripper_action_name = rospy.get_param('~gripper_action_name', '/mobipick/gripper_hw')
        self.gripper_action_timeout = rospy.get_param('~gripper_action_timeout', 2.0)
        # MoveIt's pickup/place action servers live in move_group, which on the real robot runs on the
        # robot PC; connecting to it from another machine can take seconds, so the client is created
        # once at startup (a missing server at startup is fatal) and reused for every request
        self.moveit_action_startup_timeout = rospy.get_param('~moveit_action_startup_timeout', 60.0)
        self.moveit_action_server_timeout = rospy.get_param('~moveit_action_server_timeout', 10.0)
        # plan and execute the grasps with MoveIt Task Constructor instead of move_group's pickup action
        self.use_mtc = rospy.get_param('~use_mtc', False)
        # TODO: include octomap

        if self.anygrasp_server_timeout < 0 or self.anygrasp_result_timeout < 0 or self.gripper_action_timeout < 0:
            raise ValueError('Action timeouts must be zero or positive')

        # to be able to transform PoseStamped later in the code
        self.tf_listener = TransformListener()

        # import grasp planner and make object out of it
        self.grasp_planner = getattr(importlib.import_module(import_file), import_class)()
        self.anygrasp_action_client = actionlib.SimpleActionClient(self.anygrasp_action_name, PickObjectAction)
        self.anygrasp_generate_action_client = actionlib.SimpleActionClient(
            self.anygrasp_generate_action_name, GenerateGraspsAction
        )
        self.gripper_action_client = actionlib.SimpleActionClient(self.gripper_action_name, GripperCommandAction)

        # service clients
        pose_selector_activate_srv_name = rospy.get_param('~pose_selector_activate_srv_name', '/pose_selector_activate')
        pose_selector_class_query_srv_name = rospy.get_param(
            '~pose_selector_class_query_srv_name', '/pose_selector_class_query'
        )
        pose_selector_get_all_poses_srv_name = rospy.get_param(
            '~pose_selector_get_all_poses_srv_name', '/pose_selector_get_all'
        )
        pose_selector_delete_srv_name = rospy.get_param('~pose_selector_delete_srv_name', '/pose_selector_delete')
        rospy.loginfo(
            f'waiting for pose selector services: {pose_selector_activate_srv_name},'
            f' {pose_selector_class_query_srv_name}, {pose_selector_get_all_poses_srv_name},'
            f' {pose_selector_delete_srv_name}'
        )
        # if wait_for_service fails, it will throw a
        # rospy.exceptions.ROSException, and the node will exit (as long as
        # this happens before roscpp_initialize_named()).
        rospy.wait_for_service(pose_selector_activate_srv_name, 30.0)
        rospy.wait_for_service(pose_selector_class_query_srv_name, 30.0)
        rospy.wait_for_service(pose_selector_get_all_poses_srv_name, 30.0)
        rospy.wait_for_service(pose_selector_delete_srv_name, 30.0)
        self.activate_pose_selector_srv = rospy.ServiceProxy(pose_selector_activate_srv_name, SetBool)
        self.pose_selector_class_query_srv = rospy.ServiceProxy(pose_selector_class_query_srv_name, ClassQuery)
        self.pose_selector_get_all_poses_srv = rospy.ServiceProxy(pose_selector_get_all_poses_srv_name, GetPoses)
        self.pose_selector_delete_srv = rospy.ServiceProxy(pose_selector_delete_srv_name, PoseDelete)
        rospy.loginfo('found pose selector services')

        try:
            rospy.loginfo('waiting for move_group action server')
            roscpp_initialize_named(sys.argv)
            self.robot = moveit_commander.RobotCommander()
            connect_move_groups(
                self.robot, {'arm', self.arm_group_name, gripper_group_name}, self.moveit_action_startup_timeout
            )
            self.gripper = getattr(self.robot, gripper_group_name)
            self.robot.arm.set_planning_time(self.planning_time)
            self.robot.arm.set_goal_tolerance(arm_goal_tolerance)
            self.scene = moveit_commander.PlanningSceneInterface()
            self.get_planning_scene_srv = rospy.ServiceProxy('get_planning_scene', GetPlanningScene)
            rospy.loginfo('found move_group action server')
        except RuntimeError:
            # roscpp_initialize_named overwrites the signal handler,
            # so if a RuntimeError occurs here, we have to manually call
            # signal_shutdown() in order for the node to properly exit.
            rospy.logfatal(
                'grasplan pick server could not connect to Moveit in time, exiting! \n' + traceback.format_exc()
            )
            rospy.signal_shutdown('fatal error')
            sys.exit(1)

        self.mtc = None
        if self.use_mtc:
            from grasplan.mtc_pick_place import MtcPickPlace

            self.mtc = MtcPickPlace(
                self.arm_group_name,
                self.robot.arm.get_end_effector_link(),
                self.robot.get_link_names(group=gripper_group_name),
                self.robot.get_planning_frame(),
                lambda: self.pick_action_server.is_preempt_requested(),
            )
            rospy.loginfo('pick: grasps are planned and executed with MoveIt Task Constructor')
        self.pickup_action_client = actionlib.SimpleActionClient('pickup', PickupAction)
        if not self.use_mtc:
            rospy.loginfo(f'waiting for {rospy.resolve_name("pickup")} action server')
            if not self.pickup_action_client.wait_for_server(rospy.Duration(self.moveit_action_startup_timeout)):
                rospy.logfatal(
                    f'MoveIt action server {rospy.resolve_name("pickup")} not found within '
                    f'{self.moveit_action_startup_timeout} s, grasplan pick server exiting!'
                )
                rospy.signal_shutdown('fatal error')
                sys.exit(1)
            rospy.loginfo(f'found {rospy.resolve_name("pickup")} action server')

        self.add_custom_boxes_to_ps(self.planning_scene_boxes)

        # to publish object pose for debugging purposes
        self.obj_pose_pub = rospy.Publisher('~obj_pose', PoseStamped, queue_size=1)

        # publishers
        self.event_out_pub = rospy.Publisher('~event_out', String, queue_size=1)
        self.planning_scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=1)
        self.trigger_perception_pub = rospy.Publisher('/object_recognition/event_in', String, queue_size=1)
        self.pick_grasps_marker_array_pub = rospy.Publisher('/gripper', MarkerArray, queue_size=1)
        self.pose_selector_objects_marker_array_pub = rospy.Publisher(
            '/pose_selector_objects', MarkerArray, queue_size=1
        )

        # subscribers
        self.grasp_type = 'side_grasp'  # only used for simple_pregrasp_planner at the moment
        rospy.Subscriber('~grasp_type', String, self.graspTypeCB)

        # offer action lib server
        self.pick_action_server = actionlib.SimpleActionServer(
            'pick_object', PickObjectAction, self.pick_obj_action_callback, False
        )
        # prepare fallback option because moveit pickup action server ignores preemption requests
        # create joint controller cancellers for both arm and gripper
        ns = rospy.get_namespace().strip('/')  # programatically get robot namespace
        self.action_client_helper = ActionClientHelper(ns, self.pick_action_server, controller_names=['arm', 'gripper'])
        self.pick_action_server.start()

        rospy.loginfo('pick node ready!')

    def pick_obj_action_callback(self, goal):
        # Explicitly check for preemption at the start
        if self.pick_action_server.is_preempt_requested():
            rospy.logwarn("Preemption requested at the start of the goal. Aborting goal...")
            self.pick_action_server.set_preempted()
            return

        if self.mtc is not None:
            self.mtc.failure_reason, self.mtc.executed = '', False  # nothing left over from the previous goal

        # Unknown objects are sent directly to the open-set pipeline. A known
        # object's result is final, including a failed Grasplan attempt.
        if self.grasp_planner.supports_object(goal.object_name):
            success = self.pick_object(
                goal.object_name, goal.support_surface_name, self.grasp_type, goal.ignore_object_list
            )
            status_text = 'Grasplan pick succeeded' if success else 'Grasplan pick failed'
            if not success and self.mtc is not None and self.mtc.failure_reason:
                status_text += f': {self.mtc.failure_reason}'
            preempted = False
        else:
            if self.anygrasp_handles_execution:
                success, status_text, preempted = self.pick_unknown_object_with_anygrasp(goal)
            else:
                success, status_text, preempted = self.pick_unknown_object_with_grasplan(goal)

        if preempted:
            rospy.logwarn(status_text)
            self.pick_action_server.set_preempted(PickObjectResult(success=False), status_text)
            return

        if self.pick_action_server.is_preempt_requested():
            rospy.logwarn("Preemption requested during pick goal processing.")
            self.pick_action_server.set_preempted()
            return

        # Handle the goal result
        if success:
            rospy.loginfo("Pick goal completed successfully.")
            self.pick_action_server.set_succeeded(PickObjectResult(success=True), status_text)
        else:
            rospy.logwarn("Pick goal failed to complete: %s", status_text)
            self.pick_action_server.set_aborted(PickObjectResult(success=False), status_text)

    def pick_unknown_object_with_anygrasp(self, goal):
        object_name = goal.object_name
        rospy.loginfo(
            'object %r is not known by %s; trying AnyGrasp action server %s',
            object_name,
            type(self.grasp_planner).__name__,
            rospy.resolve_name(self.anygrasp_action_name),
        )

        grasplan_action_name = rospy.resolve_name('pick_object')
        anygrasp_action_name = rospy.resolve_name(self.anygrasp_action_name)
        if anygrasp_action_name == grasplan_action_name:
            message = f'AnyGrasp fallback action {anygrasp_action_name} resolves to the Grasplan action itself'
            rospy.logerr(message)
            return False, message, False

        if not self.anygrasp_action_client.wait_for_server(rospy.Duration(self.anygrasp_server_timeout)):
            message = (
                f'object {object_name!r} is not known by Grasplan and AnyGrasp action server '
                f'{anygrasp_action_name} is unavailable'
            )
            rospy.logerr(message)
            return False, message, False

        if not self.move_to_anygrasp_view(object_name):
            message = f'failed to move arm to {self.anygrasp_arm_pose!r} before calling AnyGrasp'
            rospy.logerr(message)
            return False, message, False

        anygrasp_goal = PickObjectGoal()
        anygrasp_goal.object_name = object_name
        anygrasp_goal.support_surface_name = goal.support_surface_name
        anygrasp_goal.ignore_object_list = list(goal.ignore_object_list)
        self.anygrasp_action_client.send_goal(anygrasp_goal)

        deadline = None
        if self.anygrasp_result_timeout > 0:
            deadline = time.monotonic() + self.anygrasp_result_timeout

        while not rospy.is_shutdown():
            if self.pick_action_server.is_preempt_requested():
                self.anygrasp_action_client.cancel_goal()
                return False, 'AnyGrasp fallback was cancelled', True
            if self.anygrasp_action_client.wait_for_result(rospy.Duration(0.1)):
                break
            if deadline is not None and time.monotonic() >= deadline:
                self.anygrasp_action_client.cancel_goal()
                message = f'timed out waiting for AnyGrasp after {self.anygrasp_result_timeout:.1f} seconds'
                rospy.logerr(message)
                self.open_gripper()
                return False, message, False
        else:
            self.anygrasp_action_client.cancel_goal()
            return False, 'ROS shutdown while waiting for AnyGrasp', True

        state = self.anygrasp_action_client.get_state()
        result = self.anygrasp_action_client.get_result()
        server_text = self.anygrasp_action_client.get_goal_status_text() or 'no status text'
        success = state == GoalStatus.SUCCEEDED and result is not None and result.success
        if success:
            message = f'AnyGrasp succeeded: {server_text}'
            rospy.loginfo(message)
            return True, message, False

        message = (
            f'AnyGrasp failed with action state {ACTION_STATES.get(state, str(state))}, '
            f'result.success={getattr(result, "success", None)}: '
            f'{server_text}'
        )
        rospy.logerr(message)
        self.open_gripper()
        return False, message, False

    def pick_unknown_object_with_grasplan(self, goal):
        object_name = goal.object_name
        action_name = rospy.resolve_name(self.anygrasp_generate_action_name)
        rospy.loginfo(
            'object %r is not known by %s; requesting candidates from %s for Grasplan execution',
            object_name,
            type(self.grasp_planner).__name__,
            action_name,
        )
        if not self.anygrasp_generate_action_client.wait_for_server(rospy.Duration(self.anygrasp_server_timeout)):
            message = f'AnyGrasp generate action server {action_name} is unavailable'
            rospy.logerr(message)
            return False, message, False

        if not self.move_to_anygrasp_view(object_name):
            message = f'failed to move arm to {self.anygrasp_arm_pose!r} before calling AnyGrasp'
            rospy.logerr(message)
            return False, message, False

        generate_goal = GenerateGraspsGoal(object_name=object_name)
        self.anygrasp_generate_action_client.send_goal(generate_goal)
        deadline = None
        if self.anygrasp_result_timeout > 0:
            deadline = time.monotonic() + self.anygrasp_result_timeout

        while not rospy.is_shutdown():
            if self.pick_action_server.is_preempt_requested():
                self.anygrasp_generate_action_client.cancel_goal()
                return False, 'AnyGrasp generation was cancelled', True
            if self.anygrasp_generate_action_client.wait_for_result(rospy.Duration(0.1)):
                break
            if deadline is not None and time.monotonic() >= deadline:
                self.anygrasp_generate_action_client.cancel_goal()
                message = f'timed out waiting for AnyGrasp after {self.anygrasp_result_timeout:.1f} seconds'
                rospy.logerr(message)
                return False, message, False
        else:
            self.anygrasp_generate_action_client.cancel_goal()
            return False, 'ROS shutdown while waiting for AnyGrasp', True

        state = self.anygrasp_generate_action_client.get_state()
        result = self.anygrasp_generate_action_client.get_result()
        server_text = self.anygrasp_generate_action_client.get_goal_status_text() or 'no status text'
        if state != GoalStatus.SUCCEEDED or result is None or not result.success:
            message = (
                f'AnyGrasp generation failed with action state {ACTION_STATES.get(state, str(state))}, '
                f'result.success={getattr(result, "success", None)}: {server_text}'
            )
            rospy.logerr(message)
            return self.pick_topdown_without_anygrasp(goal, message)

        detection = result.detection
        if not detection.object.class_id or not detection.grasps:
            message = 'AnyGrasp returned an incomplete detection'
            rospy.logerr(message)
            return self.pick_topdown_without_anygrasp(goal, message)
        anchored_object_name = f'{detection.object.class_id}_{detection.object.instance_id}'
        success = self.pick_object(
            anchored_object_name,
            goal.support_surface_name,
            self.grasp_type,
            goal.ignore_object_list,
            external_grasp_candidates=detection.grasps,
            external_reference_frame=detection.object.header.frame_id,
            external_object=detection.object,
            perceive_object=False,
        )
        message = (
            f'Grasplan picked open-set object {anchored_object_name}'
            if success
            else f'Grasplan failed to pick open-set object {anchored_object_name}'
        )
        if not success and self.mtc is not None and self.mtc.failure_reason:
            message += f': {self.mtc.failure_reason}'
        return success, message, False

    def pick_topdown_without_anygrasp(self, goal, anygrasp_message):
        '''
        AnyGrasp gave no grasp at all: with ~open_set_topdown_fallback, pick the object with the top-down fallback
        from its pose selector box (the perception committed it there); returns (success, message, preempted)
        '''
        if not rospy.get_param('~open_set_topdown_fallback', False):
            return False, anygrasp_message, False
        rospy.logwarn(f'{anygrasp_message}; trying the top-down fallback from the pose selector box of '
                      f'{goal.object_name}')
        success = self.pick_object(
            goal.object_name,
            goal.support_surface_name,
            self.grasp_type,
            goal.ignore_object_list,
            external_grasp_candidates=[],
            perceive_object=False,
        )
        if success:
            return True, f'Grasplan picked open-set object {goal.object_name} with the top-down fallback', False
        message = f'{anygrasp_message}; the top-down fallback failed too'
        if self.mtc is not None and self.mtc.failure_reason:
            message += f': {self.mtc.failure_reason}'
        return False, message, False

    def graspTypeCB(self, msg):
        self.grasp_type = msg.data

    def transform_pose(self, pose, target_reference_frame):
        '''
        transform a pose from any rerence frame into the target reference frame
        '''
        self.tf_listener.getLatestCommonTime(target_reference_frame, pose.header.frame_id)
        return self.tf_listener.transformPose(target_reference_frame, pose)

    def make_object_pose_and_add_objs_to_planning_scene(
        self, object_to_pick, ignore_object_list=[], external_object=None
    ):
        '''
        ignore_object_list: if an object is inside another one, you can add it to the ignore_object_list and it will
                            not be added to the planning scene, but it will rather be removed from the planning scene
        external_object: grasplan/DetectedObject of the target as perceived by the open-set pipeline that produced
                         the grasp candidates. Its box is added to the planning scene as-is (same perception, same
                         frame, orientation kept) instead of the pose selector copy, so the pick does not depend on
                         the pose selector round trip; every other pose selector object is still added as obstacle.
        '''
        assert isinstance(object_to_pick, objectToPick)
        external_object_name = None
        if external_object is not None:
            external_object_name = f'{external_object.class_id}_{external_object.instance_id}'
        else:
            # query pose selector
            resp = self.pose_selector_class_query_srv(object_to_pick.obj_class)
            if len(resp.poses) == 0:
                rospy.logerr(
                    f'Object of class {object_to_pick.obj_class} was not perceived, therefore its pose is not available'
                    ' and cannot be picked'
                )
                return None, None, None
        # at least one object of the same class as the object we want to pick was perceived, continue
        object_to_pick_id = object_to_pick.id
        object_to_pick_pose = None
        object_to_pick_bounding_box = None
        object_found = False
        # query pose selector
        resp = self.pose_selector_get_all_poses_srv()
        perceived = {f'{o.class_id}_{o.instance_id}' for o in resp.poses.objects} | {external_object_name}
        supports = self.support_boxes(exclude=perceived) if self.clamp_object_boxes_to_support else []
        if len(resp.poses.objects) > 0:
            for pose_selector_object in resp.poses.objects:
                # object name
                object_name = pose_selector_object.class_id + '_' + str(pose_selector_object.instance_id)
                pose_selector_object.instance_id
                # object pose
                pose_stamped_msg = PoseStamped()
                pose_stamped_msg.header.frame_id = self.global_reference_frame
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
                    rospy.loginfo(
                        f'found an instance of the object class you want to pick in pose selector: {object_name}'
                    )
                elif object_to_pick.get_object_class_and_id_as_string() == object_name:
                    object_to_pick_pose = copy.deepcopy(pose_stamped_msg)
                    object_to_pick_bounding_box = copy.deepcopy(object_bounding_box)
                    object_to_pick_id = copy.deepcopy(pose_selector_object.instance_id)
                    object_found = True
                    rospy.loginfo(f'found specific object to be picked in pose selector: {object_name}')
                # planning scene exceptions
                if object_name in ignore_object_list:
                    # check if object is already in the planning scene, if so, remove it
                    if object_name in self.scene.get_known_object_names():
                        self.scene.remove_world_object(object_name)
                elif object_name == external_object_name:
                    rospy.loginfo(f'{object_name} is added from the external detection, skipping pose selector copy')
                else:
                    rospy.loginfo(f'adding object {object_name} to planning scene')
                    # add all perceived objects to planning scene (one at at time)
                    self.add_object_box(object_name, pose_stamped_msg, object_bounding_box, supports)
        if external_object is not None:
            object_to_pick_pose = PoseStamped()
            object_to_pick_pose.header.frame_id = external_object.header.frame_id or self.global_reference_frame
            object_to_pick_pose.pose = copy.deepcopy(external_object.pose)
            object_to_pick_bounding_box = [external_object.size.x, external_object.size.y, external_object.size.z]
            object_to_pick_id = external_object.instance_id
            object_found = True
            rospy.loginfo(
                f'adding object {external_object_name} to planning scene from the external detection '
                f'(frame {object_to_pick_pose.header.frame_id}, '
                f'size {[round(v, 3) for v in object_to_pick_bounding_box]})'
            )
            self.add_object_box(external_object_name, object_to_pick_pose, object_to_pick_bounding_box, supports)
        if not object_found:
            rospy.logerr(
                'the specific object you want to pick was not found:'
                f' {object_to_pick.get_object_class_and_id_as_string()}'
            )
            return None, None, None
        return (
            self.transform_pose(object_to_pick_pose, self.robot.get_planning_frame()),
            object_to_pick_bounding_box,
            object_to_pick_id,
        )

    def support_boxes(self, exclude=()):
        '''
        {frame: [(center, orientation, size)]} of the table-sized (>= 30 cm wide) boxes of the planning scene that are
        not perceived objects (tables, walls: the ones an object box may reach into), for clamp_box_to_support
        '''
        boxes = {}
        # the configured tables directly: add_custom_boxes_to_ps() has only just sent them to move_group (a topic),
        # the scene query below may not see them yet
        for b in self.planning_scene_boxes:
            boxes.setdefault(b['frame_id'], []).append((
                (b['box_position_x'], b['box_position_y'], b['box_position_z']),
                (b['box_orientation_x'], b['box_orientation_y'], b['box_orientation_z'], b['box_orientation_w']),
                (b['box_x_dimension'], b['box_y_dimension'], b['box_z_dimension']),
            ))
        for name, obj in self.scene.get_objects().items():
            if name in exclude or any(b['scene_name'] == name for b in self.planning_scene_boxes):
                continue
            for primitive, pose in zip(obj.primitives, obj.primitive_poses):
                if primitive.type != SolidPrimitive.BOX or min(primitive.dimensions[:2]) < 0.3:
                    continue  # not a table: e.g. the box of an object picked earlier
                p = obj.pose.position
                object_q, primitive_q = (
                    [q.x, q.y, q.z, q.w] if (q.x, q.y, q.z, q.w) != (0.0, 0.0, 0.0, 0.0) else [0.0, 0.0, 0.0, 1.0]
                    for q in (obj.pose.orientation, pose.orientation)
                )  # an unset quaternion (all 0) means identity
                rotation = tf.transformations.quaternion_matrix(object_q)[:3, :3]
                center = [p.x, p.y, p.z] + rotation @ [pose.position.x, pose.position.y, pose.position.z]
                orientation = tf.transformations.quaternion_multiply(object_q, primitive_q)
                boxes.setdefault(obj.header.frame_id, []).append((center, orientation, primitive.dimensions[:3]))
        return boxes

    def add_object_box(self, name, pose_stamped, size, supports=None):
        '''
        add a perceived object box to the planning scene; with ~clamp_object_boxes_to_support its bottom is raised to
        the top of the support box (supports: support_boxes()) it reaches into; only the scene box changes, the pose
        used for grasping stays as perceived (#148)
        '''
        frame_supports = (supports or {}).get(pose_stamped.header.frame_id, [])
        if frame_supports:
            p, o = pose_stamped.pose.position, pose_stamped.pose.orientation
            center, clamped, lifted = clamp_box_to_support((p.x, p.y, p.z), (o.x, o.y, o.z, o.w), size, frame_supports)
            if lifted > 0.0:
                rospy.loginfo(f'{name}: box bottom was {lifted * 100:.1f} cm inside its support, raised to its top')
                pose_stamped = copy.deepcopy(pose_stamped)
                pose_stamped.pose.position.z = float(center[2])
                size = [float(v) for v in clamped]
        self.scene.add_box(name, pose_stamped, size)

    def clean_scene(self):
        '''
        iterate over all object in the scene, delete them from the scene
        '''
        for item in self.scene.get_known_object_names():
            self.scene.remove_world_object(item)

    def clear_octomap(self, octomap_srv_name='clear_octomap'):
        '''
        call service to clear octomap
        '''
        rospy.logwarn('Clearing octomap')
        rospy.ServiceProxy(octomap_srv_name, Empty)()

    def read_allowed_collision_matrix(self):
        '''
        return the allowed collision matrix of the planning scene that move_group
        maintains, or None when it cannot be read
        '''
        try:
            request = GetPlanningSceneRequest()
            request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            acm = self.get_planning_scene_srv(request).scene.allowed_collision_matrix
        except rospy.ServiceException as e:
            rospy.logwarn(f'could not read the allowed collision matrix: {e}')
            return None
        if not acm.entry_names:
            rospy.logwarn('the allowed collision matrix is empty')
            return None
        return acm

    def publish_allowed_collision_matrix(self, acm):
        '''
        replace the allowed collision matrix of the planning scene that move_group maintains
        '''
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.allowed_collision_matrix = acm
        self.planning_scene_pub.publish(planning_scene)

    def acm_allowing_octomap_contact(self, acm, object_name):
        '''
        extend the allowed collision matrix acm so that the octomap may touch the gripper
        and the object being picked. The octomap remains an obstacle for the rest of the
        robot, and the original matrix is restored once the pick is over.
        '''
        touch_links = self.octomap_touch_links or self.robot.get_link_names(group=self.gripper_group_name)
        rospy.loginfo(f'allowing {OCTOMAP_COLLISION_NAME} to touch {object_name} and the gripper during this pick')
        return self.allow_collisions_with(acm, OCTOMAP_COLLISION_NAME, list(touch_links) + [object_name])

    def allow_collisions_with(self, acm, name, other_names):
        '''
        mark the collisions between name and every entry of other_names as allowed in the
        AllowedCollisionMatrix message acm, adding the names that it does not contain yet
        '''
        names = list(acm.entry_names)
        rows = [list(entry.enabled) for entry in acm.entry_values]
        for missing in [name] + list(other_names):
            if missing not in names:
                names.append(missing)
                for row in rows:
                    row.append(False)
                rows.append([False] * len(names))
        index = names.index(name)
        for other in other_names:
            other_index = names.index(other)
            rows[index][other_index] = True
            rows[other_index][index] = True
        acm.entry_names = names
        acm.entry_values = [AllowedCollisionEntry(enabled=row) for row in rows]
        return acm

    def move_to_anygrasp_view(self, object_name):
        '''
        point the camera at the object for AnyGrasp: the view service when there is one, else anygrasp_arm_pose
        '''
        if self.anygrasp_view_service:
            try:
                rospy.wait_for_service(self.anygrasp_view_service, timeout=1.0)
                response = rospy.ServiceProxy(self.anygrasp_view_service, ViewObject)(object_name=object_name)
                if response.success:
                    rospy.loginfo(f'AnyGrasp view of {object_name!r}: {response.message}')
                    return True
                rospy.logwarn(f'no AnyGrasp view of {object_name!r} ({response.message}), using {self.anygrasp_arm_pose!r}')
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn(f'AnyGrasp view service {self.anygrasp_view_service} unavailable ({exc}), '
                              f'using {self.anygrasp_arm_pose!r}')
        return self.move_arm_to_posture(self.anygrasp_arm_pose)

    def move_arm_to_posture(self, arm_posture_name):
        '''
        use moveit commander to send the arm to a predefined arm configuration
        defined in srdf
        '''
        rospy.loginfo(f'moving arm to {arm_posture_name}')
        self.robot.arm.set_named_target(arm_posture_name)
        # attempt to move it 2 times, (sometimes fails with only 1 time)
        if self.robot.arm.go():
            return True
        else:
            rospy.logwarn(f'failed to move arm to posture: {arm_posture_name}, will retry one more time in 1 sec')
            rospy.sleep(1.0)
            return self.robot.arm.go()

    def open_gripper(self):
        '''Open the gripper through its GripperCommand action server.'''
        if not self.gripper_action_client.wait_for_server(rospy.Duration(self.gripper_action_timeout)):
            rospy.logerr('cannot open gripper: action server %s is unavailable', self.gripper_action_name)
            return False

        open_positions = self.grasp_planner.gripper_open_distance
        open_position = open_positions[0] if isinstance(open_positions, (list, tuple)) else open_positions
        goal = GripperCommandGoal()
        goal.command.position = open_position
        self.gripper_action_client.send_goal(goal)
        if not self.gripper_action_client.wait_for_result(rospy.Duration(self.gripper_action_timeout)):
            self.gripper_action_client.cancel_goal()
            rospy.logerr('timed out while opening gripper through %s', self.gripper_action_name)
            return False
        return self.gripper_action_client.get_state() == GoalStatus.SUCCEEDED

    def move_gripper_to_posture(self, gripper_posture_name):
        '''
        WARNING, THIS FUNCTION SHOULD NOT BE USED!

        The gripper configurations in the SRDF are in radians, but the actual
        gripper action server (/mobipick/gripper_hw, type
        control_msgs/GripperCommand) expects values in meters. This means that
        when this function is used to send the gripper to the "opened"
        position, it will close and vice versa. Use the GripperCommand action
        server directly instead.

        use moveit commander to send the gripper to a predefined configuration
        defined in srdf
        '''
        self.robot.gripper.set_named_target(gripper_posture_name)
        self.robot.gripper.go()

    def ensure_attached(self, object_name):
        '''
        After a successful pick the object must hang on the gripper in the planning scene (shown purple in
        RViz): MoveIt's pickup attaches it, but when it does not (seen with open-set objects), attach the
        world box explicitly so placing, inserting and every later arm motion plan with it.
        '''
        attached = self.scene.get_attached_objects([object_name])
        if attached:
            rospy.loginfo(f'{object_name} is attached to {attached[object_name].link_name}')
            return True
        objects = self.scene.get_objects([object_name])
        if object_name not in objects:
            rospy.logwarn(f'{object_name} is neither attached nor in the planning scene after the pick')
            return False
        collision_object = objects[object_name]
        if not collision_object.primitives:
            rospy.logwarn(f'{object_name} has no primitive to attach')
            return False
        eef_link = self.robot.arm.get_end_effector_link()
        touch_links = self.robot.get_link_names(group=self.gripper.get_name())
        pose = PoseStamped()
        pose.header.frame_id = collision_object.header.frame_id
        pose.pose = collision_object.pose if any(
            (collision_object.pose.orientation.x, collision_object.pose.orientation.y,
             collision_object.pose.orientation.z, collision_object.pose.orientation.w)
        ) else collision_object.primitive_poses[0]
        size = list(collision_object.primitives[0].dimensions)
        self.scene.remove_world_object(object_name)
        self.scene.attach_box(eef_link, object_name, pose, size, touch_links=touch_links)
        rospy.logwarn(f'moveit did not attach {object_name}; attached its box to {eef_link} explicitly')
        return True

    def detach_all_objects(self):
        for attached_object in self.scene.get_attached_objects().keys():
            self.gripper.detach_object(name=attached_object)

    def clear_mesh_markers(self, namespace, publisher):
        marker_array_msg = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.ns = namespace
        marker.action = Marker.DELETEALL
        marker_array_msg.markers.append(marker)
        publisher.publish(marker_array_msg)

    def add_custom_boxes_to_ps(self, planning_scene_boxes):
        # add a list of custom boxes defined by the user to the planning scene
        for planning_scene_box in planning_scene_boxes:
            # add a box to the planning scene
            table_pose = PoseStamped()
            table_pose.header.frame_id = planning_scene_box['frame_id']
            box_x = planning_scene_box['box_x_dimension']
            box_y = planning_scene_box['box_y_dimension']
            box_z = planning_scene_box['box_z_dimension']
            table_pose.pose.position.x = planning_scene_box['box_position_x']
            table_pose.pose.position.y = planning_scene_box['box_position_y']
            table_pose.pose.position.z = planning_scene_box['box_position_z']
            table_pose.pose.orientation.x = planning_scene_box['box_orientation_x']
            table_pose.pose.orientation.y = planning_scene_box['box_orientation_y']
            table_pose.pose.orientation.z = planning_scene_box['box_orientation_z']
            table_pose.pose.orientation.w = planning_scene_box['box_orientation_w']
            self.scene.add_box(planning_scene_box['scene_name'], table_pose, (box_x, box_y, box_z))

    def pick_object(
        self,
        object_name_as_string,
        support_surface_name,
        grasp_type,
        ignore_object_list=[],
        external_grasp_candidates=None,
        external_reference_frame=None,
        external_object=None,
        perceive_object=None,
    ):
        '''
        1) move arm to a position where the attached camera can see the scene (octomap will be populated)
        2) clear octomap
        3) add table and object to be grasped to planning scene
        4) generate a list of grasp configurations
        5) call pick moveit functionality
        '''
        rospy.loginfo(f'attempting to pick object : {object_name_as_string}')
        if len(ignore_object_list) > 0:
            rospy.logwarn(f'the following objects: {ignore_object_list} will be ignored from the planning scene')

        if not self.detach_all_objects_flag and len(self.scene.get_attached_objects().keys()) > 0:
            rospy.logerr('cannot pick object, another object is currently attached to the gripper already')
            return False

        object_to_pick = objectToPick(object_name_as_string)

        # open gripper
        # rospy.loginfo('gripper will open now')
        # self.move_gripper_to_posture('open')

        # detach (all) object if any from the gripper
        if self.detach_all_objects_flag:
            self.detach_all_objects()

        # ::::::::: perceive object to be picked (optional, read from parameter server if this is required)

        if perceive_object is None:
            perceive_object = self.perceive_object
        if perceive_object:
            # send arm to a pose where objects are inside fov
            self.move_arm_to_posture(self.arm_pose_with_objs_in_fov)
            # populate pose selector with pose information
            self.activate_pose_selector_srv(True)
            # wait until pose selector gets updates
            rospy.sleep(4.0)
            # deactivate pose selector detections
            self.activate_pose_selector_srv(False)

        # ::::::::: setup planning scene
        rospy.loginfo('setup planning scene')

        # remove all objects from the planning scene if needed
        if self.clear_planning_scene:
            rospy.logwarn('Clearing planning scene')
            self.clean_scene()

        # add a list of custom boxes defined by the user to the planning scene
        self.add_custom_boxes_to_ps(self.planning_scene_boxes)

        # add all perceived objects of interest to planning scene and return the pose, bb, and id of the
        # object to be picked
        object_pose, bounding_box, id = self.make_object_pose_and_add_objs_to_planning_scene(
            object_to_pick, ignore_object_list=ignore_object_list, external_object=external_object
        )

        if object_pose is None:
            return False

        # this condition is when user only specified object class but no id, then we assign the first available id
        if id is not None:
            object_to_pick.set_id(id)

        self.obj_pose_pub.publish(object_pose)  # publish object pose for visualization purposes

        # print objects that were added to the planning scene
        rospy.loginfo(f'planning scene objects: {self.scene.get_known_object_names()}')

        # move arm to pregrasp in joint space, not really needed, can be removed
        if self.pregrasp_posture_required:
            self.move_arm_to_posture(self.pregrasp_posture)

        # check if user cancelled action
        if self.pick_action_server.is_preempt_requested():
            rospy.logwarn(
                f'grasplan {self.pick_action_server.action_server.ns} ' 'action server goal cancel request received'
            )
            return False

        # ::::::::: pick
        rospy.loginfo('picking object now')

        # generate a list of moveit grasp messages, poses are also published for visualization purposes
        if external_grasp_candidates is None:
            grasps = self.grasp_planner.make_grasps_msgs(
                object_to_pick.get_object_class_and_id_as_string(),
                object_pose,
                self.robot.arm.get_end_effector_link(),
                grasp_type,
            )
        else:
            grasps = []
            if external_grasp_candidates:
                grasps = self.grasp_planner.make_grasps_msgs_from_candidates(
                    object_to_pick.get_object_class_and_id_as_string(),
                    external_grasp_candidates,
                    external_reference_frame,
                    self.robot.arm.get_end_effector_link(),
                )
                grasps = self.raise_low_grasps(
                    grasps, support_surface_name, [candidate.width for candidate in external_grasp_candidates]
                )
            if not grasps:
                rospy.logwarn(f'no open-set grasp of {object_to_pick.get_object_class_and_id_as_string()} keeps the '
                              f'fingertips clear of {support_surface_name}')
                grasps = self.topdown_fallback_grasps(object_to_pick, object_pose, bounding_box, support_surface_name)
                if not grasps:
                    return False

        # clear octomap from the planning scene if needed
        if self.clear_octomap_flag:
            self.clear_octomap()

        # go to intermediate arm poses if needed to disentangle arm cable
        if object_to_pick.obj_class in self.list_of_disentangle_objects:
            for arm_pose in self.poses_to_go_before_pick:
                rospy.loginfo(f'going to intermediate arm pose {arm_pose} to disentangle cable')
                self.move_arm_to_posture(arm_pose)

        # check if user cancelled action
        if self.pick_action_server.is_preempt_requested():
            rospy.logwarn(
                f'grasplan {self.pick_action_server.action_server.ns} ' 'action server goal cancel request received'
            )
            return False

        # try to pick object with moveit
        # result = self._pick_with_moveit_commander(object_to_pick, grasps, support_surface_name)
        batch = self.external_grasp_batch_size if external_grasp_candidates is not None else 0
        if batch > 0 and len(grasps) > batch:
            result = None
            for start in range(0, len(grasps), batch):
                chunk = grasps[start:start + batch]
                rospy.loginfo(
                    f'trying grasps {start + 1}-{start + len(chunk)} of {len(grasps)} (best first, qualities '
                    f'{", ".join(f"{g.grasp_quality:.3f}" for g in chunk)})'
                )
                result = self._pick_with_action(object_to_pick, chunk, support_surface_name)
                if result == MoveItErrorCodes.SUCCESS or result is None:
                    break
                if self.mtc is not None and self.mtc.executed:
                    break  # the arm moved (e.g. closed on nothing): report instead of retrying from a changed state
                if self.pick_action_server.is_preempt_requested():
                    return False
        else:
            result = self._pick_with_action(object_to_pick, grasps, support_surface_name)
        # every AnyGrasp grasp failed before the arm moved (no IK, blocked lift, ...): try the top-down fallback once
        if (
            external_grasp_candidates is not None
            and result not in (MoveItErrorCodes.SUCCESS, None)
            and not any(g.id.startswith('topdown_') for g in grasps)
            and self.arm_did_not_move(result)
            and not self.pick_action_server.is_preempt_requested()
        ):
            fallback = self.topdown_fallback_grasps(object_to_pick, object_pose, bounding_box, support_surface_name)
            if fallback:
                result = self._pick_with_action(object_to_pick, fallback, support_surface_name)
        # handle moveit pick result
        if result == MoveItErrorCodes.SUCCESS:
            self.ensure_attached(object_to_pick.get_object_class_and_id_as_string())
            # remove picked object pose from pose selector
            rospy.loginfo(
                'removing picked object from pose selector due to succesfull execution (as reported by moveit)'
            )
            self.pose_selector_delete_srv(class_id=object_to_pick.obj_class, instance_id=object_to_pick.id)
            rospy.loginfo(f'Successfully picked object : {object_to_pick.get_object_class_and_id_as_string()}')
            # clear possible grasps shown as mesh in rviz
            self.clear_mesh_markers(namespace='grasp_poses', publisher=self.pick_grasps_marker_array_pub)
            self.clear_mesh_markers(namespace='object', publisher=self.pose_selector_objects_marker_array_pub)
            return True
        else:
            rospy.logerr('grasp failed')
            if result:  # if result is None it means moveit action server was not found within 2 secs
                print_moveit_error(result)  # only print moveit error if result is different than None
        return False

    def raise_low_grasps(self, grasps, support_surface_name, widths=None):
        '''
        AnyGrasp does not know the Robotiq 140 fingers, so for small objects it proposes grasps whose fingers would
        go into the table (#124 strawberry, #134 banana). Two limits above the top of the support surface (m, 0 =
        off, re-read on every goal), the higher requirement wins:
        ~open_set_min_grasp_height: of the TCP.
        ~open_set_min_fingertip_height: of the lowest fingertip point while the gripper closes from open down to the
        grasp width (widths, AnyGrasp jaw opening per grasp; 0 or missing = fully closed), from the URDF: on a tilted
        grasp the lower finger dips below the TCP, and the 140's tips move 2.4 cm forward while closing.
        A grasp that is too low is moved back along its approach axis (TCP +x) until both hold, at most
        ~open_set_max_grasp_backoff (m, 0 = no limit): further back the fingers would close on air, so such a grasp
        is dropped, as is a grasp approaching sideways (less than 17 degrees downwards) whose fingertips are too low
        (backing off does not raise it). Returns the grasps to try.
        '''
        min_height = rospy.get_param('~open_set_min_grasp_height', 0.0)
        min_fingertip_height = rospy.get_param('~open_set_min_fingertip_height', 0.0)
        max_backoff = rospy.get_param('~open_set_max_grasp_backoff', 0.0)
        if (min_height <= 0.0 and min_fingertip_height <= 0.0) or not support_surface_name or not grasps:
            return grasps
        support = self.scene.get_objects([support_surface_name]).get(support_surface_name)
        frame = grasps[0].grasp_pose.header.frame_id
        if support is None or not support.primitives or support.header.frame_id != frame:
            rospy.logwarn(f'open-set grasp height: no {support_surface_name} box in frame {frame}, grasps unchanged')
            return grasps
        top = -float('inf')
        for primitive, pose in zip(support.primitives, support.primitive_poses):
            top = max(top, support.pose.position.z + pose.position.z + primitive.dimensions[2] / 2.0)
        envelope = self.fingertip_envelope() if min_fingertip_height > 0.0 else None
        widths = list(widths) if widths is not None else [0.0] * len(grasps)
        kept, raised, dropped = [], [], []
        for grasp, width in zip(grasps, widths):
            pose = grasp.grasp_pose.pose
            q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            approach = tf.transformations.quaternion_matrix(q)[:3, 0]
            if pose.position.z < top - 0.05:
                kept.append(grasp)
                continue  # the object is not on this surface (e.g. fell to the floor)
            missing = top + min_height - pose.position.z if min_height > 0.0 else 0.0
            if envelope:
                lowest = pose.position.z + lowest_point_offset(envelope, q, width)
                missing = max(missing, top + min_fingertip_height - lowest)
            if missing <= 0.0:
                kept.append(grasp)
                continue
            if approach[2] > -0.3:
                if envelope and lowest < top + min_fingertip_height:
                    dropped.append(f'{grasp.id} ({(lowest - top) * 100:+.1f} cm)')
                else:
                    kept.append(grasp)
                continue
            shift = missing / -approach[2]
            if 0.0 < max_backoff < shift:
                dropped.append(f'{grasp.id} (needs {shift * 100:.1f} cm back)')
                continue
            pose.position.x -= shift * approach[0]
            pose.position.y -= shift * approach[1]
            pose.position.z -= shift * approach[2]
            raised.append(f'{grasp.id} +{missing * 100:.1f} cm')
            kept.append(grasp)
        if raised:
            rospy.loginfo(f'raised {len(raised)} of {len(grasps)} grasps (tcp {min_height * 100:.1f} cm, fingertips '
                          f'{min_fingertip_height * 100:.1f} cm above {support_surface_name}, top {top:.3f}): '
                          f'{", ".join(raised)}')
        if dropped:
            rospy.loginfo(f'dropped {len(dropped)} grasps that cannot keep the fingertips '
                          f'{min_fingertip_height * 100:.1f} cm above {support_surface_name} (sideways, or more than '
                          f'{max_backoff * 100:.1f} cm back): '
                          f'{", ".join(dropped)}')
        return kept

    def arm_did_not_move(self, result):
        '''whether a failed pick left the arm where it was: MTC knows, MoveIt's pickup only when planning failed'''
        if self.mtc is not None:
            return not self.mtc.executed
        return result in (MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.NO_IK_SOLUTION)

    def support_top(self, support_surface_name, frame):
        '''top z of the support surface box in frame, None if it is not in the planning scene in that frame'''
        if not support_surface_name:
            return None
        support = self.scene.get_objects([support_surface_name]).get(support_surface_name)
        if support is None or not support.primitives or support.header.frame_id != frame:
            return None
        return max(
            support.pose.position.z + pose.position.z + primitive.dimensions[2] / 2.0
            for primitive, pose in zip(support.primitives, support.primitive_poses)
        )

    def topdown_fallback_grasps(self, object_to_pick, object_pose, bounding_box, support_surface_name):
        '''
        ~open_set_topdown_fallback: when AnyGrasp offers no usable grasp of an open-set object (#148), grasp it straight
        down from its box instead (grasplan.tools.topdown_grasps): closing across a horizontal box axis at most
        ~open_set_topdown_max_width wide (narrow one first, each also turned 180 degrees), the TCP as low as
        ~open_set_min_fingertip_height (lowest fingertip point) and ~open_set_min_grasp_height (TCP) above the support
        allow, and the pads reaching at least ~open_set_topdown_min_overlap below the object top. Returns MoveIt grasps
        (ids topdown_N), [] when off or impossible.
        '''
        if not rospy.get_param('~open_set_topdown_fallback', False):
            return []
        name = object_to_pick.get_object_class_and_id_as_string()
        frame = object_pose.header.frame_id
        top = self.support_top(support_surface_name, frame)
        envelope = self.fingertip_envelope()
        if top is None or not envelope:
            rospy.logwarn(f'top-down fallback for {name}: no {support_surface_name} box in {frame} or no fingertip '
                          'envelope, skipped')
            return []
        p, o = object_pose.pose.position, object_pose.pose.orientation
        poses, skipped = topdown_grasps(
            (p.x, p.y, p.z),
            (o.x, o.y, o.z, o.w),
            bounding_box,
            top,
            # over the whole closing motion: the perceived box is often wider than the object, the fingers close
            # further than its width and the Robotiq 140 tips move down while closing
            lambda q, width: lowest_point_offset(envelope, q, 0.0),
            rospy.get_param('~open_set_min_fingertip_height', 0.0),
            rospy.get_param('~open_set_min_grasp_height', 0.0),
            rospy.get_param('~open_set_topdown_max_width', 0.10),
            rospy.get_param('~open_set_topdown_min_overlap', 0.01),
        )
        if skipped:
            rospy.loginfo(f'top-down fallback for {name}: skipped {"; ".join(skipped)}')
        if not poses:
            rospy.logerr(f'top-down fallback for {name}: no grasp, box {[round(v, 3) for v in bounding_box]}')
            return []
        candidates = []
        for position, q, width in poses:
            candidate = GraspCandidate(quality=0.5, width=width)
            candidate.pose.position.x, candidate.pose.position.y, candidate.pose.position.z = position
            candidate.pose.orientation.x, candidate.pose.orientation.y = q[0], q[1]
            candidate.pose.orientation.z, candidate.pose.orientation.w = q[2], q[3]
            candidates.append(candidate)
        grasps = self.grasp_planner.make_grasps_msgs_from_candidates(
            name, candidates, frame, self.robot.arm.get_end_effector_link()
        )
        for i, grasp in enumerate(grasps):
            grasp.id = f'topdown_{i}'
        rospy.logwarn(
            f'top-down fallback for {name}: {len(grasps)} grasp(s), widths '
            f'{", ".join(f"{w * 100:.1f}" for _, _, w in poses)} cm, TCP '
            f'{(poses[0][0][2] - top) * 100:.1f} cm above {support_surface_name}'
        )
        return grasps

    def fingertip_envelope(self):
        '''fingertip corners in the TCP frame over the closing motion (tools.gripper_envelope), [] if unknown'''
        if self._fingertip_envelope is None:
            self._fingertip_envelope = []
            try:
                from urdf_parser_py.urdf import URDF

                robot = URDF.from_xml_string(rospy.get_param('robot_description'))
                tips = [
                    link
                    for link in self.robot.get_link_names(group=self.gripper_group_name)
                    if 'fingertip' in link
                ]
                actuated = self.gripper.get_active_joints()[0]
                self._fingertip_envelope = fingertip_envelope(
                    robot, self.robot.arm.get_end_effector_link(), tips, actuated
                )
                rospy.loginfo(f'fingertip envelope from {tips}, {actuated}: {len(self._fingertip_envelope)} samples')
            except Exception as e:  # keep picking with the TCP limit only
                rospy.logerr(f'no fingertip envelope, ~open_set_min_fingertip_height is ignored: {e}')
        return self._fingertip_envelope

    def _pick_with_moveit_commander(self, object_to_pick, grasps, support_surface_name):
        self.robot.arm.set_support_surface_name(support_surface_name)
        result = self.robot.arm.pick(object_to_pick.get_object_class_and_id_as_string(), grasps)
        return result

    def _pick_with_action(self, object_to_pick, grasps, support_surface_name):
        """
        Picks the object using the action client directly, bypassing the moveit_commander.
        This is so we can set the support_surface_name without also setting allow_gripper_support_collision to "true",
        otherwise there will be collisions.
        """
        result = None
        PICK_OBJECT_SERVER_NAME = 'pickup'

        if self.mtc is not None:
            return self._pick_with_mtc(object_to_pick, grasps, support_surface_name)

        action_client = self.pickup_action_client
        if action_client.wait_for_server(timeout=rospy.Duration(self.moveit_action_server_timeout)):
            rospy.loginfo(f'found {rospy.resolve_name(PICK_OBJECT_SERVER_NAME)} action server')
            goal = PickupGoal()
            goal.target_name = object_to_pick.get_object_class_and_id_as_string()
            goal.group_name = self.arm_group_name
            goal.possible_grasps = grasps
            goal.support_surface_name = support_surface_name
            goal.allowed_planning_time = self.planning_time
            goal.planning_options.planning_scene_diff.is_diff = True
            goal.planning_options.planning_scene_diff.robot_state.is_diff = True
            goal.planning_options.replan_delay = 2.0

            # move_group validates the grasp trajectories against the planning scene it
            # maintains, while planning the pick and again on every scene update during
            # execution, so the octomap exception has to live there and not only in this goal
            original_acm = self.read_allowed_collision_matrix() if self.allow_octomap_contact else None
            if original_acm is not None:
                self.publish_allowed_collision_matrix(
                    self.acm_allowing_octomap_contact(copy.deepcopy(original_acm), goal.target_name)
                )
            try:
                rospy.loginfo(
                    f'sending pick {object_to_pick.get_object_class_and_id_as_string()} goal '
                    f'to {rospy.resolve_name(PICK_OBJECT_SERVER_NAME)} action server'
                )
                rospy.loginfo(f'waiting for result from {rospy.resolve_name(PICK_OBJECT_SERVER_NAME)} action server')
                self.action_client_helper.send_goal_to_rogue_server_and_wait(goal, action_client, patience_timeout=0.1)
                pickup_result = action_client.get_result()
                result = pickup_result.error_code.val  # get moveit error code
                if result == MoveItErrorCodes.SUCCESS:
                    executed = pickup_result.grasp
                    rospy.loginfo(
                        f'moveit executed grasp {executed.id!r} (quality {executed.grasp_quality:.3f}) out of '
                        f'{len(grasps)} offered'
                    )
            finally:
                if original_acm is not None:
                    rospy.loginfo(f'restoring the allowed collision matrix, {OCTOMAP_COLLISION_NAME} blocks again')
                    self.publish_allowed_collision_matrix(original_acm)
        else:
            rospy.logerr(
                f'action server {rospy.resolve_name(PICK_OBJECT_SERVER_NAME)} was not found within '
                f'{self.moveit_action_server_timeout} s'
            )
        return result

    def _pick_with_mtc(self, object_to_pick, grasps, support_surface_name):
        '''
        same goal as _pick_with_action, planned and executed with MoveIt Task Constructor (grasplan.mtc_pick_place).
        MTC plans in this node, so the octomap contact is allowed inside the task and move_group's allowed
        collision matrix stays untouched.
        '''
        goal = PickupGoal()
        goal.target_name = object_to_pick.get_object_class_and_id_as_string()
        goal.group_name = self.arm_group_name
        goal.possible_grasps = grasps
        goal.support_surface_name = support_surface_name
        goal.allowed_planning_time = self.planning_time
        rospy.loginfo(f'planning pick of {goal.target_name} with MTC, {len(grasps)} grasps')
        pickup_result = self.mtc.pickup(goal)
        result = pickup_result.error_code.val
        if result == MoveItErrorCodes.SUCCESS:
            executed = pickup_result.grasp
            rospy.loginfo(
                f'mtc executed grasp {executed.id!r} (quality {executed.grasp_quality:.3f}) out of {len(grasps)} offered'
            )
        return result

    def start_pick_node(self):
        # wait for trigger via topic or action lib
        rospy.loginfo('ready to receive pick requests')
        rospy.spin()
        # shutdown moveit cpp interface before exit
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    rospy.init_node('pick_object_node', anonymous=False)
    pick = PickTools()
    pick.start_pick_node()
