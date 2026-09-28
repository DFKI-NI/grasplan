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
example on how to place an object using grasplan and moveit
'''

import math
import sys
import copy
import tf2_ros
import rospy
import actionlib
import moveit_commander
import traceback

from grasplan.tools.support_plane_tools import (
    obj_to_plane,
    adjust_plane,
    gen_place_poses_from_plane,
    make_plane_marker_msg,
    OBJECT_HEIGHTS,
)
from grasplan.tools.common import separate_object_class_from_id, connect_move_groups, roscpp_initialize_named
from grasplan.tools.moveit_errors import print_moveit_error
from grasplan.tools.place_reach import order_by_reach
from std_srvs.srv import Empty, SetBool, Trigger
from object_pose_msgs.msg import ObjectList
import tf2_geometry_msgs
from geometry_msgs.msg import Vector3Stamped, PoseStamped, TransformStamped, PointStamped, Point, Transform, Vector3
from moveit_msgs.msg import (
    PlaceAction,
    PlaceGoal,
    PlaceLocation,
    GripperTranslation,
    PlanningOptions,
    Constraints,
    OrientationConstraint,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from grasplan.msg import PlaceObjectAction, PlaceObjectResult
from moveit_msgs.msg import MoveItErrorCodes
from pose_selector.srv import GetPoses
from grasplan.tools.action_client_helper import ActionClientHelper
from visualization_msgs.msg import Marker, MarkerArray
from typing import List
from std_msgs.msg import Header


class PlaceTools:
    def __init__(self, action_server_required=True):
        self.global_reference_frame = rospy.get_param('~global_reference_frame', 'map')
        self.arm_pose_with_objs_in_fov = rospy.get_param('~arm_pose_with_objs_in_fov', 'observe100cm_right')
        self.min_dist = rospy.get_param('~min_dist', 0.2)
        self.ignore_min_dist_list = rospy.get_param('~ignore_min_dist_list', ['foo_obj'])
        self.group_name = rospy.get_param('~group_name', 'arm')
        self.arm_name = rospy.get_param('~arm_name', 'ur5')
        self.gripper_joint_names = rospy.get_param('~gripper_joint_names')
        self.gripper_joint_efforts = rospy.get_param('~gripper_joint_efforts')
        self.gripper_release_distance = rospy.get_param('~gripper_release_distance', 0.1)
        self.planning_time = rospy.get_param('~planning_time', 20.0)
        # Open-set objects (no known height, grasped wherever AnyGrasp found a grasp) are placed
        # this far above the support surface: their grasps are often low, and put down exactly on
        # the surface the fingertips end up inside it. They also try yaws around the full circle,
        # since an angled grasp only fits some approach directions. Known objects are unchanged.
        self.open_set_place_clearance = rospy.get_param('~open_set_place_clearance', 0.02)
        self.open_set_place_yaw_steps = int(rospy.get_param('~open_set_place_yaw_steps', 12))
        # MoveIt's pickup/place action servers live in move_group, which on the real robot runs on the
        # robot PC; connecting to it from another machine can take seconds, so the client is created
        # once at startup (a missing server at startup is fatal) and reused for every request
        self.moveit_action_startup_timeout = rospy.get_param('~moveit_action_startup_timeout', 60.0)
        self.moveit_action_server_timeout = rospy.get_param('~moveit_action_server_timeout', 10.0)
        arm_goal_tolerance = rospy.get_param('~arm_goal_tolerance', 0.01)
        self.use_path_constraints = rospy.get_param('~use_path_constraints', False)
        self.disentangle_required = rospy.get_param('~disentangle_required', False)
        # By default the cable is untangled only when the goal asks to observe first; true also untangles
        # for goals without observe_before_place (the planner never asks for it, and the real robot's cable
        # caught on the arm while placing)
        self.disentangle_without_observe = rospy.get_param('~disentangle_without_observe', False)
        self.poses_to_go_before_place = rospy.get_param('~poses_to_go_before_place', [])
        self.max_batch_size = rospy.get_param('~max_batch_size', 20)
        self.clear_octomap_flag = rospy.get_param('~clear_octomap', False)
        # plan and execute place (and insert) with MoveIt Task Constructor instead of move_group's place action
        self.use_mtc = rospy.get_param('~use_mtc', False)

        self.plane_vis_pub = rospy.Publisher('~support_plane_as_marker', Marker, queue_size=1, latch=True)
        self.place_poses_pub = rospy.Publisher('~place_poses', ObjectList, queue_size=50)
        self.marker_array_pub = rospy.Publisher('/place_pose_selector_objects', MarkerArray, queue_size=1)

        # service clients
        place_pose_selector_activate_srv_name = rospy.get_param(
            '~place_pose_selector_activate_srv_name', '/place_pose_selector_activate'
        )
        place_pose_selector_clear_srv_name = rospy.get_param(
            '~place_pose_selector_clear_srv_name', '/place_pose_selector_clear'
        )
        pick_pose_selector_activate_srv_name = rospy.get_param(
            '~pick_pose_selector_activate_srv_name', '/pick_pose_selector_activate'
        )
        pick_pose_selector_get_all_poses_srv_name = rospy.get_param(
            '~pick_pose_selector_get_all_poses_srv_name', '/pose_selector_get_all'
        )
        rospy.loginfo(
            f'waiting for pose selector services: {place_pose_selector_activate_srv_name},'
            f' {place_pose_selector_clear_srv_name} {pick_pose_selector_activate_srv_name},'
            f' {pick_pose_selector_get_all_poses_srv_name}'
        )
        rospy.wait_for_service(place_pose_selector_activate_srv_name, 30.0)
        rospy.wait_for_service(place_pose_selector_clear_srv_name, 30.0)
        rospy.wait_for_service(pick_pose_selector_activate_srv_name, 30.0)
        rospy.wait_for_service(pick_pose_selector_get_all_poses_srv_name, 30.0)
        try:
            self.activate_place_pose_selector_srv = rospy.ServiceProxy(place_pose_selector_activate_srv_name, SetBool)
            self.place_pose_selector_clear_srv = rospy.ServiceProxy(place_pose_selector_clear_srv_name, Trigger)
            self.activate_pick_pose_selector_srv = rospy.ServiceProxy(pick_pose_selector_activate_srv_name, SetBool)
            self.get_all_poses_pick_pose_selector_srv = rospy.ServiceProxy(
                pick_pose_selector_get_all_poses_srv_name, GetPoses
            )
            rospy.loginfo('found pose selector services')
        except rospy.exceptions.ROSException:
            rospy.logfatal(
                'grasplan place server could not find pose selector services in time, exiting! \n'
                + traceback.format_exc()
            )
            rospy.signal_shutdown('fatal error')

        # activate place pose selector to be ready to store the place poses
        self.activate_place_pose_selector_srv(True)

        # wait for moveit to become available, TODO: find a cleaner way to wait for moveit
        rospy.wait_for_service('move_group/planning_scene_monitor/set_parameters', 30.0)
        rospy.sleep(2.0)

        try:
            rospy.loginfo('waiting for move_group action server')
            roscpp_initialize_named(sys.argv)
            self.robot = moveit_commander.RobotCommander()
            connect_move_groups(self.robot, {'arm', self.group_name}, self.moveit_action_startup_timeout)
            self.robot.arm.set_planning_time(self.planning_time)
            self.robot.arm.set_goal_tolerance(arm_goal_tolerance)
            self.scene = moveit_commander.PlanningSceneInterface()
            rospy.loginfo('found move_group action server')
        except RuntimeError:
            rospy.logfatal(
                'grasplan place server could not connect to Moveit in time, exiting! \n' + traceback.format_exc()
            )
            rospy.signal_shutdown('fatal error')
            sys.exit(1)

        self.mtc = None
        if self.use_mtc:
            from grasplan.mtc_pick_place import MtcPickPlace

            self.mtc = MtcPickPlace(
                self.group_name,
                self.robot.get_group(self.group_name).get_end_effector_link(),
                self.robot.get_link_names(group=rospy.get_param('~gripper_group_name', 'gripper')),
                self.robot.get_planning_frame(),
                lambda: False,  # replaced by the requesting action server in run_place_goal
            )
            rospy.loginfo('place: place goals are planned and executed with MoveIt Task Constructor')
        self.place_action_client = actionlib.SimpleActionClient('place', PlaceAction)
        if not self.use_mtc:
            rospy.loginfo(f'waiting for {rospy.resolve_name("place")} action server')
            if not self.place_action_client.wait_for_server(rospy.Duration(self.moveit_action_startup_timeout)):
                rospy.logfatal(
                    f'MoveIt action server {rospy.resolve_name("place")} not found within '
                    f'{self.moveit_action_startup_timeout} s, grasplan place server exiting!'
                )
                rospy.signal_shutdown('fatal error')
                sys.exit(1)
            rospy.loginfo(f'found {rospy.resolve_name("place")} action server')

        # offer action lib server for object placing if needed
        if action_server_required:
            self.place_action_server = actionlib.SimpleActionServer(
                'place_object', PlaceObjectAction, self.place_obj_action_callback, False
            )
            # prepare fallback option because moveit pickup action server ignores preemption requests
            # create joint controller cancellers for both arm and gripper
            ns = rospy.get_namespace().strip('/')  # programatically get robot namespace
            self.action_client_helper = ActionClientHelper(
                ns, self.place_action_server, controller_names=['arm', 'gripper']
            )
            self.place_action_server.start()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def run_place_goal(self, goal, action_server, action_client_helper):
        '''
        execute a moveit_msgs/PlaceGoal with move_group's place action or, in MTC mode, with MoveIt Task
        Constructor. Returns the PlaceResult, or None when the goal was preempted or the action failed to run.
        '''
        if self.mtc is not None:
            self.mtc.is_preempt_requested = action_server.is_preempt_requested
            rospy.loginfo(f'planning place of {goal.attached_object_name} with MTC, {len(goal.place_locations)} poses')
            result = self.mtc.place(goal)
            if action_server.is_preempt_requested():
                return None
            return result
        if not action_client_helper.send_goal_to_rogue_server_and_wait(goal, self.place_action_client, patience_timeout=0.1):
            return None
        return self.place_action_client.get_result()

    def clear_place_poses_markers(self):
        marker_array_msg = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.ns = 'object'
        marker.action = Marker.DELETEALL
        marker_array_msg.markers.append(marker)
        self.marker_array_pub.publish(marker_array_msg)

    def publish_place_box_markers(self, poses, box_size, winning_id=None):
        """Show measured object boxes at candidate poses, or the chosen pose in green."""
        markers = MarkerArray()
        for obj in poses.objects:
            if winning_id is not None and str(obj.instance_id) != winning_id:
                continue
            marker = Marker()
            marker.header.frame_id = poses.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'place_object_boxes'
            marker.id = int(obj.instance_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = obj.pose
            marker.scale.x, marker.scale.y, marker.scale.z = box_size
            if winning_id is None:
                marker.color.r, marker.color.b, marker.color.a = 1.0, 1.0, 0.4
            else:
                marker.color.g, marker.color.a = 1.0, 0.85
            markers.markers.append(marker)
        if markers.markers:
            self.marker_array_pub.publish(markers)

    def add_objs_to_planning_scene(self):
        # query all poses available in pose selector
        resp = self.get_all_poses_pick_pose_selector_srv()
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
                # add all perceived objects to planning scene (one at at time)
                self.scene.add_box(object_name, pose_stamped_msg, object_bounding_box)

    def retract_after_failure(self, what):
        '''
        a failed place or insert that moved the arm leaves it over the table (Oscar 2026-09-28: "retract the arm, not
        leave the arm in the open"): back to ~retract_pose_after_failure (default transport, empty = stays) with the
        guarded named move
        '''
        pose = rospy.get_param('~retract_pose_after_failure', 'transport')
        if not pose:
            return
        rospy.loginfo(f'{what} failed after the arm moved: retracting the arm to {pose}')
        if not self.move_arm_to_posture(pose):
            rospy.logwarn(f'could not retract the arm to {pose} after the failed {what}')

    def picked_open_set(self, object_name):
        '''
        whether object_name was picked open-set, decided like the pick node does: its class is not in the pick node's
        handcoded grasp catalog (rosparam ~pick_grasp_catalog_param, default
        /mobipick/pick_object_node/handcoded_grasp_planner_transforms); False when that catalog cannot be read
        '''
        name = rospy.get_param('~pick_grasp_catalog_param', '/mobipick/pick_object_node/handcoded_grasp_planner_transforms')
        catalog = rospy.get_param(name, None) if name else None
        if not isinstance(catalog, (dict, list)):
            return False
        return separate_object_class_from_id(object_name)[0] not in catalog

    def go_before_place(self, object_name, poses):
        '''
        the arm poses before placing or inserting object_name: the closed-set untangle detour (poses, e.g.
        untangle_cable_guide_1/2_right, the battle-tested hack against cable entanglement), or for an object picked
        open-set only ~open_set_place_start_pose (default transport, empty = none), like the open-set pick (Oscar
        2026-09-28: the fast detour swings shook a picked Pringles can out of the gripper).
        ~open_set_skip_untangle_detour false keeps the detour for every object.
        '''
        if rospy.get_param('~open_set_skip_untangle_detour', True) and self.picked_open_set(object_name):
            start = rospy.get_param('~open_set_place_start_pose', 'transport')
            rospy.loginfo(f'{object_name} was picked open-set: no untangle detour'
                          + (f', moving arm to {start} instead' if start else ''))
            if start:
                self.move_arm_to_posture(start)
            return
        for arm_pose in poses:
            rospy.loginfo(f'Going to intermediate arm pose {arm_pose} to disentangle cable')
            self.move_arm_to_posture(arm_pose)

    def move_arm_to_posture(self, arm_posture_name):
        '''
        use moveit commander to send the arm to a predefined arm configuration
        defined in srdf
        '''
        rospy.loginfo(f'moving arm to {arm_posture_name}')
        # with the MTC cable guard on, the named-pose move is planned and cable-checked before it runs (#121)
        from grasplan.mtc_pick_place import guarded_named_move
        guard = None
        if getattr(self, 'mtc', None) is not None:
            self.mtc.update_cable_guard()
            guard = self.mtc.cable_guard
        return guarded_named_move(self.robot.arm, arm_posture_name, guard)

    def place_obj_action_callback(self, goal):
        success = False
        if self.mtc is not None:
            self.mtc.failure_reason, self.mtc.executed = '', False  # nothing left over from the previous goal
        num_poses_list = [5, 25, 50]  # first try 5 poses, then 25, then 50
        override_disentangle_dont_doit = False
        override_observe_before_place_dont_doit = False
        for i, num_poses in enumerate(num_poses_list):
            if self.place_action_server.is_preempt_requested():
                break
            rospy.logwarn(f'place -> try number: {i + 1}')
            # disentangle cable only on first attempt
            if i == 0:
                override_disentangle_dont_doit = False
                override_observe_before_place_dont_doit = False
            else:
                override_disentangle_dont_doit = True
                override_observe_before_place_dont_doit = True
            # do not disentangle if we dont go to observe arm pose, unless configured to
            if not goal.observe_before_place and not self.disentangle_without_observe:
                override_disentangle_dont_doit = True
            if self.place_object(
                goal.support_surface_name,
                observe_before_place=goal.observe_before_place,
                number_of_poses=num_poses,
                max_batch_size=self.max_batch_size,
                override_disentangle_dont_doit=override_disentangle_dont_doit,
                override_observe_before_place_dont_doit=override_observe_before_place_dont_doit,
            ):
                success = True
                break
            else:
                if self.place_action_server.is_preempt_requested():
                    success = False
                    break
                if self.mtc is not None and self.mtc.executed:
                    break  # the arm moved: the caller decides about a retry (#128)
        if not success and self.mtc is not None and self.mtc.executed and not self.place_action_server.is_preempt_requested():
            self.retract_after_failure('place')
        if success:
            self.place_action_server.set_succeeded(PlaceObjectResult(success=True))
        elif self.place_action_server.is_preempt_requested():
            rospy.logwarn("Preemption requested during place goal processing.")
            self.place_action_server.set_preempted()
        else:
            reason = self.mtc.failure_reason if self.mtc is not None else ''
            self.place_action_server.set_aborted(
                PlaceObjectResult(success=False), f'place failed: {reason}' if reason else 'place failed'
            )

    def order_place_poses_by_reach(self, place_poses: ObjectList):
        """
        ~place_prefer_near_arm (#132): try the place poses nearest to the arm base (~place_reach_frame) first, as MTC
        and MoveIt try them in order; ~place_max_reach > 0 also drops those farther away (unless none is closer).
        """
        if not rospy.get_param('~place_prefer_near_arm', True) or not place_poses.objects:
            return
        reach_frame = rospy.get_param('~place_reach_frame', 'mobipick/ur5_base_link')
        try:
            base = self.tf_buffer.lookup_transform(
                place_poses.header.frame_id, reach_frame, rospy.Time(0), rospy.Duration(1.0)
            ).transform.translation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            rospy.logwarn(f'place poses stay in random order, no transform to {reach_frame}: {error}')
            return
        ordered, dropped = order_by_reach(
            place_poses.objects, (base.x, base.y), rospy.get_param('~place_max_reach', 0.8)
        )
        place_poses.objects = ordered
        nearest = math.hypot(ordered[0].pose.position.x - base.x, ordered[0].pose.position.y - base.y)
        farthest = math.hypot(ordered[-1].pose.position.x - base.x, ordered[-1].pose.position.y - base.y)
        rospy.loginfo(
            f'place poses ordered nearest to {reach_frame} first: {len(ordered)} poses, {nearest:.2f} to '
            f'{farthest:.2f} m' + (f', {dropped} beyond ~place_max_reach dropped' if dropped else '')
        )

    def transform_obj_list(self, obj_list: ObjectList, target_frame_id: str) -> ObjectList:
        """Transforms an object list from a source frame to a target frame."""
        if self.tf_buffer.can_transform(obj_list.header.frame_id, target_frame_id, rospy.Time(0)):
            for obj in obj_list.objects:
                source_pose = PoseStamped(header=obj_list.header, pose=obj.pose)
                tf = self.tf_buffer.lookup_transform(target_frame_id, obj_list.header.frame_id, rospy.Time(0))
                target_pose = tf2_geometry_msgs.do_transform_pose(source_pose, tf)
                obj.pose = target_pose.pose
        else:
            raise RuntimeError(
                f"Error while transformin obj_list, no tf from {target_frame_id} -> {obj_list.header.frame_id}"
            )
        obj_list.header.frame_id = target_frame_id
        return obj_list

    def transform_points(self, points: List[Point], target_frame_id: str, source_frame_id: str) -> List[Point]:
        """Transforms a list of points from a source frame to a target frame."""
        if self.tf_buffer.can_transform(target_frame_id, source_frame_id, rospy.Time(0)):
            transformed_points = []
            for point in points:
                source_point = PointStamped(header=Header(frame_id=source_frame_id), point=point)
                tf = self.tf_buffer.lookup_transform(target_frame_id, source_frame_id, rospy.Time(0))
                target_point = tf2_geometry_msgs.do_transform_point(source_point, tf)
                transformed_points.append(target_point.point)
        else:
            raise RuntimeError(f"Error while transforming points, no tf from {target_frame_id} -> {source_frame_id}")
        return transformed_points

    def add_planning_scene_objects(self, planning_scene) -> None:
        """Adds all objects in the planning scene to the local tf buffer as static transforms."""
        # One query for all objects: a query per object took about 1 s each over Wi-Fi to the real robot
        for obj_name, obj in planning_scene.get_objects().items():
            tf = TransformStamped(
                header=Header(frame_id="map"),
                child_frame_id=obj_name,
                transform=Transform(
                    translation=Vector3(x=obj.pose.position.x, y=obj.pose.position.y, z=obj.pose.position.z),
                    rotation=obj.pose.orientation,
                ),
            )
            self.tf_buffer.set_transform_static(tf, "grasplan")

    def extract_batch(self, global_place_poses, start_index, max_batch_size):
        # Create a new ObjectList for the batch
        batch_poses = ObjectList()
        # Copy the header from the original object
        batch_poses.header = global_place_poses.header
        # Extract the subset of poses while preserving datatype
        batch_poses.objects = global_place_poses.objects[start_index : start_index + max_batch_size]
        return batch_poses

    def place_object(
        self,
        support_object,
        observe_before_place=False,
        number_of_poses=5,
        max_batch_size=20,
        override_disentangle_dont_doit=False,
        override_observe_before_place_dont_doit=False,
    ):
        '''
        Create action lib client and call MoveIt place action server in batches of max_batch_size poses.
        param: max_batch_size: split moveit place poses into small batches to give time in between to check
                               for preemption this is necessary because moveit ignores preemption requests.
        '''
        PLACE_OBJECT_SERVER_NAME = 'place'
        assert isinstance(observe_before_place, bool)
        rospy.loginfo(f'Received request to place object on {support_object}')

        if len(self.scene.get_attached_objects().keys()) == 0:
            rospy.logerr("The robot is not currently holding any object, can't place")
            return False

        # deduce object_to_be_placed by querying which object the gripper currently has attached to its gripper
        object_to_be_placed = list(self.scene.get_attached_objects().keys())[0]
        rospy.loginfo(
            f'received request to place the object that the robot is currently holding : {object_to_be_placed}'
        )

        # TODO: find a better place for adding the static transforms, it shouldn't be done for each place
        self.add_planning_scene_objects(self.scene)

        # clear pose selector before starting to place in case some data is left over from previous runs
        self.place_pose_selector_clear_srv()
        self.clear_place_poses_markers()

        if not override_observe_before_place_dont_doit and observe_before_place:
            # optionally find free space in table: look at table, update planning scene
            self.move_arm_to_posture(self.arm_pose_with_objs_in_fov)
            # activate pick pose selector to observe table
            self.activate_pick_pose_selector_srv(True)
            rospy.sleep(5.0)  # give some time to observe
            self.activate_pick_pose_selector_srv(False)

        # in any case, add all known objects to planning scene before placing
        self.add_objs_to_planning_scene()

        action_client = self.place_action_client

        # generate plane from object surface
        plane = obj_to_plane(support_object, self.scene)

        # scale down plane to account for obj width and length
        plane = adjust_plane(plane, -0.15, -0.15)

        # publish plane as marker for visualization purposes
        self.plane_vis_pub.publish(
            make_plane_marker_msg(
                self.global_reference_frame, self.transform_points(plane, self.global_reference_frame, support_object)
            )
        )

        # generate random place poses within the plane
        object_class_tbp = separate_object_class_from_id(object_to_be_placed)[0]
        open_set_options = {}
        if object_class_tbp not in OBJECT_HEIGHTS:
            steps = max(1, self.open_set_place_yaw_steps)
            open_set_options = {
                'yaw_range': 2.0 * math.pi,
                'yaws': [2.0 * math.pi * step / steps for step in range(steps)],
                'height_offset': self.open_set_place_clearance,
            }
            rospy.loginfo(
                f'{object_class_tbp} is an open-set object: {steps} yaws around the full circle, '
                f'released {self.open_set_place_clearance * 100:.1f} cm above {support_object}'
            )
        local_place_poses = gen_place_poses_from_plane(
            self.place_action_server,
            object_class_tbp,
            support_object,
            plane,
            self.scene,
            frame_id=support_object,
            number_of_poses=number_of_poses,
            min_dist=self.min_dist,
            ignore_min_dist_list=self.ignore_min_dist_list,
            **open_set_options,
        )

        if self.place_action_server.is_preempt_requested():
            rospy.logwarn('Preemption requested. Cancelling goal.')
            action_client.cancel_goal()
            return False

        global_place_poses = self.transform_obj_list(local_place_poses, self.global_reference_frame)
        self.order_place_poses_by_reach(global_place_poses)
        self.place_poses_pub.publish(global_place_poses)
        attached_object = self.scene.get_attached_objects([object_to_be_placed])[object_to_be_placed].object
        box_size = list(attached_object.primitives[0].dimensions)
        if object_class_tbp not in OBJECT_HEIGHTS:
            self.publish_place_box_markers(global_place_poses, box_size)

        # Keep the observed occupancy map by default so unknown objects remain
        # collision obstacles during placement.  Clearing is retained as an
        # opt-in diagnostic/workaround for deployments that explicitly need it.
        if self.clear_octomap_flag:
            rospy.logwarn('Clearing octomap before placing')
            rospy.ServiceProxy('clear_octomap', Empty)()

        if not self.use_mtc and not action_client.wait_for_server(
            timeout=rospy.Duration(self.moveit_action_server_timeout)
        ):
            rospy.logerr(
                f'Action server {rospy.resolve_name(PLACE_OBJECT_SERVER_NAME)} not available within '
                f'{self.moveit_action_server_timeout} s'
            )
            return False

        rospy.loginfo(f'Found {PLACE_OBJECT_SERVER_NAME} action server')

        for i in range(0, len(global_place_poses.objects), max_batch_size):
            # for 25 objs and max_batch_size of 10, "i" would be 0 in the first loop, 10 in the 2nd and 20 in the last
            if i and object_class_tbp not in OBJECT_HEIGHTS:
                self.publish_place_box_markers(global_place_poses, box_size)
            batch_poses = self.extract_batch(global_place_poses, i, max_batch_size)
            goal = self.make_place_goal_msg(
                object_to_be_placed, support_object, batch_poses, use_path_constraints=self.use_path_constraints
            )

            # allow disentangle to happen only on first place attempt, no need to do it every time
            if not override_disentangle_dont_doit:
                # go to intermediate arm poses if needed to disentangle arm cable
                if self.disentangle_required:
                    self.go_before_place(object_to_be_placed, self.poses_to_go_before_place)

            rospy.loginfo(
                f'Sending goal with batch of {len(batch_poses.objects)} poses to place, '
                f'total poses = {len(global_place_poses.objects)}, '
                f'batch {i // max_batch_size + 1}/'
                f'{(len(global_place_poses.objects) + max_batch_size - 1) // max_batch_size}, '
                f'{(i + len(batch_poses.objects)) * 100 // len(global_place_poses.objects)}% completed'
            )
            result = self.run_place_goal(goal, self.place_action_server, self.action_client_helper)
            if result is None:
                rospy.logerr('Failed to run the place goal')
                return False

            if self.place_action_server.is_preempt_requested():
                rospy.logwarn('Preemption requested. Cancelling goal.')
                action_client.cancel_goal()
                return False

            # rospy.loginfo(f'{PLACE_OBJECT_SERVER_NAME} is done with execution, resuĺt was = "{result}"')

            # # ------ result handling

            # # The result of the place attempt
            # MoveItErrorCodes error_code

            # # The full starting state of the robot at the start of the trajectory
            # RobotState trajectory_start

            # # The trajectory that moved group produced for execution
            # RobotTrajectory[] trajectory_stages

            # string[] trajectory_descriptions

            # # The successful place location, if any
            # PlaceLocation place_location

            # # The amount of time in seconds it took to complete the plan
            # float64 planning_time

            # ---

            # how to know if place was successful from result?
            # if result.success:
            # rospy.loginfo(f'Succesfully placed {object_to_be_placed}')
            # else:
            # rospy.logerr(f'Failed to place {object_to_be_placed}')

            # ---

            # handle moveit pick result

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo('Successfully placed object')
                self.place_pose_selector_clear_srv()
                self.clear_place_poses_markers()
                winning_id = result.place_location.id
                self.publish_place_box_markers(global_place_poses, box_size, winning_id=winning_id)
                return True
            else:
                rospy.logerr('Place object failed')
                self.place_pose_selector_clear_srv()
                self.clear_place_poses_markers()
                print_moveit_error(result.error_code.val)
                if self.mtc is not None and self.mtc.executed:
                    # the arm moved (maybe the gripper already opened): report instead of retrying from a changed state
                    return False

        rospy.logerr('All batches processed, but no successful placement')
        return False

    def make_constraints_msg(self):  # TODO: not yet tested on mobipick
        constraints_msg = Constraints()
        constraints_msg.name = 'keep_object_upright'

        now = rospy.Time.now()
        self.tf_listener.waitForTransform(self.arm_name + '_base_link', 'hand_ee_link', now, rospy.Duration(2.0))
        _, rot = self.tf_listener.lookupTransform(self.arm_name + '_base_link', 'hand_ee_link', now)

        orientation_constraint_msg = OrientationConstraint()
        orientation_constraint_msg.header.frame_id = self.arm_name + '_base_link'
        orientation_constraint_msg.orientation.x = rot[0]
        orientation_constraint_msg.orientation.y = rot[1]
        orientation_constraint_msg.orientation.z = rot[2]
        orientation_constraint_msg.orientation.w = rot[3]
        orientation_constraint_msg.link_name = 'hand_ee_link'
        orientation_constraint_msg.absolute_x_axis_tolerance = 6.28
        orientation_constraint_msg.absolute_y_axis_tolerance = 0.2
        orientation_constraint_msg.absolute_z_axis_tolerance = 6.28
        orientation_constraint_msg.parameterization = 0
        orientation_constraint_msg.weight = 1.0

        constraints_msg.orientation_constraints.append(orientation_constraint_msg)

        return constraints_msg

    def make_place_goal_msg(
        self, object_to_be_placed, support_object, place_poses_as_object_list_msg, use_path_constraints
    ):
        '''
        fill place action lib goal, see: https://github.com/ros-planning/moveit_msgs/blob/master/action/Place.action
        '''
        assert isinstance(object_to_be_placed, str)
        assert isinstance(support_object, str)

        goal = PlaceGoal()

        goal.group_name = self.group_name

        # the name of the attached object to place
        goal.attached_object_name = object_to_be_placed

        # a list of possible transformations for placing the object
        # NOTE: multiple place locations are possible to be defined, we just define 1 for now
        # ---
        place_locations = []
        frame_id = place_poses_as_object_list_msg.header.frame_id  # 'mobipick/base_link'
        # object_class = separate_object_class_from_id(object_to_be_placed)[0]
        # for obj in self.gen_place_poses(object_class, frame_id=frame_id).objects:
        for obj in place_poses_as_object_list_msg.objects:
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = frame_id
            pose_stamped_msg.pose = obj.pose
            place_locations.append(self.make_place_location_msg(pose_stamped_msg, location_id=str(obj.instance_id)))

        # translation = [0.0, -0.9, 0.88] # works for simple pick n place demo
        # rotation = [-0.5, -0.5, 0.5, 0.5]
        goal.place_locations = place_locations

        # if the user prefers setting the eef pose (same as in pick) rather than
        # the location of the object, this flag should be set to true
        # bool place_eef
        goal.place_eef = False

        # the name that the support surface (e.g. table) has in the collision world
        # can be left empty if no name is available
        # string support_surface_name
        goal.support_surface_name = support_object

        # whether collisions between the gripper and the support surface should be acceptable
        # during move from pre-place to place and during retreat. Collisions when moving to the
        # pre-place location are still not allowed even if this is set to true.
        # bool allow_gripper_support_collision
        goal.allow_gripper_support_collision = False

        # Optional constraints to be imposed on every point in the motion plan
        # Constraints path_constraints
        if use_path_constraints:
            goal.path_constraints = self.make_constraints_msg()  # add orientation constraints

        # The name of the motion planner to use. If no name is specified,
        # a default motion planner will be used
        # string planner_id
        # goal.planner_id = 'RRTConnect'

        # an optional list of obstacles that we have semantic information about
        # and that can be touched/pushed/moved in the course of placing
        # string[] allowed_touch_objects
        goal.allowed_touch_objects = []

        # The maximum amount of time the motion planner is allowed to plan for
        # float64 allowed_planning_time
        goal.allowed_planning_time = self.planning_time

        # Planning options
        # PlanningOptions planning_options
        goal.planning_options = self.make_planning_options_msg()

        return goal

    def make_planning_options_msg(self):
        '''
        see: https://github.com/ros-planning/moveit_msgs/blob/master/msg/PlanningOptions.msg
        '''
        planning_options_msg = PlanningOptions()

        # The diff to consider for the planning scene (optional)
        # PlanningScene planning_scene_diff
        # planning_options_msg.planning_scene_diff =
        # NOTE: It's important to set is_diff = True, otherwise MoveIt will
        # overwrite its planning scene with this one (empty), thereby ignoring
        # collisions with e.g. the octomap
        planning_options_msg.planning_scene_diff.is_diff = True
        planning_options_msg.planning_scene_diff.robot_state.is_diff = True

        # If this flag is set to true, the action
        # returns an executable plan in the response but does not attempt execution
        # bool plan_only
        planning_options_msg.plan_only = False

        # If this flag is set to true, the action of planning &
        # executing is allowed to look around  (move sensors) if
        # it seems that not enough information is available about
        # the environment
        # bool look_around
        planning_options_msg.look_around = False

        # If this value is positive, the action of planning & executing
        # is allowed to look around for a maximum number of attempts;
        # If the value is left as 0, the default value is used, as set
        # with dynamic_reconfigure
        # int32 look_around_attempts
        planning_options_msg.look_around_attempts = 0

        # If set and if look_around is true, this value is used as
        # the maximum cost allowed for a path to be considered executable.
        # If the cost of a path is higher than this value, more sensing or
        # a new plan needed. If left as 0.0 but look_around is true, then
        # the default value set via dynamic_reconfigure is used
        # float64 max_safe_execution_cost
        planning_options_msg.max_safe_execution_cost = 0.0

        # If the plan becomes invalidated during execution, it is possible to have
        # that plan recomputed and execution restarted. This flag enables this
        # functionality
        # bool replan
        planning_options_msg.replan = False

        # The maximum number of replanning attempts
        # int32 replan_attempts
        planning_options_msg.replan_attempts = 0

        # The amount of time to wait in between replanning attempts (in seconds)
        # float64 replan_delay
        planning_options_msg.replan_delay = 2.0

        return planning_options_msg

    def make_place_location_msg(self, place_pose, allowed_touch_objects=None, location_id='1'):
        '''
        see: https://github.com/ros-planning/moveit_msgs/blob/master/msg/PlaceLocation.msg
        '''

        if allowed_touch_objects is None:
            allowed_touch_objects = []
        assert isinstance(place_pose, PoseStamped)
        assert isinstance(allowed_touch_objects, list)
        place_msg = PlaceLocation()

        # Identify the candidate so MoveIt's successful result can highlight it
        # string id
        place_msg.id = location_id

        # The internal posture of the hand for the grasp
        # positions and efforts are used
        # trajectory_msgs/JointTrajectory post_place_posture
        place_msg.post_place_posture = self.make_gripper_trajectory_msg(self.gripper_release_distance)
        # NOTE in simple pick n place demo this value is 0.1 m

        # The position of the end-effector for the grasp relative to a reference frame
        # (that is always specified elsewhere, not in this message)
        # geometry_msgs/PoseStamped place_pose
        place_msg.place_pose = place_pose

        # The estimated probability of success for this place, or some other
        # measure of how "good" it is.
        # float64 quality
        place_msg.quality = 1.0

        # The approach motion
        # GripperTranslation pre_place_approach
        # TODO after tables demo: make robot place from the left as well by parameterizing this value
        place_msg.pre_place_approach = self.make_gripper_translation_msg('mobipick/base_link', 0.2, vector_z=-1.0)

        # The retreat motion
        # GripperTranslation post_place_retreat
        place_msg.post_place_retreat = self.make_gripper_translation_msg('mobipick/gripper_tcp', 0.25, vector_x=-1.0)

        # an optional list of obstacles that we have semantic information about
        # and that can be touched/pushed/moved in the course of grasping
        # string[] allowed_touch_objects
        place_msg.allowed_touch_objects = allowed_touch_objects

        return copy.deepcopy(place_msg)

    def make_gripper_trajectory_msg(self, gripper_actuation_distance):
        '''
        Set and return the gripper posture as a trajectory_msgs/JointTrajectory
        only one point is set which is the final gripper target

        Warning: Contrary to the definition of trajectory_msgs/JointTrajectory,
        the position is the gripper opening gap in meters, not the joint angle
        in radian, if MoveIt is configured with a control_msgs/GripperCommand
        controller (e.g., on Mobipick).
        '''
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joint_names
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [gripper_actuation_distance]
        trajectory_point.effort = self.gripper_joint_efforts
        trajectory_point.time_from_start = rospy.Duration(1.0)  # NOTE in simple pick n place demo this value is 5 s
        trajectory.points.append(trajectory_point)
        return trajectory

    def make_gripper_translation_msg(
        self, frame_id, distance, vector_x=0.0, vector_y=0.0, vector_z=0.0, min_distance=0.1
    ):
        '''
        see: https://github.com/ros-planning/moveit_msgs/blob/master/msg/GripperTranslation.msg
        '''
        gripper_translation_msg = GripperTranslation()

        # defines a translation for the gripper, used in pickup or place tasks
        # for example for lifting an object off a table or approaching the table for placing

        # the direction of the translation
        # geometry_msgs/Vector3Stamped direction
        vs = Vector3Stamped()
        vs.header.frame_id = frame_id
        vs.vector.x = vector_x
        vs.vector.y = vector_y
        vs.vector.z = vector_z
        gripper_translation_msg.direction = vs

        # the desired translation distance
        # float32 desired_distance
        gripper_translation_msg.desired_distance = distance

        # the min distance that must be considered feasible before the
        # grasp is even attempted
        # float32 min_distance
        gripper_translation_msg.min_distance = min_distance

        return gripper_translation_msg

    def start_place_node(self):
        # wait for trigger action lib
        rospy.loginfo('ready to place objects')
        rospy.spin()
        # shutdown moveit cpp interface before exit
        # moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    rospy.init_node('place_object_node', anonymous=False)
    place = PlaceTools()
    place.start_place_node()
