#!/usr/bin/env python3

'''
example on how to place an object using grasplan and moveit
'''

import sys
import copy
import random
import tf
import rospy
import actionlib
import moveit_commander
import traceback

from grasplan.tools.support_plane_tools import obj_to_plane, adjust_plane_area_by_distance, gen_place_poses_from_plane, make_plane_marker_msg, compute_object_height
from grasplan.common_grasp_tools import separate_object_class_from_id
from grasplan.tools.moveit_errors import print_moveit_error
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool, Trigger
from object_pose_msgs.msg import ObjectList
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceLocation, GripperTranslation, PlanningOptions, Constraints, OrientationConstraint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
from grasplan.msg import PlaceObjectAction, PlaceObjectResult
from moveit_msgs.msg import MoveItErrorCodes
from pose_selector.srv import GetPoses
from visualization_msgs.msg import Marker, MarkerArray

class PlaceTools():
    def __init__(self, action_server_required=True):
        self.global_reference_frame = rospy.get_param('~global_reference_frame', 'map')
        self.arm_pose_with_objs_in_fov = rospy.get_param('~arm_pose_with_objs_in_fov', 'observe100cm_right')
        self.timeout = rospy.get_param('~timeout', 50.0) # in seconds
        self.min_dist = rospy.get_param('~min_dist', 0.2)
        self.ignore_min_dist_list = rospy.get_param('~ignore_min_dist_list', ['foo_obj'])
        self.group_name = rospy.get_param('~group_name', 'arm')
        self.arm_name = rospy.get_param('~arm_name', 'ur10e')
        self.gripper_joint_names = rospy.get_param('~gripper_joint_names')
        self.gripper_joint_efforts = rospy.get_param('~gripper_joint_efforts')
        self.place_object_server_name = rospy.get_param('~place_object_server_name', 'place') # /mobipick/place
        self.gripper_release_distance = rospy.get_param('~gripper_release_distance', 0.1)
        self.planning_time = rospy.get_param('~planning_time', 20.0)
        arm_goal_tolerance = rospy.get_param('~arm_goal_tolerance', 0.01)
        self.disentangle_required = rospy.get_param('~disentangle_required', False)
        self.poses_to_go_before_place = rospy.get_param('~poses_to_go_before_place', [])

        self.plane_vis_pub = rospy.Publisher('~support_plane_as_marker', Marker, queue_size=1, latch=True)
        self.place_poses_pub = rospy.Publisher('~place_poses', ObjectList, queue_size=50)
        self.marker_array_pub = rospy.Publisher('/place_pose_selector_objects', MarkerArray, queue_size=1)

        # service clients
        place_pose_selector_activate_srv_name = rospy.get_param('~place_pose_selector_activate_srv_name', '/place_pose_selector_activate')
        place_pose_selector_clear_srv_name = rospy.get_param('~place_pose_selector_clear_srv_name', '/place_pose_selector_clear')
        pick_pose_selector_activate_srv_name = rospy.get_param('~pick_pose_selector_activate_srv_name', '/pick_pose_selector_activate')
        pick_pose_selector_get_all_poses_srv_name = rospy.get_param('~pick_pose_selector_get_all_poses_srv_name', '/pose_selector_get_all')
        rospy.loginfo(f'waiting for pose selector services: {place_pose_selector_activate_srv_name}, {place_pose_selector_clear_srv_name}\
                      {pick_pose_selector_activate_srv_name}, {pick_pose_selector_get_all_poses_srv_name}')
        rospy.wait_for_service(place_pose_selector_activate_srv_name, 30.0)
        rospy.wait_for_service(place_pose_selector_clear_srv_name, 30.0)
        rospy.wait_for_service(pick_pose_selector_activate_srv_name, 30.0)
        rospy.wait_for_service(pick_pose_selector_get_all_poses_srv_name, 30.0)
        try:
            self.activate_place_pose_selector_srv = rospy.ServiceProxy(place_pose_selector_activate_srv_name, SetBool)
            self.place_pose_selector_clear_srv = rospy.ServiceProxy(place_pose_selector_clear_srv_name, Trigger)
            self.activate_pick_pose_selector_srv = rospy.ServiceProxy(pick_pose_selector_activate_srv_name, SetBool)
            self.get_all_poses_pick_pose_selector_srv = rospy.ServiceProxy(pick_pose_selector_get_all_poses_srv_name, GetPoses)
            rospy.loginfo('found pose selector services')
        except rospy.exceptions.ROSException:
            rospy.logfatal('grasplan place server could not find pose selector services in time, exiting! \n' + traceback.format_exc())
            rospy.signal_shutdown('fatal error')

        # activate place pose selector to be ready to store the place poses
        resp = self.activate_place_pose_selector_srv(True)

        # wait for moveit to become available, TODO: find a cleaner way to wait for moveit
        rospy.wait_for_service('move_group/planning_scene_monitor/set_parameters', 30.0)
        rospy.sleep(2.0)

        try:
            rospy.loginfo('waiting for move_group action server')
            moveit_commander.roscpp_initialize(sys.argv)
            self.robot = moveit_commander.RobotCommander()
            self.robot.arm.set_planning_time(self.planning_time)
            self.robot.arm.set_goal_tolerance(arm_goal_tolerance)
            self.scene = moveit_commander.PlanningSceneInterface()
            rospy.loginfo('found move_group action server')
        except RuntimeError:
            rospy.logfatal('grasplan place server could not connect to Moveit in time, exiting! \n' + traceback.format_exc())
            rospy.signal_shutdown('fatal error')

        # offer action lib server for object placing if needed
        if action_server_required:
            self.place_action_server = actionlib.SimpleActionServer('place_object', PlaceObjectAction, self.place_obj_action_callback, False)
            self.place_action_server.start()

        self.tf_listener = tf.TransformListener()
        
    def clear_place_poses_markers(self):
        marker_array_msg = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.ns = 'object'
        marker.action = Marker.DELETEALL
        marker_array_msg.markers.append(marker)
        self.marker_array_pub.publish(marker_array_msg)

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

    def move_arm_to_posture(self, arm_posture_name):
        '''
        use moveit commander to send the arm to a predefined arm configuration
        defined in srdf
        '''
        rospy.loginfo(f'moving arm to {arm_posture_name}')
        self.robot.arm.set_named_target(arm_posture_name)
        # attempt to move it 2 times, (sometimes fails with only 1 time)
        if not self.robot.arm.go():
            rospy.logwarn(f'failed to move arm to posture: {arm_posture_name}, will retry one more time in 1 sec')
            rospy.sleep(1.0)
            self.robot.arm.go()

    def place_obj_action_callback(self, goal):
        success = False
        num_poses_list = [5, 25, 50] # first try 5 poses, then 25, then 50
        override_disentangle_dont_doit = False
        override_observe_before_place_dont_doit = False
        for i, num_poses in enumerate(num_poses_list):
            rospy.logwarn(f'place -> try number: {i + 1}')
            # disentangle cable only on first attempt
            if i == 0:
                override_disentangle_dont_doit = False
                override_observe_before_place_dont_doit = False
            else:
                override_disentangle_dont_doit = True
                override_observe_before_place_dont_doit = True
            # do not disentangle if we dont go to observe arm pose
            if not goal.observe_before_place:
                override_disentangle_dont_doit = True
            if self.place_object(goal.support_surface_name, observe_before_place=goal.observe_before_place,\
                                 number_of_poses=num_poses, override_disentangle_dont_doit=override_disentangle_dont_doit,\
                                 override_observe_before_place_dont_doit=override_observe_before_place_dont_doit):
                success = True
                break
        if success:
            self.place_action_server.set_succeeded(PlaceObjectResult(success=True))
        else:
            self.place_action_server.set_aborted(PlaceObjectResult(success=False))

    def place_object(self, support_object, observe_before_place=False, number_of_poses=5,\
                     override_disentangle_dont_doit=False, override_observe_before_place_dont_doit=False):
        '''
        create action lib client and call moveit place action server
        '''
        assert isinstance(observe_before_place, bool)
        rospy.loginfo(f'received request to place object on {support_object}')

        if len(self.scene.get_attached_objects().keys()) == 0:
            rospy.logerr("the robot is not currently holding any object, can't place")
            return False

        # deduce object_to_be_placed by querying which object the gripper currently has attached to its gripper
        object_to_be_placed = list(self.scene.get_attached_objects().keys())[0]
        rospy.loginfo(f'received request to place the object that the robot is currently holding : {object_to_be_placed}')

        # clear pose selector before starting to place in case some data is left over from previous runs
        self.place_pose_selector_clear_srv()

        if not override_observe_before_place_dont_doit:
            if observe_before_place:
                # optionally find free space in table: look at table, update planning scene
                self.move_arm_to_posture(self.arm_pose_with_objs_in_fov)
                # activate pick pose selector to observe table
                resp = self.activate_pick_pose_selector_srv(True)
                rospy.sleep(0.5) # give some time to observe
                resp = self.activate_pick_pose_selector_srv(False)
                self.add_objs_to_planning_scene()

        action_client = actionlib.SimpleActionClient(self.place_object_server_name, PlaceAction)
        rospy.loginfo(f'sending place goal to {self.place_object_server_name} action server')

        # generate plane from object surface
        plane = obj_to_plane(support_object)
        # scale down plane to account for obj width and length
        plane = adjust_plane_area_by_distance(plane, 0.05)
        # publish plane as marker for visualisation purposes
        self.plane_vis_pub.publish(make_plane_marker_msg(self.global_reference_frame, plane))
        # generate random place poses within a plane
        object_class_tbp = separate_object_class_from_id(object_to_be_placed)[0]
        place_poses_as_object_list_msg = gen_place_poses_from_plane(object_class_tbp, support_object, plane,\
                frame_id=self.global_reference_frame, number_of_poses=number_of_poses, obj_height=compute_object_height(object_class_tbp), \
                min_dist=self.min_dist, ignore_min_dist_list=self.ignore_min_dist_list)

        self.place_poses_pub.publish(place_poses_as_object_list_msg)

        # clear octomap before placing, this is experimental and not sure is needed
        rospy.logwarn('clearing octomap')
        rospy.ServiceProxy('clear_octomap', Empty)()

        if action_client.wait_for_server(timeout=rospy.Duration.from_sec(2.0)):
            rospy.loginfo(f'found {self.place_object_server_name} action server')
            goal = self.make_place_goal_msg(object_to_be_placed, support_object, place_poses_as_object_list_msg, use_path_constraints=True)

            # allow disentangle to happen only on first place attempt, no need to do it every time
            if not override_disentangle_dont_doit:
                # go to intermediate arm poses if needed to disentangle arm cable
                if self.disentangle_required:
                    for arm_pose in self.poses_to_go_before_place:
                        rospy.loginfo(f'going to intermediate arm pose {arm_pose} to disentangle cable')
                        self.move_arm_to_posture(arm_pose)

            rospy.loginfo(f'sending place {object_to_be_placed} goal to {self.place_object_server_name} action server')
            action_client.send_goal(goal)
            rospy.loginfo(f'waiting for result from {self.place_object_server_name} action server')
            if action_client.wait_for_result(rospy.Duration.from_sec(self.timeout)):
                result = action_client.get_result()
                # rospy.loginfo(f'{self.place_object_server_name} is done with execution, resuĺt was = "{result}"')

                ## ------ result handling

                ## The result of the place attempt
                # MoveItErrorCodes error_code

                ## The full starting state of the robot at the start of the trajectory
                # RobotState trajectory_start

                ## The trajectory that moved group produced for execution
                # RobotTrajectory[] trajectory_stages

                # string[] trajectory_descriptions

                ## The successful place location, if any
                # PlaceLocation place_location

                ## The amount of time in seconds it took to complete the plan
                # float64 planning_time

                # ---

                # how to know if place was successful from result?
                # if result.success == True:
                    # rospy.loginfo(f'Succesfully placed {object_to_be_placed}')
                # else:
                    # rospy.logerr(f'Failed to place {object_to_be_placed}')

                # ---

                # handle moveit pick result
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo(f'Successfully placed object')
                    self.place_pose_selector_clear_srv()
                    # clear possible place poses markers in rviz
                    self.clear_place_poses_markers()
                    return True
                else:
                    rospy.logerr(f'place object failed')
                    self.place_pose_selector_clear_srv()
                    # clear possible place poses markers in rviz
                    self.clear_place_poses_markers()
                    print_moveit_error(result.error_code.val)
                return False
        else:
            rospy.logerr(f'action server {self.place_object_server_name} not available')
            return False
        return False

    def make_constraints_msg(self):
        constraints_msg = Constraints()
        constraints_msg.name = 'keep_bag_upright'

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

    def make_place_goal_msg(self, object_to_be_placed, support_object, place_poses_as_object_list_msg, use_path_constraints):
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
        frame_id = place_poses_as_object_list_msg.header.frame_id # 'mobipick/base_link'
        object_class = separate_object_class_from_id(object_to_be_placed)[0]
        # for obj in self.gen_place_poses(object_class, frame_id=frame_id).objects:
        for obj in place_poses_as_object_list_msg.objects:
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = frame_id
            pose_stamped_msg.pose = obj.pose
            place_locations.append(self.make_place_location_msg(pose_stamped_msg))

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
            goal.path_constraints = self.make_constraints_msg() # add orientation constraints

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

    def make_place_location_msg(self, place_pose, allowed_touch_objects=None):
        '''
        see: https://github.com/ros-planning/moveit_msgs/blob/master/msg/PlaceLocation.msg
        '''

        if allowed_touch_objects is None:
            allowed_touch_objects = []
        assert isinstance(place_pose, PoseStamped)
        assert isinstance(allowed_touch_objects, list)
        place_msg = PlaceLocation()

        # A name for this grasp
        # string id
        place_msg.id = '1'

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
        place_msg.pre_place_approach = self.make_gripper_translation_msg(self.arm_name + '_base_link', 0.2, vector_z=-1.0)

        # The retreat motion
        # GripperTranslation post_place_retreat
        place_msg.post_place_retreat = self.make_gripper_translation_msg('world', 0.25, vector_z=1.0)

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

    def make_gripper_translation_msg(self, frame_id, distance, vector_x=0.0, vector_y=0.0, vector_z=0.0, min_distance=0.1):
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

if __name__=='__main__':
    rospy.init_node('place_object_node', anonymous=False)
    place = PlaceTools()
    place.start_place_node()
