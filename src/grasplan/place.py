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

from grasplan.tools.support_plane_tools import obj_to_plane, reduce_plane_area, gen_place_poses_from_plane, make_plane_marker_msg
from grasplan.common_grasp_tools import separate_object_class_from_id
from grasplan.tools.moveit_errors import print_moveit_error
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
from object_pose_msgs.msg import ObjectList, ObjectPose
from geometry_msgs.msg import Vector3Stamped, PoseStamped, Pose
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceLocation, GripperTranslation, PlanningOptions
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
from pbr_msgs.msg import PlaceObjectAction, PlaceObjectResult
from moveit_msgs.msg import MoveItErrorCodes

class PlaceTools():
    def __init__(self):
        self.global_reference_frame = rospy.get_param('~global_reference_frame', 'map')
        self.timeout = rospy.get_param('~timeout', 50.0) # in seconds
        self.group_name = rospy.get_param('~group_name', 'arm')
        self.gripper_joint_names = rospy.get_param('~gripper_joint_names')
        self.gripper_joint_efforts = rospy.get_param('~gripper_joint_efforts')
        self.place_object_server_name = rospy.get_param('~place_object_server_name', 'place') # /mobipick/place
        self.gripper_release_distance = rospy.get_param('gripper_release_distance', 0.1)

        pose_selector_activate_name = rospy.get_param('~pose_selector_activate_srv_name', '/pose_selector_activate')
        rospy.loginfo(f'waiting for pose selector service: {pose_selector_activate_name}')
        rospy.wait_for_service(pose_selector_activate_name, 2.0)
        self.activate_pose_selector_srv = rospy.ServiceProxy(pose_selector_activate_name, SetBool)
        rospy.loginfo(f'found pose selector services: {pose_selector_activate_name  }')
        # activate place pose selector to be ready to store the place poses
        resp = self.activate_pose_selector_srv(True)

        moveit_commander.roscpp_initialize(sys.argv)
        self.arm = moveit_commander.MoveGroupCommander(self.group_name, wait_for_servers=20.0)

        self.plane_vis_pub = rospy.Publisher('~support_plane_as_marker', Marker, queue_size=1, latch=True)
        self.place_poses_pub = rospy.Publisher('~place_poses', ObjectList, queue_size=50, latch=True)

        try:
            rospy.loginfo('waiting for move_group action server')
            moveit_commander.roscpp_initialize(sys.argv)
            #self.robot = moveit_commander.RobotCommander()
            #self.robot.arm.set_planning_time(planning_time)
            #self.robot.arm.set_goal_tolerance(arm_goal_tolerance)
            self.scene = moveit_commander.PlanningSceneInterface()
            rospy.loginfo('found move_group action server')
        except RuntimeError:
            rospy.logfatal('grasplan place server could not connect to Moveit in time, exiting! \n' + traceback.format_exc())
            rospy.signal_shutdown('fatal error')

        # offer action lib server for object placing
        self.place_action_server = actionlib.SimpleActionServer('place_object', PlaceObjectAction, self.place_obj_action_callback, False)
        self.place_action_server.start()

    def place_obj_action_callback(self, goal):
        if self.place_object(goal.support_surface_name):
            self.place_action_server.set_succeeded(PlaceObjectResult(success=True))
        else:
            self.place_action_server.set_aborted(PlaceObjectResult(success=False))

    def place_object(self, support_object):
        '''
        create action lib client and call moveit place action server
        '''
        rospy.loginfo('received request to place object')

        if len(self.scene.get_attached_objects().keys()) == 0:
            rospy.logerr("the robot is not currently holding any object, can't place")
            return False

        # deduce object_to_be_placed by querying which object the gripper currently has attached to its gripper
        object_to_be_placed = list(self.scene.get_attached_objects().keys())[0]
        rospy.loginfo(f'received request to place the object that the robot is currently holding : {object_to_be_placed}')

        action_client = actionlib.SimpleActionClient(self.place_object_server_name, PlaceAction)
        rospy.loginfo(f'sending place goal to {self.place_object_server_name} action server')
        object_to_pick = object_to_be_placed

        # moveit::planning_interface::MoveGroupInterface& group TODO: missing?
        # group.setSupportSurfaceName('table_1');

        # generate plane from object surface
        plane = obj_to_plane(support_object)
        # scale down plane to account for obj width and length
        plane = reduce_plane_area(plane, -0.2)
        # publish plane as marker for visualisation purposes
        self.plane_vis_pub.publish(make_plane_marker_msg(self.global_reference_frame, plane))
        # generate random place poses within a plane
        object_class_tbp = separate_object_class_from_id(object_to_be_placed)[0]
        # TODO: compute object height
        place_poses_as_object_list_msg = gen_place_poses_from_plane(object_class_tbp, plane, frame_id=self.global_reference_frame, number_of_poses=5, obj_height=0.85) # 0.83
        # send places poses to place pose selector for visualisation purposes
        self.place_poses_pub.publish(place_poses_as_object_list_msg)

        # clear octomap before placing, this is experimental and not sure is needed
        rospy.loginfo('clearing octomap')
        rospy.ServiceProxy('clear_octomap', Empty)()

        if action_client.wait_for_server(timeout=rospy.Duration.from_sec(2.0)):
            rospy.loginfo(f'found {self.place_object_server_name} action server')
            goal = self.make_place_goal_msg(object_to_be_placed, support_object, place_poses_as_object_list_msg)

            self.arm.set_support_surface_name('table_1') # TODO

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
                    return True
                else:
                    rospy.logerr(f'place object failed')
                    print_moveit_error(result.error_code.val)
                return False
        else:
            rospy.logerr(f'action server {self.place_object_server_name} not available')
            return False
        return False

    def gen_place_poses(self):
        '''
        TODO: this is a test function, remove!
        x: from -0.4 to 0.7
        y: from -1.1 to -0.5
        z: from 0.72 to 1.0
        '''
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = 'mobipick/base_link'
        for i in range(1, 50):
            pose_msg = Pose()
            pose_msg.position.x = round(random.uniform(-0.4,   0.7), 2)
            pose_msg.position.y = round(random.uniform(-1.1,  -0.5), 2)
            pose_msg.position.z = round(random.uniform( 0.72,  1.0), 2)
            # roll = round(random.uniform( 0.0,  3.1415), 2)
            # pitch = round(random.uniform( 0.0,  3.1415), 2)
            # yaw = round(random.uniform( 0.0,  3.1415), 2)
            # angular_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            # ---
            # angular_q = [0,0,0,1] # power_drill laying down
            # ---
            # works for power_drill standing up correctly
            roll = 1.5708
            pitch = 0.0
            yaw = 0.0
            angular_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            pose_msg.orientation.x = angular_q[0]
            pose_msg.orientation.y = angular_q[1]
            pose_msg.orientation.z = angular_q[2]
            pose_msg.orientation.w = angular_q[3]
            pose_array_msg.poses.append(copy.deepcopy(pose_msg))
        self.place_poses_pub.publish(pose_array_msg)
        return pose_array_msg

    def make_place_goal_msg(self, object_to_be_placed, support_object, place_poses_as_object_list_msg):
        '''
        fill place action lib goal, see: https://github.com/ros-planning/moveit_msgs/blob/master/action/Place.action
        '''

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
        goal.support_surface_name = 'table_1' # TODO

        # whether collisions between the gripper and the support surface should be acceptable
        # during move from pre-place to place and during retreat. Collisions when moving to the
        # pre-place location are still not allowed even if this is set to true.
        # bool allow_gripper_support_collision
        goal.allow_gripper_support_collision = True

        # Optional constraints to be imposed on every point in the motion plan
        # Constraints path_constraints
        # NOTE: mobipick simple pick n place demo does indeed has this value set
        # goal.path_constraints =

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
        goal.allowed_planning_time = 20.0

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

    def make_place_location_msg(self, place_pose, allowed_touch_objects=[]):
        '''
        see: https://github.com/ros-planning/moveit_msgs/blob/master/msg/PlaceLocation.msg
        '''
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
        # NOTE in simple pick n place demo this value is 0.1

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
        '''
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joint_names
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [gripper_actuation_distance]
        trajectory_point.effort = self.gripper_joint_efforts
        trajectory_point.time_from_start = rospy.Duration(1.0)
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

    def start_pick_node(self):
        # wait for trigger via topic or action lib
        rospy.loginfo('ready to place objects')
        rospy.spin()
        # shutdown moveit cpp interface before exit
        # moveit_commander.roscpp_shutdown()

if __name__=='__main__':
    rospy.init_node('place_object_node', anonymous=False)
    place = PlaceTools()
    place.start_pick_node()
