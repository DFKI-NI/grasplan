#!/usr/bin/env python3

'''
example on how to pick an object using grasplan and moveit
'''

import sys
import copy
import importlib
import traceback

import rospy
import actionlib
import moveit_commander

from tf import TransformListener
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
from pose_selector.srv import ClassQuery, PoseDelete, GetPoses
from geometry_msgs.msg import PoseStamped
from grasplan.tools.moveit_errors import print_moveit_error
from moveit_msgs.msg import MoveItErrorCodes, PickupAction, PickupGoal
from grasplan.msg import PickObjectAction, PickObjectResult
from grasplan.tools.common import objectToPick
from visualization_msgs.msg import Marker, MarkerArray


class PickTools:
    def __init__(self):

        # parameters
        self.global_reference_frame = rospy.get_param('~global_reference_frame', 'map')
        self.detach_all_objects_flag = rospy.get_param('~detach_all_objects', False)
        self.arm_group_name = rospy.get_param('~arm_group_name', 'arm')
        gripper_group_name = rospy.get_param('~gripper_group_name', 'gripper')
        arm_goal_tolerance = rospy.get_param('~arm_goal_tolerance', 0.01)
        self.planning_time = rospy.get_param('~planning_time', 20.0)
        self.pregrasp_posture_required = rospy.get_param('~pregrasp_posture_required', False)
        self.pregrasp_posture = rospy.get_param('~pregrasp_posture', 'home')
        self.planning_scene_boxes = rospy.get_param('~planning_scene_boxes', [])
        self.clear_planning_scene = rospy.get_param('~clear_planning_scene', True)
        self.clear_octomap_flag = rospy.get_param('~clear_octomap', False)
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
        # TODO: include octomap

        # to be able to transform PoseStamped later in the code
        self.tf_listener = TransformListener()

        # import grasp planner and make object out of it
        self.grasp_planner = getattr(importlib.import_module(import_file), import_class)()

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
            ' {pose_selector_class_query_srv_name}, {pose_selector_get_all_poses_srv_name},'
            ' {pose_selector_delete_srv_name}'
        )
        # if wait_for_service fails, it will throw a
        # rospy.exceptions.ROSException, and the node will exit (as long as
        # this happens before moveit_commander.roscpp_initialize()).
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
            moveit_commander.roscpp_initialize(sys.argv)
            self.robot = moveit_commander.RobotCommander()
            self.gripper = getattr(self.robot, gripper_group_name)
            self.robot.arm.set_planning_time(self.planning_time)
            self.robot.arm.set_goal_tolerance(arm_goal_tolerance)
            self.scene = moveit_commander.PlanningSceneInterface()
            rospy.loginfo('found move_group action server')
        except RuntimeError:
            # moveit_commander.roscpp_initialize overwrites the signal handler,
            # so if a RuntimeError occurs here, we have to manually call
            # signal_shutdown() in order for the node to properly exit.
            rospy.logfatal(
                'grasplan pick server could not connect to Moveit in time, exiting! \n' + traceback.format_exc()
            )
            rospy.signal_shutdown('fatal error')

        # to publish object pose for debugging purposes
        self.obj_pose_pub = rospy.Publisher('~obj_pose', PoseStamped, queue_size=1)

        # publishers
        self.event_out_pub = rospy.Publisher('~event_out', String, queue_size=1)
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
        self.pick_action_server.start()
        rospy.loginfo('pick node ready!')

    def pick_obj_action_callback(self, goal):
        if self.pick_object(goal.object_name, goal.support_surface_name, self.grasp_type, goal.ignore_object_list):
            self.pick_action_server.set_succeeded(PickObjectResult(success=True))
        else:
            self.pick_action_server.set_aborted(PickObjectResult(success=False))

    def graspTypeCB(self, msg):
        self.grasp_type = msg.data

    def transform_pose(self, pose, target_reference_frame):
        '''
        transform a pose from any rerence frame into the target reference frame
        '''
        self.tf_listener.getLatestCommonTime(target_reference_frame, pose.header.frame_id)
        return self.tf_listener.transformPose(target_reference_frame, pose)

    def make_object_pose_and_add_objs_to_planning_scene(self, object_to_pick, ignore_object_list=[]):
        '''
        ignore_object_list: if an object is inside another one, you can add it to the ignore_object_list and it will
                            not be added to the planning scene, but it will rather be removed from the planning scene
        '''
        assert isinstance(object_to_pick, objectToPick)
        # query pose selector
        resp = self.pose_selector_class_query_srv(object_to_pick.obj_class)
        if len(resp.poses) == 0:
            rospy.logerr(
                f'Object of class {object_to_pick.obj_class} was not perceived, therefore its pose is not available and'
                ' cannot be picked'
            )
            return None, None, None
        # at least one object of the same class as the object we want to pick was perceived, continue
        object_to_pick_id = object_to_pick.id
        object_to_pick_pose = None
        object_to_pick_bounding_box = None
        object_found = False
        # query pose selector
        resp = self.pose_selector_get_all_poses_srv()
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
                else:
                    rospy.loginfo(f'adding object {object_name} to planning scene')
                    # add all perceived objects to planning scene (one at at time)
                    self.scene.add_box(object_name, pose_stamped_msg, object_bounding_box)
        if not object_found:
            rospy.logerr(
                'the specific object you want to pick was not found:'
                ' {object_to_pick.get_object_class_and_id_as_string()}'
            )
            return None, None, None
        return (
            self.transform_pose(object_to_pick_pose, self.robot.get_planning_frame()),
            object_to_pick_bounding_box,
            object_to_pick_id,
        )

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

    def pick_object(self, object_name_as_string, support_surface_name, grasp_type, ignore_object_list=[]):
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

        if self.perceive_object:
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
        for planning_scene_box in self.planning_scene_boxes:
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

        # add all perceived objects of interest to planning scene and return the pose, bb, and id of the
        # object to be picked
        object_pose, bounding_box, id = self.make_object_pose_and_add_objs_to_planning_scene(
            object_to_pick, ignore_object_list=ignore_object_list
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

        # ::::::::: pick
        rospy.loginfo('picking object now')

        # generate a list of moveit grasp messages, poses are also published for visualization purposes
        grasps = self.grasp_planner.make_grasps_msgs(
            object_to_pick.get_object_class_and_id_as_string(),
            object_pose,
            self.robot.arm.get_end_effector_link(),
            grasp_type,
        )

        # clear octomap from the planning scene if needed
        if self.clear_octomap_flag:
            self.clear_octomap()

        # go to intermediate arm poses if needed to disentangle arm cable
        if object_to_pick.obj_class in self.list_of_disentangle_objects:
            for arm_pose in self.poses_to_go_before_pick:
                rospy.loginfo(f'going to intermediate arm pose {arm_pose} to disentangle cable')
                self.move_arm_to_posture(arm_pose)

        # try to pick object with moveit
        # result = self._pick_with_moveit_commander(object_to_pick, grasps, support_surface_name)
        result = self._pick_with_action(object_to_pick, grasps, support_surface_name)
        # handle moveit pick result
        if result == MoveItErrorCodes.SUCCESS:
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
            print_moveit_error(result)
        return False

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
        PICK_OBJECT_SERVER_NAME = 'pickup'

        action_client = actionlib.SimpleActionClient(PICK_OBJECT_SERVER_NAME, PickupAction)
        if action_client.wait_for_server(timeout=rospy.Duration.from_sec(2.0)):
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

            rospy.loginfo(
                f'sending pick {object_to_pick.get_object_class_and_id_as_string()} goal '
                f'to {rospy.resolve_name(PICK_OBJECT_SERVER_NAME)} action server'
            )
            action_client.send_goal(goal)
            rospy.loginfo(f'waiting for result from {rospy.resolve_name(PICK_OBJECT_SERVER_NAME)} action server')
            if action_client.wait_for_result(rospy.Duration.from_sec(60.0)):
                result = action_client.get_result().error_code.val
            else:
                result = MoveItErrorCodes.TIMED_OUT
        else:
            result = MoveItErrorCodes.TIMED_OUT

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
