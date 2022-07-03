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
import tf
import moveit_commander

from tf import TransformListener
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
from pose_selector.srv import ClassQuery, PoseDelete
from geometry_msgs.msg import PoseStamped
from grasplan.tools.moveit_errors import print_moveit_error
from moveit_msgs.msg import MoveItErrorCodes
from pbr_msgs.msg import PickObjectAction, PickObjectResult
from grasplan.common_grasp_tools import objectToPick

class PickTools():
    def __init__(self):

        # parameters
        self.global_reference_frame = rospy.get_param('~global_reference_frame', 'map')
        self.objects_of_interest = rospy.get_param('~objects_of_interest', ['multimeter', 'klt', 'power_drill_with_grip', 'screwdriver', 'relay'])
        arm_group_name = rospy.get_param('~arm_group_name', 'arm')
        gripper_group_name = rospy.get_param('~gripper_group_name', 'gripper')
        arm_goal_tolerance = rospy.get_param('~arm_goal_tolerance', 0.01)
        planning_time = rospy.get_param('~planning_time', 20.0)
        self.pregrasp_posture_required = rospy.get_param('~pregrasp_posture_required', False)
        self.pregrasp_posture = rospy.get_param('~pregrasp_posture', 'home')
        self.planning_scene_boxes = rospy.get_param('~planning_scene_boxes', [])
        self.clear_planning_scene = rospy.get_param('~clear_planning_scene', False)
        self.clear_octomap_flag = rospy.get_param('~clear_octomap', False)
        # if true the arm is moved to a pose where objects are inside fov and pose selector is triggered to accept obj pose updates
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
        pose_selector_activate_name = rospy.get_param('~pose_selector_activate_srv_name', '/pose_selector_activate')
        pose_selector_query_name = rospy.get_param('~pose_selector_class_query_srv_name', '/pose_selector_class_query')
        pose_selector_delete_name = rospy.get_param('~pose_selector_delete_srv_name', '/pose_selector_delete')
        rospy.loginfo(f'waiting for pose selector services: {pose_selector_activate_name}, {pose_selector_query_name}')
        rospy.wait_for_service(pose_selector_activate_name, 30.0)
        rospy.wait_for_service(pose_selector_query_name, 30.0)
        rospy.wait_for_service(pose_selector_delete_name, 30.0)
        try:
            self.activate_pose_selector_srv = rospy.ServiceProxy(pose_selector_activate_name, SetBool)
            self.pose_selector_class_query_srv = rospy.ServiceProxy(pose_selector_query_name, ClassQuery)
            self.pose_selector_delete_srv = rospy.ServiceProxy(pose_selector_delete_name, PoseDelete)
            rospy.loginfo('found pose selector services')
        except rospy.exceptions.ROSException:
            rospy.logfatal('grasplan pick server could not find pose selector services in time, exiting! \n' + traceback.format_exc())
            rospy.signal_shutdown('fatal error')

        try:
            rospy.loginfo('waiting for move_group action server')
            moveit_commander.roscpp_initialize(sys.argv)
            self.robot = moveit_commander.RobotCommander()
            self.robot.arm.set_planning_time(planning_time)
            self.robot.arm.set_goal_tolerance(arm_goal_tolerance)
            self.scene = moveit_commander.PlanningSceneInterface()
            rospy.loginfo('found move_group action server')
        except RuntimeError:
            rospy.logfatal('grasplan pick server could not connect to Moveit in time, exiting! \n' + traceback.format_exc())
            rospy.signal_shutdown('fatal error')

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander(arm_group_name, wait_for_servers=20.0)
        self.robot.arm.set_planning_time(planning_time)
        self.gripper = moveit_commander.MoveGroupCommander(gripper_group_name, wait_for_servers=20.0)
        self.arm.set_goal_tolerance(arm_goal_tolerance)
        self.scene = moveit_commander.PlanningSceneInterface()

        # keep memory about the last object we have grasped
        self.grasped_object = ''

        # to publish object pose for debugging purposes
        self.obj_pose_pub = rospy.Publisher('~obj_pose', PoseStamped, queue_size=1)

        # publishers
        self.event_out_pub = rospy.Publisher('~event_out', String, queue_size=1)
        self.trigger_perception_pub = rospy.Publisher('/object_recognition/event_in', String, queue_size=1)

        # subscribers
        self.grasp_type = 'side_grasp' # TODO remove, get from actionlib
        rospy.Subscriber('~grasp_type', String, self.graspTypeCB) # TODO remove, get from actionlib

        # offer action lib server
        self.pick_action_server = actionlib.SimpleActionServer('pick_object', PickObjectAction, self.pick_obj_action_callback, False)
        self.pick_action_server.start()
        rospy.loginfo('pick node ready!')

    def pick_obj_action_callback(self, goal):
        if self.pick_object(goal.object_name, goal.support_surface_name, self.grasp_type):
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

    def make_object_pose_and_add_objs_to_planning_scene(self, object_to_pick):
        assert isinstance(object_to_pick, objectToPick)
        # query pose selector
        resp = self.pose_selector_class_query_srv(object_to_pick.obj_class)
        if len(resp.poses) == 0:
            rospy.logerr(f'Object of class {object_to_pick.obj_class} was not perceived, therefore its pose is not available and cannot be picked')
            return None, None, None
        # at least one object of the same class as the object we want to pick was perceived, continue
        object_to_pick_id = object_to_pick.id
        object_to_pick_pose = None
        object_to_pick_bounding_box = None
        object_found = False
        for object_of_interest in self.objects_of_interest:
            # query pose selector
            resp = self.pose_selector_class_query_srv(object_of_interest)
            if len(resp.poses) > 0:
                for pose_selector_object in resp.poses:
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
                        rospy.loginfo(f'found an instance of the object class you want to pick in pose selector: {object_name}')
                    elif object_to_pick.get_object_class_and_id_as_string() == object_name:
                        object_to_pick_pose = copy.deepcopy(pose_stamped_msg)
                        object_to_pick_bounding_box = copy.deepcopy(object_bounding_box)
                        object_to_pick_id = copy.deepcopy(pose_selector_object.instance_id)
                        object_found = True
                        rospy.loginfo(f'found specific object to be picked in pose selector: {object_name}')
                    # add all perceived objects to planning scene (one at at time)
                    self.scene.add_box(object_name, pose_stamped_msg, object_bounding_box)
            else:
                rospy.logdebug(f'object of class {object_of_interest} is not in pose selector')
        if not object_found:
            rospy.logerr(f'the specific object you want to pick was not found : {object_to_pick.get_object_class_and_id_as_string()}')
            return None, None, None
        return self.transform_pose(object_to_pick_pose, self.robot.get_planning_frame()), object_to_pick_bounding_box, object_to_pick_id

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
            self.robot.gripper.detach_object(name=attached_object)

    def pick_object(self, object_name_as_string, support_surface_name, grasp_type):
        '''
        1) move arm to a position where the attached camera can see the scene (octomap will be populated)
        2) clear octomap
        3) add table and object to be grasped to planning scene
        4) generate a list of grasp configurations
        5) call pick moveit functionality
        '''
        rospy.loginfo(f'attempting to pick object : {object_name_as_string}')

        object_to_pick = objectToPick(object_name_as_string)

        # open gripper
        #rospy.loginfo('gripper will open now')
        #self.move_gripper_to_posture('open')

        # detach (all) object if any from the gripper
        self.detach_all_objects()

        # flag to keep track of the state of the grasp
        success = False

        # ::::::::: perceive object to be picked (optional, read from parameter server if this is required)

        if self.perceive_object:
            # send arm to a pose where objects are inside fov
            self.move_arm_to_posture(self.arm_pose_with_objs_in_fov)
            # populate pose selector with pose information
            resp = self.activate_pose_selector_srv(True)
            # wait until pose selector gets updates
            rospy.sleep(10.0)
            # deactivate pose selector detections
            resp = self.activate_pose_selector_srv(False)

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

        # add all perceived objects of interest to planning scene and return the pose, bb, and id of the object to be picked
        object_pose, bounding_box, id = self.make_object_pose_and_add_objs_to_planning_scene(object_to_pick)

        if object_pose is None:
            return False

        # this condition is when user only specified object class but no id, then we assign the first available id
        if id is not None:
            object_to_pick.set_id(id)

        self.obj_pose_pub.publish(object_pose) # publish object pose for visualisation purposes

        # print objects that were added to the planning scene
        rospy.loginfo(f'planning scene objects: {self.scene.get_known_object_names()}')

        # move arm to pregrasp in joint space, not really needed, can be removed
        if self.pregrasp_posture_required:
            self.move_arm_to_posture(self.pregrasp_posture)

        # ::::::::: pick
        rospy.loginfo(f'picking object now')

        # generate a list of moveit grasp messages, poses are also published for visualisation purposes
        grasps = self.grasp_planner.make_grasps_msgs(object_to_pick.get_object_class_and_id_as_string(), object_pose, self.robot.arm.get_end_effector_link(), grasp_type)

        # clear octomap from the planning scene if needed
        if self.clear_octomap_flag:
            self.clear_octomap()

        # try to pick object with moveit
        self.robot.arm.set_support_surface_name(support_surface_name)
        result = self.robot.arm.pick(object_to_pick.get_object_class_and_id_as_string(), grasps)
        # handle moveit pick result
        if result == MoveItErrorCodes.SUCCESS:
            # remove picked object pose from pose selector
            self.pose_selector_delete_srv(class_id=object_to_pick.obj_class, instance_id=object_to_pick.id)
            rospy.loginfo(f'Successfully picked object : {object_to_pick.get_object_class_and_id_as_string()}')
            return True
        else:
            rospy.logerr(f'grasp failed')
            print_moveit_error(result)
        return False

    def start_pick_node(self):
        # wait for trigger via topic or action lib
        rospy.loginfo('ready to receive pick requests')
        rospy.spin()
        # shutdown moveit cpp interface before exit
        moveit_commander.roscpp_shutdown()

if __name__=='__main__':
    rospy.init_node('pick_object_node', anonymous=False)
    pick = PickTools()
    pick.start_pick_node()
