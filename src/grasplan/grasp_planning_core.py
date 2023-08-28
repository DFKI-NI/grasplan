#!/usr/bin/env python3

import copy
import rospy

from geometry_msgs.msg import PoseArray, PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from grasplan.tools.common import separate_object_class_from_id

class GraspPlanningCore:
    '''
    Abstract class that defines classes to interact with a grasp planning module
    It cannot be used by itself, you need to inherit from it and implement virtual methods
    '''
    def __init__(self):

        # parameters
        self.gripper_joint_names = rospy.get_param('~gripper_joint_names')
        self.gripper_close_distance = rospy.get_param('~gripper_close')
        self.gripper_open_distance = rospy.get_param('~gripper_open')
        self.gripper_joint_efforts = rospy.get_param('~gripper_joint_efforts')
        self.grasp_quality = rospy.get_param('~grasp_quality', 1.0)
        self.object_padding = rospy.get_param('~object_padding', 0.04)
        self.max_contact_force = rospy.get_param('~max_contact_force', 1.0)
        self.distance_gripper_close_per_obj = rospy.get_param('~distance_gripper_close_per_obj', None)
        self.distance_gripper_open_per_obj = rospy.get_param('~distance_gripper_open_per_obj', None)
        # pregrasp parameters
        self.pre_grasp_approach_min_dist = rospy.get_param('~pre_grasp_approach/min_dist')
        self.pre_grasp_approach_desired = rospy.get_param('~pre_grasp_approach/desired')
        self.pre_grasp_approach_axis = rospy.get_param('~pre_grasp_approach/axis')
        # post grasp retreat parameters
        self.post_grasp_retreat_frame_id = rospy.get_param('~post_grasp_retreat/frame_id')
        self.post_grasp_retreat_min_dist = rospy.get_param('~post_grasp_retreat/min_dist')
        self.post_grasp_retreat_desired = rospy.get_param('~post_grasp_retreat/desired')
        self.post_grasp_retreat_axis = rospy.get_param('~post_grasp_retreat/axis')

        # publish grasp poses as pose array
        self.pose_array_pub = rospy.Publisher('~grasp_poses', PoseArray, queue_size=50)

    def get_joint_value_from_dic(self, joint_angles, dictionary, object_class=None):
        '''
        query dictionary of objects vs how much should the gripper close to grasp them
        TODO: at the moment this does not depend on the particular grasp but in general it should
        if object_class is not found in dictionary then the default value is used
        '''
        if object_class is None or dictionary is None or object_class not in dictionary:
            return joint_angles
        return [dictionary[object_class]]

    def make_gripper_trajectory(self, joint_angles, dictionary, object_class=None):
        '''
        Set and return the gripper posture as a trajectory_msgs/JointTrajectory
        only one point is set which is the final gripper target
        '''
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joint_names
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = self.get_joint_value_from_dic(joint_angles, dictionary, object_class=object_class)
        trajectory_point.effort = self.gripper_joint_efforts
        trajectory_point.time_from_start = rospy.Duration(1.0)
        trajectory.points.append(trajectory_point)
        return trajectory

    def get_object_padding(self):
        return self.object_padding

    def make_grasps_msgs(self, object_name, object_pose, end_effector_frame, grasp_type):
        '''
        generate grasp configurations for moveit
        '''

        # make empty msg of type moveit_msgs/Grasp
        # see http://docs.ros.org/en/api/moveit_msgs/html/msg/Grasp.html
        g = Grasp()

        # The estimated probability of success for this grasp, or some other
        # measure of how 'good' it is.
        g.grasp_quality = self.grasp_quality

        # The internal posture of the hand for the pre-grasp
        # only positions are used
        object_class = separate_object_class_from_id(object_name)[0]
        g.pre_grasp_posture = self.make_gripper_trajectory(self.gripper_open_distance, self.distance_gripper_open_per_obj, object_class=object_class)

        # The approach direction to take before picking an object
        g.pre_grasp_approach = self.make_gripper_translation_msg(end_effector_frame,
            min_dist=self.pre_grasp_approach_min_dist, desired=self.pre_grasp_approach_desired, axis=self.pre_grasp_approach_axis)

        # The position of the end-effector for the grasp.  This is the pose of
        # the 'parent_link' of the end-effector, not actually the pose of any
        # link *in* the end-effector.  Typically this would be the pose of the
        # most distal wrist link before the hand (end-effector) links began.
        # g.grasp_pose = grasp_pose # will be filled later

        # The internal posture of the hand for the grasp
        # positions and efforts are used
        g.grasp_posture = self.make_gripper_trajectory(self.gripper_close_distance, self.distance_gripper_close_per_obj, object_class=object_class)

        # The retreat direction to take after a grasp has been completed (object is attached)
        g.post_grasp_retreat = self.make_gripper_translation_msg(self.post_grasp_retreat_frame_id,
            min_dist=self.post_grasp_retreat_min_dist, desired=self.post_grasp_retreat_desired, axis=self.post_grasp_retreat_axis)

        # the maximum contact force to use while grasping (<=0 to disable)
        g.max_contact_force = self.max_contact_force

        # an optional list of obstacles that we have semantic information about
        # and that can be touched/pushed/moved in the course of grasping
        g.allowed_touch_objects = [object_name]

        # A name for this grasp
        # g.id = 'top_grasp' # will be filled later

        # NOTE : one could change the orientation and generate more grasps, currently the list has only 1 grasp

        # call grasp planner
        pose_array_msg = self.gen_end_effector_grasp_poses(object_name, object_pose, grasp_type)
        # publish poses for visualisation purposes
        self.pose_array_pub.publish(pose_array_msg)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = pose_array_msg.header.frame_id
        pose_stamped.header.stamp = pose_array_msg.header.stamp
        grasps = []
        for i, pose in enumerate(pose_array_msg.poses):
            # convert to pose stamped
            pose_stamped.pose = pose
            g.grasp_pose = pose_stamped
            g.id = 'grasp_' + str(i)
            grasps.append(copy.deepcopy(g))

        return grasps

    def gen_end_effector_grasp_poses(self, object_name, object_pose):
        '''
        virtual method, derive from this class and implement
        '''
        raise NotImplementedError()

    def make_gripper_translation_msg(self, frame_id, min_dist=0.08, desired=0.25, axis=[1.0, 0.0, 0.0]):
        '''
        make moveit_msgs/GripperTranslation msg
        '''

        # make empty msg of type moveit_msgs/GripperTranslation
        # see: http://docs.ros.org/en/api/moveit_msgs/html/msg/GripperTranslation.html
        gripper_translation_msg = GripperTranslation()

        # the direction of the translation
        gripper_translation_msg.direction.header.frame_id = frame_id
        gripper_translation_msg.direction.vector.x = axis[0]
        gripper_translation_msg.direction.vector.y = axis[1]
        gripper_translation_msg.direction.vector.z = axis[2]

        # the desired translation distance
        gripper_translation_msg.desired_distance = desired

        # the min distance that must be considered feasible before the grasp is even attempted
        gripper_translation_msg.min_distance = min_dist

        return gripper_translation_msg
