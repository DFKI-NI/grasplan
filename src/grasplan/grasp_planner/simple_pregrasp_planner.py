#!/usr/bin/env python3

import rospy
from grasplan.grasp_planning_core import GraspPlanningCore
from grasplan.pose_generator import PoseGenerator
from geometry_msgs.msg import PoseStamped

class SimpleGraspPlanner(GraspPlanningCore):
    '''
    Implement concrete methods out of GraspPlanningCore class
    A simple grasp planner:
    1) receive object pose
    2) replace it with a prerecorded "example" orientation
    3) sample around it in roll, pitch, yaw angles
    '''
    def __init__(self):
        super().__init__()
        self.pose_generator = PoseGenerator()
        # grasp type
        self.grasp_orientations = rospy.get_param('~grasp_orientations')
        self.object_offset_params = rospy.get_param('~object_offset_params')

        # move object slightly in z up
        self.z_offset = rospy.get_param('~z_offset', 0.03)

        # setup publishers for visualization purposes
        self.grasp_pose_pub = rospy.Publisher('~visualization/grasp_pose', PoseStamped, queue_size=1)
        rospy.sleep(0.5) # give some time for publisher to register
        rospy.loginfo('simple pregrasp planner object was created')

    def generate_grasp_pose(self, object_pose, grasp_type):
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = object_pose.header.frame_id # object pose must be expressed w.r.t : self.robot.get_planning_frame()

        # take position from perceived object
        translation = [object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z]
        # get gripper rotation from parameters based on the desired semantic grasp type
        rotation = self.grasp_orientations[grasp_type]

        # grasp pose, this is the pose where you want the end effector to land on
        grasp_pose.pose.position.x = translation[0]
        grasp_pose.pose.position.y = translation[1]
        grasp_pose.pose.position.z = translation[2] + self.z_offset
        grasp_pose.pose.orientation.x = rotation[0]
        grasp_pose.pose.orientation.y = rotation[1]
        grasp_pose.pose.orientation.z = rotation[2]
        grasp_pose.pose.orientation.w = rotation[3]
        # publish grasp pose for visualization purposes
        self.grasp_pose_pub.publish(grasp_pose)
        return grasp_pose

    def gen_end_effector_grasp_poses(self, object_name, object_pose, grasp_type):
        '''
        receive object pose, generate multiple poses around it
        '''
        offset_vector = self.object_offset_params[object_name][grasp_type]
        # take object position and replace its orientation with a prerecorded "example" orientation
        grasp_pose = self.generate_grasp_pose(object_pose, grasp_type)
        pose_array_msg = self.pose_generator.spherical_sampling(grasp_type, grasp_pose, offset_vector)
        return pose_array_msg
