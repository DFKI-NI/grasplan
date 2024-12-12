#!/usr/bin/env python3

import rospy
import tf
from urdf_parser_py.urdf import URDF

class GripperFindTransforms:
    '''
    provides information to reconstruct a gripper in rviz using markers
    '''
    def __init__(self):
        # parameters
        robot_description = rospy.get_param('~robot_description', 'mobipick/robot_description')
        self.required_links = rospy.get_param('~required_links', ['mobipick/gripper_right_inner_knuckle',
            'mobipick/gripper_right_outer_knuckle',
            'mobipick/gripper_left_inner_knuckle',
            'mobipick/gripper_left_outer_knuckle',
            'mobipick/gripper_left_outer_finger',
            'mobipick/gripper_right_outer_finger',
            'mobipick/gripper_right_inner_finger',
            'mobipick/gripper_left_inner_finger',
            'mobipick/gripper_left_robotiq_fingertip_65mm',
            'mobipick/gripper_right_robotiq_fingertip_65mm'])
        self.end_effector_link = rospy.get_param('~ee_link', 'mobipick/gripper_tcp')
        self.tf_attempts = rospy.get_param('~tf_attempts', 3)
        # the path where to save the configuration in yaml format
        self.yaml_path = rospy.get_param('~yaml_path', 'mobipick_gripper_transformations.yaml')

        self.robot = None
        rospy.loginfo('-- parsing urdf --')
        try:
            self.robot = URDF.from_parameter_server(robot_description)
        except:
            rospy.logerr(f'could not get URDF from param server, make sure that the parameter {robot_description} is set')
            raise ValueError('could not get URDF from param server')
        rospy.loginfo('-- end of urdf info--\n')
        self.listener = tf.TransformListener()
        rospy.sleep(0.5)
        rospy.loginfo('started gripper find transformations node')

    def get_part_tf(self, reference_frame, target_frame):
        for i in range(self.tf_attempts):
            try:
                (trans, rot) = self.listener.lookupTransform(reference_frame, target_frame, rospy.Time(0))
                rospy.logdebug(f'found transform between {reference_frame} and {target_frame}')
                return trans, rot
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn(f'failed to find transform from {reference_frame} to {target_frame} , will retry')
                rospy.sleep(0.5)

    def generate_config_file(self):
        f = open(self.yaml_path,'w+')
        f.write(f'# This file was automatically generated, manual editing is not recommended\n\n')
        for link in self.robot.links:
            if link.name in self.required_links:
                for i, visual in enumerate(link.visuals):
                    # see: https://github.com/ros/urdf_parser_py/blob/melodic-devel/src/urdf_parser_py/urdf.py
                    # to help yourself navigate the parsing tree
                    translation, rotation = self.get_part_tf(self.end_effector_link, link.name)
                    tab = '  '
                    f.write(f'{link.name}:\n')
                    f.write(f'{tab}v{i}:\n')
                    # tf of visual element wrt gripper end effector
                    f.write(f'{tab}{tab}tf_translation: {translation}\n')
                    f.write(f'{tab}{tab}tf_rotation: {rotation}\n')
                    # mesh can also be translated if origin is not equal to 0 in URDF
                    f.write(f'{tab}{tab}mesh_translation: {visual.origin.position}\n')
                    f.write(f'{tab}{tab}mesh_rotation: {visual.origin.rotation}\n')
                    f.write(f"{tab}{tab}mesh_path: '{visual.geometry.filename}'\n")
                    if visual.geometry.scale is None:
                        f.write(f'{tab}{tab}mesh_scale: [1.0, 1.0, 1.0]\n')
                    else:
                        f.write(f'{tab}{tab}mesh_scale: {visual.geometry.scale}\n')
                    f.write('\n\n')
        f.close()

    def start_gripper_find_transform(self):
        rospy.loginfo('This program will find the transformations of your gripper\n')
        rospy.loginfo('Please move the gripper to a desired joint configuration')
        input("Press Enter to continue...")
        self.generate_config_file()
        rospy.loginfo(f'file generated: {self.yaml_path}')
        rospy.loginfo('bye bye!')

if __name__=='__main__':
    rospy.init_node('gripper_tf_autom_finder_node', anonymous=False)
    gft = GripperFindTransforms()
    gft.start_gripper_find_transform()
