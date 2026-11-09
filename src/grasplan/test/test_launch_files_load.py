'''The launch files the robot arguments go through load with roslaunch's own loader, which resolves their args, $(find)
and $(eval) without starting anything: namespace:=<robot> reaches the AnyGrasp names and the tf prefix, and the robot
description upload can be switched off. Needs the Mobipick image, for roslaunch and the packages the files name.'''
import os
import unittest

import roslaunch.config
import roslaunch.xmlloader

LAUNCH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'launch')
PICK_LAUNCHES = ['grasp_planning/pick_handcoded_grasp_planner.launch', 'grasp_planning/pick_simple_pregrasp_planner.launch']
BOX_HELPER = 'planning_scene/find_planning_scene_boxes_helper.launch'


def load(name, *args):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(os.path.join(LAUNCH, name), config, argv=list(args), verbose=False)
    return config


def param(config, name):
    return config.params[name].value


class TestLaunchFilesLoad(unittest.TestCase):
    def test_all_load_with_their_defaults(self):
        for name in PICK_LAUNCHES + [BOX_HELPER, 'teach_grasps/grasp_editor.launch',
                                     'visualization/gripper_tf_autom_finder.launch',
                                     'visualization/rviz_gripper_visualizer.launch']:
            self.assertTrue(load(name).nodes, name)

    def test_pick_defaults_stay_mobipick(self):
        for name in PICK_LAUNCHES:
            config = load(name)
            self.assertEqual(param(config, '/pick_object_node/anygrasp_action_name'), '/mobipick/grasp_object')
            self.assertEqual(param(config, '/pick_object_node/anygrasp_generate_action_name'),
                             '/mobipick/generate_grasps')

    def test_pick_namespace_arg(self):
        for name in PICK_LAUNCHES:
            config = load(name, 'namespace:=mobipick2')
            self.assertEqual(param(config, '/pick_object_node/anygrasp_action_name'), '/mobipick2/grasp_object')
            self.assertEqual(param(config, '/pick_object_node/anygrasp_generate_action_name'),
                             '/mobipick2/generate_grasps')

    def test_pick_action_name_args_serve_a_robot_without_a_namespace(self):
        for name in PICK_LAUNCHES:
            config = load(name, 'anygrasp_action_name:=/grasp_object', 'anygrasp_generate_action_name:=/generate_grasps')
            self.assertEqual(param(config, '/pick_object_node/anygrasp_action_name'), '/grasp_object')

    def test_pick_config_package_arg_decides_the_file(self):
        for name in PICK_LAUNCHES:
            with self.assertRaises(Exception) as caught:
                load(name, 'config_package:=grasplan')
            self.assertIn('grasplan/config/grasplan/pick_params.yaml', str(caught.exception))

    def test_box_helper_description(self):
        self.assertIn('mobipick/', str(param(load(BOX_HELPER), '/robot_description')))
        self.assertIn('mobipick2/', str(param(load(BOX_HELPER, 'namespace:=mobipick2'), '/robot_description')))
        self.assertNotIn('/robot_description', load(BOX_HELPER, 'upload_description:=false').params)


if __name__ == '__main__':
    unittest.main()
