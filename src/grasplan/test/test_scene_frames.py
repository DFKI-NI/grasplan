'''A robot in its own scene copy of a multi-robot sim works in its own map frame (robot2/map), while move_group returns
its planning-scene objects in the model frame (map). The configured tables go into the robot's map frame, and the
open-set height and support checks find the table although the scene returns it in map. Fake planning scene and tf (no
ROS master); needs the Mobipick image for the module imports.'''
import copy
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, Grasp

import grasplan.pick as pick
import grasplan.place as place
from grasplan.tools.scene_frames import (boxes_in_robot_map, objects_in_frame, pose_matrix, set_pose_from_matrix,
                                         tf_matrix)
from grasplan.msg import DetectedObject
from grasplan.tools.common import objectToPick

OFFSET_X = 16.9      # robot2/map sits 16.9 m along x in map (scene copy 2)
TOP = 0.72           # table top


def box(frame='map', name='table_1'):
    return {'scene_name': name, 'frame_id': frame, 'box_x_dimension': 1.4, 'box_y_dimension': 0.7,
            'box_z_dimension': TOP, 'box_position_x': 13.22, 'box_position_y': 2.15, 'box_position_z': TOP / 2.0,
            'box_orientation_x': 0.0, 'box_orientation_y': 0.0, 'box_orientation_z': 0.7071,
            'box_orientation_w': 0.7071}


def table_in(frame, x):
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = x, 2.15, TOP / 2.0
    obj.pose.orientation.w = 1.0
    obj.primitives = [NS(dimensions=[1.4, 0.7, TOP], type=1)]
    obj.primitive_poses = [Pose()]
    return obj


def lookup(target, source):
    '''the static transform map -> robot2/map of the sim'''
    if (target, source) == ('robot2/map', 'map'):
        return tf_matrix((-OFFSET_X, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    if (target, source) == ('map', 'robot2/map'):
        return tf_matrix((OFFSET_X, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    return None


class TestConfiguredBoxes(unittest.TestCase):
    def test_robot_1_keeps_the_config(self):
        boxes = [box()]
        self.assertEqual(boxes_in_robot_map(boxes, 'map'), boxes)

    def test_robot_2_gets_its_tables_in_its_own_map(self):
        moved = boxes_in_robot_map([box(), box('robot2/base_link', 'shelf')], 'robot2/map')
        self.assertEqual([b['frame_id'] for b in moved], ['robot2/map', 'robot2/base_link'])
        self.assertEqual(moved[0]['box_position_x'], 13.22)   # same numbers, its own room

    def test_the_config_list_is_not_changed(self):
        boxes = [box()]
        boxes_in_robot_map(boxes, 'robot2/map')
        self.assertEqual(boxes[0]['frame_id'], 'map')


class TestObjectsInFrame(unittest.TestCase):
    def test_same_frame_returns_the_scene_objects_untouched(self):
        objects = {'table_1': table_in('map', 13.22)}
        result = objects_in_frame(objects, 'map', lambda *a: self.fail('no tf lookup with one robot'))
        self.assertIs(result['table_1'], objects['table_1'])

    def test_robot_2_sees_its_table_in_its_map_frame(self):
        scene = {'table_1': table_in('map', 13.22 + OFFSET_X)}   # move_group returns it in map
        result = objects_in_frame(scene, 'robot2/map', lookup)
        self.assertEqual(result['table_1'].header.frame_id, 'robot2/map')
        self.assertAlmostEqual(result['table_1'].pose.position.x, 13.22)
        self.assertAlmostEqual(result['table_1'].pose.position.z, TOP / 2.0)
        self.assertEqual(scene['table_1'].header.frame_id, 'map')   # the scene's message is a copy, untouched

    def test_rotation_is_carried_into_the_new_frame(self):
        obj = table_in('map', 1.0)
        obj.pose.orientation.z, obj.pose.orientation.w = math.sin(math.pi / 4), math.cos(math.pi / 4)
        result = objects_in_frame({'t': obj}, 'robot2/map', lookup)['t']
        self.assertAlmostEqual(result.pose.orientation.z, math.sin(math.pi / 4))
        self.assertAlmostEqual(result.pose.position.x, 1.0 - OFFSET_X)

    def test_unknown_transform_keeps_the_object(self):
        obj = table_in('elsewhere', 1.0)
        self.assertIs(objects_in_frame({'t': obj}, 'robot2/map', lookup)['t'], obj)

    def test_unset_quaternion_is_identity(self):
        obj = table_in('map', 20.0)
        obj.pose.orientation.w = 0.0
        result = objects_in_frame({'t': obj}, 'robot2/map', lookup)['t']
        self.assertAlmostEqual(abs(result.pose.orientation.w), 1.0)


class FakeScene:
    def __init__(self, frame, x):
        self.objects = {'table_1': table_in(frame, x)}

    def get_objects(self, names=None):
        return {n: o for n, o in self.objects.items() if names is None or n in names}


def grasp(z, frame):
    g = Grasp()
    g.id = 'down'
    g.grasp_pose = PoseStamped()
    g.grasp_pose.header.frame_id = frame
    half = math.radians(90.0) / 2.0   # TCP +x straight down
    g.grasp_pose.pose.orientation.y, g.grasp_pose.pose.orientation.w = math.sin(half), math.cos(half)
    g.grasp_pose.pose.position.x, g.grasp_pose.pose.position.z = 13.22, z
    return g


def picker(map_frame, scene_frame, table_x):
    p = pick.PickTools.__new__(pick.PickTools)
    p.global_reference_frame = map_frame
    p.scene = FakeScene(scene_frame, table_x)
    p.frame_matrix = lookup
    p.fingertip_envelope = lambda: [(0.0, np.array([[0.03, 0.0, 0.0]]))]
    return p


PARAMS = {'~open_set_min_grasp_height': 0.035, '~open_set_min_fingertip_height': 0.015}


class TestPickChecksInTheRobotFrame(unittest.TestCase):
    def setUp(self):
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: PARAMS.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)

    def check_raised(self, p, frame):
        g = grasp(TOP + 0.01, frame)   # fingertips 2 cm into the table
        kept = p.raise_low_grasps([g], 'table_1', [0.0])
        self.assertEqual(len(kept), 1)
        self.assertGreaterEqual(kept[0].grasp_pose.pose.position.z, TOP + 0.045 - 1e-6)

    def test_robot_1_raises_a_low_grasp(self):
        self.check_raised(picker('map', 'map', 13.22), 'map')

    def test_robot_2_raises_a_low_grasp_although_the_scene_returns_map(self):
        # before the fix: "open-set grasp height: no table_1 box in frame robot2/map, grasps unchanged"
        self.check_raised(picker('robot2/map', 'map', 13.22 + OFFSET_X), 'robot2/map')

    def test_robot_2_support_tops(self):
        p = picker('robot2/map', 'map', 13.22 + OFFSET_X)
        self.assertAlmostEqual(p.support_top('table_1', 'robot2/map'), TOP)
        self.assertAlmostEqual(p.support_top_z('table_1', 'robot2/map'), TOP)

    def test_robot_2_support_boxes_are_keyed_by_its_map_frame(self):
        p = picker('robot2/map', 'map', 13.22 + OFFSET_X)
        p.planning_scene_boxes = boxes_in_robot_map([box()], 'robot2/map')
        p.scene.objects['table_1'].id = 'table_1'
        supports = p.support_boxes()
        self.assertEqual(list(supports), ['robot2/map'])
        self.assertAlmostEqual(supports['robot2/map'][0][0][0], 13.22)


class TestPlaceBoxesInTheRobotFrame(unittest.TestCase):
    def test_robot_2_place_supports_are_in_its_map_frame(self):
        tools = place.PlaceTools.__new__(place.PlaceTools)
        tools.global_reference_frame = 'robot2/map'
        tools.scene = FakeScene('map', 13.22 + OFFSET_X)
        tools.frame_matrix = lookup
        boxes = tools.scene_support_boxes()
        self.assertEqual(list(boxes), ['robot2/map'])
        self.assertAlmostEqual(boxes['robot2/map'][0][0][0], 13.22)


class TestObjectPoseInTheRobotFrame(unittest.TestCase):
    """the pick target comes back in the robot's map frame, so the top-down fallback finds the table under it"""
    def setUp(self):
        params = dict(PARAMS, **{'~open_set_complete_one_sided_boxes': False, '~open_set_topdown_fallback': True})
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)

    def object_pose(self, map_frame):
        p = picker(map_frame, 'map', 13.22 + (OFFSET_X if map_frame != 'map' else 0.0))
        p.scene.add_box = lambda *a: None
        p.scene.get_known_object_names = lambda: []
        p.pose_selector_get_all_poses_srv = lambda: NS(poses=NS(objects=[]))
        p.clamp_object_boxes_to_support = False
        frames = []

        def transform_pose(pose, frame):   # the sim's tf: map <-> robot2/map
            frames.append(frame)
            if pose.header.frame_id == frame:
                return pose
            moved = PoseStamped(header=copy.deepcopy(pose.header), pose=copy.deepcopy(pose.pose))
            set_pose_from_matrix(moved.pose, lookup(frame, pose.header.frame_id) @ pose_matrix(pose.pose))
            moved.header.frame_id = frame
            return moved
        p.transform_pose = transform_pose
        p.robot = mock.Mock()
        p.robot.get_planning_frame.return_value = 'map'   # the SRDF virtual joint's parent, for every robot
        target = DetectedObject(class_id='sugar_box', instance_id=1)
        target.header.frame_id = map_frame
        target.pose.position.x, target.pose.position.y, target.pose.position.z = 13.22, 2.15, TOP + 0.03
        target.pose.orientation.w = 1.0
        target.size.x, target.size.y, target.size.z = 0.05, 0.08, 0.06   # low enough for a top-down grasp
        pose, box, _ = p.make_object_pose_and_add_objs_to_planning_scene(objectToPick('sugar_box_1'),
                                                                         external_object=target)
        return p, pose, box, frames[-1]

    def test_robot_1_gets_it_in_map(self):
        self.assertEqual(self.object_pose('map')[3], 'map')

    def test_robot_2_gets_it_in_its_map_frame(self):
        # before: the planning frame 'map', while its grasps and tables are in robot2/map
        self.assertEqual(self.object_pose('robot2/map')[3], 'robot2/map')

    def test_robot_2_top_down_fallback_finds_its_table(self):
        p, pose, box, _ = self.object_pose('robot2/map')
        p.grasp_planner = mock.Mock()
        p.grasp_planner.make_grasps_msgs_from_candidates.side_effect = lambda name, c, frame, link: [Grasp() for _ in c]
        with mock.patch.object(pick.rospy, 'logwarn') as warn:
            grasps = p.topdown_fallback_grasps(objectToPick('sugar_box_1'), pose, box, 'table_1')
        self.assertTrue(grasps, [c.args for c in warn.call_args_list])
        self.assertEqual(p.grasp_planner.make_grasps_msgs_from_candidates.call_args.args[2], 'robot2/map')


if __name__ == '__main__':
    unittest.main()
