'''~clear_planning_scene renews the scene without a gap: nothing is removed before the goal's boxes are added (an object
added again is replaced in one scene update, so it stays masked out of the octomap); only the objects not added again
are removed afterwards. The explicit attach of ensure_attached sends no separate world remove either. Fake scene, no
ROS master; needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from geometry_msgs.msg import PoseStamped

import grasplan.pick as pick


class FakeScene:
    def __init__(self, names):
        self.names, self.calls = list(names), []

    def get_known_object_names(self):
        return list(self.names)

    def add_box(self, name, pose, size):
        self.calls.append(('add', name))
        if name not in self.names:
            self.names.append(name)

    def remove_world_object(self, name):
        self.calls.append(('remove', name))
        self.names.remove(name)

    def get_attached_objects(self, names):
        return {}

    def get_objects(self, names):
        box = NS(dimensions=[0.1, 0.2, 0.3])
        pose = NS(orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))
        return {n: NS(primitives=[box], header=NS(frame_id='map'), pose=pose, primitive_poses=[pose]) for n in names}

    def attach_box(self, link, name, pose, size, touch_links=()):
        self.calls.append(('attach', name))


def tools(names, clear=True):
    p = pick.PickTools.__new__(pick.PickTools)
    p.scene, p.clear_planning_scene = FakeScene(names), clear
    return p


class TestSceneRefresh(unittest.TestCase):
    def test_adds_first_then_only_the_stale_ones_go(self):
        p = tools(['table_1', 'power_drill_1', 'old_can_1'])
        before = p.scene_refresh_begin()
        self.assertEqual(p.scene.calls, [])   # nothing removed before the adds
        for name in ('table_1', 'power_drill_1'):
            p.scene.add_box(name, None, None)
            p.scene_added(name)
        p.scene_refresh_end(before)
        self.assertEqual(p.scene.calls, [('add', 'table_1'), ('add', 'power_drill_1'), ('remove', 'old_can_1')])
        p.scene_added('late')   # after the setup nothing is noted any more
        self.assertFalse(hasattr(p, 'scene_refreshed'))

    def test_already_removed_names_are_not_removed_again(self):
        p = tools(['klt_1', 'copy_1'])
        before = p.scene_refresh_begin()
        p.scene.remove_world_object('copy_1')   # the setup dropped it itself (ignore list, overlap, copy)
        p.scene_added('klt_1')
        p.scene_refresh_end(before)
        self.assertEqual(p.scene.calls, [('remove', 'copy_1')])

    def test_without_clear_planning_scene_nothing_is_removed(self):
        p = tools(['klt_1', 'old_can_1'], clear=False)
        p.scene_refresh_end(p.scene_refresh_begin())
        self.assertEqual(p.scene.calls, [])

    def test_the_real_add_paths_note_their_names(self):
        # the boxes the setup adds through add_object_box and add_custom_boxes_to_ps stay; only the rest goes
        p = tools(['table_1', 'power_drill_1', 'old_can_1'])
        p.planning_scene_boxes = [{'scene_name': 'table_1', 'frame_id': 'map', 'box_x_dimension': 1.0,
                                   'box_y_dimension': 1.0, 'box_z_dimension': 0.7, 'box_position_x': 0.0,
                                   'box_position_y': 0.0, 'box_position_z': 0.35, 'box_orientation_x': 0.0,
                                   'box_orientation_y': 0.0, 'box_orientation_z': 0.0, 'box_orientation_w': 1.0}]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.w = 1.0
        with mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: default):
            before = p.scene_refresh_begin()
            p.add_custom_boxes_to_ps(p.planning_scene_boxes)
            self.assertTrue(p.add_object_box('power_drill_1', pose, [0.1, 0.2, 0.2], supports=None))
            p.scene_refresh_end(before)
        self.assertEqual(p.scene.calls, [('add', 'table_1'), ('add', 'power_drill_1'), ('remove', 'old_can_1')])
        self.assertEqual(sorted(p.scene.names), ['power_drill_1', 'table_1'])


class TestExplicitAttach(unittest.TestCase):
    def test_attach_without_a_world_remove_first(self):
        p = tools(['power_drill_1'])
        p.robot = NS(arm=NS(get_end_effector_link=lambda: 'gripper_tcp'), get_link_names=lambda group: ['finger'])
        p.gripper = NS(get_name=lambda: 'gripper')
        self.assertTrue(p.ensure_attached('power_drill_1'))
        self.assertEqual(p.scene.calls, [('attach', 'power_drill_1')])


if __name__ == '__main__':
    unittest.main()
