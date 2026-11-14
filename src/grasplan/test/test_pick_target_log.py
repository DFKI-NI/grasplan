'''One log line per pick on the pose its grasps are built from, closed-set and open-set: position, orientation, box,
tilt from upright and the signed gap of its bottom to the support under its centre, also at 0 and when it floats (the
older line "box bottom was X cm inside its support" printed only for a sunk box, a one-sided sample). Mocks for the
scene, the pose selector and rosparam (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from test_false_boxes_and_hints import I, TABLE

import grasplan.pick as pick
from grasplan.msg import DetectedObject
from grasplan.tools.common import objectToPick
from grasplan.tools.topdown_grasps import pick_target_report, support_top_under

SOUP = (0.073, 0.073, 0.101)   # a soup can box, only for the numbers of the test


def selector_object(name, z, size=SOUP, x=0.0):
    class_id, instance_id = name.rsplit('_', 1)
    return NS(class_id=class_id, instance_id=int(instance_id),
              pose=NS(position=NS(x=x, y=0.0, z=z), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
              size=NS(x=size[0], y=size[1], z=size[2]))


def detected_object(class_id, instance_id, frame, center, size):
    target = DetectedObject(class_id=class_id, instance_id=instance_id)
    target.header.frame_id = frame
    target.pose.position.x, target.pose.position.y, target.pose.position.z = center
    target.pose.orientation.w = 1.0
    target.size.x, target.size.y, target.size.z = size
    return target


def picker(objects, clamp=True, params=None):
    '''a PickTools with the pose selector holding objects and the table of TABLE as the only support'''
    p = pick.PickTools.__new__(pick.PickTools)
    p.global_reference_frame = 'map'
    p.clamp_object_boxes_to_support = clamp
    p.added = {}
    p.scene = mock.Mock(add_box=lambda name, pose, size: p.added.__setitem__(name, (pose.pose.position.z, list(size))),
                        get_known_object_names=lambda: [])
    p.pose_selector_class_query_srv = lambda obj_class: NS(poses=[o for o in objects if o.class_id == obj_class])
    p.pose_selector_get_all_poses_srv = lambda: NS(poses=NS(objects=objects))
    p.support_boxes = lambda exclude=(): {'map': TABLE}
    p.transform_pose = lambda pose, frame: pose
    return p


class TestReportText(unittest.TestCase):
    def test_resting_sunk_floating(self):
        on = pick_target_report('soup_1', 'map', (0.0, 0.0, 0.72 + 0.0505), I, SOUP, TABLE)
        self.assertIn('pick target soup_1: frame map, position (0.000, 0.000, 0.770)', on)
        self.assertIn('orientation (0.000, 0.000, 0.000, 1.000), box (0.073, 0.073, 0.101)', on)
        self.assertIn('tilt 0.0 deg from upright', on)
        self.assertIn('support top at z 0.720, bottom +0.0 cm from it (on it)', on)
        sunk = pick_target_report('soup_1', 'map', (0.0, 0.0, 0.72 + 0.0505 - 0.013), I, SOUP, TABLE)
        self.assertIn('bottom -1.3 cm from it (inside it)', sunk)
        floating = pick_target_report('soup_1', 'map', (0.0, 0.0, 0.72 + 0.0505 + 0.02), I, SOUP, TABLE)
        self.assertIn('bottom +2.0 cm from it (above it)', floating)

    def test_no_support_and_not_loaded(self):
        self.assertTrue(pick_target_report('soup_1', 'map', (3.0, 0.0, 0.8), I, SOUP, TABLE)
                        .endswith('no support under its centre'))
        self.assertTrue(pick_target_report('soup_1', 'map', (0.0, 0.0, 0.8), I, SOUP, None)
                        .endswith('supports not loaded'))

    def test_tilt(self):
        tilted = (0.0, 0.17364817766693033, 0.0, 0.984807753012208)   # 20 deg about y
        self.assertIn('tilt 20.0 deg from upright', pick_target_report('x_1', 'map', (0, 0, 0.8), tilted, SOUP, TABLE))

    def test_highest_support_under_the_centre(self):
        shelf = ((0.0, 0.0, 0.9), I, (0.5, 0.5, 0.02))   # top 0.91, under a centre at 1.0
        self.assertAlmostEqual(support_top_under((0.0, 0.0, 1.0), TABLE + [shelf]), 0.91)
        self.assertAlmostEqual(support_top_under((0.0, 0.0, 0.8), TABLE + [shelf]), 0.72)   # below the shelf middle
        self.assertIsNone(support_top_under((3.0, 0.0, 0.8), TABLE))


class TestOneLinePerPick(unittest.TestCase):
    def setUp(self):
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: default)
        patcher.start()
        self.addCleanup(patcher.stop)

    def lines(self, p, target, **kwargs):
        with mock.patch.object(pick.rospy, 'loginfo') as info:
            result = p.make_object_pose_and_add_objs_to_planning_scene(objectToPick(target), **kwargs)
        return result, [c[0][0] for c in info.call_args_list if c[0][0].startswith('pick target ')]

    def test_closed_set_resting_target_is_logged_too(self):
        p = picker([selector_object('soup_1', 0.72 + 0.0505), selector_object('box_1', 0.77, x=0.3)])
        (pose, box, _), lines = self.lines(p, 'soup_1')
        self.assertEqual(len(lines), 1)
        self.assertIn('pick target soup_1: frame map', lines[0])
        self.assertIn('(on it)', lines[0])

    def test_closed_set_any_instance_sunk(self):
        p = picker([selector_object('soup_1', 0.72 + 0.0505 - 0.008)])
        _, lines = self.lines(p, 'soup')
        self.assertEqual(len(lines), 1)
        self.assertIn('pick target soup_1', lines[0])
        self.assertIn('bottom -0.8 cm from it (inside it)', lines[0])

    def test_supports_off_says_so(self):
        p = picker([selector_object('soup_1', 0.77)], clamp=False)
        _, lines = self.lines(p, 'soup_1')
        self.assertTrue(lines[0].endswith('supports not loaded'))

    def test_open_set_target(self):
        p = picker([])
        p.complete_one_sided_box = lambda name, pose, size: None
        target = detected_object('pear', 1, 'map', (0.0, 0.0, 0.72 + 0.04), (0.06, 0.06, 0.08))
        (_, _, _), lines = self.lines(p, 'pear_1', external_object=target)
        self.assertEqual(len(lines), 1)
        self.assertIn('pick target pear_1: frame map', lines[0])
        self.assertIn('(on it)', lines[0])

    def test_no_line_when_the_target_is_missing(self):
        p = picker([selector_object('box_1', 0.77)])
        p.fail = lambda message: None
        _, lines = self.lines(p, 'soup_1')
        self.assertEqual(lines, [])


if __name__ == '__main__':
    unittest.main()
