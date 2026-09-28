'''#151 real 2026-09-28 (Oscar via claude-b1ef): a false DOPE bleach_1 box, 11.2 cm deep in table_2 and raised right
onto the tomato soup can, blocked every grasp. A box more than 9 cm inside its support, or one that shares at least 20 %
of the smaller volume with the verified open-set target, stays out of the planning scene. And the planner's grasp-type
hint (rosparam /mobipick/grasp_type_hints/<key>: side | top | auto) steers an open-set pick. Mocks for the scene,
the pose selector and rosparam (no ROS master); needs the Mobipick image for the module imports.'''
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Grasp

import grasplan.pick as pick
from grasplan.grasp_markers import make_candidate_scores
from grasplan.msg import DetectedObject, GraspCandidate
from grasplan.tools.common import objectToPick
from grasplan.tools.topdown_grasps import box_overlap_fraction, depth_below_support_top

I = (0.0, 0.0, 0.0, 1.0)
TABLE = [((0.0, 0.0, 0.36), I, (2.0, 2.0, 0.72))]   # top at 0.72
CAN = ((0.0, 0.0, 0.775), I, (0.073, 0.073, 0.115))
BLEACH = ((0.002, -0.028, 0.738), I, (0.102, 0.068, 0.251))   # the real false box: 25 cm tall, 11 cm in the table


class TestGeometry(unittest.TestCase):
    def test_sink_depth(self):
        self.assertAlmostEqual(depth_below_support_top(*BLEACH, TABLE), 0.72 - (0.738 - 0.1255), places=4)
        self.assertAlmostEqual(depth_below_support_top(*CAN, TABLE), 0.0025, places=4)
        self.assertEqual(depth_below_support_top((3.0, 0.0, 0.8), I, (0.1, 0.1, 0.2), TABLE), 0.0)   # off the table
        self.assertEqual(depth_below_support_top((0.0, 0.0, 0.1), I, (0.1, 0.1, 0.1), TABLE), 0.0)   # under it

    def test_overlap(self):
        self.assertAlmostEqual(box_overlap_fraction(*CAN, *CAN), 1.0)
        self.assertGreater(box_overlap_fraction(*BLEACH, *CAN), 0.5)            # most of the can is in the bleach box
        neighbour = ((0.0715, 0.0, 0.775), I, (0.073, 0.073, 0.115))            # touching, 1.5 mm into it
        self.assertLess(box_overlap_fraction(*neighbour, *CAN), 0.05)
        self.assertEqual(box_overlap_fraction((0.2, 0.0, 0.775), I, (0.073, 0.073, 0.115), *CAN), 0.0)
        self.assertEqual(box_overlap_fraction((0.0, 0.0, 1.2), I, (0.073, 0.073, 0.115), *CAN), 0.0)   # above
        yaw45 = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))
        self.assertAlmostEqual(box_overlap_fraction((0, 0, 0), I, (1, 1, 1), (0, 0, 0), yaw45, (1, 1, 1)),
                               2 * (math.sqrt(2) - 1), places=6)


def stamped(center, orientation, frame='map'):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = center
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = orientation
    return pose


class Params(unittest.TestCase):
    def setUp(self):
        self.params = {}
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.p = pick.PickTools.__new__(pick.PickTools)
        self.added, self.removed = [], []
        self.p.scene = mock.Mock(add_box=lambda name, pose, size: self.added.append(name),
                                 get_known_object_names=lambda: ['bleach_1'],
                                 remove_world_object=lambda name: self.removed.append(name))


class TestImplausibleBoxes(Params):
    def test_deep_false_box_left_out_and_removed(self):
        self.assertFalse(self.p.add_object_box('bleach_1', stamped(*BLEACH[:2]), list(BLEACH[2]), {'map': TABLE}))
        self.assertEqual((self.added, self.removed), ([], ['bleach_1']))

    def test_real_klt_6_cm_deep_stays(self):
        klt = ((0.3, 0.0, 0.7295), I, (0.297, 0.197, 0.147))   # bottom 6.4 cm below the top
        self.assertTrue(self.p.add_object_box('klt_1', stamped(*klt[:2]), list(klt[2]), {'map': TABLE}))
        self.assertEqual(self.added, ['klt_1'])

    def test_the_target_is_kept_anyway(self):
        self.assertTrue(self.p.add_object_box('bleach_1', stamped(*BLEACH[:2]), list(BLEACH[2]), {'map': TABLE},
                                              keep_implausible=True))

    def test_switch_off(self):
        self.params['~max_box_sink_into_support'] = 0.0
        self.assertTrue(self.p.add_object_box('bleach_1', stamped(*BLEACH[:2]), list(BLEACH[2]), {'map': TABLE}))

    def test_overlap_rule(self):
        target = (stamped(*CAN[:2]), list(CAN[2]))
        self.assertTrue(self.p.overlaps_target('bleach_1', stamped(*BLEACH[:2]), list(BLEACH[2]), target, 'can_1'))
        far = stamped((0.3, 0.0, 0.79), I)
        self.assertFalse(self.p.overlaps_target('klt_1', far, [0.297, 0.197, 0.147], target, 'can_1'))
        other_frame = stamped(*BLEACH[:2], frame='mobipick/base_link')
        self.assertFalse(self.p.overlaps_target('bleach_1', other_frame, list(BLEACH[2]), target, 'can_1'))
        self.params['~drop_boxes_overlapping_target'] = 0.0
        self.assertFalse(self.p.overlaps_target('bleach_1', stamped(*BLEACH[:2]), list(BLEACH[2]), target, 'can_1'))


def selector_object(class_id, instance_id, center, size):
    return NS(class_id=class_id, instance_id=instance_id,
              pose=stamped(center, I).pose, size=NS(x=size[0], y=size[1], z=size[2]))


class TestSceneWithAFalseBoxOnTheTarget(Params):
    def test_bleach_on_the_can_stays_out_the_klt_goes_in(self):
        p = self.p
        objects = [selector_object('bleach', 1, *BLEACH[::2]), selector_object('klt', 1, (0.3, 0.0, 0.79), (0.297, 0.197, 0.147)),
                   selector_object('tomato_soup_can', 1, CAN[0], (0.073, 0.036, 0.115))]
        p.pose_selector_get_all_poses_srv = lambda: NS(poses=NS(objects=objects))
        p.clamp_object_boxes_to_support = False
        p.global_reference_frame = 'map'
        p.transform_pose = lambda pose, frame: pose
        p.robot = mock.Mock()
        self.params['~open_set_complete_one_sided_boxes'] = False
        target = DetectedObject(class_id='tomato_soup_can', instance_id=1)
        target.header.frame_id = 'map'
        target.pose = stamped(*CAN[:2]).pose
        target.size.x, target.size.y, target.size.z = CAN[2]
        pose, box, _ = p.make_object_pose_and_add_objs_to_planning_scene(objectToPick('tomato_soup_can_1'),
                                                                         external_object=target)
        self.assertEqual(self.added, ['klt_1', 'tomato_soup_can_1'])
        self.assertEqual(self.removed, ['bleach_1'])   # left from an earlier goal
        self.assertEqual(box, list(CAN[2]))


def candidate(approach, closing, z, nn):
    '''a GraspCandidate at height z whose TCP x axis is approach and y axis closing'''
    x = [float(v) for v in approach]
    y = [float(v) for v in closing]
    zaxis = [x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]]
    m = [[x[0], y[0], zaxis[0], 0], [x[1], y[1], zaxis[1], 0], [x[2], y[2], zaxis[2], 0], [0, 0, 0, 1]]
    q = pick.tf.transformations.quaternion_from_matrix(m)
    c = GraspCandidate(quality=nn, width=0.05, scores=make_candidate_scores(nn, nn=nn))
    c.pose.position.z = z
    c.pose.orientation.x, c.pose.orientation.y, c.pose.orientation.z, c.pose.orientation.w = q
    return c


class TestGraspTypeHint(Params):
    def test_hint_keys(self):
        self.params['/mobipick/grasp_type_hints/tomato_soup_can'] = 'side'
        self.assertEqual(self.p.grasp_type_hint('Tomato-Soup can'), 'side')
        self.params['/mobipick/grasp_type_hints/strawberry'] = 'TOP'
        self.assertEqual(self.p.grasp_type_hint('strawberry'), 'top')

    def test_auto_missing_unknown_or_off_mean_geometry(self):
        self.params['/mobipick/grasp_type_hints/apple'] = 'auto'
        self.params['/mobipick/grasp_type_hints/pear'] = 'diagonal'
        self.assertIsNone(self.p.grasp_type_hint('apple'))
        self.assertIsNone(self.p.grasp_type_hint('pear'))
        self.assertIsNone(self.p.grasp_type_hint('banana'))
        self.params['/mobipick/grasp_type_hints/banana'] = 'top'
        self.params['~grasp_type_hints_ns'] = ''
        self.assertIsNone(self.p.grasp_type_hint('banana'))

    def test_second_name_as_fallback(self):
        self.params['/mobipick/grasp_type_hints/pringles'] = 'side'
        self.assertEqual(self.p.grasp_type_hint('the can', 'pringles'), 'side')

    def test_side_hint_ranks_a_side_grasp_first(self):
        top = candidate((0, 0, -1), (0, 1, 0), 0.80, 0.5)
        side = candidate((0, 1, 0), (1, 0, 0), 0.775, 0.4)   # horizontal approach and closing, middle height
        obj = DetectedObject()
        obj.pose = stamped(*CAN[:2]).pose
        obj.size.x, obj.size.y, obj.size.z = CAN[2]
        ranked = self.p.side_ranked([top, side], obj)
        self.assertAlmostEqual(ranked[0].quality, 0.6, places=5)   # 0.4 x 1.5
        self.assertAlmostEqual(ranked[1].quality, 0.5, places=5)
        self.assertAlmostEqual(top.quality, 0.5, places=5)          # the caller's list is untouched

    def test_hint_drives_the_top_rule(self):
        grasps = [Grasp(id='v'), Grasp(id='t30')]
        for g, tilt in zip(grasps, (0.0, 30.0)):
            half = math.radians(90.0 - tilt) / 2.0
            g.grasp_pose.pose.orientation.y, g.grasp_pose.pose.orientation.w = math.sin(half), math.cos(half)
        apple, strawberry = [0.076, 0.077, 0.064], [0.058, 0.049, 0.072]
        pose = stamped((0, 0, 0.75), I)
        kept, _, low = self.p.top_grasps_for_low_objects(list(grasps), [0.0, 0.0], pose, apple, 'top')
        self.assertTrue(low)
        self.assertEqual([g.id for g in kept], ['v'])
        kept, _, low = self.p.top_grasps_for_low_objects(list(grasps), [0.0, 0.0], pose, strawberry, 'side')
        self.assertFalse(low)
        self.assertEqual(len(kept), 2)


if __name__ == '__main__':
    unittest.main()
