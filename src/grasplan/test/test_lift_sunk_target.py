'''~lift_sunk_target_to_support (default off): a closed-set pick target whose box bottom reaches into its support (real
robot: the known-object pose estimate is mostly a few mm low, a soup can 7 to 13 mm, and its grasps were built from
that low pose) is raised whole until its bottom stands 1 mm above the support top, at most ~lift_sunk_target_max, so the
grasps and the scene box start from where it stands. Never lowers; tilted, off-support, floating and too deep targets
and open-set targets stay as they are; off it changes nothing. Mocks for the scene, the pose selector and rosparam (no
ROS master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from test_false_boxes_and_hints import I, TABLE
from test_pick_target_log import SOUP, detected_object, picker, selector_object

import grasplan.pick as pick
from grasplan.tools.common import objectToPick
from grasplan.tools.topdown_grasps import clamp_box_to_support, lift_onto_support

TOP = 0.72
RESTING = TOP + SOUP[2] / 2.0   # centre z of the soup box standing on the table
TILT_25 = (0.0, 0.21643961393810288, 0.0, 0.9762960071199334)   # 25 deg about y


def bottom(z, size=SOUP):
    return z - size[2] / 2.0


class TestLiftOntoSupport(unittest.TestCase):
    def test_sunk_targets_stand_1_mm_above_the_top(self):
        for sink in (0.007, 0.013):
            z, lifted, too_deep = lift_onto_support((0.0, 0.0, RESTING - sink), I, SOUP, TABLE, 0.03)
            self.assertAlmostEqual(bottom(z), TOP + 0.001, places=6)
            self.assertAlmostEqual(lifted, sink + 0.001, places=6)
            self.assertEqual(too_deep, 0.0)
            # the scene box of the raised pose has nothing left to clamp: it goes in whole
            _, size, again = clamp_box_to_support((0.0, 0.0, z), I, SOUP, TABLE)
            self.assertEqual(again, 0.0)
            self.assertEqual(list(size), list(SOUP))

    def test_too_deep_is_left_and_said(self):
        z, lifted, too_deep = lift_onto_support((0.0, 0.0, RESTING - 0.04), I, SOUP, TABLE, 0.03)
        self.assertEqual((z, lifted), (RESTING - 0.04, 0.0))
        self.assertAlmostEqual(too_deep, 0.04, places=6)

    def test_left_as_they_are(self):
        cases = {
            'tilted 25 deg': ((0.0, 0.0, RESTING - 0.01), TILT_25),
            'centre off the table': ((1.2, 0.0, RESTING - 0.01), I),
            'floating': ((0.0, 0.0, RESTING + 0.02), I),
            'resting 2 mm above': ((0.0, 0.0, RESTING + 0.002), I),
        }
        for case, (center, orientation) in cases.items():
            z, lifted, too_deep = lift_onto_support(center, orientation, SOUP, TABLE, 0.03)
            self.assertEqual((z, lifted, too_deep), (center[2], 0.0, 0.0), case)


class TestPickTarget(unittest.TestCase):
    def setUp(self):
        self.params = {'~lift_sunk_target_to_support': True}
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)

    def pick(self, p, target='soup_1', **kwargs):
        with mock.patch.object(pick.rospy, 'loginfo') as info, mock.patch.object(pick.rospy, 'logwarn') as warn:
            pose, box, _ = p.make_object_pose_and_add_objs_to_planning_scene(objectToPick(target), **kwargs)
        said = [c[0][0] for c in info.call_args_list + warn.call_args_list]
        return pose, box, said

    def test_raised_pose_is_returned_and_its_box_goes_in_whole(self):
        for target in ('soup_1', 'soup'):   # the named instance and any instance of the class
            p = picker([selector_object('soup_1', RESTING - 0.013)])
            pose, box, said = self.pick(p, target)
            self.assertAlmostEqual(bottom(pose.pose.position.z), TOP + 0.001, places=6)
            scene_z, scene_size = p.added['soup_1']
            self.assertAlmostEqual(scene_z, pose.pose.position.z, places=9)   # the scene box where the grasps start
            self.assertEqual(scene_size, list(SOUP))                           # whole, not clamped
            self.assertIn(f'soup_1: pose raised 1.4 cm onto its support (was z {RESTING - 0.013:.3f})', said)
            self.assertTrue(any(s.startswith('pick target soup_1') and '(above it)' in s for s in said))

    def test_too_deep_target_stays_as_perceived(self):
        p = picker([selector_object('soup_1', RESTING - 0.04)])
        pose, _, said = self.pick(p)
        self.assertAlmostEqual(pose.pose.position.z, RESTING - 0.04)
        self.assertIn('soup_1: pose 4.0 cm inside its support, more than the limit 3.0 cm: left as perceived', said)

    def test_limit_is_a_parameter(self):
        self.params['~lift_sunk_target_max'] = 0.05
        p = picker([selector_object('soup_1', RESTING - 0.04)])
        pose, _, _ = self.pick(p)
        self.assertAlmostEqual(bottom(pose.pose.position.z), TOP + 0.001, places=6)

    def test_off_is_as_before(self):
        self.params['~lift_sunk_target_to_support'] = False
        p = picker([selector_object('soup_1', RESTING - 0.013)])
        pose, _, said = self.pick(p)
        self.assertAlmostEqual(pose.pose.position.z, RESTING - 0.013)    # grasps from the perceived pose
        scene_z, scene_size = p.added['soup_1']
        self.assertAlmostEqual(scene_size[2], SOUP[2] - 0.014, places=6)  # only the scene box clamped, as today
        self.assertFalse(any('pose raised' in s for s in said))

    def test_needs_the_supports(self):
        p = picker([selector_object('soup_1', RESTING - 0.013)], clamp=False)
        pose, _, _ = self.pick(p)
        self.assertAlmostEqual(pose.pose.position.z, RESTING - 0.013)

    def test_obstacles_are_not_raised(self):
        p = picker([selector_object('soup_1', RESTING), selector_object('box_1', RESTING - 0.01, x=0.3)])
        self.pick(p)
        scene_z, scene_size = p.added['box_1']
        self.assertAlmostEqual(scene_z, RESTING - 0.01 + 0.011 / 2.0, places=6)   # clamped as today, not raised
        self.assertAlmostEqual(scene_size[2], SOUP[2] - 0.011, places=6)

    def test_pose_selector_answer_is_not_changed(self):
        soup = selector_object('soup_1', RESTING - 0.013)
        self.pick(picker([soup]))
        self.assertAlmostEqual(soup.pose.position.z, RESTING - 0.013)

    def test_open_set_target_is_left_alone(self):
        p = picker([])
        p.complete_one_sided_box = lambda name, pose, size: None
        target = detected_object('pear', 1, 'map', (0.0, 0.0, TOP + 0.04 - 0.01), (0.06, 0.06, 0.08))
        pose, _, said = self.pick(p, 'pear_1', external_object=target)
        self.assertAlmostEqual(pose.pose.position.z, TOP + 0.03)
        self.assertFalse(any('pose raised' in s for s in said))

    def test_supports_of_its_own_frame(self):
        # two robots in one sim: the pose selector list of robot 2 is in its map frame, so are its supports
        objects = [selector_object('soup_1', RESTING - 0.013)]
        p = picker(objects)
        p.pose_selector_get_all_poses_srv = lambda: NS(poses=NS(objects=objects, header=NS(frame_id='robot2/map')))
        p.support_boxes = lambda exclude=(): {'robot2/map': TABLE}
        pose, _, _ = self.pick(p)
        self.assertEqual(pose.header.frame_id, 'robot2/map')
        self.assertAlmostEqual(bottom(pose.pose.position.z), TOP + 0.001, places=6)
        # supports only in another frame: nothing to compare with, nothing raised
        p = picker([selector_object('soup_1', RESTING - 0.013)])
        p.pose_selector_get_all_poses_srv = lambda: NS(poses=NS(objects=objects, header=NS(frame_id='robot2/map')))
        with mock.patch('rospy.logwarn_once'):
            pose, _, _ = self.pick(p)
        self.assertAlmostEqual(pose.pose.position.z, RESTING - 0.013)


if __name__ == '__main__':
    unittest.main()
