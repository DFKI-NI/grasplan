'''~open_set_align_jaw_to_box: a parallel jaw across a box diagonal pinches two opposite edges, and the object slides
out in the lift (sim 2026-10-27: the gripper stalled at 11.7 cm on a gelatin box of 8.9 x 7.3 cm, its top-face
diagonal). The flag gives each near-vertical open-set grasp whose jaw is well off the box axes a copy turned about its
approach onto the nearest box axis whose side fits the open gripper and moved along its jaw onto the box centre line
(turned about an off-centre tcp alone, the jaws closed beside the box end), offered right before it. The grasp orientations
below are recorded AnyGrasp output for that box (model frame = box frame). ~open_set_align_jaw_narrowest_side puts
the copy across the narrowest side that fits first, the nearest-side copy second. Mocks rosparam (no ROS master); needs
the Mobipick image for the module imports.'''
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp

import grasplan.pick as pick
from grasplan.grasp_scoring import (closes_on_narrowest_side, jaw_alignment_turn, jaw_alignment_turns,
                                    jaw_centring_shift)
from grasplan.tools.cable_rank import order_by_predicted_stretch

BOX = [0.089, 0.073, 0.030]
DIAGONAL = [-0.3697760855460149, 0.5510188965450331, 0.36592401457241097, 0.6524901821236627]   # jaw 153 deg
ALONG_X = [0.758161501571576, 0.02440146876845195, -0.6515853160863746, 0.005681700211641379]   # jaw 2 deg off x
OFF_CENTRE = [0.3179197169356835, 0.6262915939239844, -0.23283616897560624, 0.6726612902348796]   # jaw 44 deg
I3 = np.eye(3).tolist()


def grasp(name, quaternion, position=(0.002, -0.006, 0.013)):
    g = Grasp()
    g.id = name
    g.grasp_pose.header.frame_id = 'map'
    g.grasp_pose.pose.position.x, g.grasp_pose.pose.position.y, g.grasp_pose.pose.position.z = position
    o = g.grasp_pose.pose.orientation
    o.x, o.y, o.z, o.w = quaternion
    g.grasp_quality = 0.8
    return g


def axes(g):
    o = g.grasp_pose.pose.orientation
    return tft.quaternion_matrix([o.x, o.y, o.z, o.w])[:3, :3]


def centre_along_jaw(g):
    '''the box centre (the origin) off the grasp's jaw midline, along the jaw'''
    p = g.grasp_pose.pose.position
    return float(np.dot(axes(g)[:, 1], -np.array([p.x, p.y, p.z])))


def jaw_deg(g):
    '''angle of the jaw's horizontal part to the box x axis, as a line (0..180)'''
    jaw = axes(g)[:, 1]
    return math.degrees(math.atan2(jaw[1], jaw[0])) % 180.0


class TestJawAlignmentTurn(unittest.TestCase):
    def jaw(self, deg):
        return (math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0.0)

    def test_turns_onto_the_nearest_fitting_axis(self):
        down = (0.0, 0.0, -1.0)
        self.assertEqual(jaw_alignment_turn(down, self.jaw(27), BOX, I3, 0.12), (27.0, 0.089))
        turn, span = jaw_alignment_turn(down, self.jaw(62), BOX, I3, 0.12)
        self.assertAlmostEqual(turn, -28.0)
        self.assertEqual(span, 0.073)

    def test_a_jaw_near_an_axis_stays(self):
        self.assertIsNone(jaw_alignment_turn((0.0, 0.0, -1.0), self.jaw(10), BOX, I3, 0.12))
        self.assertIsNone(jaw_alignment_turn((0.0, 0.0, -1.0), self.jaw(84), BOX, I3, 0.12))

    def test_only_a_side_that_fits(self):
        # the 8.9 cm side is nearer (27 deg) but does not fit: the 7.3 cm one (63 deg away) does
        turn, span = jaw_alignment_turn((0.0, 0.0, -1.0), self.jaw(27), BOX, I3, 0.08)
        self.assertAlmostEqual(turn, -63.0)
        self.assertEqual(span, 0.073)
        self.assertIsNone(jaw_alignment_turn((0.0, 0.0, -1.0), self.jaw(27), BOX, I3, 0.07))

    def test_centring_moves_only_along_the_jaw(self):
        shift = jaw_centring_shift((0.0, 1.0, 0.0), (0.01, -0.043, 0.03), (0.0, 0.0, 0.0))
        self.assertEqual(shift, (0.0, 0.043, 0.0))

    def test_side_grasps_stay(self):
        tilted = (math.sin(math.radians(40)), 0.0, -math.cos(math.radians(40)))
        self.assertIsNone(jaw_alignment_turn(tilted, self.jaw(117), BOX, I3, 0.12))


class TestJawAlignmentTurnsNarrowest(unittest.TestCase):
    DOWN = (0.0, 0.0, -1.0)

    def turns(self, deg, size=BOX, max_span=0.12, narrowest=True):
        jaw = (math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0.0)
        return [(round(turn, 6), span) for turn, span in
                jaw_alignment_turns(self.DOWN, jaw, size, I3, max_span, narrowest=narrowest)]

    def test_off_gives_what_jaw_alignment_turn_gives(self):
        for deg in (0, 10, 27, 44, 62, 84, 117, 153):
            single = jaw_alignment_turn(self.DOWN, (math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0.0),
                                        BOX, I3, 0.12)
            self.assertEqual(self.turns(deg, narrowest=False), [] if single is None else [(round(single[0], 6),
                                                                                           single[1])])

    def test_diagonal_jaw_narrow_side_first_then_the_nearest(self):
        self.assertEqual(self.turns(27), [(-63.0, 0.073), (27.0, 0.089)])

    def test_nearest_is_the_narrow_side_one_copy(self):
        self.assertEqual(self.turns(62), [(-28.0, 0.073)])

    def test_jaw_across_the_wider_side_is_turned_onto_the_narrow_one(self):
        self.assertEqual(self.turns(5), [(-85.0, 0.073)])
        self.assertEqual(self.turns(5, narrowest=False), [])

    def test_jaw_across_the_narrow_side_stays(self):
        self.assertEqual(self.turns(84), [])

    def test_equal_sides_count_as_one(self):
        self.assertEqual(self.turns(27, size=[0.08, 0.08, 0.03]), [(27.0, 0.08)])

    def test_only_sides_that_fit(self):
        # the 4 cm side is the narrowest, the 13 cm one does not fit: one copy, and none when nothing fits
        self.assertEqual(self.turns(10, size=[0.13, 0.04, 0.03]), [(-80.0, 0.04)])
        self.assertEqual(self.turns(27, size=[0.13, 0.125, 0.03]), [])


class TestClosesOnNarrowestSide(unittest.TestCase):
    DOWN = (0.0, 0.0, -1.0)

    def narrow(self, deg, size=BOX, approach=DOWN):
        jaw = (math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0.0)
        return closes_on_narrowest_side(approach, jaw, size, I3, 0.12)

    def test_jaw_along_the_narrow_axis_closes_across_the_narrow_side(self):
        self.assertTrue(self.narrow(90.0))
        self.assertTrue(self.narrow(84.0))   # within ALIGN_JAW_MIN_OFF_DEG of it

    def test_jaw_across_the_wider_side_or_a_diagonal(self):
        self.assertFalse(self.narrow(0.0))
        self.assertFalse(self.narrow(27.0))
        self.assertFalse(self.narrow(70.0))   # 20 deg off the narrow axis

    def test_tilted_grasp(self):
        tilted = (0.0, math.sin(math.radians(30.0)), -math.cos(math.radians(30.0)))
        self.assertFalse(self.narrow(90.0, approach=tilted))

    def test_equal_sides_and_sides_that_fit(self):
        self.assertTrue(self.narrow(0.0, size=[0.08, 0.08, 0.03]))
        self.assertTrue(self.narrow(90.0, size=[0.08, 0.08, 0.03]))
        self.assertTrue(self.narrow(90.0, size=[0.13, 0.04, 0.03]))
        self.assertFalse(self.narrow(90.0, size=[0.13, 0.125, 0.03]))   # no side fits the open gripper


class TestAlignJawsToBox(unittest.TestCase):
    def setUp(self):
        self.params = {'~open_set_align_jaw_to_box': True}
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.p = pick.PickTools.__new__(pick.PickTools)
        self.p.grasp_planner = NS(gripper_open_distance=[0.14])
        self.box_pose = PoseStamped()
        self.box_pose.header.frame_id = 'map'
        self.box_pose.pose.orientation.w = 1.0

    def align(self, grasps, size=BOX):
        return self.p.align_jaws_to_box(grasps, [0.1] * len(grasps), self.box_pose, size)

    def test_off_by_default(self):
        del self.params['~open_set_align_jaw_to_box']
        grasps = [grasp('anygrasp_3', DIAGONAL)]
        self.assertEqual(self.align(grasps), (grasps, [0.1]))

    def test_diagonal_grasp_gets_an_aligned_copy_first(self):
        original = grasp('anygrasp_3', DIAGONAL)
        self.assertGreater(min(jaw_deg(original), 180.0 - jaw_deg(original)), 15.0)
        grasps, widths = self.align([original])
        self.assertEqual([g.id for g in grasps], ['anygrasp_3_aligned-27', 'anygrasp_3'])
        twin = grasps[0]
        deg = jaw_deg(twin)
        self.assertLess(min(deg, 180.0 - deg), 2.0)   # jaw along x: it closes on the two faces 8.9 cm apart
        np.testing.assert_allclose(axes(twin)[:, 0], axes(original)[:, 0], atol=1e-9)   # same approach
        self.assertLess(abs(centre_along_jaw(twin)), 1e-9)
        self.assertEqual(widths, [0.1, 0.1])
        self.assertIs(grasps[1], original)

    def test_off_centre_copy_moves_onto_the_box_centre_line(self):
        # recorded grasp 0: tcp 4.3 cm off the centre along the 8.9 cm side, jaw 44 deg off: turned onto that side,
        # its jaws closed beside the box end (sim 2026-10-27)
        original = grasp('anygrasp_0', OFF_CENTRE, position=(-0.043, 0.004, 0.011))
        grasps, _ = self.align([original])
        twin = grasps[0]
        self.assertTrue(twin.id.startswith('anygrasp_0_aligned'))
        self.assertGreater(abs(centre_along_jaw(original)), 0.02)
        self.assertLess(abs(centre_along_jaw(twin)), 1e-9)
        position = lambda g: np.array([g.grasp_pose.pose.position.x, g.grasp_pose.pose.position.y,
                                       g.grasp_pose.pose.position.z])
        moved = position(twin) - position(original)
        np.testing.assert_allclose(np.cross(moved, axes(twin)[:, 1]), 0.0, atol=1e-9)   # only along the jaw
        self.assertEqual(original.grasp_pose.pose.position.x, -0.043)   # the original stays where it was

    def test_box_turned_in_yaw(self):
        yaw = math.radians(30)
        q = tft.quaternion_from_euler(0.0, 0.0, yaw)
        o = self.box_pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        turned = tft.quaternion_multiply(q, DIAGONAL)   # the same grasp on the turned box
        grasps, _ = self.align([grasp('anygrasp_3', turned)])
        deg = (jaw_deg(grasps[0]) - 30.0) % 180.0
        self.assertLess(min(deg, 180.0 - deg), 2.0)

    def test_recorded_diagonal_grasp_narrow_side_first(self):
        self.params['~open_set_align_jaw_narrowest_side'] = True
        original = grasp('anygrasp_3', DIAGONAL)
        grasps, _ = self.align([original])
        self.assertEqual([g.id for g in grasps], ['anygrasp_3_aligned+63', 'anygrasp_3_aligned-27', 'anygrasp_3'])
        deg = jaw_deg(grasps[0])
        self.assertLess(abs(deg - 90.0), 2.0)   # jaw along y: it closes on the two faces 7.3 cm apart
        self.assertIs(grasps[2], original)

    def test_aligned_grasp_stays_alone(self):
        grasps = [grasp('anygrasp_2', ALONG_X)]
        self.assertEqual(self.align(grasps)[0], grasps)

    def test_round_box_keeps_its_grasps(self):
        grasps = [grasp('anygrasp_3', DIAGONAL)]
        self.assertEqual(self.align(grasps, size=[0.07, 0.07, 0.065])[0], grasps)

    def test_no_side_fits_the_open_gripper(self):
        grasps = [grasp('anygrasp_3', DIAGONAL)]
        self.assertEqual(self.align(grasps, size=[0.13, 0.125, 0.03])[0], grasps)


def straight_down(jaw_deg):
    '''the quaternion of a grasp approaching straight down (gripper_tcp x) with its jaw (y) at jaw_deg in the plane'''
    c, s = math.cos(math.radians(jaw_deg)), math.sin(math.radians(jaw_deg))
    m = np.identity(4)
    m[:3, 0], m[:3, 1], m[:3, 2] = (0.0, 0.0, -1.0), (c, s, 0.0), (s, -c, 0.0)
    return tft.quaternion_from_matrix(m)


def centre_along_jaw_at(g, centre):
    '''a box centre off the grasp's jaw midline, along the jaw'''
    p = g.grasp_pose.pose.position
    return float(np.dot(axes(g)[:, 1], np.array(centre) - np.array([p.x, p.y, p.z])))


class TestInventedBox(unittest.TestCase):
    '''a box no recorded or known object has (11.3 x 4.1 x 2.6 cm, turned 23 deg on the table): the alignment sees only
    the detected box, its pose and the gripper's open distance, so it works the same on it'''
    INVENTED = [0.113, 0.041, 0.026]
    YAW = 23.0
    CENTRE = (1.7, -0.4, 0.02)

    def setUp(self):
        self.params = {'~open_set_align_jaw_to_box': True}
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.p = pick.PickTools.__new__(pick.PickTools)
        self.p.grasp_planner = NS(gripper_open_distance=[0.14])
        self.box_pose = PoseStamped()
        self.box_pose.header.frame_id = 'map'
        o = self.box_pose.pose.orientation
        o.x, o.y, o.z, o.w = tft.quaternion_from_euler(0.0, 0.0, math.radians(self.YAW))
        p = self.box_pose.pose.position
        p.x, p.y, p.z = self.CENTRE

    def align(self, jaw_off_deg, size=None):
        '''one grasp straight down, 1.4 cm off the box centre, its jaw jaw_off_deg off the box x axis'''
        g = grasp('anygrasp_7', straight_down(self.YAW + jaw_off_deg), position=(1.71, -0.39, 0.02))
        return self.p.align_jaws_to_box([g], [0.05], self.box_pose, size or self.INVENTED)

    def off_box_x(self, g):
        '''the jaw's angle to the box x axis (the 11.3 cm side), as a line (0..90)'''
        deg = (jaw_deg(g) - self.YAW) % 180.0
        return min(deg, 180.0 - deg)

    def test_diagonal_jaw_turns_onto_the_nearest_fitting_side(self):
        # 40 deg off the 11.3 cm axis, 50 deg off the 4.1 cm one: the nearest fitting axis is the 11.3 cm one
        grasps, widths = self.align(40.0)
        self.assertEqual([g.id for g in grasps], ['anygrasp_7_aligned+40', 'anygrasp_7'])
        self.assertLess(self.off_box_x(grasps[0]), 0.5)
        self.assertAlmostEqual(widths[0], 0.113)
        self.assertAlmostEqual(centre_along_jaw_at(grasps[0], self.CENTRE), 0.0, places=9)

    def test_invented_round_box_keeps_its_grasps(self):
        grasps, _ = self.align(40.0, size=[0.057, 0.051, 0.046])
        self.assertEqual([g.id for g in grasps], ['anygrasp_7'])

    def test_narrowest_side_first_then_the_nearest_then_the_original(self):
        self.params['~open_set_align_jaw_narrowest_side'] = True
        grasps, widths = self.align(40.0)
        self.assertEqual([g.id for g in grasps], ['anygrasp_7_aligned-50', 'anygrasp_7_aligned+40', 'anygrasp_7'])
        self.assertGreater(self.off_box_x(grasps[0]), 89.5)   # jaw along the 4.1 cm side: closes across it
        self.assertLess(self.off_box_x(grasps[1]), 0.5)
        # the box turned 23 deg: the narrow-side jaw at 113 deg, perpendicular to the nearest-side one at 23 deg
        self.assertAlmostEqual(jaw_deg(grasps[0]), self.YAW + 90.0, places=6)
        self.assertAlmostEqual(jaw_deg(grasps[1]), self.YAW, places=6)
        np.testing.assert_allclose(widths, [0.05, 0.113, 0.05])
        for g in grasps[:2]:
            self.assertAlmostEqual(centre_along_jaw_at(g, self.CENTRE), 0.0, places=9)

    def two_grasps(self):
        '''the better grasp 40 deg off the 11.3 cm axis (a diagonal), the worse one already closing across the 4.1 cm
        side (5 deg off its axis), in AnyGrasp's quality order'''
        best = grasp('anygrasp_7', straight_down(self.YAW + 40.0), position=(1.71, -0.39, 0.02))
        best.grasp_quality = 1.9
        worse = grasp('anygrasp_9', straight_down(self.YAW + 85.0), position=(1.69, -0.41, 0.02))
        worse.grasp_quality = 1.2
        return self.p.align_jaws_to_box([best, worse], [0.05, 0.04], self.box_pose, self.INVENTED)

    def test_narrow_side_grasps_go_before_a_better_grasps_wide_copy(self):
        self.params['~open_set_align_jaw_narrowest_side'] = True
        grasps, widths = self.two_grasps()
        self.assertEqual([g.id for g in grasps],
                         ['anygrasp_7_aligned-50', 'anygrasp_9', 'anygrasp_7_aligned+40', 'anygrasp_7'])
        np.testing.assert_allclose(widths, [0.05, 0.04, 0.113, 0.05])

    def test_without_the_narrowest_flag_the_quality_order_stays(self):
        grasps, _ = self.two_grasps()
        self.assertEqual([g.id for g in grasps], ['anygrasp_7_aligned+40', 'anygrasp_7', 'anygrasp_9'])

    def test_cable_ranking_keeps_the_narrow_side_first_in_each_group(self):
        # the cable ranking still decides first: a grasp predicted to be refused stays behind the predicted-pass ones
        self.params['~open_set_align_jaw_narrowest_side'] = True
        grasps, _ = self.two_grasps()
        ordered, passing = order_by_predicted_stretch(grasps, [-0.2, -0.05, -0.3, -0.2], -0.115)
        self.assertEqual(passing, 3)
        self.assertEqual([g.id for g in ordered],
                         ['anygrasp_7_aligned-50', 'anygrasp_7_aligned+40', 'anygrasp_7', 'anygrasp_9'])

    def test_narrowest_turns_a_grasp_across_the_long_side(self):
        self.params['~open_set_align_jaw_narrowest_side'] = True
        grasps, _ = self.align(8.0)   # closes across the 11.3 cm side now: on an axis, today's flag leaves it
        self.assertEqual([g.id for g in grasps], ['anygrasp_7_aligned-82', 'anygrasp_7'])
        self.params['~open_set_align_jaw_narrowest_side'] = False
        self.assertEqual([g.id for g in self.align(8.0)[0]], ['anygrasp_7'])

    def test_narrowest_needs_the_alignment_flag(self):
        self.params = {'~open_set_align_jaw_narrowest_side': True}
        self.assertEqual([g.id for g in self.align(40.0)[0]], ['anygrasp_7'])

    def test_narrowest_keeps_round_boxes(self):
        self.params['~open_set_align_jaw_narrowest_side'] = True
        self.assertEqual([g.id for g in self.align(40.0, size=[0.057, 0.051, 0.046])[0]], ['anygrasp_7'])

    def test_invented_box_wider_than_the_open_gripper_on_both_sides(self):
        grasps, _ = self.align(40.0, size=[0.135, 0.126, 0.03])
        self.assertEqual([g.id for g in grasps], ['anygrasp_7'])


if __name__ == '__main__':
    unittest.main()
