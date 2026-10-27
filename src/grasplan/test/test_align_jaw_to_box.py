'''~open_set_align_jaw_to_box: a parallel jaw across a box diagonal pinches two opposite edges, and the object slides
out in the lift (sim 2026-10-27: the gripper stalled at 11.7 cm on a gelatin box of 8.9 x 7.3 cm, its top-face
diagonal). The flag gives each near-vertical open-set grasp whose jaw is well off the box axes a copy turned about its
approach onto the nearest box axis whose side fits the open gripper and moved along its jaw onto the box centre line
(turned about an off-centre tcp alone, the jaws closed beside the box end), offered right before it. The grasp orientations
below are recorded AnyGrasp output for that box (model frame = box frame). Mocks rosparam (no ROS master); needs the
Mobipick image for the module imports.'''
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp

import grasplan.pick as pick
from grasplan.grasp_scoring import jaw_alignment_turn, jaw_centring_shift

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

    def test_aligned_grasp_stays_alone(self):
        grasps = [grasp('anygrasp_2', ALONG_X)]
        self.assertEqual(self.align(grasps)[0], grasps)

    def test_round_box_keeps_its_grasps(self):
        grasps = [grasp('anygrasp_3', DIAGONAL)]
        self.assertEqual(self.align(grasps, size=[0.07, 0.07, 0.065])[0], grasps)

    def test_no_side_fits_the_open_gripper(self):
        grasps = [grasp('anygrasp_3', DIAGONAL)]
        self.assertEqual(self.align(grasps, size=[0.13, 0.125, 0.03])[0], grasps)


if __name__ == '__main__':
    unittest.main()
