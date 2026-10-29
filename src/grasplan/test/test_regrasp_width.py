'''the width the re-grasp trusts. The width across the pads is the extent along the TCP y of the part of the object
inside the pad footprint (x within the pad length behind the tip, |z| within half the pad width), computed from the
perceived box at the full object pose (the round shapes below are only test oracles for the geometry, never robot knowledge). Checked against hand-computed values (the partner's counterexamples) and a
brute-force surface sampler, a second implementation. Closed-set widths stay the box extent. The AnyGrasp opening is only
an upper bound. Needs the Mobipick image for the module imports (numpy, scipy).'''
import math
import unittest

import numpy as np

import grasplan.pick as pick
from grasplan.tools.pad_footprint import box_vertices, footprint_width, width_across_pads
from grasplan.tools.pad_gap import perceived_width, round_cross_section

CAN = [0.075, 0.075, 0.23]
X_TIP = 0.03   # footprint x in [-0.035, 0.03]
SIDES = 72


def cylinder_vertices(diameter, length, axis):
    '''test oracle: circumscribed 72-sided prism of a cylinder centred at the origin, axis along object axis index axis'''
    radius = diameter / 2.0 / math.cos(math.pi / SIDES)
    angles = np.arange(SIDES) * 2.0 * math.pi / SIDES
    others = [i for i in range(3) if i != axis]
    out = []
    for z in (-length / 2.0, length / 2.0):
        for u, v in np.stack([radius * np.cos(angles), radius * np.sin(angles)], axis=1):
            point = [0.0, 0.0, 0.0]
            point[others[0]], point[others[1]], point[axis] = u, v, z
            out.append(point)
    return np.array(out)


def ball_vertices(diameter, count=400):
    '''test oracle: Fibonacci points on a sphere'''
    i = np.arange(count) + 0.5
    phi = np.arccos(1.0 - 2.0 * i / count)
    theta = math.pi * (1.0 + 5.0 ** 0.5) * i
    return diameter / 2.0 * np.stack([np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)], axis=1)


def rot_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def pose(rotation=None, translation=(0, 0, 0)):
    t = np.eye(4)
    if rotation is not None:
        t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def tilted_can(deg):
    # TCP y tilted out of the horizontal by deg: the can axis in the TCP frame is (0, sin, cos)
    return pose(rot_x(math.radians(deg)).T)


def brute_cylinder(diameter, length, axis, object_in_tcp, n_angle=7200, n_axis=600):
    '''second implementation: y extent of dense surface points (side and end disks) of the cylinder inside the footprint'''
    r = diameter / 2.0
    ys = []
    others = [i for i in range(3) if i != axis]
    for chunk in np.array_split(np.arange(n_angle), 12):
        ang = chunk * 2 * math.pi / n_angle
        a = np.linspace(-length / 2, length / 2, n_axis)
        # side points: one (angle, a) grid
        ca, sa = np.cos(ang)[:, None], np.sin(ang)[:, None]
        side = np.zeros((len(ang), n_axis, 3))
        side[:, :, others[0]] = r * ca
        side[:, :, others[1]] = r * sa
        side[:, :, axis] = a[None, :]
        ys.append(inside_y(side.reshape(-1, 3), object_in_tcp))
    rng = np.random.default_rng(1)
    for cap in (-length / 2, length / 2):   # end disks, uniform random
        rad = r * np.sqrt(rng.random(400000))
        ang = rng.random(400000) * 2 * math.pi
        disk = np.zeros((400000, 3))
        disk[:, others[0]], disk[:, others[1]], disk[:, axis] = rad * np.cos(ang), rad * np.sin(ang), cap
        ys.append(inside_y(disk, object_in_tcp))
    ys = np.concatenate([y for y in ys if len(y)])
    return float(ys.max() - ys.min()) if len(ys) else None


def inside_y(points, object_in_tcp, x_tip=X_TIP):
    p = points.dot(object_in_tcp[:3, :3].T) + object_in_tcp[:3, 3]
    keep = (p[:, 0] <= x_tip) & (p[:, 0] >= x_tip - 0.065) & (np.abs(p[:, 2]) <= 0.0135)
    return p[keep, 1]


class TestFootprintHandComputed(unittest.TestCase):
    def width(self, vertices, object_in_tcp):
        return footprint_width(vertices, object_in_tcp, X_TIP)

    def test_partner_cases_tilted_can(self):
        can = cylinder_vertices(0.075, 0.23, 2)
        self.assertAlmostEqual(self.width(can, tilted_can(30)), 0.102191, delta=0.0006)   # d/cos + 27 tan
        self.assertAlmostEqual(self.width(can, tilted_can(40)), 0.120561, delta=0.0006)

    def test_straight_can_is_the_diameter_at_any_yaw(self):
        can = cylinder_vertices(0.075, 0.23, 2)
        for yaw in (0.0, 0.5, math.pi / 4, 1.3):
            self.assertAlmostEqual(self.width(can, pose(rot_z(yaw))), 0.075, delta=0.0005)

    def test_translation_along_y_does_not_change_the_width(self):
        can = cylinder_vertices(0.075, 0.23, 2)
        self.assertAlmostEqual(self.width(can, pose(translation=(0, 0.02, 0))), 0.075, delta=0.0005)
        self.assertAlmostEqual(self.width(can, pose(tilted_can(30)[:3, :3], (0, 0.02, 0))), 0.102191, delta=0.0006)

    def test_translation_along_z_keeps_the_tilted_width(self):
        # the cut slides along the tilted can: a parallelogram, same y span while inside its length
        can = cylinder_vertices(0.075, 0.23, 2)
        self.assertAlmostEqual(self.width(can, pose(tilted_can(30)[:3, :3], (0, 0, 0.04))), 0.102191, delta=0.0006)

    def test_footprint_cut_by_translation_along_x(self):
        can = cylinder_vertices(0.075, 0.23, 2)
        self.assertIsNone(self.width(can, pose(translation=(0.15, 0, 0))))   # beyond the pad tips: nothing between the pads
        # only the near edge of the can reaches the footprint: a narrower cut than the diameter
        edge = self.width(can, pose(translation=(0.03 + 0.0375 - 0.01, 0, 0)))
        self.assertAlmostEqual(edge, 2 * math.sqrt(0.0375 ** 2 - 0.0275 ** 2), delta=0.0007)

    def test_sphere_is_its_diameter_whatever_the_orientation(self):
        ball = ball_vertices(0.075)
        for rotation in (np.eye(3), rot_x(0.7), rot_z(1.1).dot(rot_x(0.3))):
            self.assertAlmostEqual(self.width(ball, pose(rotation)), 0.075, delta=0.0006)

    def test_square_prism_at_45_deg_is_the_diagonal_unlike_a_can(self):
        prism = box_vertices(CAN)
        self.assertAlmostEqual(self.width(prism, pose(rot_z(math.pi / 4))), 0.075 * math.sqrt(2), delta=0.0003)
        self.assertAlmostEqual(self.width(cylinder_vertices(0.075, 0.23, 2), pose(rot_z(math.pi / 4))), 0.075, delta=0.0005)

    def test_lying_cylinder_across_the_fingers_is_cut_to_the_pad_length(self):
        # a can lying along the finger axis: the part between the pads is its whole length within |z| <= 13.5 mm
        lying = cylinder_vertices(0.075, 0.23, 1)   # axis y
        self.assertAlmostEqual(self.width(lying, pose()), 0.23, delta=0.0005)


class TestFootprintAgainstBruteForce(unittest.TestCase):
    def test_cylinder_poses(self):
        cases = [tilted_can(30), tilted_can(40), pose(rot_z(0.6).dot(tilted_can(20)[:3, :3]), (0.005, 0.015, -0.02)),
                 pose(rot_x(0.4), (-0.01, 0.0, 0.03)), pose(tilted_can(25)[:3, :3].dot(rot_z(0.8)), (0.01, -0.02, 0.0))]
        for case in cases:
            fast = footprint_width(cylinder_vertices(0.075, 0.23, 2), case, X_TIP)
            slow = brute_cylinder(0.075, 0.23, 2, case)
            self.assertIsNotNone(fast)
            self.assertAlmostEqual(fast, slow, delta=0.0015, msg=str(case.round(2).tolist()))


class TestBoxWidth(unittest.TestCase):
    def test_source_and_confidence(self):
        # no per-object knowledge: only the perceived box. Two equal sides could hide a round object: confidence low
        self.assertEqual(width_across_pads(CAN, pose())[1:], ('box-footprint', 'low'))
        self.assertEqual(width_across_pads([0.2, 0.06, 0.04], pose(rot_z(0.3)))[1:], ('box-footprint', 'medium'))

    def test_box_is_the_footprint_extent_of_the_box(self):
        width, _, _ = width_across_pads(CAN, pose(rot_z(math.pi / 4)))
        self.assertAlmostEqual(width, 0.075 * math.sqrt(2), delta=0.0003)

    def test_box_cut_by_the_footprint_is_narrower_than_the_full_extent(self):
        dims = [0.04, 0.2, 0.06]   # a long box tilted about z: the part between the pads is shorter than the whole
        width, _, _ = width_across_pads(dims, pose(rot_z(0.8)))
        self.assertLess(width, 0.2 * math.cos(0.8) + 0.04 * math.sin(0.8) - 0.01)
        self.assertGreater(width, 0.0)

    def test_missing_scipy_falls_back_without_raising(self):
        import sys
        from unittest import mock
        with mock.patch.dict(sys.modules, {'scipy': None, 'scipy.optimize': None}):
            with self.assertLogs('grasplan.tools.pad_footprint', level='WARNING'):
                width, source, confidence = width_across_pads(CAN, pose(rot_z(0.4)))
        self.assertAlmostEqual(width, 0.075 * (math.cos(0.4) + math.sin(0.4)), delta=1e-6)
        self.assertEqual((source, confidence), ('box-extent', 'low'))

    def test_nothing_in_the_footprint_falls_back_to_the_box_extent(self):
        width, source, confidence = width_across_pads(CAN, pose(translation=(0.3, 0, 0)))
        self.assertAlmostEqual(width, 0.075, delta=1e-6)
        self.assertEqual((source, confidence), ('box-extent', 'low'))

    def test_no_object_class_input_exists(self):
        import inspect
        self.assertEqual(list(inspect.signature(width_across_pads).parameters), ['dims', 'object_in_tcp', 'opening'])

    def test_round_cross_section_flag(self):
        self.assertIsNotNone(round_cross_section(CAN))
        self.assertIsNone(round_cross_section([0.20, 0.06, 0.04]))


class TestPerceivedWidth(unittest.TestCase):
    def test_opening_is_an_upper_bound_not_a_veto(self):
        self.assertEqual(perceived_width(0.094, 0.075), (0.075, False))   # margin of 19 mm: fine, the shape counts
        self.assertEqual(perceived_width(0.070, 0.075), (0.070, False))   # tighter opening within 10 mm

    def test_object_wider_than_the_opening_disagrees(self):
        self.assertEqual(perceived_width(0.070, 0.090), (0.070, True))

    def test_tilted_can_wider_than_the_opening_is_skipped(self):
        # the opening of 94 mm against the 102 mm geometric span: 8 mm over, within the margin; 110 mm span at 40 deg: no
        self.assertFalse(perceived_width(0.094, 0.102191)[1])
        self.assertTrue(perceived_width(0.094, 0.120561)[1])

    def test_one_input_only(self):
        self.assertEqual(perceived_width(None, 0.075), (0.075, False))
        self.assertEqual(perceived_width(0.094, None), (0.094, False))
        self.assertEqual(perceived_width(None, None), (None, False))


class Pose:
    def __init__(self, rotation):
        from tf.transformations import quaternion_from_matrix
        m = np.eye(4)
        m[:3, :3] = rotation
        q = quaternion_from_matrix(m)
        self.orientation = type('Q', (), {'x': q[0], 'y': q[1], 'z': q[2], 'w': q[3]})()


class Grasp:
    def __init__(self, rotation):
        self.grasp_pose = type('S', (), {'pose': Pose(rotation)})()


class TestClosedSetWidthUnchanged(unittest.TestCase):
    def test_box_extent_along_the_finger_axis(self):
        box = type('S', (), {'pose': Pose(rot_z(math.pi / 4))})()
        self.assertAlmostEqual(pick.PickTools.box_width_for_grasp(Grasp(np.eye(3)), box, CAN), 0.075 * math.sqrt(2), places=4)
        self.assertAlmostEqual(pick.PickTools.box_width_for_grasp(Grasp(np.eye(3)), box, [0.2, 0.06, 0.04]), 0.26 * math.sqrt(0.5), delta=0.0005)
        self.assertIsNone(pick.PickTools.box_width_for_grasp(Grasp(np.eye(3)), box, None))


if __name__ == '__main__':
    unittest.main()
