'''#151 low objects (Oscar, 2026-09-28, real strawberry): an object lower than 5 cm, or with both horizontal sides below
6 cm (a filled-in, too tall box of a small dark object), is grasped from the top only: open-set grasps more than 10 deg
off vertical are dropped and the others lowered to the fingertip clearance above the table. Fake planning scene and
fingertip envelope (no ROS master); needs the Mobipick image for the module imports.'''
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Grasp

import grasplan.pick as pick
from grasplan import grasp_scoring as gs

STRAWBERRY = (0.058, 0.049, 0.072)   # real 2026-09-28: depth filled in, taller than the apple
APPLE = (0.076, 0.077, 0.064)
TENNIS_BALL = (0.067, 0.067, 0.067)
PEAR = (0.101, 0.063, 0.069)
BANANA = (0.19, 0.045, 0.04)          # lying
TOP = 0.72                            # table top in the fake scene


class TestLowObjectRule(unittest.TestCase):
    def test_oscar_examples(self):
        self.assertTrue(gs.is_low_object(STRAWBERRY))        # by its footprint: 5.8 x 4.9 cm
        self.assertFalse(gs.is_low_object(APPLE))            # 6.4 cm tall keeps the side grasp
        self.assertFalse(gs.is_low_object(TENNIS_BALL))
        self.assertFalse(gs.is_low_object(PEAR))
        self.assertTrue(gs.is_low_object(BANANA))            # 4 cm tall

    def test_footprint_rule_can_be_turned_off(self):
        self.assertFalse(gs.is_low_object(STRAWBERRY, max_footprint=0.0))
        self.assertTrue(gs.is_low_object((0.058, 0.049, 0.045), max_footprint=0.0))

    def test_height_is_the_extent_along_up_of_a_rotated_box(self):
        lying = ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0))   # box y axis vertical
        height, footprint = gs.box_height_and_footprint((0.2, 0.04, 0.09), lying)
        self.assertAlmostEqual(height, 0.04)
        self.assertAlmostEqual(footprint, 0.2)
        self.assertTrue(gs.is_low_object((0.2, 0.04, 0.09), lying))

    def test_no_size_is_never_low(self):
        self.assertFalse(gs.is_low_object((0.0, 0.0, 0.0)))

    def test_tilt_of_the_real_strawberry_grasp(self):
        self.assertAlmostEqual(gs.tilt_from_vertical_deg((0.0, 0.0, -1.0)), 0.0)
        self.assertAlmostEqual(gs.tilt_from_vertical_deg((0.155, -0.228, -0.961)), 16.0, delta=0.2)   # anygrasp_0
        self.assertAlmostEqual(gs.tilt_from_vertical_deg((1.0, 0.0, 0.0)), 90.0)


def grasp(gid, tilt_deg, z=0.80):
    '''a grasp whose approach (TCP +x) is tilt_deg off straight down, TCP at height z'''
    g = Grasp()
    g.id = gid
    g.grasp_pose = PoseStamped()
    g.grasp_pose.header.frame_id = 'map'
    half = math.radians(90.0 - tilt_deg) / 2.0   # rotation about y: x -> (cos a, 0, -sin a)
    g.grasp_pose.pose.orientation.y, g.grasp_pose.pose.orientation.w = math.sin(half), math.cos(half)
    g.grasp_pose.pose.position.z = z
    return g


def object_pose():
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.orientation.w = 1.0
    return pose


class FakeScene:
    def get_objects(self, names):
        table = NS(header=NS(frame_id='map'), pose=Pose(), primitives=[NS(dimensions=[1.0, 1.0, TOP])],
                   primitive_poses=[Pose()])
        table.pose.position.z = TOP / 2.0
        return {'table_2': table}


def picker():
    p = pick.PickTools.__new__(pick.PickTools)
    p.scene = FakeScene()
    # one fingertip point 3 cm ahead of the TCP along the approach: 3 cm below it on a vertical grasp
    p.fingertip_envelope = lambda: [(0.0, np.array([[0.03, 0.0, 0.0]]))]
    return p


PARAMS = {'~open_set_min_grasp_height': 0.035, '~open_set_min_fingertip_height': 0.015}


class TestTopGraspsForLowObjects(unittest.TestCase):
    def setUp(self):
        self.params = dict(PARAMS)
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_low_object_keeps_only_near_vertical_grasps(self):
        grasps = [grasp('v', 0.0), grasp('t30', 30.0), grasp('t16', 16.0), grasp('t8', 8.0)]
        kept, widths, low = picker().top_grasps_for_low_objects(grasps, [0.01, 0.02, 0.03, 0.04], object_pose(),
                                                                list(STRAWBERRY))
        self.assertTrue(low)
        self.assertEqual([g.id for g in kept], ['v', 't8'])   # the real 16 deg strawberry grasp is dropped too
        self.assertEqual(widths, [0.01, 0.04])

    def test_other_objects_keep_every_grasp(self):
        grasps = [grasp('v', 0.0), grasp('t30', 30.0)]
        kept, widths, low = picker().top_grasps_for_low_objects(grasps, [0.01, 0.02], object_pose(), list(APPLE))
        self.assertFalse(low)
        self.assertEqual([g.id for g in kept], ['v', 't30'])

    def test_rule_off(self):
        self.params['~open_set_low_object_max_tilt_deg'] = 0.0
        kept, _, low = picker().top_grasps_for_low_objects([grasp('t30', 30.0)], [0.0], object_pose(),
                                                           list(STRAWBERRY))
        self.assertFalse(low)
        self.assertEqual(len(kept), 1)

    def test_no_top_grasp_left_means_the_fallback(self):
        kept, widths, low = picker().top_grasps_for_low_objects([grasp('t45', 45.0)], [0.0], object_pose(),
                                                                list(STRAWBERRY))
        self.assertTrue(low)
        self.assertEqual((kept, widths), ([], []))

    def test_sink_lowers_a_top_grasp_to_the_fingertip_clearance(self):
        g = grasp('v', 0.0, z=0.80)
        kept = picker().raise_low_grasps([g], 'table_2', [0.0], sink=True)
        self.assertEqual(len(kept), 1)
        # fingertips 3 cm below the TCP must stay 1.5 cm above the table: TCP at 0.72 + 0.015 + 0.03
        self.assertAlmostEqual(g.grasp_pose.pose.position.z, TOP + 0.045, places=4)

    def test_tcp_limit_binds_when_higher(self):
        self.params['~open_set_min_grasp_height'] = 0.06
        g = grasp('v', 0.0, z=0.80)
        picker().raise_low_grasps([g], 'table_2', [0.0], sink=True)
        self.assertAlmostEqual(g.grasp_pose.pose.position.z, TOP + 0.06, places=4)

    def test_sink_follows_the_approach_of_a_slightly_tilted_grasp(self):
        g = grasp('t8', 8.0, z=0.80)
        picker().raise_low_grasps([g], 'table_2', [0.0], sink=True)
        p = g.grasp_pose.pose.position
        fingertip = p.z - 0.03 * math.cos(math.radians(8.0))
        self.assertAlmostEqual(fingertip, TOP + 0.015, places=4)
        self.assertAlmostEqual(p.x / (0.80 - p.z), math.tan(math.radians(8.0)), places=4)   # moved along the approach

    def test_without_sink_a_high_grasp_stays(self):
        g = grasp('v', 0.0, z=0.80)
        picker().raise_low_grasps([g], 'table_2', [0.0])
        self.assertAlmostEqual(g.grasp_pose.pose.position.z, 0.80)

    def test_sink_switch_off(self):
        self.params['~open_set_low_object_sink'] = False
        g = grasp('v', 0.0, z=0.80)
        picker().raise_low_grasps([g], 'table_2', [0.0], sink=True)
        self.assertAlmostEqual(g.grasp_pose.pose.position.z, 0.80)

    def test_too_low_grasps_are_still_raised(self):
        g = grasp('v', 0.0, z=0.74)
        picker().raise_low_grasps([g], 'table_2', [0.0], sink=True)
        self.assertAlmostEqual(g.grasp_pose.pose.position.z, TOP + 0.045, places=4)

    def test_sink_by_fingertips_only_without_a_tcp_limit(self):
        self.params['~open_set_min_grasp_height'] = 0.0
        g = grasp('v', 0.0, z=0.80)
        picker().raise_low_grasps([g], 'table_2', [0.0], sink=True)
        self.assertAlmostEqual(g.grasp_pose.pose.position.z, TOP + 0.045, places=4)


if __name__ == '__main__':
    unittest.main()
