'''Careful flag ~grasp_standing_tilt_band (off in code): a standing elongated object taller than 5 cm gets no grasp
whose approach lies between 25 and 65 deg off vertical (the tilted side grasps pinch the rim of a Pringles can and it tips
at the place); the unfiltered grasps stay when none is left. Values from a sim table of 31 picks. Fake picker (no ROS
master); needs the Mobipick image for the module imports.'''
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import grasplan.pick as pick
from grasplan import grasp_scoring as gs

CAN = (0.075, 0.075, 0.235)           # a standing can, box z up
APPLE = (0.076, 0.077, 0.064)
BANANA = (0.19, 0.045, 0.04)          # lying
LYING_CAN = ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0))   # box y axis vertical: the long z axis is horizontal
TABLE_TILTS = (0, 20, 25, 46, 63, 65, 90)


def grasp(gid, tilt_deg):
    '''a grasp whose approach (TCP +x) is tilt_deg off straight down'''
    g = Grasp()
    g.id = gid
    g.grasp_pose = PoseStamped()
    g.grasp_pose.header.frame_id = 'map'
    half = math.radians(90.0 - tilt_deg) / 2.0
    g.grasp_pose.pose.orientation.y, g.grasp_pose.pose.orientation.w = math.sin(half), math.cos(half)
    g.pre_grasp_posture = JointTrajectory(points=[JointTrajectoryPoint(positions=[0.082])])
    return g


def object_pose():
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.orientation.w = 1.0
    return pose


class TestShapeRule(unittest.TestCase):
    def test_a_standing_can_is_standing_elongated(self):
        self.assertTrue(gs.is_standing_elongated(CAN))

    def test_other_shapes_are_not(self):
        self.assertFalse(gs.is_standing_elongated(APPLE))                 # compact round
        self.assertFalse(gs.is_standing_elongated(BANANA))                # lying and low
        self.assertFalse(gs.is_standing_elongated(CAN, LYING_CAN))        # a can on its side
        self.assertFalse(gs.is_standing_elongated((0.0, 0.0, 0.0)))

    def test_a_tall_enough_thin_object_standing_counts_only_above_the_low_height(self):
        self.assertTrue(gs.is_standing_elongated((0.03, 0.03, 0.08)))
        self.assertFalse(gs.is_standing_elongated((0.03, 0.03, 0.08), min_height=0.09))

    def test_the_band_edges_are_open(self):
        self.assertEqual([gs.in_tilt_band(t) for t in TABLE_TILTS], [False, False, False, True, True, False, False])
        self.assertTrue(gs.in_tilt_band(30.0, 20.0, 40.0))


class TestBandFilter(unittest.TestCase):
    def setUp(self):
        self.params = {'~grasp_standing_tilt_band': True}
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.p = pick.PickTools.__new__(pick.PickTools)

    def grasps(self):
        return [grasp(f't{t}', t) for t in TABLE_TILTS]

    def test_the_tilted_grasps_of_a_standing_can_are_dropped(self):
        kept, widths = self.p.standing_tilt_band_grasps(self.grasps(), list(range(7)), object_pose(), list(CAN))
        self.assertEqual([g.id for g in kept], ['t0', 't20', 't25', 't65', 't90'])
        self.assertEqual(widths, [0, 1, 2, 5, 6])

    def test_off_in_code(self):
        self.params = {}
        kept, _ = self.p.standing_tilt_band_grasps(self.grasps(), list(range(7)), object_pose(), list(CAN))
        self.assertEqual(len(kept), 7)

    def test_other_objects_keep_every_grasp(self):
        for box in (APPLE, BANANA):
            kept, _ = self.p.standing_tilt_band_grasps(self.grasps(), list(range(7)), object_pose(), list(box))
            self.assertEqual(len(kept), 7, box)

    def test_when_every_grasp_is_in_the_band_the_unfiltered_set_stays(self):
        tilted = [grasp('a', 46), grasp('b', 63)]
        with mock.patch.object(pick.rospy, 'logwarn') as warn:
            kept, widths = self.p.standing_tilt_band_grasps(tilted, [1, 2], object_pose(), list(CAN))
        self.assertEqual(([g.id for g in kept], widths), (['a', 'b'], [1, 2]))
        self.assertIn('fallback', warn.call_args[0][0])

    def test_the_log_names_every_dropped_grasp_with_its_tilt(self):
        with mock.patch.object(pick.rospy, 'loginfo') as info:
            self.p.standing_tilt_band_grasps(self.grasps(), list(range(7)), object_pose(), list(CAN))
        self.assertIn('dropped (deg off vertical): t46 46, t63 63', info.call_args[0][0])

    def test_a_silent_return_is_logged_with_height_and_elongation(self):
        with mock.patch.object(pick.rospy, 'loginfo') as info:
            self.p.standing_tilt_band_grasps(self.grasps(), list(range(7)), object_pose(), list(APPLE))
        line = info.call_args[0][0]
        self.assertIn('not standing elongated', line)
        self.assertIn('height 0.064 m', line)
        self.assertIn('elongation 1.01', line)
        self.assertIn('all 7 grasps kept', line)

    def test_thresholds_are_parameters(self):
        self.params.update({'~grasp_standing_tilt_band_min_deg': 40.0, '~grasp_standing_tilt_band_max_deg': 50.0})
        kept, _ = self.p.standing_tilt_band_grasps(self.grasps(), list(range(7)), object_pose(), list(CAN))
        self.assertEqual([g.id for g in kept], ['t0', 't20', 't25', 't63', 't65', 't90'])

    def test_the_cable_ranking_log_carries_tilt_and_gap(self):
        self.assertEqual(self.p.tilt_and_gap(grasp('x', 46)), 'tilt 46deg gap 82mm')
        self.assertEqual(self.p.tilt_and_gap(NS()), 'tilt ? gap ?')


if __name__ == '__main__':
    unittest.main()
