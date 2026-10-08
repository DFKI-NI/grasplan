'''#121: a grasp rejected by the MTC cable guard is replanned turned about its approach axis (the wrist_3 axis).
Needs the Mobipick image (MoveIt Python bindings) and mobipick_cable_entanglement sourced.'''
import math
import os
import unittest

import numpy as np
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Grasp

from grasplan.grasp_scoring import is_compact_round
from grasplan.mtc_pick_place import has_twin, pose_to_matrix, turn_grasp_about_approach


class TestRoundRule(unittest.TestCase):
    def test_round_and_not_round(self):
        self.assertTrue(is_compact_round([0.066, 0.066, 0.066]))      # tennis ball
        self.assertTrue(is_compact_round([0.079, 0.08, 0.075]))       # apple
        self.assertFalse(is_compact_round([0.11, 0.079, 0.07]))       # apple box with a shadow: not round enough
        self.assertFalse(is_compact_round([0.19, 0.04, 0.04]))        # banana
        self.assertFalse(is_compact_round([0.0, 0.05, 0.05]))


class TestTurnGrasp(unittest.TestCase):
    def test_turn_is_about_the_own_x_axis(self):
        grasp = Grasp(id='anygrasp_3')
        grasp.grasp_pose.pose = Pose()
        grasp.grasp_pose.pose.position.x, grasp.grasp_pose.pose.position.z = 0.5, 0.8
        # a top-down grasp: tcp x (approach) points down
        q = [0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)]
        grasp.grasp_pose.pose.orientation.x, grasp.grasp_pose.pose.orientation.y = q[0], q[1]
        grasp.grasp_pose.pose.orientation.z, grasp.grasp_pose.pose.orientation.w = q[2], q[3]
        turned = turn_grasp_about_approach(grasp, 60.0)
        a, b = pose_to_matrix(grasp.grasp_pose.pose), pose_to_matrix(turned.grasp_pose.pose)
        np.testing.assert_allclose(a[:3, 3], b[:3, 3], atol=1e-9)          # same point
        np.testing.assert_allclose(a[:3, 0], b[:3, 0], atol=1e-9)          # same approach axis
        self.assertAlmostEqual(math.degrees(math.acos(np.clip(a[:3, 1].dot(b[:3, 1]), -1, 1))), 60.0, places=6)
        self.assertEqual(turned.id, 'anygrasp_3_turned+60')
        self.assertEqual(grasp.id, 'anygrasp_3')                           # the original is untouched

    def test_twin_already_offered(self):
        grasp = Grasp(id='anygrasp_4')
        grasp.grasp_pose.header.frame_id = 'map'
        grasp.grasp_pose.pose.orientation.w = 1.0
        twin = turn_grasp_about_approach(grasp, 180.0)
        self.assertTrue(has_twin([grasp, twin], grasp, 180.0))
        self.assertFalse(has_twin([grasp], grasp, 180.0))
        self.assertFalse(has_twin([grasp, turn_grasp_about_approach(grasp, 60.0)], grasp, 180.0))


class TestGuardTurn(unittest.TestCase):
    '''CableGuard.turn_for_slack on a joint state the guard rejected in the sim (tennis ball topdown_0, -0.112)'''

    def setUp(self):
        from mobipick_cable_entanglement.cable_model import CableModel, config_from_dict
        import rospkg
        from grasplan.mtc_pick_place import CableGuard
        pkg = rospkg.RosPack().get_path('mobipick_cable_entanglement')
        guard = CableGuard.__new__(CableGuard)       # without rospy params
        with open(os.path.join(pkg, 'config', 'cable_model.yaml')) as f:
            guard.cfg = config_from_dict(yaml.safe_load(f))
        with open(os.path.join(pkg, 'test', 'mobipick_sim_robot_description.urdf')) as f:
            guard.model = CableModel(f.read(), guard.cfg)
        guard.wrist_3_neutral, guard.half_window, guard.max_stretch = math.radians(165.7), math.radians(170.0), -0.115
        self.guard = guard
        deg = [-93.8, -79.6, 104.4, -61.4, 44.6, 243.5]
        self.q = {f'mobipick/ur5_{j}_joint': math.radians(v) for j, v in
                  zip(['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3'], deg)}

    def test_round_object_gets_a_turn_that_passes(self):
        turn, stretch = self.guard.turn_for_slack(self.q, round_object=True)
        self.assertGreater(turn, 0)
        self.assertLess(stretch, -0.115)

    def test_other_objects_only_flip_which_does_not_help_here(self):
        self.assertIsNone(self.guard.turn_for_slack(self.q, round_object=False))


if __name__ == '__main__':
    unittest.main()
