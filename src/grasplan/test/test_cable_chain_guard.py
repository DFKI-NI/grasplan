'''#121: the MTC cable guard also rejects a plan the sim monitor's disc chain would stop (the chain caught on the tool
while the geometry stays slack, as in the two 2026-09-28 night sim stops). Fake MTC solution messages, no ROS master;
needs the Mobipick image and mobipick_cable_entanglement.'''
import math
import os
import unittest
from types import SimpleNamespace as NS

import rospy
import yaml
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from grasplan.mtc_pick_place import CableGuard, MtcPickPlace

J = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
NAMES = [f'mobipick/ur5_{j}_joint' for j in J]
PLACE_END = [-90.4, -131.3, 140.3, -148.3, -91.0, 83.5]
VIA = [-90.0, -100.0, 120.0, -100.0, -90.0, 180.0]
ANYGRASP = [-90, -90, 114.6, -43, 90, 270]
HOME = [0.0, -149.0, 139.2, -117.5, -90.0, 179.9]   # SRDF home
# the committed disc chain (2026-10-01) catches on HOME -> PLACE_END (+0.18, geometry -0.24) and stays slack on
# HOME -> ANYGRASP; the radius rope's catch PLACE_END -> VIA -> ANYGRASP (removed 2026-09-30) is slack for the chain
CATCH = (HOME, PLACE_END)
CLEAN = (HOME, ANYGRASP)


def solution(*waypoints, seconds=5.0):
    '''an MTC solution message: one arm sub trajectory through the waypoints (degrees), plus an empty scene stage'''
    trajectory = JointTrajectory(joint_names=NAMES)
    for i, w in enumerate(waypoints):
        trajectory.points.append(JointTrajectoryPoint(positions=[math.radians(v) for v in w],
                                                      time_from_start=rospy.Duration(seconds * i)))
    empty = NS(trajectory=NS(joint_trajectory=JointTrajectory(joint_names=NAMES)))
    return NS(sub_trajectory=[empty, NS(trajectory=NS(joint_trajectory=trajectory))])


class TestChainGuard(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from mobipick_cable_entanglement.cable_model import CableModel, config_from_dict
        import copy
        import rospkg
        pkg = rospkg.RosPack().get_path('mobipick_cable_entanglement')
        with open(os.path.join(pkg, 'config', 'cable_model.yaml')) as f:
            cfg = config_from_dict(yaml.safe_load(f))
        with open(os.path.join(pkg, 'test', 'mobipick_sim_robot_description.urdf')) as f:
            urdf = f.read()
        guard = CableGuard.__new__(CableGuard)              # without rospy params / robot_description
        guard.cfg, guard.model = cfg, CableModel(urdf, cfg)
        chain_cfg = copy.deepcopy(cfg)
        chain_cfg.rope.enabled = True
        guard.chain_model = CableModel(urdf, chain_cfg)
        guard.wrist_3_neutral, guard.half_window = math.radians(165.7), math.radians(170.0)
        guard.wrist_3_joint = 'mobipick/ur5_wrist_3_joint'
        guard.max_stretch, guard.max_points, guard.chain_rate = -0.115, 40, 10.0
        cls.guard = guard

    def check(self, msg, chain):
        self.guard.chain_check = chain
        return self.guard.check(msg)

    def test_chain_catch_is_rejected_although_the_geometry_passes(self):
        msg = solution(*CATCH)
        ok, stretch, span, _ = self.check(msg, chain=False)
        self.assertTrue(ok)                                 # geometry only: passes (the night's situation)
        ok, stretch, span, q = self.check(msg, chain=True)
        self.assertFalse(ok)
        self.assertIn('chain replay', span)
        self.assertGreater(stretch, 0.02)

    def test_worst_point_is_located_in_the_task(self):
        self.check(solution(PLACE_END, VIA, ANYGRASP), chain=False)
        sub_index, point_index, points = self.guard.worst_where
        self.assertEqual((sub_index, points), (1, 3))            # the arm sub trajectory, not the empty scene stage

        class Leaf:
            def __init__(self, name):
                self.name = name

            def __getitem__(self, i):
                raise IndexError

        class Container(Leaf):
            def __init__(self, name, children):
                super().__init__(name)
                self.children = children

            def __getitem__(self, i):
                return self.children[i]

        task = Container('task', [Leaf('current state'), Container('grasp', [Leaf('approach')])])
        text = MtcPickPlace.where_in_task(task, self.guard)
        self.assertIn('grasp/approach', text)
        self.assertIn(f'point {point_index + 1}/3', text)

    def test_a_clean_path_still_passes(self):
        ok, _, _, _ = self.check(solution(*CLEAN), chain=True)
        self.assertTrue(ok)


if __name__ == '__main__':
    unittest.main()
