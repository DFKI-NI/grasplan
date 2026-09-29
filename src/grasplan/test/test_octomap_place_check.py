'''#185 (real 2026-09-28 16:48 and 18:23: a held klt placed into objects only the octomap knew, protective stops): a place
location whose footprint holds occupied octomap voxels higher than ~mtc_place_octomap_clearance above the held object's
bottom is skipped (~mtc_place_reject_octomap_occupied, default on in the sim, off on the real robot until tested). Fake
octomap and scene (no ROS master); needs the Mobipick image for the module imports.'''
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, PlaceGoal, PlaceLocation
from shape_msgs.msg import SolidPrimitive

import grasplan.mtc_pick_place as mpp
import grasplan.tools.octomap_probe as probe

RES = 0.025
TOP = 0.72
KLT = (0.297, 0.197, 0.147)


def octomap_msg(points, binary=False):
    data = probe.encode_full_octree([probe.key_of(*p, RES) for p in points], RES)
    msg = NS(header=NS(frame_id='map'), origin=Pose(), octomap=NS(binary=binary, resolution=RES,
                                                                   data=[b if b < 128 else b - 256 for b in data]))
    msg.origin.orientation.w = 1.0
    return msg


def attached_klt():
    box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(KLT))
    pose = Pose()
    pose.orientation.w = 1.0
    return NS(object=NS(id='klt_1', primitives=[box], primitive_poses=[pose]))


def location(lid, x, y, yaw=0.0):
    loc = PlaceLocation(id=str(lid))
    loc.place_pose = PoseStamped()
    loc.place_pose.header.frame_id = 'map'
    loc.place_pose.pose.position.x, loc.place_pose.pose.position.y = x, y
    loc.place_pose.pose.position.z = TOP + KLT[2] / 2.0 + 0.01   # bottom 1 cm above the table
    loc.place_pose.pose.orientation.z, loc.place_pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return loc


def planner(points, params=None):
    params = {'/use_sim_time': True, **(params or {})}   # the sim default: the check is on
    m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
    scene = NS(world=NS(octomap=octomap_msg(points)), robot_state=NS(attached_collision_objects=[attached_klt()]))
    m.get_planning_scene_srv = lambda request: NS(scene=scene)
    m.tf_buffer = mock.Mock()
    patcher = mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: params.get(name, default))
    return m, patcher


# a can standing on the table at (18.40, 15.20): voxels from 3 to 11 cm above the table top
CAN = [(18.40 + dx, 15.20 + dy, TOP + 0.03 + 0.025 * k) for dx in (-0.02, 0.02) for dy in (-0.02, 0.02) for k in range(4)]
# a flat thing 1 cm high: voxels only right at the table top
FLAT = [(18.80, 15.20, TOP + 0.012)]


class TestProbe(unittest.TestCase):
    def test_round_trip_and_free_nodes(self):
        data = probe.encode_full_octree([probe.key_of(0.11, 0.21, 0.31, RES)], RES)
        leaves = probe.occupied_leaves(data, RES)
        self.assertEqual(len(leaves), 1)
        np.testing.assert_allclose(leaves[0], [0.1125, 0.2125, 0.3125, RES])
        free = probe.encode_full_octree([probe.key_of(0.11, 0.21, 0.31, RES)], RES, value=-2.0)
        self.assertEqual(len(probe.occupied_leaves(free, RES)), 0)

    def test_rotated_box(self):
        leaves = np.array([[0.14, 0.0, 0.0, RES]])
        box = np.eye(4)
        self.assertEqual(probe.count_in_box(leaves, box, [0.15, 0.05, 0.05]), 1)
        box[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]   # yaw 90 deg: the long side along y
        self.assertEqual(probe.count_in_box(leaves, box, [0.15, 0.05, 0.05]), 0)


class TestPlaceCheck(unittest.TestCase):
    def test_can_in_the_footprint_blocks_flat_and_empty_do_not(self):
        m, patcher = planner(CAN + FLAT)
        with patcher:
            check = m.octomap_place_check('klt_1')
            self.assertIsNotNone(check)
            self.assertGreater(m.octomap_blocks_place(check, location(41, 18.45, 15.25)), 0)   # the can
            self.assertEqual(m.octomap_blocks_place(check, location(36, 18.80, 15.20)), 0)     # only the flat thing
            self.assertEqual(m.octomap_blocks_place(check, location(5, 18.30, 14.70)), 0)      # nothing there

    def test_margin_catches_a_spot_right_beside_the_can(self):
        # the klt placed with about 3 cm of gap next to the can (sim 2026-09-29: such a place brushed and pushed it)
        m, patcher = planner(CAN)
        with patcher:
            check = m.octomap_place_check('klt_1')
            self.assertGreater(m.octomap_blocks_place(check, location(7, 18.60, 15.20)), 0)
        m, patcher = planner(CAN, {'~mtc_place_octomap_margin': 0.0})
        with patcher:
            check = m.octomap_place_check('klt_1')
            self.assertEqual(m.octomap_blocks_place(check, location(7, 18.60, 15.20)), 0)   # the final footprint alone

    def test_switch_off_and_binary_map(self):
        m, patcher = planner(CAN, {'~mtc_place_reject_octomap_occupied': False})
        with patcher:
            self.assertIsNone(m.octomap_place_check('klt_1'))
        m, patcher = planner(CAN, {'/use_sim_time': False})   # the real robot default: off until tested there
        with patcher:
            self.assertIsNone(m.octomap_place_check('klt_1'))
        m, patcher = planner(CAN)
        m.get_planning_scene_srv = lambda request: NS(scene=NS(world=NS(octomap=octomap_msg(CAN, binary=True)),
                                                               robot_state=NS(attached_collision_objects=[attached_klt()])))
        with patcher:
            self.assertIsNone(m.octomap_place_check('klt_1'))

    def test_place_skips_blocked_locations_before_planning(self):
        m, patcher = planner(CAN)
        planned = []
        m.update_cable_guard = lambda: None
        m.gripper_holds_object = lambda *a, **k: True
        m.eef_to_attached_object = lambda name: np.eye(4)
        m.relaxed_retreat_min_distance = 0.05
        m.is_preempt_requested = lambda: False
        m.retreat_failed = False
        m.make_place_task = lambda goal, loc, *a: (planned.append(loc.id) or 'task', [])
        m.plan = lambda task, what: False
        m.only_retreat_failed = lambda task: False
        goal = PlaceGoal(attached_object_name='klt_1', support_surface_name='table_1',
                         place_locations=[location(41, 18.45, 15.25), location(5, 18.30, 14.70)])
        with patcher:
            result = m.place(goal)
        self.assertEqual(planned, ['5'])
        self.assertEqual(result.error_code.val, MoveItErrorCodes.PLANNING_FAILED)
        self.assertIn('1 skipped: objects only the octomap sees', m.failure_reason)


if __name__ == '__main__':
    unittest.main()
