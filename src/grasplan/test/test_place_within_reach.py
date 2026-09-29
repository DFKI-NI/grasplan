'''#217 (sim 2026-09-29 07:26, a banana place on table_3 failed 96 of 96): the place poses were drawn over the whole table
(shrunk by 15 cm), 228 of 300 then dropped as beyond ~place_max_reach, and the rest lay on the drill. With
~place_sample_within_reach the poses are drawn inside the reach disk, and with ~place_free_footprints_first the ones whose
footprint clears the perceived object boxes are tried first (both sim on, real off). Geometry of the cic table 3 stop.
Mocks (no ROS master); needs the Mobipick image for the module imports.'''
import math
import random
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from geometry_msgs.msg import Point, Pose, Quaternion
from object_pose_msgs.msg import ObjectList, ObjectPose
from shape_msgs.msg import SolidPrimitive

import grasplan.place as place
from grasplan.tools.place_reach import footprint_is_free, order_free_first
from grasplan.tools.support_plane_tools import sample_in_plane

I = (0.0, 0.0, 0.0, 1.0)
# table_3 (0.8 x 0.8 m at 21.165, 13.84, top 0.72) shrunk by 15 cm per side, the arm base (ur5_base_link) at the table 3
# stop base_table_3_pose (21.01, 14.77) + 0.038 m forward
PLANE = [Point(20.915, 13.59, 0.72), Point(21.415, 13.59, 0.72), Point(21.415, 14.09, 0.72), Point(20.915, 14.09, 0.72)]
ARM = (21.048, 14.77)
REACH = 0.8
KLT = ((21.25, 13.99, 0.7935), I, (0.297, 0.197, 0.147))
DRILL = ((20.971, 14.06, 0.815), I, (0.20, 0.07, 0.19))
SCREWDRIVER = ((21.30, 13.70, 0.737), I, (0.245, 0.034, 0.034))
BANANA = (0.19, 0.05, 0.047)
PLACE_Z = 0.72 + 0.02 + BANANA[2] / 2.0


def within_reach(x, y):
    return math.hypot(x - ARM[0], y - ARM[1]) <= REACH


class TestSampling(unittest.TestCase):
    def test_table_3_draws(self):
        random.seed(3)
        old = [sample_in_plane(PLANE) for _ in range(300)]
        new = [sample_in_plane(PLANE, (ARM[0], ARM[1], REACH)) for _ in range(300)]
        share_old = sum(within_reach(x, y) for x, y in old) / len(old)
        self.assertLess(share_old, 0.35)                           # the log: 72 of 300 kept
        self.assertTrue(all(within_reach(x, y) for x, y in new))
        self.assertTrue(all(PLANE[0].x <= x <= PLANE[1].x and PLANE[0].y <= y <= PLANE[3].y for x, y in new))

    def test_disk_missing_the_plane_falls_back(self):
        x, y = sample_in_plane(PLANE, (30.0, 30.0, 0.5), tries=20)
        self.assertTrue(PLANE[0].x <= x <= PLANE[1].x and PLANE[0].y <= y <= PLANE[3].y)


def candidate(x, y, yaw=0.0):
    q = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    return ObjectPose(class_id='banana', pose=Pose(position=Point(x, y, PLACE_Z), orientation=Quaternion(*q)))


class TestFreeFirst(unittest.TestCase):
    def test_footprint_checks(self):
        boxes = [KLT, DRILL, SCREWDRIVER]
        self.assertFalse(footprint_is_free((20.98, 14.05, PLACE_Z), I, BANANA, boxes))     # on the drill
        self.assertFalse(footprint_is_free((21.25, 14.02, PLACE_Z), I, BANANA, boxes))     # in the klt
        self.assertTrue(footprint_is_free((21.00, 13.80, PLACE_Z), I, BANANA, boxes))      # free table
        self.assertTrue(footprint_is_free((20.98, 14.05, 1.20), I, BANANA, boxes))         # far above everything

    def test_order_keeps_everything_free_ones_first(self):
        cands = [candidate(20.98, 14.05), candidate(21.25, 14.02), candidate(21.00, 13.80), candidate(21.12, 14.08, 1.57)]
        ordered, free = order_free_first(cands, BANANA, [KLT, DRILL, SCREWDRIVER])
        self.assertEqual(len(ordered), 4)
        self.assertEqual(free, 1)                              # the turned one at 21.12 still reaches into the klt
        self.assertEqual((round(ordered[0].pose.position.x, 2), round(ordered[0].pose.position.y, 2)), (21.00, 13.80))
        self.assertEqual([c.pose.position.x for c in ordered[1:]], [20.98, 21.25, 21.12])   # the rest in their order

    def test_table_3_stop_no_free_spot_within_0_8_but_within_0_9(self):
        # the finding behind #217: from the table 3 stop the reachable band of the shrunk plane (y >= 13.97) is taken by
        # the drill (x 20.87-21.07) and the klt (x 21.10-21.40); the gap between them is 3 cm. Drawing within reach and
        # trying free spots first makes the place fail fast there instead of planning 96 poses, it cannot make it work
        random.seed(5)

        def free(reach):
            spots = [sample_in_plane(PLANE, (ARM[0], ARM[1], reach)) for _ in range(300)]
            return [s for s in spots for yaw in (0.0, math.pi / 2.0)
                    if footprint_is_free((s[0], s[1], PLACE_Z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)),
                                         BANANA, [KLT, DRILL, SCREWDRIVER])]
        self.assertEqual(free(REACH), [])
        # the fallback reach 0.9 m (~place_max_reach_fallback) finds free spots behind the drill / klt band
        self.assertGreater(len(free(0.9)), 3)


class TestPlaceNode(unittest.TestCase):
    def setUp(self):
        self.params = {'/use_sim_time': True}
        patcher = mock.patch.object(place.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        p = place.PlaceTools.__new__(place.PlaceTools)
        p.tf_buffer = mock.Mock(lookup_transform=lambda *a, **k: NS(transform=NS(translation=NS(x=-0.117, y=0.93, z=0.4))))

        def box(center, size):
            return NS(header=NS(frame_id='map'), pose=Pose(position=Point(*center), orientation=Quaternion(*I)),
                      primitives=[NS(type=SolidPrimitive.BOX, dimensions=list(size))],
                      primitive_poses=[Pose(orientation=Quaternion(0.0, 0.0, 0.0, 0.0))])
        objects = {'table_3': box((21.165, 13.84, 0.36), (0.8, 0.8, 0.72)), 'klt_1': box(KLT[0], KLT[2]),
                   'power_drill_with_grip_1': box(DRILL[0], DRILL[2])}
        held = NS(object=NS(primitives=[NS(dimensions=list(BANANA))]))
        p.scene = mock.Mock(get_objects=lambda: objects,
                            get_attached_objects=lambda names: {'banana_1': held})
        self.p = p

    def test_reach_disk_in_the_support_frame(self):
        self.assertEqual(self.p.reach_disk('table_3'), (-0.117, 0.93, 0.8))
        self.params['/use_sim_time'] = False
        self.assertIsNone(self.p.reach_disk('table_3'))   # real robot default: as before

    def test_reach_radii(self):
        self.assertEqual(self.p.place_reach_radii(), [0.8, 0.9])   # the fallback when 0.8 holds no free spot
        self.assertEqual(self.p.reach_disk('table_3', 0.9), (-0.117, 0.93, 0.9))
        self.params['/use_sim_time'] = False
        self.assertEqual(self.p.place_reach_radii(), [0.8])        # real robot: the old single draw

    def test_free_first_in_place(self):
        poses = ObjectList(objects=[candidate(20.98, 14.05), candidate(21.00, 13.80)])
        poses.header.frame_id = 'map'
        self.p.order_free_footprints_first(poses, 'banana_1', 'table_3')
        self.assertEqual(round(poses.objects[0].pose.position.y, 2), 13.80)
        self.params['/use_sim_time'] = False
        poses = ObjectList(objects=[candidate(20.98, 14.05), candidate(21.00, 13.80)])
        poses.header.frame_id = 'map'
        self.p.order_free_footprints_first(poses, 'banana_1', 'table_3')
        self.assertEqual(round(poses.objects[0].pose.position.y, 2), 14.05)   # unchanged on the real robot


if __name__ == '__main__':
    unittest.main()
