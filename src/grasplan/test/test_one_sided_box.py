'''#151 one-sided boxes (real Pringles can 2026-09-28): the grasp view's box holds only the front of the object, so a
standing elongated box whose side along the line of sight is the shorter one gets its full depth behind the visible
face; flat, lying and wide boxes stay. Fake tf (no ROS master); needs the Mobipick image for the module imports.'''
import math
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped

import grasplan.pick as pick
from grasplan import grasp_scoring as gs

IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
PRINGLES_FRONT = (0.073, 0.036, 0.24)   # real K1 box, camera looking along +y


def yaw(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


class TestCompleteOneSidedBox(unittest.TestCase):
    def test_the_real_pringles_box_gets_its_back_half(self):
        size, centre = gs.complete_one_sided_box(PRINGLES_FRONT, IDENTITY, (1.0, 2.0, 0.84), (0.0, 1.0, -0.5))
        self.assertEqual([round(v, 3) for v in size], [0.073, 0.073, 0.24])
        self.assertAlmostEqual(centre[0], 1.0)
        self.assertAlmostEqual(centre[1], 2.0 + (0.073 - 0.036) / 2.0)   # away from the camera
        self.assertAlmostEqual(centre[2], 0.84)

    def test_camera_on_the_other_side_moves_the_other_way(self):
        _, centre = gs.complete_one_sided_box(PRINGLES_FRONT, IDENTITY, (1.0, 2.0, 0.84), (0.0, -1.0, 0.0))
        self.assertAlmostEqual(centre[1], 2.0 - 0.0185)

    def test_yawed_box(self):
        size, centre = gs.complete_one_sided_box((0.036, 0.073, 0.24), yaw(90.0), (0.0, 0.0, 0.5), (0.0, 1.0, 0.0))
        self.assertEqual([round(v, 3) for v in size], [0.073, 0.073, 0.24])   # its x axis points along the sight
        self.assertAlmostEqual(centre[1], 0.0185)

    def test_seen_along_its_wide_side_stays(self):
        self.assertIsNone(gs.complete_one_sided_box(PRINGLES_FRONT, IDENTITY, (0, 0, 0), (1.0, 0.0, 0.0)))

    def test_flat_lying_and_wide_boxes_stay(self):
        self.assertIsNone(gs.complete_one_sided_box((0.19, 0.045, 0.04), IDENTITY, (0, 0, 0), (0.0, 1.0, 0.0)))  # banana
        self.assertIsNone(gs.complete_one_sided_box((0.17, 0.03, 0.24), IDENTITY, (0, 0, 0), (0.0, 1.0, 0.0)))   # book
        self.assertIsNone(gs.complete_one_sided_box((0.076, 0.05, 0.064), IDENTITY, (0, 0, 0), (0.0, 1.0, 0.0)))  # apple

    def test_tilted_box_or_vertical_sight_stays(self):
        tilted = ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0))
        self.assertIsNone(gs.complete_one_sided_box(PRINGLES_FRONT, tilted, (0, 0, 0), (0.0, 1.0, 0.0)))
        self.assertIsNone(gs.complete_one_sided_box(PRINGLES_FRONT, IDENTITY, (0, 0, 0), (0.0, 0.0, -1.0)))


class FakeTf:
    def __init__(self, camera=(1.0, 1.5, 1.3), fail=False):
        self.camera, self.fail = camera, fail

    def waitForTransform(self, *args):
        if self.fail:
            raise pick.tf.Exception('no transform')

    def lookupTransform(self, target, source, stamp):
        return self.camera, (0.0, 0.0, 0.0, 1.0)


def box_pose():
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = 1.0, 2.0, 0.84
    pose.pose.orientation.w = 1.0
    return pose


class TestPickCompletesTheSceneBox(unittest.TestCase):
    def setUp(self):
        self.params = {}
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)

    def run_it(self, tf_listener):
        p = pick.PickTools.__new__(pick.PickTools)
        p.tf_listener = tf_listener
        pose, size = box_pose(), list(PRINGLES_FRONT)
        p.complete_one_sided_box('pringles_1', pose, size)
        return pose, size

    def test_box_and_pose_completed_in_place(self):
        pose, size = self.run_it(FakeTf())
        self.assertEqual([round(v, 3) for v in size], [0.073, 0.073, 0.24])
        self.assertAlmostEqual(pose.pose.position.y, 2.0185)

    def test_switch_off(self):
        self.params['~open_set_complete_one_sided_boxes'] = False
        pose, size = self.run_it(FakeTf())
        self.assertEqual(size, list(PRINGLES_FRONT))
        self.assertAlmostEqual(pose.pose.position.y, 2.0)

    def test_no_camera_transform_keeps_the_box(self):
        pose, size = self.run_it(FakeTf(fail=True))
        self.assertEqual(size, list(PRINGLES_FRONT))


if __name__ == '__main__':
    unittest.main()
