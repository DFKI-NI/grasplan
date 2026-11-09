'''~planning_scene_box_padding grows the perceived boxes of the other objects on every side, in the pick and in the
place node, so MoveIt's joint-space moves keep clear of them; the pick target stays as perceived. Off (0.0) every box
is the one added before. Mocks for the scene and rosparam (no ROS master); needs the Mobipick image for the imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

import grasplan.pick as pick
import grasplan.place as place
from grasplan.tools.common import padded_box_size

SIZE = [0.1, 0.2, 0.3]


def stamped():
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.z = 1.0
    pose.pose.orientation.w = 1.0
    return pose


class TestPaddedBoxSize(unittest.TestCase):
    def test_off_returns_the_size_itself(self):
        self.assertIs(padded_box_size(SIZE, 0.0), SIZE)

    def test_every_side_grows_by_twice_the_padding_per_axis(self):
        for got, want in zip(padded_box_size(SIZE, 0.01), [0.12, 0.22, 0.32]):
            self.assertAlmostEqual(got, want)


class TestSceneBoxes(unittest.TestCase):
    def setUp(self):
        self.params = {}
        self.sizes = []
        for module in (pick, place):
            patcher = mock.patch.object(module.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
            patcher.start()
            self.addCleanup(patcher.stop)
        scene = mock.Mock(add_box=lambda name, pose, size: self.sizes.append(list(size)))
        self.pick = pick.PickTools.__new__(pick.PickTools)
        self.pick.scene = scene
        self.place = place.PlaceTools.__new__(place.PlaceTools)
        self.place.scene = scene
        self.place.global_reference_frame = 'map'
        box = NS(class_id='klt', instance_id=1, pose=Pose(position=Point(0, 0, 1), orientation=Quaternion(0, 0, 0, 1)),
                 size=NS(x=SIZE[0], y=SIZE[1], z=SIZE[2]))
        self.place.get_all_poses_pick_pose_selector_srv = lambda: NS(poses=NS(objects=[box]))

    def test_default_boxes_are_the_ones_added_before(self):
        self.pick.add_object_box('klt_1', stamped(), list(SIZE))
        self.place.add_objs_to_planning_scene()
        self.assertEqual(self.sizes, [SIZE, SIZE])

    def test_padding_grows_the_other_objects_boxes(self):
        self.params['~planning_scene_box_padding'] = 0.01
        self.pick.add_object_box('klt_1', stamped(), list(SIZE))
        self.place.add_objs_to_planning_scene()
        for got in self.sizes:
            for side, want in zip(got, [0.12, 0.22, 0.32]):
                self.assertAlmostEqual(side, want)

    def test_the_pick_target_stays_as_perceived(self):
        self.params['~planning_scene_box_padding'] = 0.01
        self.pick.add_object_box('target_1', stamped(), list(SIZE), keep_implausible=True)
        self.assertEqual(self.sizes, [SIZE])


if __name__ == '__main__':
    unittest.main()
