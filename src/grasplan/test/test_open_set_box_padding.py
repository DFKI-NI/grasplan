'''~open_set_box_padding: the open-set detector pads every side of the target box it reports by 1 cm, so an open
fingertip right beside the object is already inside the planning-scene box (sim 2026-10-27: a 9.2 x 10.8 x 4.8 cm
detection of a box of about 7 x 9 x 3 cm, every top grasp "eef in collision"). The flag takes the padding off every
side of the externally detected box, for the scene and the grasp geometry; off (0.0) the box stays as detected.
Mocks for the scene, the pose selector and rosparam (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from geometry_msgs.msg import PoseStamped

import grasplan.pick as pick
from grasplan.msg import DetectedObject
from grasplan.tools.common import objectToPick

DETECTED = [0.092, 0.108, 0.048]   # detector box, padded 1 cm per side
CENTER = (13.0, 2.08, 0.735)


class TestStripBoxPadding(unittest.TestCase):
    def setUp(self):
        self.params = {}
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_off_by_default(self):
        self.assertEqual(pick.strip_box_padding('box_1', DETECTED), DETECTED)

    def test_one_cm_off_every_side(self):
        self.params['~open_set_box_padding'] = 0.01
        for got, want in zip(pick.strip_box_padding('box_1', DETECTED), [0.072, 0.088, 0.028]):
            self.assertAlmostEqual(got, want)

    def test_a_side_never_vanishes(self):
        self.params['~open_set_box_padding'] = 0.01
        self.assertEqual(pick.strip_box_padding('thin_1', [0.015, 0.05, 0.02])[0], pick.MIN_STRIPPED_SIDE)

    def test_scene_and_returned_box_are_stripped_the_centre_stays(self):
        self.params['~open_set_box_padding'] = 0.01
        self.params['~open_set_complete_one_sided_boxes'] = False
        added = []
        p = pick.PickTools.__new__(pick.PickTools)
        p.scene = mock.Mock(add_box=lambda name, pose, size: added.append((name, pose, size)),
                            get_known_object_names=lambda: [])
        p.pose_selector_get_all_poses_srv = lambda: NS(poses=NS(objects=[]))
        p.clamp_object_boxes_to_support = False
        p.global_reference_frame = 'map'
        p.transform_pose = lambda pose, frame: pose
        p.robot = mock.Mock()
        target = DetectedObject(class_id='gelatin_box', instance_id=1)
        target.header.frame_id = 'map'
        target.pose.position.x, target.pose.position.y, target.pose.position.z = CENTER
        target.pose.orientation.w = 1.0
        target.size.x, target.size.y, target.size.z = DETECTED
        pose, box, _ = p.make_object_pose_and_add_objs_to_planning_scene(objectToPick('gelatin_box_1'),
                                                                         external_object=target)
        self.assertEqual([name for name, _, _ in added], ['gelatin_box_1'])
        for got, want in zip(added[0][2], [0.072, 0.088, 0.028]):
            self.assertAlmostEqual(got, want)
        self.assertEqual(list(box), list(added[0][2]))     # the grasp geometry gets the same box
        self.assertAlmostEqual(pose.pose.position.z, CENTER[2])


if __name__ == '__main__':
    unittest.main()
