'''The perceived boxes are compared with the support boxes in their own frame: the pose selector list's frame in the
place node as in the pick node. Supports only in another frame are reported once. Mocks rospy (no ROS master).'''
import unittest
from types import SimpleNamespace
from unittest import mock

import grasplan.place as place
from grasplan.tools.common import supports_in_frame

BOX = ((0.0, 0.0, 0.35), (0.0, 0.0, 0.0, 1.0), (1.0, 1.0, 0.7))


class TestSupportsInFrame(unittest.TestCase):
    def test_found(self):
        self.assertEqual(supports_in_frame({'map': [BOX]}, 'map'), [BOX])

    def test_empty_says_nothing(self):
        with mock.patch('rospy.logwarn_once') as warn:
            self.assertEqual(supports_in_frame({}, 'map'), [])
            self.assertEqual(supports_in_frame([], 'map'), [])
            self.assertEqual(supports_in_frame(None, 'map'), [])
        warn.assert_not_called()

    def test_another_frame_is_reported(self):
        with mock.patch('rospy.logwarn_once') as warn:
            self.assertEqual(supports_in_frame({'map': [BOX]}, 'base_footprint'), [])
        self.assertIn('base_footprint', warn.call_args[0][0])
        self.assertIn('map', warn.call_args[0][0])


class TestPlausibleBoxFrame(unittest.TestCase):
    def setUp(self):
        self.node = place.PlaceTools.__new__(place.PlaceTools)
        self.node.global_reference_frame = 'map'
        pose = SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=0.55),
                               orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
        self.sunk = SimpleNamespace(pose=pose, size=SimpleNamespace(x=0.1, y=0.1, z=0.1))

    def plausible(self, supports, frame=None):
        with mock.patch.object(place.rospy, 'get_param', lambda name, default=None: default), \
                mock.patch.object(place.rospy, 'logwarn'), mock.patch('rospy.logwarn_once'):
            return self.node.plausible_box('box_1', self.sunk, supports, [], frame)

    def test_the_list_frame_decides(self):
        self.assertFalse(self.plausible({'base_footprint': [BOX]}, 'base_footprint'))
        self.assertTrue(self.plausible({'map': [BOX]}, 'base_footprint'))

    def test_without_a_frame_the_reference_frame_as_before(self):
        self.assertFalse(self.plausible({'map': [BOX]}))


if __name__ == '__main__':
    unittest.main()
