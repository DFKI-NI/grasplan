'''The planning-scene boxes of the pick and the place node are stamped with the frame the pose selector answers in (the
ObjectList header), falling back to ~global_reference_frame when the list carries none. Mobipick answers in the
reference frame, so its messages stay the same. Mocks (no ROS master); needs the Mobipick image for the imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from geometry_msgs.msg import Point, Pose, Quaternion

import grasplan.place as place
from grasplan.tools.common import object_list_frame

BOX = NS(class_id='klt', instance_id=1, pose=Pose(position=Point(0.0, 0.0, 0.7), orientation=Quaternion(0, 0, 0, 1)),
         size=NS(x=0.3, y=0.2, z=0.15))


class TestObjectListFrame(unittest.TestCase):
    def test_header_frame_wins(self):
        self.assertEqual(object_list_frame(NS(header=NS(frame_id='base_footprint')), 'map'), 'base_footprint')

    def test_empty_or_missing_header_falls_back(self):
        self.assertEqual(object_list_frame(NS(header=NS(frame_id='')), 'map'), 'map')
        self.assertEqual(object_list_frame(NS(objects=[]), 'map'), 'map')


class TestPlaceSceneFrame(unittest.TestCase):
    def frame_of_added_box(self, list_frame):
        added = []
        p = place.PlaceTools.__new__(place.PlaceTools)
        p.global_reference_frame = 'map'
        p.scene = mock.Mock(add_box=lambda name, pose, size: added.append(pose.header.frame_id))
        p.get_all_poses_pick_pose_selector_srv = lambda: NS(poses=NS(header=NS(frame_id=list_frame), objects=[BOX]))
        with mock.patch.object(place.rospy, 'get_param', lambda name, default=None: default):
            p.add_objs_to_planning_scene()
        return added

    def test_default_message_unchanged_for_the_reference_frame(self):
        self.assertEqual(self.frame_of_added_box('map'), ['map'])
        self.assertEqual(self.frame_of_added_box(''), ['map'])

    def test_list_frame_is_used(self):
        self.assertEqual(self.frame_of_added_box('base_footprint'), ['base_footprint'])


if __name__ == '__main__':
    unittest.main()
