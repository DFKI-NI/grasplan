'''~clear_octomap is shared by place and insert: InsertTools calls PlaceTools.clear_octomap, which must exist and call
move_group's clear_octomap service (before, insert died with AttributeError when the switch was on). Stubbed service,
no ROS master; needs the Mobipick image for the module imports.'''
import inspect
import unittest
from unittest import mock

import grasplan.insert as insert
import grasplan.place as place


class TestClearOctomapSwitch(unittest.TestCase):
    def test_place_tools_clears_through_the_service(self):
        p = place.PlaceTools.__new__(place.PlaceTools)
        calls = []
        with mock.patch.object(place.rospy, 'ServiceProxy', lambda name, srv: lambda: calls.append(name)):
            p.clear_octomap('inserting')
        self.assertEqual(calls, ['clear_octomap'])

    def test_insert_calls_an_existing_method(self):
        source = inspect.getsource(insert.InsertTools)
        self.assertIn('self.place.clear_octomap(', source)
        self.assertTrue(callable(getattr(place.PlaceTools, 'clear_octomap', None)))


if __name__ == '__main__':
    unittest.main()
