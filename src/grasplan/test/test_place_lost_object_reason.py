'''A place whose object is gone says why the attempt before failed and whether the arm is stopped, not only "no object
is attached" (real robot 2026-11-03: a protective stop during the first attempt lost the object, the second attempt
answered only that nothing was attached, and the agent had to guess the stop from a later snapshot).
Mocks (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from moveit_msgs.msg import MoveItErrorCodes

import grasplan.place as place

STOP = '; the arm is in a protective stop (UR safety mode PROTECTIVE_STOP)'
GONE = ('cannot place on table_2: no object is attached to the gripper in the planning scene, so the place has nothing '
        'to plan with (the object may have been dropped, or the pick never attached it)')


class PlaceWithTheObjectGone(unittest.TestCase):
    def setUp(self):
        for name in ('logerr', 'loginfo'):
            patcher = mock.patch.object(place.rospy, name)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.stop = mock.patch.object(place, 'arm_stop_text', return_value='')
        self.arm_stop_text = self.stop.start()
        self.addCleanup(self.stop.stop)
        self.pl = place.PlaceTools.__new__(place.PlaceTools)
        self.pl.mtc = None
        self.pl.place_failure_reason = ''
        self.pl.scene = NS(get_attached_objects=lambda: {})

    def attempt(self):
        self.assertFalse(self.pl.place_object('table_2'))
        return self.pl.failure_text()

    def test_first_attempt_says_only_that_nothing_is_attached(self):
        self.assertEqual(self.attempt(), GONE)

    def test_the_earlier_attempt_reason_is_kept(self):
        self.pl.place_failure_reason = ('the place failed: CONTROL_FAILED (-4): the motion was aborted while executing'
                                        + STOP)
        text = self.attempt()
        self.assertEqual(text, GONE + '; the attempt before this one failed: the place failed: CONTROL_FAILED (-4): '
                                      'the motion was aborted while executing' + STOP)
        self.arm_stop_text.assert_not_called()   # the stop is named once, by the earlier reason

    def test_an_mtc_reason_of_the_earlier_attempt_is_kept(self):
        self.pl.mtc = NS(failure_reason='the arm stopped while placing')
        self.assertTrue(self.attempt().endswith('; the attempt before this one failed: the arm stopped while placing'))

    def test_a_stopped_arm_is_named(self):
        self.arm_stop_text.return_value = STOP
        self.assertEqual(self.attempt(), GONE + STOP)

    def test_a_third_attempt_does_not_nest_sentences(self):
        self.pl.place_failure_reason = 'the place failed: PLANNING_FAILED (-1): no motion plan was found'
        second = self.attempt()
        third = self.attempt()
        self.assertEqual(third, second)
        self.assertEqual(third.count('no object is attached'), 1)


if __name__ == '__main__':
    unittest.main()
