'''#151 real 2026-09-28: a Pringles can slipped out before a place and the failed place left the gripper closed; every
later pick then failed at "grasp pose: eef in collision: <object> - gripper_left_inner_finger" (the closed fingers sit
inside the target box at every grasp pose). The place now opens the empty gripper (and pick.py opens it before planning,
see test_open_set_view_and_start). Fake MTC (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from unittest import mock

from moveit_msgs.msg import MoveItErrorCodes, PlaceGoal, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint

import grasplan.mtc_pick_place as mpp


def mtc(released):
    m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
    m.update_cable_guard = lambda: None
    m.gripper_holds_object = lambda *a, **k: False
    m.release_empty_grasp = lambda name, posture: released.append((name, posture))
    return m


class TestPlaceOfALostObjectOpensTheGripper(unittest.TestCase):
    def test_open_posture_of_the_first_location(self):
        released = []
        goal = PlaceGoal(attached_object_name='pringles_1')
        location = PlaceLocation()
        location.post_place_posture.points.append(JointTrajectoryPoint(positions=[0.14]))
        goal.place_locations.append(location)
        result = mtc(released).place(goal)
        self.assertEqual(result.error_code.val, MoveItErrorCodes.FAILURE)
        self.assertEqual(released[0][0], 'pringles_1')
        self.assertEqual(list(released[0][1].points[0].positions), [0.14])

    def test_no_location_no_posture(self):
        released = []
        mtc(released).place(PlaceGoal(attached_object_name='pringles_1'))
        self.assertEqual(released, [('pringles_1', None)])


if __name__ == '__main__':
    unittest.main()
