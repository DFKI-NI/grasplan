'''#202: the MTC place logs the chosen place pose (planning frame) before it executes, log only. Stubbed (no ROS master).'''
import math
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlaceLocation
import tf2_ros

import grasplan.mtc_pick_place as mpp


def location(x, y, z, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = 'table_2'
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
    pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
    pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
    return PlaceLocation(id='70', place_pose=pose)


class TestPlacePoseLog(unittest.TestCase):
    def setUp(self):
        self.m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        self.m.to_planning_frame = lambda pose: pose
        patcher = mock.patch.object(mpp.rospy, 'loginfo')
        self.log = patcher.start()
        self.addCleanup(patcher.stop)

    def test_logs_position_yaw_and_frame(self):
        self.m.log_place_pose(location(19.45, 14.0, 0.855, 90.0))
        self.log.assert_called_once()
        text = self.log.call_args[0][0]
        self.assertIn('place location 70', text)
        self.assertIn('x 19.450 y 14.000 z 0.855 yaw 90 deg in table_2', text)

    def test_a_missing_transform_logs_nothing_and_does_not_raise(self):
        def fail(pose):
            raise tf2_ros.LookupException('no frame')
        self.m.to_planning_frame = fail
        self.m.log_place_pose(location(0, 0, 0, 0))
        self.log.assert_not_called()


if __name__ == '__main__':
    unittest.main()
