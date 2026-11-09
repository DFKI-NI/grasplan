'''The place location's approach and retreat come from ~pre_place_approach and ~post_place_retreat; without them the
message is the one the node always built (20 cm straight down in <robot>/base_link, 25 cm along -x of
<robot>/gripper_tcp). Mocks rosparam (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped

import grasplan.place as place


class TestPlaceTranslations(unittest.TestCase):
    def setUp(self):
        self.params = {}
        patcher = mock.patch.object(place.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.p = place.PlaceTools.__new__(place.PlaceTools)
        self.p.gripper_joint_names = ['gripper_joint']
        self.p.gripper_joint_efforts = [10.0]
        self.p.gripper_release_distance = 0.1

    def location(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        return self.p.make_place_location_msg(pose)

    def fields(self, translation):
        v = translation.direction
        return (v.header.frame_id, (v.vector.x, v.vector.y, v.vector.z),
                round(translation.desired_distance, 6), round(translation.min_distance, 6))

    def test_default_message_is_the_one_built_before(self):
        loc = self.location()
        self.assertEqual(self.fields(loc.pre_place_approach), ('mobipick/base_link', (0.0, 0.0, -1.0), 0.2, 0.1))
        self.assertEqual(self.fields(loc.post_place_retreat), ('mobipick/gripper_tcp', (-1.0, 0.0, 0.0), 0.25, 0.1))

    def test_parameters_override_the_given_keys_only(self):
        self.params['~pre_place_approach'] = {'frame_id': 'base_link', 'desired': 0.1, 'min_dist': 0.05}
        self.params['~post_place_retreat'] = {'frame_id': '', 'axis': [0.0, 0.0, 1.0], 'desired': 0.12}
        loc = self.location()
        self.assertEqual(self.fields(loc.pre_place_approach), ('base_link', (0.0, 0.0, -1.0), 0.1, 0.05))
        self.assertEqual(self.fields(loc.post_place_retreat), ('mobipick/gripper_tcp', (0.0, 0.0, 1.0), 0.12, 0.1))


if __name__ == '__main__':
    unittest.main()
