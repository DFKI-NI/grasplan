'''pick.top_grasps_for_low_objects decides top only vs side from the shape features of the detection when
~grasp_shape_features is on (careful flag, off in code); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import grasplan.pick as pick
from grasplan import shape_features as sf
from grasplan import shape_message as sm
from grasplan.test.test_low_object_grasps import APPLE, STRAWBERRY, grasp, object_pose, picker
from grasplan.test.test_shape_features import cylinder_front, noise, sphere_front


def detected(cloud, completed_depth=False):
    '''a DetectedObject stand-in whose shape message was filled by the converter from a measured cloud'''
    features = sf.shape_features(noise(cloud), 0.0, completed_depth=completed_depth)
    return NS(shape=sm.fill_message(NS(), features))


class TestShapeRuleInPick(unittest.TestCase):
    def setUp(self):
        self.params = {'~open_set_min_grasp_height': 0.035, '~open_set_min_fingertip_height': 0.015}
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.grasps = [grasp('v', 0.0), grasp('t30', 30.0)]

    def top_grasps(self, box, external):
        return picker().top_grasps_for_low_objects(list(self.grasps), [0.01, 0.02], object_pose(), list(box), external)

    def test_flag_off_the_box_rules_decide_whatever_the_features_say(self):
        kept, _, low = self.top_grasps(STRAWBERRY, detected(cylinder_front(0.0375, 0.24)))
        self.assertTrue(low)                      # the strawberry's footprint rule, as before
        self.assertEqual([g.id for g in kept], ['v'])

    def test_a_small_ball_is_top_only_by_its_features_even_with_a_tall_box(self):
        self.params['~grasp_shape_features'] = True
        kept, _, low = self.top_grasps((0.07, 0.07, 0.12), detected(sphere_front(0.02)))   # box says tall, cloud says 4 cm ball
        self.assertTrue(low)
        self.assertEqual([g.id for g in kept], ['v'])

    def test_a_standing_can_keeps_its_side_grasps_even_when_the_box_looks_low(self):
        self.params['~grasp_shape_features'] = True
        kept, _, low = self.top_grasps(STRAWBERRY, detected(cylinder_front(0.0375, 0.24)))
        self.assertFalse(low)
        self.assertEqual([g.id for g in kept], ['v', 't30'])

    def test_no_features_in_the_detection_falls_back_to_the_box_rules(self):
        self.params['~grasp_shape_features'] = True
        kept, _, low = self.top_grasps(STRAWBERRY, NS(shape=sm.fill_message(NS(), None)))
        self.assertTrue(low)
        self.assertEqual([g.id for g in kept], ['v'])
        kept, _, low = self.top_grasps(APPLE, None)
        self.assertFalse(low)

    def test_unreliable_features_fall_back_to_the_box_rules(self):
        self.params['~grasp_shape_features'] = True
        sparse = detected(cylinder_front(0.03, 0.1, n=60))
        kept, _, low = self.top_grasps(STRAWBERRY, sparse)
        self.assertTrue(low)                      # auto: the box rule
        kept, _, low = self.top_grasps(APPLE, sparse)
        self.assertFalse(low)


if __name__ == '__main__':
    unittest.main()
