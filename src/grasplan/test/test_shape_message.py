'''shape_message: the feature dict <-> grasplan/ShapeFeatures conversion, and the message definition.'''
import os
import re
import unittest
from types import SimpleNamespace

from grasplan import shape_features as sf
from grasplan import shape_message as sm
from grasplan.test.test_shape_features import cylinder_front, noise

MSG = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'msg')


def message_fields():
    with open(os.path.join(MSG, 'ShapeFeatures.msg')) as handle:
        text = handle.read()
    return [m.group(2) for m in re.finditer(r'^(\S+)\s+(\w+)\s*(#.*)?$', text, re.M) if '=' not in m.group(0)]


class ShapeMessageTest(unittest.TestCase):
    def test_a_feature_dict_survives_the_message(self):
        features = sf.shape_features(noise(cylinder_front(0.0375, 0.24)), 0.0)
        back = sm.features_from_message(sm.fill_message(SimpleNamespace(), features))
        for key in ('points', 'shape', 'outline_reliable', 'height_reliable', 'round', 'tall', 'flat', 'elongated',
                    'graspable_across'):
            self.assertEqual(back[key], features[key], key)
        for key in sm.FLOATS:
            if features[key] is None:
                self.assertIsNone(back[key], key)
            else:
                self.assertAlmostEqual(back[key], features[key], places=9, msg=key)
        self.assertEqual(back['radii'], features['radii'])

    def test_unmeasured_features_are_nan_and_unknown_and_come_back_as_none(self):
        features = sf.shape_features(noise(cylinder_front(0.03, 0.1, n=60)), 0.0)   # too sparse
        message = sm.fill_message(SimpleNamespace(), features)
        self.assertTrue(message.valid)
        self.assertNotEqual(message.diameter, message.diameter)         # NaN
        self.assertEqual(message.tall, sm.UNKNOWN)
        back = sm.features_from_message(message)
        self.assertIsNone(back['diameter'])
        self.assertIsNone(back['tall'])

    def test_no_features_means_not_valid_and_reads_back_as_none(self):
        message = sm.fill_message(SimpleNamespace(), None)
        self.assertFalse(message.valid)
        self.assertIsNone(sm.features_from_message(message))

    def test_the_message_holds_every_field_the_converter_sets(self):
        fields = set(message_fields())
        converter = {'valid', 'outline_reliable', 'height_reliable', 'points', 'shape', 'radii'} | set(sm.FLOATS) \
            | set(sm.TRI_STATES)
        self.assertEqual(fields, converter)

    def test_detected_object_carries_the_features_and_no_object_is_named_in_the_message(self):
        with open(os.path.join(MSG, 'DetectedObject.msg')) as handle:
            text = handle.read()
        self.assertIn('grasplan/ShapeFeatures shape', text)
        self.assertFalse([f for f in message_fields() if 'name' in f or 'class' in f])


if __name__ == '__main__':
    unittest.main()
