'''#202 part (2): the release height of an open-set place by object height (~open_set_place_clearance_by_height, off by default): a tall object (the Pringles can, box 9.2 x 9.2 x 25.8 cm) is released 1 cm above the support (it tipped over
from 2 cm and stood 4/4 from 0.5 cm), the others 4 cm (claude-b1ef's series: 6/6 places without the lowering aborts of
the default 2 cm). Mocks (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import grasplan.place as place

PRINGLES = (0.092, 0.092, 0.258)
BANANA = (0.19, 0.05, 0.047)
APPLE = (0.088, 0.085, 0.065)


class TestReleaseClearance(unittest.TestCase):
    def setUp(self):
        self.params = {'/use_sim_time': True}
        patcher = mock.patch.object(place.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.p = place.PlaceTools.__new__(place.PlaceTools)
        self.p.open_set_place_clearance = 0.02   # ~open_set_place_clearance, the old single value
        self.held = {}
        self.p.scene = mock.Mock(get_attached_objects=lambda names: {n: self.held[n] for n in names if n in self.held})

    def hold(self, name, size):
        self.held[name] = NS(object=NS(primitives=[NS(dimensions=list(size))]))

    def test_tall_and_flat(self):
        self.params['~open_set_place_clearance_by_height'] = True
        self.hold('pringles_can_1', PRINGLES)
        self.hold('banana_1', BANANA)
        self.hold('apple_1', APPLE)
        self.assertEqual(self.p.open_set_release_clearance('pringles_can_1'), 0.01)
        self.assertEqual(self.p.open_set_release_clearance('banana_1'), 0.04)
        self.assertEqual(self.p.open_set_release_clearance('apple_1'), 0.04)

    def test_code_default_is_off_even_in_sim_and_unknown_box_uses_old_clearance(self):
        self.hold('pringles_can_1', PRINGLES)
        self.assertEqual(self.p.open_set_release_clearance('pringles_can_1'), 0.02)
        self.params['~open_set_place_clearance_by_height'] = True
        self.assertEqual(self.p.open_set_release_clearance('nothing_1'), 0.02)
        self.params['/use_sim_time'] = False
        self.assertEqual(self.p.open_set_release_clearance('pringles_can_1'), 0.01)

    def test_params(self):
        self.params['~open_set_place_clearance_by_height'] = True
        self.hold('pringles_can_1', PRINGLES)
        self.params.update({'~open_set_place_tall_ratio': 3.0, '~open_set_place_clearance_flat': 0.03})
        self.assertEqual(self.p.open_set_release_clearance('pringles_can_1'), 0.03)   # 25.8 / 9.2 < 3: not tall now


if __name__ == '__main__':
    unittest.main()
