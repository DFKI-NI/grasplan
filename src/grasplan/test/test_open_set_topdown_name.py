'''The top-down fallback of an open-set pick looks the object up in the pose selector under the name the open-set
perception stored it with: a goal that names the object by its description with a space ("tennis ball") found
nothing, because the perception stores tennis_ball (sim 2026-11-12: "cannot pick tennis ball: no tennis ball is in
the pose selector" right after "Grasp target tennis_ball_1 committed"). Stubbed (no ROS master); needs the Mobipick
image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import grasplan.pick as pick
from grasplan.tools.common import objectToPick


class TestSelectorName(unittest.TestCase):
    def test_description_with_space(self):
        self.assertEqual(pick.open_set_selector_name('tennis ball'), 'tennis_ball')

    def test_case_and_punctuation_like_the_perception(self):
        self.assertEqual(pick.open_set_selector_name('  Red  Coffee-Mug '), 'red_coffee_mug')

    def test_stored_names_stay(self):
        for name in ('apple', 'tennis_ball', 'tennis_ball_1', 'power_drill_with_grip_1'):
            self.assertEqual(pick.open_set_selector_name(name), name)

    def test_any_instance_of_the_class(self):
        o = objectToPick(pick.open_set_selector_name('tennis ball'))
        self.assertEqual((o.obj_class, o.any_obj_id), ('tennis_ball', True))


class TestFallbackLookup(unittest.TestCase):
    def run_fallback(self, object_name, picked=True):
        p = pick.PickTools.__new__(pick.PickTools)
        p.grasp_type = 'topdown'
        p.pick_failure_reason = ''
        p.mtc = None
        p.pick_object = mock.Mock(return_value=picked)
        goal = NS(object_name=object_name, support_surface_name='table_2', ignore_object_list=[])
        with mock.patch.object(pick.rospy, 'get_param', return_value=True), \
                mock.patch.object(pick.rospy, 'logwarn') as warn:
            result = p.pick_topdown_without_anygrasp(goal, 'AnyGrasp generation failed')
        return p.pick_object.call_args, result, warn.call_args[0][0]

    def test_space_name_asks_the_stored_name(self):
        call, (ok, message, _), warning = self.run_fallback('tennis ball')
        self.assertEqual(call.args[0], 'tennis_ball')
        self.assertTrue(ok)
        self.assertIn('tennis ball', message)
        self.assertIn("tennis_ball ('tennis ball')", warning)

    def test_stored_name_unchanged(self):
        call, _, warning = self.run_fallback('apple')
        self.assertEqual(call.args[0], 'apple')
        self.assertTrue(warning.endswith('pose selector box of apple'), warning)


if __name__ == '__main__':
    unittest.main()
