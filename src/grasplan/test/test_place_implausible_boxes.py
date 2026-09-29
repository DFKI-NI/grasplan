'''#202 (real 2026-09-28 places after the Pringles picks): the place node added every pick pose selector box, also the
false DOPE screwdriver_1 box floating 13 cm above table_2, which blocked the spots near the pick and sent the can to the
table corner, where it tipped off. With ~place_drop_implausible_boxes (sim on, real off) the place scene leaves out a
box more than ~max_box_sink_into_support inside its table and one floating more than ~max_box_float_above_support above
it unless it rests on another box. Mocks (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from geometry_msgs.msg import Point, Pose, Quaternion
from shape_msgs.msg import SolidPrimitive

import grasplan.place as place

I = Quaternion(0.0, 0.0, 0.0, 1.0)
TABLE = NS(header=NS(frame_id='map'), pose=Pose(position=Point(0.0, 0.0, 0.36), orientation=I),
           primitives=[NS(type=SolidPrimitive.BOX, dimensions=[0.8, 0.8, 0.72])],
           primitive_poses=[Pose(orientation=Quaternion(0.0, 0.0, 0.0, 0.0))])   # top at 0.72


def box(class_id, center, size):
    return NS(class_id=class_id, instance_id=1, pose=Pose(position=Point(*center), orientation=I),
              size=NS(x=size[0], y=size[1], z=size[2]))


CAN = box('tomato_soup_can', (0.1, 0.0, 0.7775), (0.07, 0.07, 0.115))           # standing on the table
SCREWDRIVER = box('screwdriver', (0.2, 0.1, 0.866), (0.245, 0.034, 0.034))      # the false box: 12.9 cm above
BLEACH = box('bleach', (-0.1, 0.25, 0.7205), (0.102, 0.068, 0.251))             # false box 12 cm inside the table
KLT = box('klt', (-0.2, -0.1, 0.7295), (0.297, 0.197, 0.147))                   # real klt, 6.4 cm deep (DOPE)
ON_CAN = box('multimeter', (0.1, 0.0, 0.86), (0.06, 0.06, 0.05))                # stacked on the can: 11.5 cm up


class TestPlaceSceneBoxes(unittest.TestCase):
    def setUp(self):
        self.params = {'/use_sim_time': True}
        patcher = mock.patch.object(place.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.added, self.removed = [], []
        p = place.PlaceTools.__new__(place.PlaceTools)
        p.global_reference_frame = 'map'
        p.scene = mock.Mock(get_objects=lambda: {'table_2': TABLE}, get_known_object_names=lambda: ['screwdriver_1'],
                            add_box=lambda name, pose, size: self.added.append(name),
                            remove_world_object=lambda name: self.removed.append(name))
        self.p = p

    def scene(self, *objects):
        self.p.get_all_poses_pick_pose_selector_srv = lambda: NS(poses=NS(objects=list(objects)))
        self.p.add_objs_to_planning_scene()
        return self.added

    def test_false_boxes_left_out_real_ones_kept(self):
        added = self.scene(CAN, SCREWDRIVER, BLEACH, KLT, ON_CAN)
        self.assertEqual(sorted(added), ['klt_1', 'multimeter_1', 'tomato_soup_can_1'])
        self.assertEqual(self.removed, ['screwdriver_1'])   # an old copy in the scene goes too

    def test_real_robot_default_adds_everything(self):
        self.params['/use_sim_time'] = False
        self.assertEqual(len(self.scene(CAN, SCREWDRIVER, BLEACH, KLT, ON_CAN)), 5)

    def test_switches(self):
        self.params['~max_box_float_above_support'] = 0.0
        self.assertIn('screwdriver_1', self.scene(SCREWDRIVER))
        self.params['~max_box_sink_into_support'] = 0.0
        self.assertIn('bleach_1', self.scene(BLEACH))

    def test_without_tables_nothing_is_judged(self):
        self.p.scene.get_objects = lambda: {}
        self.assertEqual(sorted(self.scene(SCREWDRIVER, BLEACH)), ['bleach_1', 'screwdriver_1'])


if __name__ == '__main__':
    unittest.main()
