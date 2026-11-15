'''An object the pick node's second try picked with the open-set grasp finder (see test_place_open_set_fallback_mode) is
inserted as an open-set object although its class is known: its height comes from its attached (measured) box, given
explicitly to compute_object_height_for_insertion so the class table is not used, and its insert poses get no resting
roll of its class. A normally picked object of the class keeps the table height and the roll. The insert goal drops the
note once the object is let go. Mocks for rosparam (no ROS master); needs the Mobipick image for the module imports.'''
import inspect
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import tf.transformations

import grasplan.insert as insert
from grasplan.tools import support_plane_tools as spt


def support_pose():
    return NS(pose=NS(position=NS(x=1.0, y=0.0, z=0.8), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
              size=NS(x=0.4, y=0.3, z=0.2))


class TestHeights(unittest.TestCase):
    def node(self, open_set):
        i = insert.InsertTools.__new__(insert.InsertTools)
        box = NS(object=NS(primitives=[NS(dimensions=[0.1, 0.07, 0.067])]))
        i.place = NS(scene=NS(get_attached_objects=lambda names=None: {'soup_1': box}))
        i.inserting_open_set = open_set
        i.fail = lambda text: False
        return i

    def heights(self, open_set):
        from grasplan.tools.common import objectToPick
        with mock.patch.object(insert.rospy, 'loginfo'):
            return self.node(open_set).get_object_heights_for_insertion('soup_1', 'soup', objectToPick('klt_1'),
                                                                        support_pose())

    def test_the_second_try_object_takes_its_measured_height(self):
        self.assertEqual(self.heights(True), (True, 0.067, None))
        height = spt.compute_object_height_for_insertion('soup', 'klt', object_tbi_height=0.067)
        self.assertAlmostEqual(height, spt.OBJECT_HEIGHTS['klt'] / 2.0 + 0.067 / 2.0 + 0.02)

    def test_a_normally_picked_soup_keeps_the_table_height(self):
        self.assertEqual(self.heights(False), (True, None, None))


class TestRoll(unittest.TestCase):
    def roll(self, open_set):
        with mock.patch.object(spt.rospy, 'loginfo', create=True):
            pose = spt.gen_insert_poses_from_obj('soup', support_pose(), 0.1, open_set=open_set).objects[0].pose
        q = pose.orientation
        return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[0]

    def test_no_class_roll_for_the_second_try_object(self):
        self.assertAlmostEqual(self.roll(True), 0.0)
        self.assertAlmostEqual(self.roll(False), -math.pi / 2.0)


class TestWiring(unittest.TestCase):
    def test_the_insert_reads_the_mode_and_forgets_the_note(self):
        source = inspect.getsource(insert.InsertTools.insert_object)
        mode = source.index('self.inserting_open_set = self.place.picked_by_open_set_fallback(object_to_be_inserted)')
        self.assertLess(mode, source.index('self.get_object_heights_for_insertion('))
        self.assertIn("open_set=getattr(self, 'inserting_open_set', False)", source)
        comfortable = inspect.getsource(insert.InsertTools.gen_comfortable_insert_poses)
        self.assertIn("open_set=getattr(self, 'inserting_open_set', False)", comfortable)
        callback = inspect.getsource(insert.InsertTools.insert_obj_action_callback)
        self.assertLess(callback.index('self.place.forget_open_set_fallback_held()'),
                        callback.index('self.insert_action_server.set_succeeded'))


if __name__ == '__main__':
    unittest.main()
