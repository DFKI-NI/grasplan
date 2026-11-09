'''Source checks: a robot's names reach the pick, place and insert nodes as parameters. The absolute topics of the pick
and place nodes keep their old names as defaults, and no node builds a frame or a topic from the prefix by hand, which
would give /gripper_tcp and //gripper_hw on a robot without a prefix.'''
import os
import re
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')


def source(name):
    with open(os.path.join(ROOT, name)) as f:
        return f.read()


class TestTopicParameters(unittest.TestCase):
    def test_pick_topics(self):
        text = source('pick.py')
        for param, topic in [('perception_trigger_topic', '/object_recognition/event_in'),
                             ('grasps_marker_topic', '/gripper'),
                             ('pose_selector_marker_topic', '/pose_selector_objects')]:
            self.assertIn(f"rospy.get_param('~{param}', '{topic}')", text)

    def test_place_topic(self):
        self.assertIn("rospy.get_param('~place_poses_marker_topic', '/place_pose_selector_objects')", source('place.py'))


class TestNoHandMadeNames(unittest.TestCase):
    def test_every_node_uses_the_helpers(self):
        for name in ['pick.py', 'place.py', 'insert.py', 'mtc_pick_place.py']:
            self.assertEqual(re.findall(r"\{robot_prefix\(\)\}", source(name)), [], name)
            self.assertNotIn("robot_prefix() + '/'", source(name), name)


if __name__ == '__main__':
    unittest.main()
