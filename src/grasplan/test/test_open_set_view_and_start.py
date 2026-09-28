'''#151 open-set pick (Oscar, 2026-09-28): the grasp view comes only from the view service (no fixed anygrasp pose
fallback unless ~anygrasp_named_view_fallback), and the grasp is planned from ~open_set_pick_start_pose (transport)
instead of from the upside-down view. Mocks for the services, the arm and the scene (no ROS master); needs the
Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from geometry_msgs.msg import PoseStamped

import grasplan.pick as pick


class ViewFixture(unittest.TestCase):
    def setUp(self):
        self.params = {}
        self.moves = []
        patches = [
            mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default)),
            mock.patch.object(pick.rospy, 'wait_for_service', self.wait_for_service),
            mock.patch.object(pick.rospy, 'ServiceProxy', lambda name, srv: self.view),
        ]
        for patcher in patches:
            patcher.start()
            self.addCleanup(patcher.stop)
        self.service_up = True
        self.view = lambda object_name: NS(success=True, message='camera 0.65 m from it')
        self.p = pick.PickTools.__new__(pick.PickTools)
        self.p.anygrasp_view_service = '/mobipick/grasp_view'
        self.p.anygrasp_arm_pose = 'anygrasp'
        self.p.move_arm_to_posture = lambda name: self.moves.append(name) or True

    def wait_for_service(self, name, timeout=None):
        if not self.service_up:
            raise pick.rospy.ROSException('timeout exceeded while waiting for service')


class TestGraspViewWithoutFixedPose(ViewFixture):
    def test_view_service_success(self):
        ok, message = self.p.move_to_anygrasp_view('tomato soup can')
        self.assertTrue(ok)
        self.assertEqual(self.moves, [])

    def test_no_committed_pose_asks_to_perceive_first(self):
        self.view = lambda object_name: NS(success=False, message=f'{object_name} has no committed pose in the pose selector')
        ok, message = self.p.move_to_anygrasp_view('tomato soup can')
        self.assertFalse(ok)
        self.assertIn('perceive tomato soup can first', message)
        self.assertEqual(self.moves, [])   # no fixed anygrasp pose any more

    def test_service_unavailable_fails_clearly(self):
        self.service_up = False
        ok, message = self.p.move_to_anygrasp_view('tomato soup can')
        self.assertFalse(ok)
        self.assertIn('grasp view service /mobipick/grasp_view unavailable', message)
        self.assertEqual(self.moves, [])

    def test_other_view_failure(self):
        self.view = lambda object_name: NS(success=False, message='no grasp view of it: no reachable sample')
        ok, message = self.p.move_to_anygrasp_view('pear')
        self.assertFalse(ok)
        self.assertIn('no grasp view of pear', message)

    def test_old_fallback_behind_the_switch(self):
        self.service_up = False
        self.params['~anygrasp_named_view_fallback'] = True
        ok, message = self.p.move_to_anygrasp_view('pear')
        self.assertTrue(ok)
        self.assertEqual(self.moves, ['anygrasp'])


def object_pose():
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.orientation.w = 1.0
    return pose


class TestPickStartsFromTransport(unittest.TestCase):
    def setUp(self):
        self.params = {}
        self.calls = []
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        p = pick.PickTools.__new__(pick.PickTools)
        p.detach_all_objects_flag = False
        p.scene = mock.Mock(get_attached_objects=lambda: {}, get_known_object_names=lambda: [])
        p.perceive_object = False
        p.clear_planning_scene = False
        p.add_custom_boxes_to_ps = lambda boxes: None
        p.planning_scene_boxes = []
        p.make_object_pose_and_add_objs_to_planning_scene = lambda *a, **k: (object_pose(), [0.07, 0.07, 0.11], 1)
        p.mtc = None
        p.obj_pose_pub = mock.Mock()
        p.pregrasp_posture_required = False
        p.pick_action_server = mock.Mock(is_preempt_requested=lambda: False)
        p.move_ok = True
        p.move_arm_to_posture = lambda name: self.calls.append(('move', name)) or p.move_ok
        p.topdown_fallback_grasps = lambda *a: self.calls.append(('grasps', None)) or []
        # closed-set path
        p.grasp_planner = mock.Mock(make_grasps_msgs=lambda *a: self.calls.append(('closed grasps', None)) or [])
        p.robot = mock.Mock()
        p.clear_octomap_flag = False
        p.list_of_disentangle_objects = []
        p.external_grasp_batch_size = 0
        p._pick_with_action = lambda *a: None
        self.p = p

    def pick(self, external=True):
        kwargs = dict(external_grasp_candidates=[]) if external else {}
        return self.p.pick_object('tomato_soup_can_1', 'table_2', 'side_grasp', [], **kwargs)

    def test_open_set_moves_to_transport_before_the_grasps(self):
        self.assertFalse(self.pick())   # no grasp at all in this stub
        self.assertEqual(self.calls, [('move', 'transport'), ('grasps', None)])

    def test_start_pose_switch_off(self):
        self.params['~open_set_pick_start_pose'] = ''
        self.pick()
        self.assertEqual(self.calls, [('grasps', None)])

    def test_failed_move_stops_the_pick(self):
        self.p.move_ok = False
        self.assertFalse(self.pick())
        self.assertEqual(self.calls, [('move', 'transport')])

    def test_failed_move_reason_reaches_the_action_text(self):
        self.p.move_ok = False
        self.p.mtc = NS(failure_reason='', yaw_free_objects=set(), strict_contact_order=False, grasp_ik_states={})
        self.pick()
        self.assertIn("could not move the arm to 'transport'", self.p.mtc.failure_reason)

    def test_closed_set_pick_does_not_move(self):
        self.pick(external=False)
        self.assertEqual(self.calls, [('closed grasps', None)])


if __name__ == '__main__':
    unittest.main()
