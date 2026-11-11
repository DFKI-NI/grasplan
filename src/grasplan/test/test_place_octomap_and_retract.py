'''real 2026-09-28: a place smashed klt_1 into an object on table_3 that only the octomap knew, because the place
task allowed octomap contact for the held object and the gripper from the start, so the transfer ignored it. Now the
octomap contact holds only for the place pose and the final lowering ('octomap blocks the transfer' before the
Connect). And a failed pick/place/insert that moved the arm retracts it to transport. Stubbed MTC stages and arm (no
ROS master, no robot model); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlaceGoal, PlaceLocation

import grasplan.insert as insert
import grasplan.mtc_pick_place as mpp
import grasplan.pick as pick
import grasplan.place as place
from test_pick_contact_order import Container, Stage, Task

OBJ, GRIPPER = 'klt_1', ['finger_l', 'finger_r', 'palm']


class PlaceStage(Stage):
    def detachObject(self, *args):
        pass


def build(params=None, octomap=True):
    params = params or {}
    fake_stages = mock.Mock(ModifyPlanningScene=PlaceStage)
    fake_core = mock.Mock(SerialContainer=Container)
    m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
    m.gripper_links, m.allow_octomap_contact, m.eef_link = GRIPPER, octomap, 'gripper_tcp'
    m.start_task = lambda name: (Task(name), [mpp.NOOP])
    m.connect = lambda name: Stage(name)
    m.move_relative = lambda name, translation: Stage(name)

    def ik_stage(name, pose, monitored):
        ik = Stage(name)
        ik.monitored = monitored
        return ik
    m.compute_ik = ik_stage
    m.to_planning_frame = lambda pose: pose
    goal = PlaceGoal(attached_object_name=OBJ, support_surface_name='table_3', place_eef=True)
    location = PlaceLocation(id='4', place_pose=PoseStamped())
    with mock.patch.object(mpp, 'stages', fake_stages), mock.patch.object(mpp, 'core', fake_core), \
            mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: params.get(name, default)):
        return m.make_place_task(goal, location, None)


def octomap_entries(stage):
    return [(set(b), allow) for a, b, allow in stage.allowed if a == mpp.OCTOMAP_COLLISION_NAME]


class TestPlaceOctomap(unittest.TestCase):
    def test_transfer_sees_the_octomap_the_lowering_does_not(self):
        task, roles = build()
        names = [c.name for c in task.children]
        self.assertEqual(names[:4], ['allow object contacts', 'octomap blocks the transfer', 'move to preplace', 'place'])
        self.assertEqual(octomap_entries(task.children[0]), [(set(GRIPPER + [OBJ]), True)])
        self.assertEqual(octomap_entries(task.children[1]), [(set(GRIPPER + [OBJ]), False)])
        ik = task['place'].children[1]
        self.assertIs(ik.monitored, task.children[0])   # place pose and lowering keep the octomap contact
        self.assertEqual(len(roles), 1 + 3 + len(task['place'].children))   # current state, 2 scene stages, connect

    def test_switch_off_restores_the_old_task(self):
        task, _ = build({'~mtc_place_octomap_blocks_transfer': False})
        self.assertEqual([c.name for c in task.children][:3], ['allow object contacts', 'move to preplace', 'place'])

    def test_no_octomap_allowance_no_block(self):
        task, _ = build(octomap=False)
        self.assertNotIn('octomap blocks the transfer', [c.name for c in task.children])


class TestRetractAfterFailure(unittest.TestCase):
    def setUp(self):
        self.params = {}
        self.moves = []
        for module in (pick, place):
            patcher = mock.patch.object(module.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
            patcher.start()
            self.addCleanup(patcher.stop)

    def tools(self, cls):
        t = cls.__new__(cls)
        t.move_arm_to_posture = lambda name: self.moves.append(name) or True
        return t

    def test_pick_and_place_retract_to_transport(self):
        self.tools(pick.PickTools).retract_after_failure('pick')
        self.tools(place.PlaceTools).retract_after_failure('place')
        self.assertEqual(self.moves, ['transport', 'transport'])

    def test_switch_off(self):
        self.params['~retract_pose_after_failure'] = ''
        self.tools(pick.PickTools).retract_after_failure('pick')
        self.assertEqual(self.moves, [])

    def test_failed_pick_that_moved_the_arm_retracts(self):
        p = self.pick_stub(executed=True)
        self.assertFalse(p.pick_object('banana_1', 'table_2', 'side_grasp', [], external_grasp_candidates=[]))
        self.assertEqual(self.moves, ['transport', 'transport'])   # the open-set start pose, then the retract

    def test_failed_pick_that_did_not_move_stays(self):
        p = self.pick_stub(executed=False)
        p.pick_object('banana_1', 'table_2', 'side_grasp', [], external_grasp_candidates=[])
        self.assertEqual(self.moves, ['transport'])                # only the open-set start pose

    def pick_stub(self, executed):
        p = self.tools(pick.PickTools)
        p.detach_all_objects_flag = False
        p.scene = mock.Mock(get_attached_objects=lambda: {}, get_known_object_names=lambda: [])
        p.perceive_object = False
        p.clear_planning_scene = False
        p.add_custom_boxes_to_ps = lambda boxes: None
        p.planning_scene_boxes = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.w = 1.0
        p.make_object_pose_and_add_objs_to_planning_scene = lambda *a, **k: (pose, [0.18, 0.08, 0.05], 1)
        p.mtc = NS(failure_reason='', yaw_free_objects=set(), strict_contact_order=False, grasp_ik_states={},
                   executed=executed)
        p.obj_pose_pub = mock.Mock()
        p.pregrasp_posture_required = False
        p.pick_action_server = mock.Mock(is_preempt_requested=lambda: False)
        p.open_gripper = lambda: True
        p.topdown_fallback_grasps = lambda *a: [mock.Mock(id='topdown_0', grasp_quality=0.5)]
        p.clear_octomap_flag = False
        p.list_of_disentangle_objects = []
        p.external_grasp_batch_size = 0
        p.cable_gate_passing = 0
        p._pick_with_action = lambda *a: mpp.MoveItErrorCodes.FAILURE
        p.arm_did_not_move = lambda result: not executed
        return p


class TestRetractFailureText(unittest.TestCase):
    """a retract that fails after a failed pick, place or insert joins the failure text: the caller hears where the
    arm was left"""
    def setUp(self):
        for module in (pick, place):
            for name, value in (('get_param', lambda name, default=None: default), ('logwarn', lambda *a: None),
                                ('loginfo', lambda *a: None)):
                patcher = mock.patch.object(module.rospy, name, value)
                patcher.start()
                self.addCleanup(patcher.stop)

    def tools(self, cls, moved, reason=''):
        t = cls.__new__(cls)
        t.mtc = None

        def move(name):
            t.arm_move_report = {'reason': reason} if reason else {}
            return moved
        t.move_arm_to_posture = move
        return t

    def test_a_refused_retract_is_named_with_the_guards_reason(self):
        for cls, attr, what in ((pick.PickTools, 'pick_failure_reason', 'pick'),
                                (place.PlaceTools, 'place_failure_reason', 'place')):
            t = self.tools(cls, moved=False, reason='the cable guard refused both plans')
            setattr(t, attr, 'the lift was refused')
            t.retract_after_failure(what)
            self.assertEqual(t.failure_text(), 'the lift was refused; retracting the arm to transport afterwards failed '
                             f'too (the cable guard refused both plans): the arm stays where the {what} stopped')

    def test_a_failed_retract_without_a_guard_reason_says_so(self):
        t = self.tools(pick.PickTools, moved=False)
        t.pick_failure_reason = 'no grasp could be planned'
        t.retract_after_failure('pick')
        self.assertIn('afterwards failed too (the arm move failed)', t.failure_text())

    def test_a_good_retract_or_no_failure_adds_nothing(self):
        t = self.tools(pick.PickTools, moved=True)
        t.pick_failure_reason = 'no grasp could be planned'
        t.retract_after_failure('pick')
        self.assertEqual(t.failure_text(), 'no grasp could be planned')
        t = self.tools(pick.PickTools, moved=False)
        t.pick_failure_reason = ''
        t.retract_after_failure('pick')
        self.assertEqual(t.failure_text(), '')   # no reason: no dangling '; retracting ...'

    def test_the_insert_text_carries_the_place_nodes_retract(self):
        ins = insert.InsertTools.__new__(insert.InsertTools)
        ins.place = self.tools(place.PlaceTools, moved=False, reason='refused')
        ins.place.mtc = None
        ins.insert_failure_reason = 'the insert was refused'
        ins.place.retract_after_failure('insert')
        self.assertTrue(ins.failure_text().endswith('the arm stays where the insert stopped'), ins.failure_text())


if __name__ == '__main__':
    unittest.main()
