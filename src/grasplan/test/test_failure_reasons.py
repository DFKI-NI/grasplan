'''(Oscar 2026-10-03: the robot must say why an action failed, not guess): pick, place and insert put a clear
reason into the action status text also when a check before MTC stops them, MTC names the MoveIt error code and the
arm's stop, and "none of the N grasps could be planned" says why per grasp.
Mocks (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from moveit_msgs.msg import MoveItErrorCodes

import grasplan.insert as insert
import grasplan.mtc_pick_place as mtc_pick_place
import grasplan.pick as pick
import grasplan.place as place
from grasplan.tools import moveit_errors
from grasplan.tools.common import objectToPick


def selector_object(class_id, instance_id):
    return NS(class_id=class_id, instance_id=instance_id, pose=NS(position=None, orientation=None),
              size=NS(x=0.1, y=0.1, z=0.1))


class CodeText(unittest.TestCase):
    def test_known_code_has_name_and_cause(self):
        text = moveit_errors.moveit_code_text(MoveItErrorCodes.CONTROL_FAILED)
        self.assertTrue(text.startswith('CONTROL_FAILED (-4): '), text)
        self.assertIn('controller aborted', text)

    def test_preempted_and_unknown(self):
        self.assertIn('stopped before its end', moveit_errors.moveit_code_text(MoveItErrorCodes.PREEMPTED))
        self.assertEqual(moveit_errors.moveit_code_text(-12345), 'MoveIt error code -12345')

    def test_name_only_when_no_cause(self):
        self.assertEqual(moveit_errors.moveit_code_text(MoveItErrorCodes.INVALID_LINK_NAME), 'INVALID_LINK_NAME (-18)')


class ArmStop(unittest.TestCase):
    def mode(self, value):
        proxy = mock.MagicMock(return_value=NS(safety_mode=NS(mode=value)))
        with mock.patch.object(moveit_errors.rospy, 'wait_for_service'), \
                mock.patch.object(moveit_errors.rospy, 'ServiceProxy', return_value=proxy):
            return moveit_errors.arm_stop_text()

    def test_protective_stop_is_named(self):
        from ur_dashboard_msgs.msg import SafetyMode
        self.assertEqual(self.mode(SafetyMode.PROTECTIVE_STOP),
                         '; the arm is in a protective stop (UR safety mode PROTECTIVE_STOP)')

    def test_normal_mode_says_nothing(self):
        from ur_dashboard_msgs.msg import SafetyMode
        self.assertEqual(self.mode(SafetyMode.NORMAL), '')

    def test_no_dashboard_says_nothing(self):
        def missing(name, timeout):
            raise moveit_errors.rospy.ROSException('timeout exceeded')
        with mock.patch.object(moveit_errors.rospy, 'wait_for_service', missing), \
                mock.patch.object(moveit_errors.rospy, 'logwarn'):
            self.assertEqual(moveit_errors.arm_stop_text(), '')


class Stage:
    '''an MTC stage: children by index, or a leaf with solutions and failures'''
    def __init__(self, name, children=(), solutions=0, failures=()):
        self.name, self.children = name, list(children)
        self.solutions = [object()] * solutions
        self.failures = [NS(comment=c) for c in failures]

    def __getitem__(self, index):
        return self.children[index]


class Task(Stage):
    def __init__(self, stages):
        super().__init__('task', stages)


class MtcReasons(unittest.TestCase):
    def setUp(self):
        self.m = mtc_pick_place.MtcPickPlace.__new__(mtc_pick_place.MtcPickPlace)

    def test_first_failure_is_the_first_stage_without_solutions(self):
        task = Task([Stage('current', solutions=1),
                     Stage('grasp', [Stage('approach', solutions=2, failures=['min_distance not reached']),
                                     Stage('compute ik', failures=['', 'eef in collision'])])])
        self.assertEqual(self.m.first_failure(task), 'failed at grasp/compute ik (eef in collision)')

    def test_first_failure_without_failures(self):
        self.assertEqual(self.m.first_failure(Task([Stage('current', solutions=1)])), 'not planned')

    def test_breakdown_most_frequent_first(self):
        self.m.unplanned = ['refused by the cable guard (wrist span)'] + ['failed at grasp/compute ik'] * 3
        self.assertEqual(self.m.unplanned_breakdown(),
                         ': 3 failed at grasp/compute ik, 1 refused by the cable guard (wrist span)')
        self.m.unplanned = []
        self.assertEqual(self.m.unplanned_breakdown(), '')

    def test_execution_text_prefers_the_recorded_step(self):
        self.m.execution_failure = ''
        self.assertTrue(self.m.execution_text(MoveItErrorCodes.TIMED_OUT).startswith('TIMED_OUT (-6)'))
        self.m.execution_failure = 'the gripper did not finish its command within 5 s'
        self.assertEqual(self.m.execution_text(MoveItErrorCodes.TIMED_OUT), self.m.execution_failure)

    def test_failed_arm_motion_names_code_and_stop(self):
        self.m.execute_client = NS(send_goal=lambda goal: None, wait_for_result=lambda timeout: True,
                                   get_result=lambda: NS(error_code=NS(val=MoveItErrorCodes.CONTROL_FAILED)))
        with mock.patch.object(mtc_pick_place, 'arm_stop_text', return_value='; the arm is in a protective stop'), \
                mock.patch.object(mtc_pick_place.rospy, 'logerr'), \
                mock.patch.object(mtc_pick_place, 'ExecuteTrajectoryGoal', lambda trajectory: None):
            code = self.m.execute_arm(None)
        self.assertEqual(code, MoveItErrorCodes.CONTROL_FAILED)
        self.assertTrue(self.m.execution_failure.startswith('the arm motion failed with CONTROL_FAILED (-4)'))
        self.assertTrue(self.m.execution_failure.endswith('; the arm is in a protective stop'))


class PickReasons(unittest.TestCase):
    def setUp(self):
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc = NS(failure_reason='')
        p.pick_failure_reason = ''
        self.p = p
        patcher = mock.patch.object(pick.rospy, 'logerr')
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_check_reason_wins_over_mtc_and_falls_back(self):
        self.assertEqual(self.p.failure_text(), '')
        self.p.mtc.failure_reason = 'none of the 3 grasps could be planned'
        self.assertEqual(self.p.failure_text(), 'none of the 3 grasps could be planned')
        self.assertFalse(self.p.fail('cannot pick x_1: reason'))
        self.assertEqual(self.p.failure_text(), 'cannot pick x_1: reason')

    def test_without_mtc(self):
        self.p.mtc = None
        self.p.fail('cannot pick x_1: reason')
        self.assertEqual(self.p.failure_text(), 'cannot pick x_1: reason')

    def test_gripper_already_full(self):
        self.p.detach_all_objects_flag = False
        self.p.scene = NS(get_attached_objects=lambda *a: {'relay_1': object()})
        self.assertFalse(self.p.pick_object('multimeter_1', 'table_1', None))
        self.assertEqual(self.p.failure_text(), 'cannot pick multimeter_1: the gripper already holds relay_1 (attached '
                                                'in the planning scene); place or insert it first')

    def test_class_not_in_pose_selector(self):
        self.p.pose_selector_class_query_srv = lambda cls: NS(poses=[])
        result = self.p.make_object_pose_and_add_objs_to_planning_scene(objectToPick('multimeter_1'))
        self.assertEqual(result, (None, None, None))
        self.assertIn('cannot pick multimeter_1: no multimeter is in the pose selector', self.p.failure_text())
        self.assertIn('perceive it with high confidence first', self.p.failure_text())

    def test_other_instance_in_pose_selector(self):
        self.p.pose_selector_class_query_srv = lambda cls: NS(poses=[selector_object('multimeter', 2)])
        self.p.pose_selector_get_all_poses_srv = lambda: NS(poses=NS(objects=[selector_object('multimeter', 2)]))
        self.p.clamp_object_boxes_to_support = False
        self.p.global_reference_frame = 'map'
        self.p.overlaps_target = lambda *a: False
        self.p.add_object_box = lambda *a, **k: None
        with mock.patch.object(pick.rospy, 'loginfo'):
            result = self.p.make_object_pose_and_add_objs_to_planning_scene(objectToPick('multimeter_1'))
        self.assertEqual(result, (None, None, None))
        self.assertEqual(self.p.failure_text(), 'cannot pick multimeter_1: it is not in the pose selector, which holds '
                                                'multimeter_2 of that class; perceive it again or pick one that is there')


class PlaceInsertReasons(unittest.TestCase):
    def setUp(self):
        patcher = mock.patch.object(place.rospy, 'logerr')
        patcher.start()
        self.addCleanup(patcher.stop)
        self.pl = place.PlaceTools.__new__(place.PlaceTools)
        self.pl.mtc = NS(failure_reason='')
        self.pl.scene = NS(get_attached_objects=lambda *a: {})
        self.ins = insert.InsertTools.__new__(insert.InsertTools)
        self.ins.place = self.pl

    def test_place_with_nothing_attached(self):
        with mock.patch.object(place.rospy, 'loginfo'):
            self.assertFalse(self.pl.place_object('table_2'))
        self.assertTrue(self.pl.failure_text().startswith('cannot place on table_2: no object is attached'))

    def test_place_falls_back_to_mtc(self):
        self.pl.place_failure_reason = ''
        self.pl.mtc.failure_reason = 'none of the 5 place locations could be planned'
        self.assertEqual(self.pl.failure_text(), 'none of the 5 place locations could be planned')

    def test_insert_with_nothing_attached(self):
        self.assertFalse(self.ins.insert_object('klt_1'))
        self.assertTrue(self.ins.failure_text().startswith('cannot insert into klt_1: no object is attached'))

    def test_container_not_perceived(self):
        self.ins.pick_pose_selector_class_query_srv = lambda cls: NS(poses=[])
        with mock.patch.object(insert.rospy, 'loginfo'), mock.patch.object(insert.rospy, 'logerr'):
            self.assertIsNone(self.ins.get_support_object_pose(objectToPick('klt_1')))
        self.assertIn('cannot insert into klt_1: no klt is in the pose selector', self.ins.failure_text())

    def test_other_container_perceived(self):
        self.ins.pick_pose_selector_class_query_srv = lambda cls: NS(poses=[NS(instance_id=3)])
        with mock.patch.object(insert.rospy, 'loginfo'), mock.patch.object(insert.rospy, 'logerr'):
            self.assertIsNone(self.ins.get_support_object_pose(objectToPick('klt_1')))
        self.assertIn('which holds klt_3 of that class', self.ins.failure_text())

    def test_unknown_container_height(self):
        support = objectToPick('cardboard_box_1')
        with mock.patch.object(insert.rospy, 'logerr'):
            ok = self.ins.get_object_heights_for_insertion('multimeter_1', 'multimeter', support,
                                                           NS(size=NS(z=0.0)))[0]
        self.assertFalse(ok)
        self.assertIn('cannot insert into cardboard_box_1: its height is unknown', self.ins.failure_text())


if __name__ == '__main__':
    unittest.main()
