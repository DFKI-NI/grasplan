'''A pick, place or insert that MoveIt cannot plan says how hard it tried, not only "PLANNING_FAILED (-1): no motion
plan was found" (real robot and sim runs of 2026-11-04: all six manipulation failures the planner agent got were that
bare text, so it could not tell one grasp from fifty or a full table from an unreachable one).
Mocks (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from moveit_msgs.msg import MoveItErrorCodes

import grasplan.insert as insert
import grasplan.pick as pick
import grasplan.place as place
from grasplan.tools import moveit_errors
from grasplan.tools.common import objectToPick

PLANNING_FAILED = MoveItErrorCodes.PLANNING_FAILED


class TriedText(unittest.TestCase):
    def test_counts_attempts_and_seconds(self):
        self.assertEqual(moveit_errors.tried_text(12, 'grasp', 'of relay_1', 4.06), '12 grasps of relay_1 tried, 4.1 s')
        self.assertEqual(moveit_errors.tried_text(1, 'grasp', 'of relay_1', 0.5), '1 grasp of relay_1 tried, 0.5 s')
        self.assertEqual(moveit_errors.tried_text(80, 'place pose', 'on table_3', 47.4, attempts=3),
                         '80 place poses on table_3 tried in 3 attempts, 47.4 s')
        self.assertEqual(moveit_errors.tried_text(0, 'grasp', 'of relay_1', 1.0), '')

    def test_motion_failure_text_carries_it_before_the_stop(self):
        with mock.patch.object(moveit_errors, 'arm_stop_text', return_value='; the arm is in a protective stop'):
            text = moveit_errors.motion_failure_text('the pick', PLANNING_FAILED, '12 grasps of relay_1 tried, 4.1 s')
        self.assertEqual(text, 'the pick failed: PLANNING_FAILED (-1): no motion plan was found (12 grasps of relay_1 '
                               'tried, 4.1 s); the arm is in a protective stop')

    def test_without_it_the_text_stays_as_before(self):
        with mock.patch.object(moveit_errors, 'arm_stop_text', return_value=''):
            self.assertEqual(moveit_errors.motion_failure_text('the place', PLANNING_FAILED),
                             'the place failed: PLANNING_FAILED (-1): no motion plan was found')
        self.assertEqual(moveit_errors.motion_failure_text('the place', False, 'x tried'), 'the place was cancelled')


class PickCountsEveryOfferedGrasp(unittest.TestCase):
    def test_batches_and_fallback_add_up(self):
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc = object()
        p._pick_with_mtc = lambda *a: PLANNING_FAILED
        p.grasps_tried, p.pick_started = 0, pick.time.time() - 4.0
        for batch in ([NS()] * 5, [NS()] * 5, [NS()] * 2):   # two batches of the ranked grasps, the top-down fallback
            self.assertEqual(p._pick_with_action(objectToPick('relay_1'), batch, 'table_2'), PLANNING_FAILED)
        text = p.pick_tried_text(objectToPick('relay_1'))
        self.assertRegex(text, r'^12 grasps of relay_1 tried, 4\.\d s$')

    def test_nothing_offered_gives_no_detail(self):
        p = pick.PickTools.__new__(pick.PickTools)
        self.assertEqual(p.pick_tried_text(objectToPick('relay_1')), '')


class PlaceCountsEveryBatch(unittest.TestCase):
    def setUp(self):
        for name in ('logerr', 'loginfo'):
            patcher = mock.patch.object(place.rospy, name)
            patcher.start()
            self.addCleanup(patcher.stop)
        stop = mock.patch.object(moveit_errors, 'arm_stop_text', return_value='')
        stop.start()
        self.addCleanup(stop.stop)
        self.pl = place.PlaceTools.__new__(place.PlaceTools)
        self.pl.mtc = None
        self.pl.place_failure_reason = ''
        self.pl.planning_failure = None
        self.pl.place_tried = {'poses': 0, 'attempts': 1, 'free': (0, 5), 'started': place.time.time() - 2.0}

    def test_the_text_is_renewed_per_batch_and_attempt(self):
        self.pl.note_failed_place_batch(PLANNING_FAILED, 5, 'table_3')
        self.assertRegex(self.pl.failure_text(), r'^the place failed: PLANNING_FAILED \(-1\): no motion plan was found '
                                                 r'\(5 place poses on table_3 tried, 2\.\d s; 0 of the last 5 had a '
                                                 r'free footprint\)$')
        # the next attempts draw 25 and 50 poses (in batches of 20); place_object resets the reason per attempt
        for attempt, batches, drawn in ((2, (20, 5), 25), (3, (20, 20, 10), 50)):
            self.pl.place_failure_reason = ''
            self.pl.place_tried.update(attempts=attempt, free=(12, drawn))
            for count in batches:
                self.pl.note_failed_place_batch(PLANNING_FAILED, count, 'table_3')
        self.assertIn('(80 place poses on table_3 tried in 3 attempts, 2.', self.pl.failure_text())
        self.assertTrue(self.pl.failure_text().endswith('s; 12 of the last 50 had a free footprint)'))

    def test_a_check_or_mtc_reason_is_kept(self):
        self.pl.fail('cannot place on table_3: no object is attached to the gripper')
        self.pl.note_failed_place_batch(PLANNING_FAILED, 5, 'table_3')
        self.assertEqual(self.pl.failure_text(), 'cannot place on table_3: no object is attached to the gripper')
        self.assertEqual(self.pl.place_tried['poses'], 5)   # still counted
        self.pl.place_failure_reason = ''
        self.pl.mtc = NS(failure_reason='none of the 5 place locations could be planned')
        self.pl.note_failed_place_batch(PLANNING_FAILED, 5, 'table_3')
        self.assertEqual(self.pl.failure_text(), 'none of the 5 place locations could be planned')

    def test_without_free_footprint_ordering_no_free_count(self):
        self.pl.place_tried['free'] = None
        self.pl.note_failed_place_batch(PLANNING_FAILED, 5, 'table_3')
        self.assertRegex(self.pl.failure_text(), r'\(5 place poses on table_3 tried, 2\.\d s\)$')


class InsertCountsBothAttempts(unittest.TestCase):
    def test_two_attempts(self):
        ins = insert.InsertTools.__new__(insert.InsertTools)
        ins.insert_tried = {'poses': 0, 'attempts': 0, 'started': insert.time.time() - 28.0}
        ins.count_insert_poses(8)
        ins.count_insert_poses(16)
        self.assertRegex(ins.insert_tried_text('klt_1'), r'^24 insert poses into klt_1 tried in 2 attempts, 28\.\d s$')

    def test_before_any_goal(self):
        ins = insert.InsertTools.__new__(insert.InsertTools)
        self.assertEqual(ins.insert_tried_text('klt_1'), '')

    def _ranked(self, passing, codes):
        ins = insert.InsertTools.__new__(insert.InsertTools)
        ins.insert_tried = {'poses': 0, 'attempts': 0, 'started': insert.time.time() - 28.0}
        ins.insert_gate_passing = passing
        ins.insert_action_server = NS(is_preempt_requested=lambda: False)
        replies = iter(codes)
        offered = []

        def run_place_goal(goal, server, helper):
            offered.append(len(goal.place_locations))
            return NS(error_code=NS(val=next(replies)))

        ins.place = NS(mtc=NS(executed=False), run_place_goal=run_place_goal)
        ins.action_client_helper = None
        ins.run_ranked_insert_goal(NS(place_locations=list(range(24))))
        return ins, offered

    def test_each_cable_gate_group_is_an_attempt(self):
        ins, offered = self._ranked(8, [PLANNING_FAILED, PLANNING_FAILED])
        self.assertEqual(offered, [8, 16])
        self.assertRegex(ins.insert_tried_text('klt_1'), r'^24 insert poses into klt_1 tried in 2 attempts, 28\.\d s$')

    def test_an_ungated_goal_is_one_attempt_and_a_success_counts_nothing(self):
        ins, _ = self._ranked(0, [PLANNING_FAILED])
        self.assertRegex(ins.insert_tried_text('klt_1'), r'^24 insert poses into klt_1 tried, 28\.\d s$')
        ins, offered = self._ranked(8, [PLANNING_FAILED, MoveItErrorCodes.SUCCESS])
        self.assertEqual(offered, [8, 16])
        self.assertEqual(ins.insert_tried['poses'], 8)   # only the group that failed


if __name__ == '__main__':
    unittest.main()
