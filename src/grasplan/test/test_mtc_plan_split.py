'''plan time split (log only): MTC's stage statistics turn task.plan() into scene fetch ('current state' stage) and
compute in the TIMING line (plan/init, plan/scene, plan/compute, plan/finish), the slowest stages and the scene size are
logged per plan, and a missing statistics topic costs no time. Fake task, no ROS master; needs the Mobipick image for
the message imports.'''
import time
import unittest
from unittest import mock

from moveit_task_constructor_msgs.msg import (Solution, StageDescription, StageStatistics, TaskDescription,
                                              TaskStatistics)
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

import grasplan.mtc_pick_place as mpp
from grasplan.tools import mtc_stage_times as mst
from grasplan.tools.stage_timing import TIMER


def description(task_id='t1'):
    # root task (id 1, parent 0), its serial container's leaves and one inner container with two leaves
    stages = [StageDescription(id=1, parent_id=0, name='pick banana'),
              StageDescription(id=2, parent_id=1, name='current state'),
              StageDescription(id=3, parent_id=1, name='move to pregrasp'),
              StageDescription(id=4, parent_id=1, name='grasp'),
              StageDescription(id=5, parent_id=4, name='approach'),
              StageDescription(id=6, parent_id=4, name='compute ik')]
    return TaskDescription(task_id=task_id, stages=stages)


def statistics(times, task_id='t1'):
    return TaskStatistics(task_id=task_id, stages=[StageStatistics(id=i, total_compute_time=v) for i, v in times.items()])


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class TestStageTimes(unittest.TestCase):
    def test_leaf_times_by_path_without_root_and_containers(self):
        st = mst.StageTimes(subscribe=False)
        st.on_description(description())
        leaves = st.leaf_times(statistics({1: 9.0, 2: 6.5, 3: 0.4, 4: 2.0, 5: 0.1, 6: 1.8}))
        self.assertEqual(leaves, {'current state': 6.5, 'move to pregrasp': 0.4, 'grasp/approach': 0.1,
                                  'grasp/compute ik': 1.8})
        self.assertEqual(mst.scene_seconds(leaves), 6.5)
        self.assertEqual(mst.slowest(leaves, 2), 'current state 6.50 s, grasp/compute ik 1.80 s')

    def test_unknown_task_tree_gives_none(self):
        st = mst.StageTimes(subscribe=False)
        st.on_description(description('other'))
        self.assertIsNone(st.leaf_times(statistics({2: 1.0})))
        self.assertIsNone(st.leaf_times(None))
        self.assertIsNone(mst.scene_seconds(None))
        self.assertIsNone(mst.scene_seconds({'move to pregrasp': 0.4}))
        self.assertEqual(mst.slowest(None), 'n/a')

    def test_latest_since_never_waits_without_a_topic(self):
        clock = Clock()
        st = mst.StageTimes(subscribe=False, clock=clock)
        sleeps = []

        def sleep(s):
            sleeps.append(s)
            clock.now += s
        self.assertIsNone(st.latest_since(99.0, sleep=sleep))
        self.assertEqual(sleeps, [])

    def test_latest_since_takes_only_messages_of_this_plan(self):
        clock = Clock()
        st = mst.StageTimes(subscribe=False, clock=clock)
        st.on_statistics(statistics({2: 1.0}))          # received at 100.0, an older plan

        def sleep(s):
            clock.now += s
        self.assertIsNone(st.latest_since(100.5, sleep=sleep))   # waited its 10 ms, nothing new
        self.assertGreaterEqual(clock.now, 100.01)
        clock.now = 101.0   # a message of the plan that started at 100.5
        fresh = statistics({2: 2.0})
        st.on_statistics(fresh)
        self.assertIs(st.latest_since(100.5, sleep=sleep), fresh)

    def test_scene_size_from_the_solution_start_scene(self):
        msg = Solution()
        msg.start_scene.world.octomap.octomap.data = [0] * 1234
        msg.start_scene.world.collision_objects = [CollisionObject(), CollisionObject()]
        msg.start_scene.robot_state.attached_collision_objects = [AttachedCollisionObject()]
        self.assertEqual(mst.scene_size(msg), (1234, 2, 1))
        self.assertEqual(mst.size_text(mst.scene_size(msg)), 'scene: octomap 1234 bytes, 2 world objects, 1 attached')
        self.assertIsNone(mst.scene_size('not a message'))
        self.assertEqual(mst.size_text(None), 'scene size n/a')


class FakeSolution:
    cost = 1.0

    def toMsg(self):
        msg = Solution()
        msg.start_scene.world.octomap.octomap.data = [0] * 10
        return msg


class FakeTask:
    def __init__(self, stage_times=None, stats=None, ok=True):
        self.solutions, self.published, self.stage_times, self.stats, self.ok = [], None, stage_times, stats, ok

    def init(self):
        time.sleep(0.01)

    def plan(self, count):
        time.sleep(0.05)
        if self.stage_times is not None:   # MTC publishes its statistics during the plan
            self.stage_times.on_statistics(self.stats)
        self.solutions = [FakeSolution()] if self.ok else []
        return self.ok

    def publish(self, solution):
        self.published = solution


def planner(stage_times):
    m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
    m.cable_guard, m.cable_solutions, m.start_move = None, 1, None
    m.describe_failures = lambda task: 'none'
    m.first_failure = lambda task: 'failed at grasp'
    m.stage_times = stage_times
    return m


class TestPlanSplit(unittest.TestCase):
    def setUp(self):
        for name, value in (('get_param', lambda name, default=None: default),):
            patcher = mock.patch.object(mpp.rospy, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.logs = []
        patcher = mock.patch.object(mpp.rospy, 'loginfo', self.logs.append)
        patcher.start()
        self.addCleanup(patcher.stop)
        TIMER.begin('pick', 'banana')
        self.addCleanup(lambda: TIMER.action and TIMER.end(False))

    def stage_times(self):
        st = mst.StageTimes(subscribe=False)
        st.on_description(description())
        st.on_statistics(statistics({2: 0.0}))   # an older plan's message: must not be taken
        st.latest = (st.latest[0] - 10.0, st.latest[1])
        return st

    def test_keys_sum_to_plan_and_scene_is_taken_out_of_compute(self):
        st = self.stage_times()
        task = FakeTask(st, statistics({2: 0.03, 3: 0.01, 6: 0.005}))
        self.assertTrue(planner(st).plan(task, 'grasp grasp_0'))
        totals = dict(TIMER.totals)
        for key in ('plan', 'plan/init', 'plan/scene', 'plan/compute', 'plan/finish'):
            self.assertIn(key, totals)
        self.assertAlmostEqual(totals['plan/scene'], 0.03, places=6)
        self.assertGreaterEqual(totals['plan/compute'], 0.015)
        parts = sum(v for k, v in totals.items() if k.startswith('plan/'))
        self.assertLess(abs(totals['plan'] - parts), 0.01)
        split = next(line for line in self.logs if 'plan split' in line)
        self.assertIn('scene fetch 0.03 s', split)
        self.assertIn('slowest stages: current state 0.03 s, move to pregrasp 0.01 s', split)
        planned = next(line for line in self.logs if 'planned in' in line)
        self.assertIn('scene: octomap 10 bytes, 0 world objects, 0 attached', planned)
        self.assertIsNotNone(task.published)

    def test_failed_plan_is_split_too(self):
        st = self.stage_times()
        self.assertFalse(planner(st).plan(FakeTask(st, statistics({2: 0.02}), ok=False), 'place location 1'))
        self.assertAlmostEqual(TIMER.totals['plan/scene'], 0.02, places=6)
        self.assertIn('plan/compute', TIMER.totals)
        self.assertNotIn('plan/finish', TIMER.totals)

    def test_without_statistics_all_of_task_plan_is_compute(self):
        start = time.monotonic()
        self.assertTrue(planner(None).plan(FakeTask(), 'grasp grasp_0'))
        self.assertLess(time.monotonic() - start, 0.2)
        self.assertNotIn('plan/scene', TIMER.totals)
        self.assertGreaterEqual(TIMER.totals['plan/compute'], 0.05)
        split = next(line for line in self.logs if 'plan split' in line)
        self.assertIn('scene fetch n/a (no stage statistics)', split)
        self.assertIn('slowest stages: n/a', split)

    def test_a_broken_split_never_fails_the_plan(self):
        m = planner(None)
        m.time_plan_split = mock.Mock(side_effect=RuntimeError('boom'))
        with mock.patch.object(mpp.rospy, 'logwarn') as warn, mock.patch.object(mpp.rospy, 'logdebug'):
            task = FakeTask()
            self.assertTrue(m.plan(task, 'grasp grasp_0'))
            self.assertFalse(warn.called)
        self.assertIsNotNone(task.published)

    def test_timing_line_carries_the_new_keys(self):
        st = self.stage_times()
        planner(st).plan(FakeTask(st, statistics({2: 0.03})), 'grasp grasp_0')
        line = TIMER.end(True)
        for key in ('plan=', 'plan/init=', 'plan/scene=0.03', 'plan/compute=', 'plan/finish='):
            self.assertIn(key, line)


if __name__ == '__main__':
    unittest.main()
