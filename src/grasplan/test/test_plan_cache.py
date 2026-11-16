'''~plan_cache_named_moves (careful, off in code): a named move from (almost) the start of a plan the cable guard
passed before, under the same guard settings, runs that plan again with its verdict once its path is collision free
in the current scene; anything else plans and checks as before. The pure cache (tools/plan_cache.py), then the glue with
a fake arm and the real CableGuard with the carrier chain replay (fixture of test_cable_chain_guard); no ROS master;
Mobipick image.'''
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import grasplan.mtc_pick_place as mpp
from grasplan.tools.plan_cache import MAX_TOLERANCE_RAD, PlanCache, check_states, start_offset
from test_cable_chain_guard import ANYGRASP, CATCH, CLEAN, NAMES, TestChainGuard

POSES = ['untangle_cable_guide_1_right', 'untangle_cable_guide_2_right']


def trajectory(*waypoints, seconds=5.0):
    t = RobotTrajectory(joint_trajectory=JointTrajectory(joint_names=NAMES))
    for i, w in enumerate(waypoints):
        t.joint_trajectory.points.append(JointTrajectoryPoint(positions=[math.radians(v) for v in w],
                                                              time_from_start=rospy.Duration(seconds * i)))
    return t


def q_of(waypoint, offset=0.0):
    return {n: math.radians(v) + offset for n, v in zip(NAMES, waypoint)}


class TestPlanCache(unittest.TestCase):
    def test_start_offset_is_the_largest_joint_difference_and_inf_without_a_joint(self):
        self.assertAlmostEqual(start_offset({'a': 0.0, 'b': 1.0}, {'a': 0.002, 'b': 0.995, 'c': 9.0}), 0.005)
        self.assertEqual(start_offset({'a': 0.0, 'b': 1.0}, {'a': 0.0}), float('inf'))
        self.assertEqual(start_offset({'a': 0.0}, {'a': float('nan')}), float('inf'))
        self.assertAlmostEqual(start_offset({'a': -math.pi}, {'a': math.pi}), 2 * math.pi)   # not wrapped

    def test_nearest_plan_within_tolerance_hits_and_counts(self):
        cache = PlanCache()
        cache.store('k', {'a': 0.0}, 'near')
        cache.store('k', {'a': 0.004}, 'nearer')
        self.assertEqual(cache.lookup('k', {'a': 0.003}, 0.005), ('nearer', mock.ANY))
        value, offset = cache.lookup('k', {'a': 0.02}, 0.005)
        self.assertIsNone(value)
        self.assertAlmostEqual(offset, 0.016)
        self.assertEqual(cache.lookup('other', {'a': 0.0}, 0.005), (None, float('inf')))
        self.assertEqual(cache.counts(), '1 hits, 2 misses')

    def test_per_key_and_key_limits_drop_the_oldest(self):
        cache = PlanCache(per_key=2, max_keys=2)
        for i in range(3):
            cache.store('k', {'a': float(i)}, i)
        self.assertEqual([v for _, v in cache.entries['k']], [1, 2])
        cache.store('l', {'a': 0.0}, 'l')
        cache.lookup('k', {'a': 1.0}, 0.0)   # k used last
        cache.store('m', {'a': 0.0}, 'm')
        self.assertEqual(list(cache.entries), ['k', 'm'])

    def test_drop_forgets_one_plan(self):
        cache = PlanCache()
        a, b = object(), object()
        cache.store('k', {'a': 0.0}, a)
        cache.store('k', {'a': 0.0}, b)
        cache.drop('k', b)
        self.assertIs(cache.lookup('k', {'a': 0.0}, 0.0)[0], a)
        cache.drop('k', a)
        self.assertNotIn('k', cache.entries)

    def test_check_states_are_at_most_one_step_apart(self):
        states = check_states([[0.0, 0.0], [0.12, -0.01], [0.12, -0.01]], 0.05)
        self.assertEqual(states[0], [0.0, 0.0])
        self.assertEqual(states[-1], [0.12, -0.01])
        for a, b in zip(states, states[1:]):
            self.assertLessEqual(max(abs(x - y) for x, y in zip(a, b)), 0.05 + 1e-12)
        self.assertEqual(len(states), 1 + 3 + 1)   # start, 0.12 in three steps, the repeated end once
        self.assertEqual(check_states([], 0.05), [])

    def test_the_tolerance_limit_is_moveits_start_tolerance(self):
        self.assertEqual(MAX_TOLERANCE_RAD, 0.01)


class FakeArm:
    def __init__(self, plans):
        self.plans, self.executed, self.planned, self.constraints, self.starts = list(plans), [], 0, [], []

    def get_name(self):
        return 'arm'

    def set_named_target(self, name):
        self.target = name

    def set_path_constraints(self, c):
        self.constraints.append(c)

    def clear_path_constraints(self):
        self.constraints.append(None)

    def set_start_state_to_current_state(self):
        self.starts.append('current')

    def set_start_state(self, state):
        self.starts.append(dict(zip(state.joint_state.name, state.joint_state.position)))

    def plan(self):
        self.planned += 1
        return (True, self.plans.pop(0), 0.1, None) if self.plans else (False, RobotTrajectory(), 0.1, None)

    def execute(self, t, wait=True):
        self.executed.append(t)
        return True

    def go(self):
        return True


class Glue(unittest.TestCase):
    params = {}

    def param(self, name, default=None):   # no master
        name = name.lstrip('~')
        if name in self.params:
            return self.params[name]
        if name == 'mtc_cable_guard_named_moves':
            return 'enforce'
        return default

    def setUp(self):
        self.params = {'plan_cache_named_moves': True}
        self.now = q_of(CLEAN[0])
        self.free = (True, '3 states collision free')
        for target, value in ((mpp.rospy, ('get_param', self.param)),
                              (mpp, ('_current_arm_q', lambda timeout=2.0, not_before=None: self.now)),
                              (mpp, ('_path_collision_free', lambda arm, t: self.free)),
                              (mpp, ('announce_trajectory_start', lambda source: None)),
                              (mpp, ('_named_plan_cache', None))):
            patcher = mock.patch.object(target, *value)
            patcher.start()
            self.addCleanup(patcher.stop)
        patcher = mock.patch.object(self.guard, 'check', wraps=self.guard.check)
        self.check = patcher.start()
        self.addCleanup(patcher.stop)

    @classmethod
    def setUpClass(cls):
        TestChainGuard.setUpClass()
        cls.guard = TestChainGuard.guard
        cls.guard.chain_check = True
        cls.guard.update_params = lambda: None
        mpp._current_joint = lambda joint, timeout=2.0: math.radians(180.0)


class TestNamedMoves(Glue):
    def test_off_in_code_nothing_is_cached_nor_read(self):
        self.params = {}
        self.now = None
        with mock.patch.object(mpp, '_current_arm_q', side_effect=AssertionError('read')):
            for _ in range(2):
                self.assertTrue(mpp.guarded_named_move(FakeArm([trajectory(*CLEAN)]), 'anygrasp', self.guard))
        self.assertEqual(self.check.call_count, 2)
        self.assertIsNone(mpp._named_plan_cache)

    def test_the_same_start_runs_the_checked_plan_again_without_planning_or_replay(self):
        clean = trajectory(*CLEAN)
        self.assertTrue(mpp.guarded_named_move(FakeArm([clean]), 'anygrasp', self.guard))
        self.assertEqual(self.check.call_count, 1)
        arm = FakeArm([])
        self.now = q_of(CLEAN[0], 0.004)   # within the 5 mrad default
        report = {}
        with mock.patch.object(mpp.rospy, 'loginfo') as info:
            self.assertTrue(mpp.guarded_named_move(arm, 'anygrasp', self.guard, report=report))
        self.assertEqual(arm.executed, [clean])
        self.assertEqual((arm.planned, self.check.call_count, arm.constraints), (0, 1, []))
        self.assertEqual(report['plans'][0][:2], ('anygrasp', 0))
        self.assertTrue(any('plan cache hit 4.0 mrad' in str(c) and 'cable check passed' in str(c)
                            for c in info.call_args_list))

    def test_a_start_beyond_the_tolerance_plans_and_checks_again(self):
        self.assertTrue(mpp.guarded_named_move(FakeArm([trajectory(*CLEAN)]), 'anygrasp', self.guard))
        self.now = q_of(CLEAN[0], 0.006)
        arm = FakeArm([trajectory(*CLEAN)])
        self.assertTrue(mpp.guarded_named_move(arm, 'anygrasp', self.guard))
        self.assertEqual((arm.planned, self.check.call_count), (1, 2))

    def test_a_cached_path_that_collides_now_is_dropped_and_planned_again(self):
        self.assertTrue(mpp.guarded_named_move(FakeArm([trajectory(*CLEAN)]), 'anygrasp', self.guard))
        self.free = (False, 'state 2/3 is in collision (table_1-mobipick/ur5_wrist_1_link)')
        fresh = trajectory(*CLEAN)
        arm = FakeArm([fresh])
        self.assertTrue(mpp.guarded_named_move(arm, 'anygrasp', self.guard))
        self.assertEqual((arm.executed, arm.planned, self.check.call_count), ([fresh], 1, 2))
        self.free = (True, 'ok')
        arm = FakeArm([])
        self.assertTrue(mpp.guarded_named_move(arm, 'anygrasp', self.guard))
        self.assertIs(arm.executed[0], fresh)   # the dropped one is gone, the fresh one was stored

    def test_a_refused_plan_is_never_cached(self):
        # a chain stop refuses only with ~mtc_cable_named_move_carrier_stops; by default it is logged and the path passes,
        # with or without the cache
        self.params['mtc_cable_named_move_carrier_stops'] = True
        catch, clean = trajectory(*CATCH), trajectory(*CLEAN)
        self.assertTrue(mpp.guarded_named_move(FakeArm([catch, clean]), 'anygrasp', self.guard))
        entries = [v[0] for plans in mpp.named_plan_cache().entries.values() for _, v in plans]
        self.assertEqual(entries, [clean])

    def test_other_guard_settings_or_another_pose_miss(self):
        self.assertTrue(mpp.guarded_named_move(FakeArm([trajectory(*CLEAN)]), 'anygrasp', self.guard))
        for kwargs, name in (({'chain_stops': True}, 'anygrasp'), ({}, 'transport')):
            arm = FakeArm([trajectory(*CLEAN)])
            self.assertTrue(mpp.guarded_named_move(arm, name, self.guard, **kwargs))
            self.assertEqual(arm.planned, 1)

    def test_a_tolerance_above_moveits_start_tolerance_is_refused(self):
        self.params['plan_cache_start_tolerance_rad'] = 0.02
        self.assertTrue(mpp.guarded_named_move(FakeArm([trajectory(*CLEAN)]), 'anygrasp', self.guard))
        arm = FakeArm([trajectory(*CLEAN)])
        self.assertTrue(mpp.guarded_named_move(arm, 'anygrasp', self.guard))
        self.assertEqual(arm.planned, 1)

    def test_log_mode_reuses_only_plans_the_guard_passed(self):
        self.params['mtc_cable_guard_named_moves'] = 'log'
        catch = trajectory(*CATCH)
        self.assertTrue(mpp.guarded_named_move(FakeArm([catch]), 'anygrasp', self.guard))   # moves, would refuse
        arm = FakeArm([trajectory(*CLEAN)])
        self.assertTrue(mpp.guarded_named_move(arm, 'anygrasp', self.guard))
        self.assertEqual(arm.planned, 1)
        arm = FakeArm([])
        with mock.patch.object(mpp.rospy, 'loginfo') as info:
            self.assertTrue(mpp.guarded_named_move(arm, 'anygrasp', self.guard))
        self.assertEqual(arm.planned, 0)
        self.assertTrue(any('log only, plan cache hit' in str(c) for c in info.call_args_list))


class TestStartMove(Glue):
    def test_both_untangle_legs_hit_the_second_time(self):
        there, back = trajectory(*CLEAN), trajectory(ANYGRASP, CLEAN[0])
        first = mpp.StartMove.plan(FakeArm([there, back]), POSES, self.guard)
        self.assertIsNotNone(first)
        calls = self.check.call_count
        arm = FakeArm([])
        second = mpp.StartMove.plan(arm, POSES, self.guard)
        self.assertEqual((arm.planned, self.check.call_count), (0, calls))
        self.assertEqual(second.trajectories, [there, back])
        self.assertEqual(second.end_q, first.end_q)
        self.assertEqual(arm.starts[-1], 'current')


class TestPlaceNode(Glue):
    '''the place node's named moves (the untangle detour before a place; the insert node runs them through its own
    PlaceTools) take the same cache when that node has the flag'''
    def place_tools(self, plans):
        import grasplan.place as place
        tools = place.PlaceTools.__new__(place.PlaceTools)
        tools.robot = NS(arm=FakeArm(plans))
        tools.mtc = NS(cable_guard=self.guard, update_cable_guard=lambda: None)
        return tools

    def test_the_second_detour_leg_from_the_same_start_reuses_the_checked_plan(self):
        clean = trajectory(*CLEAN)
        self.assertTrue(self.place_tools([clean]).move_arm_to_posture(POSES[0]))
        tools = self.place_tools([])
        self.now = q_of(CLEAN[0], 0.003)
        self.assertTrue(tools.move_arm_to_posture(POSES[0]))
        self.assertEqual((tools.robot.arm.executed, tools.robot.arm.planned, self.check.call_count), ([clean], 0, 1))
        self.assertEqual(tools.arm_move_report['plans'][0][:2], (POSES[0], 0))

    def test_without_the_flag_the_place_node_plans_every_move(self):
        self.params = {}
        for _ in range(2):
            tools = self.place_tools([trajectory(*CLEAN)])
            self.assertTrue(tools.move_arm_to_posture(POSES[0]))
            self.assertEqual(tools.robot.arm.planned, 1)
        self.assertEqual(self.check.call_count, 2)
        self.assertIsNone(mpp._named_plan_cache)


class TestCollisionCheck(unittest.TestCase):
    def setUp(self):
        self.requests, self.invalid_at = [], None

        def service(request):
            self.requests.append(request)
            bad = self.invalid_at is not None and len(self.requests) == self.invalid_at
            return NS(valid=not bad, contacts=[NS(contact_body_1='table_1', contact_body_2='mobipick/ur5_wrist_1_link')]
                      if bad else [])

        for target, value in ((mpp.rospy, ('get_param', lambda name, default=None: default)),
                              (mpp.rospy, ('wait_for_service', lambda name, timeout=None: None)),
                              (mpp.rospy, ('ServiceProxy', lambda name, srv, persistent=False: service)),
                              (mpp, ('_validity_service', None))):
            patcher = mock.patch.object(target, *value)
            patcher.start()
            self.addCleanup(patcher.stop)

    def test_every_state_along_the_path_is_checked_in_the_current_scene(self):
        t = trajectory([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 11.4])   # 0.2 rad on wrist_3: 4 steps of 0.05
        ok, why = mpp._path_collision_free(FakeArm([]), t)
        self.assertEqual((ok, why), (True, '5 states collision free'))
        self.assertEqual({r.group_name for r in self.requests}, {'arm'})
        self.assertTrue(all(r.robot_state.is_diff and list(r.robot_state.joint_state.name) == NAMES
                            for r in self.requests))

    def test_a_colliding_state_fails_with_its_contact(self):
        self.invalid_at = 2
        ok, why = mpp._path_collision_free(FakeArm([]), trajectory([0] * 6, [0, 0, 0, 0, 0, 11.4]))
        self.assertFalse(ok)
        self.assertEqual(why, 'state 2/5 is in collision (table_1-mobipick/ur5_wrist_1_link)')

    def test_no_service_means_no_reuse(self):
        def missing(name, timeout=None):
            raise rospy.ROSException('timeout exceeded while waiting for service')
        with mock.patch.object(mpp.rospy, 'wait_for_service', missing):
            ok, why = mpp._path_collision_free(FakeArm([]), trajectory([0] * 6, [0] * 6))
        self.assertFalse(ok)
        self.assertIn('collision check unavailable', why)


if __name__ == '__main__':
    unittest.main()
