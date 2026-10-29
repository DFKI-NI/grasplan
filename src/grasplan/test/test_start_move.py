'''~plan_pick_during_start_move (careful, off in code): the closed-set untangle poses are planned and cable-checked
up front (each from the end of the one before, like guarded_named_move), sent as one motion without waiting, and MTC
plans the pick meanwhile from their end (a prefix stage that is never executed and never replayed by the guard); the
plan is used only when the arm arrives within tolerance, else the pick is planned again from the real state. Fake arm,
action client and task; the real CableGuard with the carrier chain replay (fixture of test_cable_chain_guard); no ROS
master; Mobipick image.'''
import itertools
import math
import threading
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import rospy
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from moveit_task_constructor_msgs.msg import Solution, SubTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import grasplan.mtc_pick_place as mpp
import grasplan.pick as pick
from test_cable_chain_guard import ANYGRASP, CATCH, CLEAN, NAMES, TestChainGuard

POSES = ['untangle_cable_guide_1_right', 'untangle_cable_guide_2_right']


def trajectory(*waypoints, seconds=5.0, names=NAMES):
    t = RobotTrajectory(joint_trajectory=JointTrajectory(joint_names=list(names)))
    for i, w in enumerate(waypoints):
        t.joint_trajectory.points.append(JointTrajectoryPoint(positions=[math.radians(v) for v in w],
                                                              time_from_start=rospy.Duration(seconds * i)))
    return t


class FakeArm:
    def __init__(self, plans):
        self.plans, self.executed, self.gos, self.constraints, self.starts = list(plans), [], 0, [], []

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
        return (True, self.plans.pop(0), 0.1, None) if self.plans else (False, RobotTrajectory(), 0.1, None)

    def execute(self, t, wait=True):
        self.executed.append(t)
        return True

    def go(self):
        self.gos += 1
        return True


class FakeClient:
    """execute_trajectory client; codes: one result per goal (the last repeats); gate: an Event each goal waits for"""
    def __init__(self, code=MoveItErrorCodes.SUCCESS, gate=None):
        self.codes = list(code) if isinstance(code, list) else [code]
        self.gate, self.goals, self.waited, self.cancelled = gate, [], 0, False

    def send_goal(self, goal):
        self.goals.append(goal)

    def wait_for_result(self, timeout=None):
        if self.gate is not None:
            self.gate.wait()
        self.waited += 1
        return True

    def cancel_goal(self):
        self.cancelled = True
        if self.gate is not None:
            self.gate.set()

    def get_result(self):
        code = self.codes[min(len(self.goals), len(self.codes)) - 1]
        return None if code is None else NS(error_code=NS(val=code))


class Base(unittest.TestCase):
    params = {}

    def param(self, name, default=None):   # no master
        if 'guard_named_moves' in name:
            return 'enforce'
        if 'rope_stops' in name:   # a chain catch refuses, as in test_guarded_named_move
            return True
        return self.params.get(name, default)

    def setUp(self):
        for module in (mpp, pick):
            patcher = mock.patch.object(module.rospy, 'get_param', self.param)
            patcher.start()
            self.addCleanup(patcher.stop)
        patcher = mock.patch.object(mpp, 'announce_trajectory_start', lambda source: self.announced.append(source))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.announced = []

    @classmethod
    def setUpClass(cls):
        TestChainGuard.setUpClass()
        cls.guard = TestChainGuard.guard
        cls.guard.chain_check = True
        cls.guard.update_params = lambda: None
        mpp._current_joint = lambda joint, timeout=2.0: math.radians(180.0)


class TestStartMove(Base):
    def test_each_pose_is_planned_from_the_end_of_the_one_before_and_checked(self):
        one, two = trajectory(*CLEAN), trajectory(*CLEAN)
        arm = FakeArm([one, two])
        move = mpp.StartMove.plan(arm, POSES, self.guard)
        self.assertIsNotNone(move)
        end_of_one = dict(zip(NAMES, one.joint_trajectory.points[-1].positions))
        self.assertEqual(arm.starts, ['current', end_of_one, 'current'])   # and back to the current state after
        self.assertEqual(move.end_q, dict(zip(NAMES, two.joint_trajectory.points[-1].positions)))
        self.assertEqual(move.names, POSES)
        self.assertEqual(arm.executed, [])                              # planning only
        self.assertEqual(arm.constraints[-1], None)

    def test_a_refused_pose_means_the_old_way(self):
        arm = FakeArm([trajectory(*CLEAN)] + [trajectory(*CATCH)] * 2)
        self.assertIsNone(mpp.StartMove.plan(arm, POSES, self.guard))
        self.assertEqual(arm.executed, [])
        self.assertEqual(arm.starts[-1], 'current')

    def test_sent_at_once_one_goal_and_announcement_per_pose_and_waited_for_later(self):
        one, two = trajectory(*CLEAN), trajectory(*CLEAN)
        move = mpp.StartMove.plan(FakeArm([one, two]), POSES, self.guard)
        gate = threading.Event()
        client = FakeClient(gate=gate)
        move.start(client)
        self.assertEqual(client.waited, 0)   # start() did not wait
        gate.set()
        with mock.patch.object(mpp, '_current_arm_q', lambda timeout=2.0, not_before=None: dict(move.end_q)):
            self.assertEqual(move.wait(), (True, ''))
        # the boundaries the guard checked: each pose its own goal and its own trajectory start, as pose by pose
        self.assertEqual([g.trajectory for g in client.goals], [one, two])
        self.assertEqual(self.announced, [f'named move {name}' for name in POSES])
        self.assertEqual(move.seconds(), 10.0)

    def test_the_guard_checked_exactly_the_executed_segments(self):
        seen, real = [], self.guard

        class Spy:
            def __getattr__(self, name):
                return getattr(real, name)

            def check(self, view, **kw):
                seen.append([sub.trajectory for sub in view.sub_trajectory])
                return real.check(view, **kw)
        guard = Spy()
        one, two = trajectory(*CLEAN), trajectory(*CLEAN)
        move = mpp.StartMove.plan(FakeArm([one, two]), POSES, guard)
        client = FakeClient()
        move.start(client)
        move.join(5.0)
        self.assertEqual(seen, [[one], [two]])
        self.assertEqual([[g.trajectory] for g in client.goals], seen)

    def test_a_failed_pose_stops_the_rest(self):
        move = mpp.StartMove.plan(FakeArm([trajectory(*CLEAN), trajectory(*CLEAN)]), POSES, self.guard)
        client = FakeClient(MoveItErrorCodes.CONTROL_FAILED)
        move.start(client)
        arrived, why = move.wait()
        self.assertFalse(arrived)
        self.assertIn('error code', why)
        self.assertEqual((len(client.goals), self.announced), (1, [f'named move {POSES[0]}']))

    def test_arrival_outside_the_tolerance_or_a_failed_motion_drops_the_plan(self):
        move = mpp.StartMove.plan(FakeArm([trajectory(*CLEAN), trajectory(*CLEAN)]), POSES, self.guard)
        move.start(FakeClient())
        off = {n: v + math.radians(1.0) for n, v in move.end_q.items()}
        with mock.patch.object(mpp, '_current_arm_q', lambda timeout=2.0, not_before=None: off):
            arrived, why = move.wait()
        self.assertFalse(arrived)
        self.assertIn('1.00 deg from the end', why)
        move.start(FakeClient(MoveItErrorCodes.CONTROL_FAILED))
        arrived, why = move.wait()
        self.assertFalse(arrived)
        self.assertIn('failed', why)

    def started(self, client=None):
        move = mpp.StartMove.plan(FakeArm([trajectory(*CLEAN), trajectory(*CLEAN)]), POSES, self.guard)
        move.start(client or FakeClient())
        return move

    def arrival(self, move, now, **kw):
        with mock.patch.object(mpp, '_current_arm_q', lambda timeout=2.0, not_before=None: now):
            return move.wait(**kw)

    def test_partial_or_not_finite_feedback_drops_the_plan(self):
        move = self.started()
        one = {NAMES[0]: move.end_q[NAMES[0]]}
        arrived, why = self.arrival(move, one)
        self.assertFalse(arrived)
        self.assertIn(f'lacks {NAMES[1]}', why)
        for bad in (float('nan'), float('inf')):
            move = self.started()
            arrived, why = self.arrival(move, {n: bad for n in NAMES})
            self.assertFalse(arrived)
            self.assertIn('not finite', why)
        move = self.started()
        self.assertEqual(self.arrival(move, None), (False, 'no fresh joint state after the start move'))

    def test_a_tolerance_that_is_not_finite_negative_or_too_large_drops_the_plan(self):
        for tolerance in (float('nan'), float('inf'), -0.001, 0.2, 'x'):
            move = self.started()
            client = move.client
            far = {n: v + 1.0 for n, v in move.end_q.items()}
            arrived, why = self.arrival(move, far, tolerance=tolerance)
            self.assertFalse(arrived, tolerance)
            self.assertIn('invalid arrival tolerance', why)
            self.assertFalse(move.runner.is_alive())   # the motion still ends before anything is planned again

    def test_a_full_wrist_turn_is_not_an_arrival(self):
        move = self.started()
        turned = dict(move.end_q)
        turned[NAMES[-1]] += 2 * math.pi
        self.assertFalse(self.arrival(move, turned)[0])

    def test_a_motion_that_does_not_end_is_cancelled_and_dropped(self):
        client = FakeClient(gate=threading.Event())   # never ends by itself
        move = self.started(client)
        move.timeout_s = lambda: 0.2
        arrived, why = move.wait()
        self.assertFalse(arrived)
        self.assertIn('did not end', why)
        self.assertTrue(client.cancelled)
        move.join(5.0)
        self.assertEqual(len(client.goals), 1)   # the second pose is not sent after a cancel

    def test_no_result_is_no_arrival(self):
        move = self.started(FakeClient(None))
        arrived, why = move.wait()
        self.assertFalse(arrived)
        self.assertIn('error code None', why)

    def test_the_planned_pick_must_start_where_the_arm_stands(self):
        move = self.started()
        self.assertEqual(self.arrival(move, dict(move.end_q), plan_start=dict(move.end_q)), (True, ''))
        move = self.started()
        off = {n: v + math.radians(1.0) for n, v in move.end_q.items()}
        arrived, why = self.arrival(move, dict(move.end_q), plan_start=off)
        self.assertFalse(arrived)
        self.assertIn('the pick was planned from 1.00 deg off the end of the start move', why)
        move = self.started()
        arrived, why = self.arrival(move, dict(move.end_q), plan_start={NAMES[0]: 0.0})
        self.assertFalse(arrived)
        self.assertIn('the planned pick start lacks', why)

    def test_stale_or_unstamped_joint_states_are_not_an_arrival(self):
        from sensor_msgs.msg import JointState
        stamps = iter([rospy.Time(5), rospy.Time(0), rospy.Time(10)])

        def message(topic, cls, timeout):
            return JointState(header=NS(stamp=next(stamps)), name=list(NAMES), position=[0.0] * 6)
        with mock.patch.object(mpp.rospy, 'wait_for_message', message):
            self.assertEqual(mpp._current_arm_q(not_before=rospy.Time(10)), {n: 0.0 for n in NAMES})
        stamps = itertools.repeat(rospy.Time(5))
        with mock.patch.object(mpp.rospy, 'wait_for_message', message):
            self.assertIsNone(mpp._current_arm_q(timeout=0.01, not_before=rospy.Time(10)))


def solution_msg():
    msg = Solution()
    for points in (0, 3, 4):   # current state, start move stand-in, move to pregrasp
        sub = SubTrajectory()
        sub.trajectory.joint_trajectory.joint_names = list(NAMES)
        sub.trajectory.joint_trajectory.points = [
            JointTrajectoryPoint(positions=[0.1 * i] * 6, time_from_start=rospy.Duration(i)) for i in range(points)]
        msg.sub_trajectory.append(sub)
    return msg


def planner():
    m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
    m.cable_solutions = 1
    m.where_in_task = lambda task, guard: ''
    m.describe_failures = lambda task: 'none'
    return m


class TestMtcPrefix(Base):
    def test_the_guard_never_sees_the_stand_in_path(self):
        seen = []
        m = planner()
        m.cable_guard = NS(max_stretch=-0.115, check=lambda msg: seen.append(msg) or (True, -0.2, 'wrist', {}))
        task = NS(init=lambda: None, plan=lambda count: True, solutions=[NS(toMsg=solution_msg, cost=1.0)],
                  publish=lambda s: None)
        self.assertTrue(m.plan(task, 'grasp g'))
        self.assertEqual([len(s.trajectory.joint_trajectory.points) for s in seen[-1].sub_trajectory], [0, 3, 4])
        m.start_move = NS(end_q={}, names=POSES)
        self.assertTrue(m.plan(task, 'grasp g'))
        self.assertEqual([len(s.trajectory.joint_trajectory.points) for s in seen[-1].sub_trajectory], [0, 0, 4])

    def test_the_task_starts_with_the_stand_in_stage_only_during_a_start_move(self):
        added = []
        m = planner()
        m.task = NS(clear=lambda: added.clear(), add=added.append, name='')
        m.pipeline, m.arm_group = 'pipeline', 'arm'

        class MoveTo:
            def __init__(self, name, planner):
                self.name, self.planner = name, planner

            def setGoal(self, goal):
                self.goal = goal
        with mock.patch.object(mpp.stages, 'CurrentState', lambda name: name), \
                mock.patch.object(mpp.stages, 'MoveTo', MoveTo):
            _, roles = m.start_task('pick')
            self.assertEqual(roles, [mpp.NOOP])
            m.start_move = NS(end_q={}, names=POSES)
            _, roles = m.start_task('pick')
        self.assertEqual(roles, [mpp.NOOP, mpp.PREFIX])
        self.assertEqual((added[1].goal, added[1].planner, added[1].group), (POSES[-1], 'pipeline', 'arm'))

    def test_the_start_state_reads_are_the_start_moves_end(self):
        m = planner()
        with mock.patch.object(mpp, '_current_arm_q', lambda timeout=2.0: {'j': 1.0}):
            self.assertEqual(m.planned_start_q(), {'j': 1.0})
            m.start_move = NS(end_q={'j': 2.0})
            self.assertEqual(m.planned_start_q(), {'j': 2.0})

    def test_execution_skips_the_stand_in_path(self):
        m = planner()
        m.chosen_solution = NS(toMsg=solution_msg)
        m.is_preempt_requested = lambda: False
        ran = []
        m.execute_arm = lambda t: ran.append(len(t.joint_trajectory.points)) or MoveItErrorCodes.SUCCESS
        with mock.patch.object(mpp, 'announce_trajectory_start', lambda source: None):
            code = m.execute(None, [mpp.NOOP, mpp.PREFIX, mpp.ARM], [], None)
        self.assertEqual((code, ran), (MoveItErrorCodes.SUCCESS, [4]))


class FakeMove:
    def __init__(self, arrivals):
        self.arrivals, self.names, self.end_q, self.plan_starts = list(arrivals), POSES, {}, []

    def wait(self, tolerance=None, plan_start=None):
        self.plan_starts.append(plan_start)
        return self.arrivals.pop(0)


def pick_planner(plans):
    '''an MtcPickPlace whose planning and execution are stubs; plans: one bool per planned grasp in call order'''
    m = planner()
    m.plans, m.log = list(plans), []
    m.update_cable_guard = lambda: None
    m.gripper_links = []
    m.is_preempt_requested = lambda: False
    m.make_pick_task = lambda goal, grasp: (f'task {grasp.id}', [mpp.NOOP, mpp.PREFIX, mpp.ARM])
    m.chosen_solution = NS(toMsg=solution_msg)
    m.plan = lambda task, what: m.log.append(('plan', what, m.start_move is not None)) or m.plans.pop(0)
    m.log_failed_branch = lambda grasp: None
    m.next_branch_plan = lambda goal, grasp, what: (None, None, False)
    m.turned_grasp_plan = lambda goal, grasp: (None, None, None)
    m.gripper_holds_object = lambda when: True

    def execute(task, roles, *a, **kw):
        m.log.append(('execute', task, m.start_move is not None))
        return MoveItErrorCodes.SUCCESS
    m.execute = execute
    return m


def goal(*ids):
    return NS(target_name='relay_1', possible_grasps=[NS(id=i, pre_grasp_posture=None, grasp_posture=None) for i in ids])


class TestPickupDuringStartMove(Base):
    def test_planned_meanwhile_then_executed_after_the_arrival(self):
        m = pick_planner([True])
        move = m.start_move = FakeMove([(True, '')])
        result = m.pickup(goal('g0'))
        self.assertEqual(result.error_code.val, MoveItErrorCodes.SUCCESS)
        self.assertEqual(move.plan_starts, [{n: 0.0 for n in NAMES}])   # the first point of the retained arm path
        # planned with the start move running, executed after it ended (the re-grasp tail plans from the real state)
        self.assertEqual(m.log, [('plan', 'grasp g0 (1/1)', True), ('execute', 'task g0', False)])

    def test_an_arrival_off_the_plan_plans_again_from_the_real_state(self):
        m = pick_planner([True, True])
        m.start_move = FakeMove([(False, 'the arm stopped 1.00 deg from the end of the start move')])
        self.assertEqual(m.pickup(goal('g0')).error_code.val, MoveItErrorCodes.SUCCESS)
        self.assertEqual(m.log, [('plan', 'grasp g0 (1/1)', True), ('plan', 'grasp g0 (1/1)', False),
                                 ('execute', 'task g0', False)])

    def test_nothing_planned_meanwhile_waits_and_tries_the_old_way(self):
        # the stand-in path may fail where the real start would not: the old way decides, after the arm arrived
        m = pick_planner([False, False, True])
        move = m.start_move = FakeMove([(True, '')])
        self.assertEqual(m.pickup(goal('g0', 'g1')).error_code.val, MoveItErrorCodes.SUCCESS)
        self.assertEqual(move.arrivals, [])
        self.assertEqual([entry[2] for entry in m.log], [True, True, False, False])

    def test_without_a_start_move_nothing_changes(self):
        m = pick_planner([False, True])
        self.assertEqual(m.pickup(goal('g0', 'g1')).error_code.val, MoveItErrorCodes.SUCCESS)
        self.assertEqual([entry[0] for entry in m.log], ['plan', 'plan', 'execute'])


class TestPickToolsStartMove(Base):
    def picker(self):
        p = pick.PickTools.__new__(pick.PickTools)
        p.robot = NS(arm=FakeArm([trajectory(*CLEAN), trajectory(*CLEAN)]))
        p.mtc = NS(update_cable_guard=lambda: None, cable_guard=self.guard, execute_client=FakeClient(), start_move=None)
        return p

    def test_off_by_default_the_old_way(self):
        p = self.picker()
        self.assertFalse(p.start_move_while_planning(POSES))
        self.assertIsNone(p.mtc.start_move)
        self.assertEqual(p.mtc.execute_client.goals, [])

    def test_on_the_motion_is_sent_and_handed_to_mtc(self):
        self.params = {'~plan_pick_during_start_move': True}
        try:
            p = self.picker()
            self.assertTrue(p.start_move_while_planning(POSES))
            self.assertEqual(p.mtc.start_move.names, POSES)
            self.assertEqual(len(p.mtc.execute_client.goals), 2)   # one per pose
        finally:
            self.params = {}

    def test_on_without_mtc_the_old_way(self):
        self.params = {'~plan_pick_during_start_move': True}
        try:
            p = self.picker()
            p.mtc = None
            self.assertFalse(p.start_move_while_planning(POSES))
        finally:
            self.params = {}


if __name__ == '__main__':
    unittest.main()
