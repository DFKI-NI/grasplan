'''~plan_transport_during_pick (careful, off in code): the move to the transport pose after a closed-set pick is planned
in a background thread from where the MTC pick solution ends (object attached), with the planning and cable guard of a
named move in its current mode and without the named-move plan cache, while the pick executes; after the pick it runs
when the arm stands at its start, else the guarded named move from the real state; a failure leaves the pick
successful and says why. Fake arms, the real CableGuard with the carrier chain replay (fixture of
test_cable_chain_guard); no ROS master; Mobipick image.'''
import math
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, RobotState
from moveit_task_constructor_msgs.msg import Solution, SubTrajectory
from sensor_msgs.msg import JointState

import grasplan.mtc_pick_place as mpp
import grasplan.pick as pick
from grasplan.tools.stage_timing import StageTimer
from test_cable_chain_guard import CATCH, CLEAN, NAMES, TestChainGuard
from test_start_move import FakeArm, trajectory

TARGET = 'relay_1'


def solution(end_waypoint, attached=TARGET, arm_motion=True):
    '''an MTC pick solution: an arm motion ending at end_waypoint (deg), then the attach (scene with the object)'''
    move = SubTrajectory()
    if arm_motion:
        move.trajectory = trajectory(CLEAN[0], end_waypoint)
    attach = SubTrajectory()
    state = RobotState(joint_state=JointState(name=list(NAMES) + ['gripper_joint'],
                                              position=[0.0] * len(NAMES) + [0.4]), is_diff=True)
    if attached:
        aco = AttachedCollisionObject(link_name='gripper_tcp')
        aco.object = CollisionObject(id=attached, operation=CollisionObject.APPEND)
        state.attached_collision_objects.append(aco)
    attach.scene_diff = PlanningScene(robot_state=state, is_diff=True)
    return Solution(sub_trajectory=[move, attach])


class Params:
    def __init__(self, **values):
        self.values = values

    def __call__(self, name, default=None):
        for key, value in self.values.items():
            if name.endswith(key):
                return value
        if name == '/use_sim_time':
            return True
        return default


class TransportMoveTest(unittest.TestCase):
    mode = 'enforce'

    @classmethod
    def setUpClass(cls):
        TestChainGuard.setUpClass()
        cls.guard = TestChainGuard.guard
        cls.guard.chain_check = True
        cls.guard.update_params = lambda: None
        mpp._current_joint = lambda joint, timeout=2.0: math.radians(180.0)

    def setUp(self):
        self.params = Params(mtc_cable_guard_named_moves=self.mode, mtc_cable_named_move_carrier_stops=True,
                             plan_cache_named_moves=True, plan_pick_during_start_move_tolerance_rad=0.01)
        for target, value in ((mpp.rospy, 'get_param'),):
            patcher = mock.patch.object(target, value, self.params)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.announced = []
        for name, value in (('announce_trajectory_start', self.announced.append),
                            ('named_plan_cache', mock.Mock(side_effect=AssertionError('plan cache used'))),
                            ('_ros_now', lambda: None)):
            patcher = mock.patch.object(mpp, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        mpp.TransportMove._running = None

    def begin(self, plans, end=CLEAN[-1], **kwargs):
        self.planner = FakeArm(plans)
        with mock.patch.object(mpp, '_transport_planning_commander', lambda group: self.planner):
            move = mpp.TransportMove.begin('arm', solution(end, **kwargs), TARGET, self.guard)
        if move is not None:
            move.runner.join(120.0)   # the chain replay of a refused plan takes seconds on one CPU
            self.assertFalse(move.runner.is_alive())
        return move

    def finish(self, move, at, fallback=True):
        arm = FakeArm([])
        q = dict(zip(NAMES, [math.radians(v) for v in at]))
        with mock.patch.object(mpp, '_current_arm_q', lambda **kw: q), \
                mock.patch.object(mpp, 'guarded_named_move', mock.Mock(return_value=fallback)) as named:
            result = move.finish(arm, self.guard)
        return result, arm, named


class EndState(unittest.TestCase):
    def test_the_start_state_carries_the_object_and_the_arm_at_the_end(self):
        state, q = mpp.solution_end_state(solution(CLEAN[-1]), TARGET)
        self.assertTrue(state.is_diff)
        self.assertEqual([a.object.id for a in state.attached_collision_objects], [TARGET])
        self.assertEqual(state.attached_collision_objects[0].object.operation, CollisionObject.ADD)
        for name, value in zip(NAMES, CLEAN[-1]):
            self.assertAlmostEqual(q[name], math.radians(value))
            self.assertAlmostEqual(state.joint_state.position[state.joint_state.name.index(name)], math.radians(value))
        self.assertIn('gripper_joint', state.joint_state.name)   # the rest of the end state stays

    def test_no_attached_object_or_no_arm_motion_gives_no_state(self):
        state, why = mpp.solution_end_state(solution(CLEAN[-1], attached=''), TARGET)
        self.assertIsNone(state)
        self.assertIn('not attached', why)
        state, why = mpp.solution_end_state(solution(CLEAN[-1], arm_motion=False), TARGET)
        self.assertIsNone(state)
        self.assertIn('no arm motion', why)


class Enforce(TransportMoveTest):
    def test_planned_from_the_end_of_the_pick_without_the_plan_cache(self):
        clean = trajectory(*CLEAN)
        move = self.begin([clean])
        self.assertIs(move.trajectory, clean)
        self.assertEqual(self.planner.target, 'transport')
        start = self.planner.starts[0]
        for name, value in zip(NAMES, CLEAN[-1]):
            self.assertAlmostEqual(start[name], math.radians(value))
        self.assertEqual(self.planner.starts[-1], 'current')   # the planning commander is left at the current state

    def test_a_plan_the_guard_refuses_is_replanned_then_given_up(self):
        catch, clean = trajectory(*CATCH), trajectory(*CLEAN)
        self.assertIs(self.begin([catch, clean]).trajectory, clean)
        move = self.begin([catch, catch])
        self.assertIsNone(move.trajectory)
        self.assertIn('cable guard', move.reason)

    def test_runs_the_plan_when_the_arm_ends_at_its_start(self):
        clean = trajectory(*CLEAN)
        move = self.begin([clean])
        (moved, how), arm, named = self.finish(move, CLEAN[0])
        self.assertTrue(moved)
        self.assertEqual(arm.executed, [clean])
        self.assertEqual(self.announced, ['named move transport'])
        named.assert_not_called()
        self.assertIn('plan made during the pick', how)

    def test_off_its_start_the_guarded_named_move_plans_again(self):
        move = self.begin([trajectory(*CLEAN)])
        off = [v + 1.0 for v in CLEAN[0]]   # 1 deg > 0.01 rad
        (moved, how), arm, named = self.finish(move, off)
        self.assertTrue(moved)
        self.assertEqual(arm.executed, [])
        named.assert_called_once()
        self.assertEqual(named.call_args[0][1:], ('transport', self.guard))
        self.assertIn('deg from the planned start', how)

    def test_no_plan_falls_back_and_a_failed_fallback_says_why(self):
        move = self.begin([trajectory(*CATCH)] * 2)
        (moved, how), arm, named = self.finish(move, CLEAN[0], fallback=False)
        self.assertFalse(moved)
        self.assertEqual(arm.executed, [])
        self.assertIn('did not move to transport', how)
        self.assertIn('cable guard', how)

    def test_an_invalid_tolerance_never_runs_the_plan(self):
        self.params.values['plan_pick_during_start_move_tolerance_rad'] = 0.5
        move = self.begin([trajectory(*CLEAN)])
        (moved, how), arm, named = self.finish(move, CLEAN[0])
        self.assertEqual(arm.executed, [])
        named.assert_called_once()
        self.assertIn('invalid arrival tolerance', how)

    def test_no_object_attached_at_the_end_no_background_plan(self):
        self.assertIsNone(self.begin([trajectory(*CLEAN)], attached=''))
        self.assertEqual(self.planner.starts, [])

    def test_one_planning_at_a_time(self):
        gate = threading.Event()
        mpp.TransportMove._running = threading.Thread(target=gate.wait)
        mpp.TransportMove._running.start()
        try:
            self.assertIsNone(self.begin([trajectory(*CLEAN)]))
        finally:
            gate.set()


class LogMode(TransportMoveTest):
    mode = 'log'

    def test_the_guard_only_logs(self):
        catch = trajectory(*CATCH)
        self.assertIs(self.begin([catch]).trajectory, catch)


class Flag(unittest.TestCase):
    def test_off_in_code(self):
        with mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: default):
            self.assertFalse(mpp.plan_transport_during_pick())
            self.assertEqual(mpp.transport_during_pick_pose(), 'transport')


class PickSide(unittest.TestCase):
    def tool(self, move, preempt=False):
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc = NS(transport_move=move, cable_guard='guard', update_cable_guard=lambda: None)
        p.robot = NS(arm='arm')
        p.pick_note = ''
        p.pick_action_server = NS(is_preempt_requested=lambda: preempt)
        return p

    def test_nothing_without_a_planned_move(self):
        p = self.tool(None)
        p.transport_after_pick(True)
        self.assertEqual(p.pick_note, '')

    def test_moved(self):
        move = mock.Mock()
        move.finish.return_value = (True, 'moved to transport with the plan made during the pick')
        p = self.tool(move)
        p.transport_after_pick(True)
        move.finish.assert_called_once_with('arm', 'guard')
        self.assertIsNone(p.mtc.transport_move)
        self.assertEqual(p.pick_note, '')

    def test_a_failed_move_goes_into_the_status_only(self):
        move = mock.Mock()
        move.finish.return_value = (False, 'the arm did not move to transport: x')
        p = self.tool(move)
        p.transport_after_pick(True)
        self.assertEqual(p.pick_note, 'the arm did not move to transport: x')

    def test_dropped_after_a_failed_or_cancelled_pick(self):
        for success, preempt in ((False, False), (True, True)):
            move = mock.Mock()
            p = self.tool(move, preempt)
            p.transport_after_pick(success)
            move.finish.assert_not_called()
            self.assertIsNone(p.mtc.transport_move)


class LiftCheck(unittest.TestCase):
    '''with a move planned during the pick, the check after the lift runs while the arm moves to transport'''
    def mtc(self, move, holds=True, preempt=False, hold_check=None):
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.transport_move, m.transport_arm, m.transport_done = move, 'arm', None
        m.is_preempt_requested = lambda: preempt
        m.update_cable_guard = lambda: None
        m.cable_guard = 'guard'
        self.checks = []

        def check(when):
            self.checks.append(when)
            return hold_check() if hold_check else holds
        m.gripper_holds_object = check
        return m

    def test_without_a_planned_move_only_the_check(self):
        m = self.mtc(None)
        self.assertTrue(m.lift_check())
        self.assertEqual(self.checks, ['after the lift'])
        self.assertIsNone(m.transport_done)

    def test_the_check_and_the_move_overlap(self):
        checking, moving = threading.Event(), threading.Event()

        def hold_check():
            checking.set()
            return moving.wait(5.0)   # True only when the move runs at the same time

        def finish(arm, guard):
            moving.set()
            self.assertTrue(checking.wait(5.0))
            return True, 'moved to transport with the plan made during the pick'
        move = mock.Mock(finish=mock.Mock(side_effect=finish))
        m = self.mtc(move, hold_check=hold_check)
        self.assertTrue(m.lift_check())
        move.finish.assert_called_once_with('arm', 'guard')
        self.assertEqual(m.transport_done, (True, 'moved to transport with the plan made during the pick'))
        self.assertIsNone(m.transport_move)

    def test_a_lost_object_fails_the_check_and_leaves_the_arm_at_transport(self):
        move = mock.Mock(finish=mock.Mock(return_value=(True, 'moved to transport with the plan made during the pick')))
        m = self.mtc(move, holds=False)
        with mock.patch.object(mpp.rospy, 'logwarn') as warn:
            self.assertFalse(m.lift_check())
        move.finish.assert_called_once()
        self.assertTrue(m.transport_done[0])
        self.assertIn('lost during the lift; the arm stands at transport', warn.call_args[0][0])

    def test_a_cancel_keeps_the_old_order(self):
        move = mock.Mock()
        m = self.mtc(move, preempt=True)
        self.assertTrue(m.lift_check())
        move.finish.assert_not_called()

    def test_an_error_of_the_check_reaches_the_pick(self):
        move = mock.Mock(finish=mock.Mock(return_value=(True, 'x')))

        def broken():
            raise RuntimeError('no gripper fact')
        m = self.mtc(move, hold_check=broken)
        with self.assertRaises(RuntimeError):
            m.lift_check()


    def test_an_error_of_the_move_waits_for_the_check(self):
        done = threading.Event()

        def hold_check():
            time.sleep(0.2)
            done.set()
            return True
        move = mock.Mock(finish=mock.Mock(side_effect=RuntimeError('controller gone')))
        m = self.mtc(move, hold_check=hold_check)
        with self.assertRaises(RuntimeError):
            m.lift_check()
        self.assertTrue(done.is_set())   # the check has ended when the error reaches the pick

class PickSideAfterLiftCheck(PickSide):
    def test_a_move_run_during_the_check_is_only_reported(self):
        p = self.tool(None)
        p.mtc.transport_done = (False, 'the arm did not move to transport: x')
        p.transport_after_pick(True)
        self.assertEqual(p.pick_note, 'the arm did not move to transport: x')
        self.assertIsNone(p.mtc.transport_done)

    def test_after_a_failed_pick_nothing_is_reported(self):
        p = self.tool(None)
        p.mtc.transport_done = (True, 'moved')
        p.transport_after_pick(False)
        self.assertEqual(p.pick_note, '')


class TimerThreads(unittest.TestCase):
    def test_a_stage_of_another_thread_is_ignored(self):
        lines = []
        timer = StageTimer(log=lines.append)
        timer.begin('pick', 'relay')
        with timer.stage('execute'):
            t = threading.Thread(target=lambda: [timer.stage('named_plan').__enter__()])
            t.start()
            t.join()
            with timer.stage('arm_move'):
                pass
        line = timer.end(True)
        self.assertIn('execute/arm_move=', line)
        self.assertNotIn('named_plan', line)


if __name__ == '__main__':
    unittest.main()
