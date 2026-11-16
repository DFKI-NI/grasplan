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

from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, MoveItErrorCodes, PlanningScene,
                             PlanningSceneComponents, RobotState)
from moveit_task_constructor_msgs.msg import Solution, SubTrajectory
from sensor_msgs.msg import JointState

import grasplan.mtc_pick_place as mpp
import grasplan.pick as pick
from grasplan.tools.stage_timing import StageTimer
from test_cable_chain_guard import CATCH, CLEAN, NAMES, TestChainGuard
from test_start_move import FakeArm, trajectory

TARGET = 'relay_1'


def solution(end_waypoint, attached=TARGET, arm_motion=True, collisions_only_after=0):
    '''an MTC pick solution: an arm motion ending at end_waypoint (deg), then the attach (scene with the object), then
    collisions_only_after stages that only change the allowed collisions (scene diff without a robot state, as after a
    lift that tolerated a touching neighbour)'''
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
    collisions_only = [SubTrajectory(scene_diff=PlanningScene(is_diff=True)) for _ in range(collisions_only_after)]
    return Solution(sub_trajectory=[move, attach] + collisions_only)


class FakeScene:
    '''get_planning_scene: one (attached, in_world) of TARGET per call, the last one repeats; error: raised instead'''
    def __init__(self, states, error=None):
        self.states, self.error, self.requests = list(states), error, []

    def __call__(self, request):
        self.requests.append(request)
        if self.error:
            raise self.error
        attached, in_world = self.states.pop(0) if len(self.states) > 1 else self.states[0]
        scene = PlanningScene()
        if attached:
            scene.robot_state.attached_collision_objects.append(
                AttachedCollisionObject(link_name='gripper_tcp', object=CollisionObject(id=TARGET)))
        scene.world.collision_objects.append(CollisionObject(id='table_1'))
        if in_world:
            scene.world.collision_objects.append(CollisionObject(id=TARGET))
        return NS(scene=scene)


class FakeValidity:
    '''check_state_validity: valid or not with contact pairs; error: raised instead'''
    def __init__(self, valid=True, contacts=(), error=None):
        self.valid, self.contacts, self.error, self.requests = valid, list(contacts), error, []

    def __call__(self, request):
        self.requests.append(request)
        if self.error:
            raise self.error
        return NS(valid=self.valid, contacts=[NS(contact_body_1=a, contact_body_2=b) for a, b in self.contacts])


class PlanArm(FakeArm):
    def get_name(self):
        return 'arm'


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
        self.scene = FakeScene([(True, False)])   # after the attach: attached, no world copy
        self.validity = FakeValidity()

    def begin(self, plans, end=CLEAN[-1], join=True, attach=True, **kwargs):
        '''TransportMove.begin; attach: the pick thread reports its attach right away'''
        self.planner = PlanArm(plans)
        with mock.patch.object(mpp, '_transport_planning_commander', lambda group: self.planner), \
                mock.patch.object(mpp, '_transport_scene', lambda: self.scene), \
                mock.patch.object(mpp, '_transport_validity', lambda: self.validity):
            move = mpp.TransportMove.begin('arm', solution(end, **kwargs), TARGET, self.guard)
        if move is not None and attach:
            move.attached()
        if move is not None and join:
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

    def test_stages_after_the_attach_without_a_robot_state_are_skipped(self):
        '''a lift that tolerated a touching neighbour ends with a stage that only changes the allowed collisions: its
        scene diff has no robot state, so the end state comes from the last scene that has one'''
        state, q = mpp.solution_end_state(solution(CLEAN[-1], collisions_only_after=2), TARGET)
        self.assertIsNotNone(state, q)
        self.assertEqual([a.object.id for a in state.attached_collision_objects], [TARGET])
        self.assertEqual(state.attached_collision_objects[0].object.operation, CollisionObject.ADD)
        for name, value in zip(NAMES, CLEAN[-1]):
            self.assertAlmostEqual(q[name], math.radians(value))

    def test_the_last_robot_state_must_hold_the_object(self):
        '''skipping the empty scenes still refuses when the last robot state has no object attached, or there is none'''
        state, why = mpp.solution_end_state(solution(CLEAN[-1], attached='', collisions_only_after=1), TARGET)
        self.assertIsNone(state)
        self.assertIn('not attached', why)
        empty = Solution(sub_trajectory=[SubTrajectory(scene_diff=PlanningScene(is_diff=True))])
        state, why = mpp.solution_end_state(empty, TARGET)
        self.assertIsNone(state)
        self.assertIn('has a robot state', why)


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


class LiveAttach(TransportMoveTest):
    '''the background planning starts only after the pick thread reported its attach, the live scene shows the object
    attached without its world copy and the start state is valid there: a start state that carries the object while
    its world copy is still there collides with that copy (MoveIt then plans from a randomly moved start and prepends an
    unchecked move to it)'''
    def test_waits_for_the_attach_of_the_pick_thread(self):
        clean = trajectory(*CLEAN)
        move = self.begin([clean], join=False, attach=False)
        time.sleep(0.3)
        self.assertTrue(move.runner.is_alive())
        self.assertEqual((self.planner.starts, self.scene.requests, self.validity.requests), ([], [], []))
        move.attached()
        move.runner.join(120.0)
        self.assertIs(move.trajectory, clean)
        wanted = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS | PlanningSceneComponents.WORLD_OBJECT_NAMES
        self.assertEqual(self.scene.requests[0].components.components, wanted)
        checked = self.validity.requests[0]
        self.assertIs(checked.robot_state, move.start_state)   # the end of the pick with the object attached
        self.assertEqual(checked.group_name, 'arm')

    def test_no_attach_in_time_no_plan(self):
        self.params.values['plan_transport_during_pick_attach_wait_s'] = 0.2
        move = self.begin([trajectory(*CLEAN)], attach=False)
        self.assertIsNone(move.trajectory)
        self.assertIn(f'the pick did not attach {TARGET} within 0.2 s', move.reason)
        self.assertEqual((self.planner.starts, self.scene.requests), ([], []))

    def test_a_cancel_ends_the_wait_at_once(self):
        move = self.begin([trajectory(*CLEAN)], join=False, attach=False)
        started = time.monotonic()
        move.cancel()
        move.runner.join(5.0)
        self.assertLess(time.monotonic() - started, 1.0)
        self.assertIsNone(move.trajectory)
        self.assertIn('the pick ended before', move.reason)
        self.assertEqual(self.planner.starts, [])

    def test_the_live_scene_must_show_the_attach_without_the_world_copy(self):
        for state, why in (((True, True), 'its world copy is still there'), ((False, False), 'not attached')):
            self.scene = FakeScene([state])
            move = self.begin([trajectory(*CLEAN)])
            self.assertIsNone(move.trajectory)
            self.assertIn(f'not attached in the planning scene after the attach ({why})', move.reason)
            self.assertEqual((self.planner.starts, self.validity.requests), ([], []))

    def test_a_start_state_in_collision_gives_no_plan_and_names_the_contacts(self):
        self.validity = FakeValidity(valid=False, contacts=[(TARGET, 'table_2'), (TARGET, 'table_2')])
        move = self.begin([trajectory(*CLEAN)])
        self.assertIsNone(move.trajectory)
        self.assertIn(f'the start state is in collision in the live scene ({TARGET}-table_2)', move.reason)
        self.assertEqual(self.planner.starts, [])

    def test_a_service_error_or_an_invalid_wait_gives_no_plan(self):
        self.scene = FakeScene([(True, False)], error=RuntimeError('scene down'))
        self.assertIn('reading the planning scene failed (scene down)', self.begin([trajectory(*CLEAN)]).reason)
        self.scene = FakeScene([(True, False)])
        self.validity = FakeValidity(error=RuntimeError('check down'))
        self.assertIn('checking the start state failed (check down)', self.begin([trajectory(*CLEAN)]).reason)
        self.validity = FakeValidity()
        for value in (0.0, 'x', float('nan')):
            self.params.values['plan_transport_during_pick_attach_wait_s'] = value
            move = self.begin([trajectory(*CLEAN)])
            self.assertIsNone(move.trajectory)
            self.assertIn('invalid attach wait', move.reason)
        self.assertEqual(self.planner.starts, [])

    def test_a_finish_that_gives_up_on_the_planning_cancels_the_wait(self):
        move = self.begin([trajectory(*CLEAN)], join=False, attach=False)
        arm = FakeArm([])
        with mock.patch.object(mpp, 'guarded_named_move', mock.Mock(return_value=True)):
            moved, how = move.finish(arm, self.guard, wait_s=0.1)
        self.assertTrue(moved)
        self.assertIn('did not end within', how)
        move.runner.join(5.0)
        self.assertFalse(move.runner.is_alive())


class AttachSignal(unittest.TestCase):
    '''the pick thread reports its attach to the move planned during the pick, only when the attach succeeded'''
    def setUp(self):
        from test_regrasp_overclose import TestPickProgress
        self.harness = TestPickProgress
        TestPickProgress.setUp(self)

    def run_pick(self, attach_code):
        from test_regrasp_overclose import posture
        m = self.harness.mtc(self, MoveItErrorCodes.SUCCESS)
        m.attach = lambda *a: attach_code
        m.transport_move = mock.Mock()
        m.execute(None, [mpp.GRIPPER, mpp.ATTACH, mpp.ARM], [], posture(0.0), attach=(TARGET, []), check_grasp=True)
        return m.transport_move

    def test_a_successful_attach_starts_the_planning(self):
        self.run_pick(MoveItErrorCodes.SUCCESS).attached.assert_called_once_with()

    def test_a_failed_attach_does_not(self):
        self.run_pick(MoveItErrorCodes.FAILURE).attached.assert_not_called()


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
            move.cancel.assert_called_once_with()   # a planning still waiting for the attach ends
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
