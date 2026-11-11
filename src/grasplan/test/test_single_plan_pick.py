'''(careful flag ~mtc_single_plan_pick, default false): a closed-set pick plans its untangle poses, the grasp and the
move to ~mtc_single_plan_pick_end_pose (transport) as one MTC task and executes it as one solution; when no grasp can be
planned that way and the arm did not move, the old way (the poses one by one, then the pick). Stubs the MTC stages (no
robot model, no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, MoveItErrorCodes, PickupGoal, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import grasplan.mtc_pick_place as mpp
import grasplan.pick as pick

OBJ, GRIPPER = 'power_drill_with_grip_1', ['finger_l', 'finger_r', 'palm']
POSES = ['untangle_cable_guide_1_right', 'untangle_cable_guide_2_right']
SUCCESS, FAILED = MoveItErrorCodes.SUCCESS, MoveItErrorCodes.PLANNING_FAILED


class Stage:
    def __init__(self, name, *args):
        self.name, self.children, self.properties, self.goal, self.group = name, [], {}, None, None
        self.allowed = []

    def allowCollisions(self, *args):
        self.allowed.append(args)

    def attachObject(self, *args):
        pass

    def setGoal(self, goal):
        self.goal = goal

    def add(self, child):
        self.children.append(child)


class Task(Stage):
    def clear(self):
        self.children = []

    def __getitem__(self, name):
        return next(c for c in self.children if c.name == name)


class Container(Stage):
    pass


def leaves(stage):
    return [leaf for c in stage.children for leaf in (leaves(c) if isinstance(c, Container) else [c])]


class Guard:
    def path_constraints(self):
        return 'wrist_3 window'


def mtc(single=None, start_move=None, guard=None):
    m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
    m.gripper_links, m.allow_octomap_contact, m.eef_link = GRIPPER, True, 'gripper_tcp'
    m.strict_contact_order = False   # closed set
    m.task, m.pipeline, m.arm_group, m.connect_timeout = Task('t'), 'pipeline', 'arm', 5.0
    m.cable_guard, m.single_plan, m.start_move = guard, single, start_move
    m.connect = lambda name, shaped=None: Stage(name)
    m.move_relative = lambda name, translation: Stage(name)
    m.compute_ik = lambda name, pose, monitored: Stage(name)
    m.grasp_ik_states = {}
    m.to_planning_frame = lambda pose: pose
    return m


def build(m):
    fake_stages = mock.Mock(ModifyPlanningScene=Stage, CurrentState=Stage, MoveTo=Stage)
    fake_core = mock.Mock(SerialContainer=Container)
    goal, grasp = PickupGoal(target_name=OBJ, support_surface_name='table_2'), Grasp(id='grasp_0', grasp_pose=PoseStamped())
    with mock.patch.object(mpp, 'stages', fake_stages), mock.patch.object(mpp, 'core', fake_core), \
            mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: default):
        return m.make_pick_task(goal, grasp)


class TestSingleTaskLayout(unittest.TestCase):
    def test_stage_order_untangle_pick_transport(self):
        task, roles = build(mtc(single=(POSES, 'transport')))
        self.assertEqual([s.name for s in task.children],
                         ['current state', f'move to {POSES[0]}', f'move to {POSES[1]}', 'allow octomap contact',
                          'move to pregrasp', 'grasp', 'forbid pick contacts before the end move', 'move to transport'])
        self.assertEqual([task.children[i].goal for i in (1, 2, 7)], POSES + ['transport'])
        self.assertEqual(roles[:5], [mpp.NOOP, mpp.UNTANGLE, mpp.UNTANGLE, mpp.NOOP, mpp.ARM])
        self.assertEqual(roles[-1], mpp.END_MOVE)
        self.assertEqual(len(roles), len(leaves(task)))   # one role per sub trajectory, as execute() requires

    def test_the_end_move_counts_the_pick_contacts_again(self):
        # the lift may touch the support and the octomap; the move to transport carrying the object may not
        task, _ = build(mtc(single=(POSES, 'transport')))
        block = task['forbid pick contacts before the end move']
        self.assertIn((OBJ, ['table_2'], False), block.allowed)
        self.assertIn((mpp.OCTOMAP_COLLISION_NAME, GRIPPER + [OBJ], False), block.allowed)
        self.assertNotIn(True, [args[-1] for args in block.allowed])

    def test_flag_off_keeps_the_old_layout(self):
        task, roles = build(mtc())
        self.assertEqual([s.name for s in task.children],
                         ['current state', 'allow octomap contact', 'move to pregrasp', 'grasp'])
        self.assertNotIn(mpp.END_MOVE, roles)
        self.assertEqual(len(roles), len(leaves(task)))

    def test_end_pose_empty_plans_only_the_poses_before(self):
        task, roles = build(mtc(single=(POSES, '')))
        self.assertEqual(task.children[-1].name, 'grasp')
        self.assertNotIn(mpp.END_MOVE, roles)

    def test_moves_use_the_arm_pipeline_and_the_cable_window(self):
        task, _ = build(mtc(single=(POSES, 'transport'), guard=Guard()))
        for move in (task.children[1], task.children[2], task.children[-1]):
            self.assertEqual(move.group, 'arm')
            self.assertEqual(move.timeout, 5.0)
            self.assertEqual(move.properties['path_constraints'], 'wrist_3 window')

    def test_a_running_start_move_wins(self):
        # the prefix stands in for the poses the arm is running; never both
        start_move = mock.Mock(names=POSES)
        task, roles = build(mtc(single=(POSES, 'transport'), start_move=start_move))
        self.assertEqual(task.children[1].name, 'start move (planned meanwhile)')
        self.assertNotIn(f'move to {POSES[0]}', [s.name for s in task.children])
        self.assertNotIn(mpp.END_MOVE, roles)


def trajectory():
    t = RobotTrajectory()
    t.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.0])]
    return t


class TestExecuteEndMove(unittest.TestCase):
    def run_execute(self, roles, codes):
        m = mtc()
        m.is_preempt_requested = lambda: False
        sent = []
        m.execute_arm = lambda traj: (sent.append(traj), codes.pop(0))[1]
        m.attach = lambda *a: SUCCESS
        m.command_gripper = lambda posture: SUCCESS
        subs = [mock.Mock(trajectory=trajectory() if r in (mpp.ARM, mpp.END_MOVE, mpp.UNTANGLE) else RobotTrajectory())
                for r in roles]
        m.chosen_solution = mock.Mock(**{'toMsg.return_value': mock.Mock(sub_trajectory=subs)})
        self.announced = []
        with mock.patch.object(mpp, 'announce_trajectory_start', self.announced.append):
            code = m.execute(None, roles, [], 'close', attach=(OBJ, GRIPPER))
        return m, code, sent

    def test_untangle_legs_run_and_are_announced_like_named_moves(self):
        # v2: the monitor starts afresh at each leg and at the pick after them, as the guard checked them
        roles = [mpp.NOOP, mpp.UNTANGLE, mpp.UNTANGLE, mpp.NOOP, mpp.ARM, mpp.ARM, mpp.ATTACH, mpp.ARM, mpp.END_MOVE]
        m, code, sent = self.run_execute(roles, [SUCCESS] * 6)
        self.assertEqual(code, SUCCESS)
        self.assertEqual(len(sent), 6)
        self.assertEqual(self.announced, ['mtc solution', 'single plan untangle leg', 'single plan untangle leg',
                                          'mtc solution after the untangle legs'])

    def test_without_untangle_legs_one_announcement(self):
        self.run_execute([mpp.NOOP, mpp.ARM, mpp.ATTACH, mpp.ARM], [SUCCESS, SUCCESS])
        self.assertEqual(self.announced, ['mtc solution'])

    def test_end_move_runs_after_the_lift(self):
        roles = [mpp.NOOP, mpp.ARM, mpp.ARM, mpp.ATTACH, mpp.ARM, mpp.END_MOVE]
        m, code, sent = self.run_execute(roles, [SUCCESS] * 4)
        self.assertEqual(code, SUCCESS)
        self.assertEqual(len(sent), 4)
        self.assertTrue(m.end_move_done)
        self.assertTrue(m.pick_progress['lifted'])

    def test_failed_end_move_keeps_the_pick(self):
        # the object is held and lifted as at the end of the old way's pick: success, the end move's failure named
        roles = [mpp.NOOP, mpp.ARM, mpp.ATTACH, mpp.ARM, mpp.END_MOVE]
        m, code, _ = self.run_execute(roles, [SUCCESS, SUCCESS, MoveItErrorCodes.CONTROL_FAILED])
        self.assertEqual(code, SUCCESS)
        self.assertFalse(m.end_move_done)
        self.assertIn(str(MoveItErrorCodes.CONTROL_FAILED), m.end_move_failure)

    def test_preempted_end_move_is_no_success(self):
        roles = [mpp.NOOP, mpp.ARM, mpp.ATTACH, mpp.ARM, mpp.END_MOVE]
        m, code, _ = self.run_execute(roles, [SUCCESS, SUCCESS, MoveItErrorCodes.PREEMPTED])
        self.assertEqual(code, MoveItErrorCodes.PREEMPTED)
        self.assertEqual(m.end_move_failure, '')

    def test_failed_untangle_leg_is_named(self):
        roles = [mpp.NOOP, mpp.UNTANGLE, mpp.UNTANGLE, mpp.NOOP, mpp.ARM, mpp.ATTACH, mpp.ARM, mpp.END_MOVE]
        m, code, sent = self.run_execute(roles, [SUCCESS, MoveItErrorCodes.CONTROL_FAILED])
        self.assertEqual(code, MoveItErrorCodes.CONTROL_FAILED)
        self.assertEqual(len(sent), 2)   # nothing after the failed leg
        self.assertTrue(m.untangle_failure.startswith('untangle leg 2/2'), m.untangle_failure)
        self.assertFalse(m.pick_progress['closed'])

    def test_without_end_move_nothing_is_marked(self):
        m, code, _ = self.run_execute([mpp.NOOP, mpp.ARM, mpp.ATTACH, mpp.ARM], [SUCCESS, SUCCESS])
        self.assertEqual(code, SUCCESS)
        self.assertFalse(m.end_move_done)


def picker(offer_results, executed=False, preempt=False, untangle_failure=''):
    p = pick.PickTools.__new__(pick.PickTools)
    p.mtc = mock.Mock(single_plan=None, end_move_done=True, executed=executed, end_move_failure='',
                      untangle_failure=untangle_failure)
    p.pick_action_server = mock.Mock(**{'is_preempt_requested.return_value': preempt})
    p.calls, p.moves = [], []

    def offer(obj, grasps, support, external):
        p.calls.append((p.mtc.single_plan, external))
        return offer_results.pop(0)
    p.offer_grasps = offer
    p.move_arm_to_posture = lambda pose: p.moves.append(pose) or True
    return p


class TestPickSide(unittest.TestCase):
    def single_for(self, params, open_set=False, untangle=POSES, has_mtc=True, resets=True):
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc = object() if has_mtc else None
        with mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: params.get(name, default)), \
                mock.patch.object(mpp, 'monitor_resets_at_trajectory_start', lambda: resets):
            return p.single_plan_for(untangle, open_set)

    def test_skipped_when_the_monitor_does_not_start_afresh(self):
        # the legs are checked apart, as the monitor sees them only with reset_at_trajectory_start
        self.assertIsNone(self.single_for({'~mtc_single_plan_pick': True}, resets=False))

    def test_off_by_default(self):
        self.assertIsNone(self.single_for({}))

    def test_on_for_a_closed_set_pick(self):
        self.assertEqual(self.single_for({'~mtc_single_plan_pick': True}), (POSES, 'transport'))

    def test_never_for_open_set_or_without_mtc(self):
        self.assertIsNone(self.single_for({'~mtc_single_plan_pick': True}, open_set=True))
        self.assertIsNone(self.single_for({'~mtc_single_plan_pick': True}, has_mtc=False))

    def test_end_pose_param(self):
        on = {'~mtc_single_plan_pick': True, '~mtc_single_plan_pick_end_pose': 'home'}
        self.assertEqual(self.single_for(on), (POSES, 'home'))
        on['~mtc_single_plan_pick_end_pose'] = ''
        self.assertEqual(self.single_for(on), (POSES, ''))
        self.assertIsNone(self.single_for(on, untangle=[]))   # nothing left to plan together
        self.assertEqual(self.single_for({'~mtc_single_plan_pick': True}, untangle=[]), ([], 'transport'))

    def offer(self, p):
        return p.offer_single_plan(pick.objectToPick(OBJ), ['g0', 'g1'], 'table_2', (POSES, 'transport'))

    def test_success_needs_no_fallback(self):
        p = picker([SUCCESS])
        self.assertEqual(self.offer(p), SUCCESS)
        self.assertEqual(p.calls, [((POSES, 'transport'), False)])
        self.assertEqual(p.moves, [])
        self.assertIsNone(p.mtc.single_plan)

    def test_unplannable_falls_back_to_the_old_way(self):
        p = picker([FAILED, SUCCESS])
        self.assertEqual(self.offer(p), SUCCESS)
        self.assertEqual(p.moves, POSES)                                   # the poses one by one
        self.assertEqual(p.calls, [((POSES, 'transport'), False), (None, False)])   # then the plain pick

    def test_no_fallback_after_the_arm_moved(self):
        p = picker([MoveItErrorCodes.CONTROL_FAILED], executed=True)
        self.assertEqual(self.offer(p), MoveItErrorCodes.CONTROL_FAILED)
        self.assertEqual(p.moves, [])
        self.assertEqual(len(p.calls), 1)

    def test_a_failed_untangle_leg_goes_on_the_old_way(self):
        # the old way ignores a failed untangle move and goes on; nothing was grasped yet
        p = picker([MoveItErrorCodes.CONTROL_FAILED, SUCCESS], executed=True,
                   untangle_failure='untangle leg 1/2 (move to x) failed: stop')
        self.assertEqual(self.offer(p), SUCCESS)
        self.assertEqual(p.moves, POSES)
        self.assertEqual(p.calls[-1], (None, False))

    def test_a_refused_untangle_leg_goes_to_the_old_way(self):
        p = picker([FAILED, SUCCESS])

        def offer(obj, grasps, support, external):
            p.calls.append((p.mtc.single_plan, external))
            if p.mtc.single_plan:
                p.mtc.untangle_leg_refused = 'wrist, untangle leg 1'
            return [FAILED, SUCCESS][len(p.calls) - 1]
        p.offer_grasps = offer
        self.assertEqual(self.offer(p), SUCCESS)
        self.assertEqual(p.moves, POSES)

    def test_a_failed_end_move_reaches_the_status_text(self):
        p = picker([SUCCESS])
        p.mtc.end_move_done, p.mtc.end_move_failure = False, 'protective stop'
        self.assertEqual(self.offer(p), SUCCESS)
        self.assertEqual(p.pick_note, 'the move to transport failed: protective stop, the arm is not at transport')

    def test_the_old_way_failure_names_why_the_single_plan_was_given_up(self):
        p = picker([MoveItErrorCodes.CONTROL_FAILED, FAILED], executed=True,
                   untangle_failure='untangle leg 1/2 (move to x) failed: stop')
        p.pick_failure_reason, p.mtc.failure_reason = '', 'no grasp could be planned'
        self.assertEqual(self.offer(p), FAILED)
        self.assertTrue(p.pick_failure_reason.startswith('single plan given up (untangle leg 1/2'), p.pick_failure_reason)
        self.assertIn('then the old way failed: no grasp could be planned', p.pick_failure_reason)

    def test_no_fallback_when_preempted(self):
        p = picker([False])
        self.assertIs(self.offer(p), False)
        self.assertEqual(p.moves, [])
        p = picker([FAILED], preempt=True)
        self.assertEqual(self.offer(p), FAILED)
        self.assertEqual(p.moves, [])

    def test_single_plan_cleared_when_planning_raises(self):
        p = picker([])
        p.offer_grasps = mock.Mock(side_effect=RuntimeError('boom'))
        with self.assertRaises(RuntimeError):
            self.offer(p)
        self.assertIsNone(p.mtc.single_plan)



def solution(points_per_sub):
    from types import SimpleNamespace
    subs = []
    for n in points_per_sub:
        t = RobotTrajectory()
        t.joint_trajectory.points = [JointTrajectoryPoint(positions=[float(i)]) for i in range(n)]
        subs.append(SimpleNamespace(trajectory=t))
    return SimpleNamespace(sub_trajectory=subs)


class TestSegmentedGuardCheck(unittest.TestCase):
    """v2: the single plan's solution is checked in the legs the old way checks: each untangle leg alone with the named
    moves' criterion, then the pick and the end move from a fresh chain with the MTC margin (series_2359: one replay
    over all legs refused every grasp)."""

    def setUp(self):
        self.calls = []
        self.results = []
        guard = mock.Mock()
        guard.stop_stretch.return_value = 0.02

        def check(msg, max_stretch=None, chain_stops=True):
            kept = [i for i, sub in enumerate(msg.sub_trajectory) if sub.trajectory.joint_trajectory.points]
            self.calls.append((kept, max_stretch, chain_stops))
            return self.results.pop(0) if self.results else (True, -0.2, 'wrist', {})
        guard.check.side_effect = check
        self.m = mtc(single=(POSES, 'transport'), guard=guard)
        self.m.cable_guard = guard
        self.m.start_move = None
        self.m.task_legs = 2   # start_task of the single-plan pick task
        self.resets = True
        patcher = mock.patch.object(mpp, 'monitor_resets_at_trajectory_start', lambda: self.resets)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.params = {}
        patcher = mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_legs_alone_then_the_pick_from_a_fresh_chain(self):
        # sub trajectories: current state, 2 untangle legs, pregrasp, approach, lift, transport
        ok = self.m.check_solution(solution([0, 5, 5, 5, 3, 3, 4]))
        self.assertTrue(ok[0])
        self.assertEqual(self.calls, [([1], 0.02, False), ([2], 0.02, False), ([3, 4, 5, 6], None, True)])

    def test_carrier_stops_param_reaches_the_legs(self):
        self.params['~mtc_cable_named_move_rope_stops'] = True
        self.m.check_solution(solution([0, 5, 5, 5]))
        self.assertEqual([c[2] for c in self.calls[:2]], [True, True])

    def test_a_refused_leg_refuses_the_solution(self):
        self.results = [(True, -0.3, 'chord', {}), (False, 0.05, 'wrist', {'w': 1})]
        ok, stretch, span, _ = self.m.check_solution(solution([0, 5, 5, 5]))
        self.assertFalse(ok)
        self.assertEqual((stretch, span), (0.05, 'wrist, untangle leg 2'))
        self.assertEqual(len(self.calls), 2)   # the pick is not checked any more

    def test_without_single_plan_one_check_of_everything(self):
        self.m.single_plan, self.m.task_legs = None, 0
        self.m.check_solution(solution([0, 5, 3]))
        self.assertEqual(self.calls, [([1, 2], None, True)])

    def test_a_regrasp_task_during_the_single_plan_has_no_legs(self):
        # 'regrasp deeper' and the tail are planned while single_plan is set: one check, no bogus legs
        with mock.patch.object(mpp, 'stages', mock.Mock(CurrentState=Stage, MoveTo=Stage)):
            self.m.start_task('regrasp deeper')
        self.m.check_solution(solution([0, 5, 3]))
        self.assertEqual(self.calls, [([1, 2], None, True)])

    def test_without_the_monitor_reset_one_check_of_everything(self):
        self.resets = False
        self.m.check_solution(solution([0, 5, 5, 5, 3]))
        self.assertEqual(self.calls, [([1, 2, 3, 4], None, True)])


class TestRefusedLeg(unittest.TestCase):
    """a refused untangle leg is no grasp rejection: no turned grasp, no chain detour, the other grasps not planned"""

    def setUp(self):
        self.m = mtc(single=(POSES, 'transport'), guard=mock.Mock(chain_stopped=True, max_stretch=0.0))
        self.m.task_legs, self.m.chain_rejections, self.m.cable_rejections = 2, 0, 0
        self.m.check_solution = lambda msg: (False, 0.05, 'wrist, untangle leg 1', {})
        patcher = mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: default)
        patcher.start()
        self.addCleanup(patcher.stop)

    def task(self):
        sol = mock.Mock(cost=1.0)
        return mock.Mock(**{'plan.return_value': True, 'solutions': [sol]})

    def test_leg_refusal_is_remembered_and_counts_no_rejection(self):
        self.assertFalse(self.m.plan(self.task(), 'pick g0'))
        self.assertEqual(self.m.untangle_leg_refused, 'wrist, untangle leg 1')
        self.assertIsNone(self.m.guard_rejected_q)
        self.assertEqual((self.m.chain_rejections, self.m.cable_rejections), (0, 0))

    def test_later_grasps_are_not_planned(self):
        self.m.untangle_leg_refused = 'wrist, untangle leg 1'
        task = self.task()
        self.assertFalse(self.m.plan(task, 'pick g1'))
        task.plan.assert_not_called()
        self.assertIn('untangle leg 1', self.m.last_plan_failure)


if __name__ == '__main__':
    unittest.main()
