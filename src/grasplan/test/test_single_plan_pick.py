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

    def allowCollisions(self, *args):
        pass

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
                          'move to pregrasp', 'grasp', 'move to transport'])
        self.assertEqual([task.children[i].goal for i in (1, 2, 6)], POSES + ['transport'])
        self.assertEqual(roles[:5], [mpp.NOOP, mpp.ARM, mpp.ARM, mpp.NOOP, mpp.ARM])
        self.assertEqual(roles[-1], mpp.END_MOVE)
        self.assertEqual(len(roles), len(leaves(task)))   # one role per sub trajectory, as execute() requires

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
        subs = [mock.Mock(trajectory=trajectory() if r in (mpp.ARM, mpp.END_MOVE) else RobotTrajectory()) for r in roles]
        m.chosen_solution = mock.Mock(**{'toMsg.return_value': mock.Mock(sub_trajectory=subs)})
        with mock.patch.object(mpp, 'announce_trajectory_start', lambda source: None):
            code = m.execute(None, roles, [], 'close', attach=(OBJ, GRIPPER))
        return m, code, sent

    def test_end_move_runs_after_the_lift(self):
        roles = [mpp.NOOP, mpp.ARM, mpp.ARM, mpp.ATTACH, mpp.ARM, mpp.END_MOVE]
        m, code, sent = self.run_execute(roles, [SUCCESS] * 4)
        self.assertEqual(code, SUCCESS)
        self.assertEqual(len(sent), 4)
        self.assertTrue(m.end_move_done)
        self.assertTrue(m.pick_progress['lifted'])

    def test_failed_end_move_is_reported(self):
        roles = [mpp.NOOP, mpp.ARM, mpp.ATTACH, mpp.ARM, mpp.END_MOVE]
        m, code, _ = self.run_execute(roles, [SUCCESS, SUCCESS, MoveItErrorCodes.CONTROL_FAILED])
        self.assertEqual(code, MoveItErrorCodes.CONTROL_FAILED)
        self.assertFalse(m.end_move_done)

    def test_without_end_move_nothing_is_marked(self):
        m, code, _ = self.run_execute([mpp.NOOP, mpp.ARM, mpp.ATTACH, mpp.ARM], [SUCCESS, SUCCESS])
        self.assertEqual(code, SUCCESS)
        self.assertFalse(m.end_move_done)


def picker(offer_results, executed=False, preempt=False):
    p = pick.PickTools.__new__(pick.PickTools)
    p.mtc = mock.Mock(single_plan=None, end_move_done=True, executed=executed)
    p.pick_action_server = mock.Mock(**{'is_preempt_requested.return_value': preempt})
    p.calls, p.moves = [], []

    def offer(obj, grasps, support, external):
        p.calls.append((p.mtc.single_plan, external))
        return offer_results.pop(0)
    p.offer_grasps = offer
    p.move_arm_to_posture = lambda pose: p.moves.append(pose) or True
    return p


class TestPickSide(unittest.TestCase):
    def single_for(self, params, open_set=False, untangle=POSES, has_mtc=True):
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc = object() if has_mtc else None
        with mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: params.get(name, default)):
            return p.single_plan_for(untangle, open_set)

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


if __name__ == '__main__':
    unittest.main()
