'''#271 phase 3: the cable replay detour of the MTC pick (~mtc_cable_replay_detour, careful, off in code): a closed-set
pick the chain replay refused goes to a detour pose as a separate motion and is planned once more; never after a cancel,
for open-set picks or while the named-move guard only logs. The trigger is the count of chain replay rejections that
plan() keeps. Mocks (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from moveit_msgs.msg import MoveItErrorCodes

import grasplan.mtc_pick_place as mpp
import grasplan.pick as pick



class FakeMtc:
    def __init__(self, codes, chain_rejections):
        self.codes, self.chain_rejections = list(codes), chain_rejections
        self.executed, self.cable_guard, self.failure_reason, self.goals = False, object(), '', []

    def pickup(self, goal):
        self.goals.append(goal)
        code = self.codes.pop(0)
        if code != MoveItErrorCodes.SUCCESS:
            self.failure_reason = 'none of the 1 grasps could be planned'
        return NS(error_code=NS(val=code), grasp=NS(id='grasp_1', grasp_quality=0.9))


class CableReplayDetour(unittest.TestCase):
    def setUp(self):
        self.params = {'~mtc_cable_replay_detour': True}
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.moves = []
        self.reachable = {'untangle_cable_guide_2_right': True, 'untangle_cable_guide_1_right': True}

        def move(arm, name, guard, attempts=2, via=None, chain_stops=None):
            self.moves.append((name, via, chain_stops))
            return self.reachable[name]
        patcher = mock.patch('grasplan.mtc_pick_place.guarded_named_move', move)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.mode = 'enforce'
        patcher = mock.patch('grasplan.mtc_pick_place.named_move_guard_mode', lambda: self.mode)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.monitor_resets = True
        patcher = mock.patch('grasplan.mtc_pick_place.monitor_resets_at_trajectory_start', lambda: self.monitor_resets)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.preempt = False

    def tools(self, codes, chain_rejections, open_set=False):
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc, p.robot = FakeMtc(codes, chain_rejections), NS(arm='arm')
        p.pick_action_server = NS(is_preempt_requested=lambda: self.preempt)
        p.open_set_pick, p.detoured = open_set, ''
        p.arm_group_name, p.planning_time = 'arm', 5.0
        return p

    def pick(self, p):
        target = NS(get_object_class_and_id_as_string=lambda: 'multimeter_1')
        return p._pick_with_mtc(target, ['grasp_1'], 'table_2')

    def test_flag_off_no_detour(self):
        self.params = {}
        p = self.tools([MoveItErrorCodes.PLANNING_FAILED], 1)
        self.assertEqual(self.pick(p), MoveItErrorCodes.PLANNING_FAILED)
        self.assertEqual((self.moves, len(p.mtc.goals)), ([], 1))

    def test_chain_refusal_goes_via_untangle_2_and_plans_again(self):
        p = self.tools([MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.SUCCESS], 1)
        self.assertEqual(self.pick(p), MoveItErrorCodes.SUCCESS)
        self.assertEqual(self.moves, [('untangle_cable_guide_2_right', [], True)])
        self.assertEqual(len(p.mtc.goals), 2)
        self.assertEqual(p.detoured, 'untangle_cable_guide_2_right')

    def test_geometry_refusal_alone_no_detour(self):
        # the detour only helps when the chain catches; a geometrically taut path stays refused
        p = self.tools([MoveItErrorCodes.PLANNING_FAILED], 0)
        self.assertIsNone(p.cable_replay_detour(MoveItErrorCodes.PLANNING_FAILED))
        self.assertEqual(self.moves, [])

    def test_something_moved_no_detour(self):
        p = self.tools([MoveItErrorCodes.CONTROL_FAILED], 1)
        p.mtc.executed = True
        self.assertEqual(self.pick(p), MoveItErrorCodes.CONTROL_FAILED)
        self.assertEqual(self.moves, [])

    def test_next_pose_when_the_first_is_refused(self):
        self.params['~mtc_cable_replay_detour_poses'] = ['untangle_cable_guide_2_right', 'untangle_cable_guide_1_right']
        self.reachable['untangle_cable_guide_2_right'] = False
        p = self.tools([MoveItErrorCodes.PLANNING_FAILED], 1)
        self.assertEqual(p.cable_replay_detour(MoveItErrorCodes.PLANNING_FAILED), 'untangle_cable_guide_1_right')

    def test_no_pose_reachable_stays_refused(self):
        self.reachable['untangle_cable_guide_2_right'] = False
        p = self.tools([MoveItErrorCodes.PLANNING_FAILED], 1)
        self.assertEqual(self.pick(p), MoveItErrorCodes.PLANNING_FAILED)
        self.assertEqual(len(p.mtc.goals), 1)

    def test_failure_after_the_detour_says_so(self):
        p = self.tools([MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.PLANNING_FAILED], 1)
        self.assertEqual(self.pick(p), MoveItErrorCodes.PLANNING_FAILED)
        self.assertTrue(p.mtc.failure_reason.startswith(
            'after the cable replay detour to untangle_cable_guide_2_right: '))

    def test_cancelled_pick_no_detour(self):
        # pickup() returns PREEMPTED after a cancel: the arm must not move, whatever earlier grasps were refused for
        p = self.tools([MoveItErrorCodes.PREEMPTED], 1)
        self.assertEqual(self.pick(p), MoveItErrorCodes.PREEMPTED)
        self.assertEqual((self.moves, len(p.mtc.goals)), ([], 1))
        self.preempt = True   # cancel requested while pickup() still reported a planning failure
        p = self.tools([MoveItErrorCodes.PLANNING_FAILED], 1)
        self.assertEqual(self.pick(p), MoveItErrorCodes.PLANNING_FAILED)
        self.assertEqual((self.moves, len(p.mtc.goals)), ([], 1))

    def test_open_set_pick_no_detour(self):
        # open-set picks stay free of the untangle detour
        p = self.tools([MoveItErrorCodes.PLANNING_FAILED], 1, open_set=True)
        self.assertEqual(self.pick(p), MoveItErrorCodes.PLANNING_FAILED)
        self.assertEqual((self.moves, len(p.mtc.goals)), ([], 1))

    def test_log_mode_no_detour(self):
        # real robot default: the named-move guard only logs, so the leg would move whatever the chain replay says
        self.mode = 'log'
        p = self.tools([MoveItErrorCodes.PLANNING_FAILED], 1)
        self.assertEqual(self.pick(p), MoveItErrorCodes.PLANNING_FAILED)
        self.assertEqual((self.moves, len(p.mtc.goals)), ([], 1))

    def test_monitor_carrying_its_chain_no_detour(self):
        # without the monitor's reset at trajectory start its chain carries what the leg did, and the fresh-chain check
        # of the pick after the leg would pass a pick the monitor stops (sim 2026-10-27)
        self.monitor_resets = False
        p = self.tools([MoveItErrorCodes.PLANNING_FAILED], 1)
        self.assertEqual(self.pick(p), MoveItErrorCodes.PLANNING_FAILED)
        self.assertEqual((self.moves, len(p.mtc.goals)), ([], 1))

    def test_arm_moved_after_a_detour(self):
        # the #198(d) retry and the top-down fallback must not take a detoured arm for one that did not move
        p = self.tools([MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.PLANNING_FAILED], 1)
        self.assertTrue(p.arm_did_not_move(MoveItErrorCodes.PLANNING_FAILED))
        self.pick(p)
        self.assertFalse(p.arm_did_not_move(MoveItErrorCodes.PLANNING_FAILED))


class ChainRejectionCount(unittest.TestCase):
    '''plan() counts the solutions the guard rejected by the chain replay alone; geometry rejections do not count'''

    class Guard:
        max_stretch = -0.115

        def __init__(self, verdicts):
            self.verdicts, self.worst_q_rad, self.chain_stopped = list(verdicts), None, False

        def check(self, solution_msg):
            self.chain_stopped = self.verdicts.pop(0) == 'chain'
            return False, -0.05, 'wrist', {}

    class Task:
        def __init__(self):
            self.solutions = [NS(cost=1.0, toMsg=lambda: 'solution')]

        def init(self):
            pass

        def plan(self, count):
            return True

    def test_only_chain_rejections_count(self):
        with mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: default):
            m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
            m.cable_guard, m.cable_solutions, m.chain_rejections = self.Guard(['geometry', 'chain']), 1, 0
            m.where_in_task = lambda task, guard: ''
            self.assertFalse(m.plan(self.Task(), 'grasp a'))
            self.assertEqual(m.chain_rejections, 0)
            self.assertFalse(m.plan(self.Task(), 'grasp b'))
            self.assertEqual(m.chain_rejections, 1)


if __name__ == '__main__':
    unittest.main()
