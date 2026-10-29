'''(~pick_start_move_carrier_stops, careful, off in code): the open-set pick's start move from the grasp view to
transport refuses a plan the cable guard's chain replay predicts to stop, replans, then detours via the observation
pose, each leg checked the same way; refused there too, nothing moves and the pick says why. The four outcomes with the
real CableGuard and its carrier link chain replay (fixture of test_cable_chain_guard) and a fake arm; the pick side with
stubs. Mobipick image.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import grasplan.mtc_pick_place as mpp
import grasplan.pick as pick
from grasplan.mtc_pick_place import guarded_named_move
from test_cable_chain_guard import CATCH, CLEAN
import test_guarded_named_move as tgnm
from test_guarded_named_move import FakeArm, trajectory

VIA = ['observe100cm_right']


class ChainStopsPerCall(unittest.TestCase):
    """the fixture of test_guarded_named_move (real guard, fake arm), with the global ~mtc_cable_named_move_carrier_stops
    false (log only, night19): only the per-call chain_stops refuses"""
    chain_stops, mode, sim_time = False, 'enforce', True
    param = tgnm.TestGuardedNamedMove.param
    setUp = tgnm.TestGuardedNamedMove.setUp

    @classmethod
    def setUpClass(cls):
        tgnm.TestGuardedNamedMove.setUpClass()
        cls.guard = tgnm.TestGuardedNamedMove.guard

    def move(self, plans):
        arm, report = FakeArm(plans), {}
        ok = guarded_named_move(arm, 'transport', self.guard, 3, via=VIA, chain_stops=True, report=report)
        return ok, arm, report

    def test_pass(self):
        clean = trajectory(*CLEAN)
        ok, arm, report = self.move([clean])
        self.assertTrue(ok)
        self.assertEqual(arm.executed, [clean])
        self.assertEqual([(p[0], p[1], p[3]) for p in report['plans']], [('transport', 1, True)])
        self.assertNotIn('via', report)

    def test_replan_passes(self):
        catch, clean = trajectory(*CATCH), trajectory(*CLEAN)
        ok, arm, report = self.move([catch, clean])
        self.assertTrue(ok)
        self.assertEqual(arm.executed, [clean])   # the predicted stop never runs
        self.assertEqual([p[3] for p in report['plans']], [False, True])

    def test_detour_via_the_observation_pose(self):
        catch, clean = trajectory(*CATCH), trajectory(*CLEAN)
        ok, arm, report = self.move([catch, catch, catch, clean, clean])
        self.assertTrue(ok)
        self.assertEqual(arm.executed, [clean, clean])   # observe100cm_right, then transport
        self.assertEqual(report['via'], 'observe100cm_right')
        self.assertEqual([(p[0], p[3]) for p in report['plans']][-2:],
                         [('observe100cm_right', True), ('transport', True)])

    def test_refused_nothing_moves(self):
        ok, arm, report = self.move([trajectory(*CATCH)] * 9)
        self.assertFalse(ok)
        self.assertEqual((arm.executed, arm.gos), ([], 0))
        self.assertIn('cable guard', report['reason'])

    def test_without_the_per_call_switch_the_global_log_only_stays(self):
        catch = trajectory(*CATCH)
        arm = FakeArm([catch])
        self.assertTrue(guarded_named_move(arm, 'transport', self.guard))
        self.assertEqual(arm.executed, [catch])   # the old path: logged, moved


def run_start(flag, ok=True, report=None, mode=None):
    params, calls = {'~pick_start_move_carrier_stops': flag}, []
    if mode is not None:
        params['~cable_detour_mode'] = mode
    p = pick.PickTools.__new__(pick.PickTools)
    p.mtc = NS(cable_guard='guard', failure_reason='', update_cable_guard=lambda: None)
    p.robot = NS(arm='arm')
    p.pick_start_refused, p.pick_start_failed = '', ''
    p.move_arm_to_posture = lambda name: calls.append(('posture', name)) or True

    def guarded(arm, name, guard, attempts, via=None, chain_stops=None, report=None):
        calls.append(('guarded', name, attempts, via, chain_stops))
        report.update(given)
        return ok
    given = report or {}
    monitor = NS(start=lambda: None, peak=0.012)
    with mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: params.get(name, default)), \
            mock.patch.object(mpp, 'guarded_named_move', guarded), mock.patch.object(mpp, 'monitor_stretch', lambda: monitor), \
            mock.patch.object(pick.rospy, 'loginfo') as info:
        result = p.move_arm_to_pick_start('transport')
    return p, calls, result, [str(c) for c in info.call_args_list]


class PickSide(unittest.TestCase):
    def test_flag_off_is_the_old_posture_move(self):
        p, calls, ok, _ = run_start(False)
        self.assertTrue(ok)
        self.assertEqual(calls, [('posture', 'transport')])

    def test_cable_aware_mode_is_the_old_posture_move_also_with_the_flag(self):
        p, calls, ok, _ = run_start(True, mode='cable_aware')
        self.assertTrue(ok)
        self.assertEqual(calls, [('posture', 'transport')])

    def test_flag_on_refuses_chain_stops_with_three_plans_and_the_observation_detour(self):
        p, calls, ok, logs = run_start(True, report={'plans': [('transport', 1, 0.022, False, 'wrist'),
                                                               ('transport', 2, -0.105, True, 'wrist')]})
        self.assertTrue(ok)
        self.assertEqual(calls, [('guarded', 'transport', 3, VIA, True)])
        line = next(entry for entry in logs if 'pick start move to transport' in entry)
        self.assertIn('transport #1 +0.022 refused, transport #2 -0.105 pass', line)
        self.assertIn('monitor max stretch during the move: +0.012', line)

    def test_refusal_reaches_the_pick_result(self):
        p, calls, ok, _ = run_start(True, ok=False, report={'reason': 'cable guard: stretch +0.022 in the wrist span'})
        self.assertFalse(ok)
        self.assertEqual(p.pick_start_refused, 'cable guard: stretch +0.022 in the wrist span')
        p.pick_start_failed = 'transport'
        with mock.patch.object(pick.rospy, 'logerr'):
            self.assertIs(p.start_pose_failed(), False)
        self.assertEqual(p.mtc.failure_reason,
                         'pick start move refused by the cable guard (cable guard: stretch +0.022 in the wrist span)')
