'''The MTC cable guard's chain replay from the cable monitor's live chain (~mtc_cable_replay_live_state, off in code):
off, the replay starts from a fresh chain as before and never asks the monitor; on, the path is replayed from a fresh
chain and also from the monitor's chain when that is fresh and taken where the path starts and the monitor does not
reset its chain at every trajectory start (else the fresh replay alone, with the reason logged); the path passes only
when both replays pass. Fake
monitor service and fake MTC solution messages, no ROS master; needs the Mobipick image and
mobipick_cable_entanglement.'''
import json
import math
import os
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import rospy
import yaml
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from grasplan.mtc_pick_place import CableGuard

J = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
NAMES = [f'mobipick/ur5_{j}_joint' for j in J]
HOME = [0.0, -149.0, 139.2, -117.5, -90.0, 179.9]
PLACE_END = [-90.4, -131.3, 140.3, -148.3, -91.0, 83.5]   # HOME -> PLACE_END catches the chain on the tool
ANYGRASP = [-90, -90, 114.6, -43, 90, 270]
NOW = 100.0


def q(deg, names=NAMES):
    return {n: math.radians(v) for n, v in zip(names, deg)}


def solution(*waypoints, seconds=3.0):
    trajectory = JointTrajectory(joint_names=NAMES)
    for i, w in enumerate(waypoints):
        trajectory.points.append(JointTrajectoryPoint(positions=[math.radians(v) for v in w],
                                                      time_from_start=rospy.Duration(seconds * i)))
    return NS(sub_trajectory=[NS(trajectory=NS(joint_trajectory=trajectory))])


class TestLiveChainState(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from mobipick_cable_entanglement.cable_model import CableModel, config_from_dict
        from mobipick_cable_entanglement.path_check import resample
        import copy
        import rospkg
        pkg = rospkg.RosPack().get_path('mobipick_cable_entanglement')
        with open(os.path.join(pkg, 'config', 'cable_model.yaml')) as f:
            cfg = config_from_dict(yaml.safe_load(f))
        with open(os.path.join(pkg, 'test', 'mobipick_sim_robot_description.urdf')) as f:
            cls.urdf = f.read()
        chain_cfg = copy.deepcopy(cfg)
        chain_cfg.cable_carrier.enabled = True
        cls.chain_cfg = chain_cfg
        # the monitor after the motion from home to the place pose (one update per 0.1 s tick)
        monitor = CableModel(cls.urdf, copy.deepcopy(chain_cfg))
        monitor.reset(q(HOME))
        last = None
        for t, qt in resample([(0.0, q(HOME)), (3.0, q(PLACE_END))], 10.0):
            monitor.update(qt, 0.0 if last is None else t - last, use_carrier=True)
            last = t
        cls.state = json.loads(json.dumps(monitor.chain_state()))
        cls.state['stamp'] = NOW - 0.05

    def guard(self, live):
        from mobipick_cable_entanglement.cable_model import CableModel
        import copy
        guard = CableGuard.__new__(CableGuard)
        guard.chain_model = CableModel(self.urdf, copy.deepcopy(self.chain_cfg))
        guard.chain_rate, guard.replay_live_state = 10.0, live
        return guard

    def reply(self, state):
        return lambda name, kind: (lambda: NS(success=True, message=json.dumps(state)))

    def test_off_never_asks_the_monitor_and_replays_from_a_fresh_chain(self):
        from mobipick_cable_entanglement.path_check import carrier_replay
        guard = self.guard(live=False)
        with mock.patch.object(rospy, 'wait_for_service', side_effect=AssertionError('asked the monitor')):
            verdict = guard.chain_verdict(solution(PLACE_END, ANYGRASP))
        fresh = carrier_replay(self.guard(live=False).chain_model,
                               [(0.0, q(PLACE_END)), (3.0, q(ANYGRASP))], 10.0)
        self.assertEqual((verdict.max_stretch, verdict.span, verdict.updates), (fresh.max_stretch, fresh.span,
                                                                                fresh.updates))

    def test_on_replays_also_from_the_monitor_chain(self):
        from mobipick_cable_entanglement.path_check import carrier_replay
        guard = self.guard(live=True)
        with mock.patch.object(rospy, 'wait_for_service'), \
                mock.patch.object(rospy, 'ServiceProxy', self.reply(self.state)), \
                mock.patch.object(rospy, 'get_time', return_value=NOW), \
                mock.patch.object(rospy, 'get_param', side_effect=lambda name, default=None: default):
            verdict = guard.chain_verdict(solution(PLACE_END, ANYGRASP))
        samples = [(0.0, q(PLACE_END)), (3.0, q(ANYGRASP))]
        fresh = carrier_replay(self.guard(live=False).chain_model, samples, 10.0)
        live = carrier_replay(self.guard(live=False).chain_model, samples, 10.0, start_state=self.state)
        # the live chain is still caught from the motion before: its replay sees more stretch than the fresh one
        self.assertGreater(live.max_stretch, fresh.max_stretch + 0.005,
                           f'live {live.max_stretch:+.4f}, fresh {fresh.max_stretch:+.4f}')
        self.assertTrue(verdict.stop)   # both refuse here: the fresh refusal is reported first
        self.assertEqual((verdict.max_stretch, verdict.span), (fresh.max_stretch, fresh.span))

    def combined(self, fresh_stops, live_stops):
        '''chain_verdict with the live state found and both replays faked: (verdict, start states replayed)'''
        guard = self.guard(live=True)
        guard.live_chain_state = lambda q0: (self.state, 'the monitor\'s live chain')
        starts = []

        def replay(model, samples, rate, start_state=None):
            starts.append(start_state)
            stop = live_stops if start_state is not None else fresh_stops
            return NS(stop=stop, max_stretch=0.03 if stop else -0.05, span='wrist', limiting='stretch', q={},
                      updates=len(samples), origin='live' if start_state is not None else 'fresh')
        with mock.patch('mobipick_cable_entanglement.path_check.carrier_replay', replay), \
                mock.patch.object(rospy, 'logwarn') as warn:
            verdict = guard.chain_verdict(solution(PLACE_END, ANYGRASP))
        return verdict, starts, warn

    def test_both_replays_pass_and_the_path_passes(self):
        verdict, starts, warn = self.combined(fresh_stops=False, live_stops=False)
        self.assertEqual(starts, [None, self.state])
        self.assertFalse(verdict.stop)
        warn.assert_not_called()

    def test_the_fresh_replay_keeps_its_veto_with_a_slack_live_chain(self):
        verdict, starts, warn = self.combined(fresh_stops=True, live_stops=False)
        self.assertEqual(starts, [None, self.state])
        self.assertTrue(verdict.stop)
        self.assertEqual(verdict.origin, 'fresh')
        self.assertIn('the fresh chain refuses, the live chain passes -> path refused', warn.call_args[0][0])

    def test_a_live_refusal_refuses_the_path_the_fresh_replay_passes(self):
        verdict, starts, warn = self.combined(fresh_stops=False, live_stops=True)
        self.assertTrue(verdict.stop)
        self.assertEqual(verdict.origin, 'live')
        self.assertIn('the fresh chain passes, the live chain refuses -> path refused', warn.call_args[0][0])

    def test_without_a_live_state_the_fresh_replay_alone_decides(self):
        guard = self.guard(live=True)
        guard.live_chain_state = lambda q0: (None, 'a fresh chain (live chain unused: stale)')
        starts = []

        def replay(model, samples, rate, start_state=None):
            starts.append(start_state)
            return NS(stop=True, max_stretch=0.03, span='wrist', limiting='stretch', q={}, updates=len(samples))
        with mock.patch('mobipick_cable_entanglement.path_check.carrier_replay', replay):
            verdict = guard.chain_verdict(solution(PLACE_END, ANYGRASP))
        self.assertEqual(starts, [None])
        self.assertTrue(verdict.stop)

    def test_falls_back_to_a_fresh_chain_with_the_reason(self):
        guard = self.guard(live=True)
        stale = dict(self.state, stamp=NOW - 5.0)
        elsewhere = dict(self.state, q=q(ANYGRASP))
        common = [mock.patch.object(rospy, 'get_time', return_value=NOW),
                  mock.patch.object(rospy, 'get_param', side_effect=lambda name, default=None: default),
                  mock.patch.object(rospy, 'logwarn_throttle'), mock.patch.object(rospy, 'loginfo_throttle')]
        for p in common:
            p.start()
        self.addCleanup(mock.patch.stopall)
        resetting = dict(self.state, reset_at_trajectory_start=True)
        for state, why in ((stale, 'stale'), (elsewhere, 'elsewhere'), (resetting, 'afresh')):
            with mock.patch.object(rospy, 'wait_for_service'), \
                    mock.patch.object(rospy, 'ServiceProxy', self.reply(state)):
                found, origin = guard.live_chain_state(q(PLACE_END))
            self.assertIsNone(found)
            self.assertIn(why, origin)
        with mock.patch.object(rospy, 'wait_for_service', side_effect=rospy.ROSException('timeout')):
            found, origin = guard.live_chain_state(q(PLACE_END))
        self.assertIsNone(found)
        self.assertIn('no cable monitor', origin)
        with mock.patch.object(rospy, 'wait_for_service'), \
                mock.patch.object(rospy, 'ServiceProxy', self.reply(self.state)):
            found, origin = guard.live_chain_state(q(PLACE_END))
        self.assertEqual(found['fingerprint'], self.state['fingerprint'])
        self.assertIn('live chain', origin)


if __name__ == '__main__':
    unittest.main()
