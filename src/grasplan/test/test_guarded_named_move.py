'''#121: named-pose arm moves (anygrasp, observe, transport, home) are planned and cable-checked before they run.
Fake MoveGroupCommander, the real CableGuard with the rope replay (fixture of test_cable_rope_guard); Mobipick image.'''
import math
import unittest
from unittest import mock

import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import grasplan.mtc_pick_place as mpp
from grasplan.mtc_pick_place import guarded_named_move
from test_cable_rope_guard import ANYGRASP, NAMES, PLACE_END, VIA, TestRopeGuard


def trajectory(*waypoints, seconds=5.0):
    t = RobotTrajectory(joint_trajectory=JointTrajectory(joint_names=NAMES))
    for i, w in enumerate(waypoints):
        t.joint_trajectory.points.append(JointTrajectoryPoint(positions=[math.radians(v) for v in w],
                                                              time_from_start=rospy.Duration(seconds * i)))
    return t


class FakeArm:
    def __init__(self, plans, go_result=True):
        self.plans, self.go_result = list(plans), go_result
        self.executed, self.gos, self.constraints = [], 0, []

    def set_named_target(self, name):
        self.target = name

    def set_path_constraints(self, c):
        self.constraints.append(c)

    def clear_path_constraints(self):
        self.constraints.append(None)

    def plan(self):
        return (True, self.plans.pop(0), 0.1, None) if self.plans else (False, RobotTrajectory(), 0.1, None)

    def execute(self, t, wait=True):
        self.executed.append(t)
        return True

    def go(self):
        self.gos += 1
        return self.go_result


class TestGuardedNamedMove(unittest.TestCase):
    rope_stops = True   # ~mtc_cable_named_move_rope_stops; the default (False) only logs a rope stop
    mode = 'enforce'    # ~mtc_cable_guard_named_moves (None: default from /use_sim_time)
    sim_time = True

    def param(self, name, default=None):   # no master
        if 'rope_stops' in name:
            return self.rope_stops
        if 'guard_named_moves' in name:
            return self.mode
        if name == '/use_sim_time':
            return self.sim_time
        return default

    def setUp(self):
        patcher = mock.patch.object(mpp.rospy, 'get_param', self.param)
        patcher.start()
        self.addCleanup(patcher.stop)

    @classmethod
    def setUpClass(cls):
        TestRopeGuard.setUpClass()
        cls.guard = TestRopeGuard.guard
        cls.guard.rope_check = True
        cls.guard.update_params = lambda: None           # no rospy params in the test
        cls.w3 = math.radians(180.0)
        mpp._current_joint = lambda joint, timeout=2.0: cls.w3   # no joint_states topic in the test

    def test_rope_catch_plan_is_replaced_by_a_clean_one(self):
        catch, clean = trajectory(PLACE_END, VIA, ANYGRASP), trajectory(PLACE_END, ANYGRASP)
        arm = FakeArm([catch, clean])
        self.assertTrue(guarded_named_move(arm, 'anygrasp', self.guard))
        self.assertEqual(arm.executed, [clean])
        self.assertEqual(arm.constraints[-1], None)       # path constraints cleared again

    def test_start_outside_the_window_plans_without_the_constraint(self):
        type(self).w3 = math.radians(-60.0)
        try:
            arm = FakeArm([trajectory(PLACE_END, ANYGRASP)])
            self.assertTrue(guarded_named_move(arm, 'home', self.guard))
            self.assertEqual(arm.constraints, [None])      # never set, only cleared
        finally:
            type(self).w3 = math.radians(180.0)

    def test_a_standard_pose_below_the_stop_threshold_passes(self):
        # night19: the anygrasp view at about -0.09 was refused by the -0.115 planning margin; the monitor would not stop
        near = [-90.7, -77.4, 100.1, -54.8, 45.1, 241.6]
        arm = FakeArm([trajectory(ANYGRASP, near)])
        self.assertTrue(guarded_named_move(arm, 'anygrasp', self.guard))
        self.assertEqual(len(arm.executed), 1)

    def test_goes_via_transport_when_the_direct_plans_catch(self):
        catch, clean = trajectory(PLACE_END, VIA, ANYGRASP), trajectory(PLACE_END, ANYGRASP)
        arm = FakeArm([catch, catch, clean, clean])      # anygrasp x2 refused, transport ok, anygrasp ok
        self.assertTrue(guarded_named_move(arm, 'anygrasp', self.guard))
        self.assertEqual(len(arm.executed), 2)

    def test_nothing_moves_when_every_plan_catches(self):
        arm = FakeArm([trajectory(PLACE_END, VIA, ANYGRASP)] * 6)
        self.assertFalse(guarded_named_move(arm, 'anygrasp', self.guard))
        self.assertEqual(arm.executed, [])
        self.assertEqual(arm.gos, 0)

    def test_rope_stop_only_logs_by_default(self):
        # suite 2026-09-28: the rope check refused a re-pick the unguarded code does fine; log only unless asked for
        self.rope_stops = False
        catch = trajectory(PLACE_END, VIA, ANYGRASP)
        arm = FakeArm([catch])
        with mock.patch.object(mpp.rospy, 'logwarn') as warn:
            self.assertTrue(guarded_named_move(arm, 'anygrasp', self.guard))
        self.assertEqual(arm.executed, [catch])
        self.assertTrue(any('log only' in str(c) for c in warn.call_args_list))

    def test_log_mode_moves_the_refused_plan_and_only_warns(self):
        # real robot (Oscar 2026-09-28): no refusal, no replan, no via transport, no wrist_3 constraint
        self.mode = 'log'
        catch = trajectory(PLACE_END, VIA, ANYGRASP)
        arm = FakeArm([catch, catch])
        with mock.patch.object(mpp.rospy, 'logwarn') as warn:
            self.assertTrue(guarded_named_move(arm, 'anygrasp', self.guard))
        self.assertEqual(arm.executed, [catch])
        self.assertEqual(arm.constraints, [])
        self.assertTrue(any('would refuse it (log only)' in str(c) for c in warn.call_args_list))

    def test_log_mode_without_a_plan_falls_back_to_go(self):
        self.mode = 'log'
        arm = FakeArm([])
        self.assertTrue(guarded_named_move(arm, 'home', self.guard))
        self.assertEqual(arm.gos, 1)

    def test_off_mode_is_the_old_go(self):
        self.mode = 'off'
        arm = FakeArm([trajectory(PLACE_END, VIA, ANYGRASP)])
        self.assertTrue(guarded_named_move(arm, 'home', self.guard))
        self.assertEqual((arm.gos, arm.executed), (1, []))

    def test_default_mode_enforce_in_sim_log_on_the_real_robot(self):
        self.mode = None
        self.assertEqual(mpp.named_move_guard_mode(), 'enforce')
        self.sim_time = False
        self.assertEqual(mpp.named_move_guard_mode(), 'log')

    def test_old_bool_param_and_unknown_values(self):
        for value, want in ((True, 'enforce'), (False, 'off'), ('LOG', 'log'), ('maybe', 'log')):
            self.mode = value
            self.assertEqual(mpp.named_move_guard_mode(), want)

    def test_without_guard_the_old_go(self):
        arm = FakeArm([])
        self.assertTrue(guarded_named_move(arm, 'home', None))
        self.assertEqual(arm.gos, 1)


if __name__ == '__main__':
    unittest.main()
