'''~plan_place_during_detour (careful, off in code): the closed-set untangle detour before a place is planned and
cable-checked up front and sent as one motion (StartMove, as the pick's start move), MTC plans the place meanwhile from
its end; the plan is used only when the arm arrives within tolerance (else planned again from the real state), and the
gripper check before placing runs after the arrival, before the first place motion. The insert, an open-set object and
the flag off keep the old pose-by-pose detour. Fake arm, action client and task; the real CableGuard (fixture of
test_cable_chain_guard); no ROS master; Mobipick image.'''
import unittest
from types import SimpleNamespace as NS

from moveit_msgs.msg import MoveItErrorCodes

import grasplan.mtc_pick_place as mpp
import grasplan.place as place
from test_cable_chain_guard import CLEAN
from test_start_move import POSES, Base, FakeArm, FakeClient, FakeMove, planner, solution_msg, trajectory

ARRIVED = (True, '')


def place_goal(*ids):
    return NS(attached_object_name='relay_1', place_locations=[NS(id=i, post_place_posture=None) for i in ids])


def place_planner(plans, held=True):
    '''an MtcPickPlace whose place planning and execution are stubs; plans: one bool per planned location in call
    order; log entries (what, detail, start move still set)'''
    m = planner()
    m.plans, m.log, m.held = list(plans), [], held
    m.update_cable_guard = lambda: None
    m.is_preempt_requested = lambda: False
    m.eef_to_attached_object = lambda name: 'eef to object'
    m.octomap_place_check = lambda name: None
    m.relaxed_retreat_min_distance = None
    m.make_place_task = lambda goal, location, *a: (f'task {location.id}', [mpp.NOOP, mpp.PREFIX, mpp.ARM])
    m.chosen_solution = NS(toMsg=solution_msg)
    m.plan = lambda task, what: m.log.append(('plan', what, m.start_move is not None)) or m.plans.pop(0)
    m.only_retreat_failed = lambda task: False
    m.log_place_pose = lambda location: None
    m.gripper_holds_object = lambda when, delay=None: m.log.append(('hold', when, m.start_move is not None)) or m.held
    m.release_empty_grasp = lambda name, posture: m.log.append(('release', name, m.start_move is not None))

    def execute(task, roles, *a, **kw):
        m.log.append(('execute', task, m.start_move is not None))
        return MoveItErrorCodes.SUCCESS
    m.execute = execute
    return m


class TestMtcPlaceDuringDetour(Base):
    def test_planned_meanwhile_checked_after_the_arrival_then_executed(self):
        m = place_planner([True])
        move = m.start_move = FakeMove([ARRIVED])
        result = m.place(place_goal('p0'))
        self.assertEqual(result.error_code.val, MoveItErrorCodes.SUCCESS)
        self.assertEqual(m.log, [('plan', 'place location p0 (1/1)', True), ('hold', 'before placing', False),
                                 ('execute', 'task p0', False)])
        self.assertEqual(len(move.plan_starts), 1)
        self.assertIsNotNone(move.plan_starts[0])   # the arrival is compared with where the planned place starts
        self.assertIsNone(m.start_move)

    def test_without_a_detour_motion_nothing_changes(self):
        m = place_planner([False, True])
        self.assertEqual(m.place(place_goal('p0', 'p1')).error_code.val, MoveItErrorCodes.SUCCESS)
        self.assertEqual(m.log, [('hold', 'before placing', False), ('plan', 'place location p0 (1/2)', False),
                                 ('plan', 'place location p1 (2/2)', False), ('execute', 'task p1', False)])

    def test_an_arrival_off_the_plan_plans_again_from_the_real_state(self):
        m = place_planner([True, True])
        m.start_move = FakeMove([(False, 'the arm stopped 1.00 deg from the end of the start move')])
        self.assertEqual(m.place(place_goal('p0')).error_code.val, MoveItErrorCodes.SUCCESS)
        self.assertEqual(m.log, [('plan', 'place location p0 (1/1)', True), ('hold', 'before placing', False),
                                 ('plan', 'place location p0 (1/1)', False), ('execute', 'task p0', False)])

    def test_an_object_lost_in_the_detour_fails_before_any_place_motion(self):
        m = place_planner([True], held=False)
        m.start_move = FakeMove([ARRIVED])
        result = m.place(place_goal('p0'))
        self.assertEqual(result.error_code.val, MoveItErrorCodes.FAILURE)
        self.assertEqual(m.log, [('plan', 'place location p0 (1/1)', True), ('hold', 'before placing', False),
                                 ('release', 'relay_1', False)])
        self.assertTrue(m.executed)
        self.assertIn('no longer in the gripper', m.failure_reason)

    def test_nothing_planned_meanwhile_waits_and_plans_again_from_the_real_state(self):
        m = place_planner([False, False, True])
        move = m.start_move = FakeMove([ARRIVED])
        self.assertEqual(m.place(place_goal('p0', 'p1')).error_code.val, MoveItErrorCodes.SUCCESS)
        self.assertEqual(move.arrivals, [])
        self.assertEqual(m.log, [('plan', 'place location p0 (1/2)', True), ('plan', 'place location p1 (2/2)', True),
                                 ('hold', 'before placing', False), ('plan', 'place location p0 (1/2)', False),
                                 ('execute', 'task p0', False)])

    def test_a_preempt_during_the_detour_waits_for_it(self):
        m = place_planner([True])
        m.is_preempt_requested = lambda: True
        move = m.start_move = FakeMove([ARRIVED])
        self.assertEqual(m.place(place_goal('p0')).error_code.val, MoveItErrorCodes.PREEMPTED)
        self.assertEqual((move.arrivals, m.start_move), ([], None))
        self.assertEqual(m.log, [])   # no hold check, no plan, nothing executed

    def test_an_error_before_planning_waits_for_the_detour(self):
        m = place_planner([True])

        def no_transform(name):
            raise ValueError('relay_1 is not attached')
        m.eef_to_attached_object = no_transform
        move = m.start_move = FakeMove([ARRIVED])
        self.assertEqual(m.place(place_goal('p0')).error_code.val, MoveItErrorCodes.PLANNING_FAILED)
        self.assertEqual((move.arrivals, m.start_move), ([], None))


class TestPlaceToolsDetour(Base):
    def tools(self, open_set=False):
        p = place.PlaceTools.__new__(place.PlaceTools)
        p.robot = NS(arm=FakeArm([trajectory(*CLEAN), trajectory(*CLEAN)]))
        p.mtc = NS(update_cable_guard=lambda: None, cable_guard=self.guard, execute_client=FakeClient(), start_move=None,
                   cable_rejections=0)
        p.detour_state = ''
        p.picked_open_set = lambda name: open_set
        p.moved = []
        p.move_arm_to_posture = lambda name: p.moved.append(name) or True
        return p

    def flag_on(self):
        self.params = {'~plan_place_during_detour': True}
        self.addCleanup(setattr, self, 'params', {})

    def test_off_by_default_the_old_pose_by_pose_detour(self):
        p = self.tools()
        p.go_before_place('relay_1', POSES, plan_meanwhile=True)
        self.assertEqual(p.moved, POSES)
        self.assertIsNone(p.mtc.start_move)
        self.assertEqual(p.mtc.execute_client.goals, [])

    def test_on_the_detour_is_sent_as_one_motion_and_handed_to_mtc(self):
        self.flag_on()
        p = self.tools()
        p.go_before_place('relay_1', POSES, plan_meanwhile=True)
        self.assertEqual(p.moved, [])
        self.assertEqual((p.mtc.start_move.names, p.mtc.start_move.what), (POSES, 'place'))
        p.mtc.start_move.join(5.0)
        self.assertEqual(len(p.mtc.execute_client.goals), 2)   # one goal per pose, each its own trajectory start

    def test_a_forced_detour_counts_as_done(self):
        self.flag_on()
        p = self.tools()
        p.detour_state = 'forced'
        p.go_before_place('relay_1', POSES, plan_meanwhile=True)
        self.assertEqual(p.detour_state, 'done')
        self.assertIsNotNone(p.mtc.start_move)

    def test_the_insert_keeps_the_old_way(self):
        self.flag_on()
        p = self.tools()
        p.go_before_place('relay_1', POSES)   # the insert node calls it without plan_meanwhile
        self.assertEqual((p.moved, p.mtc.start_move), (POSES, None))

    def test_an_open_set_object_keeps_its_one_move(self):
        self.flag_on()
        p = self.tools(open_set=True)
        p.go_before_place('cup_1', POSES, plan_meanwhile=True)
        self.assertEqual((p.moved, p.mtc.start_move), (['transport'], None))

    def test_a_pose_that_cannot_be_planned_and_checked_moves_the_old_way(self):
        self.flag_on()
        p = self.tools()
        p.robot.arm.plans = [trajectory(*CLEAN)]   # the second pose has no plan
        p.go_before_place('relay_1', POSES, plan_meanwhile=True)
        self.assertEqual((p.moved, p.mtc.start_move), (POSES, None))

    def test_without_mtc_the_old_way(self):
        self.flag_on()
        p = self.tools()
        p.mtc = None
        self.assertFalse(p.detour_while_planning(POSES))

    def test_a_detour_left_running_is_waited_for(self):
        p = self.tools()
        move = FakeMove([ARRIVED])
        p.mtc.start_move = move
        p.mtc.finish_start_move = lambda: (setattr(p.mtc, 'start_move', None), move.wait())[1]
        p.finish_detour_move()
        self.assertEqual((move.arrivals, p.mtc.start_move), ([], None))


if __name__ == '__main__':
    unittest.main()
