'''~grasp_pinch_retry_next (off in code): after a pinch verdict an open-set MTC pick backs the arm out and offers the
grasps ranked after the pinched one, at most ~grasp_pinch_retry_max more times; off, or after any other failure, the pick
ends as before. Fake MTC and scene (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from geometry_msgs.msg import Vector3Stamped
from moveit_msgs.msg import CollisionObject, GripperTranslation, MoveItErrorCodes

import grasplan.mtc_pick_place as mpp
import grasplan.pick as pick

SUCCESS, FAILURE, UNPLANNED = MoveItErrorCodes.SUCCESS, MoveItErrorCodes.FAILURE, MoveItErrorCodes.PLANNING_FAILED


def grasps(count):
    return [NS(id=f'g{i}', grasp_quality=1.0 - i / 100.0) for i in range(1, count + 1)]


class FakeMtc:
    '''plays one outcome per pickup: "pinch", "nothing" (closed on nothing), "ok" or "unplanned" (nothing moves)'''

    def __init__(self, outcomes, backing_out=''):
        self.outcomes, self.backing_out = list(outcomes), backing_out
        self.executed, self.executed_grasp, self.executed_offer_id = False, None, None
        self.failure_reason, self.pinch_verdict = '', False
        self.cable_guard, self.offers, self.left = None, [], []

    def pickup(self, goal):
        offered = goal.possible_grasps
        self.offers.append([g.id for g in offered])
        outcome = self.outcomes.pop(0)
        self.executed, self.pinch_verdict = outcome != 'unplanned', outcome == 'pinch'
        self.executed_grasp = NS(id=offered[0].id + '_turned') if self.executed else None
        self.executed_offer_id = offered[0].id if self.executed else self.executed_offer_id
        self.failure_reason = {'pinch': f'pinch at {offered[0].id}', 'nothing': f'closed on nothing at {offered[0].id}',
                               'ok': '', 'unplanned': f'none of the {len(offered)} grasps could be planned'}[outcome]
        code = {'ok': SUCCESS, 'unplanned': UNPLANNED}.get(outcome, FAILURE)
        return NS(error_code=NS(val=code), grasp=offered[0])

    def leave_pinched_grasp(self, grasp, scene_object):
        self.left.append((grasp.id, scene_object))
        return self.backing_out


class OfferAfterPinch(unittest.TestCase):
    def setUp(self):
        self.params = {'~grasp_pinch_retry_next': True}
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.preempt = False
        self.box = CollisionObject(id='box_1')

    def tools(self, outcomes, batch=0, passing=99, backing_out=''):
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc = FakeMtc(outcomes, backing_out)
        p.mtc.cable_guard = object()
        p.robot = NS(arm='arm')
        p.pick_action_server = NS(is_preempt_requested=lambda: self.preempt)
        p.open_set_pick, p.detoured = True, ''
        p.arm_group_name, p.planning_time = 'arm', 5.0
        p.external_grasp_batch_size, p.cable_gate_passing = batch, passing
        self.removed = []
        p.remove_from_scene = self.removed.append
        self.start_moves, self.start_move_ok = [], True

        def start_move(pose):
            self.start_moves.append((pose, len(p.mtc.left), len(p.mtc.offers)))
            return self.start_move_ok
        p.move_arm_to_pick_start = start_move
        p.scene = NS(get_objects=lambda names: {'box_1': self.box})
        return p

    def offer(self, p, count=6, external=True):
        target = NS(get_object_class_and_id_as_string=lambda: 'box_1')
        return p.offer_grasps(target, grasps(count), 'table_1', external)

    def test_off_in_code_a_pinch_ends_the_pick_without_reading_the_scene(self):
        self.params = {}
        p = self.tools(['pinch'])
        p.scene = None
        self.assertEqual(self.offer(p), FAILURE)
        self.assertEqual(p.mtc.offers, [['g1', 'g2', 'g3', 'g4', 'g5', 'g6']])
        self.assertEqual(p.mtc.left, [])
        self.assertEqual(p.mtc.failure_reason, 'pinch at g1')

    def test_a_closed_set_pick_is_not_retried(self):
        p = self.tools(['pinch'])
        p.scene = None
        self.assertEqual(self.offer(p, external=False), FAILURE)
        self.assertEqual(len(p.mtc.offers), 1)

    def test_the_next_grasp_is_tried_after_backing_out_with_the_perceived_box(self):
        p = self.tools(['pinch', 'ok'])
        self.assertEqual(self.offer(p), SUCCESS)
        self.assertEqual(p.mtc.offers, [['g1', 'g2', 'g3', 'g4', 'g5', 'g6'], ['g2', 'g3', 'g4', 'g5', 'g6']])
        self.assertEqual(p.mtc.left, [('g1_turned', self.box)])   # the executed copy backs out, the offer goes on after g1
        self.assertEqual(self.removed, [])
        self.assertEqual(p.mtc.failure_reason, '')

    def test_the_arm_returns_to_the_pick_start_after_backing_out_and_before_the_next_offer(self):
        p = self.tools(['pinch', 'ok'])
        self.offer(p)
        self.assertEqual(self.start_moves, [('transport', 1, 1)])   # one back-out and one offer before it, none after

    def test_the_pick_start_pose_is_the_pick_s_own(self):
        self.params['~open_set_pick_start_pose'] = 'observe100cm_right'
        p = self.tools(['pinch', 'ok'])
        self.offer(p)
        self.assertEqual(self.start_moves[0][0], 'observe100cm_right')

    def test_an_empty_pick_start_pose_stays_where_it_backed_out_to(self):
        self.params['~open_set_pick_start_pose'] = ''
        p = self.tools(['pinch', 'ok'])
        self.assertEqual(self.offer(p), SUCCESS)
        self.assertEqual(self.start_moves, [])

    def test_a_failed_return_ends_it_with_the_reason(self):
        p = self.tools(['pinch'])
        self.start_move_ok = False
        p.pick_start_refused = 'chain replay stops'
        self.assertEqual(self.offer(p), FAILURE)
        self.assertEqual(len(p.mtc.offers), 1)
        self.assertEqual(p.mtc.failure_reason,
                         'pinch at g1; no other grasp tried: the arm could not return to transport (chain replay stops)')

    def test_no_return_when_the_arm_cannot_back_out(self):
        p = self.tools(['pinch'], backing_out='backing out could not be planned (stage failed)')
        self.offer(p)
        self.assertEqual(self.start_moves, [])

    def test_it_goes_on_in_the_batch_after_the_pinched_one(self):
        p = self.tools(['unplanned', 'pinch', 'ok'], batch=2)
        self.assertEqual(self.offer(p), SUCCESS)
        self.assertEqual(p.mtc.offers, [['g1', 'g2'], ['g3', 'g4'], ['g4', 'g5']])

    def test_the_cable_gate_keeps_its_place_in_the_shorter_offer(self):
        p = self.tools(['pinch', 'ok'], passing=3)
        self.assertEqual(self.offer(p), SUCCESS)
        self.assertEqual(p.cable_gate_passing, 2)
        self.assertEqual(p.mtc.offers, [['g1', 'g2', 'g3'], ['g2', 'g3']])

    def test_a_retry_never_offers_the_grasps_held_back_as_predicted_to_fail(self):
        p = self.tools(['pinch', 'unplanned'], passing=3)
        self.assertEqual(self.offer(p), UNPLANNED)
        self.assertEqual(p.mtc.offers, [['g1', 'g2', 'g3'], ['g2', 'g3']])
        self.assertEqual(p.mtc.left, [('g1_turned', self.box)])

    def test_no_untried_grasp_predicted_to_pass_ends_the_pick_without_any_retry_motion(self):
        p = self.tools(['pinch'], passing=1)
        self.assertEqual(self.offer(p), FAILURE)
        self.assertEqual(p.mtc.offers, [['g1']])
        self.assertEqual(p.mtc.left, [])
        self.assertEqual(self.start_moves, [])
        self.assertEqual(p.mtc.failure_reason,
                         'pinch at g1; no other grasp tried: no untried grasp is predicted to pass the cable guard')

    def test_without_any_prediction_the_pick_ends_after_the_pinch(self):
        p = self.tools(['pinch'], passing=0)
        self.assertEqual(self.offer(p), FAILURE)
        self.assertEqual(p.mtc.left, [])
        self.assertEqual(len(p.mtc.offers), 1)

    def test_with_the_cable_plan_check_off_a_pinch_ends_the_pick_as_with_the_switch_off(self):
        p = self.tools(['pinch'])
        p.mtc.cable_guard = None
        logged = []
        with mock.patch.object(pick.rospy, 'logwarn', logged.append):
            self.assertEqual(self.offer(p), FAILURE)
        self.assertEqual(len(p.mtc.offers), 1)
        self.assertEqual(p.mtc.left, [])
        self.assertEqual(self.start_moves, [])
        self.assertEqual(p.mtc.failure_reason, 'pinch at g1')
        self.assertEqual(logged, ['pinch retry of box_1 skipped: the arm cable plan check is off'])

    def test_every_retry_counts_and_the_limit_ends_it_with_each_pinch_named(self):
        p = self.tools(['pinch', 'pinch', 'pinch'])
        self.assertEqual(self.offer(p), FAILURE)
        self.assertEqual(len(p.mtc.offers), 3)
        self.assertEqual(len(p.mtc.left), 2)
        self.assertEqual(p.mtc.failure_reason, 'pinch at g1; pinch at g2; pinch at g3; no other grasp tried: '
                                               '~grasp_pinch_retry_max (2) reached')

    def test_the_limit_is_a_parameter(self):
        self.params['~grasp_pinch_retry_max'] = 0
        p = self.tools(['pinch'])
        self.assertEqual(self.offer(p), FAILURE)
        self.assertEqual(p.mtc.left, [])

    def test_any_other_failure_of_a_retry_ends_the_pick_and_names_the_pinch(self):
        p = self.tools(['pinch', 'nothing'])
        self.assertEqual(self.offer(p), FAILURE)
        self.assertEqual(len(p.mtc.offers), 2)
        self.assertEqual(p.mtc.failure_reason, 'pinch at g1; closed on nothing at g2')
        self.assertEqual(self.removed, [])   # the failed close took it out of the scene itself

    def test_any_other_first_failure_is_not_retried(self):
        p = self.tools(['nothing'])
        self.assertEqual(self.offer(p), FAILURE)
        self.assertEqual(p.mtc.left, [])
        self.assertEqual(p.mtc.failure_reason, 'closed on nothing at g1')

    def test_a_retry_that_plans_nothing_still_counts_as_moved_and_the_object_leaves_the_scene(self):
        p = self.tools(['pinch', 'unplanned'])
        self.assertEqual(self.offer(p), UNPLANNED)
        self.assertTrue(p.mtc.executed)   # so the caller retracts and no fallback runs as if the arm never moved
        self.assertFalse(p.arm_did_not_move(UNPLANNED))
        self.assertEqual(p.mtc.failure_reason, 'pinch at g1; none of the 5 grasps could be planned')
        self.assertEqual(self.removed, ['box_1'])

    def test_no_grasp_left(self):
        p = self.tools(['pinch'])
        self.assertEqual(self.offer(p, count=1), FAILURE)
        self.assertEqual(p.mtc.left, [])
        self.assertEqual(p.mtc.failure_reason, 'pinch at g1; no other grasp tried: no grasp left in the offer')

    def test_a_pinched_grasp_outside_the_offer_ends_it_with_the_reason(self):
        p = self.tools(['pinch'])
        pickup = p.mtc.pickup

        def pickup_of_another_id(goal):
            result = pickup(goal)
            p.mtc.executed_offer_id = 'other'
            return result
        p.mtc.pickup = pickup_of_another_id
        self.assertEqual(self.offer(p), FAILURE)
        self.assertEqual(p.mtc.left, [])
        self.assertIn('the pinched grasp other is not in the offer', p.mtc.failure_reason)

    def test_an_arm_that_cannot_back_out_ends_it_with_the_reason(self):
        p = self.tools(['pinch'], backing_out='backing out could not be planned (stage failed)')
        self.assertEqual(self.offer(p), FAILURE)
        self.assertEqual(len(p.mtc.offers), 1)
        self.assertTrue(p.mtc.executed)
        self.assertEqual(p.mtc.failure_reason,
                         'pinch at g1; no other grasp tried: backing out could not be planned (stage failed)')

    def test_an_object_not_in_the_scene_is_not_retried(self):
        p = self.tools(['pinch'])
        p.scene = NS(get_objects=lambda names: {})
        self.assertEqual(self.offer(p), FAILURE)
        self.assertEqual(p.mtc.left, [])
        self.assertIn('box_1 was not in the planning scene', p.mtc.failure_reason)

    def test_a_cancel_stops_the_retry(self):
        p = self.tools(['pinch'])
        self.preempt = True
        self.offer(p)
        self.assertEqual(p.mtc.left, [])


class LeavePinchedGrasp(unittest.TestCase):
    def setUp(self):
        self.params = {}
        patcher = mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        patcher = mock.patch.object(mpp, 'stages')
        patcher.start()
        self.addCleanup(patcher.stop)
        self.events = []
        patcher = mock.patch.object(mpp, 'announce_trajectory_start', self.events.append)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.grasp = NS(id='g1', pre_grasp_approach=GripperTranslation(
            direction=Vector3Stamped(), min_distance=0.05, desired_distance=0.12))
        self.grasp.pre_grasp_approach.direction.header.frame_id = 'tcp'
        self.grasp.pre_grasp_approach.direction.vector.x = 1.0
        self.box = CollisionObject(id='box_1', operation=CollisionObject.REMOVE)

    def mtc(self, holds=False, planned=True, code=SUCCESS):
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.gripper_fact, m.allow_octomap_contact, m.gripper_links = object(), True, ['pad']
        m.gripper_holds_object = lambda when: holds
        self.task, self.moves, self.applied, self.ran = mock.Mock(), [], [], []
        m.start_task = lambda name: (self.task, [mpp.NOOP])

        def move_relative(name, translation):
            self.moves.append(translation)
            return name
        m.move_relative = move_relative
        m.last_plan_failure = 'stage back out failed'
        m.plan = lambda task, what: planned
        arm = NS(trajectory=NS(joint_trajectory=NS(points=[1, 2])))
        none = NS(trajectory=NS(joint_trajectory=NS(points=[])))
        m.chosen_solution = NS(toMsg=lambda: NS(sub_trajectory=[none, none, arm]))
        m.execute_arm = lambda trajectory: self.events.append('arm') or self.ran.append(trajectory) or code
        m.execution_text = lambda c: f'code {c}'
        m.apply_planning_scene_srv = self.applied.append
        return m

    def test_it_backs_out_along_the_reverse_approach_and_restores_the_object(self):
        self.assertEqual(self.mtc().leave_pinched_grasp(self.grasp, self.box), '')
        self.assertEqual(self.moves[0].direction.vector.x, -1.0)
        self.assertEqual(self.moves[0].direction.header.frame_id, 'tcp')
        self.assertEqual((self.moves[0].min_distance, self.moves[0].desired_distance), (0.05, 0.12))
        self.assertEqual(self.grasp.pre_grasp_approach.direction.vector.x, 1.0)   # the grasp itself stays as it was
        self.assertEqual(len(self.ran), 1)
        self.assertEqual(self.events, ['back out of a pinch', 'arm'])   # the monitor starts its chain afresh, as the replay did
        self.assertTrue(self.applied[0].is_diff)
        self.assertEqual(self.applied[0].world.collision_objects, [self.box])
        self.assertEqual(self.box.operation, CollisionObject.ADD)

    def test_an_approach_that_cannot_be_turned_into_a_motion_is_a_reason(self):
        m = self.mtc()

        def refuse(name, translation):
            raise ValueError('gripper translation without direction')
        m.move_relative = refuse
        self.assertIn('without direction', m.leave_pinched_grasp(self.grasp, self.box))
        self.assertEqual((self.ran, self.applied), ([], []))

    def test_a_gripper_still_holding_does_not_move(self):
        reason = self.mtc(holds=True).leave_pinched_grasp(self.grasp, self.box)
        self.assertIn('still reports the object', reason)
        self.assertEqual((self.ran, self.applied), ([], []))

    def test_no_gripper_check_is_no_verdict(self):
        self.params['~mtc_check_grasp'] = False
        self.assertEqual(self.mtc(holds=True).leave_pinched_grasp(self.grasp, self.box), '')

    def test_a_refused_plan_does_not_move_and_keeps_the_object_out(self):
        reason = self.mtc(planned=False).leave_pinched_grasp(self.grasp, self.box)
        self.assertIn('stage back out failed', reason)
        self.assertEqual((self.ran, self.applied), ([], []))

    def test_a_failed_motion_keeps_the_object_out(self):
        reason = self.mtc(code=FAILURE).leave_pinched_grasp(self.grasp, self.box)
        self.assertEqual(reason, f'backing out failed: code {FAILURE}')
        self.assertEqual(self.applied, [])

    def test_a_scene_service_error_is_a_reason(self):
        m = self.mtc()

        def refuse(diff):
            raise mpp.rospy.ServiceException('down')
        m.apply_planning_scene_srv = refuse
        self.assertIn('could not be put back', m.leave_pinched_grasp(self.grasp, self.box))


if __name__ == '__main__':
    unittest.main()
