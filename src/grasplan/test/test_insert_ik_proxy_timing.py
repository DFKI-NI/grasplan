'''Where the insert order's time goes, and one compute_ik connection for all its poses: the IK calls go over one kept
(persistent) connection, opened again once when it breaks; the TIMING line splits insert_order into the IK calls that
found a solution, those that did not (each with its count) and the cable prediction. The chain replay log line says
whether its fresh start came from the cable model's start-pose cache. Log and connection only, no behaviour change, no
flag. Fake compute_ik and cable model for the insert; the real cable model for the replay line; no ROS master;
Mobipick image.'''
import copy
import os
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import rospy
import yaml
from moveit_msgs.msg import MoveItErrorCodes

import grasplan.insert as insert
import grasplan.mtc_pick_place as mpp
from grasplan.tools.stage_timing import TIMER

W3 = 'mobipick/ur5_wrist_3_joint'


def solved(w3):
    return NS(error_code=NS(val=MoveItErrorCodes.SUCCESS), solution=NS(joint_state=NS(name=[W3], position=[w3])))


UNSOLVED = NS(error_code=NS(val=MoveItErrorCodes.NO_IK_SOLUTION), solution=NS(joint_state=NS(name=[], position=[])))


class FakeProxy:
    '''a compute_ik proxy: answers (a response or an exception to raise) in call order'''
    def __init__(self, answers):
        self.answers, self.calls, self.closed = list(answers), 0, False

    def wait_for_service(self, timeout):
        pass

    def __call__(self, request):
        self.calls += 1
        answer = self.answers.pop(0)
        if isinstance(answer, Exception):
            raise answer
        return answer

    def close(self):
        self.closed = True


class InsertIkTest(unittest.TestCase):
    def setUp(self):
        self.params = {}
        patcher = mock.patch.object(insert.rospy, 'get_param',
                                    lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.tool = insert.InsertTools.__new__(insert.InsertTools)
        self.tool.tcp_frame = 'mobipick/gripper_tcp'
        start = NS(joint_state=NS(name=[W3], position=[2.0]))
        self.evaluated = []
        model = NS(evaluate=lambda q: self.evaluated.append(q) or NS(max_stretch=-0.2))
        mtc = NS(cable_guard=NS(model=model, max_stretch=-0.115), update_cable_guard=lambda: None)
        self.tool.place = NS(robot=NS(get_current_state=lambda: start), mtc=mtc, group_name='arm',
                             global_reference_frame='map')
        self.fresh = []   # the proxies opened again, in order

        def new_proxy():
            proxy = self.fresh.pop(0)
            self.opened.append(proxy)
            return proxy
        self.opened = []
        self.tool.new_compute_ik_proxy = new_proxy

    def bodies(self, n):
        out = []
        for i in range(n):
            body = np.eye(4)
            body[0, 3] = i + 1
            out.append(body)
        return out

    def order(self, ranked):
        return [int(body[0, 3]) for body in ranked]


class TestPersistentProxy(InsertIkTest):
    def test_the_node_keeps_one_connection(self):
        proxy = insert.InsertTools.new_compute_ik_proxy()
        self.assertTrue(proxy.persistent)
        self.assertEqual(proxy.resolved_name, '/compute_ik')

    def test_all_poses_go_over_the_same_connection(self):
        proxy = self.tool.compute_ik_srv = FakeProxy([solved(2.3), solved(2.1), solved(1.95)])
        self.assertEqual(self.order(self.tool.sort_by_wrist_3_change(self.bodies(3), np.eye(4))), [3, 2, 1])
        self.assertEqual((proxy.calls, proxy.closed, self.opened), (3, False, []))

    def test_a_broken_connection_is_opened_again_once_and_the_order_goes_on(self):
        broken = self.tool.compute_ik_srv = FakeProxy(
            [solved(2.3), rospy.exceptions.TransportTerminated('connection closed by move_group')])
        again = FakeProxy([solved(2.1), solved(1.95)])
        self.fresh = [again]
        self.assertEqual(self.order(self.tool.sort_by_wrist_3_change(self.bodies(3), np.eye(4))), [3, 2, 1])
        self.assertTrue(broken.closed)
        self.assertIs(self.tool.compute_ik_srv, again)
        self.assertEqual((broken.calls, again.calls), (2, 2))

    def test_a_second_failure_leaves_the_poses_unsorted(self):
        self.tool.compute_ik_srv = FakeProxy([solved(2.3), rospy.ServiceException('transport error')])
        last = FakeProxy([rospy.ServiceException('unable to connect to service')])
        self.fresh = [last, FakeProxy([])]
        self.assertEqual(self.order(self.tool.sort_by_wrist_3_change(self.bodies(3), np.eye(4))), [1, 2, 3])
        self.assertEqual(len(self.opened), 2)   # the next insert starts on a fresh connection
        self.assertTrue(last.closed)


class TestInsertOrderTiming(InsertIkTest):
    def setUp(self):
        super().setUp()
        TIMER.begin('insert', 'relay_1')
        self.addCleanup(TIMER.end, False)

    def test_ik_solved_unsolved_and_the_cable_prediction_each_with_their_count(self):
        self.params['~insert_cable_gate'] = True
        self.tool.compute_ik_srv = FakeProxy([solved(2.3), UNSOLVED, solved(2.1), UNSOLVED, UNSOLVED])
        self.tool.sort_by_wrist_3_change(self.bodies(5), np.eye(4))
        self.assertEqual(TIMER.counts, {'insert_order/ik_solved': 2, 'insert_order/ik_unsolved': 3,
                                        'insert_order/cable_prediction': 2, 'insert_order': 1})
        self.assertTrue(self.evaluated)
        self.assertGreaterEqual(TIMER.totals['insert_order'],
                                sum(v for k, v in TIMER.totals.items() if k.startswith('insert_order/')))
        line = TIMER.end(True)
        self.assertIn(' insert_order/ik_solved=', line)
        self.assertIn(' insert_order/ik_solved#=2 ', line)
        self.assertIn(' insert_order/ik_unsolved#=3 ', line)
        self.assertIn(' insert_order/cable_prediction#=2 ', line)

    def test_without_the_gate_no_cable_prediction_stage(self):
        self.tool.compute_ik_srv = FakeProxy([solved(2.3), UNSOLVED])
        self.tool.sort_by_wrist_3_change(self.bodies(2), np.eye(4))
        self.assertEqual(TIMER.counts, {'insert_order/ik_solved': 1, 'insert_order/ik_unsolved': 1, 'insert_order': 1})
        self.assertEqual(self.evaluated, [])


class TestReplayCacheWord(unittest.TestCase):
    def test_the_word(self):
        self.assertEqual(mpp.reset_cache_word((3, 4), (4, 4)), ' (start-pose cache hit)')
        self.assertEqual(mpp.reset_cache_word((3, 4), (3, 5)), ' (start-pose cache miss)')
        self.assertEqual(mpp.reset_cache_word((3, 4), (3, 4)), '')   # cache off or the live chain
        self.assertEqual(mpp.reset_cache_word(None, None), '')

    def test_a_model_without_the_counters_has_none(self):
        self.assertIsNone(mpp.reset_cache_counts(NS()))
        self.assertIsNone(mpp.reset_cache_counts(mock.Mock()))
        self.assertEqual(mpp.reset_cache_counts(NS(reset_cache_hits=2, reset_cache_misses=5)), (2, 5))

    def test_the_replay_line_says_miss_then_hit_with_the_real_model(self):
        from mobipick_cable_entanglement.cable_model import CableModel, config_from_dict
        import rospkg
        from test_cable_chain_guard import CLEAN, solution
        pkg = rospkg.RosPack().get_path('mobipick_cable_entanglement')
        with open(os.path.join(pkg, 'config', 'cable_model.yaml')) as f:
            cfg = config_from_dict(yaml.safe_load(f))
        with open(os.path.join(pkg, 'test', 'mobipick_sim_robot_description.urdf')) as f:
            urdf = f.read()
        cfg = copy.deepcopy(cfg)
        cfg.cable_carrier.enabled = True
        guard = mpp.CableGuard.__new__(mpp.CableGuard)
        guard.cfg, guard.chain_rate, guard.replay_live_state = cfg, 10.0, False
        lines = {}
        for mrad in (0.0, 5.0):
            cfg.cable_carrier.link_chain.reset_cache_mrad = mrad
            guard.chain_model = CableModel(urdf, cfg)
            with mock.patch.object(mpp.rospy, 'loginfo') as log:
                for _ in range(2):
                    guard.chain_verdict(solution(*CLEAN))
            lines[mrad] = [c.args[0] for c in log.call_args_list if 'chain replay of' in c.args[0]]
        self.assertEqual(len(lines[5.0]), 2)
        self.assertIn('from a fresh chain (start-pose cache miss): max stretch', lines[5.0][0])
        self.assertIn('from a fresh chain (start-pose cache hit): max stretch', lines[5.0][1])
        self.assertTrue(all('cache' not in line and 'from a fresh chain: max stretch' in line for line in lines[0.0]))


if __name__ == '__main__':
    unittest.main()
