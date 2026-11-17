'''The insert order asks compute_ik once per candidate pose with ~insert_order_ik_timeout per call (TRAC-IK in Distance
mode spends its whole timeout on every call, found or not): 0.05 s when the parameter is not set (as before), the
parameter's value when it is (the safe feature flag value is 0.01), read again for every insert, never below 1 ms.
Fake compute_ik, no ROS master; Mobipick image.'''
import unittest

import numpy as np

import grasplan.insert as insert
from test_insert_ik_proxy_timing import UNSOLVED, InsertIkTest, solved


class TimeoutProxy:
    '''a compute_ik proxy that keeps the timeout of every request'''
    def __init__(self, answers):
        self.answers, self.timeouts = list(answers), []

    def wait_for_service(self, timeout):
        pass

    def close(self):
        pass

    def __call__(self, request):
        self.timeouts.append(request.ik_request.timeout.to_sec())
        return self.answers.pop(0)


class TestInsertOrderIkTimeout(InsertIkTest):
    def sort(self, answers):
        proxy = self.tool.compute_ik_srv = TimeoutProxy(answers)
        order = self.order(self.tool.sort_by_wrist_3_change(self.bodies(len(answers)), np.eye(4)))
        return order, proxy.timeouts

    def test_unset_keeps_50_ms_per_call(self):
        order, timeouts = self.sort([solved(2.3), UNSOLVED, solved(2.1)])
        self.assertEqual(timeouts, [0.05, 0.05, 0.05])
        self.assertEqual(order, [3, 1, 2])

    def test_the_flag_value_goes_into_every_call(self):
        self.params['~insert_order_ik_timeout'] = 0.01
        order, timeouts = self.sort([solved(2.3), UNSOLVED, solved(2.1)])
        self.assertEqual(timeouts, [0.01, 0.01, 0.01])
        self.assertEqual(order, [3, 1, 2])   # the same answers give the same order: only the time per call changes

    def test_read_again_for_every_insert(self):
        self.params['~insert_order_ik_timeout'] = 0.01
        _, first = self.sort([solved(2.3), UNSOLVED])
        del self.params['~insert_order_ik_timeout']
        _, second = self.sort([solved(2.3), UNSOLVED])
        self.assertEqual((first, second), ([0.01, 0.01], [0.05, 0.05]))

    def test_never_below_1_ms(self):
        for value in (0.0, -1.0):
            self.params['~insert_order_ik_timeout'] = value
            self.assertEqual(insert.insert_order_ik_timeout(), 0.001)
        self.params['~insert_order_ik_timeout'] = '0.02'   # a string from a launch arg without type
        self.assertEqual(insert.insert_order_ik_timeout(), 0.02)


if __name__ == '__main__':
    unittest.main()
