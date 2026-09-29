'''#198 (f): the open-set IK pre-filter asks compute_ik with ~external_grasp_ik_timeout per call (TRAC-IK Distance always
spends its whole timeout: 0.1 s x up to 2 calls x ~46 grasps = 3.4-5.3 s per pick). Default 0.01 s in the sim, 0.1 s on
the real robot until tested there. Fake compute_ik (no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from moveit_msgs.msg import MoveItErrorCodes

import grasplan.pick as pick


class FakeIK:
    def __init__(self, reachable):
        self.reachable, self.timeouts = reachable, []

    def wait_for_service(self, timeout):
        pass

    def close(self):
        pass

    def __call__(self, request):
        self.timeouts.append(request.ik_request.timeout.to_sec())
        ok = request.ik_request.pose_stamped in self.reachable
        return NS(error_code=NS(val=MoveItErrorCodes.SUCCESS if ok else MoveItErrorCodes.NO_IK_SOLUTION),
                  solution=NS(joint_state=NS(name=[], position=[])))


class TestIkPrefilterTimeout(unittest.TestCase):
    def setUp(self):
        self.params = {}
        self.ik = FakeIK(reachable=['pose_a'])
        for target, name, value in ((pick.rospy, 'get_param', lambda n, d=None: self.params.get(n, d)),
                                    (pick.rospy, 'ServiceProxy', lambda *a, **k: self.ik)):
            patcher = mock.patch.object(target, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        p = pick.PickTools.__new__(pick.PickTools)
        p.mtc = None
        p.external_grasp_ik_prefilter = True
        p.external_grasp_max_attempts = 0
        p.cable_gate_passing = 0
        p.arm_group_name = 'arm'
        p.robot = NS(get_current_state=lambda: NS(joint_state=NS(name=[], position=[])),
                     arm=NS(get_end_effector_link=lambda: 'mobipick/gripper_tcp'))
        self.p = p
        self.grasps = [NS(id='anygrasp_1', grasp_pose='pose_a'), NS(id='anygrasp_2', grasp_pose='pose_b')]

    def test_sim_default_10_ms(self):
        self.params['/use_sim_time'] = True
        kept = self.p.reachable_external_grasps(self.grasps)
        self.assertEqual([g.id for g in kept], ['anygrasp_1'])
        self.assertEqual(self.ik.timeouts, [0.01, 0.01])

    def test_real_robot_default_as_before(self):
        self.p.reachable_external_grasps(self.grasps)
        self.assertEqual(self.ik.timeouts, [0.1, 0.1])

    def test_param(self):
        self.params['~external_grasp_ik_timeout'] = 0.005
        self.p.reachable_external_grasps(self.grasps)
        self.assertEqual(self.ik.timeouts, [0.005, 0.005])


if __name__ == '__main__':
    unittest.main()
