'''open-set grasps moved back for the table clearance, ordered by how far they were moved back when
~open_set_prefer_less_backoff is on (rig, heavy lying can, approach 39 deg off vertical: moved back 0.9 cm held, 1.3 cm
dropped). Fake planning scene and fingertip envelope (no ROS master); needs the Mobipick image for the module imports.'''
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Grasp

import grasplan.pick as pick

TOP = 0.72   # table top in the fake scene


def grasp(gid, tilt_deg, z):
    '''a grasp whose approach (TCP +x) is tilt_deg off straight down, TCP at height z'''
    g = Grasp()
    g.id = gid
    g.grasp_pose = PoseStamped()
    g.grasp_pose.header.frame_id = 'map'
    half = math.radians(90.0 - tilt_deg) / 2.0
    g.grasp_pose.pose.orientation.y, g.grasp_pose.pose.orientation.w = math.sin(half), math.cos(half)
    g.grasp_pose.pose.position.z = z
    return g


class FakeScene:
    def get_objects(self, names):
        table = NS(header=NS(frame_id='map'), pose=Pose(), primitives=[NS(dimensions=[1.0, 1.0, TOP])],
                   primitive_poses=[Pose()])
        table.pose.position.z = TOP / 2.0
        return {'table_2': table}


def picker():
    p = pick.PickTools.__new__(pick.PickTools)
    p.scene = FakeScene()
    p.fingertip_envelope = lambda: [(0.0, np.array([[0.03, 0.0, 0.0]]))]   # one tip 3 cm ahead of the TCP
    return p


# 39 deg: the tip is 0.03 * cos 39 = 2.33 cm below the TCP, so the TCP needs TOP + 3.83 cm (fingertips 1.5 cm)
NEED = 0.015 + 0.03 * math.cos(math.radians(39.0))


def three_grasps():
    return [grasp('back13', 39.0, TOP + NEED - 0.010),   # 1.0 cm too low -> 1.29 cm back
            grasp('back09', 39.0, TOP + NEED - 0.007),   # 0.7 cm too low -> 0.90 cm back
            grasp('none', 39.0, TOP + 0.06)]             # high enough


class TestPreferLessBackoff(unittest.TestCase):
    def setUp(self):
        self.params = {'~open_set_min_grasp_height': 0.035, '~open_set_min_fingertip_height': 0.015,
                       '~open_set_max_grasp_backoff': 0.04}
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)

    def run_pick_steps(self):
        p = picker()
        kept = p.raise_low_grasps(three_grasps(), 'table_2', [0.0, 0.0, 0.0])
        return p, p.prefer_less_backoff(kept)

    def test_flag_off_keeps_the_finder_order(self):
        p, ordered = self.run_pick_steps()
        self.assertEqual([g.id for g in ordered], ['back13', 'back09', 'none'])
        self.assertAlmostEqual(p.grasp_backoff['back13'], 0.010 / math.cos(math.radians(39.0)), places=4)
        self.assertAlmostEqual(p.grasp_backoff['back09'], 0.007 / math.cos(math.radians(39.0)), places=4)
        self.assertEqual(p.grasp_backoff['none'], 0.0)   # measured, not moved back

    def test_flag_on_puts_the_less_moved_back_grasps_first(self):
        self.params['~open_set_prefer_less_backoff'] = True
        p, ordered = self.run_pick_steps()
        self.assertEqual([g.id for g in ordered], ['none', 'back09', 'back13'])

    def test_the_max_back_off_still_drops_as_before(self):
        self.params['~open_set_prefer_less_backoff'] = True
        self.params['~open_set_max_grasp_backoff'] = 0.01   # 1.29 cm needed -> dropped by the clearance step, as before
        p, ordered = self.run_pick_steps()
        self.assertEqual([g.id for g in ordered], ['none', 'back09'])

    def test_back_off_is_forgotten_between_picks(self):
        self.params['~open_set_prefer_less_backoff'] = True
        p, _ = self.run_pick_steps()
        self.params['~open_set_min_fingertip_height'] = 0.0
        self.params['~open_set_min_grasp_height'] = 0.0   # clearance off: the step returns at once
        later = [grasp('back13', 39.0, 0.9), grasp('x', 39.0, 0.9)]
        p.raise_low_grasps(later, 'table_2', [0.0, 0.0])
        self.assertEqual(p.grasp_backoff, {})   # not measured: unknown, sorted as 0, order unchanged
        self.assertEqual([g.id for g in p.prefer_less_backoff(later)], ['back13', 'x'])

    def offer(self, flag, prefilter, attempts=0, ik=None):
        '''the open-set steps from the clearance to the offer: raise, prefer, IK pre-filter (stubbed), limit'''
        self.params['~open_set_prefer_less_backoff'] = flag
        p = picker()
        p.external_grasp_ik_prefilter = prefilter
        p.external_grasp_max_attempts = attempts
        p.cable_gate_passing = 0
        p.mtc = None
        if ik is not None:
            p.mtc = FakeMtc()
            p.arm_group_name = 'arm'
            p.robot = NS(get_current_state=lambda: NS(joint_state=NS(name=[], position=[])),
                         arm=NS(get_end_effector_link=lambda: 'tcp'))
        grasps = p.prefer_less_backoff(p.raise_low_grasps(three_grasps(), 'table_2', [0.0, 0.0, 0.0]))
        if ik is None:
            return p, p.reachable_external_grasps(grasps)
        by_pose = {id(g.grasp_pose): g.id for g in grasps}
        with mock.patch.object(pick.rospy, 'ServiceProxy', lambda *a, **k: FakeIk(ik, by_pose)):
            return p, p.reachable_external_grasps(grasps)

    def test_ik_unavailable_and_the_offer_limit_keep_the_less_moved_back(self):
        _, offered = self.offer(True, prefilter=False, attempts=2)
        self.assertEqual([g.id for g in offered], ['none', 'back09'])
        _, offered = self.offer(False, prefilter=False, attempts=2)
        self.assertEqual([g.id for g in offered], ['back13', 'back09'])   # flag off: the finder's first two

    def test_collision_free_ik_group_stays_ahead_and_the_retry_keeps_the_order(self):
        # 'none' has only a colliding IK solution: it goes after the collision-free ones despite no back-off
        ik = {'none': 'colliding', 'back09': 'free', 'back13': 'free'}
        p, offered = self.offer(True, prefilter=True, ik=ik)
        self.assertEqual([g.id for g in offered], ['back09', 'back13', 'none'])
        self.assertEqual(list(p.mtc.grasp_ik_states), ['back09', 'back13'])   # IK pins paired with their grasps
        self.assertEqual([g.id for g in p.rank_prefiltered(NS(name=[], position=[]))], ['back09', 'back13', 'none'])
        _, offered = self.offer(False, prefilter=True, ik=ik)
        self.assertEqual([g.id for g in offered], ['back13', 'back09', 'none'])

    def test_offer_limit_after_the_ik_groups_logs_the_cut(self):
        ik = {'none': 'free', 'back09': 'colliding', 'back13': 'free'}
        with mock.patch.object(pick.rospy, 'loginfo') as log:
            _, offered = self.offer(True, prefilter=True, attempts=2, ik=ik)
        self.assertEqual([g.id for g in offered], ['none', 'back13'])   # collision-free first, then by back-off
        cut = [c.args[0] for c in log.call_args_list if 'before / after the offer limit' in c.args[0]]
        self.assertEqual(len(cut), 1)
        self.assertIn('collision-free IK 2 / 2', cut[0])
        self.assertIn('step 0: 1, step 1: 1, step 2: 1 / step 0: 1, step 2: 1', cut[0])


class FakeMtc:
    strict_contact_order = True
    cable_guard = None

    def __init__(self):
        self.grasp_ik_states, self.grasp_ik_branches = {}, {}

    def update_cable_guard(self):
        pass

    def least_stretch_branch(self, ik, request, solution, start, grasp_id):
        return solution


class FakeIk:
    '''compute_ik: 'free' = solved with collision checking, 'colliding' = only without it'''

    def __init__(self, kinds, by_pose):
        self.kinds, self.by_pose = kinds, by_pose

    def wait_for_service(self, timeout):
        pass

    def close(self):
        pass

    def __call__(self, request):
        kind = self.kinds[self.by_pose[id(request.ik_request.pose_stamped)]]
        ok = kind == 'free' or not request.ik_request.avoid_collisions
        return NS(error_code=NS(val=1 if ok else -31), solution=NS(joint_state=f'q_{kind}'))


if __name__ == '__main__':
    unittest.main()
