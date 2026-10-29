'''the cable-shaped pick task: the flag off leaves the task as before; on, the model's waypoints become joint-
interpolated MoveTo stages before the Connect, a slack checked path is interpolated to the end, a stretched one gets the
wrist corridor. Real MTC stages, fake guard and shaper; no ROS master (Mobipick image).'''
import math
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from moveit.task_constructor import stages
from sensor_msgs.msg import JointState

import grasplan.mtc_pick_place as mpp
from grasplan import cable_shaping
from grasplan.cable_shaping import ShapedPath

PREFIX = 'mobipick/'
J = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']


def joints(*deg):
    return {j: math.radians(v) for j, v in zip(J, deg)}


class Task:
    def __init__(self):
        self.stages = []

    def add(self, stage):
        self.stages.append(stage)


class FakeShaper:
    def __init__(self, path):
        self.path, self.calls = path, []

    def plan(self, start, goal, margin):
        self.calls.append((start, goal, margin))
        return self.path

    def corridor_constraints(self, box, prefix):
        return cable_shaping.CablePathShaper.corridor_constraints(self, box, prefix)


def mtc(path, params):
    m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
    m.arm_group = 'arm'
    m.pipeline = mpp.core.PipelinePlanner()
    m.connect_timeout = 5.0
    m.cable_guard = NS(max_stretch=-0.115, wrist_3_joint=PREFIX + 'ur5_wrist_3_joint',
                       path_constraints=lambda: 'wide wrist_3 window')
    m.cable_shaper = FakeShaper(path)
    return m, mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: params.get(name.lstrip('~'), default))


GOAL = JointState(name=[PREFIX + f'ur5_{j}_joint' for j in J], position=list(joints(-90, -90, 115, -43, 90, 270).values()))


class TestShapeTo(unittest.TestCase):
    def setUp(self):
        self.now = mock.patch.object(mpp, '_current_arm_q', lambda timeout=2.0: {
            PREFIX + f'ur5_{j}_joint': v for j, v in joints(0, -149, 139, -117, -90, 180).items()})
        self.now.start()

    def tearDown(self):
        self.now.stop()

    def test_flag_off_adds_nothing(self):
        m, params = mtc(ShapedPath([joints(0, -149, 139, -100, -90, 200)], -0.2, 0.01, False, None, 5.0), {})
        task, roles = Task(), []
        with params:
            self.assertIsNone(m.shape_to(task, roles, GOAL, 'grasp g'))
        self.assertEqual((task.stages, roles, m.cable_shaper.calls), ([], [], []))

    def test_waypoints_become_moveto_stages_and_the_last_segment_is_interpolated(self):
        path = ShapedPath([joints(0, -149, 139, -100, -90, 200), joints(-45, -120, 125, -70, 0, 240)], -0.2, 0.01,
                          False, None, 5.0)
        m, params = mtc(path, {'mtc_cable_path_shaping': True})
        task, roles = Task(), []
        with params:
            shaped = m.shape_to(task, roles, GOAL, 'grasp g')
            connect = m.connect('move to pregrasp', shaped)
        self.assertIs(shaped, path)
        self.assertEqual([s.name for s in task.stages], ['cable waypoint 1', 'cable waypoint 2'])
        self.assertTrue(all(isinstance(s, stages.MoveTo) for s in task.stages))
        self.assertEqual(roles, [mpp.ARM, mpp.ARM])
        self.assertIsInstance(connect, stages.Connect)
        start, goal, margin = m.cable_shaper.calls[0]
        self.assertEqual(margin, -0.115)
        self.assertIn(PREFIX + 'ur5_wrist_3_joint', goal)

    def test_waypoints_off_keeps_the_path_but_adds_no_stage(self):
        path = ShapedPath([joints(0, -149, 139, -100, -90, 200)], -0.2, 0.01, False, None, 5.0)
        m, params = mtc(path, {'mtc_cable_path_shaping': True, 'mtc_cable_path_waypoints': False})
        m.cable_shaper.evaluate_max_line = lambda a, b: -0.05
        task, roles = Task(), []
        with params:
            shaped = m.shape_to(task, roles, GOAL, 'grasp g')
        self.assertEqual(task.stages, [])
        self.assertEqual(shaped.worst, -0.05)   # the straight line's stretch, for the Connect's choice

    def test_stretched_path_gets_the_corridor(self):
        box = {'wrist_1': (math.radians(-120), math.radians(-40)), 'wrist_2': (math.radians(-95), math.radians(95)),
               'wrist_3': (math.radians(170), math.radians(275))}
        path = ShapedPath([], -0.05, 0.0, False, box, 5.0)
        m, params = mtc(path, {'mtc_cable_path_shaping': True})
        task, roles = Task(), []
        with params:
            shaped = m.shape_to(task, roles, GOAL, 'grasp g')
            with mock.patch.object(mpp.stages, 'Connect', lambda name, planners: NS(name=name, planners=planners,
                                                                                   properties={})):
                connect = m.connect('move to pregrasp', shaped)
        constraints = connect.properties['path_constraints']
        self.assertEqual(sorted(c.joint_name for c in constraints.joint_constraints),
                         [PREFIX + f'ur5_wrist_{k}_joint' for k in (1, 2, 3)])
        w3 = [c for c in constraints.joint_constraints if c.joint_name.endswith('wrist_3_joint')][0]
        self.assertAlmostEqual(math.degrees(w3.position - w3.tolerance_below), 170.0)

    def test_corridor_off_keeps_the_wide_window(self):
        path = ShapedPath([], -0.05, 0.0, False, {'wrist_3': (3.0, 4.0)}, 5.0)
        m, params = mtc(path, {'mtc_cable_path_shaping': True, 'mtc_cable_corridor': False})
        with params, mock.patch.object(mpp.stages, 'Connect', lambda name, planners: NS(properties={})):
            connect = m.connect('move to pregrasp', path)
        self.assertEqual(connect.properties['path_constraints'], 'wide wrist_3 window')

    def test_no_pinned_ik_no_shaping(self):
        m, params = mtc(ShapedPath([], -0.2, 0.0, True, None, 1.0), {'mtc_cable_path_shaping': True})
        with params:
            self.assertIsNone(m.shape_to(Task(), [], None, 'grasp g'))



class TestPickupFallbacks(unittest.TestCase):
    """a failed shaped plan is replanned unshaped; a failed branch falls through to the next ranked one"""

    def make(self, results):
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.is_preempt_requested = lambda: False
        m.update_cable_guard = lambda: None
        m.gripper_links = []
        m.built = []
        results = list(results)

        def make_pick_task(goal, grasp):
            pinned = getattr(m, 'grasp_ik_states', {}).get(grasp.id)
            m.built.append(('unshaped' if m.no_shaping else 'shaped', pinned))
            m.shaped_path = None if m.no_shaping else 'path'
            return 'task', []
        m.no_shaping = False
        m.make_pick_task = make_pick_task
        m.plan = lambda task, what: results.pop(0)
        m.turned_grasp_plan = lambda goal, grasp: (None, None, None)
        m.log_failed_branch = lambda grasp: None
        m.execute = lambda *a, **k: mpp.MoveItErrorCodes.SUCCESS
        m.gripper_holds_object = lambda when: True
        m.regrasp_request = lambda goal, grasp: None
        return m

    def goal(self):
        grasp = NS(id='g', pre_grasp_posture='open', grasp_posture='closed')
        return NS(target_name='relay', possible_grasps=[grasp])

    def test_unshaped_retry(self):
        m = self.make([False, True])
        with mock.patch.object(mpp.rospy, 'get_param', lambda n, d=None: d):
            result = m.pickup(self.goal())
        self.assertEqual(result.error_code.val, mpp.MoveItErrorCodes.SUCCESS)
        self.assertEqual([b[0] for b in m.built], ['shaped', 'unshaped'])

    def test_next_branch_after_both_fail(self):
        m = self.make([False, False, True])
        m.grasp_ik_states = {'g': 'branch 1'}
        m.grasp_ik_branches = {'g': ['branch 1', 'branch 2']}
        with mock.patch.object(mpp.rospy, 'get_param', lambda n, d=None: d):
            result = m.pickup(self.goal())
        self.assertEqual(result.error_code.val, mpp.MoveItErrorCodes.SUCCESS)
        self.assertEqual(m.built, [('shaped', 'branch 1'), ('unshaped', 'branch 1'), ('shaped', 'branch 2')])
        self.assertEqual(m.grasp_ik_states['g'], 'branch 2')   # the branch that planned stays pinned

    def test_all_branches_fail_restores_the_pin(self):
        m = self.make([False] * 4)
        m.grasp_ik_states = {'g': 'branch 1'}
        m.grasp_ik_branches = {'g': ['branch 1', 'branch 2']}
        with mock.patch.object(mpp.rospy, 'get_param', lambda n, d=None: d):
            result = m.pickup(self.goal())
        self.assertNotEqual(result.error_code.val, mpp.MoveItErrorCodes.SUCCESS)
        self.assertEqual(m.grasp_ik_states['g'], 'branch 1')


if __name__ == '__main__':
    unittest.main()
