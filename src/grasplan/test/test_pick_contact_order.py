'''#151: in the MTC pick task the gripper may touch the object only from the grasp pose on (MTC demo layout), so the
free-space move to the pregrasp and the approach cannot sweep the open gripper through the object. Stubs the MTC
stages (no robot model, no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, PickupGoal

import grasplan.mtc_pick_place as mpp

OBJ, GRIPPER = 'pringles_can_1', ['finger_l', 'finger_r', 'palm']


class Stage:
    def __init__(self, name, *args):
        self.name, self.children, self.allowed = name, [], []

    def allowCollisions(self, a, b, allow):
        self.allowed.append((a, tuple(b) if isinstance(b, list) else b, allow))

    def attachObject(self, *args):
        pass

    def add(self, child):
        self.children.append(child)


class Task(Stage):
    def __getitem__(self, name):
        return next(c for c in self.children if c.name == name)


def leaves(stage):
    return [leaf for c in stage.children for leaf in (leaves(c) if c.children or isinstance(c, Container) else [c])]


class Container(Stage):
    pass


def ik_stage(name, pose, monitored):
    ik = Stage(name)
    ik.monitored = monitored   # the stage whose planning scene the IK is checked in
    return ik


def build(early=False, strict=True, grasp_id='g', ik_states=None):
    fake_stages = mock.Mock(ModifyPlanningScene=Stage)
    fake_core = mock.Mock(SerialContainer=Container)
    m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
    m.gripper_links, m.allow_octomap_contact, m.eef_link = GRIPPER, True, 'gripper_tcp'
    m.strict_contact_order = strict   # pick.py: True for open-set (external) grasps
    m.start_task = lambda name: (Task(name), [mpp.NOOP])
    m.connect = lambda name: Stage(name)
    m.move_relative = lambda name, translation: Stage(name)
    m.compute_ik = ik_stage
    if ik_states is not None:
        m.grasp_ik_states = ik_states   # pick.py clears them for closed-set picks
    m.to_planning_frame = lambda pose: pose
    goal, grasp = PickupGoal(target_name=OBJ, support_surface_name='table_2'), Grasp(id=grasp_id, grasp_pose=PoseStamped())
    with mock.patch.object(mpp, 'stages', fake_stages), mock.patch.object(mpp, 'core', fake_core), \
            mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: early if 'early' in name else default):
        task, roles = m.make_pick_task(goal, grasp)
    return task, roles


def object_allowed(stage):
    return any(a == OBJ and set(b) == set(GRIPPER) and allow for a, b, allow in stage.allowed)


class TestPickContactOrder(unittest.TestCase):
    def test_object_contact_only_after_the_grasp_pose(self):
        task, roles = build()
        names = [s.name for s in [task.children[0], task.children[1]] + task['grasp'].children]
        self.assertEqual(names[:5], ['allow octomap contact', 'move to pregrasp', 'approach', 'grasp pose',
                                     'allow gripper contacts'])
        self.assertFalse(object_allowed(task.children[0]))                  # not before the Connect
        self.assertTrue(object_allowed(task['grasp'].children[2]))          # right after the grasp IK
        self.assertTrue(any(a == mpp.OCTOMAP_COLLISION_NAME for a, _, _ in task.children[0].allowed))

    def test_one_role_per_leaf_stage(self):
        task, roles = build()
        # start_task's current state + before + connect + the grasp leaves
        self.assertEqual(len(roles), 1 + 2 + len(task['grasp'].children))
        self.assertEqual(roles[4:6], [mpp.NOOP, mpp.NOOP])                 # grasp pose, allow gripper contacts

    def test_closed_set_picks_keep_the_allowance_from_the_start(self):
        task, _ = build(strict=False)                      # DOPE box: the fingers sit inside it at the grasp pose
        self.assertTrue(object_allowed(task.children[0]))

    def test_closed_set_handcoded_grasps_plan_like_the_committed_layout(self):
        # suite 151c (relay, grasp_0..3): the IK sees the object allowance and no IK pin is set
        costs = []
        Stage.setCostTerm = lambda self, fn: costs.append(fn)
        try:
            for grasp_id in ['grasp_0', 'grasp_1', 'grasp_2', 'grasp_3']:
                task, _ = build(strict=False, grasp_id=grasp_id, ik_states={})
                ik = task['grasp'].children[1]
                self.assertIs(ik.monitored, task.children[0])
                self.assertTrue(object_allowed(ik.monitored))
        finally:
            del Stage.setCostTerm
        self.assertEqual(costs, [])

    def test_early_param_restores_the_old_allowance(self):
        task, _ = build(early=True)
        self.assertTrue(object_allowed(task.children[0]))


if __name__ == '__main__':
    unittest.main()
