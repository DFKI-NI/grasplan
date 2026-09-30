'''#229: with ~mtc_lift_tolerate_touching_neighbours the attach stage of the MTC pick may touch only the scene boxes that
already touch the target, the lift goes straight up (planning frame +z), and a stage after the lift forbids these
contacts again. Flag off or no neighbours: the old task. Also the neighbour search itself on a fake planning scene.
Stubbed MTC stages (no robot model, no ROS master); needs the Mobipick image for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject, Grasp, PickupGoal
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

import grasplan.mtc_pick_place as mpp
from test_pick_contact_order import Container, Stage, Task, GRIPPER, OBJ


def build(flag, neighbours):
    fake_stages = mock.Mock(ModifyPlanningScene=Stage)
    fake_core = mock.Mock(SerialContainer=Container)
    m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
    m.gripper_links, m.allow_octomap_contact, m.eef_link = GRIPPER, True, 'gripper_tcp'
    m.strict_contact_order, m.planning_frame = False, 'map'
    m.start_task = lambda name: (Task(name), [mpp.NOOP])
    m.connect = lambda name: Stage(name)
    lifts = []
    m.move_relative = lambda name, translation: lifts.append((name, translation)) or Stage(name)
    m.compute_ik = lambda name, pose, monitored: Stage(name)
    m.to_planning_frame = lambda pose: pose
    m.touching_neighbours = mock.Mock(return_value=neighbours)
    goal = PickupGoal(target_name=OBJ, support_surface_name='table_2')
    grasp = Grasp(id='grasp_0', grasp_pose=PoseStamped())
    grasp.post_grasp_retreat.direction.header.frame_id = 'gripper_tcp'
    grasp.post_grasp_retreat.direction.vector.x = -1.0
    params = {'~mtc_lift_tolerate_touching_neighbours': flag}
    with mock.patch.object(mpp, 'stages', fake_stages), mock.patch.object(mpp, 'core', fake_core), \
            mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: params.get(name, default)):
        task, roles = m.make_pick_task(goal, grasp)
    return m, task, roles, {n: t for n, t in lifts}, grasp


def stage(task, name):
    return next(c for c in task['grasp'].children if c.name == name)


class TestTolerantLift(unittest.TestCase):
    def test_flag_on_scopes_the_allowance_to_the_lift(self):
        m, task, roles, lifts, _ = build(True, ['klt_1'])
        attach = stage(task, 'attach object')
        self.assertIn((OBJ, ('klt_1',), True), attach.allowed)
        self.assertEqual([c.name for c in task['grasp'].children][-2:], ['lift', 'forbid neighbour contacts after lift'])
        self.assertEqual(stage(task, 'forbid neighbour contacts after lift').allowed, [(OBJ, ('klt_1',), False)])
        direction = lifts['lift'].direction
        self.assertEqual((direction.header.frame_id, direction.vector.z), ('map', 1.0))
        self.assertEqual(len(roles), 1 + 2 + len(task['grasp'].children))

    def test_flag_off_or_no_neighbours_is_the_old_task(self):
        for flag, neighbours in ((False, ['klt_1']), (True, [])):
            m, task, roles, lifts, grasp = build(flag, neighbours)
            names = [c.name for c in task['grasp'].children]
            self.assertEqual(names[-1], 'lift')
            self.assertNotIn('forbid neighbour contacts after lift', names)
            self.assertEqual(lifts['lift'].direction.header.frame_id, 'gripper_tcp')
            self.assertFalse(any(a == OBJ and 'klt_1' in b for a, b, _ in stage(task, 'attach object').allowed))
        self.assertFalse(build(False, ['klt_1'])[0].touching_neighbours.called)


def box_object(name, x, y, size=(0.2, 0.2, 0.1)):
    return CollisionObject(id=name, header=Header(frame_id='map'),
                           primitives=[SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))],
                           primitive_poses=[Pose(position=Point(x, y, 0.0), orientation=Quaternion(0, 0, 0, 1))])


class TestNeighbourSearch(unittest.TestCase):
    def neighbours(self, objects):
        m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        m.planning_frame = 'map'
        scene = NS(scene=NS(world=NS(collision_objects=objects)))
        m.get_planning_scene_srv = lambda request: scene
        return m.touching_neighbours(OBJ, 'table_2')

    def test_only_touching_boxes_not_support_not_far_ones(self):
        objects = [box_object(OBJ, 0.0, 0.0), box_object('klt_1', 0.205, 0.0), box_object('far', 0.5, 0.0),
                   box_object('table_2', 0.0, 0.0)]
        self.assertEqual(self.neighbours(objects), ['klt_1'])

    def test_target_absent_gives_none(self):
        self.assertEqual(self.neighbours([box_object('klt_1', 0.0, 0.0)]), [])


if __name__ == '__main__':
    unittest.main()
