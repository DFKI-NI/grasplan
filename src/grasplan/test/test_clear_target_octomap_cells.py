'''~clear_target_octomap_cells: once per pick, before the first MTC plan, the target's box as the planning scene holds it
goes to move_group's clear_octomap_box service; flag off, no target in the scene or a second call: nothing; service
missing or refusing: a warning and the pick goes on. Stubbed scene and service, no ROS master; needs the Mobipick image
for the module imports.'''
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import rospy

import grasplan.pick as pick


def tools(objects=('drill_1',)):
    p = pick.PickTools.__new__(pick.PickTools)
    p.scene = NS(get_objects=lambda names: {n: NS(id=n) for n in names if n in objects})
    p.target_cells_cleared = False
    return p


def run(p, flag=True, service=None, target='drill_1'):
    calls = []

    def proxy(name, srv):
        def call(request):
            calls.append((name, [o.id for o in request.scene.world.collision_objects]))
            if isinstance(service, Exception):
                raise service
            return NS(success=service if service is not None else True)
        return call
    params = {} if flag is None else {'~clear_target_octomap_cells': flag}   # None: the parameter is not set at all
    with mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: params.get(name, default)), \
            mock.patch.object(pick.rospy, 'wait_for_service', lambda name, timeout=None: None), \
            mock.patch.object(pick.rospy, 'ServiceProxy', proxy), \
            mock.patch.object(pick.rospy, 'logwarn') as warn, \
            mock.patch.object(pick.rospy, 'loginfo'), \
            mock.patch.object(pick.rospy, 'get_time', lambda: now[0]), \
            mock.patch.object(pick.rospy, 'sleep', lambda s: sleeps.append(round(s, 3)) or now.__setitem__(0, now[0] + s)):
        p.clear_target_octomap_cells(target)
    return calls, warn


now, sleeps = [100.0], []


class TestClearTargetCells(unittest.TestCase):
    def setUp(self):
        now[0] = 100.0
        sleeps.clear()

    def test_waits_until_one_second_after_the_add(self):
        p = tools()
        p.target_added_at = 99.7
        run(p)
        self.assertEqual(sleeps, [0.7])
        p = tools()
        p.target_added_at = 98.0
        sleeps.clear()
        run(p)
        self.assertEqual(sleeps, [])

    def test_once_with_the_scene_box(self):
        p = tools()
        calls, warn = run(p)
        self.assertEqual(calls, [('move_group/clear_octomap_box', ['drill_1'])])
        self.assertFalse(warn.called)
        self.assertEqual(run(p)[0], [])   # the second MTC plan of the same pick: nothing

    def test_flag_off_does_nothing(self):
        p = tools()
        self.assertEqual(run(p, flag=False)[0], [])
        self.assertFalse(p.target_cells_cleared)

    def test_flag_not_set_does_nothing(self):
        # careful flag: a node launched without the parameter must not clear anything
        p = tools()
        self.assertEqual(run(p, flag=None)[0], [])
        self.assertFalse(p.target_cells_cleared)

    def test_not_set_up_by_pick_object_does_nothing(self):
        p = tools()
        del p.target_cells_cleared
        self.assertEqual(run(p)[0], [])

    def test_missing_target_service_error_or_refusal_warn_only(self):
        calls, warn = run(tools(objects=()))
        self.assertEqual(calls, [])
        self.assertTrue(warn.called)
        for service in (rospy.ServiceException('no service'), False):
            calls, warn = run(tools(), service=service)
            self.assertEqual(len(calls), 1)
            self.assertTrue(warn.called)


if __name__ == '__main__':
    unittest.main()
