'''~open_set_grasp_funnel_markers: a debug output that publishes the open-set grasps left after each filter step. Off
(the default) nothing is advertised or published; on, the steps are published in order and the grasps handed to the
planner, their order and the calls of the pick stay the same. Mocks (no ROS master); needs the Mobipick image for the
module imports.'''
import json
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp

import grasplan.pick as pick
from grasplan.grasp_markers import FUNNEL_COUNTS_NAMESPACE, funnel_step_markers, grasp_marker_id
from test_open_set_view_and_start import TestPickStartsFromTransport, object_pose
from visualization_msgs.msg import Marker


def grasp(grasp_id, z=0.8):
    g = Grasp()
    g.id = grasp_id
    g.grasp_pose = PoseStamped()
    g.grasp_pose.header.frame_id = 'map'
    g.grasp_pose.pose.position.z = z
    g.grasp_pose.pose.orientation.w = 1.0
    return g


class FunnelMarkers(unittest.TestCase):
    def test_marker_ids_follow_the_grasp_ids(self):
        self.assertEqual(grasp_marker_id('anygrasp_12', 0), 12)
        self.assertEqual(grasp_marker_id('topdown_1', 0), 10001)
        self.assertEqual(grasp_marker_id('grasp', 7), 7)

    def test_one_arrow_per_grasp_and_a_count_label(self):
        grasps = [grasp('anygrasp_3'), grasp('anygrasp_5')]
        markers = funnel_step_markers(grasps[0].grasp_pose.header, 'ik_reachable', 2, grasps, object_pose())
        arrows = [m for m in markers if m.type == Marker.ARROW]
        self.assertEqual([(m.ns, m.id) for m in arrows], [('ik_reachable', 3), ('ik_reachable', 5)])
        label = [m for m in markers if m.ns == FUNNEL_COUNTS_NAMESPACE]
        self.assertEqual([(m.id, m.text) for m in label], [(2, '3 ik_reachable: 2')])


class FunnelSwitch(unittest.TestCase):
    def setUp(self):
        self.params = {}
        patcher = mock.patch.object(pick.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        publisher = mock.patch.object(pick.rospy, 'Publisher')
        self.publisher = publisher.start()
        self.addCleanup(publisher.stop)
        self.p = pick.PickTools.__new__(pick.PickTools)
        self.p.mtc = None

    def test_off_advertises_and_publishes_nothing(self):
        self.p.grasp_funnel_start('apple_1', object_pose())
        grasps = [grasp('anygrasp_0')]
        self.assertIs(self.p.grasp_funnel('candidates', grasps), grasps)
        self.p.grasp_funnel_ranked(grasps)
        self.publisher.assert_not_called()

    def test_on_publishes_every_step_in_order_and_keeps_the_grasps(self):
        self.params['~open_set_grasp_funnel_markers'] = True
        self.p.grasp_funnel_start('apple_1', object_pose())
        grasps = [grasp('anygrasp_0'), grasp('anygrasp_1'), grasp('anygrasp_2')]
        order = list(grasps)
        self.assertIs(self.p.grasp_funnel('candidates', grasps), grasps)
        self.p.grasp_funnel('low_object_kept', grasps[:2])
        self.assertEqual(grasps, order)
        markers_publisher = self.publisher.return_value
        summary = json.loads(markers_publisher.publish.call_args_list[-1][0][0].data)
        self.assertEqual(summary['object'], 'apple_1')
        self.assertEqual([(s['step'], s['count']) for s in summary['steps']], [('candidates', 3), ('low_object_kept', 2)])
        self.assertEqual(summary['steps'][1]['ids'], ['anygrasp_0', 'anygrasp_1'])
        # the latched marker array holds every step so far, after one DELETEALL
        markers = markers_publisher.publish.call_args_list[-2][0][0].markers
        self.assertEqual(markers[0].action, Marker.DELETEALL)
        self.assertEqual({m.ns for m in markers[1:]}, {'candidates', 'low_object_kept', FUNNEL_COUNTS_NAMESPACE})

    def test_ranked_steps(self):
        self.params['~open_set_grasp_funnel_markers'] = True
        self.p.grasp_funnel_start('apple_1', object_pose())
        reachable, colliding = [grasp('anygrasp_0'), grasp('anygrasp_1')], [grasp('anygrasp_4')]
        self.p.ik_prefilter = (reachable, [None, None], colliding)
        self.p.mtc = mock.Mock(cable_guard=object())
        self.p.cable_gate_passing = 1
        self.p.grasp_funnel_ranked(reachable + colliding)
        summary = json.loads(self.publisher.return_value.publish.call_args_list[-1][0][0].data)
        self.assertEqual([(s['step'], s['count']) for s in summary['steps']],
                         [('ik_reachable', 3), ('cable_pass', 1), ('offered', 3)])


class FunnelLeavesThePickAlone(TestPickStartsFromTransport):
    '''the open-set pick calls the same steps in the same order and offers the same grasps with the flag on and off'''

    def run_pick(self):
        self.calls = []
        grasps = [grasp('anygrasp_0'), grasp('anygrasp_1', z=0.75), grasp('anygrasp_2')]
        p = self.p
        p.grasp_planner = mock.Mock(make_grasps_msgs_from_candidates=lambda *a: self.calls.append('make') or grasps)
        p.top_grasps_for_low_objects = lambda g, w, *a: self.calls.append('low') or (g[:2], w[:2], True)
        p.raise_low_grasps = lambda g, *a, **k: self.calls.append('clearance') or g[::-1]
        p.reachable_external_grasps = lambda g: self.calls.append('ik') or list(g)
        p.pick_start_failed = ''
        p.ik_prefilter = None
        p.stayed_for_pick, p.cable_gate_passing = '', 0
        offered = []
        p.offer_grasps = lambda obj, g, *a: offered.append([x.id for x in g]) or None
        candidates = [mock.Mock(width=0.05) for _ in grasps]
        with mock.patch.object(pick.rospy, 'Publisher') as publisher:
            self.assertFalse(p.pick_object('apple_1', 'table_2', 'side_grasp', [],
                                           external_grasp_candidates=candidates))
        return list(self.calls), offered, publisher

    def test_same_steps_and_grasps_with_the_flag_on(self):
        calls_off, offered_off, publisher_off = self.run_pick()
        publisher_off.assert_not_called()
        self.params['~open_set_grasp_funnel_markers'] = True
        calls_on, offered_on, publisher_on = self.run_pick()
        self.assertEqual(calls_on, calls_off)
        self.assertEqual(offered_on, offered_off)
        self.assertEqual(offered_on, [['anygrasp_1', 'anygrasp_0']])
        summary = json.loads(publisher_on.return_value.publish.call_args_list[-1][0][0].data)
        self.assertEqual([(s['step'], s['count']) for s in summary['steps']],
                         [('candidates', 3), ('low_object_kept', 2), ('clearance_kept', 2), ('offered', 2)])


del TestPickStartsFromTransport   # its own tests run from their own file


if __name__ == '__main__':
    unittest.main()
