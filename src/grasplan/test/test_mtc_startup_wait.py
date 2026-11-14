'''The MTC startup waits of the pick, place and insert nodes never give up on a slow server: a missing
execute_trajectory action server or planning scene service after ~moveit_action_startup_timeout logs a warning and
waits another round (a fresh action client each round), instead of raising: the nodes are required nodes of the demo
launch, and the exception of one of them ended the whole simulation (sim 2026-11-14: the insert node's client gave up
while move_group already executed trajectories). Only a ROS shutdown ends the wait. Mocks only (no ROS master); needs
the Mobipick image for the module imports.'''
import unittest
from unittest import mock

import rospy

import grasplan.mtc_pick_place as mpp


class Client:
    '''a SimpleActionClient stand-in whose server answers in the round given by found_in (0 = first)'''
    made = []

    def __init__(self, found_in):
        self.found_in = found_in

    def factory(self, name, action_type):
        client = mock.Mock(name=f'client {len(Client.made)}')
        client.wait_for_server.return_value = len(Client.made) >= self.found_in
        Client.made.append(client)
        return client


class TestStartupWait(unittest.TestCase):
    def setUp(self):
        Client.made = []
        for name in ('loginfo', 'logwarn'):
            patcher = mock.patch.object(mpp.rospy, name)
            setattr(self, name, patcher.start())
            self.addCleanup(patcher.stop)
        patcher = mock.patch.object(mpp.rospy, 'resolve_name', lambda n: '/mobipick/' + n)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_a_server_found_at_once_gives_its_client_and_no_warning(self):
        client = mpp.wait_for_action_server('execute_trajectory', object, 60.0, Client(0).factory)
        self.assertIs(client, Client.made[0])
        self.logwarn.assert_not_called()

    def test_a_slow_server_is_waited_for_with_a_fresh_client_each_round(self):
        # a shutdown after 5 rounds bounds the test: a client that never finds its server fails it instead of hanging
        with mock.patch.object(mpp.rospy, 'is_shutdown', side_effect=[False] * 5 + [True] * 100):
            client = mpp.wait_for_action_server('execute_trajectory', object, 60.0, Client(2).factory)
        self.assertEqual(len(Client.made), 3)
        self.assertIs(client, Client.made[2])
        self.assertEqual(self.logwarn.call_count, 2)
        self.assertIn('not found after 120 s, still waiting', self.logwarn.call_args[0][0])
        # the clients of the failed rounds are stopped, the one that found its server is kept
        self.assertEqual([c.action_client.stop.call_count for c in Client.made], [1, 1, 0])

    def test_only_a_shutdown_ends_the_wait_with_an_error(self):
        with mock.patch.object(mpp.rospy, 'is_shutdown', return_value=True), self.assertRaises(RuntimeError):
            mpp.wait_for_action_server('execute_trajectory', object, 60.0, Client(5).factory)

    def test_a_slow_service_is_waited_for(self):
        calls = []

        def wait(name, timeout):
            calls.append(name)
            if len(calls) < 3:
                raise rospy.ROSException('timeout exceeded')

        with mock.patch.object(mpp.rospy, 'wait_for_service', wait), \
                mock.patch.object(mpp.rospy, 'is_shutdown', side_effect=[False] * 5 + [True] * 100):
            mpp.wait_for_service_until_up('apply_planning_scene', 60.0)
        self.assertEqual(len(calls), 3)
        self.assertEqual(self.logwarn.call_count, 2)

    def test_a_service_wait_ends_at_shutdown(self):
        def wait(name, timeout):
            raise rospy.ROSException('timeout exceeded')

        with mock.patch.object(mpp.rospy, 'wait_for_service', wait), \
                mock.patch.object(mpp.rospy, 'is_shutdown', return_value=True), self.assertRaises(rospy.ROSException):
            mpp.wait_for_service_until_up('apply_planning_scene', 60.0)

    def test_the_constructor_uses_the_waits(self):
        import inspect
        source = inspect.getsource(mpp.MtcPickPlace.__init__)
        self.assertIn("wait_for_action_server('execute_trajectory'", source)
        self.assertIn('wait_for_service_until_up(service, server_timeout)', source)
        self.assertNotIn('action server not found', source)


if __name__ == '__main__':
    unittest.main()
