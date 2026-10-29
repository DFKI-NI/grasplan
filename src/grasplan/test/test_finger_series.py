'''the closing of the gripper as a measurement (log only, no motion): contact angle, creep and effort from synthetic
finger series (a stall, a creeping close, a close on air), the tilt of the pad closing axis, the one log line and its
gating. No ROS master; needs the Mobipick image for the module imports.'''
import math
import unittest
from unittest import mock

import grasplan.mtc_pick_place as mpp
from grasplan.tools.finger_series import analyse_close, closing_axis_tilt_deg


def series(angles, dt=0.1, effort=None):
    return [(i * dt, a, effort) for i, a in enumerate(angles)]


class TestAnalyseClose(unittest.TestCase):
    def test_a_clean_stall_has_the_contact_where_it_stopped_and_no_creep(self):
        result = analyse_close(series([0.0, 0.1, 0.2, 0.289, 0.289, 0.289, 0.289, 0.289], effort=12.0))
        self.assertAlmostEqual(result['contact'], 0.289, places=3)
        self.assertAlmostEqual(result['final'], 0.289, places=3)
        self.assertAlmostEqual(result['creep'], 0.0, places=3)
        self.assertEqual(result['effort'], 12.0)
        self.assertEqual(result['n'], 8)

    def test_a_creeping_close_reports_the_creep(self):
        # stops at 0.30 (speed under the limit), then keeps closing slowly to 0.34 (pressing into a soft object)
        angles = [0.0, 0.15, 0.30, 0.302, 0.304, 0.31, 0.318, 0.326, 0.334, 0.34, 0.34, 0.34]
        result = analyse_close(series(angles))
        self.assertAlmostEqual(result['contact'], 0.302, places=3)
        self.assertGreater(result['creep'], 0.03)

    def test_a_stop_before_any_travel_is_not_a_contact(self):
        # the fingers wait (command delay) before they move: that is not a contact
        angles = [0.0, 0.0, 0.0, 0.0, 0.2, 0.4, 0.5, 0.5, 0.5]
        result = analyse_close(series(angles))
        self.assertAlmostEqual(result['contact'], 0.5, places=3)

    def test_too_few_samples(self):
        self.assertIsNone(analyse_close([]))
        self.assertIsNone(analyse_close(series([0.0, 0.1])))

    def test_effort_missing_or_nan(self):
        self.assertIsNone(analyse_close(series([0.0, 0.2, 0.4, 0.4, 0.4]))['effort'])
        self.assertIsNone(analyse_close(series([0.0, 0.2, 0.4, 0.4, 0.4], effort=float('nan')))['effort'])

    def test_peak_effort(self):
        samples = [(0.0, 0.0, 1.0), (0.1, 0.2, -30.0), (0.2, 0.4, 5.0), (0.3, 0.4, 5.0)]
        self.assertEqual(analyse_close(samples)['peak_effort'], 30.0)


class TestTilt(unittest.TestCase):
    def test_closing_axis_tilt(self):
        self.assertAlmostEqual(closing_axis_tilt_deg([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), 0.0)
        c, s = math.cos(math.radians(13.5)), math.sin(math.radians(13.5))
        self.assertAlmostEqual(closing_axis_tilt_deg([[1, 0, 0], [0, c, -s], [0, s, c]]), 13.5, places=3)
        self.assertAlmostEqual(closing_axis_tilt_deg([[1, 0, 0], [0, c, s], [0, -s, c]]), 13.5, places=3)


class TestLogLine(unittest.TestCase):
    def setUp(self):
        self.params = {}
        patcher = mock.patch.object(mpp.rospy, 'get_param', lambda name, default=None: self.params.get(name, default))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.m = mpp.MtcPickPlace.__new__(mpp.MtcPickPlace)
        self.request = {'object': 'apple_1', 'grasp_id': 'g8', 'log_only': True, 'width_open': 0.100,
                        'width_box': 0.105, 'width_extent': 0.119, 'tilt_deg': 13.5}

    def test_one_line_with_contact_creep_effort_and_tilt(self):
        samples = series([0.0, 0.1, 0.2, 0.289, 0.289, 0.289, 0.289], effort=8.0)
        with mock.patch.object(mpp.rospy, 'loginfo') as log:
            self.m.log_close_series(self.request, samples)
        line = log.call_args[0][0]
        self.assertIn('mtc: stall: apple_1 grasp g8 contact_theta 0.289', line)
        self.assertIn('contact_gap 91 mm', line)
        self.assertIn('creep 0.0 mm', line)
        self.assertIn('effort 8.0', line)
        self.assertIn('tilt 13.5 deg', line)
        self.assertIn('extent_y 119 mm', line)

    def test_no_samples_is_logged_not_raised(self):
        with mock.patch.object(mpp.rospy, 'loginfo') as log:
            self.m.log_close_series(self.request, [])
        self.assertIn('no analysis', log.call_args[0][0])

    def test_a_failure_never_raises(self):
        with mock.patch.object(mpp.rospy, 'logwarn') as warn:
            self.m.log_close_series(self.request, [(0, 'x', None)] * 4)
        self.assertIn('could not analyse', warn.call_args[0][0])

    def test_recording_only_in_log_only_mode(self):
        self.assertIsNone(self.m.start_close_recording(None))
        self.assertIsNone(self.m.start_close_recording(dict(self.request, log_only=False)))
        with mock.patch.object(mpp, 'FingerRecorder') as recorder:
            self.assertIs(self.m.start_close_recording(self.request), recorder.return_value)
        with mock.patch.object(mpp, 'FingerRecorder', side_effect=RuntimeError('no master')):
            with mock.patch.object(mpp.rospy, 'logwarn'):
                self.assertIsNone(self.m.start_close_recording(self.request))


class TestRecorder(unittest.TestCase):
    def test_keeps_only_its_joint(self):
        from sensor_msgs.msg import JointState
        with mock.patch.object(mpp.rospy, 'Subscriber'):
            recorder = mpp.FingerRecorder('mobipick/gripper_finger_joint')
        msg = JointState(name=['a', 'mobipick/gripper_finger_joint'], position=[1.0, 0.3], effort=[0.0, 7.0])
        msg.header.stamp.secs = 5
        recorder.callback(msg)
        recorder.callback(JointState(name=['a'], position=[1.0]))
        self.assertEqual(recorder.samples, [(5.0, 0.3, 7.0)])


if __name__ == '__main__':
    unittest.main()
