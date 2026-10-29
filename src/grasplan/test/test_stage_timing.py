'''The timing line of an action (pure Python, fake clock, no ROS master).'''
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import stage_timing as st  # noqa: E402


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


class TestStageTiming(unittest.TestCase):
    def setUp(self):
        self.clock, self.lines = Clock(), []
        self.timer = st.StageTimer(self.lines.append, self.clock)

    def test_line_has_total_stages_nesting_counts_and_other(self):
        t, c = self.timer, self.clock
        t.begin('pick', 'banana')
        with t.stage('view'):
            c.now += 12.5
        with t.stage('plan'):
            c.now += 1.0
            for _ in range(3):
                with t.stage('guard_replay'):
                    c.now += 2.0
        c.now += 0.5
        line = t.end(True)
        self.assertEqual(self.lines, [line])
        self.assertEqual(line, 'TIMING pick banana ok=1 total=20.00 view=12.50 plan=7.00 plan/guard_replay=6.00 '
                               'plan/guard_replay#=3 other=0.50')

    def test_open_close_and_a_stage_left_open_are_closed_by_end(self):
        t, c = self.timer, self.clock
        t.begin('place', 'table_2')
        t.open('anygrasp')
        c.now += 3.0
        t.close()
        t.open('wait')
        c.now += 1.0
        line = t.end(False)
        self.assertEqual(line, 'TIMING place table_2 ok=0 total=4.00 anygrasp=3.00 wait=1.00 other=0.00')

    def test_outside_an_action_everything_is_a_noop(self):
        with self.timer.stage('plan'):
            self.clock.now += 1.0
        self.timer.open('x')
        self.timer.close()
        self.assertIsNone(self.timer.end(True))
        self.assertEqual(self.lines, [])

    def test_a_stage_that_raises_is_still_closed(self):
        t, c = self.timer, self.clock
        t.begin('pick', 'x')
        with self.assertRaises(ValueError):
            with t.stage('plan'):
                c.now += 2.0
                raise ValueError
        self.assertIn('plan=2.00', t.end(False))

    def test_a_new_begin_logs_the_unfinished_action(self):
        t = self.timer
        t.begin('pick', 'a')
        t.begin('pick', 'b')
        self.assertEqual(len(self.lines), 1)
        self.assertIn('pick a ok=0', self.lines[0])

    def test_decorators_and_the_server_result(self):
        timer = st.TIMER
        timer._log, timer._clock = self.lines.append, self.clock

        class Server:
            def set_succeeded(self, *a):
                pass

            def set_aborted(self, *a):
                pass

            def set_preempted(self, *a):
                pass

        server = Server()
        st.watch_server(server)

        class Node:
            @st.action('pick', lambda goal: goal.object_name)
            def callback(self, goal):
                self.view()
                server.set_succeeded()

            @st.timed('view')
            def view(self):
                self_clock.now += 4.0

        self_clock = self.clock
        try:
            Node().callback(SimpleNamespace(object_name='can'))
        finally:
            timer._log, timer._clock = None, st.time.monotonic
        self.assertEqual(self.lines, ['TIMING pick can ok=1 total=4.00 view=4.00 other=0.00'])


if __name__ == '__main__':
    unittest.main()
