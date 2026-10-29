'''the cable model's path search (tools/cable_path.py, cable_shaping.verified_path) on synthetic stretch grids,
and on the real model (mobipick_cable_entanglement, amenable_ws) with a coarse grid. No ROS master.'''
import math
import os
import tempfile
import unittest

import numpy as np

from grasplan.tools import cable_path as cp

MARGIN = -0.1
R = math.radians


def wall_grid(gap=True):
    '''2-D span over (wrist_1, wrist_2), 0..180 deg in 10 deg: a stretched wall at wrist_1 = 90 deg, with a gap at
    wrist_2 = 160..180 deg when gap'''
    a = np.radians(np.arange(0, 181, 10.0))
    values = np.full((len(a), len(a)), -0.3)
    values[9, :] = 0.2
    if gap:
        values[9, 16:] = -0.3
    return cp.CableGrid([cp.SpanGrid('wrist', ['wrist_1', 'wrist_2'], [a, a], values)])


def q(w1, w2, pan=0.0):
    return {'shoulder_pan': R(pan), 'wrist_1': R(w1), 'wrist_2': R(w2)}


class TestSlackPath(unittest.TestCase):
    def test_slack_line_needs_no_waypoint(self):
        found = cp.slack_path(wall_grid(), q(10, 20), q(60, 120), MARGIN)
        self.assertEqual(found[0], [])
        self.assertLessEqual(found[1], MARGIN)

    def test_goes_around_the_wall_through_the_gap(self):
        grid = wall_grid()
        start, goal = q(20, 20, pan=0), q(160, 20, pan=90)
        self.assertGreater(grid.worst_along(start, goal), MARGIN)   # the straight line crosses the wall
        waypoints, worst = cp.slack_path(grid, start, goal, MARGIN)
        self.assertTrue(waypoints)
        self.assertLessEqual(worst, MARGIN)
        self.assertTrue(any(w['wrist_2'] >= R(155) for w in waypoints))   # through the gap
        pans = [w['shoulder_pan'] for w in waypoints]
        self.assertEqual(pans, sorted(pans))   # the joint no span depends on moves monotonically
        self.assertTrue(all(0 < p < R(90) for p in pans))

    def test_no_path_through_a_closed_wall(self):
        self.assertIsNone(cp.slack_path(wall_grid(gap=False), q(20, 20), q(160, 20), MARGIN))

    def test_a_stretched_start_may_leave(self):
        grid = wall_grid()
        start = q(90, 50)   # on the wall
        found = cp.slack_path(grid, start, q(20, 50), MARGIN)
        self.assertIsNotNone(found)
        self.assertLessEqual(found[1], grid.stretch(start) + 1e-9)

    def test_interpolation_and_upper_bound(self):
        span = wall_grid().spans[0]
        x = np.array([R(85), R(50)])
        self.assertAlmostEqual(span.stretch(x), -0.3 + 0.5 * 0.5, places=6)
        self.assertAlmostEqual(span.stretch_upper(x), 0.2)


class TestCorridor(unittest.TestCase):
    def test_box_shrinks_away_from_the_wall(self):
        box = cp.corridor(wall_grid(), q(40, 20), q(70, 60), MARGIN, pad_deg=30.0)
        self.assertIsNotNone(box)
        self.assertLess(box['wrist_1'][1], R(90))     # 70 + 30 would reach the wall: shrunk
        self.assertLessEqual(box['wrist_1'][0], R(40))
        self.assertGreaterEqual(box['wrist_2'][1], R(60))

    def test_window_clips(self):
        box = cp.corridor(wall_grid(), q(10, 20), q(40, 60), MARGIN, window={'wrist_2': (R(30), R(70))})
        self.assertAlmostEqual(box['wrist_2'][0], R(30))

    def test_no_box_across_the_wall(self):
        self.assertIsNone(cp.corridor(wall_grid(), q(20, 20), q(160, 20), MARGIN))


class TestGridFile(unittest.TestCase):
    def test_save_and_load(self):
        grid = wall_grid()
        grid.key = 'abc'
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'sub', 'g.npz')
            grid.save(path)
            loaded = cp.CableGrid.load(path)
        self.assertEqual(loaded.key, 'abc')
        self.assertEqual(loaded.joints, ['wrist_1', 'wrist_2'])
        np.testing.assert_array_equal(loaded.spans[0].values, grid.spans[0].values)

    def test_build_from_an_evaluator(self):
        grid = cp.build_grid(lambda qq: {'wrist': qq['wrist_1'] - qq['wrist_2']}, {'wrist': ['wrist_1', 'wrist_2']},
                             {'wrist_1': (0.0, 1.0), 'wrist_2': (0.0, 0.5)}, step_deg=20.0)
        self.assertAlmostEqual(grid.stretch({'wrist_1': 0.7, 'wrist_2': 0.2}), 0.5, places=9)   # linear: exact

    def test_key_changes_with_the_config(self):
        lim = {'wrist_1': (0.0, 1.0)}
        self.assertNotEqual(cp.grid_key({'a': 1}, 'u', 10, lim), cp.grid_key({'a': 2}, 'u', 10, lim))


class TestBranches(unittest.TestCase):
    LIMITS = {'shoulder_pan': (-math.pi, math.pi), 'shoulder_lift': (-math.pi, 0.74), 'elbow': (0.0, 2.93),
              'wrist_1': (-math.pi, 0.0), 'wrist_2': (-math.pi, math.pi), 'wrist_3': (R(30), R(275))}

    def test_eight_seeds_own_branch_first_within_limits(self):
        start = {j: R(v) for j, v in zip(cp.UR_JOINTS, [-90, -90, 114.6, -43, 90, 80])}
        seeds = cp.ur_branch_seeds(start, self.LIMITS)
        self.assertEqual(len(seeds), 8)
        for j in cp.UR_JOINTS:
            self.assertAlmostEqual(seeds[0][j], start[j])
        for s in seeds:
            for j, (lo, hi) in self.LIMITS.items():
                self.assertTrue(lo - 1e-6 <= s[j] <= hi + 1e-6, (j, s[j]))
        wrist = seeds[1]   # the wrist flip alone: exact for a UR (wrist_1 + 180 leaves its limits here: clamped)
        self.assertAlmostEqual(wrist['wrist_2'], -start['wrist_2'])
        self.assertAlmostEqual(math.remainder(wrist['wrist_3'] - start['wrist_3'] - math.pi, 2 * math.pi), 0.0)

    def test_dedupe(self):
        a = {'x': 0.0, 'y': 1.0}
        self.assertEqual(len(cp.dedupe_branches([a, {'x': 0.001, 'y': 1.0}, {'x': 1.0, 'y': 1.0}])), 2)

    def test_rank_prefers_the_slack_goal(self):
        grid = wall_grid()
        start = q(20, 20)
        stretched_goal, slack_goal = q(90, 40), q(160, 40)
        ranked = cp.rank_branches(grid, start, [stretched_goal, slack_goal], MARGIN)
        self.assertIs(ranked[0][0], slack_goal)


class TestVerifiedPath(unittest.TestCase):
    def test_offset_grows_until_the_exact_check_holds(self):
        from grasplan.cable_shaping import verified_path
        grid = wall_grid()
        # the exact model is 0.05 worse than the grid everywhere: the plain margin fails, a larger offset holds
        exact = lambda qq: grid.stretch(qq) + 0.05  # noqa: E731
        found = verified_path(grid, exact, q(20, 20), q(160, 20), MARGIN, offsets=(0.0, 0.1, 0.2))
        waypoints, worst, offset, line_slack = found
        self.assertTrue(waypoints)
        self.assertFalse(line_slack)
        self.assertGreaterEqual(offset, 0.0)
        self.assertLessEqual(worst, MARGIN + 0.05 + 1e-9)

    def test_slack_line_is_kept(self):
        from grasplan.cable_shaping import verified_path
        grid = wall_grid()
        found = verified_path(grid, grid.stretch, q(10, 20), q(60, 120), MARGIN)
        self.assertEqual(found[0], [])
        self.assertTrue(found[3])


class TestAcceptance(unittest.TestCase):
    """the acceptance cases of the path shaping design"""

    def test_peak_between_grid_nodes_is_reported_not_hidden(self):
        from grasplan.cable_shaping import verified_path
        grid = wall_grid()
        # the exact model has a narrow peak at wrist_2 = 45 deg (between nodes 40 and 50), the grid does not see it
        exact = lambda qq: max(grid.stretch(qq), 0.3 if abs(qq['wrist_2'] - R(45)) < R(1) else -1)  # noqa: E731
        found = verified_path(grid, exact, q(10, 20), q(60, 120), MARGIN)
        self.assertGreater(found[1], MARGIN)          # the exact re-check finds the peak
        self.assertFalse(found[3])

    def test_off_grid_endpoints(self):
        found = cp.slack_path(wall_grid(), q(23.7, 11.1), q(157.3, 13.9), MARGIN)
        self.assertIsNotNone(found)
        self.assertLessEqual(found[1], MARGIN)

    def test_windings_two_pi_apart_are_different_states(self):
        a = np.radians(np.arange(-360, 361, 10.0))
        values = np.where(np.abs(a) > R(200), 0.2, -0.3)   # one winding stretched, its 2 pi twin slack
        grid = cp.CableGrid([cp.SpanGrid('wrist', ['wrist_3'], [a], values)])
        self.assertLess(grid.stretch({'wrist_3': R(0)}), MARGIN)
        self.assertGreater(grid.stretch({'wrist_3': R(360)}), MARGIN)

    def test_key_changes_with_urdf_margin_free_limits(self):
        lim = {'wrist_1': (0.0, 1.0)}
        base = cp.grid_key({'anchor': 'support'}, 'urdf', 10, lim)
        self.assertNotEqual(base, cp.grid_key({'anchor': 'carrier_tip'}, 'urdf', 10, lim))
        self.assertNotEqual(base, cp.grid_key({'anchor': 'support'}, 'urdf2', 10, lim))
        self.assertNotEqual(base, cp.grid_key({'anchor': 'support'}, 'urdf', 10, {'wrist_1': (0.0, 1.1)}))
        self.assertNotEqual(base, cp.grid_key({'anchor': 'support'}, 'urdf', 5, lim))

    def test_margin_change_uses_its_own_slack_graph(self):
        grid = wall_grid()
        tight = cp.slack_path(grid, q(20, 20), q(160, 20), -0.5)   # nothing is slack at -0.5
        loose = cp.slack_path(grid, q(20, 20), q(160, 20), MARGIN)
        self.assertIsNone(tight)
        self.assertIsNotNone(loose)


class TestRealModel(unittest.TestCase):
    '''the wrist span of the committed cable model on a coarse (20 deg) grid'''

    @classmethod
    def setUpClass(cls):
        try:
            import rospkg
            import yaml
            from mobipick_cable_entanglement.cable_model import CableModel, config_from_dict
            pkg = rospkg.RosPack().get_path('mobipick_cable_entanglement')
        except Exception as e:  # noqa: BLE001
            raise unittest.SkipTest(f'cable model not available: {e}')
        cfg = config_from_dict(yaml.safe_load(open(os.path.join(pkg, 'config', 'cable_model.yaml'))))
        cfg.cable_carrier.enabled = False
        cls.model = CableModel(open(os.path.join(pkg, 'test', 'mobipick_sim_robot_description.urdf')).read(), cfg)
        cls.limits = {'wrist_1': (-math.pi, 0.0), 'wrist_2': (-math.pi, math.pi), 'wrist_3': (R(30), R(275))}
        cls.grid = cp.build_grid(cp.model_evaluator(cls.model), cp.model_span_joints(cls.model), cls.limits, 20.0,
                                 reference={'shoulder_pan': 0.0, 'shoulder_lift': -2.6, 'elbow': 2.43})

    def evaluate_max(self, qq):
        return self.model.evaluate(qq).max_stretch

    def test_wrist_span_ignores_the_big_joints(self):
        a = {j: R(v) for j, v in zip(cp.UR_JOINTS, [0, -149, 139, -117, -90, 180])}
        b = dict(a, shoulder_pan=R(-90), shoulder_lift=R(-60), elbow=R(40))
        self.assertAlmostEqual(self.model.evaluate(a).spans['wrist'].stretch,
                               self.model.evaluate(b).spans['wrist'].stretch, places=9)

    def test_shaped_path_is_slacker_than_the_line(self):
        from grasplan.cable_shaping import verified_path
        rng = np.random.default_rng(3)
        better = tried = 0
        while tried < 6:
            pair = []
            for _ in range(2):
                w = {j: rng.uniform(*self.limits[j]) for j in self.limits}
                pair.append(dict(w, shoulder_pan=0.0, shoulder_lift=R(-90), elbow=R(90)))
            if max(self.evaluate_max(p) for p in pair) > -0.115:
                continue   # only pairs whose ends are slack: a stretched end decides the worst point anyway
            line = max(self.evaluate_max({j: pair[0][j] + (pair[1][j] - pair[0][j]) * k / 24 for j in pair[0]})
                       for k in range(25))
            if line <= -0.115:
                continue   # the line is slack: nothing to shape
            tried += 1
            found = verified_path(self.grid, self.evaluate_max, pair[0], pair[1], -0.115)
            if found is None:
                continue
            self.assertLessEqual(found[1], line + 2e-3)   # never worse than the straight line (sampling differs)
            better += found[1] < line - 1e-3
        self.assertGreater(better, 0)

if __name__ == '__main__':
    unittest.main()
