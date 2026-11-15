'''unit tests of grasplan.tools.backoff_rank (no ROS): python3 -m pytest <this file>'''
from grasplan.tools.backoff_rank import backoff_step, order_by_backoff


def test_less_moved_back_grasps_come_first():
    # recorded heavy-can pick: two grasps moved back 1.29 and 2.18 cm (both lost) ahead of grasps that needed none
    grasps = ['ag3', 'ag5', 'ag1', 'ag9', 'ag2']
    shifts = [0.0129, 0.0218, None, 0.009, 0.0]
    assert order_by_backoff(grasps, shifts) == ['ag1', 'ag2', 'ag9', 'ag3', 'ag5']


def test_inside_a_step_the_finder_order_stays():
    # 0.1, 0.4 and 0 mm are one step (0); 6 and 9 mm another (1): the original order inside each
    grasps = ['a', 'b', 'c', 'd', 'e']
    shifts = [0.009, 0.0004, 0.006, 0.0, 0.0001]
    assert order_by_backoff(grasps, shifts) == ['b', 'd', 'e', 'a', 'c']


def test_no_back_off_keeps_the_order_and_nothing_is_dropped():
    grasps = ['x', 'y', 'z']
    assert order_by_backoff(grasps, [None, 0.0, None]) == grasps
    assert order_by_backoff([], []) == []


def test_step_edges_and_forward_moves():
    assert backoff_step(0.005) == 1          # exactly one step, no float rounding down
    assert backoff_step(0.0049) == 0
    assert backoff_step(0.0129) == 2
    assert backoff_step(-0.01) == 0          # moved forward (a low object's sink): no back-off
    assert backoff_step(None) == 0


def test_cable_pass_prefix_stays_first():
    # sorted by back-off once, then the cable ranking partitions stably: cable pass first, back-off order inside each
    from grasplan.tools.cable_rank import order_by_predicted_stretch

    grasps = ['far_pass', 'near_fail', 'near_pass', 'mid_fail']
    by_backoff = order_by_backoff(grasps, [0.03, 0.0, 0.002, 0.012])
    assert by_backoff == ['near_fail', 'near_pass', 'mid_fail', 'far_pass']
    stretch = {'far_pass': -0.2, 'near_fail': 0.1, 'near_pass': -0.2, 'mid_fail': 0.1}
    ordered, passing = order_by_predicted_stretch(by_backoff, [stretch[g] for g in by_backoff], -0.115)
    assert ordered == ['near_pass', 'far_pass', 'near_fail', 'mid_fail'] and passing == 2


def test_recorded_lying_can_candidates():
    # sim 2026-11-15 07:02 (lying soup can, 0.01 kg): reachable collision-free anygrasp_15 (+2.0 cm at 45 deg),
    # _21 (+0.4 cm at 39 deg), _30 / _31 (+0.6 cm at 70 deg), all predicted to pass the cable guard; _15 was executed
    # and lost at the lift. Back-off = raise / cos tilt.
    import math

    raise_tilt = {'anygrasp_15': (0.020, 45), 'anygrasp_21': (0.004, 39), 'anygrasp_30': (0.006, 70),
                  'anygrasp_31': (0.006, 70)}
    grasps = list(raise_tilt)
    shifts = [r / math.cos(math.radians(t)) for r, t in raise_tilt.values()]
    assert order_by_backoff(grasps, shifts) == ['anygrasp_21', 'anygrasp_30', 'anygrasp_31', 'anygrasp_15']


def test_step_counts_for_the_log():
    from types import SimpleNamespace as NS

    from grasplan.tools.backoff_rank import backoff_step_counts

    grasps = [NS(id=i) for i in ('a', 'b', 'c', 'd')]
    assert backoff_step_counts(grasps, {'a': 0.0, 'b': 0.012, 'c': 0.001}) == 'step 0: 2, step 2: 1, unknown: 1'
    assert backoff_step_counts([], {}) == 'none'
