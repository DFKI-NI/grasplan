'''unit tests of grasplan.tools.cable_rank (#151, no ROS): python3 -m pytest <this file>'''
from grasplan.tools.cable_rank import order_by_predicted_stretch


def test_predicted_passing_grasps_come_first_in_their_own_order():
    # night13 guard results (stretch, limit -0.115): only a grasp below the limit can be executed
    grasps = ['g1', 'g6', 'g15', 'g24', 'g30', 'g31']
    stretches = [-0.080, 0.081, -0.140, -0.112, None, -0.130]
    ordered, passing = order_by_predicted_stretch(grasps, stretches, -0.115)
    assert ordered == ['g15', 'g31', 'g1', 'g6', 'g24', 'g30']
    assert passing == 2


def test_nothing_is_dropped_and_no_prediction_keeps_the_order():
    ordered, passing = order_by_predicted_stretch(['a', 'b'], [None, None], -0.115)
    assert ordered == ['a', 'b'] and passing == 0


def test_joint_path_is_linear_ends_at_the_goal_and_keeps_joints_missing_in_the_start():
    from grasplan.tools.cable_rank import joint_path

    path = joint_path({'a': 0.0, 'b': 1.0}, {'a': 1.0, 'b': 3.0, 'c': 5.0}, samples=4)
    assert len(path) == 4
    assert path[0] == {'a': 0.25, 'b': 1.5, 'c': 5.0}
    assert path[-1] == {'a': 1.0, 'b': 3.0, 'c': 5.0}
    assert joint_path(None, {'a': 2.0}) == [{'a': 2.0}]


def test_gate_chunks_never_mix_predicted_pass_and_held_back_grasps():
    from grasplan.tools.cable_rank import gate_chunks

    # 09:23 tennis ball: 6 reachable, 2 predicted to pass, batch 3 -> [p p] first, not [p p f]
    assert gate_chunks(6, 2, 3) == [(0, 2), (2, 5), (5, 6)]
    assert gate_chunks(12, 4, 3) == [(0, 3), (3, 4), (4, 7), (7, 10), (10, 12)]
    assert gate_chunks(6, 0, 3) == [(0, 3), (3, 6)]     # no prediction: batches as before
    assert gate_chunks(6, 6, 3) == [(0, 3), (3, 6)]
    assert gate_chunks(5, 2, 0) == [(0, 2), (2, 5)]     # no batching: one call per group
    assert gate_chunks(0, 0, 3) == []
