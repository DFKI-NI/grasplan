'''
Order grasps by the arm cable stretch the cable guard predicts at their IK solution (#151): the MTC cable guard
(mtc_pick_place.CableGuard) rejects a plan whose stretch exceeds ~mtc_cable_max_stretch, so grasps predicted to pass
at their grasp pose are offered first. Ranking only: nothing is dropped. The guard checks the whole path, the grasp
end state is a proxy for it. Pure python, no ROS.
'''


def order_by_predicted_stretch(grasps, stretches, max_stretch):
    '''
    grasps best first and one predicted stretch per grasp (None = no prediction) -> (ordered grasps, passing count):
    the grasps predicted to pass (stretch <= max_stretch) first, then the others, each group in its original order.
    '''
    passing = [g for g, s in zip(grasps, stretches) if s is not None and s <= max_stretch]
    others = [g for g, s in zip(grasps, stretches) if s is None or s > max_stretch]
    return passing + others, len(passing)


def joint_path(start, goal, samples=10):
    '''
    joint states (dicts name -> rad) from start to goal, joint-linear, samples steps, goal included; joints missing in
    start keep their goal value. start None -> only the goal. A cheap stand-in for a planned path to the same goal.
    '''
    if not start:
        return [dict(goal)]
    path = []
    for step in range(1, samples + 1):
        a = step / float(samples)
        path.append({name: (start[name] + (value - start[name]) * a) if name in start else value
                     for name, value in goal.items()})
    return path
