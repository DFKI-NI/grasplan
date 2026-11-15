'''
Order open-set grasps by how far the table clearance step (pick.raise_low_grasps) moved them back along their
approach: the further back, the less of the finger pads reaches around the object (rig, heavy lying can, approach 39
deg off vertical: moved back 0.9 cm held, 1.3 cm dropped). Ranking only: nothing is dropped. Pure python, no ROS.
'''

BACKOFF_STEP = 0.005   # m: grasps moved back by about the same amount keep the grasp finder's order


def backoff_step(shift, step=BACKOFF_STEP):
    '''the step index of a back-off (m): 0 for none (or a grasp moved forward), 1 from one step on, ...'''
    if not shift or shift <= 0.0:
        return 0
    return int(shift / step + 1e-9)


def order_by_backoff(grasps, shifts, step=BACKOFF_STEP):
    '''
    grasps best first and one back-off per grasp (m, None or 0 = not moved back) -> the grasps in back-off steps,
    smallest first; a stable sort, so inside a step the original order stays.
    '''
    order = sorted(range(len(grasps)), key=lambda i: backoff_step(shifts[i], step))
    return [grasps[i] for i in order]


def backoff_step_counts(grasps, backoff, step=BACKOFF_STEP):
    '''"step 0: 3, step 1: 2, unknown: 1" for the log: how many of grasps (with .id) are in each back-off step'''
    counts, unknown = {}, 0
    for grasp in grasps:
        if grasp.id not in backoff:
            unknown += 1
            continue
        k = backoff_step(backoff[grasp.id], step)
        counts[k] = counts.get(k, 0) + 1
    parts = [f'step {k}: {n}' for k, n in sorted(counts.items())] + ([f'unknown: {unknown}'] if unknown else [])
    return ', '.join(parts) or 'none'
