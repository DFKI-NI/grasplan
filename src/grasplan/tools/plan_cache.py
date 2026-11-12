'''
A cache of the arm's named-pose plans and their cable guard verdicts, so a named move from (almost) the
same start as an earlier one reuses that plan instead of planning and replaying the cable chain again (1-2.5 s per
move in the sim). ROS-free; mtc_pick_place glues it to the named moves behind the careful flag
~plan_cache_named_moves.

Why it is sound:
- the guard's verdict is a function of the joint path alone (the geometric check and the chain replay from a chain
  settled at the path's first point; mobipick_cable_entanglement.path_check) and of the guard's settings: the same
  path under the same settings gets the same verdict, so a hit reuses the verdict of exactly the trajectory that runs.
  The settings are part of the key.
- a hit runs the cached trajectory unchanged: its first point must be within ``tolerance`` of where the arm stands
  (at most MAX_TOLERANCE_RAD, MoveIt's trajectory_execution/allowed_start_tolerance, so MoveIt accepts the start; the
  default 5 mrad is the rounding of the guard's start-pose cache, whose verdicts were identical on 629 real segments).
- collisions are checked again on every hit, in the current planning scene (objects, octomap, attached object) along
  states no further apart than ``check_states``' step: a scene hash would miss on every octomap update.

Why not the MTC pick plan: in two recorded closed-set sim series the arm stood 5-20 mrad apart at the grasp from one
run to the next (the base slides a little per pick, perception varies), so 0 of 80 picks would have found a reusable
pick plan, and with plan_pick_during_start_move the pick plan already runs while the arm moves.
'''
import math
from collections import OrderedDict

MAX_TOLERANCE_RAD = 0.01   # MoveIt's allowed_start_tolerance (mobipick_moveit_config trajectory_execution.launch.xml)


def start_offset(reference, actual):
    '''largest |actual - reference| (rad) over the joints of reference; inf when actual lacks one or a value is not
    finite. Not wrapped: a full wrist turn is a different cable state.'''
    worst = 0.0
    for name, value in reference.items():
        other = actual.get(name)
        if other is None or not (math.isfinite(value) and math.isfinite(other)):
            return float('inf')
        worst = max(worst, abs(other - value))
    return worst


def check_states(points, step):
    '''the joint vectors to collision-check along a path of joint vectors (lists): every point, plus evenly spaced ones
    between two points that differ by more than step (rad) on some joint'''
    if not points:
        return []
    out = [list(points[0])]
    for a, b in zip(points, points[1:]):
        gap = max((abs(y - x) for x, y in zip(a, b)), default=0.0)
        n = max(1, int(math.ceil(gap / step))) if step > 0 else 1
        for k in range(1, n + 1):
            out.append([x + (y - x) * k / n for x, y in zip(a, b)])
    return out


class PlanCache:
    '''
    Plans by key, each with the joint state it starts from. ``lookup`` returns the stored value whose start is nearest
    to the given one within tolerance; ``store`` keeps at most ``per_key`` plans per key (the oldest goes) and at most
    ``max_keys`` keys (the least recently used goes). ``drop`` forgets one plan (e.g. it collides now). Counts hits
    and misses for the log.
    '''

    def __init__(self, per_key=8, max_keys=64):
        self.per_key, self.max_keys = int(per_key), int(max_keys)
        self.entries = OrderedDict()   # key -> [(start, value)], newest last
        self.hits = self.misses = 0

    def lookup(self, key, start, tolerance):
        '''(value, offset) of the nearest plan of key within tolerance (rad) of start, else (None, nearest offset or
        inf); counts a hit or a miss'''
        best, best_offset = None, float('inf')
        for entry_start, value in self.entries.get(key, []):
            offset = start_offset(entry_start, start)
            if offset < best_offset:
                best, best_offset = value, offset
        if best is not None and best_offset <= tolerance:
            self.entries.move_to_end(key)
            self.hits += 1
            return best, best_offset
        self.misses += 1
        return None, best_offset

    def store(self, key, start, value):
        plans = self.entries.setdefault(key, [])
        plans.append((dict(start), value))
        del plans[:-self.per_key]
        self.entries.move_to_end(key)
        while len(self.entries) > self.max_keys:
            self.entries.popitem(last=False)

    def drop(self, key, value):
        plans = self.entries.get(key, [])
        plans[:] = [(s, v) for s, v in plans if v is not value]
        if not plans:
            self.entries.pop(key, None)

    def counts(self):
        return f'{self.hits} hits, {self.misses} misses'
