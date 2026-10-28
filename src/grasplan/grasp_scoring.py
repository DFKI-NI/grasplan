#!/usr/bin/env python3

# Copyright (c) 2026 DFKI GmbH
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

'''
Ranking of external (AnyGrasp-style) grasp candidates.

The AnyGrasp action server and its recorded-grasp mockup share this module so
both rank candidates identically: the raw network score ("nn") is multiplied
by a top-down bonus that grows with how well the approach direction points
straight down in a gravity-aligned frame, and by an extra straight bonus for
approaches within a few degrees of vertical: a tilted gripper dips one finger
towards the table (#134), so near-vertical grasps must win clearly. Every
function is ROS-free apart from the GraspCandidate message fields it reads and
writes.
'''

import math

from grasplan.grasp_markers import make_candidate_scores

NN_SCORE_NAME = 'nn'
# default straight bonus of the AnyGrasp server and the mockup (#134): x1.5 at vertical, fading out at 20 deg
STRAIGHT_GRASP_SCORE_MULTIPLIER = 1.5
STRAIGHT_GRASP_MAX_ANGLE_DEG = 20.0


def top_down_alignment(approach_vector):
    '''Fraction in [0, 1] of how well an approach vector (x, y, z) points along -z of its frame.'''
    x, y, z = approach_vector
    norm = math.sqrt(x * x + y * y + z * z)
    if norm == 0.0:
        return 0.0
    return max(0.0, min(1.0, -z / norm))


def top_down_multiplier(alignment, score_multiplier, straight_multiplier=1.0, straight_max_angle=0.0):
    '''
    Linear bonus: 1.0 for a horizontal approach, score_multiplier for a vertical one; times the straight bonus,
    which grows linearly from 1.0 at straight_max_angle (rad) off vertical to straight_multiplier at vertical.
    '''
    top_down = 1.0 + (score_multiplier - 1.0) * alignment
    return top_down * straight_bonus(alignment, straight_multiplier, straight_max_angle)


def straight_bonus(alignment, straight_multiplier, straight_max_angle):
    '''1.0 beyond straight_max_angle (rad) off vertical, rising linearly in the angle to straight_multiplier.'''
    if straight_multiplier == 1.0 or straight_max_angle <= 0.0:
        return 1.0
    angle = math.acos(max(-1.0, min(1.0, alignment)))
    return 1.0 + (straight_multiplier - 1.0) * max(0.0, 1.0 - angle / straight_max_angle)


def nn_score(candidate):
    '''The raw network score of a candidate, falling back to quality for legacy producers.'''
    for score in getattr(candidate, 'scores', []):
        if score.name == NN_SCORE_NAME:
            return float(score.value)
    return float(candidate.quality)


def apply_top_down_bonus(
    candidates, approach_vectors, score_multiplier, straight_multiplier=1.0, straight_max_angle=0.0
):
    '''
    Re-score candidates in place: quality = nn * top_down_multiplier, scores = [nn, top_bonus, final].

    approach_vectors: one (x, y, z) per candidate, expressed in the gravity-aligned frame.
    '''
    for candidate, approach in zip(candidates, approach_vectors):
        raw = nn_score(candidate)
        multiplier = top_down_multiplier(
            top_down_alignment(approach), score_multiplier, straight_multiplier, straight_max_angle
        )
        candidate.quality = raw * multiplier
        candidate.scores = make_candidate_scores(candidate.quality, nn=raw, top_bonus=multiplier)


def rank_by_quality(candidates):
    '''Return the candidates ordered best first; ties keep their input order.'''
    return sorted(candidates, key=lambda candidate: candidate.quality, reverse=True)


# ---------------------------------------------------------------- long cylinders: side grasps (Oscar, 2026-09-28)
# A top grasp of a standing Pringles can closed on nothing on the real robot; across a long cylinder is the grasp that
# holds. A box counts as a long cylinder by its shape, not its height: the longest side at least CYLINDER_MIN_ELONGATION
# times the middle one, and a roughly round cross-section (middle side at most CYLINDER_MAX_ROUNDNESS times the
# shortest). Pringles 8.6 x 9.9 x 23.7 cm and a coke can 6.6 x 6.6 x 12 cm are; a sugar box 4 x 9 x 17 cm is not.
CYLINDER_MIN_ELONGATION = 1.5
CYLINDER_MAX_ROUNDNESS = 1.3
SIDE_GRASP_SCORE_MULTIPLIER = 1.5
# By size (decided 2026-09-28): an object whose longest side is below TINY_OBJECT_MAX_SIZE (m) keeps the strict top-down
# ranking; a compact round one (all sides within ROUND_MAX_RATIO of each other, e.g. a ball) is grasped from the side
# at its centre height; flat or elongated ones stay top-down.
TINY_OBJECT_MAX_SIZE = 0.06
ROUND_MAX_RATIO = 1.3
STANDING_MAX_TILT_DEG = 35.0   # a lying cylinder (banana, a can on its side) keeps top-down grasps across it


def _unit(v):
    norm = math.sqrt(sum(c * c for c in v))
    return tuple(c / norm for c in v) if norm > 0.0 else (0.0, 0.0, 0.0)


def cylinder_axis(size, rotation=None, min_elongation=CYLINDER_MIN_ELONGATION, max_roundness=CYLINDER_MAX_ROUNDNESS):
    '''
    (axis, length) of a box that is a long cylinder, else None. size: (x, y, z) side lengths; rotation: 3x3 rows of
    the box orientation in the frame the axis is wanted in (default: the box is axis-aligned).
    '''
    dims = [float(s) for s in size]
    if min(dims) <= 0.0:
        return None
    order = sorted(range(3), key=lambda i: dims[i])
    short, middle, long_ = (dims[i] for i in order)
    if long_ < min_elongation * middle or middle > max_roundness * short:
        return None
    rotation = rotation or ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    column = order[2]
    return _unit(tuple(rotation[row][column] for row in range(3))), long_


def cylinder_side_multiplier(approach, closing, offset, axis, length, score_multiplier=SIDE_GRASP_SCORE_MULTIPLIER):
    '''
    Bonus for grasping across a long cylinder: up to score_multiplier when the approach and the finger closing
    direction are both perpendicular to its axis (sin of their angles to it) and the grasp centre lies in the middle
    half of its length, fading to 1.0 towards the ends. offset: grasp centre minus box centre, same frame as axis.
    '''
    a, c = _unit(approach), _unit(closing)
    sin_a = math.sqrt(max(0.0, 1.0 - sum(x * y for x, y in zip(a, axis)) ** 2))
    sin_c = math.sqrt(max(0.0, 1.0 - sum(x * y for x, y in zip(c, axis)) ** 2))
    along = abs(sum(x * y for x, y in zip(offset, axis))) / max(length / 2.0, 1e-9)
    middle = 1.0 if along <= 0.5 else max(0.0, (1.0 - along) / 0.5)
    return 1.0 + (score_multiplier - 1.0) * sin_a * sin_c * middle


def apply_cylinder_side_bonus(candidates, grasp_axes, centre, axis, length, score_multiplier=SIDE_GRASP_SCORE_MULTIPLIER):
    '''
    Re-score candidates in place for a long cylinder: quality = nn * side bonus (no top-down bonus),
    scores = [nn, side_bonus, final]. grasp_axes: one (approach, closing, position) per candidate, in the frame of
    centre and axis.
    '''
    for candidate, (approach, closing, position) in zip(candidates, grasp_axes):
        raw = nn_score(candidate)
        offset = tuple(p - c for p, c in zip(position, centre))
        multiplier = cylinder_side_multiplier(approach, closing, offset, axis, length, score_multiplier)
        candidate.quality = raw * multiplier
        candidate.scores = make_candidate_scores(candidate.quality, nn=raw, side_bonus=multiplier)


def is_compact_round(size):
    '''True for a roughly round object box (all sides within ROUND_MAX_RATIO of each other: a small round object): a
    two-finger grasp of it holds the same when turned about its approach axis.'''
    dims = [float(v) for v in size]
    return min(dims) > 0.0 and max(dims) <= ROUND_MAX_RATIO * min(dims)


def side_grasp_axis(size, rotation=None, up=(0.0, 0.0, 1.0)):
    '''
    (axis, length) along which side grasps should close across, or None for top-down ranking (Oscar, 2026-09-28):
    tiny objects (longest side < TINY_OBJECT_MAX_SIZE) None; a STANDING long cylinder (axis within
    STANDING_MAX_TILT_DEG of ``up``) its own axis, a lying one (banana) None; a compact round object
    (all sides within ROUND_MAX_RATIO) the vertical ``up`` with its height; anything else (flat, elongated) None.
    size/rotation as for cylinder_axis; up in the frame of rotation.
    '''
    dims = [float(v) for v in size]
    if min(dims) <= 0.0 or max(dims) < TINY_OBJECT_MAX_SIZE:
        return None
    up = _unit(up)
    cylinder = cylinder_axis(dims, rotation)
    if cylinder is not None:
        standing = abs(sum(a * u for a, u in zip(cylinder[0], up))) >= math.cos(math.radians(STANDING_MAX_TILT_DEG))
        return cylinder if standing else None
    if max(dims) > ROUND_MAX_RATIO * min(dims):
        return None
    rotation = rotation or ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    height = sum(abs(sum(up[r] * rotation[r][c] for r in range(3))) * dims[c] for c in range(3))
    return up, height


# ---------------------------------------------------------------- low objects: top grasps only (Oscar, 2026-09-28)
# A real strawberry (#151) was grasped 16 deg off vertical with the fingertips at its top and slipped out in the lift. An
# object lower than LOW_OBJECT_MAX_HEIGHT is grasped from the top only: within LOW_OBJECT_MAX_TILT_DEG of straight down,
# as low as the fingertip clearance above the table allows (Oscar: below 5 cm, the strawberry yes, the 6.4 cm apple and
# the tennis ball keep their side grasps). A small dark object often has no real depth and its box height is filled in
# (the strawberry's came out 7.2 cm, taller than the apple's), so both horizontal sides below SMALL_OBJECT_MAX_FOOTPRINT
# count as low too.
LOW_OBJECT_MAX_HEIGHT = 0.05
SMALL_OBJECT_MAX_FOOTPRINT = 0.06
LOW_OBJECT_MAX_TILT_DEG = 10.0


def box_height_and_footprint(size, rotation=None, up=(0.0, 0.0, 1.0)):
    '''(extent along up, longer of the two box sides most perpendicular to up) of a box; size/rotation as for
    cylinder_axis, up in the frame of rotation'''
    dims = [float(v) for v in size]
    rotation = rotation or ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    up = _unit(up)
    along = [abs(sum(up[r] * rotation[r][c] for r in range(3))) for c in range(3)]
    height = sum(a * d for a, d in zip(along, dims))
    return height, max(dims[c] for c in sorted(range(3), key=lambda c: along[c])[:2])


def is_low_object(size, rotation=None, max_height=LOW_OBJECT_MAX_HEIGHT, max_footprint=SMALL_OBJECT_MAX_FOOTPRINT,
                  up=(0.0, 0.0, 1.0)):
    '''True for an object to grasp from the top only: lower than max_height, or both horizontal sides below
    max_footprint (0 = no footprint rule); a box without a size never is'''
    if len(size) != 3 or min(float(v) for v in size) <= 0.0:
        return False
    height, footprint = box_height_and_footprint(size, rotation, up)
    return height < max_height or footprint < max_footprint


def tilt_from_vertical_deg(approach, up=(0.0, 0.0, 1.0)):
    '''angle (deg) between an approach direction and straight down (-up)'''
    a, u = _unit(approach), _unit(up)
    return math.degrees(math.acos(max(-1.0, min(1.0, -sum(x * y for x, y in zip(a, u))))))


# ---------------------------------------------------------------- one-sided boxes (2026-09-28, real Pringles can)
# The grasp view sees only the front of an object: the box of a standing Pringles can came out 7.3 x 3.6 x 24 cm for a
# ~7.5 cm can, so the planning scene lacked its back half (the ranking uses the fuller committed box, or widens a
# one-view box itself). A standing elongated box (height at least CYLINDER_MIN_ELONGATION times its wider horizontal
# side) whose side along the line of sight is the shorter one gets a square footprint, extended away from the camera;
# flat, lying and wide boxes stay as measured.


def complete_one_sided_box(size, rotation, centre, sight, min_elongation=CYLINDER_MIN_ELONGATION):
    '''
    (size, centre) of a box seen from one side, completed as above, or None when it stays. size: (x, y, z) side lengths
    with the box z axis vertical (within STANDING_MAX_TILT_DEG); rotation: 3x3 rows of the box orientation; centre:
    (x, y, z); sight: direction from the camera to the box in the same frame (its vertical part is ignored).
    '''
    dims = [float(v) for v in size]
    s = _unit((float(sight[0]), float(sight[1]), 0.0))
    if min(dims) <= 0.0 or s == (0.0, 0.0, 0.0):
        return None
    axes = [tuple(float(rotation[r][c]) for r in range(3)) for c in range(3)]
    if abs(axes[2][2]) < math.cos(math.radians(STANDING_MAX_TILT_DEG)):
        return None
    along = [sum(a * b for a, b in zip(axes[c], s)) for c in (0, 1)]
    depth_axis = 0 if abs(along[0]) >= abs(along[1]) else 1
    depth, width = dims[depth_axis], dims[1 - depth_axis]
    if depth >= width or dims[2] < min_elongation * width:
        return None
    grow = (width - depth) / 2.0 * (1.0 if along[depth_axis] >= 0.0 else -1.0)
    completed = list(dims)
    completed[depth_axis] = width
    return completed, tuple(float(c) + grow * a for c, a in zip(centre, axes[depth_axis]))


# ---------------------------------------------------------------- jaw along a box axis (2026-10-27, sim)
# A parallel jaw across a box diagonal touches two opposite edges only: on a flat box whose long side crosses the jaw
# the gripper stalled at the top-face diagonal and the box slid out in the lift. Every reachable AnyGrasp grasp of that
# box was 28-44 deg off the box axes; turned about its approach onto a box axis, the same grasp closes on two faces.
ALIGN_JAW_MIN_OFF_DEG = 15.0   # a jaw closer than this to a box axis is left as it is
ALIGN_JAW_MAX_TILT_DEG = 20.0  # only near-vertical grasps: turning those about the approach keeps the jaw horizontal


def jaw_alignment_turn(approach, jaw, size, rotation, max_span, min_off_deg=ALIGN_JAW_MIN_OFF_DEG,
                       max_tilt_deg=ALIGN_JAW_MAX_TILT_DEG, up=(0.0, 0.0, 1.0)):
    '''
    (turn_deg, span) that puts a grasp's jaw along the nearest horizontal box axis whose side is at most max_span, or
    None: the grasp is more than max_tilt_deg off vertical, its jaw is within min_off_deg of a horizontal box axis
    already, or no side fits. turn_deg is right-handed about the approach (the gripper_tcp x axis, as in
    mtc_pick_place.turn_grasp_about_approach), in [-90, 90); span is the side the jaw then closes across.
    approach, jaw: the grasp's x and y axes; size: box side lengths; rotation: 3x3 rows of the box orientation; all in
    the frame of up.
    '''
    turns = jaw_alignment_turns(approach, jaw, size, rotation, max_span, min_off_deg=min_off_deg,
                                max_tilt_deg=max_tilt_deg, up=up)
    return turns[0] if turns else None


def _jaw_side_turns(approach, jaw, size, rotation, max_tilt_deg, up):
    '''
    for a grasp within max_tilt_deg of straight down: (turn_deg, span) per horizontal box side, the turn about the
    approach that lays the jaw along that box axis and the box size along it, which the jaw then closes across;
    [] for a tilted grasp or a degenerate box
    '''
    dims = [float(v) for v in size]
    if len(dims) != 3 or min(dims) <= 0.0 or tilt_from_vertical_deg(approach, up) > max_tilt_deg:
        return []
    a, j, u = _unit(approach), _unit(jaw), _unit(up)
    axes = [tuple(float(rotation[r][c]) for r in range(3)) for c in range(3)]
    horizontal = sorted(range(3), key=lambda c: abs(sum(x * y for x, y in zip(axes[c], u))))[:2]
    turns = []
    for c in horizontal:
        # the box axis in the plane of the jaw's turn, then the signed angle from the jaw to it about the approach
        along = sum(x * y for x, y in zip(axes[c], a))
        e = _unit(tuple(x - along * y for x, y in zip(axes[c], a)))
        cross = (j[1] * e[2] - j[2] * e[1], j[2] * e[0] - j[0] * e[2], j[0] * e[1] - j[1] * e[0])
        angle = math.degrees(math.atan2(sum(x * y for x, y in zip(cross, a)), sum(x * y for x, y in zip(j, e))))
        turns.append(((angle + 90.0) % 180.0 - 90.0, dims[c]))   # a jaw is the same line both ways
    return turns


def jaw_alignment_turns(approach, jaw, size, rotation, max_span, narrowest=False, min_off_deg=ALIGN_JAW_MIN_OFF_DEG,
                        max_tilt_deg=ALIGN_JAW_MAX_TILT_DEG, up=(0.0, 0.0, 1.0)):
    '''
    the turns (turn_deg, span) to offer for one grasp, best first, as in jaw_alignment_turn. narrowest False: at most
    the one jaw_alignment_turn gives. narrowest True: first the turn onto the narrowest horizontal side that fits
    (unless the jaw lies within min_off_deg of it already, also when it closes across the wider side now), then the
    turn onto the nearest fitting side when that is another one and the jaw is off both axes. Equal sides count as one.
    '''
    turns = _jaw_side_turns(approach, jaw, size, rotation, max_tilt_deg, up)
    fitting = [(turn, span) for turn, span in turns if span <= max_span]
    if not fitting:
        return []
    on_an_axis = min(abs(turn) for turn, _ in turns) <= min_off_deg
    nearest = min(fitting, key=lambda t: abs(t[0]))
    if not narrowest:
        return [] if on_an_axis else [nearest]
    narrow = min(fitting, key=lambda t: (t[1], abs(t[0])))
    offered = [] if abs(narrow[0]) <= min_off_deg else [narrow]
    if not on_an_axis and nearest[1] != narrow[1]:
        offered.append(nearest)
    return offered


def closes_on_narrowest_side(approach, jaw, size, rotation, max_span, min_off_deg=ALIGN_JAW_MIN_OFF_DEG,
                             max_tilt_deg=ALIGN_JAW_MAX_TILT_DEG, up=(0.0, 0.0, 1.0)):
    '''
    True when a grasp within max_tilt_deg of straight down closes across the narrowest horizontal box side that fits
    the open gripper (max_span): its jaw lies within min_off_deg of that box axis, as the turned copies of
    jaw_alignment_turns(narrowest=True) do. Equal sides count as one.
    '''
    fitting = [(turn, span) for turn, span in _jaw_side_turns(approach, jaw, size, rotation, max_tilt_deg, up)
               if span <= max_span]
    if not fitting:
        return False
    narrow = min(span for _, span in fitting)
    return any(abs(turn) <= min_off_deg for turn, span in fitting if span == narrow)


def jaw_centring_shift(jaw, tcp, centre):
    '''the move (x, y, z) of a grasp's tcp along its jaw axis that puts the jaw's midline through centre (the box
    centre), so the closing jaws meet the object around its middle; nothing changes across the jaw or along the
    approach'''
    j = _unit(jaw)
    along = sum(a * (c - t) for a, c, t in zip(j, centre, tcp))
    return tuple(along * a for a in j)
