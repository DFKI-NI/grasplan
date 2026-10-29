'''
Robotiq 2F-140 fingertip pad geometry for the gripper pre-open check and the re-grasp after over-closing. Tables
from the real robot's URDF (the finger links' pad positions sampled every 0.05 rad, linear in between, gap within
0.6 mm): pad gap 139.5 mm open at 0 rad, closed at 0.755 rad; the pad tips move forward while closing.
Pure functions, no ROS.
'''

THETA_STEP = 0.05
# pad gap (m) at theta = 0, 0.05, ... 0.75 rad
PAD_GAP_M = [0.1395, 0.1317, 0.1237, 0.1153, 0.1066, 0.0978, 0.0887, 0.0794, 0.0699, 0.0603, 0.0505, 0.0407,
             0.0308, 0.0208, 0.0108, 0.0008]
# the pad tip's x in the TCP frame (m) at the same angles
PAD_TIP_X_M = [0.0122, 0.0154, 0.0183, 0.021, 0.0235, 0.0258, 0.0279, 0.0298, 0.0314, 0.0327, 0.0339, 0.0348,
               0.0354, 0.0358, 0.0359, 0.0358]
MAX_THETA = 0.755


def _lookup(table, theta):
    theta = min(max(theta, 0.0), MAX_THETA)
    index = min(int(theta / THETA_STEP), len(table) - 2)
    fraction = min((theta - index * THETA_STEP) / THETA_STEP, 1.0)
    return table[index] + (table[index + 1] - table[index]) * fraction


def pad_gap(theta):
    '''gap (m) between the two pad faces at finger angle theta (rad), clamped to the stroke'''
    return _lookup(PAD_GAP_M, theta)


def pad_tip_x(theta):
    '''x (m) of the pad tip in the TCP frame at finger angle theta'''
    return _lookup(PAD_TIP_X_M, theta)


PRE_OPEN_GAP_TOLERANCE_M = 0.015   # the open fingers read 0.048 rad = 132 mm of the configured 140 mm (sim)


def reads_open(theta, open_width, tolerance=PRE_OPEN_GAP_TOLERANCE_M):
    '''True when the finger angle theta (rad) puts the pads within tolerance (m) of the configured open width (m), so a
    pre-open goal would change nothing; False for a missing reading (None) or a partly closed gripper'''
    if theta is None:
        return False
    return pad_gap(theta) >= min(open_width, PAD_GAP_M[0]) - tolerance


def theta_for_gap(width):
    '''finger angle (rad) at which the pad faces are width (m) apart; clamped to the stroke'''
    if width >= PAD_GAP_M[0]:
        return 0.0
    for index in range(len(PAD_GAP_M) - 1):
        high, low = PAD_GAP_M[index], PAD_GAP_M[index + 1]
        if width >= low:
            return (index + (high - width) / (high - low)) * THETA_STEP
    return MAX_THETA


def overclose(theta, width):
    '''how far (m) the pads closed past the object's width: positive = the fingers went in further than flat pads on an
    object of that width would allow (a fingertip pinch, a tilted pad), 0 or negative = at or short of it'''
    return width - pad_gap(theta)


def box_extent_along(dims, rotation, axis):
    '''extent (m) of a box (full side lengths dims along its own x, y, z; rotation = its axes as columns in the frame
    of axis) along the unit vector axis: sum of |axis . box axis i| * dims[i]'''
    return float(sum(abs(sum(rotation[r][i] * axis[r] for r in range(3))) * dims[i] for i in range(3)))


PAD_WIDTH_M = 0.027    # pad extent along the TCP z (2F-140)
PAD_LENGTH_M = 0.065   # pad extent along the TCP x


def round_cross_section(dims, tolerance=0.15):
    '''(diameter, length, axis index) when the perceived box could be a cylinder or a ball (two sides within tolerance
    of each other: a can, a bottle, an apple), None for any other box. The perception only gives the box, so a
    square-section box looks the same: the shape stays unknown (low confidence).'''
    d = [float(v) for v in dims]
    if min(d) <= 0.0:
        return None
    for axis in range(3):
        a, b = (d[i] for i in range(3) if i != axis)
        if max(a, b) <= (1.0 + tolerance) * min(a, b):
            return (a + b) / 2.0, d[axis], axis
    return None


def perceived_width(opening, shape_width, margin=0.010):
    '''(width, disagree) from the AnyGrasp jaw opening and the width of the object's shape across the pads (either may
    be None). The opening carries a margin of 10-30 mm over the object, so it is only an upper bound: the shape width is
    used when there is one (the smaller of the two when the opening is the tighter one). The inputs disagree only when
    the object is wider than the planned opening by more than margin (the perception is wrong; no re-grasp).'''
    if opening and shape_width:
        return min(opening, shape_width), shape_width > opening + margin
    return (opening or shape_width), False


def box_max_x_in_tcp(dims, box_to_tcp):
    '''largest x (m) of the box's corners in the TCP frame; box_to_tcp is the 4x4 pose of the box centre in the TCP frame
    (nested lists)'''
    best = -float('inf')
    for sx in (-0.5, 0.5):
        for sy in (-0.5, 0.5):
            for sz in (-0.5, 0.5):
                p = (sx * dims[0], sy * dims[1], sz * dims[2])
                best = max(best, sum(box_to_tcp[0][i] * p[i] for i in range(3)) + box_to_tcp[0][3])
    return best


def regrasp_max_depth(extent_beyond_tips, cap, table_room=None, backed_off=False, minimum=0.005):
    '''
    how far (m) the re-grasp may move the open gripper deeper along the approach, and why not further:
    (depth, reason) with reason 'backoff' (the table backoff moved this grasp back: deeper undoes it), 'extent' (the
    object has little left beyond the pad tips), 'table' (the fingertips would come too close to the support) or
    'cap'. depth < minimum = shallow by design, the re-grasp is skipped (reason says why).
    table_room: m the approach may go before the fingertips reach the clearance above the support, None = no limit
    '''
    if backed_off:
        return 0.0, 'backoff'
    limits = [(cap, 'cap'), (max(extent_beyond_tips, 0.0), 'extent')]
    if table_room is not None:
        limits.append((max(table_room, 0.0), 'table'))
    depth, reason = min(limits, key=lambda item: item[0])
    return (depth, reason) if depth >= minimum else (0.0, reason)
