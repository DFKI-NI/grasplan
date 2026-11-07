'''
Width of an object across the gripper pads: the extent along the TCP y axis (the pads close along y) of the part of
the object that lies inside the pad footprint (TCP x between the pad tip minus the pad length and the pad tip, |z| within
half the pad width), computed geometrically for any pose of the object in the TCP frame. The shape is the perceived oriented box
(no per-object knowledge of open-set objects, only what the robot perceives); the extent is two small linear programs. Pure
functions (numpy, scipy), no ROS.
'''
import logging
import math
import numpy as np

from grasplan.tools.pad_gap import PAD_LENGTH_M, PAD_WIDTH_M

LOG = logging.getLogger(__name__)


def box_vertices(dims):
    '''the 8 corners of a box centred at the origin (full side lengths dims)'''
    return np.array([[sx * dims[0] / 2.0, sy * dims[1] / 2.0, sz * dims[2] / 2.0]
                     for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def footprint_width(vertices, object_in_tcp, x_tip, pad_length=PAD_LENGTH_M, pad_width=PAD_WIDTH_M):
    '''
    extent (m) along the TCP y of the convex hull of vertices (object frame, centred at the origin) inside the pad
    footprint x in [x_tip - pad_length, x_tip], |z| <= pad_width / 2, with object_in_tcp the 4x4 pose of the object in the
    TCP frame (translation included). None when the object does not reach the footprint.
    '''
    from scipy.optimize import linprog
    pose = np.asarray(object_in_tcp, dtype=float)
    pts = np.asarray(vertices, dtype=float).dot(pose[:3, :3].T) + pose[:3, 3]   # n x 3 in the TCP frame
    n = len(pts)
    a_ub = np.vstack([pts[:, 0], -pts[:, 0], pts[:, 2], -pts[:, 2]])
    b_ub = np.array([x_tip, -(x_tip - pad_length), pad_width / 2.0, pad_width / 2.0])
    a_eq, b_eq = np.ones((1, n)), np.array([1.0])
    ys = []
    for sign in (-1.0, 1.0):   # minimise -y (the largest y), then y (the smallest)
        res = linprog(sign * pts[:, 1], A_ub=a_ub, b_ub=b_ub, A_eq=a_eq, b_eq=b_eq, bounds=(0, None), method='highs')
        if res.status != 0:
            return None
        ys.append(res.fun)
    return float(-ys[0] - ys[1])


def width_across_pads(dims, object_in_tcp, opening=None):
    '''
    (width m, source, confidence) of an object with the perceived box dims across the pads: the footprint extent of the
    box at object_in_tcp, the pad tip x taken from the pad table at the width, two passes (the first at the opening or
    the box's y extent). Confidence 'low' when two sides are about equal (the box could hide a round object, whose width
    across the pads is smaller than the box's), else 'medium'. When no part of the box lies inside the footprint: the
    box's extent along the finger axis, confidence 'low', source 'box-extent'.
    '''
    from grasplan.tools.pad_gap import box_extent_along, pad_tip_x, theta_for_gap
    pose = np.asarray(object_in_tcp, dtype=float)
    from grasplan.tools.pad_gap import round_cross_section
    vertices = box_vertices(dims)
    source, confidence = 'box-footprint', ('low' if round_cross_section(dims) else 'medium')
    full = box_extent_along(dims, pose[:3, :3].tolist(), [0.0, 1.0, 0.0])
    width = opening or full
    for _ in range(2):
        x_tip = pad_tip_x(theta_for_gap(width))
        try:
            found = footprint_width(vertices, pose, x_tip)
        except Exception as error:   # no scipy on the robot image, a solver failure: never break the pick
            LOG.warning('footprint width failed (%s): using the box extent, confidence low', error)
            return full, 'box-extent', 'low'
        if found is None:
            return full, 'box-extent', 'low'
        width = found
    return width, source, confidence


PINCH_SIDE_MAX_OFF_DEG = 20.0   # a closing axis this close to a box side turns the free box flat between the pads
MIN_PINCH_SIDE_M = 0.005


def pinch_width(dims, object_in_tcp, opening=None, padding=0.0, side_max_off_deg=PINCH_SIDE_MAX_OFF_DEG):
    '''
    (m) the object's width a finger gap is judged against after a close that held, for the rim pinch check. Not the footprint
    width as it stands: the perceived box still carries the detector's padding (``padding`` per side, what is left of it), and
    the footprint is the width at the pose before the close, while the closing pads turn a free box flat between them. A box
    9 deg off its 45 mm side, perceived 53 mm with a padding of 5 mm: footprint 60 mm, the pads end at about 40 mm and that is
    a good grasp. So the footprint of the box without the padding, and at most the box side nearest the closing axis when the
    axis is within ``side_max_off_deg`` of it. Further off (a jaw across the diagonal) the footprint stays: there the pads
    really close on two edges.
    '''
    sides = [max(float(side) - 2.0 * float(padding), MIN_PINCH_SIDE_M) for side in dims]
    width = width_across_pads(sides, object_in_tcp, opening)[0]
    closing_along_sides = np.abs(np.asarray(object_in_tcp, dtype=float)[1, :3])   # the TCP y on the box axes
    nearest = int(np.argmax(closing_along_sides))
    if math.degrees(math.acos(min(1.0, float(closing_along_sides[nearest])))) <= side_max_off_deg:
        width = min(width, sides[nearest])
    return width
