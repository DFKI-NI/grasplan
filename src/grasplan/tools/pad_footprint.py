'''
Width of an object across the gripper pads: the extent along the TCP y axis (the pads close along y) of the part of
the object that lies inside the pad footprint (TCP x between the pad tip minus the pad length and the pad tip, |z| within
half the pad width), computed geometrically for any pose of the object in the TCP frame. The shape is the perceived oriented box
(no per-object knowledge of open-set objects, only what the robot perceives); the extent is two small linear programs. Pure
functions (numpy, scipy), no ROS.
'''
import logging
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
