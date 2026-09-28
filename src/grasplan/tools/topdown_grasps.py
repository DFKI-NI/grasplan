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
Geometry of the open-set fallback grasp and of the object box clamp (#148), numpy only (no ROS).

topdown_grasps(): when AnyGrasp offers no usable grasp of a small object, grasp it straight down from its box: the
TCP (x = approach, y = closing axis) above the box centre, closing across a horizontal box axis (the narrow one
first), as low as the fingertips may go above the support surface.

clamp_box_to_support(): a perceived box that reaches into the table (noisy depth) makes the object collide with its
support, which blocks e.g. the lift after the grasp; its bottom is raised to the support top.
'''

import numpy as np


def quaternion_to_matrix(q):
    '''3x3 rotation matrix of an (x, y, z, w) quaternion; an unset one (all 0) is the identity'''
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        return np.eye(3)
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quaternion(m):
    '''(x, y, z, w) quaternion of a 3x3 rotation matrix'''
    m = np.asarray(m, dtype=float)
    trace = np.trace(m)
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
        q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (m[j, i] + m[i, j]) / s
        q[k] = (m[k, i] + m[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def vertical_half_extent(orientation, size):
    '''half height (world z) of a box with this orientation (x, y, z, w) and size (x, y, z)'''
    rotation = quaternion_to_matrix(orientation)
    return 0.5 * float(np.sum(np.abs(rotation[2, :]) * np.asarray(size, dtype=float)))


def horizontal_axes(orientation, size):
    '''
    [(unit horizontal direction, extent)] of the two box axes that are closest to horizontal, narrow first; the
    extent along the projected direction also counts the other axes' share (a tilted box looks wider)
    '''
    rotation = quaternion_to_matrix(orientation)
    size = np.asarray(size, dtype=float)
    vertical = int(np.argmax(np.abs(rotation[2, :])))
    axes = []
    for i in range(3):
        if i == vertical:
            continue
        direction = rotation[:, i].copy()
        direction[2] = 0.0
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            continue
        direction /= norm
        extent = float(np.sum(np.abs(direction @ rotation) * size))
        axes.append((direction, extent))
    return sorted(axes, key=lambda axis: axis[1])


def topdown_orientation(closing_direction):
    '''(x, y, z, w) of a TCP pointing straight down (x = -world z) whose closing axis y is closing_direction'''
    x = np.array([0.0, 0.0, -1.0])
    y = np.asarray(closing_direction, dtype=float)
    y = y - (y @ x) * x
    y /= np.linalg.norm(y)
    return matrix_to_quaternion(np.column_stack((x, y, np.cross(x, y))))


def topdown_grasps(
    center, orientation, size, support_top, lowest_point_offset, min_fingertip_height, min_tcp_height=0.0,
    max_width=0.12, min_overlap=0.01,
):
    '''
    Straight-down grasps of a box (center, orientation (x, y, z, w), size) standing on a surface whose top is at
    support_top (all in one frame with z up). Per horizontal box axis that fits the gripper (extent <= max_width),
    narrow first, two grasps closing across it (the second turned 180 degrees about the approach axis, as the wrist_3
    limits allow only one of them in some arm poses). The TCP goes above the box centre, as low as the lowest
    fingertip point keeps min_fingertip_height above support_top and the TCP min_tcp_height; a grasp whose
    fingertips would then reach less than min_overlap below the box top would close on air and is skipped.
    lowest_point_offset(orientation, width): lowest fingertip z relative to the TCP while closing down to width
    (grasplan.tools.gripper_envelope). Returns ([(position, orientation, width)], [reasons for skipped axes]).
    '''
    center = np.asarray(center, dtype=float)
    top = center[2] + vertical_half_extent(orientation, size)
    grasps, skipped = [], []
    for direction, extent in horizontal_axes(orientation, size):
        if extent > max_width:
            skipped.append(f'{extent * 100:.1f} cm wide (> {max_width * 100:.1f} cm)')
            continue
        for closing in (direction, -direction):
            q = topdown_orientation(closing)
            lowest = lowest_point_offset(q, extent)
            tcp_z = max(support_top + min_fingertip_height - lowest, support_top + min_tcp_height)
            if tcp_z + lowest > top - min_overlap:
                skipped.append(
                    f'fingertips {(tcp_z + lowest - support_top) * 100:.1f} cm above the support, object top '
                    f'{(top - support_top) * 100:.1f} cm'
                )
                break
            grasps.append((np.array([center[0], center[1], tcp_z]), q, extent))
    return grasps, skipped


def clamp_box_to_support(center, orientation, size, supports, clearance=0.001):
    '''
    Raise the bottom of a box that reaches into its support to clearance above the support top, keeping its top.
    supports: [(center, orientation (x, y, z, w), size)] of the support boxes (tables); the support is one whose
    footprint holds the box centre with the centre above its top. Only a box standing on one of its faces (an axis
    within ~18 degrees of vertical) is clamped. Returns (center, size, lifted_m), unchanged with lifted_m 0 when
    nothing needs a change.
    '''
    center = np.asarray(center, dtype=float).copy()
    size = np.asarray(size, dtype=float).copy()
    rotation = quaternion_to_matrix(orientation)
    vertical = int(np.argmax(np.abs(rotation[2, :])))
    if abs(rotation[2, vertical]) < 0.95:
        return center, size, 0.0
    half = vertical_half_extent(orientation, size)
    bottom = center[2] - half
    for support_center, support_orientation, support_size in supports:
        support_center = np.asarray(support_center, dtype=float)
        local = quaternion_to_matrix(support_orientation).T @ (center - support_center)
        if np.any(np.abs(local[:2]) > np.asarray(support_size, dtype=float)[:2] / 2.0):
            continue
        top = support_center[2] + vertical_half_extent(support_orientation, support_size)
        if center[2] <= top or bottom >= top + clearance:
            continue
        lift = top + clearance - bottom
        if lift >= 2.0 * half:
            return center, size, 0.0  # nothing would be left of the box: the support top is wrong, leave it
        size[vertical] -= lift / abs(rotation[2, vertical])
        center[2] += lift / 2.0
        return center, size, float(lift)
    return center, size, 0.0


def depth_below_support_top(center, orientation, size, supports):
    '''
    how far (m) the bottom of a box reaches below the top of a support box (supports as for clamp_box_to_support) whose
    footprint holds its centre, with the centre above the middle of that support; 0 when it does not. Unlike
    clamp_box_to_support also for a box sunk so deep that nothing of it would be left above the top.
    '''
    center = np.asarray(center, dtype=float)
    bottom = center[2] - vertical_half_extent(orientation, size)
    deepest = 0.0
    for support_center, support_orientation, support_size in supports:
        support_center = np.asarray(support_center, dtype=float)
        local = quaternion_to_matrix(support_orientation).T @ (center - support_center)
        if np.any(np.abs(local[:2]) > np.asarray(support_size, dtype=float)[:2] / 2.0) or center[2] < support_center[2]:
            continue
        deepest = max(deepest, support_center[2] + vertical_half_extent(support_orientation, support_size) - bottom)
    return deepest


def _footprint_prism(center, orientation, size):
    '''(convex hull of the box corners in xy, counter-clockwise; min z; max z)'''
    rotation = quaternion_to_matrix(orientation)
    half = np.asarray(size, dtype=float) / 2.0
    corners = [np.asarray(center, dtype=float) + rotation @ (half * np.array([sx, sy, sz]))
               for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    points = sorted({(round(float(c[0]), 9), round(float(c[1]), 9)) for c in corners})

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    hull = []
    for sequence in (points, points[::-1]):   # Andrew's monotone chain: lower then upper hull
        part = []
        for point in sequence:
            while len(part) >= 2 and cross(part[-2], part[-1], point) <= 0.0:
                part.pop()
            part.append(point)
        hull += part[:-1]
    zs = [float(c[2]) for c in corners]
    return hull, min(zs), max(zs)


def _polygon_area(polygon):
    return 0.5 * abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(polygon, polygon[1:] + polygon[:1])))


def _clip(subject, clip):
    '''Sutherland-Hodgman: subject polygon clipped by a convex counter-clockwise clip polygon'''
    output = list(subject)
    for a, b in zip(clip, clip[1:] + clip[:1]):
        if not output:
            break

        def inside(p):
            return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0.0

        def meet(p, q):
            x1, y1, x2, y2 = a[0], a[1], b[0], b[1]
            x3, y3, x4, y4 = p[0], p[1], q[0], q[1]
            d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
            t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / d
            return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))

        points, output = output, []
        for p, q in zip(points, points[1:] + points[:1]):
            if inside(q):
                if not inside(p):
                    output.append(meet(p, q))
                output.append(q)
            elif inside(p):
                output.append(meet(p, q))
    return output


def box_overlap_fraction(center_a, orientation_a, size_a, center_b, orientation_b, size_b):
    '''
    shared volume of two boxes as a fraction of the smaller one, each taken as the vertical prism over its footprint
    (exact for boxes standing on a face, e.g. yawed perception boxes); 0 when they do not overlap
    '''
    hull_a, a0, a1 = _footprint_prism(center_a, orientation_a, size_a)
    hull_b, b0, b1 = _footprint_prism(center_b, orientation_b, size_b)
    dz = min(a1, b1) - max(b0, a0)
    smaller = min(_polygon_area(hull_a) * (a1 - a0), _polygon_area(hull_b) * (b1 - b0))
    if dz <= 0.0 or smaller <= 0.0 or len(hull_a) < 3 or len(hull_b) < 3:
        return 0.0
    shared = _clip(hull_a, hull_b)
    return min(1.0, _polygon_area(shared) * dz / smaller) if len(shared) >= 3 else 0.0
