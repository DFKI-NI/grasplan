"""Oriented box proximity for narrowly scoped MTC contact allowances."""

from itertools import product

import numpy as np


def boxes_touch(pose_a, dimensions_a, pose_b, dimensions_b, margin=0.0):
    """Whether two oriented boxes touch or overlap within ``margin`` metres.

    Poses are 4x4 transforms in the same frame. Testing each face normal and
    pair of edge cross products handles rotated boxes without widening their
    axis-aligned bounds.
    """
    def corners(pose, dimensions):
        local = np.asarray(list(product((-0.5, 0.5), repeat=3))) * np.asarray(dimensions)
        return local @ pose[:3, :3].T + pose[:3, 3]

    a = corners(pose_a, dimensions_a)
    b = corners(pose_b, dimensions_b)
    axes_a = pose_a[:3, :3].T
    axes_b = pose_b[:3, :3].T
    axes = list(axes_a) + list(axes_b)
    axes.extend(np.cross(u, v) for u in axes_a for v in axes_b)
    for axis in axes:
        length = np.linalg.norm(axis)
        if length < 1e-9:
            continue
        direction = axis / length
        projection_a = a @ direction
        projection_b = b @ direction
        if projection_a.max() + margin < projection_b.min() or projection_b.max() + margin < projection_a.min():
            return False
    return True
