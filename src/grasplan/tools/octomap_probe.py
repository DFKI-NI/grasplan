#!/usr/bin/env python3
'''
Occupied voxels of a planning scene octomap (octomap_msgs/Octomap in the full "OcTree" format, which is what
move_group's get_planning_scene returns), and the ones inside an oriented box, so a place location whose footprint
holds something that only the octomap sees (an open-set object on the table, a missed DOPE object) can be rejected
before it is planned (#180 follow-up, real 2026-09-28: a klt placed into objects on table_1 and table_3).
'''
import struct

import numpy as np

TREE_DEPTH = 16


def _as_bytes(data):
    '''octomap_msgs data is int8[]: rospy gives a list of ints in -128..127 (or bytes)'''
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    return bytes(int(b) & 0xFF for b in data)


def occupied_leaves(data, resolution, threshold=0.0, depth=TREE_DEPTH):
    '''
    (N, 4) array of (x, y, z, size) of the occupied leaves (log-odds > threshold) of a full OcTree serialisation, in
    the octomap frame: depth first from the root, per node a float32 log-odds and one byte with bit i set when child i
    exists; child i lies on the +x side when bit 0 of i is set, +y bit 1, +z bit 2; the root cube is centred at 0.
    '''
    buf = _as_bytes(data)
    out = []
    pos = 0

    def node(cx, cy, cz, size):
        nonlocal pos
        value = struct.unpack_from('<f', buf, pos)[0]
        children = buf[pos + 4]
        pos += 5
        if not children:
            if value > threshold:
                out.append((cx, cy, cz, size))
            return
        offset = size / 4.0   # centre of a child: half a child size from the parent centre
        for i in range(8):
            if children & (1 << i):
                node(cx + (offset if i & 1 else -offset), cy + (offset if i & 2 else -offset),
                     cz + (offset if i & 4 else -offset), size / 2.0)

    if buf:
        node(0.0, 0.0, 0.0, resolution * (1 << depth))
    return np.array(out, dtype=float).reshape(-1, 4)


def count_in_box(leaves, box_to_frame, half_extents):
    '''number of leaves whose centre lies inside the box (4x4 box pose in the leaves' frame, half extents)'''
    if len(leaves) == 0:
        return 0
    inverse = np.linalg.inv(np.asarray(box_to_frame, dtype=float))
    points = np.c_[leaves[:, :3], np.ones(len(leaves))] @ inverse.T
    inside = np.all(np.abs(points[:, :3]) <= np.asarray(half_extents, dtype=float), axis=1)
    return int(np.count_nonzero(inside))


def encode_full_octree(keys, resolution, depth=TREE_DEPTH, value=2.0):
    '''test helper: the full OcTree serialisation of occupied max-depth leaves given as integer keys (kx, ky, kz),
    key k covering [(k - 2^(depth-1)) * resolution, (k - 2^(depth-1) + 1) * resolution)'''
    tree = {}
    for key in keys:
        level = tree
        for d in range(depth):
            bit = depth - 1 - d
            index = ((key[0] >> bit) & 1) | (((key[1] >> bit) & 1) << 1) | (((key[2] >> bit) & 1) << 2)
            level = level.setdefault(index, {})
    out = bytearray()

    def write(level):
        out.extend(struct.pack('<f', value))
        children = 0
        for index in level:
            children |= 1 << index
        out.append(children)
        for index in range(8):
            if index in level:
                write(level[index])

    write(tree)
    return bytes(out)


def key_of(x, y, z, resolution, depth=TREE_DEPTH):
    '''integer octomap key of a point'''
    half = 1 << (depth - 1)
    return tuple(int(np.floor(v / resolution)) + half for v in (x, y, z))
