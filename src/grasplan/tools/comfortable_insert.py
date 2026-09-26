#!/usr/bin/env python3
'''
Insert poses for small objects that do not have to keep the orientation they were grasped in.

A place pose normally fixes the object's orientation, so MoveIt reproduces the grasp orientation over the
container: an oblique AnyGrasp grasp then needs a large wrist rotation that winds the arm cable up. For small
objects (a ball, a fruit) the orientation does not matter, so here the gripper orientation is chosen instead:
the approach axis (+x of the TCP) points straight down, like the multimeter insert, and the rotation about it
is taken as close as possible to the current one, which keeps the wrist (and its cable) where it is.

All transforms are 4x4 numpy matrices; the ROS glue lives in insert.py.
'''

import math

import numpy as np
import tf.transformations as tft


def pose_to_matrix(pose):
    '''geometry_msgs/Pose -> 4x4; a zero quaternion (unset) counts as identity'''
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    if not any(q):
        q = [0.0, 0.0, 0.0, 1.0]
    m = tft.quaternion_matrix(q)
    m[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return m


def matrix_to_pose(m, pose):
    '''write a 4x4 into a geometry_msgs/Pose and return it'''
    q = tft.quaternion_from_matrix(m)
    pose.position.x, pose.position.y, pose.position.z = m[:3, 3]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


def half_extent_along(rotation, primitive_type, dims, axis):
    '''
    half the extent of a collision primitive along a world axis (unit 3-vector), given the primitive's
    rotation in the world; primitive_type: 'box' (dims x, y, z), 'cylinder' (dims height, radius; axis z),
    'sphere' (dims radius)
    '''
    local = rotation.T.dot(axis)
    if primitive_type == 'box':
        return sum(abs(local[i]) * dims[i] / 2.0 for i in range(3))
    if primitive_type == 'cylinder':
        height, radius = dims[0], dims[1]
        return abs(local[2]) * height / 2.0 + math.sqrt(max(0.0, 1.0 - local[2] ** 2)) * radius
    return dims[0]  # sphere


def long_axis(primitive_type, dims):
    '''unit vector of the primitive's longest axis in its own frame, None when it has none (ball, cube)'''
    if primitive_type == 'box':
        order = sorted(range(3), key=lambda i: dims[i])
        if dims[order[2]] < 1.3 * dims[order[1]]:
            return None
        axis = np.zeros(3)
        axis[order[2]] = 1.0
        return axis
    if primitive_type == 'cylinder' and dims[0] > 2.6 * dims[1]:
        return np.array([0.0, 0.0, 1.0])
    return None


def top_down_rotation(theta):
    '''TCP rotation with the approach axis (+x) pointing down and the TCP y axis at angle theta in the xy plane'''
    x_axis = np.array([0.0, 0.0, -1.0])
    y_axis = np.array([math.cos(theta), math.sin(theta), 0.0])
    z_axis = np.cross(x_axis, y_axis)
    rotation = np.eye(3)
    rotation[:, 0], rotation[:, 1], rotation[:, 2] = x_axis, y_axis, z_axis
    return rotation


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def comfortable_insert_candidates(
    current_tcp_rotation,
    tcp_to_body,
    body_to_primitive,
    primitive_type,
    dims,
    target_xy,
    support_top_z,
    support_long_axis_yaw=None,
    gap=0.02,
    yaw_step=math.radians(15.0),
    max_yaw_offset=math.radians(180.0),
    long_axis_tolerance=math.radians(25.0),
):
    '''
    Object body poses (4x4, world frame) to hand to MoveIt's place, best first.

    current_tcp_rotation: 3x3 rotation of the TCP in the world now (after the cable untangle poses)
    tcp_to_body: 4x4 pose of the attached object's body frame in the TCP frame
    body_to_primitive: 4x4 pose of its collision primitive in the body frame
    target_xy: world xy where the primitive centre is released (the container centre)
    support_top_z: world z of the container top; the primitive's lowest point is released gap above it
    support_long_axis_yaw: yaw of the container's long side; an elongated object is only released along it

    Candidates keep the approach axis straight down and differ in the rotation about it, sorted by how little
    it differs from the current one (the least wrist rotation, so the cable stays as it is).
    '''
    current_y = current_tcp_rotation[:, 1]
    current_theta = math.atan2(current_y[1], current_y[0])
    tcp_to_primitive = tcp_to_body.dot(body_to_primitive)
    object_long_axis = long_axis(primitive_type, dims)
    candidates = []
    steps = int(round(2 * math.pi / yaw_step))
    for step in range(steps):
        offset = wrap(step * yaw_step)
        if abs(offset) > max_yaw_offset + 1e-9:
            continue
        rotation = top_down_rotation(current_theta + offset)
        primitive_rotation = rotation.dot(tcp_to_primitive[:3, :3])
        if object_long_axis is not None and support_long_axis_yaw is not None:
            world_axis = primitive_rotation.dot(object_long_axis)
            if math.hypot(world_axis[0], world_axis[1]) > 0.5:  # lying: its long side must follow the box
                axis_yaw = math.atan2(world_axis[1], world_axis[0])
                misalignment = abs(wrap(2.0 * (axis_yaw - support_long_axis_yaw))) / 2.0  # direction-free
                if misalignment > long_axis_tolerance:
                    continue
        z_half = half_extent_along(primitive_rotation, primitive_type, dims, np.array([0.0, 0.0, 1.0]))
        primitive_centre = np.array([target_xy[0], target_xy[1], support_top_z + gap + z_half])
        tcp = np.eye(4)
        tcp[:3, :3] = rotation
        tcp[:3, 3] = primitive_centre - rotation.dot(tcp_to_primitive[:3, 3])
        candidates.append((abs(offset), tcp.dot(tcp_to_body)))
    candidates.sort(key=lambda c: c[0])
    return [body for _, body in candidates]
