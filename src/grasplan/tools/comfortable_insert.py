#!/usr/bin/env python3
'''
Insert poses that do not reproduce the full orientation the object was grasped in.

A place pose normally fixes the object's orientation, so MoveIt reproduces the grasp orientation over the
container: an oblique AnyGrasp grasp then needs a large wrist rotation that winds the arm cable up. Two ways
out, both sorted by the least rotation from the current TCP (which keeps the wrist and its cable where it is):

- free_yaw_insert_candidates: the object keeps the roll and pitch it had when it was picked (how it stood on
  the table), only its yaw about the vertical is free (Oscar's decision in #106)
- comfortable_insert_candidates: the gripper orientation is chosen instead, the approach axis (+x of the TCP)
  points straight down like the multimeter insert; changes the object's roll and pitch after an oblique grasp

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


def body_candidate(
    tcp_rotation, tcp_to_body, tcp_to_primitive, primitive_type, dims, object_long_axis, target_xy, support_top_z,
    support_long_axis_yaw, gap, long_axis_tolerance,
):
    '''
    object body pose (4x4) for a TCP rotation, with the primitive centre over target_xy and its lowest point gap
    above support_top_z; None when an elongated object would lie across the container's long side
    '''
    primitive_rotation = tcp_rotation.dot(tcp_to_primitive[:3, :3])
    if object_long_axis is not None and support_long_axis_yaw is not None:
        world_axis = primitive_rotation.dot(object_long_axis)
        if math.hypot(world_axis[0], world_axis[1]) > 0.5:  # lying: its long side must follow the box
            axis_yaw = math.atan2(world_axis[1], world_axis[0])
            misalignment = abs(wrap(2.0 * (axis_yaw - support_long_axis_yaw))) / 2.0  # direction-free
            if misalignment > long_axis_tolerance:
                return None
    z_half = half_extent_along(primitive_rotation, primitive_type, dims, np.array([0.0, 0.0, 1.0]))
    primitive_centre = np.array([target_xy[0], target_xy[1], support_top_z + gap + z_half])
    tcp = np.eye(4)
    tcp[:3, :3] = tcp_rotation
    tcp[:3, 3] = primitive_centre - tcp_rotation.dot(tcp_to_primitive[:3, 3])
    return tcp.dot(tcp_to_body)


def rotation_angle(r_a, r_b):
    '''angle (rad) of the rotation between two 3x3 rotations'''
    return math.acos(max(-1.0, min(1.0, (np.trace(r_a.T.dot(r_b)) - 1.0) / 2.0)))


def free_yaw_insert_candidates(
    current_tcp_rotation,
    tcp_to_body,
    body_to_primitive,
    picked_body_rotation,
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
    Object body poses (4x4, world frame) to hand to MoveIt's place, best first: the body keeps the roll and pitch
    of picked_body_rotation (3x3, its world rotation when it was picked; only its tilt matters, the yaw is
    ignored) and is turned about the world vertical in yaw_step steps.

    The yaw that needs the least TCP rotation from current_tcp_rotation comes first, then the others sorted by
    that rotation angle; max_yaw_offset limits the yaws to that many rad from the best one. The other arguments
    are those of comfortable_insert_candidates.
    '''
    tcp_to_primitive = tcp_to_body.dot(body_to_primitive)
    object_long_axis = long_axis(primitive_type, dims)
    # TCP rotation for yaw psi: Rz(psi) * tcp_rotation_0; the psi that maximises trace(current^T * that) is the
    # least rotation from the current TCP (closed form of the trace of Rz(psi) * m)
    tcp_rotation_0 = picked_body_rotation.dot(tcp_to_body[:3, :3].T)
    m = tcp_rotation_0.dot(current_tcp_rotation.T)
    best_yaw = math.atan2(m[0, 1] - m[1, 0], m[0, 0] + m[1, 1])
    candidates = []
    steps = int(round(2 * math.pi / yaw_step))
    for step in range(steps):
        offset = wrap(step * yaw_step)
        if abs(offset) > max_yaw_offset + 1e-9:
            continue
        cos_yaw, sin_yaw = math.cos(best_yaw + offset), math.sin(best_yaw + offset)
        yaw_rotation = np.array([[cos_yaw, -sin_yaw, 0.0], [sin_yaw, cos_yaw, 0.0], [0.0, 0.0, 1.0]])
        rotation = yaw_rotation.dot(tcp_rotation_0)
        body = body_candidate(
            rotation, tcp_to_body, tcp_to_primitive, primitive_type, dims, object_long_axis, target_xy,
            support_top_z, support_long_axis_yaw, gap, long_axis_tolerance,
        )
        if body is not None:
            candidates.append((rotation_angle(current_tcp_rotation, rotation), body))
    candidates.sort(key=lambda c: c[0])
    return [body for _, body in candidates]


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
        body = body_candidate(
            top_down_rotation(current_theta + offset), tcp_to_body, tcp_to_primitive, primitive_type, dims,
            object_long_axis, target_xy, support_top_z, support_long_axis_yaw, gap, long_axis_tolerance,
        )
        if body is not None:
            candidates.append((abs(offset), body))
    candidates.sort(key=lambda c: c[0])
    return [body for _, body in candidates]
