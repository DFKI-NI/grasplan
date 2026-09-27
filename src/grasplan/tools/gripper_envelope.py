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
Where the fingertips of a gripper can be, seen from its TCP, over the whole closing motion.

A grasp pose places the TCP; on a tilted grasp the lower fingertip reaches below it, and on linkage grippers such
as the Robotiq 2F-140 the tips also move along the approach axis while closing. fingertip_envelope() samples the
actuated joint from its lower to its upper limit (mimic joints follow) and returns, per sample, the corners of the
collision boxes of the given links in the TCP frame and the gap between the pads; lowest_point_offset() gives the
lowest point a grasp reaches while closing down to the object width. Takes a urdf_parser_py URDF, no ROS calls.
'''

import numpy as np
import tf.transformations as tft


def _joint_transform(joint, q):
    origin = joint.origin
    xyz = origin.xyz if origin is not None and origin.xyz is not None else [0.0, 0.0, 0.0]
    rpy = origin.rpy if origin is not None and origin.rpy is not None else [0.0, 0.0, 0.0]
    transform = tft.euler_matrix(*rpy)
    transform[:3, 3] = xyz
    axis = joint.axis if joint.axis is not None else [1.0, 0.0, 0.0]
    if joint.type in ('revolute', 'continuous'):
        transform = transform @ tft.rotation_matrix(q, axis)
    elif joint.type == 'prismatic':
        transform = transform @ tft.translation_matrix(np.array(axis) * q)
    return transform


def _link_in_root(robot, link, positions):
    '''transform root <- link, following the parent joints'''
    parent_joint = {joint.child: joint for joint in robot.joints}
    transform = np.identity(4)
    while link in parent_joint:
        joint = parent_joint[link]
        transform = _joint_transform(joint, positions.get(joint.name, 0.0)) @ transform
        link = joint.parent
    return transform


def _box_corners(link):
    corners = []
    for collision in link.collisions if link.collisions else []:
        size = getattr(collision.geometry, 'size', None)
        if size is None:
            continue  # only boxes (fingertip pads); meshes are not sampled
        origin = collision.origin
        box = tft.euler_matrix(*(origin.rpy if origin is not None and origin.rpy is not None else [0.0, 0.0, 0.0]))
        box[:3, 3] = origin.xyz if origin is not None and origin.xyz is not None else [0.0, 0.0, 0.0]
        for sx in (-0.5, 0.5):
            for sy in (-0.5, 0.5):
                for sz in (-0.5, 0.5):
                    corners.append(box @ np.array([sx * size[0], sy * size[1], sz * size[2], 1.0]))
    return corners


def fingertip_envelope(robot, tcp_link, tip_links, actuated_joint, samples=12):
    '''
    [(gap, points)] for actuated_joint at samples values from its lower to its upper limit (mimic joints follow,
    other joints at 0): points is an Nx3 array of the corners of the collision boxes of tip_links in the tcp_link
    frame, gap the distance between the pads along the TCP y (closing) axis. robot: urdf_parser_py URDF.
    '''
    joints = {joint.name: joint for joint in robot.joints}
    links = {link.name: link for link in robot.links}
    actuated = joints[actuated_joint]
    envelope = []
    for q in np.linspace(actuated.limit.lower, actuated.limit.upper, samples):
        positions = {actuated_joint: q}
        for joint in robot.joints:
            if joint.mimic is not None and joint.mimic.joint == actuated_joint:
                positions[joint.name] = (joint.mimic.multiplier or 1.0) * q + (joint.mimic.offset or 0.0)
        tcp_from_root = np.linalg.inv(_link_in_root(robot, tcp_link, positions))
        points = []
        for tip in tip_links:
            tcp_from_tip = tcp_from_root @ _link_in_root(robot, tip, positions)
            points.extend((tcp_from_tip @ corner)[:3] for corner in _box_corners(links[tip]))
        points = np.array(points)
        envelope.append((2.0 * float(np.min(np.abs(points[:, 1]))), points))
    return envelope


def lowest_point_offset(envelope, orientation, width=0.0):
    '''
    lowest z (relative to the TCP) the fingertips reach when a grasp with this orientation (x, y, z, w quaternion)
    closes from fully open until the pads are width apart (0 = fully closed): all samples with a gap of at least
    width, plus the next closer one, as the pads stop in between
    '''
    rotation = tft.quaternion_matrix(orientation)[:3, :3]
    lowest = float('inf')
    for gap, points in envelope:
        lowest = min(lowest, float(np.min((points @ rotation.T)[:, 2])))
        if gap < width:
            break
    return lowest
