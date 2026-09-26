#!/usr/bin/env python3
'''unit tests of grasplan.tools.comfortable_insert (no ROS master needed): python3 -m pytest <this file>'''
import math

import numpy as np
import tf.transformations as tft

from grasplan.tools.comfortable_insert import comfortable_insert_candidates, top_down_rotation


def oblique_grasp():
    '''object held 5 cm in front of the TCP, tilted 40 degrees, like an AnyGrasp side grasp of a ball'''
    tcp_to_body = tft.euler_matrix(0.3, 0.7, -0.4)
    tcp_to_body[:3, 3] = [0.05, 0.0, 0.01]
    return tcp_to_body


def test_primitive_released_above_target_with_gripper_down():
    current = tft.euler_matrix(0.2, 0.5, 1.0)[:3, :3]
    tcp_to_body = oblique_grasp()
    bodies = comfortable_insert_candidates(
        current, tcp_to_body, np.eye(4), 'cylinder', [0.067, 0.0335], (21.2, 14.0), 0.85, max_yaw_offset=math.pi
    )
    assert len(bodies) == 24
    for body in bodies:
        tcp = body.dot(np.linalg.inv(tcp_to_body))
        assert np.allclose(tcp[:3, 0], [0, 0, -1], atol=1e-9)  # approach axis straight down
        assert np.allclose(body[:2, 3], [21.2, 14.0], atol=1e-9)  # primitive centre over the container
        assert body[2, 3] > 0.85 + 0.02  # released above the rim


def test_sorted_by_wrist_rotation_and_limited():
    current = top_down_rotation(0.3)
    bodies = comfortable_insert_candidates(
        current, oblique_grasp(), np.eye(4), 'sphere', [0.033], (0, 0), 0.8, max_yaw_offset=math.radians(90)
    )
    assert len(bodies) == 13  # -90..90 in 15 degree steps
    first_tcp = bodies[0].dot(np.linalg.inv(oblique_grasp()))
    assert np.allclose(first_tcp[:3, :3], current, atol=1e-9)  # no wrist rotation at all first


def test_elongated_object_follows_the_container():
    body_to_primitive = np.eye(4)
    bodies = comfortable_insert_candidates(
        top_down_rotation(0.0), np.eye(4), body_to_primitive, 'box', [0.03, 0.03, 0.14], (0, 0), 0.8,
        support_long_axis_yaw=0.0, max_yaw_offset=math.pi,
    )
    # the box's long z axis equals the TCP z axis, horizontal: it must lie along world x (yaw 0 or 180)
    for body in bodies:
        axis = body[:3, 2]
        assert abs(axis[1]) < math.sin(math.radians(25.0)) + 1e-9
    assert 0 < len(bodies) < 24
