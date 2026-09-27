'''unit tests of grasplan.tools.topdown_grasps (#148, numpy only, no ROS): python3 -m pytest <this file>'''
import math

import numpy as np

from grasplan.tools.topdown_grasps import (
    clamp_box_to_support,
    horizontal_axes,
    quaternion_to_matrix,
    topdown_grasps,
    topdown_orientation,
)

UPRIGHT = (0.0, 0.0, 0.0, 1.0)
TABLE_TOP = 0.72
TABLE = ((0.0, 0.0, 0.36), UPRIGHT, (0.8, 0.8, 0.72))


def yawed(angle):
    return (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))


def pad_tip(orientation, width):
    return -0.02  # fingertip pads end 2 cm below the TCP when pointing straight down


def strawberry(z_center=0.751):
    # the real strawberry_1 box of 2026-09-27: 5.6 x 7.8 x 6.2 cm, yawed 30 degrees
    return (0.0, 0.0, z_center), yawed(math.radians(30)), (0.056, 0.078, 0.062)


def test_topdown_orientation_points_down_and_closes_along_the_direction():
    direction = np.array([math.cos(0.4), math.sin(0.4), 0.0])
    rotation = quaternion_to_matrix(topdown_orientation(direction))
    assert np.allclose(rotation[:, 0], [0.0, 0.0, -1.0])
    assert np.allclose(rotation[:, 1], direction)
    assert math.isclose(np.linalg.det(rotation), 1.0)


def test_horizontal_axes_narrow_first():
    center, orientation, size = strawberry()
    (first, narrow), (second, wide) = horizontal_axes(orientation, size)
    assert math.isclose(narrow, 0.056) and math.isclose(wide, 0.078)
    assert np.allclose(first, [math.cos(math.radians(30)), math.sin(math.radians(30)), 0.0])
    assert math.isclose(abs(first @ second), 0.0, abs_tol=1e-9)


def test_strawberry_gets_four_grasps_narrow_axis_first_with_fingertips_above_the_table():
    center, orientation, size = strawberry()
    grasps, skipped = topdown_grasps(center, orientation, size, TABLE_TOP, pad_tip, min_fingertip_height=0.015,
                                     min_tcp_height=0.035)
    assert not skipped
    assert [round(width, 3) for _, _, width in grasps] == [0.056, 0.056, 0.078, 0.078]
    for position, q, _ in grasps:
        assert np.allclose(position[:2], center[:2])
        assert position[2] + pad_tip(q, 0) >= TABLE_TOP + 0.015 - 1e-9   # fingertips >= 1.5 cm above the table
        assert position[2] >= TABLE_TOP + 0.035 - 1e-9                    # TCP limit
        assert np.allclose(quaternion_to_matrix(q)[:, 0], [0.0, 0.0, -1.0])
    first, twin = (quaternion_to_matrix(q)[:, 1] for _, q, _ in grasps[:2])
    assert np.allclose(first, -twin)                                     # 180 degrees about the approach


def test_too_wide_axis_is_skipped():
    grasps, skipped = topdown_grasps((0, 0, 0.8), UPRIGHT, (0.05, 0.2, 0.1), TABLE_TOP, pad_tip, 0.015, max_width=0.12)
    assert [round(width, 3) for _, _, width in grasps] == [0.05, 0.05]
    assert len(skipped) == 1 and '20.0 cm wide' in skipped[0]


def test_flat_object_would_close_on_air():
    # 1.5 cm tall: the fingertips must stay 1.5 cm above the table, 1 cm overlap impossible
    grasps, skipped = topdown_grasps((0, 0, TABLE_TOP + 0.0075), UPRIGHT, (0.04, 0.04, 0.015), TABLE_TOP, pad_tip,
                                     0.015)
    assert grasps == [] and len(skipped) == 2 and 'object top 1.5 cm' in skipped[0]


def test_clamp_raises_a_box_reaching_into_the_table():
    center, orientation, size = strawberry(z_center=0.74)          # bottom 0.709 < table top 0.72
    new_center, new_size, lifted = clamp_box_to_support(center, orientation, size, [TABLE])
    assert math.isclose(lifted, 0.72 + 0.001 - 0.709, abs_tol=1e-9)
    assert math.isclose(new_center[2] - new_size[2] / 2, 0.721, abs_tol=1e-9)   # bottom on the table
    assert math.isclose(new_center[2] + new_size[2] / 2, 0.771, abs_tol=1e-9)   # top kept
    assert np.allclose(new_size[:2], size[:2])


def test_clamp_leaves_boxes_alone_that_are_clear_off_or_not_above_the_table():
    center, orientation, size = strawberry(z_center=0.76)          # bottom 0.729: clear
    assert clamp_box_to_support(center, orientation, size, [TABLE])[2] == 0.0
    beside = ((1.0, 0.0, 0.74), orientation, size)                  # not over the table
    assert clamp_box_to_support(*beside, [TABLE])[2] == 0.0
    sunk = ((0.0, 0.0, 0.70), UPRIGHT, (0.05, 0.05, 0.08))          # centre below the table top
    assert clamp_box_to_support(*sunk, [TABLE])[2] == 0.0


def test_clamp_uses_the_rotated_table_footprint():
    table = ((0.0, 0.0, 0.36), yawed(math.radians(45)), (1.0, 0.2, 0.72))
    on_it = ((0.3, 0.3, 0.74), UPRIGHT, (0.04, 0.04, 0.06))         # along the rotated long side
    off_it = ((0.3, -0.3, 0.74), UPRIGHT, (0.04, 0.04, 0.06))
    assert clamp_box_to_support(*on_it, [table])[2] > 0.0
    assert clamp_box_to_support(*off_it, [table])[2] == 0.0


def test_unset_quaternion_is_identity():
    assert np.allclose(quaternion_to_matrix((0.0, 0.0, 0.0, 0.0)), np.eye(3))
    table = ((0.0, 0.0, 0.36), (0.0, 0.0, 0.0, 0.0), (0.8, 0.8, 0.72))
    assert clamp_box_to_support((0.0, 0.0, 0.74), UPRIGHT, (0.04, 0.04, 0.06), [table])[2] > 0.0
