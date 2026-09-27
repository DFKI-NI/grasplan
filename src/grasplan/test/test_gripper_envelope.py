'''unit tests of grasplan.tools.gripper_envelope and the straight bonus of grasplan.grasp_scoring (#134, no ROS master
needed): python3 -m pytest <this file>'''
import math

import numpy as np
import tf.transformations as tft
from urdf_parser_py.urdf import URDF

from grasplan.grasp_scoring import straight_bonus, top_down_multiplier
from grasplan.tools.gripper_envelope import fingertip_envelope, lowest_point_offset

# parallel gripper: TCP 0.2 m in front of the base (TCP x = base z, the approach axis), two 4 cm long, 1 cm thick
# fingertip pads centred on the TCP along the approach, 13 cm apart when open, closing along y
GRIPPER = '''<robot name="gripper">
  <link name="base"/>
  <link name="tcp"/>
  <joint name="tcp_joint" type="fixed">
    <origin xyz="0 0 0.2" rpy="0 -1.5707963267948966 0"/><parent link="base"/><child link="tcp"/>
  </joint>
  <link name="left_tip"><collision><geometry><box size="0.02 0.01 0.04"/></geometry></collision></link>
  <link name="right_tip"><collision><geometry><box size="0.02 0.01 0.04"/></geometry></collision></link>
  <joint name="finger_joint" type="prismatic">
    <origin xyz="0 0.07 0.2"/><axis xyz="0 -1 0"/><parent link="base"/><child link="left_tip"/>
    <limit lower="0" upper="0.065" effort="1" velocity="1"/>
  </joint>
  <joint name="right_finger_joint" type="prismatic">
    <origin xyz="0 -0.07 0.2"/><axis xyz="0 1 0"/><parent link="base"/><child link="right_tip"/>
    <limit lower="0" upper="0.065" effort="1" velocity="1"/><mimic joint="finger_joint" multiplier="1"/>
  </joint>
</robot>'''

DOWN = tft.quaternion_from_euler(0.0, math.pi / 2, 0.0)  # TCP x (approach) along world -z


def envelope():
    return fingertip_envelope(URDF.from_xml_string(GRIPPER), 'tcp', ['left_tip', 'right_tip'], 'finger_joint')


def tilted(angle, axis):
    return tft.quaternion_multiply(DOWN, tft.quaternion_about_axis(angle, axis))


def test_gap_closes_from_open_to_closed():
    gaps = [gap for gap, _ in envelope()]
    assert math.isclose(gaps[0], 0.13, abs_tol=1e-9)
    assert math.isclose(gaps[-1], 0.0, abs_tol=1e-9)
    assert all(a > b for a, b in zip(gaps, gaps[1:]))


def test_vertical_grasp_reaches_the_pad_tip():
    for width in (0.0, 0.05, 0.13):
        assert math.isclose(lowest_point_offset(envelope(), DOWN, width), -0.02, abs_tol=1e-9)


def test_tilt_towards_a_finger_dips_the_open_finger():
    angle = math.radians(30)
    expected = -(0.02 * math.cos(angle) + 0.075 * math.sin(angle))  # open pad: outer face 7.5 cm off the TCP
    lowest = lowest_point_offset(envelope(), tilted(angle, (0, 0, 1)), 0.05)
    assert math.isclose(lowest, expected, abs_tol=1e-9)


def test_tilt_with_level_fingers_only_dips_the_pad_edge():
    angle = math.radians(30)
    expected = -(0.02 * math.cos(angle) + 0.01 * math.sin(angle))  # pad half width 1 cm
    assert math.isclose(lowest_point_offset(envelope(), tilted(angle, (0, 1, 0)), 0.05), expected, abs_tol=1e-9)


def test_straight_bonus():
    vertical, limit = 1.0, math.radians(20)
    assert math.isclose(straight_bonus(vertical, 1.5, limit), 1.5)
    assert math.isclose(straight_bonus(math.cos(math.radians(10)), 1.5, limit), 1.25)
    assert straight_bonus(math.cos(math.radians(25)), 1.5, limit) == 1.0
    assert straight_bonus(vertical, 1.0, limit) == 1.0  # off by default
    assert math.isclose(top_down_multiplier(vertical, 2.0, 1.5, limit), 3.0)
    assert math.isclose(top_down_multiplier(0.5, 2.0), 1.5)  # unchanged without the straight bonus


def test_straight_grasp_beats_a_better_oblique_one():
    '''#134: an oblique grasp (30 deg) with a 20 % higher network score no longer wins over a vertical one'''
    limit = math.radians(20)
    oblique = 1.2 * top_down_multiplier(math.cos(math.radians(30)), 2.0, 1.5, limit)
    straight = 1.0 * top_down_multiplier(1.0, 2.0, 1.5, limit)
    assert straight > oblique
