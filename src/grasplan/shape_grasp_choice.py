#!/usr/bin/env python3

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
Grasp-type rule from measured shape features (spec of 2026-10-02): top only, side ranking or auto.

Pure function over the dict of grasplan.shape_features.shape_features: no ROS, no names, no classes (rule 8a). The
decision is in this order and returns the rule that fired, so every pick can log it:

  top_only          the object is low (height reliable and below ``low_max_height``), flat, or sphere-like and small
                    (``sphere_top_max_height``): only grasps from straight above (the top_grasps_for_low_objects path)
  side_standing     a standing cylinder or a tall box-like object that fits the gripper across and is at least
                    ``side_min_height`` high: side grasps across its vertical axis (side_ranked / side bonus)
  side_short_axis   a lying cylinder or an elongated object that fits the gripper across its short side: side grasps
                    across the short axis
  auto              everything else, and every case in which the outline or height needed is not reliable: today's
                    box rules (grasp_scoring.side_grasp_axis / is_low_object) decide, so a bad cloud degrades to the
                    current behaviour and never to something worse
'''

LOW_MAX_HEIGHT = 0.05           # m, below: top only (the 2026-09-28 low-object rule)
SPHERE_TOP_MAX_HEIGHT = 0.055   # m, a sphere-like object up to this size is grasped from the top; larger balls keep side grasps
SIDE_MIN_HEIGHT = 0.06          # m, a standing object needs this much height for a side grasp

TOP_ONLY = 'top_only'
SIDE_STANDING = 'side_standing'
SIDE_SHORT_AXIS = 'side_short_axis'
AUTO = 'auto'


def choose_grasp_rule(features, low_max_height=LOW_MAX_HEIGHT, sphere_top_max_height=SPHERE_TOP_MAX_HEIGHT,
                      side_min_height=SIDE_MIN_HEIGHT):
    '''(rule, reason): the grasp-type rule for an object's shape features and a short text of why, for the log'''
    height = features.get('height')
    height_ok = bool(features.get('height_reliable')) and height is not None
    outline_ok = bool(features.get('outline_reliable'))
    shape = features.get('shape')
    if height_ok and height < low_max_height:
        return TOP_ONLY, f'low object: height {height:.3f} m < {low_max_height:.3f} m'
    if features.get('flat') and height_ok and shape != 'lying_cylinder':   # its height is its diameter, not a thin plate
        return TOP_ONLY, f'flat object: height {height:.3f} m, smaller footprint side {features.get("footprint_min"):.3f} m'
    if outline_ok and shape == 'sphere':
        size = height if height_ok else features.get('diameter')
        if size is not None and size < sphere_top_max_height:
            return TOP_ONLY, f'small sphere-like object: {size:.3f} m < {sphere_top_max_height:.3f} m'
    if not outline_ok:
        return AUTO, 'outline not reliable: ' + '; '.join(features.get('reasons') or ['no points'])
    graspable = features.get('graspable_across')
    if shape in ('cylinder', 'box') and height_ok and features.get('tall') and graspable \
            and height >= side_min_height:
        return SIDE_STANDING, (f'standing {shape}: height {height:.3f} m, footprint_min {features["footprint_min"]:.3f} m, '
                               'fits across')
    if shape == 'sphere' and height_ok and graspable and height >= side_min_height:
        return SIDE_STANDING, f'sphere-like object, {height:.3f} m: side grasps (not small)'
    if shape == 'lying_cylinder' and graspable:
        return SIDE_SHORT_AXIS, f'lying cylinder: across its short axis {features["footprint_min"]:.3f} m'
    if shape == 'box' and features.get('elongated') and graspable:
        return SIDE_SHORT_AXIS, f'elongated box: across its short side {features["footprint_min"]:.3f} m'
    if graspable is False:
        return AUTO, 'does not fit the gripper across: the box rules decide'
    return AUTO, f'{shape} object without a clear rule: the box rules decide'
