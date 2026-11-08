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
Conversion between the shape feature dict of grasplan.shape_features and the grasplan/ShapeFeatures message.

ROS-free: the message class is passed in, so the real server, the sim mockup and grasplan convert the same way and the
tests need no ROS. None <-> NaN (floats) and UNKNOWN (tri-state).
'''
import math

FLOATS = ('height', 'diameter', 'footprint_min', 'footprint_max', 'circle_residual', 'arc_span_deg', 'rect_residual')
TRI_STATES = {'round_outline': 'round', 'tall': 'tall', 'flat': 'flat', 'elongated': 'elongated',
              'graspable_across': 'graspable_across'}   # message field -> feature key
UNKNOWN, NO, YES = -1, 0, 1


def _float(value):
    return float('nan') if value is None else float(value)


def _tri(value):
    return UNKNOWN if value is None else (YES if value else NO)


def fill_message(message, features):
    '''set the fields of a ShapeFeatures ``message`` from a shape_features dict (None: valid stays False); returns it'''
    if not features:
        message.valid = False
        return message
    message.valid = True
    message.outline_reliable = bool(features['outline_reliable'])
    message.height_reliable = bool(features['height_reliable'])
    message.points = int(features['points'])
    message.shape = str(features['shape'])
    for name in FLOATS:
        setattr(message, name, _float(features.get(name)))
    message.radii = [_float(r) for r in (features.get('radii') or [])]
    for field, key in TRI_STATES.items():
        setattr(message, field, _tri(features.get(key)))
    return message


def features_from_message(message):
    '''the shape_features dict of a ShapeFeatures message, or None when it holds no features (valid false)'''
    if not message.valid:
        return None
    features = {'points': int(message.points), 'shape': str(message.shape),
                'outline_reliable': bool(message.outline_reliable), 'height_reliable': bool(message.height_reliable),
                'reasons': []}
    for name in FLOATS:
        value = float(getattr(message, name))
        features[name] = None if math.isnan(value) else value
    features['radii'] = [None if math.isnan(r) else float(r) for r in message.radii] or None
    for field, key in TRI_STATES.items():
        value = int(getattr(message, field))
        features[key] = None if value == UNKNOWN else bool(value)
    return features
