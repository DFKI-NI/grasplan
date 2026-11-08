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
Generic shape features of an object's point cloud: round or box-like, tall, flat, elongated, how wide across.

ROS-free (numpy only) so the real AnyGrasp server and the sim mockup run the SAME function. The input is the object's
points in the gravity-aligned floor frame (z up) and the height of the support plane; nothing here knows an object's name,
class or dimensions (rule 8a, AGENTS.md): every number is measured from the points, and a feature the points cannot
support is None, never a guess.

A single view sees only the front of an object, so the shape is read from what is visible: a circle fitted to the
horizontal outline gives the diameter of a can from its front arc alone, a rectangle fit gives the two sides of a box
from the faces seen. The support plane, not the cloud's own bottom, is the zero of the height: the bottom edge of one view
lies above the table.
'''
import math

import numpy as np

MIN_POINTS = 150                # fewer points than this after trimming: no reliable outline or height
MIN_SLICE_POINTS = 20           # points a height slice needs for a fit
SLICE_FRACTIONS = (0.25, 0.5, 0.75)
ROUND_MAX_RESIDUAL = 0.10       # rms circle residual / radius below which the outline is round
ROUND_MIN_ARC_DEG = 90.0        # visible arc a circle fit needs
NOISE_FLOOR_M = 0.002           # depth noise (m): a circle residual below it counts as round on a small object
CONSTANT_RADIUS_TOLERANCE = 0.15
SPHERE_BULGE = 1.1              # middle radius / larger end radius from which the profile is sphere-like
TALL_RATIO = 1.5                # height / smaller footprint side
FLAT_RATIO = 0.4                # height / smaller footprint side, at most
ELONGATED_RATIO = 2.0           # larger / smaller footprint side
MIN_SECOND_SIDE = 0.02          # a box face-on shows one side only: the other is unknown below this extent (m)
DEFAULT_MAX_OPENING = 0.14      # gripper opening (m)
OPENING_MARGIN = 0.01           # kept free of the opening (m)
FINGER_PAD_MARGIN = 0.01        # clearance per finger beside the object (m)
HEIGHT_PERCENTILE = 99.0


def fit_circle(xy):
    '''
    Least-squares circle (Kasa) through 2-D points: (centre (2,), radius, rms residual / radius, visible arc in deg), or
    None when the points do not span a circle (collinear, fewer than 3).
    '''
    xy = np.asarray(xy, dtype=float)
    if len(xy) < 3:
        return None
    mean = xy.mean(axis=0)
    shifted = xy - mean
    design = np.column_stack([2.0 * shifted[:, 0], 2.0 * shifted[:, 1], np.ones(len(xy))])
    target = (shifted ** 2).sum(axis=1)
    try:
        (a, b, c), *_ = np.linalg.lstsq(design, target, rcond=None)
    except np.linalg.LinAlgError:
        return None
    radius_squared = c + a * a + b * b
    if not np.isfinite(radius_squared) or radius_squared <= 0.0:
        return None
    radius = math.sqrt(radius_squared)
    centre = np.array([a, b])
    distances = np.hypot(shifted[:, 0] - a, shifted[:, 1] - b)
    residual = float(np.sqrt(np.mean((distances - radius) ** 2)) / radius)
    angles = np.sort(np.arctan2(shifted[:, 1] - b, shifted[:, 0] - a))
    gaps = np.diff(np.concatenate([angles, [angles[0] + 2.0 * math.pi]]))
    arc = 360.0 - math.degrees(float(gaps.max()))
    return centre + mean, radius, residual, arc


def fit_rectangle(xy):
    '''
    Rectangle through 2-D points along their principal axes: (side lengths (major, minor) in m, rms distance of the
    points to the nearest rectangle side in m, unit major axis (2,)). The points of a box seen from one view lie on one
    or two of its sides, which a bounding rectangle reproduces with a small residual; a circular arc does not.
    '''
    xy = np.asarray(xy, dtype=float)
    centred = xy - xy.mean(axis=0)
    if len(xy) >= 2:
        eigenvalues, eigenvectors = np.linalg.eigh(np.cov(centred.T))
        major = eigenvectors[:, int(np.argmax(eigenvalues))]
    else:
        major = np.array([1.0, 0.0])
    minor = np.array([-major[1], major[0]])
    u, v = centred @ major, centred @ minor
    low = np.array([u.min(), v.min()])
    high = np.array([u.max(), v.max()])
    sides = high - low
    nearest = np.minimum.reduce([u - low[0], high[0] - u, v - low[1], high[1] - v])
    return (float(sides[0]), float(sides[1])), float(np.sqrt(np.mean(nearest ** 2))), major


def _outline(xy):
    '''Round or box-like reading of one horizontal slice, or None for too few points.'''
    if len(xy) < MIN_SLICE_POINTS:
        return None
    rectangle = fit_rectangle(xy)
    circle = fit_circle(xy)
    result = {'sides': rectangle[0], 'rect_residual': rectangle[1], 'circle': None}
    if circle is not None:
        centre, radius, residual, arc = circle
        result['circle'] = {'radius': radius, 'residual': residual, 'arc': arc}
        # the rectangle residual is metres, the circle one a fraction of the radius: compare both as fractions of the size
        rect_fraction = rectangle[1] / max(radius, 1e-9)
        fits = residual * radius < max(ROUND_MAX_RESIDUAL * radius, NOISE_FLOOR_M)
        result['round'] = bool(fits and arc >= ROUND_MIN_ARC_DEG and residual <= rect_fraction)
    else:
        result['round'] = False
    return result


def _lying_cylinder(points, height):
    '''
    (radius, length) of a cylinder lying on the support, or None: the horizontal outline is elongated (its major axis at
    least ELONGATED_RATIO times the minor one) and the cross-section across that axis (minor axis and height) is a circle
    of about the object's own height.
    '''
    xy = points[:, :2]
    sides, _, major = fit_rectangle(xy)
    if sides[1] <= 0.0 or sides[0] / sides[1] < ELONGATED_RATIO:
        return None
    minor = np.array([-major[1], major[0]])
    section = np.column_stack([(xy - xy.mean(axis=0)) @ minor, points[:, 2]])
    circle = fit_circle(section)
    if circle is None:
        return None
    _, radius, residual, arc = circle
    if residual >= ROUND_MAX_RESIDUAL or arc < ROUND_MIN_ARC_DEG:
        return None
    if not 0.7 * height <= 2.0 * radius <= 1.3 * height:
        return None
    return radius, sides[0]


def shape_features(points_floor, support_z, completed_depth=False, max_opening=DEFAULT_MAX_OPENING):
    '''
    Generic shape features of an object from its points (N x 3, floor frame, z up) and the support plane height.

    Returns a dict (metres and ratios; None where the points cannot support a feature):
      points, height, height_reliable, outline_reliable,
      shape: 'cylinder' (standing, round outline, constant radius), 'sphere' (round outline, radius bulging in the
        middle), 'lying_cylinder', 'box' (box-like outline), 'unknown',
      round (outline is round), diameter (round shapes), footprint_min, footprint_max, radii (per height slice),
      circle_residual, arc_span_deg, rect_residual, tall, flat, elongated, graspable_across, reasons (why a group is not
      reliable).
    completed_depth: the depth of the object was filled in (a small dark object has none of its own): its height is then
    not reliable.
    '''
    points = np.asarray(points_floor, dtype=float).reshape(-1, 3)
    features = {
        'points': int(len(points)), 'height': None, 'height_reliable': False, 'outline_reliable': False,
        'shape': 'unknown', 'round': None, 'diameter': None, 'footprint_min': None, 'footprint_max': None,
        'radii': None, 'circle_residual': None, 'arc_span_deg': None, 'rect_residual': None, 'tall': None,
        'flat': None, 'elongated': None, 'graspable_across': None, 'reasons': [],
    }
    reasons = features['reasons']
    if len(points) < MIN_POINTS:
        reasons.append(f'only {len(points)} points (need {MIN_POINTS})')
        return features
    height = float(np.percentile(points[:, 2], HEIGHT_PERCENTILE) - support_z)
    if height <= 0.0:
        reasons.append('the object is not above the support plane')
        return features
    features['height'] = height
    if completed_depth:
        reasons.append('depth was filled in: height not reliable')
    else:
        features['height_reliable'] = True

    # horizontal outline at the three height slices
    outlines, radii = [], []
    for fraction in SLICE_FRACTIONS:
        centre_z = support_z + fraction * height
        half = max(0.06 * height, 0.003)
        band = points[np.abs(points[:, 2] - centre_z) <= half][:, :2]
        outline = _outline(band)
        outlines.append(outline)
        radii.append(outline['circle']['radius'] if outline and outline['circle'] and outline['round'] else None)
    usable = [o for o in outlines if o is not None]
    if len(usable) < 2:
        reasons.append('too few points in the height slices for an outline')
        return features
    features['outline_reliable'] = True
    round_votes = sum(1 for o in usable if o['round'])
    middle = outlines[1] if outlines[1] is not None else usable[0]
    features['rect_residual'] = float(np.median([o['rect_residual'] for o in usable]))
    if middle['circle'] is not None:
        features['circle_residual'] = float(middle['circle']['residual'])
        features['arc_span_deg'] = float(middle['circle']['arc'])

    lying = _lying_cylinder(points, height) if round_votes < len(usable) else None
    if round_votes * 2 > len(usable):
        features['round'] = True
        known = [r for r in radii if r is not None]
        features['radii'] = [None if r is None else float(r) for r in radii]
        end = max(r for r in (radii[0], radii[2]) if r is not None) if (radii[0] or radii[2]) else None
        diameter = 2.0 * float(np.median(known))
        if radii[1] is not None and end is not None and radii[1] >= SPHERE_BULGE * end:
            features['shape'] = 'sphere'
            diameter = 2.0 * radii[1]   # the widest slice is the ball's own diameter; the others are cut off-centre
        elif max(known) - min(known) <= CONSTANT_RADIUS_TOLERANCE * float(np.median(known)):
            features['shape'] = 'cylinder'
        else:
            features['shape'] = 'unknown'
            reasons.append('round outline whose radius changes along the height')
        features['diameter'] = diameter
        features['footprint_min'] = features['footprint_max'] = diameter
    elif lying is not None:
        features['round'] = True
        features['shape'] = 'lying_cylinder'
        features['diameter'] = 2.0 * lying[0]
        features['footprint_min'] = 2.0 * lying[0]
        features['footprint_max'] = lying[1]
    else:
        features['round'] = False
        features['shape'] = 'box'
        sides = np.median(np.array([o['sides'] for o in usable]), axis=0)
        features['footprint_max'] = float(sides[0])
        if sides[1] >= MIN_SECOND_SIDE:
            features['footprint_min'] = float(sides[1])
        else:
            reasons.append('one side seen face-on: the other side is unknown')

    low, high = features['footprint_min'], features['footprint_max']
    if features['height_reliable'] and low:
        features['tall'] = bool(height / low >= TALL_RATIO)
    if features['height_reliable'] and low:
        # flat: BOTH footprint sides are at least 1 / FLAT_RATIO times the height (a plate), not a long bar that is merely
        # lower than it is long: the spec's ratio to the larger side called a 5 x 25 x 7 cm bar flat
        features['flat'] = bool(height / low <= FLAT_RATIO)
    if low and high:
        features['elongated'] = bool(high / low >= ELONGATED_RATIO)
        features['graspable_across'] = bool(low + 2.0 * FINGER_PAD_MARGIN <= max_opening - OPENING_MARGIN)
    return features
