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
Heat-map style RViz markers for scored grasp candidates.

Every grasplan/GraspCandidate may carry any number of named scores (see
GraspScore.msg). This module renders one MarkerArray layer per score name so
that the raw network score, any bonus terms and the final ranking value can be
compared side by side in RViz by toggling namespaces. Colours run from blue
(lowest score in the current set) through green and yellow to red (highest),
normalised over the candidates that are passed in, because absolute score
ranges differ between producers.

The grasp approach direction is the +x axis of the candidate pose (AnyGrasp
convention): arrows start behind the grasp point and end at it.
'''

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

FINAL_SCORE_NAME = 'final'
SELECTED_NAMESPACE = 'selected'
REJECTED_NAMESPACE = 'rejected'


def score_colour(value, alpha=1.0):
    '''Map a normalised score in [0, 1] to a blue -> green -> yellow -> red gradient.'''
    value = min(1.0, max(0.0, float(value)))
    if value < 1.0 / 3.0:
        t = value * 3.0
        r, g, b = 0.0, t, 1.0 - t
    elif value < 2.0 / 3.0:
        t = (value - 1.0 / 3.0) * 3.0
        r, g, b = t, 1.0, 0.0
    else:
        t = (value - 2.0 / 3.0) * 3.0
        r, g, b = 1.0, 1.0 - t, 0.0
    return ColorRGBA(r=r, g=g, b=b, a=alpha)


def normalise(values):
    '''Scale a list of numbers to [0, 1]; a constant list maps to 1.0 everywhere.'''
    if not values:
        return []
    low, high = min(values), max(values)
    if high - low < 1e-9:
        return [1.0] * len(values)
    return [(value - low) / (high - low) for value in values]


def candidate_scores(candidate):
    '''Return {name: value} for a GraspCandidate, always including "final".'''
    scores = {score.name: float(score.value) for score in getattr(candidate, 'scores', [])}
    scores.setdefault(FINAL_SCORE_NAME, float(candidate.quality))
    return scores


def rotate_x_axis(orientation):
    '''Unit +x axis of a quaternion (geometry_msgs/Quaternion) as a tuple.'''
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    return (
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + z * w),
        2.0 * (x * z - y * w),
    )


def arrow_marker(header, namespace, marker_id, pose, colour, length, shaft_diameter):
    marker = Marker()
    marker.header = header
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    approach = rotate_x_axis(pose.orientation)
    tip = pose.position
    tail = Point(
        x=tip.x - approach[0] * length,
        y=tip.y - approach[1] * length,
        z=tip.z - approach[2] * length,
    )
    marker.points = [tail, Point(x=tip.x, y=tip.y, z=tip.z)]
    marker.scale.x = shaft_diameter
    marker.scale.y = shaft_diameter * 2.5
    marker.scale.z = length * 0.3
    marker.color = colour
    marker.lifetime = rospy_duration(0)
    return marker


def text_marker(header, namespace, marker_id, pose, text, colour, height):
    marker = Marker()
    marker.header = header
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = pose.position.x
    marker.pose.position.y = pose.position.y
    marker.pose.position.z = pose.position.z + height * 1.5
    marker.pose.orientation.w = 1.0
    marker.scale.z = height
    marker.color = colour
    marker.text = text
    marker.lifetime = rospy_duration(0)
    return marker


def rospy_duration(seconds):
    # Imported lazily so this module stays importable without a ROS master.
    import rospy

    return rospy.Duration(seconds)


def delete_all_marker(header):
    marker = Marker()
    marker.header = header
    marker.action = Marker.DELETEALL
    return marker


def scored_grasp_markers(
    candidates,
    header,
    selected_index=None,
    rejected_indices=(),
    arrow_length=0.08,
    shaft_diameter=0.006,
    label_top_n=3,
    namespace_prefix='',
):
    '''
    Build a MarkerArray with one arrow layer per score name.

    candidates: iterable of grasplan/GraspCandidate (poses in header.frame_id)
    selected_index: index of the grasp chosen for execution, drawn white and thick
    rejected_indices: indices rejected as unreachable, drawn grey and thin
    label_top_n: number of best "final" grasps that get a text label with their score
    namespace_prefix: prepended to every namespace, e.g. "anygrasp/"
    '''
    candidates = list(candidates)
    markers = MarkerArray()
    markers.markers.append(delete_all_marker(header))
    if not candidates:
        return markers

    rejected = set(rejected_indices)
    per_name = {}
    for index, candidate in enumerate(candidates):
        for name, value in candidate_scores(candidate).items():
            per_name.setdefault(name, {})[index] = value

    for name, values_by_index in per_name.items():
        indices = sorted(values_by_index)
        normalised = normalise([values_by_index[index] for index in indices])
        namespace = namespace_prefix + name
        for index, norm in zip(indices, normalised):
            if index in rejected or index == selected_index:
                continue
            pose = candidates[index].pose
            colour = score_colour(norm, alpha=0.9)
            markers.markers.append(arrow_marker(header, namespace, index, pose, colour, arrow_length, shaft_diameter))

    grey = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.35)
    for index in sorted(rejected):
        if index == selected_index or not 0 <= index < len(candidates):
            continue
        pose = candidates[index].pose
        markers.markers.append(
            arrow_marker(header, namespace_prefix + REJECTED_NAMESPACE, index, pose, grey, arrow_length, shaft_diameter * 0.6)
        )

    if selected_index is not None and 0 <= selected_index < len(candidates):
        pose = candidates[selected_index].pose
        white = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        markers.markers.append(
            arrow_marker(header, namespace_prefix + SELECTED_NAMESPACE, selected_index, pose, white, arrow_length * 1.5, shaft_diameter * 2.5)
        )
        final = candidate_scores(candidates[selected_index]).get(FINAL_SCORE_NAME, 0.0)
        markers.markers.append(
            text_marker(header, namespace_prefix + SELECTED_NAMESPACE, 10000 + selected_index, pose, f'selected {final:.2f}', white, 0.02)
        )

    final_values = per_name.get(FINAL_SCORE_NAME, {})
    ranked = sorted(final_values, key=lambda index: final_values[index], reverse=True)
    labels_namespace = namespace_prefix + 'labels'
    for rank, index in enumerate(ranked[:label_top_n]):
        if index == selected_index:
            continue
        pose = candidates[index].pose
        markers.markers.append(
            text_marker(header, labels_namespace, index, pose, f'#{rank + 1} {final_values[index]:.2f}', ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9), 0.015)
        )
    return markers


def make_candidate_scores(final, **named):
    '''Convenience: build a GraspScore list from a final value plus named components.'''
    from grasplan.msg import GraspScore

    scores = [GraspScore(name=name, value=float(value)) for name, value in named.items()]
    scores.append(GraspScore(name=FINAL_SCORE_NAME, value=float(final)))
    return scores
