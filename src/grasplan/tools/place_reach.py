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
Order place candidates by how comfortably the arm reaches them (#132), no ROS.

gen_place_poses_from_plane() samples place poses anywhere on the support plane, and MTC and MoveIt try them in the
given order, so an object could end up about 1 m away, where a later pick from the same base pose fails. The
candidates are therefore tried nearest to the arm base first (horizontal distance), and optionally those beyond a
reach radius are dropped.
'''

import math


def order_by_reach(objects, base_xy, max_reach=0.0):
    '''
    objects: items with .pose.position (e.g. ObjectPose) in the frame of base_xy. Returns (objects nearest to base_xy
    first, number dropped). max_reach > 0 drops the objects farther than max_reach, unless that would drop all of
    them (the nearest ones are still worth a try then).
    '''

    def distance(obj):
        return math.hypot(obj.pose.position.x - base_xy[0], obj.pose.position.y - base_xy[1])

    ordered = sorted(objects, key=distance)
    if max_reach > 0.0:
        within = [obj for obj in ordered if distance(obj) <= max_reach]
        if within:
            return within, len(ordered) - len(within)
    return ordered, 0


def footprint_is_free(center, orientation, size, boxes, margin=0.04, headroom=0.15):
    '''
    #217: whether the held object's box at a place pose (center, orientation (x, y, z, w), size), grown by margin on each
    horizontal side and by headroom above it (the gripper around it), clears every box of boxes ([(center, orientation,
    size)], the perceived objects on the support). Exact for boxes standing on a face (tools.topdown_grasps prisms).
    '''
    from grasplan.tools.topdown_grasps import box_overlap_fraction
    grown = (size[0] + 2.0 * margin, size[1] + 2.0 * margin, size[2] + headroom)
    lifted = (center[0], center[1], center[2] + headroom / 2.0)
    return all(box_overlap_fraction(lifted, orientation, grown, c, o, s) <= 0.0 for c, o, s in boxes)


def order_free_first(objects, size, boxes, margin=0.04, headroom=0.15):
    '''
    objects (items with .pose, the place candidates in the boxes' frame, already in the order to try): the ones whose
    footprint is free (footprint_is_free) first, each group in its order; returns (ordered, number free). Nothing is
    dropped: MTC's collision check still decides, the free ones are only tried first.
    '''
    free, occupied = [], []
    for obj in objects:
        p, o = obj.pose.position, obj.pose.orientation
        (free if footprint_is_free((p.x, p.y, p.z), (o.x, o.y, o.z, o.w), size, boxes, margin, headroom)
         else occupied).append(obj)
    return free + occupied, len(free)
