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
