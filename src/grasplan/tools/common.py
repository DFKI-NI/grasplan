# Copyright (c) 2024 DFKI GmbH
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


import functools


def padded_box_size(size, padding):
    """The side lengths of a planning-scene box grown by ``padding`` metres on every side (0 = the box as given)."""
    if not padding:
        return size
    return [float(side) + 2.0 * padding for side in size]


def as_position_list(value, name='gripper position'):
    """The joint positions of a gripper posture from a parameter value: a number is the one position of a gripper with
    one joint, a list or tuple is one position per joint (a gripper with two joints names two positions). Anything else
    raises a ValueError that names the parameter."""
    values = value if isinstance(value, (list, tuple)) else [value]
    try:
        return [float(v) for v in values]
    except (TypeError, ValueError):
        raise ValueError(f'{name}: expected a number or a list of numbers, got {value!r}') from None


def gripper_positions(value, joint_names, name='gripper position'):
    """as_position_list() for a posture of these gripper joints: one position per joint, else a readable ValueError."""
    positions = as_position_list(value, name)
    if len(positions) != len(joint_names):
        raise ValueError(f'{name}: {len(positions)} position(s) {positions} for the {len(joint_names)} gripper '
                         f'joint(s) {list(joint_names)}')
    return positions


def object_list_frame(object_list, default):
    """The frame the poses of a pose selector answer (an ObjectList) are given in: its header frame when it carries one,
    else ``default`` (the node's ~global_reference_frame)."""
    header = getattr(object_list, 'header', None)
    return getattr(header, 'frame_id', '') or default


@functools.lru_cache(maxsize=1)
def arm_joint_tag():
    """What the names of the arm's joints contain, among all joints of a joint state: ~arm_joint_tag, default 'ur5_'.
    Read once per node, as joint state callers ask often. An empty tag would count every joint as an arm joint, so it
    raises a ValueError that says so."""
    import rospy

    tag = rospy.get_param('~arm_joint_tag', 'ur5_')
    if not tag:
        raise ValueError(f'~arm_joint_tag must name what the arm joints contain, got {tag!r}: it would match every joint')
    return tag


def supports_in_frame(supports, frame):
    """The support boxes (a dict frame -> boxes) given in ``frame``, the frame the perceived boxes are in. Supports only
    in other frames cannot be compared with them, so that is said once instead of passing every box unchecked."""
    found = (supports or {}).get(frame, [])
    if not found and supports:
        import rospy

        rospy.logwarn_once(f'no support boxes in frame {frame}, only in {sorted(supports)}: the perceived boxes are '
                           f'not checked against their supports')
    return found


def robot_prefix(fallback='mobipick'):
    """The robot's name for per-robot defaults (several Mobipicks in one sim): ~robot_prefix if set, else the
    node's namespace (the grasplan nodes run in the robot namespace, ns="mobipick"), else ``fallback``. Frames are
    '<prefix>/gripper_tcp', robot names '/<prefix>/gripper_hw'; a single robot keeps 'mobipick'. A robot whose nodes,
    frames and joints carry no prefix at all sets ~robot_prefix to '/': the result is then empty, so build the names
    with robot_frame() and robot_topic()."""
    import rospy

    param = str(rospy.get_param('~robot_prefix', '') or '')
    if param and not param.strip('/'):
        return ''
    name = (param or str(rospy.get_namespace())).strip('/')
    return name.split('/')[0] if name else fallback


def robot_frame(name):
    """A frame or joint name of this robot: '<prefix>/<name>', or name alone for a robot without a prefix."""
    prefix = robot_prefix()
    return f'{prefix}/{name}' if prefix else name


def robot_topic(name):
    """An absolute topic, action or parameter name of this robot: '/<prefix>/<name>', or '/<name>' without a prefix."""
    prefix = robot_prefix()
    return f'/{prefix}/{name}' if prefix else f'/{name}'

def connect_move_groups(robot, group_names, wait_for_servers):
    '''
    Construct the MoveGroupCommander of each group with a custom wait for the move_group action servers and
    cache it in robot, so that robot.<group> returns it. RobotCommander.get_group() always waits MoveIt's
    default 5 s, which is too short when move_group runs on another machine (the real robot seen from a laptop).
    '''
    import moveit_commander

    for name in group_names:
        if name not in robot._groups:
            robot._groups[name] = moveit_commander.MoveGroupCommander(
                name, robot._robot_description, robot._ns, wait_for_servers
            )


def roscpp_initialize_named(args):
    '''
    moveit_commander.roscpp_initialize(args), but the roscpp node of this process is called <rospy node name>_cpp
    (e.g. /mobipick/pick_object_node_cpp) instead of the anonymous move_group_commander_wrappers_<time>. MTC
    publishes its task introspection under that node (<name>/solution, description, statistics), so the RViz
    "Motion Planning Tasks" display can use a fixed topic. A __name remap turns off roscpp's anonymous suffix.
    '''
    import rospy
    from moveit_ros_planning_interface import _moveit_roscpp_initializer

    name = rospy.get_name().split('/')[-1] + '_cpp'
    args = [a for a in args if not a.startswith('__name:=')] + [f'__name:={name}']
    _moveit_roscpp_initializer.roscpp_init(name, args)


def separate_object_class_from_id(anchored_object_as_string):
    '''
    e.g. input  : relay_1
            output : relay, 1
    '''
    if '_' not in anchored_object_as_string:
        return anchored_object_as_string, None
    count = 0
    for char in reversed(anchored_object_as_string):
        count += 1
        if char == '_':
            break
    chars_before_number = anchored_object_as_string[:-count]
    try:
        object_id = int(anchored_object_as_string[-count + 1 :])
        return chars_before_number, object_id
    except ValueError:
        return anchored_object_as_string, None


class objectToPick:
    def __init__(self, obj_class_and_id_as_string=None):
        '''
        this class is meant to be used as a struct that holds info on:
        object class, id, and a boolean to represent whether any obj should be picked.
        self.any_obj_id : if true it means any object_id can be picked
        '''
        self.obj_class = None
        self.id = None
        self.any_obj_id = None
        if obj_class_and_id_as_string is not None:
            assert isinstance(obj_class_and_id_as_string, str)
            self.obj_class, self.id = separate_object_class_from_id(obj_class_and_id_as_string)
            if self.id is None:
                self.any_obj_id = True
            else:
                self.any_obj_id = False

    def set_object_class(self, obj_class):
        assert isinstance(obj_class, str)
        self.obj_class = obj_class

    def set_id(self, id):
        assert isinstance(id, int)
        if id is None:
            self.any_obj_id = True
            return
        self.id = id
        self.object_name = self.obj_class + '_' + str(self.id)

    def set_any_object(self, value):
        assert isinstance(value, bool)
        self.any_obj_id = value

    def get_object_class_and_id_as_string(self):
        assert self.obj_class is not None
        if self.id is None:
            return self.obj_class
        else:
            return self.obj_class + '_' + str(self.id)

    def get_all(self):
        return self.obj_class, self.id, self.any_obj_id
