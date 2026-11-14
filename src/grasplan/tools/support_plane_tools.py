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

import copy
import os
import random
import rospy
import tf
import math
import numpy as np
import yaml

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3, PointStamped
from std_msgs.msg import Header
from object_pose_msgs.msg import ObjectList, ObjectPose
from moveit_msgs.msg import CollisionObject, PlanningScene
from typing import List, Optional


def make_plane_marker_msg(ref_frame, plane):
    '''
    receives 4 points, makes 2 triangles to form a squared plane
    '''
    assert isinstance(plane, list)
    assert len(plane) == 4
    for i in range(4):
        assert isinstance(plane[i], Point)
    marker_msg = Marker()
    # marker_msg.lifetime = rospy.Duration(15.0)
    marker_msg.ns = 'support_plane'
    marker_msg.pose.orientation.x = 0.0
    marker_msg.pose.orientation.y = 0.0
    marker_msg.pose.orientation.z = 0.0
    marker_msg.pose.orientation.w = 1.0
    marker_msg.header.frame_id = ref_frame
    marker_msg.type = Marker.TRIANGLE_LIST
    # p0, p1, p2
    marker_msg.points.append(plane[0])
    marker_msg.points.append(plane[1])
    marker_msg.points.append(plane[2])
    # p2, p3, p0
    marker_msg.points.append(plane[2])
    marker_msg.points.append(plane[3])
    marker_msg.points.append(plane[0])
    marker_msg.scale = Vector3(1.0, 1.0, 1.0)
    marker_msg.color = ColorRGBA(1.0, 0.61, 0.16, 1.0)  # orange
    return marker_msg


# resting height (vertical extent in the object's stable pose) of the objects grasplan knows, used by insertion;
# ohd : objects height dictionary
OBJECT_HEIGHTS = {
    'power_drill_with_grip': 0.2205359935760498,
    'hot_glue_gun': 0.2,
    'klt': 0.14699999809265138,
    'multimeter': 0.04206399992108345,
    'relay': 0.10436400026082993,
    'screwdriver': 0.034412000328302383,
    'bleach': 0.25,
    'mustard': 0.2,
    'soup': 0.1,
    'meat': 0.1,
}


# classes whose mesh frame lies flat (z is its smallest extent) and that rest upright: their place and insert
# poses roll the model by -pi/2 about x so the mesh y axis points up. This is the resting pose grasplan chooses; it
# cannot be read from the mesh (a mesh has no preferred resting side), so the choice stays a named constant.
UPRIGHT_ROLL_CLASSES = ['power_drill_with_grip', 'hot_glue_gun', 'bleach', 'mustard', 'soup', 'meat']


def resting_roll(object_class):
    '''roll (rad) applied to the model of a closed-set object in its place and insert poses'''
    return -math.pi / 2.0 if object_class in UPRIGHT_ROLL_CLASSES else 0.0


def _object_package_config(file_name, package='pbr_objects'):
    import rospkg

    path = os.path.join(rospkg.RosPack().get_path(package), 'config', file_name)
    with open(path) as config_file:
        return yaml.safe_load(config_file)


def mesh_resting_height(object_class, dimensions=None, model_transforms=None):
    '''
    resting height (m) of a closed-set object read from the bounding box of its recognised mesh (`dimensions.yaml` of
    pbr_objects, cm along the mesh x, y, z): the box is turned like the object stands, first by the optional model
    transform of the detector, then by the roll of its resting pose, and the vertical extent is taken. Returns None
    when the class has no box.
    dimensions, model_transforms : the parsed config files (read from pbr_objects when None)
    '''
    if dimensions is None:
        dimensions = _object_package_config('dimensions.yaml').get('dimensions', {})
    if model_transforms is None:
        model_transforms = _object_package_config('model_transforms.yaml').get('model_transforms', {})
    box = dimensions.get(object_class)
    if box is None:
        return None
    extent = np.abs(np.array(box, dtype=float)) / 100.0
    if object_class in model_transforms:
        extent = np.abs(np.array(model_transforms[object_class], dtype=float)[:3, :3]).dot(extent)
    roll = resting_roll(object_class)
    # a roll about x mixes y and z only
    return abs(math.sin(roll)) * extent[1] + abs(math.cos(roll)) * extent[2]


def object_height(object_class, use_mesh_heights=False):
    '''
    resting height of a closed-set object: the hand-typed table value, or with use_mesh_heights (feature flag
    `use_mesh_object_heights`) the mesh bounding box value when the class has one. The table also tells which classes
    are known: a class outside OBJECT_HEIGHTS raises KeyError whatever the flag says.
    '''
    table_height = OBJECT_HEIGHTS[object_class]
    if not use_mesh_heights:
        return table_height
    try:
        mesh_height = mesh_resting_height(object_class)
    except Exception as exception:  # no pbr_objects or a broken config: keep the table
        rospy.logwarn_once(f'mesh heights unavailable ({exception}), using the height table')
        return table_height
    return table_height if mesh_height is None else mesh_height


def compute_object_height_for_insertion(
    object_class_tbi,
    support_obj_class,
    gap_between_objects=0.02,
    object_tbi_height=None,
    support_obj_height=None,
    use_mesh_heights=False,
):
    '''
    height of the insert pose above the support object centre: half of each object plus a gap
    object_class_tbi : object class to be inserted
    object_tbi_height, support_obj_height : measured heights (e.g. from the planning scene or pose selector) for
        objects that are not in OBJECT_HEIGHTS, such as open-set objects picked through AnyGrasp; when given they
        take precedence over the table
    use_mesh_heights : read the table classes' heights from their mesh boxes (flag `use_mesh_object_heights`)
    '''
    if object_tbi_height is None:
        if object_class_tbi not in OBJECT_HEIGHTS:
            raise ValueError(f'unknown height of object to insert: {object_class_tbi}, pass object_tbi_height')
        object_tbi_height = object_height(object_class_tbi, use_mesh_heights)
    if support_obj_height is None:
        if support_obj_class not in OBJECT_HEIGHTS:
            raise ValueError(f'unknown height of support object: {support_obj_class}, pass support_obj_height')
        support_obj_height = object_height(support_obj_class, use_mesh_heights)
    return (support_obj_height / 2.0) + (object_tbi_height / 2.0) + gap_between_objects


def gen_insert_poses_from_obj(
    object_class, support_object_pose, obj_height, frame_id='map', same_orientation_as_support_obj=False, open_set=False
):
    '''
    open_set: a known object inserted as an open-set one (picked by the pick node's second try): no resting roll of its
        class
    if same_orientation_as_support_obj is True then the object is aligned with the support object (e.g. box)
    if same_orientation_as_support_obj is False then 360 degree orientations are used to place the object (can
        be used if first time fails)
    '''
    object_list_msg = ObjectList()
    object_list_msg.header.frame_id = frame_id
    insert_poses_id = 1
    # only one insert pose for now
    object_pose_msg = ObjectPose()
    object_pose_msg.class_id = object_class
    object_pose_msg.pose.position.x = support_object_pose.pose.position.x
    object_pose_msg.pose.position.y = support_object_pose.pose.position.y
    object_pose_msg.pose.position.z = support_object_pose.pose.position.z + obj_height
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    roll = 0.0 if open_set else resting_roll(object_class)

    if not same_orientation_as_support_obj:
        for i in range(8):
            angular_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            object_pose_msg.pose.orientation.x = angular_q[0]
            object_pose_msg.pose.orientation.y = angular_q[1]
            object_pose_msg.pose.orientation.z = angular_q[2]
            object_pose_msg.pose.orientation.w = angular_q[3]
            object_pose_msg.instance_id = insert_poses_id
            object_list_msg.objects.append(copy.deepcopy(object_pose_msg))
            insert_poses_id += 1
            yaw += 0.5  # ~ 30 degree
    else:
        q = [
            support_object_pose.pose.orientation.x,
            support_object_pose.pose.orientation.y,
            support_object_pose.pose.orientation.z,
            support_object_pose.pose.orientation.w,
        ]
        euler_rot = tf.transformations.euler_from_quaternion(q)
        if roll != 0.0:
            q_new = tf.transformations.quaternion_from_euler(
                euler_rot[0] + roll, euler_rot[1], euler_rot[2]
            )  # yaw + 180 degree
        else:
            q_new = tf.transformations.quaternion_from_euler(
                euler_rot[0], euler_rot[1], euler_rot[2]
            )
        object_pose_msg.pose.orientation.x = q_new[0]
        object_pose_msg.pose.orientation.y = q_new[1]
        object_pose_msg.pose.orientation.z = q_new[2]
        object_pose_msg.pose.orientation.w = q_new[3]
        object_pose_msg.instance_id = insert_poses_id
        object_list_msg.objects.append(copy.deepcopy(object_pose_msg))
        insert_poses_id += 1
        if roll != 0.0:
            q_new = tf.transformations.quaternion_from_euler(
                euler_rot[0] + roll, euler_rot[1], euler_rot[2] + 3.14159
            )  # yaw + 180 degree
        else:
            q_new = tf.transformations.quaternion_from_euler(
                euler_rot[0], euler_rot[1], euler_rot[2] + 3.14159
            )
        object_pose_msg.pose.orientation.x = q_new[0]
        object_pose_msg.pose.orientation.y = q_new[1]
        object_pose_msg.pose.orientation.z = q_new[2]
        object_pose_msg.pose.orientation.w = q_new[3]
        object_pose_msg.instance_id = insert_poses_id
        object_list_msg.objects.append(copy.deepcopy(object_pose_msg))
        insert_poses_id += 1
    return object_list_msg


def well_separated(x_y_list, candidate_x, candidate_y, min_dist=0.2):
    if len(x_y_list) == 0:
        return True
    true_count = 0
    for p in x_y_list:
        if math.dist([candidate_x, candidate_y], p) > min_dist:
            true_count += 1
    if true_count == len(x_y_list):
        return True
    return False


def sample_in_plane(plane, within=None, tries=200):
    '''
    (x, y) uniformly in the axis-aligned plane (4 points as from adjust_plane); with within (cx, cy, r) also inside that
    disk (the arm's reach, so no draw is wasted on spots the reach ordering drops), falling back to the plane when
    tries draws found no such point (the disk misses the plane)
    '''
    for _ in range(tries if within is not None else 1):
        x = round(random.uniform(plane[0].x, plane[1].x), 4)
        y = round(random.uniform(plane[0].y, plane[3].y), 4)
        if within is None or math.hypot(x - within[0], y - within[1]) <= within[2]:
            return x, y
    return x, y


def gen_place_poses_from_plane(
    place_action_server,
    object_class: str,
    support_object: str,
    plane: List[str],
    planning_scene: PlanningScene,
    frame_id: str = "map",
    number_of_poses: int = 10,
    min_dist: float = 0.2,
    ignore_min_dist_list: List[str] = [],
    yaw_range: float = math.pi,
    yaws: Optional[List[float]] = None,
    height_offset: float = 0.001,
    within: Optional[tuple] = None,
    open_set: bool = False,
):
    '''
    random sample poses within a plane and populate object list msg with the result
    open_set: a known object placed as an open-set one (picked by the pick node's second try): no resting roll of its
        class, its attached box is the measured one

    yaw_range: a single pose per position gets a random yaw in [0, yaw_range)
    yaws: the yaws every position is tried with when more than 20 poses are asked for
          (default 7 yaws 0.5 rad apart, i.e. about half a circle)
    height_offset: clearance between the object's bottom and the support surface
    '''
    if yaws is None:
        yaws = [0.5 * step for step in range(7)]
    if number_of_poses > 100:
        min_dist = 0.03
        rospy.logwarn(
            f'number of poses is greater than 100, min_dist will be set to 0.03 instead of desired value of {min_dist}'
        )
    object_list_msg = ObjectList()
    object_list_msg.header.frame_id = frame_id
    x_y_list = []
    place_poses_id = 1
    # Set once the plane holds no further well separated spot: the remaining poses skip the
    # min_dist check instead of each spending the whole attempt budget in vain (25 or 50 poses
    # on a table with room for a handful took minutes, beyond the callers' place timeout).
    separation_exhausted = False
    # The same for every pose; each planning scene query takes seconds over Wi-Fi to the real robot
    place_z = attached_obj_height(support_object, planning_scene, offset=height_offset)
    for _ in range(1, number_of_poses + 1):
        object_pose_msg = ObjectPose()
        object_pose_msg.class_id = object_class
        count = 0
        while 1:
            if place_action_server.is_preempt_requested():
                rospy.logwarn('Preemption requested. Abort place poses generation.')
                return object_list_msg
            candidate_x, candidate_y = sample_in_plane(plane, within)
            if support_object in ignore_min_dist_list:
                rospy.logwarn(f'ignoring min dist param for object: {object_class}')
                break
            if separation_exhausted:
                break
            if well_separated(x_y_list, candidate_x, candidate_y, min_dist=min_dist):
                break
            count += 1
            # Cap the attempts: once the plane holds no more well separated spots every further
            # pose would search in vain (50000 attempts took about 5 s per pose, 2 min per place).
            # 2000 random tries still find a free spot covering 0.5 % of the plane with 99.99 %.
            if count > 2000:
                rospy.logwarn(
                    f'Could not generate poses too much separated from each other, min dist : {min_dist}; '
                    'the remaining poses ignore it'
                )
                separation_exhausted = True
                break
        x_y_list.append([candidate_x, candidate_y])

        object_pose_msg.pose.position.x = candidate_x
        object_pose_msg.pose.position.y = candidate_y

        object_pose_msg.pose.position.z = place_z

        roll = 0.0
        pitch = 0.0
        yaw = round(random.uniform(0.0, yaw_range), 4)
        roll = 0.0 if open_set else resting_roll(object_class)
        if number_of_poses > 20:
            rospy.loginfo(f'trying {len(yaws)} yaws for each pose')
            for yaw in yaws:
                angular_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
                object_pose_msg.pose.orientation.x = angular_q[0]
                object_pose_msg.pose.orientation.y = angular_q[1]
                object_pose_msg.pose.orientation.z = angular_q[2]
                object_pose_msg.pose.orientation.w = angular_q[3]
                object_pose_msg.instance_id = place_poses_id
                object_list_msg.objects.append(copy.deepcopy(object_pose_msg))
                place_poses_id += 1
        else:
            angular_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            object_pose_msg.pose.orientation.x = angular_q[0]
            object_pose_msg.pose.orientation.y = angular_q[1]
            object_pose_msg.pose.orientation.z = angular_q[2]
            object_pose_msg.pose.orientation.w = angular_q[3]
            object_pose_msg.instance_id = place_poses_id
            object_list_msg.objects.append(copy.deepcopy(object_pose_msg))
            place_poses_id += 1
    return object_list_msg


# TODO: consider using shape_msgs/Plane instead of 4 points
def adjust_plane(
    plane: List[Point], x_extend: float = 0.0, y_extend: float = 0.0, x_offset: float = 0.0, y_offset: float = 0.0
) -> List[Point]:
    """
    Adjust the dimensions and position of a plane.

    Parameters
    ----------
    plane : List[Point]
        A list of four points representing a plane.
    x_extend : float, optional
        The amount to increase/decrease all points on the x-axis. Defaults to 0.0.
    y_extend : float, optional
        The amount to increase/decrease all points on the y-axis. Defaults to 0.0.
    x_offset : float, optional
        The amount to offset all points on the x-axis. Defaults to 0.0.
    y_offset : float, optional
        The amount to offset all points on the y-axis. Defaults to 0.0.

    Raises
    ------
    ValueError
        If the plane is not parallel to the XY plane.

    Returns
    -------
    List[Point]
        A list of four points representing the adjusted plane.
    """
    if not len(set([p.z for p in plane])) <= 1:
        raise ValueError("Plane is not parallel to the XY plane")

    plane = [Point(p.x + x_offset, p.y + y_offset, p.z) for p in plane]

    min_x, max_x = min(p.x for p in plane), max(p.x for p in plane)
    min_y, max_y = min(p.y for p in plane), max(p.y for p in plane)

    return [
        Point(min_x - x_extend, min_y - y_extend, plane[0].z),
        Point(max_x + x_extend, min_y - y_extend, plane[0].z),
        Point(max_x + x_extend, max_y + y_extend, plane[0].z),
        Point(min_x - x_extend, max_y + y_extend, plane[0].z),
    ]


def visualize_points(points: List[Point], point_publisher: rospy.Publisher) -> None:
    """Publish points to view them in RViz"""
    for point in points:
        point_publisher.publish(PointStamped(header=Header(frame_id='map'), point=point))
        rospy.sleep(0.5)


def get_obj_from_planning_scene(obj_name: str, planning_scene: PlanningScene) -> CollisionObject:
    """
    Get a Object from a MoveIt PlanningScene by its name.

    Parameters
    ----------
    obj_name : str
        The name of the object to retrieve.
    planning_scene : PlanningScene
        The MoveIt PlanningScene to retrieve the object from.

    Returns
    -------
    CollisionObject
        The CollisionObject with the specified name.

    Raises
    ------
    ValueError
        If the object with the specified name is not in the planning scene.
    """
    if obj_name not in planning_scene.get_known_object_names():
        raise ValueError(f"Object '{obj_name}' not in planning scene")
    return planning_scene.get_objects([obj_name])[obj_name]


def obj_to_plane(support_obj: str, planning_scene: PlanningScene, offset: float = 0.001) -> List[Point]:
    """
    Get a surface plane from an object in the MoveIt planning scene.

    NOTE: Currently only works for boxes

    Parameters
    ----------
    support_obj : str
        The name of the box in the MoveIt planning scene.
    planning_scene : PlanningScene
        The MoveIt planning scene.
    offset : float, optional
        The offset from the MoveIt object and the created plane. Defaults to 0.001.

    Returns
    -------
    List[Point]
        A list of four points (top right, top left, buttom left, buttom right)
        representing the corners of the top surface plane.
    """
    collision_object = get_obj_from_planning_scene(support_obj, planning_scene)

    if len(collision_object.primitives) != 1 or collision_object.primitives[0].type != 1:
        raise ValueError(f"Object '{support_obj}' is not a box")

    half_width = collision_object.primitives[0].dimensions[0] / 2
    half_depth = collision_object.primitives[0].dimensions[1] / 2

    corners = [
        (half_width, half_depth),
        (-half_width, half_depth),
        (-half_width, -half_depth),
        (half_width, -half_depth),
    ]

    half_height = collision_object.primitives[0].dimensions[2] / 2

    return [Point(dx, dy, half_height + offset) for dx, dy in corners]


def attached_obj_height(support_object: str, planning_scene: PlanningScene, offset: float = 0.001) -> float:
    """
    Get the height where an object attached to the gripper needs to have in order to
    be placed correctly on the support object.

    Parameters
    ----------
    support_object : str
        The name of the surface where the attached object needs to be placed on.
        The name must match one in the planning scene.
    planning_scene : PlanningScene
        The planning scene object.
    offset : float, optional
        An offset for the height. by default 0.001.

    Returns
    -------
    float
        The height of the attached object in the global reference frame.
    """

    support_object_collision_obj = get_obj_from_planning_scene(support_object, planning_scene)
    dimension_index = 2

    # the upright classes are rolled: their vertical mesh axis is y
    if any(
        object_class in key
        for object_class in UPRIGHT_ROLL_CLASSES
        for key in planning_scene.get_attached_objects()
    ):
        dimension_index = 1

    half_height_att_obj = (
        list(planning_scene.get_attached_objects().values())[0].object.primitives[0].dimensions[dimension_index] / 2
    )

    # object pose is expressed wrt the table, that's why we need to add only half of the table height
    return half_height_att_obj + support_object_collision_obj.primitives[0].dimensions[2] / 2 + offset


# Example usage
# if __name__ == '__main__':
#     rospy.init_node('plane_visualizer', anonymous=False)
#     support_plane_marker_pub = rospy.Publisher('support_plane_as_marker', Marker, queue_size=1, latch=True)
#     point_pub = rospy.Publisher('plane_points', PointStamped, queue_size=1, latch=False)
#     place_poses_pub = rospy.Publisher('~place_poses', ObjectList, queue_size=1, latch=True)
#     rospy.loginfo('test started')
#     rospy.sleep(0.2)
#
#     support_object = (
#         'table_3'  # the object from which a surface will be generated and later on an object needs to be placed
#     )
#     object_tbp = 'power_drill_with_grip'  # the obj class to be place on a surface
#     plane_1 = obj_to_plane(support_object)
#     # currently the points need to be specified in a specific order (this is a workaround)
#     # the animation helps to make sure the order is correct so that the functions can work correctly
#     animate_plane_points(plane_1, point_pub)
#     plane_1 = reduce_plane_area(plane_1, -0.2)
#     # visualize plane as marker
#     marker_msg = make_plane_marker_msg('map', plane_1)
#     support_plane_marker_pub.publish(marker_msg)
#     object_list_msg = gen_place_poses_from_plane(object_tbp, support_object, plane_1)
#     place_poses_pub.publish(object_list_msg)
#
#     rospy.loginfo('test finished')
#     rospy.sleep(1.0)
