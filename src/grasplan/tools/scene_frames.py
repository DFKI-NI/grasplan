'''
Planning-scene objects and configured boxes in the robot's own map frame.

MoveIt returns the world objects of its planning scene in its model frame (the parent frame of the SRDF's virtual
joint, "map"), while a robot that lives in its own copy of the scene (several robots in one simulator) works in its
own map frame (~global_reference_frame, e.g. robot2/map, with a static transform from map). The geometric checks of
the pick and place nodes compare boxes, grasps and poses that must be in one frame: these helpers express the scene
objects and the configured table boxes in the robot's map frame. With one robot both frames are "map" and nothing
changes.
'''
import copy

import numpy as np
import tf.transformations as tft

# the frame the planning-scene box configs give their tables in: the map of the robot that loads them
CONFIG_MAP_FRAME = 'map'


def boxes_in_robot_map(boxes, map_frame):
    '''
    ~planning_scene_boxes give the tables and walls in the robot's own map (frame_id 'map' in the config, the same
    file for every robot of a multi-robot sim): copies whose 'map' frame is map_frame. Other frames stay as they are.
    '''
    boxes = list(boxes or [])
    if not map_frame or map_frame.lstrip('/') == CONFIG_MAP_FRAME:
        return boxes
    moved = []
    for box in boxes:
        box = dict(box)
        if str(box.get('frame_id', CONFIG_MAP_FRAME)).lstrip('/') == CONFIG_MAP_FRAME:
            box['frame_id'] = map_frame
        moved.append(box)
    return moved


def pose_matrix(pose):
    '''4x4 matrix of a geometry_msgs/Pose; an unset quaternion (all 0) is the identity'''
    o = pose.orientation
    q = [o.x, o.y, o.z, o.w] if (o.x, o.y, o.z, o.w) != (0.0, 0.0, 0.0, 0.0) else [0.0, 0.0, 0.0, 1.0]
    m = tft.quaternion_matrix(q)
    m[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return m


def set_pose_from_matrix(pose, m):
    '''write the 4x4 matrix m into the geometry_msgs/Pose pose'''
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in m[:3, 3])
    q = tft.quaternion_from_matrix(m)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (float(v) for v in q)


def objects_in_frame(objects, frame, lookup):
    '''
    {name: moveit_msgs/CollisionObject} -> the same objects, each expressed in frame.
    lookup(target, source): 4x4 numpy matrix that maps points of source into target, or None when unknown (that
    object is then returned as it is, the callers' frame checks skip it as before). Objects already in frame, or
    without a frame, are returned unchanged; the others are copies (the scene's own messages stay untouched). The
    object pose moves; primitive, mesh and subframe poses are relative to it and stay.
    '''
    if not frame:
        return objects
    frame = frame.lstrip('/')
    transforms = {}
    result = {}
    for name, obj in objects.items():
        source = (obj.header.frame_id or '').lstrip('/')
        if not source or source == frame:
            result[name] = obj
            continue
        if source not in transforms:
            transforms[source] = lookup(frame, source)
        m = transforms[source]
        if m is None:
            result[name] = obj
            continue
        moved = copy.deepcopy(obj)
        set_pose_from_matrix(moved.pose, np.asarray(m) @ pose_matrix(obj.pose))
        moved.header.frame_id = frame
        result[name] = moved
    return result


def tf_matrix(translation, rotation):
    '''4x4 matrix of a (translation xyz, quaternion xyzw) pair, as tf's lookupTransform returns it'''
    m = tft.quaternion_matrix(list(rotation))
    m[:3, 3] = list(translation)
    return m
