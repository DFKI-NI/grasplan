#!/usr/bin/env python3

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

'''
example on how to insert an object using grasplan and moveit
'''

import math

import numpy as np
import rospy
import actionlib
import tf2_ros
import tf.transformations as tft
from grasplan.place import PlaceTools
from grasplan.tools.common import separate_object_class_from_id
from grasplan.tools.support_plane_tools import (
    gen_insert_poses_from_obj,
    compute_object_height_for_insertion,
    OBJECT_HEIGHTS,
)
from grasplan.tools.moveit_errors import print_moveit_error
from object_pose_msgs.msg import ObjectList
from moveit_msgs.msg import MoveItErrorCodes
from pose_selector.srv import ClassQuery
from grasplan.tools.common import objectToPick  # name is misleading, in this case we want to insert an object in it
from grasplan.msg import InsertObjectAction, InsertObjectResult
from grasplan.tools.action_client_helper import ActionClientHelper
from grasplan.tools.comfortable_insert import comfortable_insert_candidates, pose_to_matrix, matrix_to_pose
from object_pose_msgs.msg import ObjectPose
from shape_msgs.msg import SolidPrimitive


class InsertTools:
    def __init__(self):
        # create instance of place
        self.place = PlaceTools(
            action_server_required=False
        )  # we will advertise our own action lib server for insertion

        # parameters
        pick_pose_selector_class_query_srv_name = rospy.get_param(
            '~pick_pose_selector_class_query_srv_name', '/pose_selector_class_query'
        )
        self.disentangle_required = rospy.get_param('~disentangle_required', False)
        self.poses_to_go_before_insert = rospy.get_param('~poses_to_go_before_insert', [])
        # False: small objects are released with the gripper pointing straight down and turned as little as
        # possible from where it is, instead of in the orientation they were grasped in (which can need large
        # wrist rotations that wind up the arm cable); objects bigger than comfortable_orientation_max_size
        # (longest side, m) always keep their orientation
        self.place_same_orientation_as_picked = rospy.get_param('~place_same_orientation_as_picked', True)
        self.comfortable_orientation_max_size = rospy.get_param('~comfortable_orientation_max_size', 0.15)
        self.tcp_frame = rospy.get_param('~tcp_frame', 'mobipick/gripper_tcp')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo(
            f'insert: place_same_orientation_as_picked={self.place_same_orientation_as_picked}, '
            f'comfortable_orientation_max_size={self.comfortable_orientation_max_size} m'
        )

        rospy.loginfo(f'waiting for pose selector services: {pick_pose_selector_class_query_srv_name}')
        rospy.wait_for_service(pick_pose_selector_class_query_srv_name, 30.0)
        self.pick_pose_selector_class_query_srv = rospy.ServiceProxy(
            pick_pose_selector_class_query_srv_name, ClassQuery
        )
        self.insert_poses_pub = rospy.Publisher('place_object_node/place_poses', ObjectList, queue_size=50)

        self.insert_action_server = actionlib.SimpleActionServer(
            'insert_object', InsertObjectAction, self.insert_obj_action_callback, False
        )
        # prepare fallback option because moveit pickup action server ignores preemption requests
        # create joint controller cancellers for both arm and gripper
        ns = rospy.get_namespace().strip('/')  # programatically get robot namespace
        self.action_client_helper = ActionClientHelper(
            ns, self.insert_action_server, controller_names=['arm', 'gripper']
        )
        self.insert_action_server.start()

        # give some time for publishers and Subscribers to register
        rospy.sleep(0.2)

    def insert_obj_action_callback(self, goal):
        success = False
        for i in range(2):  # 0, 1 = 2 attemps
            if self.insert_action_server.is_preempt_requested():
                break
            rospy.loginfo(f'Insert object: attempt number {i + 1}')
            if i == 0:  # first try, optimistic, keep same orientation as support object
                # disentangle cable only on first attempt
                override_disentangle_dont_doit = False
                override_observe_before_place_dont_doit = False
                same_orientation_as_support_obj = True
                # do not disentangle if we dont go to observe arm pose
                if not goal.observe_before_insert:
                    override_disentangle_dont_doit = True
            else:  # second try, generate 360 degree orientations
                override_disentangle_dont_doit = True
                override_observe_before_place_dont_doit = True
                same_orientation_as_support_obj = False
                # do not disentangle if we dont go to observe arm pose
                if not goal.observe_before_insert:
                    override_disentangle_dont_doit = True
            if self.insert_object(
                goal.support_surface_name,
                observe_before_insert=goal.observe_before_insert,
                same_orientation_as_support_obj=same_orientation_as_support_obj,
                override_disentangle_dont_doit=override_disentangle_dont_doit,
                override_observe_before_place_dont_doit=override_observe_before_place_dont_doit,
                # comfortable orientation: all rotations about the vertical, sorted by the least wrist rotation (in
                # the sim, +-90 degrees alone often had no reachable pose and cost a retry)
                comfortable_max_yaw_offset=math.radians(180.0),
            ):
                success = True
                break
        if success:
            self.insert_action_server.set_succeeded(InsertObjectResult(success=True))
        elif self.insert_action_server.is_preempt_requested():
            rospy.logwarn("Preemption requested during insert goal processing.")
            self.insert_action_server.set_preempted()
        else:
            self.insert_action_server.set_aborted(InsertObjectResult(success=False))

    def get_support_object_pose(self, support_object):
        '''
        get object position from pose selector
        '''
        rospy.loginfo(f'getting object {support_object.get_object_class_and_id_as_string()} position')
        # query pose selector
        resp = self.pick_pose_selector_class_query_srv(support_object.obj_class)
        if len(resp.poses) == 0:
            rospy.logerr(
                f'Object of class {support_object.obj_class} was not perceived, therefore its pose is not available'
                ' and cannot be picked'
            )
            return None
        # at least one object of the same class as the object we want to pick was perceived, continue
        for pose in resp.poses:
            if pose.instance_id == support_object.id:
                rospy.loginfo(
                    f'success at getting support object pose: {support_object.get_object_class_and_id_as_string()}'
                )
                return pose  # of type object_pose_msgs/ObjectPose.msg
        rospy.logerr(
            f'At least one object of the class {support_object.obj_class} was perceived but is not the one you want,'
            f' with id: {support_object.id}'
        )
        return None

    def get_object_heights_for_insertion(
        self, object_to_be_inserted, object_class_tbi, support_object, support_object_pose
    ):
        '''
        heights of the object to insert and of the support object for objects that are not in the insertion height
        table (open-set objects picked through AnyGrasp): the attached collision box was added to the planning scene
        map-axis-aligned in the object's resting pose, so its z dimension is the resting height; the support object
        height comes from the pose selector box. Returns (ok, object height, support height); a height is None for
        objects in the table so compute_object_height_for_insertion keeps using it.
        '''
        object_tbi_height = None
        support_obj_height = None
        if object_class_tbi not in OBJECT_HEIGHTS:
            attached_object = self.place.scene.get_attached_objects([object_to_be_inserted]).get(object_to_be_inserted)
            if attached_object is None or len(attached_object.object.primitives) == 0:
                rospy.logerr(
                    f'{object_to_be_inserted} is not in the insertion height table and has no collision primitive'
                    ' attached to the gripper, cannot compute its height'
                )
                return False, None, None
            object_tbi_height = attached_object.object.primitives[0].dimensions[2]
            rospy.loginfo(
                f'{object_to_be_inserted} height taken from its attached collision box: {object_tbi_height:.3f} m'
            )
        if support_object.obj_class not in OBJECT_HEIGHTS:
            support_obj_height = support_object_pose.size.z
            if support_obj_height <= 0.0:
                rospy.logerr(
                    f'{support_object.get_object_class_and_id_as_string()} is not in the insertion height table and'
                    ' the pose selector reports no size for it, cannot compute its height'
                )
                return False, None, None
            rospy.loginfo(
                f'{support_object.get_object_class_and_id_as_string()} height taken from the pose selector:'
                f' {support_obj_height:.3f} m'
            )
        return True, object_tbi_height, support_obj_height

    def insert_object(
        self,
        support_object_name_as_string,
        observe_before_insert=False,
        same_orientation_as_support_obj=False,
        override_disentangle_dont_doit=False,
        override_observe_before_place_dont_doit=False,
        comfortable_max_yaw_offset=math.pi,
    ):
        '''
        use place functionality by creating 1 pose above the support_object_name_as_string object for now
        NOTE: support_object_name_as_string has an id
        '''
        assert isinstance(observe_before_insert, bool)
        assert isinstance(support_object_name_as_string, str)

        INSERT_OBJECT_SERVER_NAME = 'place'  # we use the same action server as in place object

        support_object = objectToPick(support_object_name_as_string)

        if len(self.place.scene.get_attached_objects().keys()) == 0:
            rospy.logerr("the robot is not currently holding any object, can't insert")
            return False

        # deduce object_to_be_inserted by querying which object the gripper currently has attached to its gripper
        object_to_be_inserted = list(self.place.scene.get_attached_objects().keys())[0]
        object_class_tbi = separate_object_class_from_id(object_to_be_inserted)[0]

        rospy.loginfo(
            f'received request to insert the object I am currently holding ({object_to_be_inserted})\
                        into {support_object.get_object_class_and_id_as_string()}'
        )

        # clear pose selector before starting to place in case some data is left over from previous runs
        self.place.place_pose_selector_clear_srv()

        if not override_observe_before_place_dont_doit:
            if observe_before_insert:
                # optionally find the support object: look at table, update planning scene
                self.place.move_arm_to_posture(self.place.arm_pose_with_objs_in_fov)
                # activate pick pose selector to observe table
                self.place.activate_pick_pose_selector_srv(True)
                rospy.loginfo('sleeping for 5.0 seconds')
                rospy.sleep(5.0)  # give some time to observe
                self.place.activate_pick_pose_selector_srv(False)
                self.place.add_objs_to_planning_scene()

        # allow disentangle to happen only on first place attempt, no need to do it every time
        if not override_disentangle_dont_doit:
            # go to intermediate arm poses if needed to disentangle arm cable
            if self.disentangle_required:
                for arm_pose in self.poses_to_go_before_insert:
                    rospy.loginfo(f'going to intermediate arm pose {arm_pose} to disentangle cable')
                    self.place.move_arm_to_posture(arm_pose)

        action_client = self.place.place_action_client  # same 'place' server, connected once at startup
        rospy.loginfo(f'sending insert command as a place goal to {INSERT_OBJECT_SERVER_NAME} action server')

        # get object position [x, y] -> without orientation for now
        support_object_pose = self.get_support_object_pose(support_object)
        if support_object_pose is None:
            return False

        heights_ok, object_tbi_height, support_obj_height = self.get_object_heights_for_insertion(
            object_to_be_inserted, object_class_tbi, support_object, support_object_pose
        )
        if not heights_ok:
            return False

        place_poses_as_object_list_msg = None
        # read per goal so it can be switched with rosparam set between runs (the node is required, restarting it
        # takes the whole bringup down)
        self.place_same_orientation_as_picked = rospy.get_param(
            '~place_same_orientation_as_picked', self.place_same_orientation_as_picked
        )
        self.comfortable_orientation_max_size = rospy.get_param(
            '~comfortable_orientation_max_size', self.comfortable_orientation_max_size
        )
        if not self.place_same_orientation_as_picked:
            place_poses_as_object_list_msg = self.gen_comfortable_insert_poses(
                object_to_be_inserted, object_class_tbi, support_object, support_object_pose, support_obj_height,
                comfortable_max_yaw_offset,
            )
        if place_poses_as_object_list_msg is None:
            place_poses_as_object_list_msg = gen_insert_poses_from_obj(
                object_class_tbi,
                support_object_pose,
                compute_object_height_for_insertion(
                    object_class_tbi,
                    support_object.obj_class,
                    object_tbi_height=object_tbi_height,
                    support_obj_height=support_obj_height,
                ),
                frame_id=self.place.global_reference_frame,
                same_orientation_as_support_obj=same_orientation_as_support_obj,
            )
        if len(place_poses_as_object_list_msg.objects) == 0:
            rospy.logerr(f'no insert pose for {object_to_be_inserted} in {support_object_name_as_string}')
            return False
        # send places poses to place pose selector for visualization purposes
        self.insert_poses_pub.publish(place_poses_as_object_list_msg)

        # Insertion shares PlaceTools' opt-in switch.  By default the octomap
        # stays intact so obstacles outside the current camera view are kept.
        if self.place.clear_octomap_flag:
            self.place.clear_octomap()

        if self.insert_action_server.is_preempt_requested():
            return False

        if action_client.wait_for_server(timeout=rospy.Duration(self.place.moveit_action_server_timeout)):
            rospy.loginfo(f'found {INSERT_OBJECT_SERVER_NAME} action server')
            goal = self.place.make_place_goal_msg(
                object_to_be_inserted,
                support_object.get_object_class_and_id_as_string(),
                place_poses_as_object_list_msg,
                use_path_constraints=False,
            )

            rospy.loginfo(f'sending place {object_to_be_inserted} goal to {INSERT_OBJECT_SERVER_NAME} action server')
            rospy.loginfo(f'waiting for result from {INSERT_OBJECT_SERVER_NAME} action server')
            if self.action_client_helper.send_goal_to_rogue_server_and_wait(goal, action_client, patience_timeout=0.1):
                if self.insert_action_server.is_preempt_requested():
                    return False

                result = action_client.get_result()

                # handle moveit pick result
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo('Successfully inserted object')
                    self.place.place_pose_selector_clear_srv()
                    # clear possible place poses markers in rviz
                    self.place.clear_place_poses_markers()
                    return True
                else:
                    rospy.logerr('insert object failed')
                    self.place.place_pose_selector_clear_srv()
                    # clear possible place poses markers in rviz
                    # self.place.clear_place_poses_markers() # leave markers for debugging if failed to insert
                    print_moveit_error(result.error_code.val)
                return False
        else:
            rospy.logerr(f'action server {INSERT_OBJECT_SERVER_NAME} not available (we use it for insertion)')
            return False
        return False

    def gen_comfortable_insert_poses(
        self, object_name, object_class, support_object, support_object_pose, support_obj_height, max_yaw_offset
    ):
        '''
        insert poses with the gripper pointing straight down and turned as little as possible from its current
        rotation (see grasplan.tools.comfortable_insert); None when the object is too big (keeps the grasp
        orientation) or its geometry is unknown, then the caller falls back to the usual insert poses
        '''
        attached = self.place.scene.get_attached_objects([object_name]).get(object_name)
        if attached is None or len(attached.object.primitives) == 0:
            rospy.logwarn(f'{object_name} has no attached collision primitive, using the grasp orientation')
            return None
        collision_object = attached.object
        primitive = collision_object.primitives[0]
        dims = list(primitive.dimensions)
        if primitive.type == SolidPrimitive.BOX:
            primitive_type, size = 'box', max(dims)
        elif primitive.type == SolidPrimitive.CYLINDER:
            primitive_type, size = 'cylinder', max(dims[0], 2.0 * dims[1])
        elif primitive.type == SolidPrimitive.SPHERE:
            primitive_type, size = 'sphere', 2.0 * dims[0]
        else:
            rospy.logwarn(f'{object_name}: unsupported primitive type {primitive.type}, using the grasp orientation')
            return None
        if size > self.comfortable_orientation_max_size:
            rospy.loginfo(
                f'{object_name} is {size:.3f} m long (> {self.comfortable_orientation_max_size} m): '
                'inserting it in the orientation it was grasped in'
            )
            return None
        try:
            tcp_to_header = self.tf_buffer.lookup_transform(
                self.tcp_frame, collision_object.header.frame_id, rospy.Time(0), rospy.Duration(2.0)
            )
            world_to_tcp = self.tf_buffer.lookup_transform(
                self.place.global_reference_frame, self.tcp_frame, rospy.Time(0), rospy.Duration(2.0)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f'no transform for the comfortable insert ({e}), using the grasp orientation')
            return None

        def transform_to_matrix(t):
            r, p = t.transform.rotation, t.transform.translation
            m = tft.quaternion_matrix([r.x, r.y, r.z, r.w])
            m[:3, 3] = [p.x, p.y, p.z]
            return m

        tcp_to_body = transform_to_matrix(tcp_to_header).dot(pose_to_matrix(collision_object.pose))
        body_to_primitive = (
            pose_to_matrix(collision_object.primitive_poses[0])
            if len(collision_object.primitive_poses) > 0
            else np.eye(4)
        )
        support_height = support_obj_height if support_obj_height is not None else OBJECT_HEIGHTS.get(
            support_object.obj_class, 0.0
        )
        support = support_object_pose.pose
        support_yaw = math.atan2(
            2.0 * (support.orientation.w * support.orientation.z + support.orientation.x * support.orientation.y),
            1.0 - 2.0 * (support.orientation.y ** 2 + support.orientation.z ** 2),
        )
        if support_object_pose.size.y > support_object_pose.size.x:
            support_yaw += math.pi / 2.0
        bodies = comfortable_insert_candidates(
            transform_to_matrix(world_to_tcp)[:3, :3],
            tcp_to_body,
            body_to_primitive,
            primitive_type,
            dims,
            (support.position.x, support.position.y),
            support.position.z + support_height / 2.0,
            support_long_axis_yaw=support_yaw,
            max_yaw_offset=max_yaw_offset,
        )
        object_list_msg = ObjectList()
        object_list_msg.header.frame_id = self.place.global_reference_frame
        for index, body in enumerate(bodies):
            object_pose_msg = ObjectPose()
            object_pose_msg.class_id = object_class
            object_pose_msg.instance_id = index + 1
            matrix_to_pose(body, object_pose_msg.pose)
            object_list_msg.objects.append(object_pose_msg)
        rospy.loginfo(
            f'{object_name} ({size:.3f} m): {len(bodies)} comfortable insert poses, gripper straight down, '
            f'within {math.degrees(max_yaw_offset):.0f} deg of the current wrist rotation'
        )
        return object_list_msg

    def start_insert_node(self):
        rospy.loginfo('ready to insert objects')
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('insert_object_node', anonymous=False)
    insert = InsertTools()
    insert.start_insert_node()
