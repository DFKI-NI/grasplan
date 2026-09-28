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
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from pose_selector.srv import ClassQuery
from grasplan.tools.common import objectToPick  # name is misleading, in this case we want to insert an object in it
from grasplan.msg import InsertObjectAction, InsertObjectResult
from grasplan.tools.action_client_helper import ActionClientHelper
from grasplan.tools.comfortable_insert import (
    comfortable_insert_candidates,
    free_yaw_insert_candidates,
    matrix_to_pose,
    pose_to_matrix,
)
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
        # insert_orientation (see INSERT_ORIENTATIONS): how the held object is turned over the container; empty
        # means place_same_orientation_as_picked decides (true: as_picked, false: gripper_down); gripper_down
        # applies only to objects up to comfortable_orientation_max_size (longest side, m)
        self.insert_orientation = rospy.get_param('~insert_orientation', '')
        self.place_same_orientation_as_picked = rospy.get_param('~place_same_orientation_as_picked', True)
        self.comfortable_orientation_max_size = rospy.get_param('~comfortable_orientation_max_size', 0.15)
        self.tcp_frame = rospy.get_param('~tcp_frame', 'mobipick/gripper_tcp')
        # free_yaw: order the yaws by the wrist_3 change an IK solution (seeded with the current arm state) needs,
        # instead of by the TCP rotation, which does not predict wrist_3 when the arm has to reconfigure (#106)
        self.insert_sort_by_ik = rospy.get_param('~insert_sort_by_ik', True)
        self.compute_ik_srv = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo(
            f'insert: insert_orientation={self.read_insert_orientation()}, '
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
        if getattr(self.place, 'mtc', None) is not None:
            self.place.mtc.failure_reason, self.place.mtc.executed = '', False  # nothing left over from the previous goal
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
            if getattr(self.place, 'mtc', None) is not None and self.place.mtc.executed:
                break  # the arm moved (maybe the gripper already opened): report instead of retrying (#128)
        if success:
            self.insert_action_server.set_succeeded(InsertObjectResult(success=True))
        elif self.insert_action_server.is_preempt_requested():
            rospy.logwarn("Preemption requested during insert goal processing.")
            self.insert_action_server.set_preempted()
        else:
            reason = self.place.mtc.failure_reason if getattr(self.place, 'mtc', None) is not None else ''
            self.insert_action_server.set_aborted(
                InsertObjectResult(success=False), f'insert failed: {reason}' if reason else 'insert failed'
            )

    # as_picked: the object keeps its grasp orientation (the container's yaw or 8 fixed yaws), can need large
    #     wrist rotations that wind up the arm cable (#103)
    # free_yaw: the object keeps the roll and pitch it was picked with, its yaw about the vertical is chosen for the
    #     least wrist rotation (#106)
    # gripper_down: small objects are released with the gripper pointing straight down and turned as little as
    #     possible, changes the object's roll and pitch after an oblique grasp
    INSERT_ORIENTATIONS = ('as_picked', 'free_yaw', 'gripper_down')

    def read_insert_orientation(self):
        '''
        the insert orientation mode, read on every goal so it can be switched with rosparam set between runs (the
        node is required, restarting it takes the whole bringup down)
        '''
        self.insert_orientation = rospy.get_param('~insert_orientation', self.insert_orientation)
        self.place_same_orientation_as_picked = rospy.get_param(
            '~place_same_orientation_as_picked', self.place_same_orientation_as_picked
        )
        self.comfortable_orientation_max_size = rospy.get_param(
            '~comfortable_orientation_max_size', self.comfortable_orientation_max_size
        )
        if not self.insert_orientation:
            return 'as_picked' if self.place_same_orientation_as_picked else 'gripper_down'
        if self.insert_orientation not in self.INSERT_ORIENTATIONS:
            rospy.logerr(
                f'unknown insert_orientation {self.insert_orientation}, expected one of {self.INSERT_ORIENTATIONS}:'
                ' using as_picked'
            )
            return 'as_picked'
        return self.insert_orientation

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
                # the closed-set untangle detour, or transport for an object picked open-set (PlaceTools.go_before_place)
                self.place.go_before_place(object_to_be_inserted, self.poses_to_go_before_insert)

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
        insert_orientation = self.read_insert_orientation()
        if insert_orientation != 'as_picked':
            place_poses_as_object_list_msg = self.gen_comfortable_insert_poses(
                object_to_be_inserted, object_class_tbi, support_object, support_object_pose, support_obj_height,
                comfortable_max_yaw_offset, free_yaw=insert_orientation == 'free_yaw',
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

        if self.place.use_mtc or action_client.wait_for_server(
            timeout=rospy.Duration(self.place.moveit_action_server_timeout)
        ):
            rospy.loginfo(f'found {INSERT_OBJECT_SERVER_NAME} action server')
            goal = self.place.make_place_goal_msg(
                object_to_be_inserted,
                support_object.get_object_class_and_id_as_string(),
                place_poses_as_object_list_msg,
                use_path_constraints=False,
            )

            rospy.loginfo(f'sending place {object_to_be_inserted} goal to {INSERT_OBJECT_SERVER_NAME} action server')
            rospy.loginfo(f'waiting for result from {INSERT_OBJECT_SERVER_NAME} action server')
            result = self.place.run_place_goal(goal, self.insert_action_server, self.action_client_helper)
            if result is not None:
                if self.insert_action_server.is_preempt_requested():
                    return False

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
        self,
        object_name,
        object_class,
        support_object,
        support_object_pose,
        support_obj_height,
        max_yaw_offset,
        free_yaw=False,
    ):
        '''
        insert poses that need little wrist rotation (see grasplan.tools.comfortable_insert), best first:
        free_yaw: the object keeps the roll and pitch it was picked with and only its yaw is chosen (any size);
        otherwise the gripper points straight down, turned as little as possible from its current rotation.
        None when the object is too big for the gripper-down mode or its geometry is unknown, then the caller
        falls back to the usual insert poses
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
        if not free_yaw and size > self.comfortable_orientation_max_size:
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
        target_xy = (support.position.x, support.position.y)
        support_top_z = support.position.z + support_height / 2.0
        current_tcp_rotation = transform_to_matrix(world_to_tcp)[:3, :3]
        if free_yaw:
            # the roll and pitch the object was picked with: the one the usual insert poses give it (open-set
            # objects were added map-axis-aligned, some known ones lie on their side), their yaw does not matter
            picked_pose = gen_insert_poses_from_obj(object_class, support_object_pose, 0.0).objects[0].pose
            picked_rotation = pose_to_matrix(picked_pose)[:3, :3]
            o = collision_object.pose.orientation
            if not any((o.x, o.y, o.z, o.w)):
                # no object pose: the body frame is the link it hangs on, the picked tilt belongs to the primitive
                picked_rotation = picked_rotation.dot(body_to_primitive[:3, :3].T)
            bodies = free_yaw_insert_candidates(
                current_tcp_rotation,
                tcp_to_body,
                body_to_primitive,
                picked_rotation,
                primitive_type,
                dims,
                target_xy,
                support_top_z,
                support_long_axis_yaw=support_yaw,
                max_yaw_offset=max_yaw_offset,
            )
        else:
            bodies = comfortable_insert_candidates(
                current_tcp_rotation,
                tcp_to_body,
                body_to_primitive,
                primitive_type,
                dims,
                target_xy,
                support_top_z,
                support_long_axis_yaw=support_yaw,
                max_yaw_offset=max_yaw_offset,
            )
        if free_yaw and rospy.get_param('~insert_sort_by_ik', self.insert_sort_by_ik):
            bodies = self.sort_by_wrist_3_change(bodies, tcp_to_body)
        object_list_msg = ObjectList()
        object_list_msg.header.frame_id = self.place.global_reference_frame
        for index, body in enumerate(bodies):
            object_pose_msg = ObjectPose()
            object_pose_msg.class_id = object_class
            object_pose_msg.instance_id = index + 1
            matrix_to_pose(body, object_pose_msg.pose)
            object_list_msg.objects.append(object_pose_msg)
        mode = 'picked roll and pitch, free yaw' if free_yaw else 'gripper straight down'
        rospy.loginfo(
            f'{object_name} ({size:.3f} m): {len(bodies)} comfortable insert poses, {mode}, '
            f'within {math.degrees(max_yaw_offset):.0f} deg of the least wrist rotation'
        )
        return object_list_msg

    def sort_by_wrist_3_change(self, bodies, tcp_to_body):
        '''
        object body poses sorted by how far wrist_3 turns from where it is to reach them (collision-aware IK seeded
        with the current arm state); poses without an IK solution keep their order at the end. MTC samples its own
        IK solutions, so this predicts the wrist_3 it uses rather than fixing it. Unsorted when IK is unavailable.
        '''
        try:
            self.compute_ik_srv.wait_for_service(2.0)
        except rospy.ROSException:
            rospy.logwarn(f'{self.compute_ik_srv.resolved_name} not available: insert poses not sorted by wrist_3')
            return bodies
        current_state = self.place.robot.get_current_state()
        wrist_3_names = [n for n in current_state.joint_state.name if n.endswith('ur5_wrist_3_joint')]
        if not wrist_3_names:
            return bodies
        wrist_3_name = wrist_3_names[0]
        wrist_3_now = current_state.joint_state.position[current_state.joint_state.name.index(wrist_3_name)]
        body_to_tcp = np.linalg.inv(tcp_to_body)
        keyed = []
        for index, body in enumerate(bodies):
            request = GetPositionIKRequest()
            request.ik_request.group_name = self.place.group_name
            request.ik_request.robot_state = current_state
            request.ik_request.avoid_collisions = True
            request.ik_request.ik_link_name = self.tcp_frame
            request.ik_request.pose_stamped.header.frame_id = self.place.global_reference_frame
            matrix_to_pose(body.dot(body_to_tcp), request.ik_request.pose_stamped.pose)
            request.ik_request.timeout = rospy.Duration(0.05)
            try:
                response = self.compute_ik_srv(request)
            except rospy.ServiceException as e:
                rospy.logwarn(f'compute_ik failed ({e}): insert poses not sorted by wrist_3')
                return bodies
            key = (1, float(index), None)
            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                names = response.solution.joint_state.name
                if wrist_3_name in names:
                    wrist_3 = response.solution.joint_state.position[names.index(wrist_3_name)]
                    key = (0, abs(wrist_3 - wrist_3_now), wrist_3)
            keyed.append((key, body))
        keyed.sort(key=lambda k: k[0][:2])
        reachable = [k for k, _ in keyed if k[0] == 0]
        first = ', '.join(f'{math.degrees(k[2]):.0f}' for k in reachable[:6])
        rospy.loginfo(
            f'insert poses by wrist_3 change (now {math.degrees(wrist_3_now):.0f} deg): {len(reachable)} of '
            f'{len(bodies)} with IK, wrist_3 {first}{" ..." if len(reachable) > 6 else ""}'
        )
        return [body for _, body in keyed]

    def start_insert_node(self):
        rospy.loginfo('ready to insert objects')
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('insert_object_node', anonymous=False)
    insert = InsertTools()
    insert.start_insert_node()
