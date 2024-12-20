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

import rospy
import actionlib
from grasplan.place import PlaceTools
from grasplan.tools.common import separate_object_class_from_id
from grasplan.tools.support_plane_tools import gen_insert_poses_from_obj, compute_object_height_for_insertion
from grasplan.tools.moveit_errors import print_moveit_error
from object_pose_msgs.msg import ObjectList
from moveit_msgs.msg import PlaceAction
from moveit_msgs.msg import MoveItErrorCodes
from std_srvs.srv import Empty
from pose_selector.srv import ClassQuery
from grasplan.tools.common import objectToPick  # name is misleading, in this case we want to insert an object in it
from grasplan.msg import InsertObjectAction, InsertObjectResult
from grasplan.tools.action_client_helper import ActionClientHelper


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

    def insert_object(
        self,
        support_object_name_as_string,
        observe_before_insert=False,
        same_orientation_as_support_obj=False,
        override_disentangle_dont_doit=False,
        override_observe_before_place_dont_doit=False,
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

        action_client = actionlib.SimpleActionClient(INSERT_OBJECT_SERVER_NAME, PlaceAction)
        rospy.loginfo(f'sending insert command as a place goal to {INSERT_OBJECT_SERVER_NAME} action server')

        # get object position [x, y] -> without orientation for now
        support_object_pose = self.get_support_object_pose(support_object)
        if support_object_pose is None:
            return False

        place_poses_as_object_list_msg = gen_insert_poses_from_obj(
            object_class_tbi,
            support_object_pose,
            compute_object_height_for_insertion(object_class_tbi, support_object.obj_class),
            frame_id=self.place.global_reference_frame,
            same_orientation_as_support_obj=same_orientation_as_support_obj,
        )
        # send places poses to place pose selector for visualization purposes
        self.insert_poses_pub.publish(place_poses_as_object_list_msg)

        # clear octomap before placing, this is experimental and not sure is needed
        rospy.loginfo('clearing octomap')
        rospy.ServiceProxy('clear_octomap', Empty)()

        if self.insert_action_server.is_preempt_requested():
            return False

        if action_client.wait_for_server(timeout=rospy.Duration.from_sec(2.0)):
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

    def start_insert_node(self):
        rospy.loginfo('ready to insert objects')
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('insert_object_node', anonymous=False)
    insert = InsertTools()
    insert.start_insert_node()
