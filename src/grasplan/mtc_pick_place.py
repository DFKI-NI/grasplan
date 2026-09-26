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
MoveIt Task Constructor (MTC) replacement for move_group's pickup and place actions (the old pick n place
pipeline). It takes the same goals grasplan sends to those actions (moveit_msgs/PickupGoal with a list of
moveit_msgs/Grasp, moveit_msgs/PlaceGoal with a list of moveit_msgs/PlaceLocation) and returns the same
results, so pick, place and insert only switch the backend (parameter ~use_mtc).

Candidates are tried one after the other in the order given (grasplan passes them best first), each one
planned as a whole MTC task before anything moves:

    pick:  current state, allow contacts, connect, [approach, grasp pose IK, close, attach, lift]
    place: current state, allow contacts, connect, [lower, place pose IK, open, detach, retreat]

The Mobipick SRDF gripper group has no joints and its GripperCommand driver takes the opening in metres,
so MTC cannot plan the gripper. MTC plans the arm and the planning scene changes, and the solution is
executed here: arm segments through move_group's execute_trajectory action, the gripper steps through
the GripperCommand action with the metre values of the goal, attach and detach through
apply_planning_scene. This also makes the execution preemptible, which the pickup action was not.
'''

import copy
import time

import numpy as np
import rospy
import actionlib
import tf2_ros
import tf.transformations as tft

from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from geometry_msgs.msg import PoseStamped, Vector3Stamped, Vector3
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    ExecuteTrajectoryAction,
    ExecuteTrajectoryGoal,
    MoveItErrorCodes,
    PickupResult,
    PlaceResult,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPlanningSceneRequest
from moveit.task_constructor import core, stages

# the name MoveIt gives the octomap inside the planning scene (planning_scene::PlanningScene::OCTOMAP_NS)
OCTOMAP_COLLISION_NAME = '<octomap>'

# what the executor does with each MTC sub trajectory, in the order the leaf stages were added
ARM, GRIPPER, ATTACH, DETACH, NOOP = 'arm', 'gripper', 'attach', 'detach', 'noop'


def pose_to_matrix(pose):
    q = pose.orientation
    matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def matrix_to_pose(matrix, pose):
    pose.position.x, pose.position.y, pose.position.z = matrix[:3, 3]
    q = tft.quaternion_from_matrix(matrix)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


def transform_to_matrix(transform):
    t, r = transform.transform.translation, transform.transform.rotation
    matrix = tft.quaternion_matrix([r.x, r.y, r.z, r.w])
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


class MtcPickPlace:
    def __init__(self, arm_group, eef_link, gripper_links, planning_frame, is_preempt_requested):
        '''
        arm_group: MoveIt group that moves the gripper (the chain ending in eef_link)
        eef_link: link whose pose moveit_msgs/Grasp.grasp_pose describes (the arm's end effector link)
        gripper_links: links allowed to touch the object, the support surface contact and the octomap
        planning_frame: frame the MTC stages are given poses and directions in
        is_preempt_requested: callable, polled while planning candidates and while executing
        '''
        self.arm_group = arm_group
        self.eef_link = eef_link
        self.gripper_links = list(gripper_links)
        self.planning_frame = planning_frame
        self.is_preempt_requested = is_preempt_requested

        self.connect_timeout = rospy.get_param('~mtc_connect_timeout', 5.0)
        self.planner_id = rospy.get_param('~mtc_planner_id', 'RRTConnect')
        self.max_ik_solutions = int(rospy.get_param('~mtc_max_ik_solutions', 8))
        self.cartesian_step_size = rospy.get_param('~mtc_cartesian_step_size', 0.005)
        self.velocity_scaling = rospy.get_param('~mtc_max_velocity_scaling_factor', 1.0)
        self.acceleration_scaling = rospy.get_param('~mtc_max_acceleration_scaling_factor', 1.0)
        # the open-set octomap has voxels on the target itself; like the pick pipeline workaround in pick.py the
        # gripper and the held object may touch it, the rest of the robot may not
        self.allow_octomap_contact = rospy.get_param('~allow_octomap_contact_during_pick', True)
        self.gripper_action_name = rospy.get_param('~gripper_action_name', '/mobipick/gripper_hw')
        self.gripper_action_timeout = rospy.get_param('~mtc_gripper_action_timeout', 10.0)
        self.relaxed_retreat_min_distance = rospy.get_param('~mtc_relaxed_retreat_min_distance', 0.05)
        self.retreat_failed = False
        server_timeout = rospy.get_param('~moveit_action_startup_timeout', 60.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.execute_client = actionlib.SimpleActionClient('execute_trajectory', ExecuteTrajectoryAction)
        self.gripper_client = actionlib.SimpleActionClient(self.gripper_action_name, GripperCommandAction)
        rospy.loginfo(f'mtc: waiting for {rospy.resolve_name("execute_trajectory")} and {self.gripper_action_name}')
        if not self.execute_client.wait_for_server(rospy.Duration(server_timeout)):
            raise RuntimeError(f'{rospy.resolve_name("execute_trajectory")} action server not found')
        rospy.wait_for_service('apply_planning_scene', server_timeout)
        rospy.wait_for_service('get_planning_scene', server_timeout)
        self.apply_planning_scene_srv = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
        self.get_planning_scene_srv = rospy.ServiceProxy('get_planning_scene', GetPlanningScene)

        # one task, reused for every candidate: the pipeline planner keeps the robot model of the first task it
        # plans for and refuses the model a new Task would load
        self.task = core.Task()
        self.task.loadRobotModel()
        self.cartesian = core.CartesianPath()
        self.cartesian.step_size = self.cartesian_step_size
        self.cartesian.min_fraction = 1.0  # min/max distance of MoveRelative decide how far is enough
        self.cartesian.max_velocity_scaling_factor = self.velocity_scaling
        self.cartesian.max_acceleration_scaling_factor = self.acceleration_scaling
        self.pipeline = core.PipelinePlanner()
        self.pipeline.planner = self.planner_id
        self.pipeline.max_velocity_scaling_factor = self.velocity_scaling
        self.pipeline.max_acceleration_scaling_factor = self.acceleration_scaling
        rospy.loginfo(f'mtc: ready (planner {self.planner_id}, connect timeout {self.connect_timeout} s)')

    # ------------------------------------------------------------------ public api

    def pickup(self, goal):
        '''plan and execute a moveit_msgs/PickupGoal with MTC, trying its grasps in order; returns PickupResult'''
        result = PickupResult()
        result.error_code.val = MoveItErrorCodes.PLANNING_FAILED
        touch = self.gripper_links + [goal.target_name]
        for index, grasp in enumerate(goal.possible_grasps):
            if self.is_preempt_requested():
                result.error_code.val = MoveItErrorCodes.PREEMPTED
                return result
            try:
                task, roles = self.make_pick_task(goal, grasp)
            except (tf2_ros.TransformException, ValueError) as e:
                rospy.logwarn(f'mtc: grasp {grasp.id} skipped: {e}')
                continue
            if not self.plan(task, f'grasp {grasp.id} ({index + 1}/{len(goal.possible_grasps)})'):
                continue
            # the pre grasp posture (open) is not a stage, it is sent before the first arm motion
            code = self.execute(task, roles, [grasp.pre_grasp_posture], grasp.grasp_posture,
                                attach=(goal.target_name, touch))
            result.error_code.val = code
            if code == MoveItErrorCodes.SUCCESS:
                result.grasp = grasp
            return result  # something moved: never try another grasp from a changed state
        rospy.logerr(f'mtc: none of the {len(goal.possible_grasps)} grasps of {goal.target_name} could be planned')
        return result

    def place(self, goal):
        '''
        plan and execute a moveit_msgs/PlaceGoal with MTC, trying its locations in order; returns PlaceResult.
        When no location can be planned but some only failed at the retreat (the gripper opened, then could not
        back off the full min_distance, e.g. its camera touches the box rim), the locations are tried again
        accepting a retreat of at least ~mtc_relaxed_retreat_min_distance: the object is already released there
        and the next arm motion is planned collision free anyway.
        '''
        result = PlaceResult()
        result.error_code.val = MoveItErrorCodes.PLANNING_FAILED
        try:
            eef_to_object = self.eef_to_attached_object(goal.attached_object_name)
        except (tf2_ros.TransformException, ValueError) as e:
            rospy.logerr(f'mtc: cannot place {goal.attached_object_name}: {e}')
            return result
        for retreat_min_distance in (None, self.relaxed_retreat_min_distance):
            if retreat_min_distance is not None:
                if not self.retreat_failed:
                    break
                rospy.logwarn(f'mtc: retrying the place locations accepting a retreat of {retreat_min_distance:.2f} m')
            self.retreat_failed = False
            for index, location in enumerate(goal.place_locations):
                if self.is_preempt_requested():
                    result.error_code.val = MoveItErrorCodes.PREEMPTED
                    return result
                try:
                    task, roles = self.make_place_task(goal, location, eef_to_object, retreat_min_distance)
                except (tf2_ros.TransformException, ValueError) as e:
                    rospy.logwarn(f'mtc: place location {location.id} skipped: {e}')
                    continue
                if not self.plan(task, f'place location {location.id} ({index + 1}/{len(goal.place_locations)})'):
                    self.retreat_failed |= self.only_retreat_failed(task)
                    continue
                code = self.execute(task, roles, [], location.post_place_posture,
                                    detach=goal.attached_object_name)
                result.error_code.val = code
                if code == MoveItErrorCodes.SUCCESS:
                    result.place_location = location
                return result
        rospy.logerr(f'mtc: none of the {len(goal.place_locations)} place locations could be planned')
        return result

    # ------------------------------------------------------------------ task construction

    def make_pick_task(self, goal, grasp):
        obj = goal.target_name
        task, roles = self.start_task(f'pick {obj} {grasp.id}')
        touch = self.gripper_links + [obj]

        allow = stages.ModifyPlanningScene('allow gripper contacts')
        allow.allowCollisions(obj, self.gripper_links, True)
        if self.allow_octomap_contact:
            allow.allowCollisions(OCTOMAP_COLLISION_NAME, touch, True)
        task.add(allow)
        roles.append(NOOP)

        task.add(self.connect('move to pregrasp'))
        roles.append(ARM)

        grasp_stages = core.SerialContainer('grasp')
        grasp_stages.add(self.move_relative('approach', grasp.pre_grasp_approach))
        roles.append(ARM)
        grasp_stages.add(self.compute_ik('grasp pose', self.to_planning_frame(grasp.grasp_pose), task['allow gripper contacts']))
        roles.append(NOOP)
        grasp_stages.add(stages.ModifyPlanningScene('close gripper'))
        roles.append(GRIPPER)
        attach = stages.ModifyPlanningScene('attach object')
        attach.attachObject(obj, self.eef_link)
        if goal.support_surface_name:
            attach.allowCollisions(obj, [goal.support_surface_name], True)
        grasp_stages.add(attach)
        roles.append(ATTACH)
        grasp_stages.add(self.move_relative('lift', grasp.post_grasp_retreat))
        roles.append(ARM)
        task.add(grasp_stages)
        return task, roles

    def make_place_task(self, goal, location, eef_to_object, retreat_min_distance=None):
        obj = goal.attached_object_name
        task, roles = self.start_task(f'place {obj} {location.id}')

        allow = stages.ModifyPlanningScene('allow object contacts')
        if goal.support_surface_name:
            allow.allowCollisions(obj, [goal.support_surface_name], True)
            if goal.allow_gripper_support_collision:
                allow.allowCollisions(goal.support_surface_name, self.gripper_links, True)
        if self.allow_octomap_contact:
            allow.allowCollisions(OCTOMAP_COLLISION_NAME, self.gripper_links + [obj], True)
        task.add(allow)
        roles.append(NOOP)

        task.add(self.connect('move to preplace'))
        roles.append(ARM)

        # MoveIt's place gives the pose of the object (place_eef false); MTC needs the gripper pose
        place_pose = self.to_planning_frame(location.place_pose)
        if goal.place_eef:
            eef_pose = place_pose
        else:
            eef_pose = PoseStamped()
            eef_pose.header.frame_id = self.planning_frame
            matrix_to_pose(pose_to_matrix(place_pose.pose).dot(np.linalg.inv(eef_to_object)), eef_pose.pose)

        place_stages = core.SerialContainer('place')
        place_stages.add(self.move_relative('lower', location.pre_place_approach))
        roles.append(ARM)
        place_stages.add(self.compute_ik('place pose', eef_pose, task['allow object contacts']))
        roles.append(NOOP)
        place_stages.add(stages.ModifyPlanningScene('open gripper'))
        roles.append(GRIPPER)
        detach = stages.ModifyPlanningScene('detach object')
        detach.detachObject(obj, self.eef_link)
        detach.allowCollisions(obj, self.gripper_links, True)
        place_stages.add(detach)
        roles.append(DETACH)
        retreat = copy.deepcopy(location.post_place_retreat)
        if retreat_min_distance is not None:
            retreat.min_distance = min(retreat.min_distance, retreat_min_distance)
        place_stages.add(self.move_relative('retreat', retreat))
        roles.append(ARM)
        task.add(place_stages)
        return task, roles

    def start_task(self, name):
        task = self.task
        task.clear()
        task.name = name
        task.add(stages.CurrentState('current state'))
        return task, [NOOP]

    def connect(self, name):
        connect = stages.Connect(name, [(self.arm_group, self.pipeline)])
        connect.timeout = self.connect_timeout
        return connect

    def move_relative(self, name, gripper_translation):
        '''MoveRelative along a moveit_msgs/GripperTranslation (direction, min and desired distance)'''
        move = stages.MoveRelative(name, self.cartesian)
        move.group = self.arm_group
        move.ik_frame = PoseStamped(header=rospy.Header(frame_id=self.eef_link))
        move.min_distance = gripper_translation.min_distance
        move.max_distance = gripper_translation.desired_distance
        move.setDirection(self.direction_in_planning_frame(gripper_translation.direction))
        return move

    def compute_ik(self, name, pose, monitored_stage):
        generator = stages.GeneratePose(name + ' generator')
        generator.pose = pose
        generator.setMonitoredStage(monitored_stage)
        ik = stages.ComputeIK(name, generator)
        ik.group = self.arm_group
        ik.ik_frame = PoseStamped(header=rospy.Header(frame_id=self.eef_link))
        ik.max_ik_solutions = self.max_ik_solutions
        ik.properties.configureInitFrom(core.Stage.PropertyInitializerSource.INTERFACE, ['target_pose'])
        return ik

    # ------------------------------------------------------------------ frames

    def to_planning_frame(self, pose_stamped):
        if pose_stamped.header.frame_id in ('', self.planning_frame):
            pose = copy.deepcopy(pose_stamped)
            pose.header.frame_id = self.planning_frame
            return pose
        transform = self.tf_buffer.lookup_transform(
            self.planning_frame, pose_stamped.header.frame_id, rospy.Time(0), rospy.Duration(2.0)
        )
        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        matrix_to_pose(transform_to_matrix(transform).dot(pose_to_matrix(pose_stamped.pose)), pose.pose)
        return pose

    def direction_in_planning_frame(self, direction):
        '''
        a direction given in the end effector link stays local to it (MTC then moves along the link's axis),
        any other frame is rotated into the planning frame here, since frames outside the robot model (odom)
        are not known inside the MTC planning scene
        '''
        vector = [direction.vector.x, direction.vector.y, direction.vector.z]
        if np.linalg.norm(vector) == 0.0:
            raise ValueError('gripper translation without direction')
        frame = direction.header.frame_id or self.planning_frame
        if frame in (self.eef_link, self.planning_frame):
            return Vector3Stamped(header=rospy.Header(frame_id=frame), vector=Vector3(*vector))
        rotation = transform_to_matrix(
            self.tf_buffer.lookup_transform(self.planning_frame, frame, rospy.Time(0), rospy.Duration(2.0))
        )[:3, :3]
        return Vector3Stamped(header=rospy.Header(frame_id=self.planning_frame), vector=Vector3(*rotation.dot(vector)))

    def eef_to_attached_object(self, object_name):
        '''4x4 pose of the attached object relative to the end effector link, read from move_group's scene'''
        request = GetPlanningSceneRequest()
        request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        attached = self.get_planning_scene_srv(request).scene.robot_state.attached_collision_objects
        for attached_object in attached:
            if attached_object.object.id == object_name:
                break
        else:
            raise ValueError(f'{object_name} is not attached to the robot')
        collision_object = attached_object.object
        header_frame = collision_object.header.frame_id or attached_object.link_name
        eef_to_header = transform_to_matrix(
            self.tf_buffer.lookup_transform(self.eef_link, header_frame, rospy.Time(0), rospy.Duration(2.0))
        )
        return eef_to_header.dot(pose_to_matrix(collision_object.pose))

    # ------------------------------------------------------------------ planning and execution

    def plan(self, task, what):
        start = time.monotonic()
        try:
            task.init()
            planned = bool(task.plan(1)) and len(task.solutions) > 0
        except Exception as e:  # MTC raises InitStageError for inconsistent stage setups
            rospy.logwarn(f'mtc: {what}: {e}')
            return False
        elapsed = time.monotonic() - start
        if planned:
            rospy.loginfo(f'mtc: {what} planned in {elapsed:.1f} s (cost {task.solutions[0].cost:.2f})')
            task.publish(task.solutions[0])
        else:
            rospy.loginfo(f'mtc: {what} not feasible ({elapsed:.1f} s): {self.describe_failures(task)}')
        return planned

    @staticmethod
    def only_retreat_failed(task):
        '''true when the place stages up to the gripper opening found solutions and only the retreat failed'''
        try:
            place_stages = task['place']
            return (len(place_stages['detach object'].solutions) > 0 and len(list(place_stages['retreat'].failures)) > 0)
        except Exception:
            return False

    @staticmethod
    def describe_failures(task):
        '''solutions and failures of every stage, with the first failure comment, e.g. "approach 0 ok / 3 failed
        (min_distance not reached)"'''
        report = []

        def visit(stage, prefix):
            index = 0
            while True:
                try:
                    child = stage[index]
                except Exception:  # past the last child, or not a container
                    break
                index += 1
                visit(child, prefix + child.name + '/')
            if index == 0:
                failures = list(stage.failures)
                comment = next((f.comment for f in failures if f.comment), '')
                report.append(
                    f'{prefix[:-1]} {len(stage.solutions)} ok / {len(failures)} failed' + (f' ({comment})' if comment else '')
                )

        index = 0
        while True:
            try:
                stage = task[index]
            except Exception:
                break
            index += 1
            visit(stage, stage.name + '/')
        return '; '.join(report)

    def execute(self, task, roles, postures_before, stage_posture, attach=None, detach=None):
        '''
        run the first solution of task. roles has one entry per leaf stage in the order they were added.
        postures_before are gripper postures (trajectory_msgs/JointTrajectory, opening in metres) sent before
        anything else, stage_posture is the one for the GRIPPER stage
        '''
        sub_trajectories = list(task.solutions[0].toMsg().sub_trajectory)
        if len(sub_trajectories) != len(roles):
            rospy.logerr(
                f'mtc: solution has {len(sub_trajectories)} sub trajectories, expected {len(roles)} '
                f'({roles}); not executing'
            )
            return MoveItErrorCodes.FAILURE
        postures = list(postures_before) + [stage_posture]
        steps = [(GRIPPER, None)] * len(postures_before) + list(zip(roles, sub_trajectories))
        for role, sub in steps:
            if self.is_preempt_requested():
                return MoveItErrorCodes.PREEMPTED
            if role == ARM:
                if not sub.trajectory.joint_trajectory.points:
                    continue
                code = self.execute_arm(sub.trajectory)
            elif role == GRIPPER:
                code = self.command_gripper(postures.pop(0))
            elif role == ATTACH:
                code = self.attach(*attach)
            elif role == DETACH:
                code = self.detach(detach)
            else:
                if sub.trajectory.joint_trajectory.points:
                    rospy.logerr('mtc: unexpected motion in a planning scene stage, aborting')
                    return MoveItErrorCodes.FAILURE
                continue
            if code != MoveItErrorCodes.SUCCESS:
                return code
        return MoveItErrorCodes.SUCCESS

    def execute_arm(self, robot_trajectory):
        self.execute_client.send_goal(ExecuteTrajectoryGoal(trajectory=robot_trajectory))
        while not self.execute_client.wait_for_result(rospy.Duration(0.1)):
            if rospy.is_shutdown() or self.is_preempt_requested():
                self.execute_client.cancel_goal()
                self.execute_client.wait_for_result(rospy.Duration(2.0))
                return MoveItErrorCodes.PREEMPTED
        code = self.execute_client.get_result().error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            rospy.logerr(f'mtc: arm trajectory execution failed with MoveIt error code {code}')
        return code

    def command_gripper(self, posture):
        '''send the single point posture of a Grasp/PlaceLocation (opening in metres) to the GripperCommand server'''
        if not self.gripper_client.wait_for_server(rospy.Duration(2.0)):
            rospy.logerr(f'mtc: gripper action server {self.gripper_action_name} not available')
            return MoveItErrorCodes.CONTROL_FAILED
        point = posture.points[-1]
        goal = GripperCommandGoal()
        goal.command.position = point.positions[0]
        goal.command.max_effort = point.effort[0] if point.effort else 0.0
        rospy.loginfo(f'mtc: gripper to {goal.command.position:.3f} m (max effort {goal.command.max_effort})')
        self.gripper_client.send_goal(goal)
        if not self.gripper_client.wait_for_result(rospy.Duration(self.gripper_action_timeout)):
            self.gripper_client.cancel_goal()
            rospy.logerr('mtc: gripper command timed out')
            return MoveItErrorCodes.TIMED_OUT
        # closing on an object stalls the fingers before the commanded position, which is a successful grasp
        result = self.gripper_client.get_result()
        if self.gripper_client.get_state() == GoalStatus.SUCCEEDED or (result is not None and result.stalled):
            return MoveItErrorCodes.SUCCESS
        rospy.logwarn(
            f'mtc: gripper ended in state {self.gripper_client.get_state()} '
            f'(position {getattr(result, "position", None)}), continuing like the pick pipeline'
        )
        return MoveItErrorCodes.SUCCESS

    def attach(self, object_name, touch_links):
        '''attach the world object to the end effector link in move_group's scene, keeping its geometry'''
        attached = AttachedCollisionObject()
        attached.link_name = self.eef_link
        attached.object.id = object_name
        attached.object.operation = CollisionObject.ADD
        attached.touch_links = touch_links
        return self.apply_scene(attached, f'attached {object_name} to {self.eef_link}')

    def detach(self, object_name):
        '''detach the object; move_group puts it back into the world where it is'''
        detached = AttachedCollisionObject()
        detached.link_name = self.eef_link
        detached.object.id = object_name
        detached.object.operation = CollisionObject.REMOVE
        return self.apply_scene(detached, f'detached {object_name}')

    def apply_scene(self, attached_collision_object, message):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached_collision_object]
        try:
            if self.apply_planning_scene_srv(scene).success:
                rospy.loginfo(f'mtc: {message}')
                return MoveItErrorCodes.SUCCESS
        except rospy.ServiceException as e:
            rospy.logerr(f'mtc: apply_planning_scene failed: {e}')
        return MoveItErrorCodes.FAILURE
