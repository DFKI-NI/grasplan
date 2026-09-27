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

Optional arm cable guard (~mtc_cable_constraints, re-read on every goal): the cable model of
mobipick_sim_cable_entanglement (#118) turns into constraints on the joint path instead of joint limits that
reject grasps (#120, #124). Every free motion (Connect) gets a MoveIt path constraint that keeps wrist_3 within
half a turn of the angle where the tube is least wound (no extra winding), several solutions are planned, and
the cheapest one whose whole joint path keeps the cable model's stretch below ~mtc_cable_max_stretch is executed.
'''

import copy
import math
import os
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
    Constraints,
    JointConstraint,
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


class CableGuard:
    '''
    Arm cable entanglement as constraints on MTC paths (#124), from the cable model of
    mobipick_sim_cable_entanglement (doc/cable_model.md there, #118):

    - path constraint: wrist_3 stays within ~mtc_cable_wrist_3_half_window_deg of the neutral angle of the model's
      wrist span (where the tube takes the short way around the joint), so no motion adds a turn of winding. The
      model alone cannot see extra turns once the wrist_3 housing no longer holds the tube, hence this bound.
    - trajectory check: the stretch s = L_req / L_free - 1 of every span, evaluated along the whole joint path
      (geometry only, no rope history), must stay below ~mtc_cable_max_stretch (0 = tube just taut).
    '''

    def __init__(self, joint_prefix):
        from mobipick_sim_cable_entanglement.cable_model import CableModel, config_from_dict  # amenable_ws
        import rospkg
        import yaml

        config = rospy.get_param('~mtc_cable_model_config', '') or os.path.join(
            rospkg.RosPack().get_path('mobipick_sim_cable_entanglement'), 'config', 'cable_model.yaml'
        )
        with open(config) as f:
            self.cfg = config_from_dict(yaml.safe_load(f))
        self.model = CableModel(rospy.get_param('robot_description'), self.cfg)
        wrist_3 = [wj for span in self.cfg.spans for wj in span.joints if wj.joint == 'ur5_wrist_3_joint']
        self.wrist_3_neutral = wrist_3[0].neutral if wrist_3 else math.pi - 0.25
        self.wrist_3_joint = joint_prefix + 'ur5_wrist_3_joint'
        self.max_stretch = 0.0
        self.half_window = math.radians(170.0)
        self.max_points = 40
        self.update_params()
        rospy.loginfo(
            f'mtc cable guard: model {config}, wrist_3 {math.degrees(self.wrist_3_neutral):.1f} +- '
            f'{math.degrees(self.half_window):.0f} deg, max stretch {self.max_stretch:+.3f}'
        )

    def update_params(self):
        self.max_stretch = rospy.get_param('~mtc_cable_max_stretch', 0.0)
        self.half_window = math.radians(rospy.get_param('~mtc_cable_wrist_3_half_window_deg', 170.0))
        self.max_points = int(rospy.get_param('~mtc_cable_points_per_trajectory', 40))

    def path_constraints(self):
        constraints = Constraints()
        constraints.name = 'arm cable: wrist_3 winding'
        constraints.joint_constraints.append(
            JointConstraint(
                joint_name=self.wrist_3_joint,
                position=self.wrist_3_neutral,
                tolerance_above=self.half_window,
                tolerance_below=self.half_window,
                weight=1.0,
            )
        )
        return constraints

    def check(self, solution_msg):
        '''(ok, worst stretch, worst span, worst joint angles in degrees) along all sub trajectories'''
        worst, worst_span, worst_q = -float('inf'), '', {}
        lo, hi = self.wrist_3_neutral - self.half_window, self.wrist_3_neutral + self.half_window
        for sub in solution_msg.sub_trajectory:
            trajectory = sub.trajectory.joint_trajectory
            points = trajectory.points
            if not points:
                continue
            step = max(1, len(points) // self.max_points)
            for point in list(points[::step]) + [points[-1]]:
                q = dict(zip(trajectory.joint_names, point.positions))
                w3 = q.get(self.wrist_3_joint)
                if w3 is not None and not lo - 1e-3 <= w3 <= hi + 1e-3:
                    return False, float('inf'), 'wrist_3 window', {k: round(math.degrees(v), 1) for k, v in q.items()}
                state = self.model.evaluate(q)
                if state.max_stretch > worst:
                    worst, worst_span = state.max_stretch, state.worst_span
                    worst_q = {k.split('/')[-1]: round(math.degrees(v), 1) for k, v in q.items()}
        return worst < self.max_stretch, worst, worst_span, worst_q


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
        self.cable_guard = None  # CableGuard, built on the first goal with ~mtc_cable_constraints true
        self.cable_solutions = 1
        self.chosen_solution = None
        # why the last goal failed (for the action status text) and whether it moved anything
        self.failure_reason = ''
        self.executed = False
        self.check_grasp = rospy.get_param('~mtc_check_grasp', True)
        self.grasp_check_delay = rospy.get_param('~mtc_grasp_check_delay', 1.0)
        self.gripper_fact = self.make_gripper_fact() if self.check_grasp else None
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
        self.failure_reason, self.executed = '', False
        self.update_cable_guard()
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
            self.executed = True
            code = self.execute(task, roles, [grasp.pre_grasp_posture], grasp.grasp_posture,
                                attach=(goal.target_name, touch), check_grasp=True)
            if code == MoveItErrorCodes.SUCCESS and not self.gripper_holds_object('after the lift'):
                self.failure_reason = f'{goal.target_name} was lost during the lift (the gripper is empty)'
                code = MoveItErrorCodes.FAILURE
            if code != MoveItErrorCodes.SUCCESS and self.failure_reason:
                rospy.logerr(f'mtc: pick failed: {self.failure_reason}')
                self.release_empty_grasp(goal.target_name, grasp.pre_grasp_posture)
            result.error_code.val = code
            if code == MoveItErrorCodes.SUCCESS:
                result.grasp = grasp
            elif not self.failure_reason:
                self.failure_reason = f'executing grasp {grasp.id} failed with MoveIt error code {code}'
            return result  # something moved: never try another grasp from a changed state
        self.failure_reason = f'none of the {len(goal.possible_grasps)} grasps could be planned'
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
        self.failure_reason, self.executed = '', False
        self.update_cable_guard()
        if not self.gripper_holds_object('before placing', delay=0.2):
            # e.g. it slipped out while the arm swung through the observe or untangle poses
            self.failure_reason = f'{goal.attached_object_name} is no longer in the gripper (gripper_has_object false)'
            rospy.logerr(f'mtc: place failed: {self.failure_reason}')
            self.release_empty_grasp(goal.attached_object_name, None)
            self.executed = True
            result.error_code.val = MoveItErrorCodes.FAILURE
            return result
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
                self.executed = True
                code = self.execute(task, roles, [], location.post_place_posture,
                                    detach=goal.attached_object_name)
                result.error_code.val = code
                if code == MoveItErrorCodes.SUCCESS:
                    result.place_location = location
                else:
                    self.failure_reason = f'executing place location {location.id} failed with MoveIt error code {code}'
                return result
        rospy.logerr(f'mtc: none of the {len(goal.place_locations)} place locations could be planned')
        self.failure_reason = f'none of the {len(goal.place_locations)} place locations could be planned'
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
        if self.cable_guard is not None:
            connect.properties['path_constraints'] = self.cable_guard.path_constraints()
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

    def update_cable_guard(self):
        '''(re)read ~mtc_cable_constraints on every goal, so the guard can be switched without a restart'''
        if not rospy.get_param('~mtc_cable_constraints', False):
            if self.cable_guard is not None:
                rospy.loginfo('mtc: cable guard off')
            self.cable_guard = None
            return
        if self.cable_guard is None:
            if getattr(self, '_cable_guard_cache', None) is None:
                prefix = self.eef_link.rsplit('/', 1)[0] + '/' if '/' in self.eef_link else ''
                try:
                    self._cable_guard_cache = CableGuard(prefix)
                except Exception as e:  # the model package lives in amenable_ws, it may be missing
                    rospy.logerr(f'mtc: cable guard requested but not available, planning without it: {e}')
                    return
            self.cable_guard = self._cable_guard_cache
        self.cable_guard.update_params()
        self.cable_solutions = max(1, int(rospy.get_param('~mtc_cable_solutions', 4)))

    def plan(self, task, what):
        '''plan task; sets self.chosen_solution (the cheapest solution, or with the cable guard the cheapest one
        whose joint path passes the cable check)'''
        start = time.monotonic()
        self.chosen_solution = None
        try:
            task.init()
            planned = bool(task.plan(self.cable_solutions if self.cable_guard else 1)) and len(task.solutions) > 0
        except Exception as e:  # MTC raises InitStageError for inconsistent stage setups
            rospy.logwarn(f'mtc: {what}: {e}')
            return False
        elapsed = time.monotonic() - start
        if not planned:
            rospy.loginfo(f'mtc: {what} not feasible ({elapsed:.1f} s): {self.describe_failures(task)}')
            return False
        if self.cable_guard is None:
            self.chosen_solution = task.solutions[0]
        else:
            checks = []
            for index, solution in enumerate(task.solutions):
                ok, stretch, span, q = self.cable_guard.check(solution.toMsg())
                checks.append(f'{index}: cost {solution.cost:.1f} stretch {stretch:+.3f} ({span})')
                if ok:
                    self.chosen_solution = solution
                    break
                rospy.loginfo(f'mtc: {what} solution {index} rejected by the cable guard: stretch {stretch:+.3f} '
                              f'> {self.cable_guard.max_stretch:+.3f} in the {span} span at {q}')
            if self.chosen_solution is None:
                rospy.loginfo(f'mtc: {what}: all {len(task.solutions)} solutions entangle the arm cable '
                              f'({"; ".join(checks)})')
                return False
            rospy.loginfo(f'mtc: {what} cable check passed ({checks[-1]})')
        elapsed = time.monotonic() - start
        rospy.loginfo(f'mtc: {what} planned in {elapsed:.1f} s (cost {self.chosen_solution.cost:.2f})')
        task.publish(self.chosen_solution)
        return True

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

    def execute(self, task, roles, postures_before, stage_posture, attach=None, detach=None, check_grasp=False):
        '''
        run the first solution of task. roles has one entry per leaf stage in the order they were added.
        postures_before are gripper postures (trajectory_msgs/JointTrajectory, opening in metres) sent before
        anything else, stage_posture is the one for the GRIPPER stage
        '''
        sub_trajectories = list(self.chosen_solution.toMsg().sub_trajectory)
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
                if code == MoveItErrorCodes.SUCCESS and check_grasp and not postures:  # the closing step
                    if not self.gripper_holds_object('after closing'):
                        self.failure_reason = f'the gripper closed on nothing at {attach[0] if attach else "the object"}'
                        return MoveItErrorCodes.FAILURE
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

    def make_gripper_fact(self):
        '''
        the gripper_has_object fact of symbolic_fact_generation (tested in sim and on the real robot: finger joint
        position and effort in the sim, the Robotiq gOBJ register on the real robot), with the parameters of its
        facts_config.yaml so both agree
        '''
        try:
            import rospkg
            import yaml
            from symbolic_fact_generation.gripper_facts_generator import GripperHasObjectGenerator

            config = rospy.get_param('~mtc_gripper_fact_config', '') or os.path.join(
                rospkg.RosPack().get_path('symbolic_fact_generation'), 'config', 'facts_config.yaml'
            )
            with open(config) as f:
                facts = yaml.safe_load(f)
            entries = facts.get('facts', facts) if isinstance(facts, dict) else facts
            params = next(e['gripper_has_object']['params'] for e in entries if 'gripper_has_object' in e)
            rospy.loginfo(f'mtc: grasp check with gripper_has_object from {config}')
            return GripperHasObjectGenerator('gripper_has_object', *params)
        except Exception as e:
            rospy.logerr(f'mtc: grasp check unavailable, picks are not verified: {e}')
            return None

    def gripper_holds_object(self, when, delay=None):
        if self.gripper_fact is None:
            return True
        rospy.sleep(self.grasp_check_delay if delay is None else delay)  # the fingers settle, the fact's effort hold and gOBJ catch up
        holds = bool(self.gripper_fact.generate_facts())
        rospy.loginfo(f'mtc: gripper_has_object {when}: {holds}')
        return holds

    def release_empty_grasp(self, object_name, open_posture):
        '''after a failed grasp: the object is not in the hand, so drop it from the scene and open the gripper'''
        self.detach(object_name)
        removal = PlanningScene()
        removal.is_diff = True
        removal.world.collision_objects = [CollisionObject(id=object_name, operation=CollisionObject.REMOVE)]
        try:
            self.apply_planning_scene_srv(removal)
        except rospy.ServiceException as e:
            rospy.logwarn(f'mtc: could not remove {object_name} from the scene: {e}')
        if open_posture is not None:
            self.command_gripper(open_posture)

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
