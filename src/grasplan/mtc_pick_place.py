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
mobipick_cable_entanglement (#118) turns into constraints on the joint path instead of joint limits that
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
from shape_msgs.msg import SolidPrimitive
from moveit.task_constructor import core, stages

from grasplan.tools.octomap_probe import count_in_box, occupied_leaves
from grasplan.tools.common import robot_prefix
from grasplan.tools.touching_boxes import boxes_touch

# the name MoveIt gives the octomap inside the planning scene (planning_scene::PlanningScene::OCTOMAP_NS)
OCTOMAP_COLLISION_NAME = '<octomap>'

# STOPGAP until the cable model refit (#118): default ~mtc_cable_max_stretch, sim and real. On the real robot the
# guarded plans that caught or nearly caught the tube had a wrist stretch of -0.108 / -0.103, all clean ones
# -0.126..-0.217 (#121), so the model's 0.0 (tube just taut) let tangling plans through. Revisit after the refit.
DEFAULT_CABLE_MAX_STRETCH = -0.115

# a close width (m) above this is a per-object width short of fully closed, which ~mtc_grasp_check_squeeze may close
# on to check the grasp (#188); the klt and materialbox widths (0.0005) count as fully closed
GRASP_CHECK_SQUEEZE_MIN_WIDTH = 0.005

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


# a round object's grasp may turn by any angle about its approach only this close to straight down (#121)
TURN_ANY_ANGLE_MAX_TILT_DEG = 20.0


def turn_grasp_about_approach(grasp, turn_deg):
    '''copy of a moveit_msgs/Grasp whose grasp pose (of the gripper_tcp) is turned by turn_deg about its own x axis,
    the approach axis and, on the Mobipick, the wrist_3 axis (#121)'''
    turned = copy.deepcopy(grasp)
    turned.id = f'{grasp.id}_turned{turn_deg:+.0f}'
    matrix_to_pose(pose_to_matrix(grasp.grasp_pose.pose).dot(tft.rotation_matrix(math.radians(turn_deg), (1, 0, 0))),
                   turned.grasp_pose.pose)
    return turned


def speed_changes_on():
    """~mtc_speed_changes (default true, read per goal): false turns off the untested 2026-09-28 pick speed changes
    (IK pin, one solution per candidate without a replan after a cable guard reject; pick.py's cable gate reads it too)
    for a real-robot run where they misbehave (#151)"""
    return bool(rospy.get_param('~mtc_speed_changes', True))


def solution_joint_state(msg):
    """joint name -> position at the end of a moveit_task_constructor_msgs/Solution: the full start scene state,
    updated by the robot state of the last sub trajectory's scene diff (a ComputeIK solution: the IK state)"""
    js = msg.start_scene.robot_state.joint_state
    q = dict(zip(js.name, js.position))
    if msg.sub_trajectory:
        diff = msg.sub_trajectory[-1].scene_diff.robot_state.joint_state
        q.update(zip(diff.name, diff.position))
    return q


def pinned_ik_cost(reference, tolerance):
    '''MTC cost term for the grasp-pose IK stage (#151): an IK solution farther than tolerance (rad, any joint) from
    reference (joint name -> rad, the IK the cable ranking judged) is marked as failed; the others cost their summed
    joint distance, so the pinned branch is planned (and cable-checked) first. MTC 0.1.3 calls it with
    (SubTrajectory, comment): pybind picks that overload for any callable. The state comes from the solution message:
    sub.end.scene is a C++ PlanningScene without a Python conversion in this MTC build.'''
    def cost(sub, comment=''):
        try:
            positions = solution_joint_state(sub.toMsg())
        except Exception as e:  # no state on this solution: no pin rather than a broken pick
            rospy.logwarn_once(f'mtc: grasp IK pin not applicable ({e}); planning without it')
            return 0.0
        diffs = [abs(positions[n] - v) for n, v in reference.items() if n in positions]
        if not diffs:
            rospy.logwarn_once('mtc: grasp IK pin: no reference joint in the IK solution; planning without it')
            return 0.0
        if max(diffs) > tolerance:
            sub.markAsFailure(f'not the ranked IK branch (max joint offset {math.degrees(max(diffs)):.0f} deg)')
            rospy.loginfo_throttle(
                5.0, f'mtc: grasp IK pin rejected an IK branch {math.degrees(max(diffs)):.0f} deg from the ranked one '
                     f'(tolerance {math.degrees(tolerance):.0f} deg)')
            return float('inf')
        return float(sum(diffs))
    return cost


def has_twin(grasps, grasp, turn_deg, max_angle_deg=5.0, max_offset=0.01):
    '''True when grasps already hold grasp turned by turn_deg about its approach axis (same frame, TCP within
    max_offset m, orientation within max_angle_deg), e.g. the 180 deg twins the grasp server adds itself'''
    target = pose_to_matrix(turn_grasp_about_approach(grasp, turn_deg).grasp_pose.pose)
    frame = grasp.grasp_pose.header.frame_id
    for other in grasps:
        if other is grasp or other.grasp_pose.header.frame_id != frame:
            continue
        m = pose_to_matrix(other.grasp_pose.pose)
        if np.linalg.norm(m[:3, 3] - target[:3, 3]) > max_offset:
            continue
        cos = (np.trace(m[:3, :3].T.dot(target[:3, :3])) - 1.0) / 2.0
        if math.degrees(math.acos(max(-1.0, min(1.0, cos)))) <= max_angle_deg:
            return True
    return False


def transform_to_matrix(transform):
    t, r = transform.transform.translation, transform.transform.rotation
    matrix = tft.quaternion_matrix([r.x, r.y, r.z, r.w])
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix



def place_retreat_passes(relaxed_min_distance, force_straight_up=False):
    '''
    (retreat_min_distance or None for the configured one, straight_up) per place pass (#136): the retreat along the
    gripper axis, straight up, then both with the relaxed minimum distance; each later pass runs only when locations
    of the earlier ones failed at the retreat. force_straight_up (a test switch) leaves out the axis passes.
    '''
    passes = [(None, False), (None, True), (relaxed_min_distance, False), (relaxed_min_distance, True)]
    return [p for p in passes if p[1]] if force_straight_up else passes

class CableGuard:
    '''
    Arm cable entanglement as constraints on MTC paths (#124), from the cable model of
    mobipick_cable_entanglement (doc/cable_model.md there, #118):

    - path constraint: wrist_3 stays within ~mtc_cable_wrist_3_half_window_deg of the neutral angle of the model's
      wrist span (where the tube takes the short way around the joint), so no motion adds a turn of winding. The
      model alone cannot see extra turns once the wrist_3 housing no longer holds the tube, hence this bound.
    - trajectory check: the stretch s = L_req / L_free - 1 of every span, evaluated along the whole joint path
      (geometry only, no chain history), must stay below ~mtc_cable_max_stretch (0 = tube just taut; default
      DEFAULT_CABLE_MAX_STRETCH, a stopgap).
    '''

    def __init__(self, joint_prefix):
        from mobipick_cable_entanglement.cable_model import CableModel, config_from_dict  # separate package
        import rospkg
        import yaml

        config = rospy.get_param('~mtc_cable_model_config', '') or os.path.join(
            rospkg.RosPack().get_path('mobipick_cable_entanglement'), 'config', 'cable_model.yaml'
        )
        with open(config) as f:
            self.cfg = config_from_dict(yaml.safe_load(f))
        self.cfg.tf_prefix = robot_prefix() + '/'   # #226: this robot's links (mobipick/, mobipick2/, ...)
        self.model = CableModel(rospy.get_param('robot_description'), self.cfg)
        wrist_3 = [wj for span in self.cfg.spans for wj in span.joints if wj.joint == 'ur5_wrist_3_joint']
        self.wrist_3_neutral = wrist_3[0].neutral if wrist_3 else math.pi - 0.25
        self.wrist_3_joint = joint_prefix + 'ur5_wrist_3_joint'
        self.max_stretch = DEFAULT_CABLE_MAX_STRETCH
        self.half_window = math.radians(170.0)
        self.max_points = 40
        self.update_params()
        rospy.loginfo(
            f'mtc cable guard: model {config}, wrist_3 {math.degrees(self.wrist_3_neutral):.1f} +- '
            f'{math.degrees(self.half_window):.0f} deg, max stretch {self.max_stretch:+.3f}'
        )

    def update_params(self):
        # the disc chain replay (the monitor's chain, #228); the parameter names keep the word rope from the removed
        # radius rope so launch files and profiles stay valid
        self.chain_check = bool(rospy.get_param('~mtc_cable_rope_check', True))
        self.chain_rate = float(rospy.get_param('~mtc_cable_rope_rate', 10.0))
        self.max_stretch = rospy.get_param('~mtc_cable_max_stretch', DEFAULT_CABLE_MAX_STRETCH)
        self.half_window = math.radians(rospy.get_param('~mtc_cable_wrist_3_half_window_deg', 170.0))
        self.max_points = int(rospy.get_param('~mtc_cable_points_per_trajectory', 40))

    def stop_stretch(self):
        '''the cable monitor's stop threshold (config decision.stop_stretch, e.g. +0.02)'''
        return float(self.cfg.decision.stop_stretch)

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

    def check(self, solution_msg, max_stretch=None, chain_stops=True):
        '''(ok, worst stretch, worst span, worst joint angles in degrees) along all sub trajectories; max_stretch
        overrides the planning margin ~mtc_cable_max_stretch (named-pose moves use the monitor's stop threshold);
        chain_stops False only logs a chain replay stop instead of rejecting the path'''
        limit = self.max_stretch if max_stretch is None else max_stretch
        worst, worst_span, worst_q = -float('inf'), '', {}
        self.worst_q_rad = None   # the state turn_for_slack() starts from; never one of an earlier solution
        lo, hi = self.wrist_3_neutral - self.half_window, self.wrist_3_neutral + self.half_window
        self.worst_where = None   # (sub trajectory index, point index, points) of the worst point, for the logs
        for sub_index, sub in enumerate(solution_msg.sub_trajectory):
            trajectory = sub.trajectory.joint_trajectory
            points = trajectory.points
            if not points:
                continue
            step = max(1, len(points) // self.max_points)
            indices = list(range(0, len(points), step)) + [len(points) - 1]
            for point_index in indices:
                point = points[point_index]
                q = dict(zip(trajectory.joint_names, point.positions))
                w3 = q.get(self.wrist_3_joint)
                if w3 is not None and not lo - 1e-3 <= w3 <= hi + 1e-3:
                    return False, float('inf'), 'wrist_3 window', {k: round(math.degrees(v), 1) for k, v in q.items()}
                state = self.model.evaluate(q)
                if state.max_stretch > worst:
                    worst, worst_span = state.max_stretch, state.worst_span
                    worst_q = {k.split('/')[-1]: round(math.degrees(v), 1) for k, v in q.items()}
                    self.worst_q_rad = dict(q)   # for turn_for_slack()
                    self.worst_where = (sub_index, point_index, len(points))
        if worst < limit and getattr(self, 'chain_check', False):
            verdict = self.chain_verdict(solution_msg)
            if verdict is not None and verdict.stop and not chain_stops:
                rospy.logwarn(f'mtc cable guard: the chain replay would stop this path ({verdict.span}, '
                              f'{verdict.limiting}, {verdict.max_stretch:+.3f}), not rejected: chain check is log only')
            elif verdict is not None and verdict.stop:
                # the sim monitor would stop this path: its chain catches on the tool, which the geometry cannot see
                self.worst_q_rad = dict(verdict.q)   # a turn must fix the chain's worst point, not the geometry's
                return False, verdict.max_stretch, f'{verdict.span} ({verdict.limiting}, chain replay)', {
                    k.split('/')[-1]: round(math.degrees(v), 1) for k, v in verdict.q.items()}
        return worst < limit, worst, worst_span, worst_q

    def chain_verdict(self, solution_msg):
        '''the cable monitor's decision (disc chain with history, same model and thresholds) along the whole joint
        path of the solution, from a freshly settled chain at its start (mobipick_cable_entanglement.path_check, #121)'''
        from mobipick_cable_entanglement.cable_model import CableModel
        from mobipick_cable_entanglement.path_check import rope_replay
        if getattr(self, 'chain_model', None) is None:
            cfg = copy.deepcopy(self.cfg)
            cfg.rope.enabled = True
            self.chain_model = CableModel(rospy.get_param('robot_description'), cfg)
        samples, offset = [], 0.0
        for sub in solution_msg.sub_trajectory:
            trajectory = sub.trajectory.joint_trajectory
            if not trajectory.points:
                continue
            for point in trajectory.points:
                samples.append((offset + point.time_from_start.to_sec(), dict(zip(trajectory.joint_names, point.positions))))
            offset = samples[-1][0] + 1e-3
        start = time.monotonic()
        verdict = rope_replay(self.chain_model, samples, self.chain_rate)
        if verdict is not None:
            rospy.loginfo(f'mtc cable guard: chain replay of {verdict.updates} ticks in {time.monotonic() - start:.1f} s: '
                          f'max stretch {verdict.max_stretch:+.3f} ({verdict.limiting}), stop {verdict.stop}')
        return verdict

    def turn_for_slack(self, q, round_object):
        '''(turn in degrees, predicted stretch) about the grasp approach axis (= the wrist_3 axis) that gives the most
        slack at the rejected joint state q (radians): any angle within +-60 deg or the 180 deg flip for a round
        object, only the flip otherwise; None when no turn is predicted to pass (#121)'''
        from mobipick_cable_entanglement.slack import FLIP_ONLY, ROUND_TURNS, best_wrist_3_turn
        return best_wrist_3_turn(
            self.model, q, self.max_stretch,
            (self.wrist_3_neutral - self.half_window, self.wrist_3_neutral + self.half_window),
            ROUND_TURNS if round_object else FLIP_ONLY,
        )


def named_move_guard_mode():
    '''~mtc_cable_guard_named_moves: enforce (refuse, replan, go via transport), log (move as without the guard, only
    log what it would do) or off; default enforce in the sim (/use_sim_time) and log on the real robot (Oscar
    2026-09-28: the real named moves only log for now). Booleans of the old param: true enforce, false off.'''
    mode = rospy.get_param('~mtc_cable_guard_named_moves', None)
    if mode is None:
        return 'enforce' if rospy.get_param('/use_sim_time', False) else 'log'
    if isinstance(mode, bool):
        return 'enforce' if mode else 'off'
    mode = str(mode).strip().lower()
    if mode not in ('enforce', 'log', 'off'):
        rospy.logwarn_once(f'~mtc_cable_guard_named_moves {mode!r} unknown (enforce, log, off): logging only')
        return 'log'
    return mode


def _plain_named_move(arm, name):
    '''the old named move: go, one retry after 1 s'''
    if arm.go():
        return True
    rospy.logwarn(f'failed to move arm to posture: {name}, will retry one more time in 1 sec')
    rospy.sleep(1.0)
    return bool(arm.go())


def _logged_named_move(arm, name, guard):
    '''plan the named move without the wrist_3 constraint like arm.go() does, log what the guard (stop_stretch and
    chain replay) would say, and execute that same plan whatever it says; no plan: the old go with its retry'''
    planned = arm.plan()
    success, trajectory = (planned[0], planned[1]) if isinstance(planned, tuple) else (True, planned)
    if not success or not trajectory.joint_trajectory.points:
        return _plain_named_move(arm, name)
    try:
        guard.update_params()
        ok, stretch, span, q = guard.check(_SolutionView([trajectory]), max_stretch=guard.stop_stretch(),
                                           chain_stops=True)
        if ok:
            rospy.loginfo(f'moving arm to {name} (cable check passed: stretch {stretch:+.3f}, log only)')
        else:
            rospy.logwarn(f'moving arm to {name} although the cable guard would refuse it (log only): stretch '
                          f'{stretch:+.3f} in the {span} span at {q}')
    except Exception as e:   # the log must never keep the arm from moving
        rospy.logwarn(f'moving arm to {name}: cable check failed ({e}), log only')
    return bool(arm.execute(trajectory, wait=True))


def guarded_named_move(arm, name, guard, attempts=2, via=None):
    '''
    Move a MoveGroupCommander arm to the SRDF pose name like arm.go(), but plan first and let the MTC cable guard
    (geometry and the monitor's chain replay, #121) check the joint path before anything moves: the 2026-09-28 sim
    stop at 02:03 came from such an unguarded move (to the anygrasp pose). The check uses the monitor's own stop
    criterion (decision.stop_stretch, chain stop), not the MTC planning margin ~mtc_cable_max_stretch, which refused the
    standard anygrasp view (-0.09) after a place (night19). A rejected plan is replanned (the planner is randomized) up
    to attempts times, then the move goes via ~mtc_cable_named_move_via (default [transport]); if that fails too,
    nothing more moves and False is returned with the reason logged. guard None: the old behaviour (go, one retry).
    A chain replay stop only logs unless ~mtc_cable_named_move_rope_stops is true: it refused a re-pick the unguarded
    code does fine (suite 2026-09-28, #121). ~mtc_cable_guard_named_moves (see named_move_guard_mode) can make the
    whole guard log only: the arm then moves as without it and the checks are only logged.
    '''
    arm.set_named_target(name)
    mode = named_move_guard_mode() if guard is not None else 'off'
    if mode == 'log':
        return _logged_named_move(arm, name, guard)
    if mode == 'off':
        return _plain_named_move(arm, name)
    guard.update_params()
    chain_stops = bool(rospy.get_param('~mtc_cable_named_move_rope_stops', False))
    w3 = _current_joint(guard.wrist_3_joint)
    lo, hi = guard.wrist_3_neutral - guard.half_window, guard.wrist_3_neutral + guard.half_window
    if w3 is not None and lo <= w3 <= hi:
        arm.set_path_constraints(guard.path_constraints())
    else:
        # a start outside the window (manual jog, after a protective stop) would make every plan fail: plan without
        # the constraint so recovery moves (home, transport) stay possible; the cable check below still runs
        rospy.logwarn(f'arm move to {name}: wrist_3 {"unknown" if w3 is None else f"{math.degrees(w3):.0f} deg"} '
                      f'outside the cable window, planning without the wrist_3 constraint')
    try:
        reason = 'no plan'
        for attempt in range(1, attempts + 1):
            planned = arm.plan()
            success, trajectory = (planned[0], planned[1]) if isinstance(planned, tuple) else (True, planned)
            if not success or not trajectory.joint_trajectory.points:
                reason = 'no plan'
                continue
            # the monitor's own stop criterion (stop_stretch, chain stop), not the MTC planning margin: the standard
            # poses (anygrasp view at about -0.09) would otherwise be refused after a place (night19)
            ok, stretch, span, q = guard.check(_SolutionView([trajectory]), max_stretch=guard.stop_stretch(),
                                                chain_stops=chain_stops)
            if ok:
                rospy.loginfo(f'moving arm to {name} (cable check passed: stretch {stretch:+.3f}, attempt {attempt})')
                return bool(arm.execute(trajectory, wait=True))
            reason = f'cable guard: stretch {stretch:+.3f} in the {span} span at {q}'
            rospy.logwarn(f'arm move to {name}: plan {attempt}/{attempts} rejected by the {reason}')
    finally:
        arm.clear_path_constraints()
    # still refused: go via an intermediate pose (~mtc_cable_named_move_via, e.g. transport) and try once more
    via = rospy.get_param('~mtc_cable_named_move_via', ['transport']) if via is None else via
    for pose in [p for p in via if p != name]:
        rospy.logwarn(f'arm move to {name} refused ({reason}); trying via {pose}')
        if guarded_named_move(arm, pose, guard, attempts, via=[]) and guarded_named_move(arm, name, guard, attempts, via=[]):
            return True
    rospy.logerr(f'not moving the arm to {name}: {reason}')
    return False


def _current_joint(joint, timeout=2.0):
    '''position (rad) of joint from one fresh joint_states message of this node's namespace, None if none came'''
    from sensor_msgs.msg import JointState
    try:
        msg = rospy.wait_for_message('joint_states', JointState, timeout=timeout)
    except rospy.ROSException:
        return None
    return dict(zip(msg.name, msg.position)).get(joint)


class _SolutionView:
    '''a moveit_msgs/RobotTrajectory list shaped like an MTC solution message for CableGuard.check'''

    class _Sub:
        def __init__(self, trajectory):
            self.trajectory = trajectory

    def __init__(self, trajectories):
        self.sub_trajectory = [self._Sub(t) for t in trajectories]


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
        self.gripper_action_name = rospy.get_param('~gripper_action_name', f'/{robot_prefix()}/gripper_hw')
        self.gripper_action_timeout = rospy.get_param('~mtc_gripper_action_timeout', 10.0)
        self.relaxed_retreat_min_distance = rospy.get_param('~mtc_relaxed_retreat_min_distance', 0.05)
        self.retreat_failed = False
        self.cable_guard = None  # CableGuard, built on the first goal with ~mtc_cable_constraints true
        # solutions the cable guard rejected since the caller last set it to 0 (place/insert goal: #198 (e) takes the
        # untangle detour only after such a rejection)
        self.cable_rejections = 0
        self.cable_solutions = 1
        self.chosen_solution = None
        # why the last goal failed (for the action status text) and whether it moved anything
        self.failure_reason = ''
        self.executed = False
        # ~mtc_check_grasp, ~mtc_grasp_check_delay and ~mtc_grasp_check_squeeze are read at every check, so the check
        # can be switched off without a restart (#188); the fact is built anyway, it only subscribes to gripper topics
        self.gripper_fact = self.make_gripper_fact()
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
                grasp, task, roles = self.turned_grasp_plan(goal, grasp)
                if grasp is None:
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
        back off the full min_distance), the locations are tried again: first retreating straight up (planning
        frame +z) instead of back along the gripper axis, which on a tilted grasp (open-set, insert with free_yaw)
        runs into the box wall (#136); then accepting a retreat of at least ~mtc_relaxed_retreat_min_distance
        (e.g. the camera touches the box rim) along the axis, then straight up. The object is already released
        there and the next arm motion is planned collision free anyway.
        '''
        result = PlaceResult()
        result.error_code.val = MoveItErrorCodes.PLANNING_FAILED
        self.failure_reason, self.executed = '', False
        self.update_cable_guard()
        if not self.gripper_holds_object('before placing', delay=0.2):
            # e.g. it slipped out while the arm swung through the observe or untangle poses
            self.failure_reason = f'{goal.attached_object_name} is no longer in the gripper (gripper_has_object false)'
            rospy.logerr(f'mtc: place failed: {self.failure_reason}')
            # open the empty gripper too: left closed, the next pick finds its fingers inside every target box
            open_posture = goal.place_locations[0].post_place_posture if goal.place_locations else None
            self.release_empty_grasp(goal.attached_object_name,
                                     open_posture if open_posture is not None and open_posture.points else None)
            self.executed = True
            result.error_code.val = MoveItErrorCodes.FAILURE
            return result
        try:
            eef_to_object = self.eef_to_attached_object(goal.attached_object_name)
        except (tf2_ros.TransformException, ValueError) as e:
            rospy.logerr(f'mtc: cannot place {goal.attached_object_name}: {e}')
            return result
        # place locations whose footprint holds octomap-only obstacles are not offered (see octomap_blocks_place)
        octomap_check = self.octomap_place_check(goal.attached_object_name)
        # testing #136 (read per goal): skip the axis retreat so the straight-up retreat is planned and executed
        force_straight_up = bool(rospy.get_param('~mtc_test_force_straight_up', False))
        if force_straight_up:
            rospy.logwarn('mtc: TEST ~mtc_test_force_straight_up: planning the place locations with a retreat straight up')
        passes = place_retreat_passes(self.relaxed_retreat_min_distance, force_straight_up)
        # per location, nearest first (#132): its later retreat variants are tried before the next location, so a
        # near spot whose axis retreat is blocked is placed with a straight-up retreat instead of the object going to
        # a far spot that plans at once (sim 2026-09-28 night20: tennis ball 27 cm deep into table 2, re-pick no IK)
        blocked = []
        for index, location in enumerate(goal.place_locations):
            if octomap_check is not None:
                hits = self.octomap_blocks_place(octomap_check, location)
                if hits:
                    blocked.append(location.id)
                    rospy.logwarn(f'mtc: place location {location.id} ({index + 1}/{len(goal.place_locations)}) skipped: '
                                  f'{hits} octomap voxels inside the held {goal.attached_object_name} there (an object '
                                  f'only the octomap sees)')
                    continue
            for number, (retreat_min_distance, straight_up) in enumerate(passes):
                if self.is_preempt_requested():
                    result.error_code.val = MoveItErrorCodes.PREEMPTED
                    return result
                if number > 0:
                    if not self.retreat_failed:
                        break
                    how = 'straight up' if straight_up else 'along the gripper axis'
                    if retreat_min_distance is not None:
                        how += f' of at least {retreat_min_distance:.2f} m'
                    rospy.logwarn(f'mtc: retrying place location {location.id} with a retreat {how}')
                self.retreat_failed = False
                try:
                    task, roles = self.make_place_task(goal, location, eef_to_object, retreat_min_distance, straight_up)
                except (tf2_ros.TransformException, ValueError) as e:
                    rospy.logwarn(f'mtc: place location {location.id} skipped: {e}')
                    break
                if task is None and force_straight_up:
                    # its own retreat is already vertical: the axis plan is the straight-up plan
                    task, roles = self.make_place_task(goal, location, eef_to_object, retreat_min_distance, False)
                if task is None:
                    self.retreat_failed = True  # same plan as the axis retreat, keep going to the next variant
                    continue
                if not self.plan(task, f'place location {location.id} ({index + 1}/{len(goal.place_locations)})'):
                    self.retreat_failed = self.only_retreat_failed(task)
                    continue
                self.executed = True
                self.log_place_pose(location)
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
        if blocked:
            self.failure_reason += f' ({len(blocked)} skipped: objects only the octomap sees in the footprint)'
        return result

    def turned_grasp_plan(self, goal, grasp):
        '''
        after the cable guard rejected every plan of grasp: plan it once more turned about its approach axis
        (gripper_tcp +x, which is the wrist_3 axis, so only wrist_3 changes) by the angle the cable model predicts to
        give the tube the most slack (#121); the guard checks the new plan as usual. Round objects
        (~yaw_free_objects, set per goal by pick.py) approached within TURN_ANY_ANGLE_MAX_TILT_DEG of straight down may
        turn by any angle, everything else only by 180 deg (the fingers are symmetric). Returns (turned grasp, task, roles) when that plan passed, else (None, None, None).
        '''
        q = getattr(self, 'guard_rejected_q', None)
        if self.cable_guard is None or q is None or not rospy.get_param('~mtc_cable_turn_rejected_grasps', True):
            return None, None, None
        round_object = goal.target_name in getattr(self, 'yaw_free_objects', ())
        if round_object:
            # any angle only for a near top-down approach: turning a tilted grasp moves a fingertip towards the table,
            # while the 180 deg flip swaps two identical fingers
            approach = pose_to_matrix(self.to_planning_frame(grasp.grasp_pose).pose)[:3, 0]
            round_object = -approach[2] >= math.cos(math.radians(TURN_ANY_ANGLE_MAX_TILT_DEG))
        best = self.cable_guard.turn_for_slack(q, round_object)
        if best is not None and abs(abs(best[0]) - 180.0) < 1e-6 and has_twin(goal.possible_grasps, grasp, best[0]):
            rospy.loginfo(f'mtc: grasp {grasp.id}: its 180 deg twin is offered already, not replanning it turned')
            return None, None, None
        if best is None:
            rospy.loginfo(f'mtc: grasp {grasp.id}: no turn about its approach axis is predicted to give the cable slack')
            return None, None, None
        turned = turn_grasp_about_approach(grasp, best[0])
        rospy.loginfo(f'mtc: grasp {grasp.id}: trying it turned {best[0]:+.0f} deg about its approach axis '
                      f'({"round object" if round_object else "flip only"}; model: stretch {best[1]:+.3f})')
        try:
            task, roles = self.make_pick_task(goal, turned)
        except (tf2_ros.TransformException, ValueError) as e:
            rospy.logwarn(f'mtc: grasp {turned.id} skipped: {e}')
            return None, None, None
        if not self.plan(task, f'grasp {turned.id}'):
            return None, None, None
        return turned, task, roles

    # ------------------------------------------------------------------ task construction

    def make_pick_task(self, goal, grasp):
        obj = goal.target_name
        neighbours = self.touching_neighbours(obj, goal.support_surface_name) if rospy.get_param(
            '~mtc_lift_tolerate_touching_neighbours', False
        ) else []
        if neighbours:
            rospy.loginfo(f'mtc: lift of {obj} tolerates touching neighbours only: {neighbours}')
        task, roles = self.start_task(f'pick {obj} {grasp.id}')
        touch = self.gripper_links + [obj]
        # the gripper may touch the object only from the grasp pose on (MTC pick demo layout, #151): allowed from the
        # start, the free-space Connect to the pregrasp and the approach could sweep the open gripper through the
        # object (night10: a side grasp knocked the Pringles can off the table). ~mtc_allow_gripper_contacts_early
        # restores the old layout if the stricter check rejects too many grasps (e.g. padded object boxes).
        early = bool(rospy.get_param('~mtc_allow_gripper_contacts_early', False)) or not getattr(
            self, 'strict_contact_order', False)   # set per pick by pick.py: open-set grasps only

        before = stages.ModifyPlanningScene('allow octomap contact')
        if self.allow_octomap_contact:
            before.allowCollisions(OCTOMAP_COLLISION_NAME, touch, True)
        if early:
            before.allowCollisions(obj, self.gripper_links, True)
        task.add(before)
        roles.append(NOOP)

        task.add(self.connect('move to pregrasp'))
        roles.append(ARM)

        grasp_stages = core.SerialContainer('grasp')
        grasp_stages.add(self.move_relative('approach', grasp.pre_grasp_approach))
        roles.append(ARM)
        ik = self.compute_ik('grasp pose', self.to_planning_frame(grasp.grasp_pose), task['allow octomap contact'])
        ik_state = getattr(self, 'grasp_ik_states', {}).get(grasp.id)
        if ik_state is not None and speed_changes_on() and rospy.get_param('~mtc_pin_grasp_ik', True):
            # the IK solution pick.py's cable ranking judged (#151): MTC's own IK branch differed and failed the guard
            reference = {n: p for n, p in zip(ik_state.name, ik_state.position) if 'ur5_' in n}
            tolerance = math.radians(rospy.get_param('~mtc_pin_grasp_ik_tolerance_deg', 20.0))
            ik.setCostTerm(pinned_ik_cost(reference, tolerance))
            rospy.loginfo(f'mtc: grasp IK pin attached to grasp {grasp.id} (tolerance {math.degrees(tolerance):.0f} deg)')
        grasp_stages.add(ik)
        roles.append(NOOP)
        allow = stages.ModifyPlanningScene('allow gripper contacts')
        allow.allowCollisions(obj, self.gripper_links, True)
        grasp_stages.add(allow)
        roles.append(NOOP)
        grasp_stages.add(stages.ModifyPlanningScene('close gripper'))
        roles.append(GRIPPER)
        attach = stages.ModifyPlanningScene('attach object')
        attach.attachObject(obj, self.eef_link)
        if goal.support_surface_name:
            attach.allowCollisions(obj, [goal.support_surface_name], True)
        if neighbours:
            attach.allowCollisions(obj, neighbours, True)
        grasp_stages.add(attach)
        roles.append(ATTACH)
        retreat = copy.deepcopy(grasp.post_grasp_retreat)
        if neighbours:
            retreat.direction = Vector3Stamped(
                header=rospy.Header(frame_id=self.planning_frame), vector=Vector3(0.0, 0.0, 1.0)
            )
        grasp_stages.add(self.move_relative('lift', retreat))
        roles.append(ARM)
        if neighbours:
            block = stages.ModifyPlanningScene('forbid neighbour contacts after lift')
            block.allowCollisions(obj, neighbours, False)
            grasp_stages.add(block)
            roles.append(NOOP)
        task.add(grasp_stages)
        return task, roles

    def touching_neighbours(self, object_name, support_name):
        """World boxes already touching the target, with a 1 cm uncertainty margin.

        The support surface has its existing contact allowance. Unknown shapes
        and TF failures are not granted a new allowance.
        """
        request = GetPlanningSceneRequest()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        try:
            world = self.get_planning_scene_srv(request).scene.world
        except rospy.ServiceException as error:
            rospy.logwarn(f'mtc: cannot inspect touching neighbours of {object_name}: {error}')
            return []

        def boxes(collision_object):
            frame = collision_object.header.frame_id or self.planning_frame
            try:
                transform = (transform_to_matrix(self.tf_buffer.lookup_transform(
                    self.planning_frame, frame, rospy.Time(0), rospy.Duration(1.0)
                )) if frame != self.planning_frame else np.eye(4))
            except tf2_ros.TransformException as error:
                rospy.logwarn(f'mtc: cannot inspect {collision_object.id} in {frame}: {error}')
                return []
            transform = transform.dot(pose_to_matrix(collision_object.pose))
            return [
                (transform.dot(pose_to_matrix(pose)), primitive.dimensions[:3])
                for primitive, pose in zip(collision_object.primitives, collision_object.primitive_poses)
                if primitive.type == SolidPrimitive.BOX
            ]

        objects = {item.id: item for item in world.collision_objects}
        target = objects.get(object_name)
        if target is None:
            rospy.logwarn(f'mtc: {object_name} is absent from the planning scene; no neighbour tolerance')
            return []
        target_boxes = boxes(target)
        if not target_boxes:
            rospy.logwarn(f'mtc: {object_name} has no box in the planning scene; no neighbour tolerance')
            return []
        margin = 0.01
        return sorted(name for name, candidate in objects.items()
                      if name not in (object_name, support_name)
                      and any(boxes_touch(a_pose, a_size, b_pose, b_size, margin)
                              for a_pose, a_size in target_boxes for b_pose, b_size in boxes(candidate)))

    def make_place_task(self, goal, location, eef_to_object, retreat_min_distance=None, straight_up=False):
        '''place task for one location; with straight_up the retreat goes along planning frame +z instead of its
        own direction, and (None, None) is returned when that direction already is within 10 degrees of it'''
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
        if self.allow_octomap_contact and rospy.get_param('~mtc_place_octomap_blocks_transfer', True):
            # the octomap contact above is for the place pose and the final lowering only (the IK monitors 'allow
            # object contacts'): the transfer is planned in the scene before the connect, so forbidding it again here
            # makes 'move to preplace' avoid octomap obstacles with the held object and the gripper, and a pre-place
            # pose inside one infeasible (real 2026-09-28: a klt smashed into an unknown object on table_3)
            block = stages.ModifyPlanningScene('octomap blocks the transfer')
            block.allowCollisions(OCTOMAP_COLLISION_NAME, self.gripper_links + [obj], False)
            task.add(block)
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
        if straight_up:
            if self.world_direction(retreat.direction, eef_pose)[2] > math.cos(math.radians(10)):
                return None, None
            retreat.direction = Vector3Stamped(
                header=rospy.Header(frame_id=self.planning_frame), vector=Vector3(0.0, 0.0, 1.0)
            )
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

    def log_place_pose(self, location):
        '''log only (#202): where the held object is set down (planning frame, z includes the release clearance), so a
        place that ends with the object on the floor can be tied to a pose, a table edge distance and a yaw'''
        try:
            pose = self.to_planning_frame(location.place_pose)
        except (tf2_ros.TransformException, ValueError):
            return
        q = pose.pose.orientation
        yaw = math.degrees(tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2])
        rospy.loginfo(f'mtc: place location {location.id}: object set down at x {pose.pose.position.x:.3f} '
                      f'y {pose.pose.position.y:.3f} z {pose.pose.position.z:.3f} yaw {yaw:.0f} deg '
                      f'in {pose.header.frame_id}')

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

    def world_direction(self, direction, eef_pose):
        '''unit direction in the planning frame; a direction in the end effector link is taken at eef_pose'''
        if (direction.header.frame_id or self.planning_frame) == self.eef_link:
            vector = pose_to_matrix(eef_pose.pose)[:3, :3].dot(
                [direction.vector.x, direction.vector.y, direction.vector.z])
        else:
            v = self.direction_in_planning_frame(direction).vector
            vector = np.array([v.x, v.y, v.z])
        return vector / np.linalg.norm(vector)

    def octomap_place_check(self, object_name):
        '''
        ~mtc_place_reject_octomap_occupied (default on in the sim, off on the real robot until tested there): data to reject place locations whose footprint holds
        something only the octomap sees (real 2026-09-28: a klt placed into objects on table_1 and table_3, because the
        place pose and the lowering allow octomap contact for the held object): (occupied octomap leaves in the octomap
        frame, octomap frame, 4x4 box pose relative to the object frame, box half extents with the bottom raised by
        ~mtc_place_octomap_clearance and the sides grown by ~mtc_place_octomap_margin, default 0.05 m: in the sim a klt
        placed with a few cm of gap next to a can only the octomap saw brushed and pushed it on the way down, #185).
        None when off, or without an octomap or a box shape of the held object.
        '''
        if not rospy.get_param('~mtc_place_reject_octomap_occupied', rospy.get_param('/use_sim_time', False)):
            return None
        try:
            request = GetPlanningSceneRequest()
            request.components.components = (PlanningSceneComponents.OCTOMAP
                                             | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
            scene = self.get_planning_scene_srv(request).scene
        except rospy.ServiceException as e:
            rospy.logwarn(f'mtc: no octomap for the place check: {e}')
            return None
        octomap = scene.world.octomap
        if not octomap.octomap.data or octomap.octomap.binary:
            return None   # MoveIt sends the full format; a binary map has no occupancy values
        box = None
        for attached in scene.robot_state.attached_collision_objects:
            obj = attached.object
            if obj.id == object_name:
                for primitive, primitive_pose in zip(obj.primitives, obj.primitive_poses):
                    if primitive.type == SolidPrimitive.BOX:
                        box = (pose_to_matrix(primitive_pose), np.asarray(primitive.dimensions[:3], dtype=float))
                        break
        if box is None:
            rospy.logwarn(f'mtc: {object_name} has no box shape: place locations are not checked against the octomap')
            return None
        leaves = occupied_leaves(octomap.octomap.data, octomap.octomap.resolution,
                                 rospy.get_param('~mtc_place_octomap_threshold', 0.0))
        origin = pose_to_matrix(octomap.origin)
        if not np.allclose(origin, np.eye(4)):
            leaves = np.c_[(np.c_[leaves[:, :3], np.ones(len(leaves))] @ origin.T)[:, :3], leaves[:, 3]]
        clearance = rospy.get_param('~mtc_place_octomap_clearance', 0.03)
        box_pose, dims = box
        half = dims / 2.0
        # only voxels higher than clearance above the object's bottom count: the support surface itself is not one
        raise_bottom = np.eye(4)
        raise_bottom[2, 3] = clearance / 2.0
        half = half - np.array([0.0, 0.0, clearance / 2.0])
        # and voxels up to margin beside it: the held object and the gripper sweep past its final footprint
        margin = rospy.get_param('~mtc_place_octomap_margin', 0.05)
        half = half + np.array([margin, margin, 0.0])
        return leaves, octomap.header.frame_id, box_pose.dot(raise_bottom), half

    def octomap_blocks_place(self, check, location):
        '''number of occupied octomap leaves inside the held object's box at this place location (0 when fewer than
        ~mtc_place_octomap_min_voxels, or when the pose cannot be transformed)'''
        leaves, frame, box_pose, half = check
        pose = location.place_pose
        try:
            frame_to_pose = transform_to_matrix(
                self.tf_buffer.lookup_transform(frame, pose.header.frame_id or frame, rospy.Time(0), rospy.Duration(1.0))
            ) if pose.header.frame_id and pose.header.frame_id != frame else np.eye(4)
        except tf2_ros.TransformException as e:
            rospy.logwarn(f'mtc: place location {location.id}: no octomap check ({e})')
            return 0
        box_in_frame = frame_to_pose.dot(pose_to_matrix(pose.pose)).dot(box_pose)
        hits = count_in_box(leaves, box_in_frame, half)
        return hits if hits >= rospy.get_param('~mtc_place_octomap_min_voxels', 2) else 0

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
                except Exception as e:  # the model package is separate, it may be missing
                    rospy.logerr(f'mtc: cable guard requested but not available, planning without it: {e}')
                    return
            self.cable_guard = self._cable_guard_cache
        self.cable_guard.update_params()
        self.cable_solutions = max(1, int(rospy.get_param('~mtc_cable_solutions', 4)))

    def plan(self, task, what):
        '''plan task; sets self.chosen_solution (the cheapest solution, or with the cable guard the cheapest one
        whose joint path passes the cable check). With the guard one solution is planned, and a rejected one is not
        replanned for more (#151): asked for 4 at once, MTC searched until its stage timeouts for solutions that mostly
        do not exist (~9 s per grasp, suite 2026-09-28), and asked again after a reject it returned copies of the
        rejected path (same cost, stretch and worst point; real run A1 2026-09-28: ~9 s lost per grasp). The caller
        turns a rejected grasp about its approach axis (guard_rejected_q) or tries the next candidate instead.
        ~mtc_speed_changes false restores the old search for ~mtc_cable_solutions at once.'''
        start = time.monotonic()
        self.chosen_solution = None
        self.guard_rejected_q = None   # joint state (radians) of the least bad solution when the guard rejects all
        if self.cable_guard is None or speed_changes_on():
            count = 1
        else:
            count = self.cable_solutions   # the old search for all solutions at once
        try:
            task.init()
            planned = bool(task.plan(count)) and len(task.solutions) > 0
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
            least_bad, checks = float('inf'), []
            for index, solution in enumerate(task.solutions):
                ok, stretch, span, q = self.cable_guard.check(solution.toMsg())
                if not ok and span != 'wrist_3 window' and stretch < least_bad:
                    least_bad, self.guard_rejected_q = stretch, self.cable_guard.worst_q_rad
                checks.append(f'{index}: cost {solution.cost:.1f} stretch {stretch:+.3f} ({span})')
                if ok:
                    self.chosen_solution = solution
                    break
                where = self.where_in_task(task, self.cable_guard)
                self.cable_rejections = getattr(self, 'cable_rejections', 0) + 1
                rospy.loginfo(f'mtc: {what} solution {index} rejected by the cable guard{where}: stretch {stretch:+.3f} '
                              f'> {self.cable_guard.max_stretch:+.3f} in the {span} span at {q}')
            if self.chosen_solution is None:
                rospy.loginfo(f'mtc: {what}: all {len(task.solutions)} solutions entangle the arm cable '
                              f'({"; ".join(checks)}) in {elapsed:.1f} s')
                return False
            self.guard_rejected_q = None
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
    def leaf_stage_names(task):
        '''names of the leaf stages in solution order (one sub trajectory each), e.g. grasp/approach'''
        names = []

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
                names.append(prefix[:-1])

        index = 0
        while True:
            try:
                stage = task[index]
            except Exception:
                break
            index += 1
            visit(stage, stage.name + '/')
        return names

    @classmethod
    def where_in_task(cls, task, guard):
        '''" at stage <name> point i/n" for the guard's worst point (the geometric one), '' when unknown'''
        where = getattr(guard, 'worst_where', None)
        if where is None:
            return ''
        names = cls.leaf_stage_names(task)
        sub_index, point_index, points = where
        name = names[sub_index] if sub_index < len(names) else f'sub trajectory {sub_index}'
        return f' (worst point in {name}, point {point_index + 1}/{points})'

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
                posture = postures.pop(0)
                code = self.command_gripper(posture)
                if code == MoveItErrorCodes.SUCCESS and check_grasp and not postures:  # the closing step
                    if not self.closed_on_object(posture):
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
            from symbolic_fact_generation.common.lib import retarget_robot_names

            config = rospy.get_param('~mtc_gripper_fact_config', '') or os.path.join(
                rospkg.RosPack().get_path('symbolic_fact_generation'), 'config', 'facts_config.yaml'
            )
            with open(config) as f:
                facts = yaml.safe_load(f)
            entries = facts.get('facts', facts) if isinstance(facts, dict) else facts
            params = next(e['gripper_has_object']['params'] for e in entries if 'gripper_has_object' in e)
            robot_name = robot_prefix()
            params = retarget_robot_names(params, robot_name)
            rospy.loginfo(f'mtc: grasp check with gripper_has_object from {config} for {robot_name}')
            return GripperHasObjectGenerator('gripper_has_object', *params)
        except Exception as e:
            rospy.logerr(f'mtc: grasp check unavailable, picks are not verified: {e}')
            return None

    def gripper_holds_object(self, when, delay=None):
        if not rospy.get_param('~mtc_check_grasp', True):
            rospy.logwarn_once('mtc: ~mtc_check_grasp is false: picks and places do not check the gripper')
            return True
        if self.gripper_fact is None:
            return True
        if delay is None:
            delay = rospy.get_param('~mtc_grasp_check_delay', 1.0)
        rospy.sleep(delay)  # the fingers settle, the fact's effort hold and gOBJ catch up
        holds = bool(self.gripper_fact.generate_facts())
        rospy.loginfo(f'mtc: gripper_has_object {when}: {holds}')
        return holds

    def closed_on_object(self, close_posture):
        '''
        the check after closing. A per-object close width (gripper_distance_values_per_object.yaml, e.g. bleach
        0.045 m) can be reached with the object between the fingers but without stalling on it: soft plastic gives
        way, or the object is a little narrower there. The Robotiq then reports "reached the requested position" (gOBJ
        3) and the fact says empty, like a close on air (#188: the bleach bottle held twice on the real robot, both
        picks failed as "closed on nothing"). With ~mtc_grasp_check_squeeze such a grasp closes on to fully closed:
        an object stalls the fingers short of it and the fact confirms it, on air they close fully and the pick fails
        as before. Default on in the sim (/use_sim_time), off on the real robot until tested there (Oscar 2026-09-28).
        '''
        if self.gripper_holds_object('after closing'):
            return True
        if not rospy.get_param('~mtc_check_grasp', True) or self.gripper_fact is None:
            return True
        width = close_posture.points[-1].positions[0] if close_posture.points else 0.0
        if width <= GRASP_CHECK_SQUEEZE_MIN_WIDTH:
            return False  # already sent to fully closed: nothing between the fingers
        if not rospy.get_param('~mtc_grasp_check_squeeze', rospy.get_param('/use_sim_time', False)):
            return False
        rospy.logwarn(f'mtc: the gripper reached its {width:.3f} m close width without an object: closing fully to '
                      'check whether the object is between the fingers')
        squeeze = copy.deepcopy(close_posture)
        squeeze.points = [squeeze.points[-1]]
        squeeze.points[0].positions = [0.0] + list(squeeze.points[0].positions[1:])
        if self.command_gripper(squeeze) != MoveItErrorCodes.SUCCESS:
            return False
        return self.gripper_holds_object('after closing fully')

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
