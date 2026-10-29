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

Optional arm cable guard (~mtc_cable_plan_check, old name ~mtc_cable_constraints, re-read on every goal): the cable model of
mobipick_cable_entanglement (#118) turns into constraints on the joint path instead of joint limits that
reject grasps (#120, #124). Every free motion (Connect) gets a MoveIt path constraint that keeps wrist_3 within
half a turn of the angle where the tube is least wound (no extra winding), several solutions are planned, and
the cheapest one whose whole joint path keeps the cable model's stretch below ~mtc_cable_max_stretch is executed.
With ~mtc_cable_replay_live_state (careful flag, off in code) the path is also replayed through the monitor's carrier
link chain from the cable monitor's live chain, on top of the replay from a fresh chain: the path passes only when
both pass, so the live chain can refuse more paths but never pass one the fresh replay refuses. Where the monitor
starts its chain afresh at every trajectory start anyway (its reset_at_trajectory_start) the fresh replay alone runs.
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
from grasplan.tools.finger_series import analyse_close
from grasplan.tools.pad_gap import PINCH_GAP_RATIO, PINCH_MAX_TILT_DEG, is_pinch, overclose, pad_gap, pinch_checkable
from grasplan import cable_shaping
from grasplan.tools import cable_path as cp
from grasplan.tools.touching_boxes import boxes_touch
from grasplan.tools.stage_timing import timed
from grasplan.tools.moveit_errors import arm_stop_text, moveit_code_text

# the name MoveIt gives the octomap inside the planning scene (planning_scene::PlanningScene::OCTOMAP_NS)
OCTOMAP_COLLISION_NAME = '<octomap>'

# STOPGAP until the cable model refit (#118): default ~mtc_cable_max_stretch, sim and real. On the real robot the
# guarded plans that caught or nearly caught the tube had a wrist stretch of -0.108 / -0.103, all clean ones
# -0.126..-0.217 (#121), so the model's 0.0 (tube just taut) let tangling plans through. Revisit after the refit.
DEFAULT_CABLE_MAX_STRETCH = -0.115
# the margin with the support anchor (the sim's wrist anchor), behind the careful flag ~mtc_cable_support_margin. On
# the 629 real motions of 2026-09-27/28 the support anchor's geometric stretch of the five real cable stops lies at
# -6.4..-3.7 % (the closest 0.6 points above): -0.115 rejected 65 % of the safe motions, -0.07 rejects 9 % and still
# catches all five
SUPPORT_CABLE_MAX_STRETCH = -0.07

# a close width (m) above this is a per-object width short of fully closed, which ~mtc_grasp_check_squeeze may close
# on to check the grasp (#188); the klt and materialbox widths (0.0005) count as fully closed
GRASP_CHECK_SQUEEZE_MIN_WIDTH = 0.005

# pinch (m) the fingers may close past the object's width before the re-grasp fires
# (~grasp_regrasp_overclose, 20 mm: the pads sink 8-12 mm into a flat holding grasp in the sim, and of 16 Pringles picks
# those closed >= 19 mm past the can's diameter rotated it in the grasp 7 of 8 times; the first sim number, 15 mm, sat
# below that boundary); the deepest the re-grasp goes is ~grasp_regrasp_max_depth, set per grasp by pick.py.
REGRASP_THRESHOLD_M = 0.020

# what the executor does with each MTC sub trajectory, in the order the leaf stages were added
ARM, GRIPPER, ATTACH, DETACH, NOOP = 'arm', 'gripper', 'attach', 'detach', 'noop'
# a stage that only puts the plan's start where a StartMove ends; never executed, never replayed by the guard
PREFIX = 'prefix'


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
        if getattr(self.cfg, 'wrist_anchor', None) == 'support' and rospy.get_param('~mtc_cable_support_margin', False):
            # careful flag, off in code; only with the support anchor (the tip anchor keeps its own margin)
            self.max_stretch = rospy.get_param('~mtc_cable_support_max_stretch', SUPPORT_CABLE_MAX_STRETCH)
        self.half_window = math.radians(rospy.get_param('~mtc_cable_wrist_3_half_window_deg', 170.0))
        self.max_points = int(rospy.get_param('~mtc_cable_points_per_trajectory', 40))
        # careful flag, off in code: the path is also replayed from the monitor's live chain; both replays must pass
        self.replay_live_state = bool(rospy.get_param('~mtc_cable_replay_live_state', False))

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

    @timed('guard_check')
    def check(self, solution_msg, max_stretch=None, chain_stops=True):
        '''(ok, worst stretch, worst span, worst joint angles in degrees) along all sub trajectories; max_stretch
        overrides the planning margin ~mtc_cable_max_stretch (named-pose moves use the monitor's stop threshold);
        chain_stops False only logs a chain replay stop instead of rejecting the path'''
        limit = self.max_stretch if max_stretch is None else max_stretch
        worst, worst_span, worst_q = -float('inf'), '', {}
        self.worst_q_rad = None   # the state turn_for_slack() starts from; never one of an earlier solution
        lo, hi = self.wrist_3_neutral - self.half_window, self.wrist_3_neutral + self.half_window
        self.worst_where = None   # (sub trajectory index, point index, points) of the worst point, for the logs
        self.chain_stopped = False   # rejected by the chain replay alone (the cable replay detour reads it)
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
                self.chain_stopped = True
                return False, verdict.max_stretch, f'{verdict.span} ({verdict.limiting}, chain replay)', {
                    k.split('/')[-1]: round(math.degrees(v), 1) for k, v in verdict.q.items()}
        return worst < limit, worst, worst_span, worst_q

    @timed('guard_replay')
    def chain_verdict(self, solution_msg):
        '''the cable monitor's decision (disc chain with history, same model and thresholds) along the whole joint
        path of the solution, from a freshly settled chain at its start (mobipick_cable_entanglement.path_check);
        with ~mtc_cable_replay_live_state also from the monitor's live chain, which keeps what the motions before did:
        the path stops when either replay stops (the fresh one first), so the live chain never loosens the guard'''
        from mobipick_cable_entanglement.cable_model import CableModel
        from mobipick_cable_entanglement.path_check import carrier_replay
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
        state, origin = None, None
        if samples and getattr(self, 'replay_live_state', False):
            state, origin = self.live_chain_state(samples[0][1])
        # the fresh replay always runs (its origin line says why the live chain is unused, if it is); the live one on top
        runs = [(None, origin if state is None and origin else 'a fresh chain')] + ([(state, origin)] if state else [])
        verdicts = []
        for start_state, where in runs:
            start = time.monotonic()
            verdict = carrier_replay(self.chain_model, samples, self.chain_rate, start_state=start_state)
            if verdict is None:
                continue
            rospy.loginfo(f'mtc cable guard: chain replay of {verdict.updates} ticks in {time.monotonic() - start:.1f} s '
                          f'from {where}: max stretch '
                          f'{verdict.max_stretch:+.3f} ({verdict.limiting}), stop {verdict.stop}')
            verdicts.append(verdict)
        if not verdicts:
            return None
        if len(verdicts) == 2 and verdicts[0].stop != verdicts[1].stop:
            rospy.logwarn(f'mtc cable guard: the chain replays differ: the fresh chain '
                          f'{"refuses" if verdicts[0].stop else "passes"}, the live chain '
                          f'{"refuses" if verdicts[1].stop else "passes"} -> path refused (both must pass)')
        stopped = [v for v in verdicts if v.stop]
        return stopped[0] if stopped else max(verdicts, key=lambda v: v.max_stretch)

    def live_chain_state(self, q0):
        '''(chain state, where the replay starts) for a path starting at q0: the cable monitor's chain after its last
        tick (its chain_state service), or (None, a fresh chain and why) when the monitor resets its chain at every
        trajectory start (then the fresh chain is its model), or when the state is missing, stale, taken elsewhere or
        from another model config; the replay then starts as without the flag'''
        import json
        from std_srvs.srv import Trigger
        from mobipick_cable_entanglement.path_check import live_state_problem
        name = rospy.get_param('~mtc_cable_chain_state_service',
                               f'/{robot_prefix()}/cable_entanglement_monitor/chain_state')
        state = None
        try:
            rospy.wait_for_service(name, timeout=0.2)
            reply = rospy.ServiceProxy(name, Trigger)()
            state = json.loads(reply.message) if reply.success else None
            problem = None if reply.success else f'the monitor has none ({reply.message})'
        except (rospy.ROSException, rospy.ServiceException, ValueError) as e:
            problem = f'no cable monitor chain state at {name} ({e})'
        if problem is None:
            problem = live_state_problem(self.chain_model, state, q0, rospy.get_time())
        if problem is not None:
            log = rospy.loginfo_throttle if state and state.get('reset_at_trajectory_start') else rospy.logwarn_throttle
            log(60.0, f'mtc cable guard: no chain replay from the monitor\'s live chain, the fresh chain\'s alone: '
                      f'{problem}')
            return None, f'a fresh chain (live chain unused: {problem})'
        return state, f'the monitor\'s live chain ({rospy.get_time() - float(state["stamp"]):.2f} s old)'

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



def monitor_resets_at_trajectory_start():
    '''the cable monitor's careful flag cable_model/reset_at_trajectory_start: it starts its chain afresh at each
    announced executed trajectory, as the guard's replay of a trajectory does. Only then does a trajectory the guard
    passed from a fresh chain also pass the monitor; without it the monitor carries its chain from one trajectory to the
    next (sim 2026-10-27: after the detour leg left the chain nearly taut, the fresh-chain check passed the pick and the
    monitor stopped it)'''
    value = rospy.get_param(f'/{robot_prefix()}/cable_model/reset_at_trajectory_start', False)
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


_trajectory_start_pub = None


def announce_trajectory_start(source):
    '''
    tell the cable monitor that an executed trajectory starts now (oscar 2026-10-01): one per executed MTC
    solution (approach, grasp and lift together) and one per named move this module executes. With its careful flag
    cable_model/reset_at_trajectory_start the monitor starts its chain afresh here, as the guard's replay of the
    trajectory did (path_check.start_chain); without it nobody listens. Never keeps the arm from moving.
    '''
    global _trajectory_start_pub
    try:
        from std_msgs.msg import String

        if _trajectory_start_pub is None:
            _trajectory_start_pub = rospy.Publisher(
                f'/{robot_prefix()}/cable_model/trajectory_start', String, queue_size=10
            )
            rospy.sleep(0.5)  # the monitor connects to a new publisher asynchronously: do not lose the first one
        _trajectory_start_pub.publish(String(data=source))
    except Exception as e:  # noqa: BLE001 - the announcement is only for the monitor
        rospy.logwarn(f'cable trajectory start not announced ({source}): {e}')


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
    trajectory, _ = _plan_named_logged(arm, name, guard)
    if trajectory is None:
        return _plain_named_move(arm, name)
    announce_trajectory_start(f'named move {name}')
    return bool(arm.execute(trajectory, wait=True))


def _plan_named_logged(arm, name, guard):
    '''the log mode of guarded_named_move without the motion: one plan of arm's target, the guard's verdict only logged
    (guard None: not even that, the off mode). Returns (trajectory, '') or (None, 'no plan').'''
    planned = arm.plan()
    success, trajectory = (planned[0], planned[1]) if isinstance(planned, tuple) else (True, planned)
    if not success or not trajectory.joint_trajectory.points:
        return None, 'no plan'
    if guard is None:
        return trajectory, ''
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
    return trajectory, ''


def guarded_named_move(arm, name, guard, attempts=2, via=None, chain_stops=None, report=None):
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
    chain_stops: True/False decides for this move and its via legs whether a chain replay stop refuses a plan
    (the cable replay detour must not take a leg the chain replay would stop); None keeps
    ~mtc_cable_named_move_rope_stops. report: a dict that collects the guard's verdict per plan
    ('plans': [(pose, attempt, stretch, ok, span)]), the via pose taken ('via') and the last refusal ('reason').
    '''
    arm.set_named_target(name)
    mode = named_move_guard_mode() if guard is not None else 'off'
    if mode == 'log':
        return _logged_named_move(arm, name, guard)
    if mode == 'off':
        return _plain_named_move(arm, name)
    trajectory, reason = _plan_named_checked(arm, name, guard, attempts, _current_joint(guard.wrist_3_joint),
                                             chain_stops, report)
    if trajectory is not None:
        announce_trajectory_start(f'named move {name}')
        return bool(arm.execute(trajectory, wait=True))
    # still refused: go via an intermediate pose (~mtc_cable_named_move_via, e.g. transport) and try once more
    via = rospy.get_param('~mtc_cable_named_move_via', ['transport']) if via is None else via
    for pose in [p for p in via if p != name]:
        rospy.logwarn(f'arm move to {name} refused ({reason}); trying via {pose}')
        if report is not None:
            report['via'] = pose
        if (guarded_named_move(arm, pose, guard, attempts, via=[], chain_stops=chain_stops, report=report)
                and guarded_named_move(arm, name, guard, attempts, via=[], chain_stops=chain_stops, report=report)):
            return True
        reason = (report or {}).get('reason', reason)
    if report is not None:
        report['reason'] = reason
    rospy.logerr(f'not moving the arm to {name}: {reason}')
    return False


def _plan_named_checked(arm, name, guard, attempts, w3, chain_stops=None, report=None):
    '''the enforce mode of guarded_named_move without the motion: plan arm's target (set by the caller, from its start
    state) up to attempts times until the cable guard passes a plan; w3 is the start's wrist_3 angle (rad, None if
    unknown); chain_stops None: ~mtc_cable_named_move_rope_stops; report: see guarded_named_move. Returns
    (trajectory, '') or (None, reason).'''
    guard.update_params()
    if chain_stops is None:
        chain_stops = rospy.get_param('~mtc_cable_named_move_rope_stops', False)
    chain_stops = bool(chain_stops)
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
            if report is not None:
                report.setdefault('plans', []).append((name, attempt, stretch, ok, span))
            if ok:
                rospy.loginfo(f'moving arm to {name} (cable check passed: stretch {stretch:+.3f}, attempt {attempt})')
                return trajectory, ''
            reason = f'cable guard: stretch {stretch:+.3f} in the {span} span at {q}'
            rospy.logwarn(f'arm move to {name}: plan {attempt}/{attempts} rejected by the {reason}')
    finally:
        arm.clear_path_constraints()
    if report is not None:
        report['reason'] = reason
    return None, reason


def pick_start_move_carrier_stops():
    '''~pick_start_move_carrier_stops (careful, off in code): the pick's start move from the grasp view to its start
    pose refuses a plan the chain replay predicts to stop, replans, then detours via ~pick_start_move_via'''
    return bool(rospy.get_param('~pick_start_move_carrier_stops', False))


class MonitorStretch:
    '''the cable monitor's own ~max_stretch (Float32) while a move runs, for the log line next to the guard's
    replay verdict; subscribed on first use, None when the monitor publishes nothing (not running, real robot log mode)'''

    def __init__(self, topic):
        self.topic, self.sub, self.latest, self.peak = topic, None, None, None

    def start(self):
        if self.sub is None:
            try:
                from std_msgs.msg import Float32

                self.sub = rospy.Subscriber(self.topic, Float32, self._on, queue_size=10)
            except Exception as e:  # noqa: BLE001 - only for the log
                rospy.logwarn(f'cable monitor stretch not watched ({self.topic}): {e}')
        self.peak = self.latest

    def _on(self, msg):
        self.latest = float(msg.data)
        if self.peak is None or self.latest > self.peak:
            self.peak = self.latest


_monitor_stretch = None


def monitor_stretch():
    '''the MonitorStretch of this robot's cable monitor (one subscription per process)'''
    global _monitor_stretch
    if _monitor_stretch is None:
        _monitor_stretch = MonitorStretch(f'/{robot_prefix()}/cable_entanglement_monitor/max_stretch')
    return _monitor_stretch


def plan_pick_during_start_move():
    '''~plan_pick_during_start_move (careful, off in code): the closed-set pick is planned while the arm runs its
    untangle poses (see StartMove)'''
    return bool(rospy.get_param('~plan_pick_during_start_move', False))


class StartMove:
    '''
    (~plan_pick_during_start_move): the named poses a pick visits before its grasp (the closed-set untangle poses),
    each planned from where the one before ends and checked by the cable guard exactly like guarded_named_move, then run
    in a background thread, one execute_trajectory goal per pose with its own trajectory start for the cable monitor
    (the boundaries the guard checked, as the old pose-by-pose moves), so that MTC plans the pick meanwhile from end_q
    (a prefix stage in the task that is neither executed nor replayed by the guard). wait() returns once the arm is
    there; the plan is used only when the arm stands within tolerance of end_q, else it is planned again from the real
    state.
    '''

    MAX_TOLERANCE_RAD = 0.05   # a larger ~plan_pick_during_start_move_tolerance_rad is refused, not clipped

    def __init__(self, names, trajectories):
        self.names = list(names)
        self.trajectories = list(trajectories)
        last = self.trajectories[-1].joint_trajectory
        self.end_q = dict(zip(last.joint_names, last.points[-1].positions))
        self.runner, self.code, self.cancelled = None, None, False
        if not self.end_q or not all(math.isfinite(v) for v in self.end_q.values()):
            raise ValueError(f'start move to {self.names}: its end is not a complete finite joint state ({self.end_q})')
        self.client = None
        self.started = None

    @classmethod
    def plan(cls, arm, names, guard, attempts=2):
        '''the StartMove for names, or None (logged) when a pose cannot be planned and checked from the end of the one
        before: the caller then moves the old way (guarded_named_move, with its replans via transport)'''
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        mode = named_move_guard_mode() if guard is not None else 'off'
        trajectories, start = [], None
        try:
            for name in names:
                if start is None:
                    arm.set_start_state_to_current_state()
                    w3 = _current_joint(guard.wrist_3_joint) if guard is not None else None
                else:
                    arm.set_start_state(RobotState(joint_state=JointState(name=list(start), position=list(start.values())),
                                                   is_diff=True))
                    w3 = start.get(guard.wrist_3_joint) if guard is not None else None
                arm.set_named_target(name)
                if mode == 'enforce':
                    trajectory, reason = _plan_named_checked(arm, name, guard, attempts, w3)
                else:
                    trajectory, reason = _plan_named_logged(arm, name, guard if mode == 'log' else None)
                if trajectory is None:
                    rospy.logwarn(f'start move to {names}: {name}: {reason}; moving the old way, one pose after the other')
                    return None
                trajectories.append(trajectory)
                points = trajectory.joint_trajectory
                start = dict(zip(points.joint_names, points.points[-1].positions))
        finally:
            arm.set_start_state_to_current_state()
        try:
            return cls(names, trajectories)
        except ValueError as e:
            rospy.logwarn(f'{e}; moving the old way, one pose after the other')
            return None

    def start(self, client):
        '''run the poses one after the other on the execute_trajectory action client in a background thread; returns at
        once'''
        import threading
        self.client, self.started = client, time.monotonic()
        self.runner = threading.Thread(target=self.run, args=(client,), name='start move', daemon=True)
        self.runner.start()
        rospy.loginfo(f'start move to {self.names} running ({self.seconds():.1f} s of motion); planning the pick meanwhile')

    def run(self, client):
        '''the background part of start(): each pose as its own goal and trajectory start; stops at the first failure.
        Sets self.code (a MoveItErrorCodes value, None without a result).'''
        code = None
        for name, trajectory in zip(self.names, self.trajectories):
            if self.cancelled:
                break
            announce_trajectory_start(f'named move {name}')
            client.send_goal(ExecuteTrajectoryGoal(trajectory=trajectory))
            client.wait_for_result()
            result = client.get_result()
            code = result.error_code.val if result is not None else None
            if code != MoveItErrorCodes.SUCCESS:
                break
        self.code = code

    def seconds(self):
        return sum(t.joint_trajectory.points[-1].time_from_start.to_sec() for t in self.trajectories)

    @timed('start_move_wait')
    def wait(self, tolerance=None, plan_start=None):
        '''(arrived, reason): waits for the motion (bounded), then requires a complete, finite and fresh arm joint state
        within tolerance of end_q for every joint of the start move, and, when given, plan_start (the first point of the
        retained MTC arm trajectory, name -> rad) within tolerance of end_q and of that state. Differences are not
        wrapped: a full wrist turn is a different cable state. Anything missing or not finite drops the plan.'''
        if self.client is None:
            return False, 'the start move was never sent'
        client, self.client = self.client, None
        if tolerance is None:
            tolerance = rospy.get_param('~plan_pick_during_start_move_tolerance_rad', 0.01)
        try:
            tolerance = float(tolerance)
        except (TypeError, ValueError):
            tolerance = float('nan')
        if not (math.isfinite(tolerance) and 0.0 <= tolerance <= self.MAX_TOLERANCE_RAD):
            self.join(self.timeout_s())
            return False, f'invalid arrival tolerance {tolerance} rad (0 to {self.MAX_TOLERANCE_RAD})'
        if not self.join(self.timeout_s()):
            self.cancelled = True
            client.cancel_goal()
            return False, f'the start move to {self.names} did not end within {self.timeout_s():.0f} s (cancelled)'
        code = self.code
        if code != MoveItErrorCodes.SUCCESS:
            return False, f'the start move to {self.names} failed (MoveIt error code {code})'
        now = _current_arm_q(not_before=_ros_now())
        if now is None:
            return False, 'no fresh joint state after the start move'
        off, why = _joint_offset(self.end_q, now, 'the joint state')
        if why:
            return False, why
        if off > tolerance:
            return False, f'the arm stopped {math.degrees(off):.2f} deg from the end of the start move (tolerance ' \
                          f'{math.degrees(tolerance):.2f} deg)'
        if plan_start is not None:
            for reference, label in ((self.end_q, 'the end of the start move'), (now, 'the arm')):
                off, why = _joint_offset(reference, plan_start, 'the planned pick start')
                if why:
                    return False, why
                if off > tolerance:
                    return False, f'the pick was planned from {math.degrees(off):.2f} deg off {label} (tolerance ' \
                                  f'{math.degrees(tolerance):.2f} deg)'
        return True, ''

    def join(self, timeout):
        '''True when the background run ended within timeout seconds'''
        self.runner.join(timeout)
        return not self.runner.is_alive()

    def timeout_s(self):
        '''how long wait() waits for the motion: twice its duration plus 10 s'''
        return 2.0 * self.seconds() + 10.0


def _joint_offset(reference, actual, label):
    '''(largest |actual - reference| over every joint of reference, '') or (inf, why) when actual misses one of them or
    a value is not finite'''
    missing = [n for n in reference if n not in actual]
    if missing:
        return float('inf'), f'{label} lacks {", ".join(missing)}'
    bad = [n for n in reference if not (math.isfinite(reference[n]) and math.isfinite(actual[n]))]
    if bad:
        return float('inf'), f'{label} is not finite for {", ".join(bad)}'
    return max(abs(actual[n] - reference[n]) for n in reference), ''


def _ros_now():
    try:
        return rospy.Time.now()
    except rospy.exceptions.ROSInitException:   # no node (unit tests)
        return None


def _current_joint(joint, timeout=2.0):
    '''position (rad) of joint from one fresh joint_states message of this node's namespace, None if none came'''
    from sensor_msgs.msg import JointState
    try:
        msg = rospy.wait_for_message('joint_states', JointState, timeout=timeout)
    except rospy.ROSException:
        return None
    return dict(zip(msg.name, msg.position)).get(joint)


class FingerRecorder:
    '''collects (time, angle, effort) of one finger joint from joint_states until stop() (the closing as a
    measurement, see tools/finger_series.py)'''

    def __init__(self, joint):
        from sensor_msgs.msg import JointState
        self.joint = joint
        self.samples = []
        self.subscriber = rospy.Subscriber('joint_states', JointState, self.callback, queue_size=50)

    def callback(self, msg):
        if self.joint not in msg.name:
            return
        i = msg.name.index(self.joint)
        effort = msg.effort[i] if len(msg.effort) > i else None
        self.samples.append((msg.header.stamp.to_sec() or rospy.get_time(), msg.position[i], effort))

    def stop(self):
        self.subscriber.unregister()
        return self.samples


def _current_arm_q(timeout=2.0, not_before=None):
    '''the arm joints (name -> rad) of one fresh joint_states message of this node's namespace, None if none came;
    not_before (rospy.Time): None also for a message stamped before it (or not stamped)'''
    from sensor_msgs.msg import JointState
    deadline = time.monotonic() + timeout
    while True:
        try:
            msg = rospy.wait_for_message('joint_states', JointState, timeout=max(deadline - time.monotonic(), 0.05))
        except rospy.ROSException:
            return None
        arm = {n: p for n, p in zip(msg.name, msg.position) if 'ur5_' in n}
        if not_before is None or (not msg.header.stamp.is_zero() and msg.header.stamp >= not_before):
            return arm
        if time.monotonic() >= deadline:
            return None


class _SolutionView:
    '''a moveit_msgs/RobotTrajectory list shaped like an MTC solution message for CableGuard.check'''

    class _Sub:
        def __init__(self, trajectory):
            self.trajectory = trajectory

    def __init__(self, trajectories):
        self.sub_trajectory = [self._Sub(t) for t in trajectories]


class MtcPickPlace:
    start_move = None   # the StartMove the arm runs while the pick is planned (set by pick.py)

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
        self.cable_guard = None  # CableGuard, built on the first goal with ~mtc_cable_plan_check true
        self.cable_shaper = None  # cable_shaping.CablePathShaper of that guard, with ~mtc_cable_path_shaping
        self.shaped_path = None   # the ShapedPath of the task being built, None when unshaped
        self.no_shaping = False   # set while a grasp is replanned unshaped after its shaped plan failed
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
        self.failure_reason, self.executed, self.executed_grasp = '', False, None
        self.unplanned = []   # why each grasp could not be planned, for the failure reason
        self.chain_rejections = 0   # solutions the chain replay alone rejected in this pickup (cable replay detour)
        self.update_cable_guard()
        touch = self.gripper_links + [goal.target_name]
        for index, grasp in enumerate(goal.possible_grasps):
            if self.is_preempt_requested():
                result.error_code.val = MoveItErrorCodes.PREEMPTED
                return result
            self.last_plan_failure = ''
            try:
                task, roles = self.make_pick_task(goal, grasp)
            except (tf2_ros.TransformException, ValueError) as e:
                rospy.logwarn(f'mtc: grasp {grasp.id} skipped: {e}')
                self.unplanned.append(f'could not be set up ({e})')
                continue
            what = f'grasp {grasp.id} ({index + 1}/{len(goal.possible_grasps)})'
            planned = self.plan(task, what)
            if not planned and getattr(self, 'shaped_path', None) is not None:
                # the shaped plan failed (a waypoint or the interpolated last segment collides): the old way
                rospy.loginfo(f'mtc: {what}: the cable-shaped plan failed, replanning it unshaped')
                self.no_shaping = True
                try:
                    task, roles = self.make_pick_task(goal, grasp)
                    planned = self.plan(task, what + ' unshaped')
                finally:
                    self.no_shaping = False
            if not planned:
                self.log_failed_branch(grasp)
                task, roles, planned = self.next_branch_plan(goal, grasp, what)
            if not planned:
                grasp, task, roles = self.turned_grasp_plan(goal, grasp)
                if grasp is None:
                    self.unplanned.append(getattr(self, 'last_plan_failure', '') or 'not planned')
                    continue
            if self.start_move is not None:
                # planned while the arm ran its start move; use the plan only when the arm is where it starts
                arrived, why = self.finish_start_move(roles)
                if not arrived:
                    rospy.logwarn(f'mtc: {why}: planning the pick again from the current state')
                    return self.pickup(goal)
                rospy.loginfo(f'mtc: {what} was planned during the start move')
            # the pre grasp posture (open) is not a stage, it is sent before the first arm motion
            self.executed = True
            self.executed_grasp = grasp   # for the grasp funnel debug output of pick.py
            code = self.execute(task, roles, [grasp.pre_grasp_posture], grasp.grasp_posture,
                                attach=(goal.target_name, touch), check_grasp=True,
                                regrasp=self.regrasp_request(goal, grasp))
            if code == MoveItErrorCodes.SUCCESS and not self.gripper_holds_object('after the lift'):
                self.failure_reason = f'{goal.target_name} was lost during the lift (the gripper is empty)'
                code = MoveItErrorCodes.FAILURE
            if code != MoveItErrorCodes.SUCCESS and self.failure_reason:
                rospy.logerr(f'mtc: pick failed: {self.failure_reason}')
                self.release_empty_grasp(goal.target_name, grasp.pre_grasp_posture)
            elif code != MoveItErrorCodes.SUCCESS:
                self.open_after_failed_grasp(goal.target_name, grasp.pre_grasp_posture)
            result.error_code.val = code
            if code == MoveItErrorCodes.SUCCESS:
                result.grasp = grasp
            elif not self.failure_reason:
                self.failure_reason = f'executing grasp {grasp.id} failed: {self.execution_text(code)}'
            return result  # something moved: never try another grasp from a changed state
        if self.start_move is not None:
            # nothing planned from the start move's end (also when only its stand-in path failed): once the arm
            # is there, the old way, from the real state
            arrived, why = self.finish_start_move()
            rospy.logwarn(f'mtc: no grasp planned during the start move{f" ({why})" if why else ""}: planning the pick '
                          f'again from the current state')
            return self.pickup(goal)
        self.failure_reason = (f'none of the {len(goal.possible_grasps)} grasps could be planned'
                               + self.unplanned_breakdown())
        rospy.logerr(f'mtc: none of the {len(goal.possible_grasps)} grasps of {goal.target_name} could be planned'
                     f'{self.unplanned_breakdown()}')
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
        self.unplanned = []   # why each place location could not be planned, for the failure reason
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
            self.last_plan_failure = ''
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
                    self.last_plan_failure = f'could not be set up ({e})'
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
                    self.failure_reason = f'executing place location {location.id} failed: {self.execution_text(code)}'
                return result
            self.unplanned.append(getattr(self, 'last_plan_failure', '') or 'not planned')
        rospy.logerr(f'mtc: none of the {len(goal.place_locations)} place locations could be planned')
        self.failure_reason = (f'none of the {len(goal.place_locations)} place locations could be planned'
                               + self.unplanned_breakdown())
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

        self.ensure_shaping_ik(grasp)
        ik_state = getattr(self, 'grasp_ik_states', {}).get(grasp.id)
        shaped = self.shape_to(task, roles, ik_state, f'grasp {grasp.id}')
        task.add(self.connect('move to pregrasp', shaped) if shaped is not None else self.connect('move to pregrasp'))
        roles.append(ARM)

        grasp_stages = core.SerialContainer('grasp')
        grasp_stages.add(self.move_relative('approach', grasp.pre_grasp_approach))
        roles.append(ARM)
        ik = self.compute_ik('grasp pose', self.to_planning_frame(grasp.grasp_pose), task['allow octomap contact'])
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
        # what a re-grasp needs to plan the lift again from the arm's new state
        self.pick_tail = {'object': obj, 'support': goal.support_surface_name, 'neighbours': neighbours,
                          'retreat': retreat, 'touch': touch}
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
        if self.start_move is None:
            return task, [NOOP]
        # the arm is on its way to the start move's last pose; plan from there (a stand-in path, dropped)
        prefix = stages.MoveTo('start move (planned meanwhile)', self.pipeline)
        prefix.group = self.arm_group
        prefix.setGoal(self.start_move.names[-1])
        task.add(prefix)
        return task, [NOOP, PREFIX]

    def planned_start_q(self, timeout=2.0):
        '''the arm joints (name -> rad) the pick is planned from: the end of the running StartMove, else the
        current state'''
        if self.start_move is not None:
            return dict(self.start_move.end_q)
        return _current_arm_q(timeout=timeout)

    def without_prefix(self, solution_msg):
        '''the solution message with the stand-in path of the start move prefix emptied (same sub trajectory indices): what
        the arm will really run, from the start move's end on'''
        if self.start_move is None:
            return solution_msg
        msg = copy.deepcopy(solution_msg)
        for sub in msg.sub_trajectory[1:2]:
            sub.trajectory.joint_trajectory.points = []
        return msg

    def finish_start_move(self, roles=None):
        '''(arrived, reason) of the running StartMove, waiting for it; (True, '') when none runs. Clears it. roles: those
        of the planned task, whose first arm trajectory after the stand-in must start where the arm stands'''
        if self.start_move is None:
            return True, ''
        move, self.start_move = self.start_move, None
        if roles is None:
            return move.wait()
        plan_start = self.retained_start_q(roles)
        if plan_start is None:
            move.wait()
            return False, 'the planned pick has no arm trajectory to compare with the arrival'
        return move.wait(plan_start=plan_start)

    def retained_start_q(self, roles):
        '''the first point (name -> rad) of the first non-empty arm trajectory of the chosen solution after the start move
        stand-in: where the executed pick starts; None when there is none'''
        subs = list(self.chosen_solution.toMsg().sub_trajectory)
        for role, sub in list(zip(roles, subs))[roles.index(PREFIX) + 1 if PREFIX in roles else 0:]:
            trajectory = sub.trajectory.joint_trajectory
            if role == ARM and trajectory.points:
                return dict(zip(trajectory.joint_names, trajectory.points[0].positions))
        return None

    def connect(self, name, shaped=None):
        '''
        the free-space Connect; shaped (a cable_shaping.ShapedPath to this Connect's goal): when its checked path
        is slack, the Connect only interpolates joints (the executed path is the checked one, ~mtc_cable_path_interpolate
        _last), else the planner gets the model's wrist corridor instead of the wide wrist_3 window
        (~mtc_cable_corridor)
        '''
        margin = self.cable_guard.max_stretch if getattr(self, 'cable_guard', None) is not None else None
        if shaped is not None and margin is not None and shaped.slack(margin) and rospy.get_param(
                '~mtc_cable_path_interpolate_last', True):
            connect = stages.Connect(name, [(self.arm_group, core.JointInterpolationPlanner())])
            connect.timeout = self.connect_timeout
            rospy.loginfo(f'mtc: {name}: joint interpolation along the cable-checked path')
            return connect
        connect = stages.Connect(name, [(self.arm_group, self.pipeline)])
        connect.timeout = self.connect_timeout
        if self.cable_guard is not None:
            if shaped is not None and shaped.last_box and cable_shaping.sub_flag('~mtc_cable_corridor'):
                connect.properties['path_constraints'] = self.cable_shaper.corridor_constraints(
                    shaped.last_box, self.joint_prefix())
                rospy.loginfo(f'mtc: {name}: planner in the cable corridor '
                              + ', '.join(f'{j} {math.degrees(lo):.0f}..{math.degrees(hi):.0f}'
                                          for j, (lo, hi) in sorted(shaped.last_box.items())))
            else:
                connect.properties['path_constraints'] = self.cable_guard.path_constraints()
        return connect

    def joint_prefix(self):
        return self.cable_guard.wrist_3_joint[: -len('ur5_wrist_3_joint')] if self.cable_guard is not None else ''

    @timed('cable_shape')
    def shape_to(self, task, roles, goal_state, what):
        '''
        (~mtc_cable_path_shaping, careful, off in code): the cable model's path from the current arm state to
        goal_state (sensor_msgs/JointState, e.g. the pinned grasp IK); adds one joint-interpolated MoveTo stage per
        model waypoint (~mtc_cable_path_waypoints) to task and returns the ShapedPath for the Connect, None when
        unshaped (flag off, no grid yet, no path, no goal state)
        '''
        self.shaped_path = None
        if (goal_state is None or getattr(self, 'no_shaping', False) or getattr(self, 'cable_guard', None) is None
                or getattr(self, 'cable_shaper', None) is None or not cable_shaping.shaping_on()):
            return None
        start = self.planned_start_q()
        goal = {n: p for n, p in zip(goal_state.name, goal_state.position) if 'ur5_' in n}
        margin = self.cable_guard.max_stretch
        try:
            shaped = self.cable_shaper.plan(start or {}, goal, margin)
        except Exception as e:  # noqa: BLE001 - shaping must never break a pick: plan unshaped
            rospy.logwarn(f'mtc: cable path to {what} failed ({e}), planning unshaped')
            return None
        if shaped is None:
            rospy.loginfo(f'mtc: cable path to {what}: none (grid not ready or no slack path), planning unshaped')
            return None
        if not cable_shaping.sub_flag('~mtc_cable_path_waypoints') and shaped.waypoints:
            rospy.loginfo(f'mtc: cable path to {what}: {len(shaped.waypoints)} waypoint(s) found but '
                          f'~mtc_cable_path_waypoints is off')
            shaped.waypoints, shaped.worst = [], self.cable_shaper.evaluate_max_line(start, goal)
        rospy.loginfo(f'mtc: cable path to {what}: {shaped.summary(margin)}; start {cable_shaping.degrees(start)} '
                      f'goal {cable_shaping.degrees(goal)}'
                      + ''.join(f'; waypoint {i + 1} {cable_shaping.degrees(w)}' for i, w in enumerate(shaped.waypoints)))
        for i, waypoint in enumerate(shaped.waypoints):
            move = stages.MoveTo(f'cable waypoint {i + 1}', core.JointInterpolationPlanner())
            move.group = self.arm_group
            move.setGoal(cable_shaping.full_names(waypoint, self.joint_prefix()))
            task.add(move)
            roles.append(ARM)
        self.shaped_path = shaped
        return shaped

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
        '''(re)read ~mtc_cable_plan_check on every goal, so the guard can be switched without a restart; the old
        name ~mtc_cable_constraints still works when the new one is not set'''
        if not rospy.get_param('~mtc_cable_plan_check', rospy.get_param('~mtc_cable_constraints', False)):
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
        if cable_shaping.shaping_on():
            if getattr(self, 'cable_shaper', None) is None or self.cable_shaper.guard is not self.cable_guard:
                try:
                    self.cable_shaper = cable_shaping.CablePathShaper(self.cable_guard)
                except Exception as e:  # noqa: BLE001 - shaping is optional, the guard goes on alone
                    rospy.logerr(f'mtc: cable path shaping requested but not available: {e}')
                    self.cable_shaper = None
            if self.cable_shaper is not None:
                self.cable_shaper.ready()   # loads the grid or starts its build in the background

    @timed('cable_branch')
    def least_stretch_branch(self, ik, request, solution, start, grasp_id):
        '''
        (~mtc_cable_path_branches under ~mtc_cable_path_shaping, careful, off in code): the IK branch of this grasp
        pose with the most cable slack. The other UR branches are found by seeding compute_ik near each of them
        (cp.ur_branch_seeds; the polished IK plugin returns the branch nearest its seed), deduplicated and
        ranked by the cable grid (goal slack first, then the slack path's worst stretch from start). Returns solution
        unchanged when off, without a grid or when no other branch is better.
        '''
        if not cable_shaping.sub_flag('~mtc_cable_path_branches'):
            return solution
        self.update_cable_guard()
        shaper, guard = getattr(self, 'cable_shaper', None), getattr(self, 'cable_guard', None)
        grid = shaper.ready() if shaper is not None else None
        if grid is None or guard is None:
            return solution
        try:
            started = time.time()
            arm = [n for n in solution.name if 'ur5_' in n]
            short = {cp.short_name(n): n for n in arm}
            first = {cp.short_name(n): p for n, p in zip(solution.name, solution.position) if n in arm}
            branches = [first]
            for seed in cp.ur_branch_seeds(first, shaper.limits)[1:]:
                req = copy.deepcopy(request)
                names, positions = list(req.ik_request.robot_state.joint_state.name), \
                    list(req.ik_request.robot_state.joint_state.position)
                for j, v in seed.items():
                    if short.get(j) in names:
                        positions[names.index(short[j])] = v
                req.ik_request.robot_state.joint_state.position = positions
                response = ik(req)
                if response.error_code.val == MoveItErrorCodes.SUCCESS:
                    js = response.solution.joint_state
                    branches.append({cp.short_name(n): p for n, p in zip(js.name, js.position) if n in arm})
            branches = cp.dedupe_branches(branches)
            start_q = {cp.short_name(n): p for n, p in zip(start.name, start.position) if n in arm}
            ranked = cp.rank_branches(grid, start_q, branches, guard.max_stretch)
            best = ranked[0][0]
            # the next ranked branches, for MTC to fall through to when the chosen one cannot be planned
            branch_states = []
            for row in ranked:
                state = copy.deepcopy(solution)
                state.position = [row[0][cp.short_name(n)] if n in arm else p
                                  for n, p in zip(solution.name, solution.position)]
                branch_states.append(state)
            self.grasp_ik_branches = dict(getattr(self, 'grasp_ik_branches', None) or {})
            self.grasp_ik_branches[grasp_id] = branch_states
            rospy.loginfo(f'cable branches of grasp {grasp_id}: {len(branches)} found in {time.time() - started:.2f} s; '
                          + '; '.join(f'{"chosen " if r[0] is best else ""}goal {r[1]:+.3f} path {r[2]:+.3f}'
                                      f'{"" if r[3] is None else f" ({len(r[3])} wp)"}' for r in ranked))
            return solution if best is first else branch_states[0]
        except Exception as e:  # noqa: BLE001 - the branch choice is optional: keep compute_ik's branch
            rospy.logwarn(f'cable branches of grasp {grasp_id}: {e}; keeping the first IK branch')
            return solution

    def ensure_shaping_ik(self, grasp):
        '''
        a grasp without a pinned IK state (closed-set picks: the pre-filter runs for open-set grasps only) gets one
        when the branch choice is on: compute_ik of its grasp pose seeded with the current state, then the branch with
        the most cable slack; it is pinned like the pre-filter's (grasp_ik_states), so MTC plans to it
        '''
        states = getattr(self, 'grasp_ik_states', None)
        if states is None:
            states = self.grasp_ik_states = {}
        if grasp.id in states or not cable_shaping.sub_flag('~mtc_cable_path_branches'):
            return
        from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
        from sensor_msgs.msg import JointState
        try:
            current = rospy.wait_for_message('joint_states', JointState, timeout=2.0)
            if self.start_move is not None:   # seeded where the pick will start, not where the arm is now
                current.position = [self.start_move.end_q.get(n, p) for n, p in zip(current.name, current.position)]
            ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
            ik.wait_for_service(2.0)
            request = GetPositionIKRequest()
            request.ik_request.group_name = self.arm_group
            request.ik_request.robot_state.joint_state = current
            request.ik_request.avoid_collisions = False   # the fingers touch the object at the grasp pose
            request.ik_request.ik_link_name = self.eef_link
            request.ik_request.pose_stamped = grasp.grasp_pose
            request.ik_request.timeout = rospy.Duration(float(rospy.get_param('~external_grasp_ik_timeout', 0.05)))
            response = ik(request)
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                rospy.loginfo(f'mtc: cable branches of grasp {grasp.id}: no IK ({response.error_code.val}), MTC IK as before')
                return
            states[grasp.id] = self.least_stretch_branch(ik, request, response.solution.joint_state, current, grasp.id)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn(f'mtc: cable branches of grasp {grasp.id}: compute_ik unavailable ({e}), MTC IK as before')

    def next_branch_plan(self, goal, grasp, what):
        '''
        when the pinned IK branch of grasp could not be planned, the next branches pick.py ranked by cable slack
        (grasp_ik_branches, ~mtc_cable_path_branches), each pinned and planned in turn (shaped, else unshaped);
        (task, roles, planned)
        '''
        branches = list(getattr(self, 'grasp_ik_branches', {}).get(grasp.id, ()))[1:]
        states = getattr(self, 'grasp_ik_states', {})
        pinned = states.get(grasp.id)
        try:
            for k, state in enumerate(branches, start=2):
                rospy.loginfo(f'mtc: {what}: trying the next IK branch by cable slack ({k}/{len(branches) + 1})')
                states[grasp.id] = state
                task, roles = self.make_pick_task(goal, grasp)
                if self.plan(task, f'{what} branch {k}'):
                    pinned = state   # keep the branch that planned
                    return task, roles, True
                if getattr(self, 'shaped_path', None) is not None:
                    self.no_shaping = True
                    try:
                        task, roles = self.make_pick_task(goal, grasp)
                        if self.plan(task, f'{what} branch {k} unshaped'):
                            pinned = state
                            return task, roles, True
                    finally:
                        self.no_shaping = False
            return None, None, False
        finally:
            if pinned is not None:
                states[grasp.id] = pinned

    def log_failed_branch(self, grasp):
        '''on a failed grasp, the start and the pinned target branch, so a log tells reachability
        from cable trouble (the failing stages and guard verdicts are logged by plan())'''
        ik_state = getattr(self, 'grasp_ik_states', {}).get(grasp.id)
        start = self.planned_start_q(timeout=0.5)
        target = {n: p for n, p in zip(ik_state.name, ik_state.position) if 'ur5_' in n} if ik_state else None
        rospy.loginfo(f'mtc: grasp {grasp.id} not planned: start {cable_shaping.degrees(start or {})}, '
                      f'pinned target {cable_shaping.degrees(target) if target else "none (MTC IK)"}')

    @staticmethod
    def motion_seconds(solution, skip=0):
        '''summed duration of the arm sub trajectories of an MTC solution (s), without its first skip ones'''
        total = 0.0
        for sub in solution.toMsg().sub_trajectory[skip:]:
            points = sub.trajectory.joint_trajectory.points
            if points:
                total += points[-1].time_from_start.to_sec()
        return total

    @timed('plan')
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
            self.last_plan_failure = f'could not be set up ({e})'
            return False
        elapsed = time.monotonic() - start
        if not planned:
            rospy.loginfo(f'mtc: {what} not feasible ({elapsed:.1f} s): {self.describe_failures(task)}')
            self.last_plan_failure = self.first_failure(task)
            return False
        if self.cable_guard is None:
            self.chosen_solution = task.solutions[0]
        else:
            least_bad, checks = float('inf'), []
            for index, solution in enumerate(task.solutions):
                ok, stretch, span, q = self.cable_guard.check(self.without_prefix(solution.toMsg()))
                if not ok and span != 'wrist_3 window' and stretch < least_bad:
                    least_bad, self.guard_rejected_q = stretch, self.cable_guard.worst_q_rad
                checks.append(f'{index}: cost {solution.cost:.1f} stretch {stretch:+.3f} ({span})')
                if ok:
                    self.chosen_solution = solution
                    break
                where = self.where_in_task(task, self.cable_guard)
                self.cable_rejections = getattr(self, 'cable_rejections', 0) + 1
                if getattr(self.cable_guard, 'chain_stopped', False):
                    self.chain_rejections = getattr(self, 'chain_rejections', 0) + 1
                rospy.loginfo(f'mtc: {what} solution {index} rejected by the cable guard{where}: stretch {stretch:+.3f} '
                              f'> {self.cable_guard.max_stretch:+.3f} in the {span} span at {q}')
            if self.chosen_solution is None:
                self.last_plan_failure = f'refused by the cable guard ({span} span)'
                rospy.loginfo(f'mtc: {what}: all {len(task.solutions)} solutions entangle the arm cable '
                              f'({"; ".join(checks)}) in {elapsed:.1f} s')
                return False
            self.guard_rejected_q = None
            rospy.loginfo(f'mtc: {what} cable check passed ({checks[-1]})')
        elapsed = time.monotonic() - start
        try:
            motion = f', motion {self.motion_seconds(self.chosen_solution, 2 if self.start_move is not None else 0):.1f} s'
        except Exception:  # noqa: BLE001 - log only
            motion = ''
        rospy.loginfo(f'mtc: {what} planned in {elapsed:.1f} s (cost {self.chosen_solution.cost:.2f}{motion})')
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

    def unplanned_breakdown(self):
        '''": 15 failed at grasp/compute ik (eef in collision), 3 refused by the cable guard (wrist span)" from
        self.unplanned, most frequent first; '' when nothing was recorded'''
        counts = {}
        for why in getattr(self, 'unplanned', []):
            counts[why] = counts.get(why, 0) + 1
        if not counts:
            return ''
        return ': ' + ', '.join(f'{n} {why}' for why, n in sorted(counts.items(), key=lambda item: -item[1]))

    @staticmethod
    def first_failure(task):
        '''"failed at <stage> (<first failure comment>)" for the first leaf stage (in solution order) that found
        no solution, else the one with the most failures; "not planned" when MTC kept no failures'''
        leaves = []

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
                try:
                    failures = list(stage.failures)
                    leaves.append((prefix[:-1], len(stage.solutions), failures))
                except Exception:  # noqa: BLE001 - a stage without introspection
                    pass

        index = 0
        while True:
            try:
                stage = task[index]
            except Exception:
                break
            index += 1
            visit(stage, stage.name + '/')
        failing = [leaf for leaf in leaves if leaf[2]]
        if not failing:
            return 'not planned'
        name, _, failures = next((leaf for leaf in failing if leaf[1] == 0), max(failing, key=lambda leaf: len(leaf[2])))
        comment = next((f.comment for f in failures if f.comment), '')
        if len(comment) > 100:
            comment = comment[:97] + '...'
        return f'failed at {name}' + (f' ({comment})' if comment else '')

    def execution_text(self, code):
        '''Why the executed solution failed: what execute() recorded, else the MoveIt code in words'''
        return getattr(self, 'execution_failure', '') or moveit_code_text(code)

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

    @timed('execute')
    def execute(self, task, roles, postures_before, stage_posture, attach=None, detach=None, check_grasp=False,
                regrasp=None):
        '''
        run the first solution of task. roles has one entry per leaf stage in the order they were added.
        postures_before are gripper postures (trajectory_msgs/JointTrajectory, opening in metres) sent before
        anything else, stage_posture is the one for the GRIPPER stage
        '''
        self.execution_failure = ''   # set by the step that fails
        sub_trajectories = list(self.chosen_solution.toMsg().sub_trajectory)
        if len(sub_trajectories) != len(roles):
            self.execution_failure = (f'the planned solution has {len(sub_trajectories)} motions where '
                                      f'{len(roles)} were expected, so it was not executed')
            rospy.logerr(
                f'mtc: solution has {len(sub_trajectories)} sub trajectories, expected {len(roles)} '
                f'({roles}); not executing'
            )
            return MoveItErrorCodes.FAILURE
        postures = list(postures_before) + [stage_posture]
        steps = [(GRIPPER, None)] * len(postures_before) + list(zip(roles, sub_trajectories))
        try:
            stage_names = [''] * len(postures_before) + self.leaf_stage_names(task)
        except Exception:  # noqa: BLE001 - names only label the failure
            stage_names = []
        # how far a pick got, for pickup(): closed on the object, attached to the scene, lifted
        self.pick_progress = {'closed': False, 'attached': False, 'lifted': False}
        announce_trajectory_start('mtc solution')  # the whole solution is one executed trajectory for the cable monitor
        for step, (role, sub) in enumerate(steps):
            if self.is_preempt_requested():
                return MoveItErrorCodes.PREEMPTED
            if role == ARM:
                if not sub.trajectory.joint_trajectory.points:
                    continue
                code = self.execute_arm(sub.trajectory)
            elif role == GRIPPER:
                posture = postures.pop(0)
                recorder = self.start_close_recording(regrasp) if check_grasp and not postures else None
                code = self.command_gripper(posture)
                if recorder is not None:
                    self.log_close_series(regrasp, recorder.stop())
                if code == MoveItErrorCodes.SUCCESS and check_grasp and not postures:  # the closing step
                    if not self.closed_on_object(posture):
                        self.failure_reason = f'the gripper closed on nothing at {attach[0] if attach else "the object"}'
                        return MoveItErrorCodes.FAILURE
                    self.pick_progress['closed'] = True
                    if regrasp is not None and regrasp.get('pinch_check') and self.pinched(regrasp):
                        return MoveItErrorCodes.FAILURE   # failure_reason says why; the caller opens the gripper
                    if regrasp is not None and regrasp.get('regrasp', True) and self.regrasp_after_overclose(regrasp, posture):
                        if not self.closed_on_object(posture):   # the second close, as the first
                            self.failure_reason = f'the gripper closed on nothing after the re-grasp at {regrasp["object"]}'
                            return MoveItErrorCodes.FAILURE
                        # the arm moved deeper: the attach and lift of the solution start from the old grasp pose, so the
                        # controller would refuse them (start deviates, CONTROL_FAILED -4); plan them again from here
                        return self.execute_tail(attach)
            elif role == ATTACH:
                code = self.attach(*attach)
                if code == MoveItErrorCodes.SUCCESS:
                    self.pick_progress['attached'] = True
            elif role == DETACH:
                code = self.detach(detach)
            elif role == PREFIX:
                continue   # the start move already ran this part
            else:
                if sub.trajectory.joint_trajectory.points:
                    rospy.logerr('mtc: unexpected motion in a planning scene stage, aborting')
                    return MoveItErrorCodes.FAILURE
                continue
            if code != MoveItErrorCodes.SUCCESS:
                if self.execution_failure and step < len(stage_names) and stage_names[step]:
                    self.execution_failure += f' (stage {stage_names[step]})'
                return code
            if role == ARM and self.pick_progress['attached']:
                self.pick_progress['lifted'] = True
        return MoveItErrorCodes.SUCCESS

    def regrasp_request(self, goal, grasp):
        '''
        what the re-grasp after over-closing needs for this grasp, or None: neither ~grasp_regrasp_on_overclose
        (the code default is off) nor the diagnostic ~grasp_regrasp_log_only (measure and log, never move) is on, or no
        width for the grasp (pick.py sets regrasp_info per grasp: the AnyGrasp opening, the box width along the finger
        axis, the width the rule uses, and the deepest the re-grasp may go; below 5 mm = shallow by design)
        '''
        log_only = bool(rospy.get_param('~grasp_regrasp_log_only', False))
        regrasp = bool(rospy.get_param('~grasp_regrasp_on_overclose', False)) or log_only
        pinch_check = bool(rospy.get_param('~grasp_standing_tilt_band', False))
        if not regrasp and not pinch_check:
            return None
        info = getattr(self, 'regrasp_info', {}).get(grasp.id)
        if not info or not (info.get('width') or info.get('width_open') or info.get('width_box')):
            rospy.loginfo(f'mtc: regrasp: {goal.target_name} grasp {grasp.id}: no width, no re-grasp')
            return None
        return {'object': goal.target_name, 'grasp_id': grasp.id, 'width': info.get('width'),
                'width_open': info.get('width_open'), 'width_box': info.get('width_box'),
                'width_extent': info.get('width_extent'), 'tilt_deg': info.get('tilt_deg'),
                'disagree': info.get('disagree'),
                'source': info.get('source'), 'confidence': info.get('confidence'),
                'max_depth': info.get('max_depth', 0.0),
                'reason': info.get('reason', ''), 'open_set': info.get('open_set', True), 'log_only': log_only,
                'regrasp': regrasp, 'pinch_check': pinch_check,
                'open_posture': grasp.pre_grasp_posture, 'approach': grasp.pre_grasp_approach}

    def start_close_recording(self, request):
        '''the recorder of the closing, only when ~grasp_regrasp_log_only is on (a diagnostic, no motion), else None'''
        if request is None or not request.get('log_only'):
            return None
        try:
            return FingerRecorder(rospy.get_param('~grasp_regrasp_finger_joint',
                                                  f'{robot_prefix()}/gripper_finger_joint'))
        except Exception as error:   # a diagnostic never breaks the pick
            rospy.logwarn(f'mtc: stall: no finger recording ({error})')
            return None

    def log_close_series(self, request, samples):
        '''one line per close (log only): the finger angle where the fingers first stopped (contact), at the end and
        the creep between, the pad gap at both, the effort, the tilt of the pad closing axis of the grasp (a tilted grasp
        reads wider than the object by design) and the perceived widths for reference'''
        mm = lambda value: 'n/a' if value is None else f'{value * 1000:.0f} mm'   # noqa: E731
        try:
            result = analyse_close(samples)
            if result is None:
                rospy.loginfo(f"mtc: stall: {request['object']} grasp {request['grasp_id']}: {len(samples)} finger "
                              'samples, no analysis')
                return
            creep = pad_gap(result['contact']) - pad_gap(result['final'])
            effort = 'n/a' if result['effort'] is None else f"{result['effort']:.1f}"
            peak = 'n/a' if result['peak_effort'] is None else f"{result['peak_effort']:.1f}"
            tilt = request.get('tilt_deg')
            rospy.loginfo(
                f"mtc: stall: {request['object']} grasp {request['grasp_id']} contact_theta {result['contact']:.3f} "
                f"contact_gap {mm(pad_gap(result['contact']))} final_theta {result['final']:.3f} "
                f"final_gap {mm(pad_gap(result['final']))} creep {creep * 1000:.1f} mm "
                f"contact_after {result['contact_time']:.2f} s effort {effort} peak_effort {peak} samples {result['n']} "
                f"tilt {'n/a' if tilt is None else f'{tilt:.1f}'} deg width_open {mm(request.get('width_open'))} "
                f"width_box {mm(request.get('width_box'))} extent_y {mm(request.get('width_extent'))}")
        except Exception as error:   # a diagnostic never breaks the pick
            rospy.logwarn(f'mtc: stall: could not analyse the close ({error})')

    def finger_angle(self):
        '''finger joint angle (rad) now, None when no joint state comes'''
        return _current_joint(rospy.get_param('~grasp_regrasp_finger_joint', f'{robot_prefix()}/gripper_finger_joint'))

    def pinched(self, request):
        '''
        careful flag ~grasp_standing_tilt_band, after the close held: the pads sit on a rim or a corner when the
        finger gap is below PINCH_GAP_RATIO (0.7, ~grasp_pinch_gap_ratio) of the object's width across the pads (the box
        width of the grasp, the reading the re-grasp uses; open-set grasps only: a closed-set close is bounded by its
        configured width). True sets failure_reason ("pinch (gap X of width Y)"); the failed close opens the gripper
        like a close on nothing. Every check is one log line. A grasp whose closing axis is tilted more than
        ~grasp_pinch_max_tilt_deg (25, the band edge) out of the horizontal is not checked (log line): the box length along
        a tilted axis is no width across the pads (a can at 53 deg read 111 mm for 66 mm and a good grasp failed).
        '''
        width = request.get('width_box')
        if not request.get('open_set', True) or not width:
            return False
        tilt, max_tilt = request.get('tilt_deg'), rospy.get_param('~grasp_pinch_max_tilt_deg', PINCH_MAX_TILT_DEG)
        if not pinch_checkable(tilt, max_tilt):
            rospy.loginfo(f"mtc: pinch check skipped: {request['object']} grasp {request['grasp_id']} closing axis "
                          f"tilted {tilt:.0f} deg (limit {max_tilt:.0f}), width_box {width * 1000:.0f} mm is along it")
            return False
        theta = self.finger_angle()
        if theta is None:
            return False
        gap = pad_gap(theta)
        ratio = rospy.get_param('~grasp_pinch_gap_ratio', PINCH_GAP_RATIO)
        pinch = is_pinch(gap, width, ratio)
        rospy.loginfo(f"mtc: pinch check: {request['object']} grasp {request['grasp_id']} gap {gap * 1000:.0f} mm "
                      f"width_box {width * 1000:.0f} mm ratio {gap / width:.2f} (limit {ratio:.2f}) conf "
                      f"{request.get('confidence')}: {'PINCH' if pinch else 'ok'}")
        if pinch:
            self.failure_reason = (f"pinch (gap {gap * 1000:.0f} of width {width * 1000:.0f} mm): the fingertips hold "
                                   f"{request['object']} at a rim, not across it")
        return pinch

    def regrasp_after_overclose(self, request, close_posture):
        '''
        careful flag ~grasp_regrasp_on_overclose, after the close held: compare the finger angle with the one
        flat pads would show on an object of the perceived width (pad gap table of the real robot's URDF). The width is
        the AnyGrasp opening only when it agrees with the box width along the finger axis (within 10 mm: the opening
        carries a margin of 10-30 mm over the object, T2 2026-10-01); the agreed width is the smaller one. Closed
        ~grasp_regrasp_overclose (0.020 m) or more past it: open, move deeper along the grasp axis by
        min(excess, max depth of this grasp), close again; once per pick, whatever the second excess is. Skipped: fully
        closed (nothing between the fingers), widths disagree, shallow by design (max depth < 5 mm), closed-set grasps
        (measured only). ~grasp_regrasp_log_only: every check is logged with both width inputs and nothing moves.
        Returns True when it fired. Every check is one log line (the tester's counts come from these).
        '''
        mm = lambda value: 'n/a' if value is None else f'{value * 1000:.0f} mm'   # noqa: E731
        what = (f"regrasp: {request['object']} grasp {request['grasp_id']} width_perc {mm(request['width'])} "
                f"width_open {mm(request['width_open'])} width_box {mm(request['width_box'])} "
                f"extent_y {mm(request.get('width_extent'))} src {request.get('source')} conf {request.get('confidence')}")
        theta = self.finger_angle()
        if theta is None:
            rospy.logwarn(f'mtc: {what}: no finger angle, no check')
            return False
        gap = pad_gap(theta)
        excesses = ' '.join(f'excess_{name} {(overclose(theta, request[key]) * 1000):+.0f} mm'
                            for name, key in (('open', 'width_open'), ('box', 'width_box'), ('extent', 'width_extent'))
                            if request.get(key))
        measured = f'theta {theta:.3f} gap {gap * 1000:.0f} mm {excesses}'
        threshold = rospy.get_param('~grasp_regrasp_overclose', REGRASP_THRESHOLD_M)
        if request['log_only']:
            rospy.loginfo(f'mtc: {what} {measured}: log only, nothing moves')
            return False
        if request.get('confidence') == 'low':
            rospy.logwarn(f'mtc: {what} {measured}: skipped, shape unknown (confidence low), no re-grasp')
            return False
        if request['width'] is None or request['disagree']:
            rospy.logwarn(f'mtc: {what} {measured}: skipped, widths disagree (open {mm(request["width_open"])}, '
                          f'box {mm(request["width_box"])}) or no width')
            return False
        excess = overclose(theta, request['width'])
        measured = f'theta {theta:.3f} gap {gap * 1000:.0f} mm excess {excess * 1000:+.0f} mm'
        if gap < 0.005 or excess < threshold:
            rospy.loginfo(f'mtc: {what} {measured}: not fired (threshold {threshold * 1000:.0f} mm)')
            return False
        if not request['open_set']:
            rospy.logwarn(f'mtc: {what} {measured}: closed-set grasp, no re-grasp (check the yaml close width)')
            return False
        depth = min(excess, request['max_depth'])
        if request['max_depth'] < 0.005:
            rospy.logwarn(f"mtc: regrasp: skipped shallow by design: {request['object']} grasp {request['grasp_id']} "
                          f"{measured}: max depth {request['max_depth'] * 1000:.0f} mm, reason {request['reason']}")
            return False
        rospy.logwarn(f"mtc: {what} {measured}: FIRED max_depth {request['max_depth'] * 1000:.0f} mm "
                      f"({request['reason']}): opening, {depth * 1000:.0f} mm deeper")
        moved = 0.0
        if self.command_gripper(request['open_posture']) == MoveItErrorCodes.SUCCESS and self.move_deeper(request, depth):
            moved = depth
        else:
            rospy.logwarn(f'mtc: {what}: the deeper move failed, closing again where it stands')
        self.command_gripper(close_posture)
        rospy.sleep(rospy.get_param('~mtc_grasp_check_delay', 1.0))
        again = self.finger_angle()
        second = (f'theta2 {again:.3f} gap2 {pad_gap(again) * 1000:.0f} mm excess2 '
                  f'{overclose(again, request["width"]) * 1000:+.0f} mm') if again is not None else 'no second angle'
        rospy.logwarn(f'mtc: {what} {measured} moved {moved * 1000:.0f} mm -> {second}')
        self.regrasp_fired = getattr(self, 'regrasp_fired', 0) + 1
        return True

    def execute_tail(self, attach):
        '''
        after a re-grasp: plan the attach and the lift again from the arm's current state, with the lift of the
        original pick (same retreat, straight up when neighbours touch), and run them. The arm stands deeper than the
        grasp pose the original solution was planned from, so its remaining sub trajectories would start off the
        current state. Returns a MoveItErrorCodes value.
        '''
        tail = getattr(self, 'pick_tail', None)
        if tail is None:
            self.failure_reason = 'no lift to plan again after the re-grasp'
            return MoveItErrorCodes.FAILURE
        obj, neighbours = tail['object'], tail['neighbours']
        task, roles = self.start_task('lift after regrasp')
        if self.allow_octomap_contact:
            before = stages.ModifyPlanningScene('allow octomap contact')
            before.allowCollisions(OCTOMAP_COLLISION_NAME, tail['touch'], True)
            task.add(before)
            roles.append(NOOP)
        allow = stages.ModifyPlanningScene('allow gripper contacts')
        allow.allowCollisions(obj, self.gripper_links, True)
        task.add(allow)
        roles.append(NOOP)
        attach_stage = stages.ModifyPlanningScene('attach object')
        attach_stage.attachObject(obj, self.eef_link)
        if tail['support']:
            attach_stage.allowCollisions(obj, [tail['support']], True)
        if neighbours:
            attach_stage.allowCollisions(obj, neighbours, True)
        task.add(attach_stage)
        roles.append(ATTACH)
        task.add(self.move_relative('lift', tail['retreat']))
        roles.append(ARM)
        if neighbours:
            block = stages.ModifyPlanningScene('forbid neighbour contacts after lift')
            block.allowCollisions(obj, neighbours, False)
            task.add(block)
            roles.append(NOOP)
        if not self.plan(task, 'lift after the re-grasp'):
            self.failure_reason = f'the lift of {obj} could not be planned again after the re-grasp'
            return MoveItErrorCodes.PLANNING_FAILED
        for role, sub in zip(roles, self.chosen_solution.toMsg().sub_trajectory):
            if role == ARM and sub.trajectory.joint_trajectory.points:
                code = self.execute_arm(sub.trajectory)
            elif role == ATTACH:
                code = self.attach(*attach)
                if code == MoveItErrorCodes.SUCCESS:
                    self.pick_progress['attached'] = True
            else:
                continue
            if code != MoveItErrorCodes.SUCCESS:
                return code
            if role == ARM:
                self.pick_progress['lifted'] = True
        return MoveItErrorCodes.SUCCESS

    def move_deeper(self, request, depth):
        '''plan and run a straight move of the open gripper along the grasp approach by depth (m); False if it cannot'''
        task, roles = self.start_task('regrasp deeper')
        allow = stages.ModifyPlanningScene('allow gripper contacts')
        allow.allowCollisions(request['object'], self.gripper_links, True)
        task.add(allow)
        roles.append(NOOP)
        deeper = copy.deepcopy(request['approach'])
        deeper.min_distance = 0.5 * depth
        deeper.desired_distance = depth
        task.add(self.move_relative('deeper', deeper))
        roles.append(ARM)
        if not self.plan(task, f'regrasp deeper by {depth * 1000:.0f} mm'):
            return False
        for role, sub in zip(roles, self.chosen_solution.toMsg().sub_trajectory):
            if role == ARM and sub.trajectory.joint_trajectory.points:
                return self.execute_arm(sub.trajectory) == MoveItErrorCodes.SUCCESS
        return False

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

    @timed('hold_check')
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

    def open_after_failed_grasp(self, object_name, open_posture):
        '''
        a pick that failed AFTER the gripper closed on the object and BEFORE the arm lifted it (e.g. the lift was refused,
        MoveIt error -4): the object still stands on its support, so open the gripper here, over the table, before the
        caller retracts the arm. Retracting with the closed gripper carried the object away and dropped it wherever the
        gripper opened next (a sim test: the can ended on the robot's deck). The object stays in the scene (detached if it
        was attached); it is not removed as for a grasp on nothing.
        '''
        progress = getattr(self, 'pick_progress', None) or {}
        if not progress.get('closed') or progress.get('lifted'):
            return
        rospy.logwarn(f'mtc: pick of {object_name} failed after the close and before the lift: opening the gripper '
                      'where the object stands, before the arm retracts')
        if progress.get('attached'):
            self.detach(object_name)
        if open_posture is not None:
            self.command_gripper(open_posture)

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

    @timed('arm_move')
    def execute_arm(self, robot_trajectory):
        self.execute_client.send_goal(ExecuteTrajectoryGoal(trajectory=robot_trajectory))
        while not self.execute_client.wait_for_result(rospy.Duration(0.1)):
            if rospy.is_shutdown() or self.is_preempt_requested():
                self.execute_client.cancel_goal()
                self.execute_client.wait_for_result(rospy.Duration(2.0))
                return MoveItErrorCodes.PREEMPTED
        code = self.execute_client.get_result().error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.execution_failure = f'the arm motion failed with {moveit_code_text(code)}{arm_stop_text()}'
            rospy.logerr(f'mtc: arm trajectory execution failed: {self.execution_failure}')
        return code

    @timed('gripper')
    def command_gripper(self, posture):
        '''send the single point posture of a Grasp/PlaceLocation (opening in metres) to the GripperCommand server'''
        if not self.gripper_client.wait_for_server(rospy.Duration(2.0)):
            self.execution_failure = f'the gripper action server {self.gripper_action_name} is not available'
            rospy.logerr(f'mtc: {self.execution_failure}')
            return MoveItErrorCodes.CONTROL_FAILED
        point = posture.points[-1]
        goal = GripperCommandGoal()
        goal.command.position = point.positions[0]
        goal.command.max_effort = point.effort[0] if point.effort else 0.0
        rospy.loginfo(f'mtc: gripper to {goal.command.position:.3f} m (max effort {goal.command.max_effort})')
        self.gripper_client.send_goal(goal)
        if not self.gripper_client.wait_for_result(rospy.Duration(self.gripper_action_timeout)):
            self.gripper_client.cancel_goal()
            self.execution_failure = f'the gripper did not finish its command within {self.gripper_action_timeout} s'
            rospy.logerr(f'mtc: {self.execution_failure}')
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
