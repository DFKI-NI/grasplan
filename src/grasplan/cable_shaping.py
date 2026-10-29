'''
The cable model shapes the arm's path before MoveIt plans it, behind the careful flag ~mtc_cable_path_shaping
(off in code); the MTC cable guard (mtc_pick_place.CableGuard) stays unchanged as the final check of the chosen path.

Sub-flags (read on every goal, effective only with the flag on): ~mtc_cable_path_waypoints (the model's waypoints),
~mtc_cable_corridor (wrist joint box on a free-planned segment), ~mtc_cable_path_branches (pick.py: the IK branch with
the most slack). The stretch tables (tools/cable_path.CableGrid) are built once per model config in a background
thread (about a minute) and cached in $ROS_HOME/grasplan/; until they exist, planning is unshaped.
'''
import math
import os
import threading
import time

import rospy
from moveit_msgs.msg import Constraints, JointConstraint

from grasplan.tools import cable_path as cp

URDF_LIMIT_FALLBACK = (-2 * math.pi, 2 * math.pi)


def shaping_on():
    return bool(rospy.get_param('~mtc_cable_path_shaping', False))


def sub_flag(name, default=True):
    return shaping_on() and bool(rospy.get_param(name, default))


def urdf_limits(urdf, joints, prefix=''):
    '''{short joint: (lower, upper)} from the robot description'''
    import xml.etree.ElementTree as ET
    root = ET.fromstring(urdf)
    out = {}
    for joint in root.iter('joint'):
        name = joint.get('name', '')
        short = cp.short_name(name)
        if short in joints and (not prefix or name.startswith(prefix)):
            limit = joint.find('limit')
            if limit is not None:
                out[short] = (float(limit.get('lower', URDF_LIMIT_FALLBACK[0])),
                              float(limit.get('upper', URDF_LIMIT_FALLBACK[1])))
    return out


def verified_path(grid, evaluate_max, start, goal, margin, offsets=(0.01, 0.02, 0.03), samples_per_rad=25.0):
    '''
    (waypoints, worst exact stretch, offset, line slack) from start to goal: the grid search with the margin lowered by
    each offset in turn (the grid interpolates between 10 deg nodes and may miss a peak), each result checked with the
    exact model evaluate_max(q) on joint-linear samples; the first that holds, else the least bad one; None when the
    grid finds no path. line slack: the straight line passed exactly (no waypoint needed).
    '''
    chain_cache = {}

    def exact(chain):
        key = tuple(tuple(round(q[j], 6) for j in sorted(q)) for q in chain)
        if key not in chain_cache:
            worst = -float('inf')
            for a, b in zip(chain, chain[1:]):
                span = max(abs(b[j] - a[j]) for j in grid.joints) if grid.joints else 0.0
                n = max(4, int(math.ceil(span * samples_per_rad)))
                for k in range(n + 1):
                    worst = max(worst, evaluate_max({j: a[j] + (b[j] - a[j]) * k / n for j in b if j in a}))
            chain_cache[key] = worst
        return chain_cache[key]

    line = exact([start, goal])
    if line < margin:
        return [], line, 0.0, True
    best = None
    for offset in offsets:
        found = cp.slack_path(grid, start, goal, margin - offset)
        if found is None:
            continue
        waypoints = found[0]
        worst = exact([start] + waypoints + [goal])
        if best is None or worst < best[1]:
            best = (waypoints, worst, offset, False)
        if worst < margin:
            break
    if best is not None and best[1] >= line:
        return [], line, 0.0, False   # the waypoints do not help: the straight line is no worse
    return best


# the guard passes a path only with stretch < its margin: a shaped path is called slack only this far below it
SLACK_CUSHION = 0.002


class ShapedPath:
    def __init__(self, waypoints, worst, offset, line_slack, last_box, search_ms):
        self.waypoints = waypoints      # joint dicts (short names), in order
        self.worst = worst              # exact geometric stretch along start -> waypoints -> goal
        self.offset = offset
        self.line_slack = line_slack
        self.last_box = last_box        # corridor of the last segment (short joint -> (lo, hi)) or None
        self.search_ms = search_ms

    def slack(self, margin):
        return self.worst < margin - SLACK_CUSHION

    def summary(self, margin):
        return (f'{len(self.waypoints)} waypoint(s), exact stretch {self.worst:+.3f} '
                f'({"<=" if self.worst <= margin else ">"} {margin:+.3f}), search {self.search_ms:.0f} ms'
                + (f', grid offset {self.offset:.2f}' if self.waypoints else ''))


class CablePathShaper:
    '''grid + search for one CableGuard (its model, config and wrist_3 window)'''

    def __init__(self, guard):
        self.guard = guard
        self.grid = None
        self.building = False
        self.lock = threading.Lock()
        model = guard.model
        self.span_joints = cp.model_span_joints(model)
        joints = [j for js in self.span_joints.values() for j in js]
        urdf = rospy.get_param('robot_description')
        prefix = getattr(guard.cfg, 'tf_prefix', '')
        self.limits = urdf_limits(urdf, set(cp.UR_JOINTS), prefix)
        w3 = cp.short_name(guard.wrist_3_joint)
        if w3 in self.limits:   # the grid covers only the cable window of wrist_3
            lo, hi = self.limits[w3]
            self.limits[w3] = (max(lo, guard.wrist_3_neutral - guard.half_window),
                               min(hi, guard.wrist_3_neutral + guard.half_window))
        self.window = {w3: self.limits.get(w3, URDF_LIMIT_FALLBACK)}
        import dataclasses
        try:
            cfg_dict = dataclasses.asdict(guard.cfg)
        except TypeError:
            cfg_dict = repr(guard.cfg)
        self.key = cp.grid_key(cfg_dict, urdf.replace(prefix, '') if prefix else urdf,
                               cp.GRID_STEP_DEG, {j: self.limits[j] for j in joints if j in self.limits})
        self.path = rospy.get_param('~mtc_cable_path_grid_file', '') or cp.default_cache_path(self.key)

    def ready(self):
        '''the grid, loading it or starting its build when missing; None until it exists'''
        with self.lock:
            if self.grid is not None:
                return self.grid
            if os.path.exists(self.path):
                try:
                    grid = cp.CableGrid.load(self.path)
                    if grid.key == self.key:
                        self.grid = grid
                        rospy.loginfo(f'mtc cable path: stretch grid {self.path} loaded')
                        return grid
                    rospy.logwarn(f'mtc cable path: {self.path} is for another model config, rebuilding')
                except Exception as e:  # noqa: BLE001 - a broken cache file is rebuilt
                    rospy.logwarn(f'mtc cable path: cannot read {self.path} ({e}), rebuilding')
            if not self.building:
                self.building = True
                threading.Thread(target=self._build, daemon=True).start()
            return None

    def _build(self):
        from mobipick_cable_entanglement.cable_model import CableModel
        import copy
        try:
            start = time.monotonic()
            cfg = copy.deepcopy(self.guard.cfg)
            cfg.cable_carrier.enabled = False
            model = CableModel(rospy.get_param('robot_description'), cfg)   # own instance: the guard's is not shared
            limits = {j: self.limits[j] for js in self.span_joints.values() for j in js}
            rospy.loginfo(f'mtc cable path: building the stretch grid ({cp.GRID_STEP_DEG:.0f} deg, spans '
                          f'{list(self.span_joints)}) in the background; planning stays unshaped until it is done')
            grid = cp.build_grid(cp.model_evaluator(model), self.span_joints, limits, cp.GRID_STEP_DEG,
                                 reference={'shoulder_pan': 0.0, 'shoulder_lift': -2.6, 'elbow': 2.43}, key=self.key)
            grid.save(self.path)
            with self.lock:
                self.grid = grid
            rospy.loginfo(f'mtc cable path: stretch grid built in {time.monotonic() - start:.0f} s -> {self.path}')
        except Exception as e:  # noqa: BLE001 - shaping stays off, planning goes on unshaped
            rospy.logerr(f'mtc cable path: building the stretch grid failed, planning unshaped: {e}')
        finally:
            self.building = False

    def evaluate_max(self, q):
        return self.guard.model.evaluate(q).max_stretch

    def evaluate_max_line(self, start, goal, samples=24):
        '''exact max stretch on the joint line start -> goal (dicts, any names)'''
        a = {cp.short_name(k): v for k, v in start.items()}
        b = {cp.short_name(k): v for k, v in goal.items()}
        return max(self.evaluate_max({j: a[j] + (b[j] - a[j]) * k / samples for j in b if j in a})
                   for k in range(samples + 1))

    def plan(self, start, goal, margin):
        '''ShapedPath from start to goal (joint dicts, any names, radians) or None (no grid, no path)'''
        grid = self.ready()
        if grid is None:
            return None
        t0 = time.monotonic()
        a = {cp.short_name(k): v for k, v in start.items() if cp.short_name(k) in cp.UR_JOINTS}
        b = {cp.short_name(k): v for k, v in goal.items() if cp.short_name(k) in cp.UR_JOINTS}
        if set(a) != set(b) or not set(grid.joints) <= set(a):
            return None
        found = verified_path(grid, self.evaluate_max, a, b, margin)
        if found is None:
            return None
        waypoints, worst, offset, line_slack = found
        last = waypoints[-1] if waypoints else a
        box = cp.corridor(grid, last, b, margin, float(rospy.get_param('~mtc_cable_corridor_pad_deg', 15.0)),
                          self.window)
        return ShapedPath(waypoints, worst, offset, line_slack, box, (time.monotonic() - t0) * 1000.0)

    def corridor_constraints(self, box, joint_prefix):
        '''moveit Constraints: one JointConstraint per wrist joint of the box (the plain wrist_3 window inside it)'''
        constraints = Constraints()
        constraints.name = 'arm cable: corridor'
        for joint, (lo, hi) in sorted(box.items()):
            constraints.joint_constraints.append(JointConstraint(
                joint_name=f'{joint_prefix}ur5_{joint}_joint', position=0.5 * (lo + hi),
                tolerance_above=0.5 * (hi - lo), tolerance_below=0.5 * (hi - lo), weight=1.0))
        return constraints


def full_names(q, joint_prefix):
    '''short joint dict -> MoveIt joint names of this robot'''
    return {f'{joint_prefix}ur5_{j}_joint': v for j, v in q.items()}


def degrees(q):
    return {cp.short_name(k): round(math.degrees(v), 1) for k, v in q.items()}
