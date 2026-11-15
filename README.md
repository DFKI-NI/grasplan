# grasplan

Documentation can be found under: https://grasplan-documentation.readthedocs.io/en/latest/

Open-set grasp fallback
-----------------------

The pick action uses the configured Grasplan planner for objects present in
that planner's object catalog. If the object class is not in the catalog, the
goal is forwarded to an AnyGrasp server using the same `PickObjectAction`
interface. A failed Grasplan attempt for a known object is final and does not
trigger the fallback, unless `known_pick_open_set_fallback` is on (see below).

`known_pick_open_set_fallback` (default `false`, careful, read per goal): a known object whose stored grasps were all
refused in planning (MTC ended with "none of the N grasps could be planned") before any stage of a pick solution ran in
the goal (the goal-scoped latch of the MTC executor, set before every gripper command and pick motion) gets one second
try in the same goal with the open-set path above: the grasp view, a `GenerateGrasps` goal on the server's known-target
action (`anygrasp_generate_known_action_name`, default the generate action's name + `_known_target`: the server takes
exactly that pose selector entry as its target, segments the image in that entry's box and leaves the entry as it is; a
server without that action gets no second try), and the measured box as the planning object. The grasp finder's object
is picked only when all 8 corners of its measured box lie inside the known entry's box grown by
`known_pick_fallback_target_margin` (default `0.03` m) per side, in the frame of that box; the log says how far the
measured box reaches beyond the known one along each axis. A check of position and extent (a measured box that took in a
neighbour, a wall or the table spills out), not an identity proof: an object wholly inside that volume passes; a box measured around only part of the object can still spill out and be refused. The status text names both attempts. An object attached by the second try is noted in
`~open_set_fallback_held`, so the place and insert nodes handle it as an open-set object; a normal pick clears it. With
the flag off nothing of this runs and no parameter is written.

For the handcoded planner, known classes are the keys below the
`handcoded_grasp_planner_transforms` parameter. Numeric instance suffixes are
ignored, so (for example) `relay_3` is checked as class `relay`. The fallback
forwards the original object text unchanged.

An optional second grasp list per class, `handcoded_grasp_planner_extra_transforms` (same format), is used only with
`extra_grasp_sets` (default `false`, careful, read on every pick), and only after the whole pick with the stored grasps
(the single plan and the old way) failed in planning: then its grasps go through the same pipeline, with the
ids after the stored ones (`grasp_32` ... for 32 stored grasps). A pick the stored grasps can do never sees them; once
they are exhausted an extra grasp may run, also on an object the extras were not made for (for example a standing
can). The failure text of a pick that tried both is the one of the extras. Extras never make a class known. The arm may
already stand at its pre-pick pose (the untangle poses and the start move of the old way), as the stored grasps did;
any pick stage of the goal that ran rules the extras out: any gripper command of a pick solution (its pre-open too,
also when it fails; a single plan sends it before its untangle legs) and any arm motion from the move to the pregrasp
on; so do a detour after a refused pick and a cancel. The gripper opening before planning is no pick stage. Without
MTC (`use_mtc` false, MoveIt's pickup) only the planning and IK failure codes MoveIt reports let them run; that
path is not tested with extras.

The fallback is configured by these private parameters on `pick_object_node`:

- `anygrasp_action_name` (default `/mobipick/grasp_object`)
- `anygrasp_generate_action_name` (default `/mobipick/generate_grasps`)
- `anygrasp_handles_execution` (default `false`): when `false`, AnyGrasp only generates candidates and Grasplan executes them through MoveIt; set it to `true` to retain AnyGrasp-owned execution
- `anygrasp_server_timeout` (default `2.0` seconds)
- `anygrasp_result_timeout` (default `300.0` seconds; `0` waits indefinitely)
- `anygrasp_view_service` (default `/mobipick/grasp_view`): a `grasplan/ViewObject` service that moves the
  camera to a view of the object suited to AnyGrasp (sampled around its committed pose, the whole object within the
  depth range); when it is unavailable or fails (e.g. no committed pose: perceive the object first) the pick fails
- `anygrasp_named_view_fallback` (default `false`): `true` restores the old fallback to `anygrasp_arm_pose`
  (default `anygrasp`, an SRDF arm pose) when the view service is unavailable or fails
- `open_set_pick_start_pose` (default `transport`): after the grasp view, open-set picks move the arm to this SRDF
  pose (guarded named move) and plan the grasp from there instead of from the view; empty plans from the view
- `open_set_low_object_max_height` (default `0.05` m) and `open_set_small_object_max_footprint` (default `0.06` m, `0`
  = off): an object lower than the first, or with both horizontal sides below the second, is grasped from the top
  only: grasps more than `open_set_low_object_max_tilt_deg` (default `10`, `0` = rule off) off vertical are dropped,
  the others lowered to the fingertip clearance (`open_set_low_object_sink`, default `true`), else the top-down grasps
- `open_set_grasp_funnel_markers` (default `false`, read once per pick; debug output, no behaviour change): after each
  open-set filter step the surviving grasps are published as arrows on the latched MarkerArray `~grasp_funnel`, one
  namespace per step in this order: `candidates`, `low_object_kept`, `clearance_kept`, `ik_reachable` (an IK solution,
  also one in collision only), `cable_pass` (cable guard on: the leading grasps of `offered` that the cable ranking
  predicts to pass; MTC tries them first), `offered` (every grasp handed to MTC, at most
  `external_grasp_max_attempts`: the `cable_pass` ones, then the held-back rest, tried only after all of those failed),
  `topdown_fallback` (when used), `executed` (the grasp MTC ran), with a count label per step above the object
  (namespace `funnel_counts`). `cable_pass` ranks, it removes nothing, so `offered` can hold more grasps; the steps
  with their counts and grasp ids go as JSON to the latched `~grasp_funnel_summary` (std_msgs/String). The retry
  from the start pose is not shown
- `open_set_box_padding` (default `0.0` m = off): taken off every side of the externally detected target box (each
  side shorter by twice the value, at least 5 mm, centre kept) before it goes into the planning scene and the grasp
  geometry; the open-set detector pads every side of the box it reports, so an open fingertip right beside the object
  would already be inside it
- `open_set_align_jaw_to_box` (default `false`): a near-vertical open-set grasp whose jaw is more than 15 deg off
  both horizontal axes of the target box (a jaw across the box diagonal pinches two edges and the object slides out in
  the lift) gets a copy turned about its approach onto the nearest box axis whose side fits the open gripper (1 cm
  clearance per side) and moved along its jaw so the jaw's midline passes through the box centre, offered right before
  the original; round boxes are left alone
- `open_set_align_jaw_narrowest_side` (default `false`, with `open_set_align_jaw_to_box`): the first turned copy closes
  across the narrowest horizontal side of the target box that fits the open gripper instead of the nearest one, also
  for a grasp that already closes across a wider side; the nearest-side copy, when it is another side, comes second,
  then the original. Every grasp that then closes across that narrowest side, copy or original, is tried before all
  the others (each group in its quality order); the cable ranking still puts the grasps predicted to pass the cable
  guard first. It uses the detected box and the gripper's open distance only
- `open_set_strip_reported_padding` (default `false`): take off the padding the detection itself reports
  (`DetectedObject.box_padding`) instead of `open_set_box_padding`, so the value always matches the detector that
  produced the box; a detection that reports none (0) falls back to `open_set_box_padding`
- `open_set_complete_one_sided_boxes` (default `true`): a standing box at least 1.5x taller than wide whose side along
  the line of sight from `open_set_camera_frame` is the shorter one (one view sees only the front of a can) gets its
  full depth behind the visible face, in the planning scene and for grasping
- `max_box_sink_into_support` (default `0.09` m, `0` = off): a perceived box whose bottom is deeper than this inside
  its support is left out of the planning scene as implausible (a false detection); the object being picked stays
- `lift_sunk_target_to_support` (default `false`): a closed-set pick target whose box bottom reaches into its
  support (a pose estimate a few mm too low) is raised whole until its bottom is 1 mm above the support top, so its
  grasps and its planning scene box start from where it stands; a lift of at most `lift_sunk_target_max` (default
  `0.03` m) including the 1 mm clearance (a sink of up to 29 mm), a deeper one stays as perceived and is reported. Never lowers; tilted, floating and open-set targets are left as
  they are. Acts only with `clamp_object_boxes_to_support` on (it needs the support boxes). It changes where the
  gripper goes: the release at a place or insert is up to the lift lower than before
- `drop_boxes_overlapping_target` (default `0.2`, `0` = off): a perceived box that shares at least this fraction of
  the smaller volume with the open-set target is left out of the planning scene (a false detection on the target)
- `drop_unknown_copy_of_target` (default `false`, careful): for a closed-set pick, a pose selector entry whose class
  has no stored grasps and that may be the target itself, stored a second time by the search for unknown objects, is
  left out of the planning scene for this pick: each centre lies inside the other box, the boxes share at least half
  of the smaller volume and their volumes are within a factor two. Such a copy otherwise blocks every lift (the held
  object starts inside it). The entry stays out of the planning scene until the next goal builds the scene from the
  pose selector again (as the open-set path does with boxes on its target), whatever the end of this goal; the pose
  selector keeps it, and a successful pick says it was left out. Meanwhile named arm moves have no box for the part of
  a wrongly judged distinct object outside the target's box A heuristic: geometry cannot prove
  that two boxes are one object, and the node knows no container classes, so distinct contents filling most of a
  known box can be taken for a copy of it

On `place_object_node` and `insert_object_node` (with `disentangle_required`): an object picked open-set (its class is
not in the pick node's catalog `pick_grasp_catalog_param`, default
`/mobipick/pick_object_node/handcoded_grasp_planner_transforms`) skips the `poses_to_go_before_place` /
`poses_to_go_before_insert` untangle detour and moves to `open_set_place_start_pose` (default `transport`, empty = no
move) instead; `open_set_skip_untangle_detour: false` keeps the detour for every object.
- `external_grasp_ik_prefilter` (default `true`): before planning, drop AnyGrasp grasps whose grasp pose has no
  IK solution (`compute_ik`, 0.1 s each, no collision check); `external_grasp_max_attempts` (default `12`, `0` =
  all): then offer only the best N to the planner. A standing Pringles can once cost ~2 min of MTC on 50 grasps.
  The log gives the time of the pre-filter and of every batch.
- `anygrasp_use_gripper_width` (default `false`): when disabled, unknown
  objects use the fully closed `gripper_close` posture. When enabled, each
  AnyGrasp candidate's predicted jaw width is used instead.
- `anygrasp_gripper_width_offset` (default `0.0` metres): added to predicted
  widths when width control is enabled; use a small negative value to close
  farther. The result is clamped between `gripper_close` and `gripper_open`.
- `anygrasp_gripper_max_effort` (default `-1.0`): non-negative values override
  the grasp posture's effort limit for unknown objects. Negative values retain
  `gripper_joint_efforts`. Units and torque/force interpretation depend on the
  configured gripper controller.

With the Grasplan pick server and AnyGrasp server running, exercise the
fallback with an object description that is absent from the active planner's
catalog:

```bash
rosrun grasplan open_set_pick_obj_test_action_client "red mug"
```


pre-commit Formatting Checks
----------------------------

This repo has a [pre-commit](https://pre-commit.com/) check that runs in CI.
You can use this locally and set it up to run automatically before you commit
something. To install, use pip:

```bash
pip3 install --user pre-commit
```

To run over all the files in the repo manually:

```bash
pre-commit run -a
```

To run pre-commit automatically before committing in the local repo, install the git hooks:

```bash
pre-commit install
