# grasplan

Documentation can be found under: https://grasplan-documentation.readthedocs.io/en/latest/

Open-set grasp fallback
-----------------------

The pick action uses the configured Grasplan planner for objects present in
that planner's object catalog. If the object class is not in the catalog, the
goal is forwarded to an AnyGrasp server using the same `PickObjectAction`
interface. A failed Grasplan attempt for a known object is final and does not
trigger the fallback.

For the handcoded planner, known classes are the keys below the
`handcoded_grasp_planner_transforms` parameter. Numeric instance suffixes are
ignored, so (for example) `relay_3` is checked as class `relay`. The fallback
forwards the original object text unchanged.

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
- `grasp_type_hints_ns` (default `/mobipick/grasp_type_hints`; empty ignores hints): the planner may set
  `<ns>/<object key>` (the object name lowercased, spaces and dashes as underscores, e.g. `tomato_soup_can`) to
  `side`, `top` or `auto` before an open-set pick: `side` re-ranks the AnyGrasp grasps with the side-grasp bonus
  (`grasp_type_hint_side_multiplier`, default 1.5) across the vertical axis, `top` applies the low-object rule below
  whatever the box, `auto` or nothing lets the geometry decide
- `open_set_low_object_max_height` (default `0.05` m) and `open_set_small_object_max_footprint` (default `0.06` m, `0`
  = off): an object lower than the first, or with both horizontal sides below the second, is grasped from the top
  only: grasps more than `open_set_low_object_max_tilt_deg` (default `10`, `0` = rule off) off vertical are dropped,
  the others lowered to the fingertip clearance (`open_set_low_object_sink`, default `true`), else the top-down grasps
- `open_set_complete_one_sided_boxes` (default `true`): a standing box at least 1.5x taller than wide whose side along
  the line of sight from `open_set_camera_frame` is the shorter one (one view sees only the front of a can) gets its
  full depth behind the visible face, in the planning scene and for grasping
- `max_box_sink_into_support` (default `0.09` m, `0` = off): a perceived box whose bottom is deeper than this inside
  its support is left out of the planning scene as implausible (a false detection); the object being picked stays
- `drop_boxes_overlapping_target` (default `0.2`, `0` = off): a perceived box that shares at least this fraction of
  the smaller volume with the open-set target is left out of the planning scene (a false detection on the target)

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
