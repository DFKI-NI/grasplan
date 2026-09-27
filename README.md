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
- `anygrasp_arm_pose` (default `anygrasp`): SRDF arm pose from which AnyGrasp sees the object
- `anygrasp_view_service` (default `/mobipick/grasp_view`): a `grasplan/ViewObject` service that moves the
  camera to a view of the object suited to AnyGrasp (e.g. the whole object within the depth range); when it is
  unavailable or fails, the arm goes to `anygrasp_arm_pose`; empty disables it
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
