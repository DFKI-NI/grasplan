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
- `anygrasp_server_timeout` (default `2.0` seconds)
- `anygrasp_result_timeout` (default `300.0` seconds; `0` waits indefinitely)


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
