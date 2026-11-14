# Changelog

## Unreleased
- armdoc_cpp, found by running it against a live UR3 Gazebo sim: (1) it reported a lag of 1.8 billion seconds because the sim stamps messages with sim time while the node compares against the wall clock; it now ignores gaps over 1e4 s like the Python tool. (2) It judged the Robotiq `finger_joint`, resting at its closed limit, as an arm joint (`NEAR_LIMIT`); it now asks `<ns>/controller_manager/list_controllers` which joints the arm controllers claim (same `is_arm_controller` rule and the same keep-the-last-set behaviour as the Python tool), or takes an `arm_joints` parameter. `is_arm_controller` is checked against Python in the equivalence test.
- Fix: deactivating the arm controller no longer widens the checks to every joint. An inactive controller claims no interfaces, so armdoc forgot which joints are the arm's and judged gripper fingers resting at their closed limit as arm joints (`NEAR_LIMIT`). Found by running armchaos against a Franka Panda; the last known arm joints are kept (`update_arm_joints`).
- Verified on two non-UR arms with the real ros2_control controller manager (mock hardware): FANUC (6 joints) and Franka Panda (7 joints), through armchaos, all expected codes raised.
- New `armdoc_cpp` colcon package (in this repo): `ros2 run armdoc_cpp armdoc_fast` is a C++ port of the live `/joint_states` checks (`JOINT_STATES_NONE`/`STALE`, `JOINT_NAN`, `LIMIT_POS`, `NEAR_LIMIT`, `LIMIT_VEL`, `EFFORT_SAT`, `NOISY_JOINTS`, `STREAM_LAG`), read-only, for leaving next to a fast controller. On a 50 Hz stream it used 0.9% of a core and 22 MB, against 5.5% and 65 MB for the Python tool (one 20 s run). Controller, MoveIt and action checks stay in the Python tool.
- `test/test_cpp_equivalence.py` feeds identical randomized `/joint_states` streams (healthy, noisy, limits, NaN, effort, lag, stale, joint subset, slow) to `armdoc/core.py` and the C++ core and requires the same findings, level, code and message.

## 0.3.0
- New `NOISY_JOINTS` check: position jitter estimated from the fourth difference of the joint positions over 1 s (`--noise-rad`, default 0.002 rad). Silent below 20 Hz.
- New `STREAM_LAG` check: `/joint_states` header stamp versus arrival time, median over 2 s (`--lag-max`, default 0.5 s). Silent when the stamps use another clock (set `use_sim_time` for sim stamps).
- With `--moveit`: `MOVEIT_PLAN_FAILED` (a `/move_action` goal aborted in the last 30 s) and `NO_PLANNING_SCENE` (`move_group` is up but no scene is published or served).
- Goals that finished before armdoc attached are no longer counted as new aborts (judged by the goal's stamp).
- `ros2 launch armdoc armdoc.launch.py` with `config/armdoc.yaml`; thresholds are ROS parameters of the node and win over the CLI flags. `--ros-args` is accepted on the command line.
- Dockerfile and `docker/` demo: a healthy fake arm, a stalled trajectory, and a fake MoveIt without a planning scene.
- `test/fake_arm.py --moveit [--no-scene] [--plan-fail]`; CI also checks the two MoveIt codes.
- Validated against armchaos `noise` and `lag` faults (`docs/e2e_*.txt`). A clean exit when launch interrupts the node.
- README quick start rewritten.

## 0.2.0
- `--record FILE.jsonl` logs timestamped joint positions and findings while armdoc runs; `armdoc report FILE.jsonl --html report.html --png report.png` renders a joint timeline with shaded finding bands, a controller lane and a findings table (self-contained HTML with hover and legend toggles, or a matplotlib PNG). The report step needs no ROS.
- `--ready [SECS]`: pre-flight mode. Watches SECS (default 10), prints a suggested next step under each finding and a `READY` / `NOT READY` verdict, exits 1 if not ready. `--hints` prints the same hints in normal mode.
- `--diagnostics`: opt-in, also publishes findings to `/diagnostics` (`diagnostic_msgs/DiagnosticArray`, one status per finding, or one OK status). It is the only thing armdoc ever publishes and is off by default.
- GitHub Actions CI: pytest on the diagnosis rules, plus a Humble job that runs `test/fake_arm.py --stall` and asserts `armdoc --once` exits 1 with `TRAJ_STALLED`.
- Verified on a UR3 in Gazebo (ros2_control): `--ready` and `--diagnostics` healthy and with the arm controller deactivated.

## 0.1.0
- First release: a read-only doctor for ros2_control / MoveIt 2 arms.
- Checks: `/joint_states` missing or stale, NaN joints, joint position, velocity and effort limits (from `/robot_description`, arm joints only), inactive arm controllers, missing `move_group`, trajectory tracking error, stalled trajectory goals, repeated goal aborts.
- Modes: live reports, `--once SECS` (exit 1 on ERROR, for CI), `--json`, `--ns`, `--moveit`.
- Verified on a UR3 in Gazebo (healthy, controller deactivated, sim paused); `test/fake_arm.py` fakes a graph for CI.
