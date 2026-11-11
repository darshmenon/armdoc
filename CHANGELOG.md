# Changelog

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
