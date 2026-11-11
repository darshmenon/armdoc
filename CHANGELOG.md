# Changelog

## 0.1.0
- First release: a read-only doctor for ros2_control / MoveIt 2 arms.
- Checks: `/joint_states` missing or stale, NaN joints, joint position, velocity and effort limits (from `/robot_description`, arm joints only), inactive arm controllers, missing `move_group`, trajectory tracking error, stalled trajectory goals, repeated goal aborts.
- Modes: live reports, `--once SECS` (exit 1 on ERROR, for CI), `--json`, `--ns`, `--moveit`.
- Verified on a UR3 in Gazebo (healthy, controller deactivated, sim paused); `test/fake_arm.py` fakes a graph for CI.
