# armdoc

**Why isn't my arm moving?** Attach to a running ros2_control / MoveIt 2 arm and get a one-line cause. Read-only: it subscribes to topics and lists controllers, and never publishes or changes anything.

```bash
ros2 run armdoc armdoc                     # live, a report every 2 s
ros2 run armdoc armdoc --ns /ur3 --moveit  # namespaced arm, also require move_group
ros2 run armdoc armdoc --once 10           # watch 10 s, print one report, exit 1 on any ERROR (for CI)
```

```
ERROR [CTRL_INACTIVE] controller arm_controller (joint_trajectory_controller/JointTrajectoryController) is inactive, not active
ERROR [TRAJ_STALLED] a trajectory goal has been active for 10s but the joints moved 0.0000 rad
```

## What it checks

| Code | Level | Means |
|---|---|---|
| `JOINT_STATES_NONE` / `JOINT_STATES_STALE` | warn / error | no `/joint_states`, or it went silent (broadcaster or controller manager down) |
| `JOINT_NAN` | error | a joint position is NaN/inf (diverged controller) |
| `NEAR_LIMIT` / `LIMIT_POS` | warn / error | within 1% of a URDF joint limit, or outside it |
| `LIMIT_VEL` | warn | joint faster than its URDF velocity limit |
| `EFFORT_SAT` | warn | a joint pinned at its effort limit for 2 s (contact or payload) |
| `NO_CONTROLLER_MANAGER` | warn | `list_controllers` does not answer |
| `CTRL_INACTIVE` / `NO_ACTIVE_ARM_CTRL` | error | an arm controller is not active, or none is |
| `CTRL_CONFLICT` | warn | several arm controllers active at once |
| `NO_MOVE_GROUP` | error | `--moveit` and no `move_group` node |
| `TRACKING_ERROR` | warn | trajectory controller error above 0.05 rad for 3 s |
| `TRAJ_STALLED` | error | a `follow_joint_trajectory` goal is executing but the joints are not moving |
| `GOAL_ABORTS` | warn | 3 or more trajectory goals aborted or cancelled within 30 s |

Limits come from `/robot_description`. Missing-thing checks wait out a startup grace (`--grace`, 20 s).

## Try it without a robot

`test/fake_arm.py` publishes a fake ros2_control graph: healthy, or `--freeze`, `--inactive`, `--stall`.

```bash
python3 test/fake_arm.py --stall &
ros2 run armdoc armdoc --once 12 --stall-time 4      # TRAJ_STALLED, exit 1
python3 -m pytest test                               # the diagnosis rules, no ROS needed
```

## Limits

- Humble tested. Controller state topics assume `<controller>/state` (`JointTrajectoryControllerState`).
- One arm per run; use `--ns` for each.
- Text or `--json` output only, no `/diagnostics` yet.
