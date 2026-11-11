# armdoc

**Why isn't my arm moving?** Attach to a running ros2_control / MoveIt 2 arm and get a one-line cause. Read-only: it subscribes to topics and lists controllers, and never publishes or changes anything.

```bash
ros2 run armdoc armdoc                     # live, a report every 2 s
ros2 run armdoc armdoc --ns /ur3 --moveit  # namespaced arm, also require move_group
ros2 run armdoc armdoc --once 10           # watch 10 s, print one report, exit 1 on any ERROR (for CI)
ros2 run armdoc armdoc --ready             # pre-flight: hints + READY / NOT READY verdict, exit 1 if not ready
ros2 run armdoc armdoc --diagnostics       # also publish findings to /diagnostics (opt-in, see below)
```

```
ERROR [CTRL_INACTIVE] controller arm_controller (joint_trajectory_controller/JointTrajectoryController) is inactive, not active
ERROR [TRAJ_STALLED] a trajectory goal has been active for 10s but the joints moved 0.0000 rad
```

![armdoc on a UR3 in Gazebo](docs/ur3_gazebo.png)

*Real output from a UR3 in Gazebo: healthy, then the arm controller deactivated, then the sim paused while armdoc was attached. Source: `docs/ur3_gazebo_demo.txt`, `docs/make_screenshot.py`.*

## What it checks

| Code | Level | Means |
|---|---|---|
| `JOINT_STATES_NONE` / `JOINT_STATES_STALE` | error | no `/joint_states`, or it went silent (broadcaster or controller manager down) |
| `JOINT_NAN` | error | a joint position is NaN/inf (diverged controller) |
| `NEAR_LIMIT` / `LIMIT_POS` | warn / error | within 1% of a URDF joint limit, or outside it |
| `LIMIT_VEL` | warn | joint faster than its URDF velocity limit |
| `EFFORT_SAT` | warn | a joint pinned at its effort limit for 2 s (contact or payload) |
| `NO_CONTROLLER_MANAGER` | warn | `list_controllers` does not answer |
| `CTRL_INACTIVE` / `NO_ACTIVE_ARM_CTRL` | error | an arm controller is not active, or none is |
| `NO_MOVE_GROUP` | error | `--moveit` and no `move_group` node |
| `TRACKING_ERROR` | warn | trajectory controller error above 0.05 rad for 3 s |
| `TRAJ_STALLED` | error | a `follow_joint_trajectory` goal is executing but the joints are not moving |
| `GOAL_ABORTS` | warn | 3 or more trajectory goals aborted or cancelled within 30 s |

Limits come from `/robot_description`, and are checked only for joints that arm controllers claim (grippers and broadcasters are skipped). Missing-thing checks wait out a startup grace (`--grace`, 20 s).

## `/diagnostics` (opt-in)

armdoc never publishes anything unless you pass `--diagnostics`. Then each report goes to `/diagnostics` as a `diagnostic_msgs/DiagnosticArray`: one status per finding (`armdoc: <CODE>`, level WARN or ERROR, the message), or a single OK status when the arm looks healthy. It publishes nothing else, and never commands or reconfigures the arm.

## Timeline report (visualization)

Record a run, then turn it into a timeline: joint positions with shaded bands where findings were active, a controller lane, and a findings table.

```bash
ros2 run armdoc armdoc --record run.jsonl                       # Ctrl-C to stop; logs joints (10 Hz) + findings (1 Hz)
armdoc report run.jsonl --html report.html --png report.png     # no ROS needed for this step
```

![armdoc timeline of a UR3 in Gazebo](docs/ur3_report.png)

*Real recording of a UR3 in Gazebo (80 s): two moves, then `arm_controller` deactivated (the move commanded meanwhile is ignored, and the arm only follows after reactivation), then the sim paused until `/joint_states` went silent. Source: `docs/ur3_run.jsonl`; open `docs/ur3_report.html` for the interactive version.*

- **HTML:** one self-contained file (inline SVG + a little JS, no network). Hover for a crosshair with every joint value and the findings active at that moment, click a legend entry to hide a joint, light/dark follows the OS. A 1 Hz table of the joint positions is folded under the findings table.
- **PNG:** the same timeline via matplotlib.
- Lines break where `/joint_states` was silent, so a gap is shown as a gap. The controller lane shows only controllers that had been active and then dropped out.

## Try it without a robot

`test/fake_arm.py` publishes a fake ros2_control graph: healthy, or `--freeze`, `--inactive`, `--stall`.

```bash
python3 test/fake_arm.py --stall &
ros2 run armdoc armdoc --once 12 --stall-time 4      # TRAJ_STALLED, exit 1
python3 -m pytest test                               # the diagnosis rules, no ROS needed
```

## Limits

- Verified on Humble against a live UR3 Gazebo sim (ros2_control, `arm_controller` + `gripper_controller`). Controller state topics assume `<controller>/state` (`JointTrajectoryControllerState`).
- One arm per run; use `--ns` for each.
- Text or `--json` output; `/diagnostics` only with `--diagnostics`.
