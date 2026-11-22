import math

from armdoc.core import (HINTS, Config, Finding, State, evaluate, feed_joints, format_findings, parse_limits, verdict,
                         verdict_line)

URDF = """<robot name="r"><link name="a"/><link name="b"/>
<joint name="j1" type="revolute"><parent link="a"/><child link="b"/><limit lower="-1" upper="1" velocity="2" effort="10"/></joint>
<joint name="j2" type="continuous"><parent link="a"/><child link="b"/><limit velocity="3" effort="5"/></joint>
<joint name="fx" type="fixed"><parent link="a"/><child link="b"/></joint></robot>"""


def codes(f):
    return {x.code for x in f}


def state():
    s = State(t0=0.0)
    s.limits = parse_limits(URDF)
    return s


def test_parse_limits_skips_fixed():
    assert set(parse_limits(URDF)) == {"j1", "j2"}


def test_no_joint_states_only_after_grace():
    s = state()
    assert evaluate(s, 5.0) == []
    assert "JOINT_STATES_NONE" in codes(evaluate(s, 30.0))


def test_stale_joint_states():
    s = state()
    feed_joints(s, 1.0, ["j1"], [0.0], [0.0], [0.0])
    assert "JOINT_STATES_STALE" in codes(evaluate(s, 5.0))


def test_limits_and_nan():
    s = state()
    feed_joints(s, 1.0, ["j1", "j2"], [0.995, 9.0], [0.0, 0.0], [0.0, 0.0])
    assert codes(evaluate(s, 1.0)) == {"NEAR_LIMIT"}      # continuous joint is never "out of range"
    feed_joints(s, 1.1, ["j1"], [1.2], [0.0], [0.0])
    assert "LIMIT_POS" in codes(evaluate(s, 1.1))
    feed_joints(s, 1.2, ["j1"], [math.nan], [0.0], [0.0])
    assert "JOINT_NAN" in codes(evaluate(s, 1.2))


def test_velocity_limit():
    s = state()
    feed_joints(s, 1.0, ["j1"], [0.0], [2.5], [0.0])
    assert "LIMIT_VEL" in codes(evaluate(s, 1.0))


def test_effort_saturation_needs_duration():
    s = state()
    for t in (1.0, 2.0, 3.5):
        feed_joints(s, t, ["j1"], [0.0], [0.0], [9.9])
        out = evaluate(s, t)
    assert "EFFORT_SAT" in codes(out)
    feed_joints(s, 4.0, ["j1"], [0.0], [0.0], [1.0])
    assert "EFFORT_SAT" not in codes(evaluate(s, 4.0))


def test_inactive_controller():
    s = state()
    feed_joints(s, 29.9, ["j1"], [0.0], [0.0], [0.0])
    s.controllers = [("arm_controller", "joint_trajectory_controller/JointTrajectoryController", "inactive"),
                     ("jsb", "joint_state_broadcaster/JointStateBroadcaster", "active")]
    out = codes(evaluate(s, 30.0))
    assert {"CTRL_INACTIVE", "NO_ACTIVE_ARM_CTRL"} <= out


def test_gripper_controller_is_not_an_arm_controller():
    s = state()
    feed_joints(s, 29.9, ["j1"], [0.0], [0.0], [0.0])
    s.controllers = [("arm_controller", "joint_trajectory_controller/JointTrajectoryController", "active"),
                     ("gripper_controller", "position_controllers/GripperActionController", "inactive")]
    assert codes(evaluate(s, 30.0)) == set()


def test_limit_checks_only_cover_arm_joints():
    s = state()
    s.arm_joints = {"j2"}
    feed_joints(s, 1.0, ["j1", "j2"], [3.0, 0.0], [0.0, 0.0], [0.0, 0.0])
    assert evaluate(s, 1.0) == []
    s.arm_joints = {"j1"}
    assert "LIMIT_POS" in codes(evaluate(s, 1.0))


def test_tracking_error_sustained():
    s = state()
    for t in (1.0, 2.0, 5.0):
        s.track["arm"] = (t, 0.2)
        out = evaluate(s, t)
    assert "TRACKING_ERROR" in codes(out)


def test_stalled_goal():
    s = state()
    s.goal_active_since = 0.0
    for i in range(0, 80):
        feed_joints(s, i * 0.1, ["j1"], [0.1], [0.0], [0.0])
    assert "TRAJ_STALLED" in codes(evaluate(s, 7.9, Config(grace=1000)))
    feed_joints(s, 8.0, ["j1"], [0.5], [0.0], [0.0])
    s.pos_hist.clear()
    for i in range(0, 80):
        feed_joints(s, 8.0 + i * 0.1, ["j1"], [0.1 + 0.01 * i], [0.0], [0.0])
    assert "TRAJ_STALLED" not in codes(evaluate(s, 15.9, Config(grace=1000)))


def test_goal_aborts_flapping_and_expiry():
    s = state()
    s.aborts.extend([1.0, 2.0, 3.0])
    assert "GOAL_ABORTS" in codes(evaluate(s, 4.0))
    assert "GOAL_ABORTS" not in codes(evaluate(s, 60.0))


def test_missing_move_group():
    s = state()
    s.moveit_nodes = False
    assert "NO_MOVE_GROUP" not in codes(evaluate(s, 5.0))
    assert "NO_MOVE_GROUP" in codes(evaluate(s, 30.0))


def test_verdict_ready_and_not_ready():
    assert verdict([]) == (True, [], [])
    assert verdict_line([]) == "READY"
    w = [Finding("WARN", "NEAR_LIMIT", "x")]
    assert verdict_line(w) == "READY - 1 warning(s): NEAR_LIMIT"
    e = w + [Finding("ERROR", "CTRL_INACTIVE", "y")]
    assert verdict(e)[0] is False
    assert verdict_line(e) == "NOT READY - 1 blocking: CTRL_INACTIVE"


def test_hints_cover_every_code_and_format_adds_them():
    import re
    import armdoc.core as core
    emitted = set(re.findall(r'Finding\((?:ERROR|WARN), "([A-Z_]+)"', open(core.__file__).read()))
    assert emitted <= set(HINTS), emitted - set(HINTS)
    out = format_findings([Finding("ERROR", "CTRL_INACTIVE", "y")], hints=True)
    assert "-> ros2 control set_controller_state" in out
    assert "->" not in format_findings([Finding("ERROR", "CTRL_INACTIVE", "y")])
    assert format_findings([]) == "OK    arm looks healthy"


def test_deactivating_the_arm_controller_does_not_widen_the_checks_to_gripper_fingers():
    """Found on a Franka Panda (ros2_control mock hardware): with the arm controller inactive it claims no interfaces,
    and armdoc then judged the gripper fingers, resting at their closed limit, as arm joints."""
    from armdoc.core import update_arm_joints
    arm = {"panda_joint1", "panda_joint2"}
    assert update_arm_joints(None, arm) == arm            # first poll, controller active
    assert update_arm_joints(arm, set()) == arm           # controller deactivated: keep the known arm joints
    assert update_arm_joints(None, set()) is None         # never saw an arm controller: check everything, as before
    assert update_arm_joints(arm, {"j7"}) == {"j7"}       # a changed claim replaces the set

    s = State(t0=0.0)
    s.limits = {"panda_finger_joint1": dict(lower=0.0, upper=0.04, velocity=0.2, effort=20.0, continuous=False)}
    s.arm_joints = update_arm_joints(arm, set())
    feed_joints(s, 100.0, ["panda_finger_joint1"], [0.0], [0.0], [0.0])
    assert "NEAR_LIMIT" not in [f.code for f in evaluate(s, 100.5)]
    s.arm_joints = None                                    # arm joints never learned: fingers are still skipped by name
    assert "NEAR_LIMIT" not in [f.code for f in evaluate(s, 100.5)]
    s.limits = {"joint_a": dict(lower=0.0, upper=0.04, velocity=0.2, effort=20.0, continuous=False)}
    s.joints.clear()
    feed_joints(s, 100.0, ["joint_a"], [0.0], [0.0], [0.0])
    assert "NEAR_LIMIT" in [f.code for f in evaluate(s, 100.5)]   # a non-gripper joint is still judged


def test_baseline_names_a_slowed_joint_states_and_stays_quiet_otherwise():
    from armdoc import baseline as bl
    s = state()
    for i in range(600):                                              # 20 s at 30 Hz
        feed_joints(s, i / 30, ["j1"], [0.0], [0.0], [0.0])
    s.baseline = bl.learn({"joint_states": s.js_times})
    assert abs(s.baseline["joint_states"]["hz"] - 30) < 1.5
    assert not codes(evaluate(s, 20.0)) & {"RATE_DROP", "RATE_SPIKE"}
    for i in range(80):                                               # then 8 Hz for 10 s
        feed_joints(s, 20 + i / 8, ["j1"], [0.0], [0.0], [0.0])
    found = evaluate(s, 29.9)
    assert "RATE_DROP" in codes(found) and "JOINT_STATES_STALE" not in codes(found)
    assert HINTS["RATE_DROP"] and HINTS["RATE_SPIKE"]


def test_baseline_leaves_a_dead_stream_to_the_stale_check():
    from armdoc import baseline as bl
    s = state()
    for i in range(600):
        feed_joints(s, i / 30, ["j1"], [0.0], [0.0], [0.0])
    s.baseline = bl.learn({"joint_states": s.js_times})
    found = codes(evaluate(s, 60.0))
    assert "JOINT_STATES_STALE" in found and "RATE_DROP" not in found


# ---- /rosout log checks ---------------------------------------------------------------------------

LOG_CASES = {
    "Unable to sample any valid states for goal tree": "LOG_PLAN_GOAL_INVALID",
    "Goal state is in collision": "LOG_PLAN_GOAL_INVALID",
    "Start state appears to be in collision with respect to group arm": "LOG_START_STATE_BAD",
    "Invalid Trajectory: start point deviates from current robot state more than 0.01": "LOG_START_DEVIATES",
    "Unable to identify any set of controllers that can actuate the specified joints: [ j1 j2 ]": "LOG_NO_CONTROLLER",
    "Action client not connected to action server: arm_controller/follow_joint_trajectory": "LOG_ACTION_NOT_CONNECTED",
    "Didn't receive robot state (joint angles) with recent timestamp within 1.0 seconds.": "LOG_ROBOT_STATE_STALE",
    "Failed to fetch current robot state.": "LOG_ROBOT_STATE_STALE",
    "Aborted due to path tolerance violation on joint j3": "LOG_TOLERANCE_VIOLATED",
    "Controller handle arm_controller reports status ABORTED": "LOG_EXEC_ABORTED",
}


def test_log_lines_are_classified():
    from armdoc.core import classify_log
    for msg, code in LOG_CASES.items():
        assert classify_log(msg) == code, msg
    for noise in ("Loading robot model 'ur3'", "Planning request received", "Execution completed: SUCCEEDED", ""):
        assert classify_log(noise) is None, noise


def test_only_arm_stack_loggers_count():
    from armdoc.core import log_from
    assert all(log_from(n) for n in ("move_group", "moveit.ros.trajectory_execution_manager", "controller_manager",
                                     "arm_controller", "ros2_control_node"))
    assert not any(log_from(n) for n in ("rviz2", "robot_state_publisher", "gazebo", ""))


def test_log_finding_is_reported_counted_and_ages_out():
    from armdoc.core import feed_log
    s = state()
    for i in range(3):
        feed_log(s, "Action client not connected to action server: arm_controller/follow_joint_trajectory", 10.0 + i)
    feed_log(s, "Loading robot model", 11.0)                        # noise is dropped
    f = [x for x in evaluate(s, 13.0) if x.code == "LOG_ACTION_NOT_CONNECTED"]
    assert len(f) == 1 and f[0].level == "WARN" and f[0].msg.startswith("3x in the last 30s")
    assert "LOG_ACTION_NOT_CONNECTED" not in codes(evaluate(s, 60.0))


def test_every_log_code_has_a_hint():
    from armdoc.core import LOG_PATTERNS
    for _, code in LOG_PATTERNS:
        assert HINTS.get(code), code
