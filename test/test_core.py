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
    s.arm_joints = None                                    # without the fix the fingers are judged: the old behaviour
    assert "NEAR_LIMIT" in [f.code for f in evaluate(s, 100.5)]
