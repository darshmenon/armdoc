import math

from armdoc.core import Config, State, evaluate, feed_joints, parse_limits

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


def test_inactive_controller_and_conflict():
    s = state()
    feed_joints(s, 29.9, ["j1"], [0.0], [0.0], [0.0])
    s.controllers = [("arm_controller", "joint_trajectory_controller/JointTrajectoryController", "inactive"),
                     ("jsb", "joint_state_broadcaster/JointStateBroadcaster", "active")]
    out = codes(evaluate(s, 30.0))
    assert {"CTRL_INACTIVE", "NO_ACTIVE_ARM_CTRL"} <= out
    s.controllers = [("a", "joint_trajectory_controller/JointTrajectoryController", "active"),
                     ("b", "effort_controllers/JointGroupEffortController", "active")]
    assert "CTRL_CONFLICT" in codes(evaluate(s, 30.0))


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
