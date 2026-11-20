import math
import random

from armdoc.core import (HINTS, Config, State, estimate_noise, evaluate, feed_joints, format_findings)


def codes(f):
    return {x.code for x in f}


def stream(s, seconds, hz=50, sigma=0.0, joint_hz=0.5, lag=None, seed=1, t_start=0.0):
    """Feed a smooth sinusoid (plus gaussian noise) as joint_states and return the last time fed."""
    rng = random.Random(seed)
    t = t_start
    for _ in range(int(seconds * hz)):
        pos = [math.sin(2 * math.pi * joint_hz * t + i) + rng.gauss(0.0, sigma) for i in range(3)]
        feed_joints(s, t, ["j0", "j1", "j2"], pos, [0.0] * 3, [0.0] * 3, lag=lag)
        t += 1.0 / hz
    return t - 1.0 / hz


def test_noise_estimate_recovers_sigma_and_ignores_motion():
    s = State(t0=0.0)
    now = stream(s, 3.0, sigma=0.02)
    est = estimate_noise(s, now, Config())
    assert all(0.012 < v < 0.03 for v in est.values()), est     # true sigma is 0.02
    s = State(t0=0.0)
    now = stream(s, 3.0, sigma=0.0, joint_hz=2.0)               # fast smooth motion, no noise
    assert max(estimate_noise(s, now, Config()).values()) < 1e-3


def test_noisy_joints_fires_only_when_noisy():
    s = State(t0=0.0)
    now = stream(s, 3.0, sigma=0.0)
    assert "NOISY_JOINTS" not in codes(evaluate(s, now, Config(grace=1000)))
    s = State(t0=0.0)
    now = stream(s, 3.0, sigma=0.02)
    out = evaluate(s, now, Config(grace=1000))
    assert "NOISY_JOINTS" in codes(out)
    assert "NOISY_JOINTS" not in codes(evaluate(s, now, Config(grace=1000, noise_rad=0.1)))    # configurable


def test_noise_check_skips_slow_streams_and_short_history():
    s = State(t0=0.0)
    now = stream(s, 30.0, hz=5, sigma=0.02)                     # 5 Hz: too coarse to separate noise from motion
    assert estimate_noise(s, now, Config()) == {}
    s = State(t0=0.0)
    now = stream(s, 0.1, sigma=0.05)
    assert estimate_noise(s, now, Config()) == {}


def test_noise_check_only_looks_at_arm_joints():
    s = State(t0=0.0)
    s.arm_joints = {"j1"}
    rng = random.Random(3)
    t = 0.0
    for _ in range(150):
        feed_joints(s, t, ["j0", "j1"], [rng.gauss(0, 0.05), 0.0], [0.0, 0.0], [0.0, 0.0])
        t += 0.02
    assert "NOISY_JOINTS" not in codes(evaluate(s, t, Config(grace=1000)))


def test_stream_lag():
    s = State(t0=0.0)
    now = stream(s, 3.0, lag=0.05)
    assert "STREAM_LAG" not in codes(evaluate(s, now, Config(grace=1000)))
    s = State(t0=0.0)
    now = stream(s, 3.0, lag=2.5)
    out = evaluate(s, now, Config(grace=1000))
    assert "STREAM_LAG" in codes(out)
    assert "2.50s" in next(x.msg for x in out if x.code == "STREAM_LAG")
    s = State(t0=0.0)
    now = stream(s, 3.0, lag=None)                              # no stamp information: no verdict
    assert "STREAM_LAG" not in codes(evaluate(s, now, Config(grace=1000)))


def test_stream_lag_uses_median_so_one_late_sample_does_not_fire():
    s = State(t0=0.0)
    now = stream(s, 3.0, lag=0.02)
    feed_joints(s, now + 0.02, ["j0"], [0.0], [0.0], [0.0], lag=5.0)
    assert "STREAM_LAG" not in codes(evaluate(s, now + 0.02, Config(grace=1000)))


def test_moveit_plan_failed_only_when_moveit_requested_and_recent():
    s = State(t0=0.0)
    feed_joints(s, 9.0, ["j0"], [0.0], [0.0], [0.0])
    s.moveit_aborts.append(8.0)
    assert "MOVEIT_PLAN_FAILED" not in codes(evaluate(s, 9.5, Config(grace=1000)))      # moveit_nodes is None: not asked
    s.moveit_nodes = True
    assert "MOVEIT_PLAN_FAILED" in codes(evaluate(s, 9.5, Config(grace=1000)))
    assert "MOVEIT_PLAN_FAILED" not in codes(evaluate(s, 80.0, Config(grace=1000)))     # outside the 30 s window
    s.moveit_aborts.extend([81.0, 82.0])
    assert "MOVEIT_PLAN_FAILED" not in codes(evaluate(s, 83.0, Config(grace=1000, moveit_abort_count=3)))
    assert "MOVEIT_PLAN_FAILED" in codes(evaluate(s, 83.0, Config(grace=1000, moveit_abort_count=2)))


def test_no_planning_scene():
    s = State(t0=0.0)
    s.moveit_nodes = True
    s.planning_scene = False
    assert "NO_PLANNING_SCENE" not in codes(evaluate(s, 5.0))        # still inside the startup grace
    assert "NO_PLANNING_SCENE" in codes(evaluate(s, 30.0))
    s.planning_scene = True
    assert "NO_PLANNING_SCENE" not in codes(evaluate(s, 30.0))
    s.planning_scene, s.moveit_nodes = False, False                 # no move_group: NO_MOVE_GROUP says it, not this
    assert "NO_PLANNING_SCENE" not in codes(evaluate(s, 30.0))
    s.planning_scene, s.moveit_nodes = False, None                  # --moveit not given
    assert "NO_PLANNING_SCENE" not in codes(evaluate(s, 30.0))


def test_new_codes_have_hints():
    for code in ("NOISY_JOINTS", "STREAM_LAG", "MOVEIT_PLAN_FAILED", "NO_PLANNING_SCENE"):
        assert code in HINTS


JTC = "joint_trajectory_controller/JointTrajectoryController"


def ctrl_state(controllers, **kw):
    s = State(t0=0.0)
    s.controllers = controllers
    for k, v in kw.items():
        setattr(s, k, v)
    return s


def test_active_trajectory_controller_without_action_server_is_an_error():
    s = ctrl_state([("arm_controller", JTC, "active")], traj_action_pubs={"arm_controller": 0})
    out = evaluate(s, 100.0, Config(grace=1))
    assert "NO_TRAJ_ACTION" in codes(out)
    assert "NO_TRAJ_ACTION" in HINTS
    ok = ctrl_state([("arm_controller", JTC, "active")], traj_action_pubs={"arm_controller": 1})
    assert "NO_TRAJ_ACTION" not in codes(evaluate(ok, 100.0, Config(grace=1)))
    unknown = ctrl_state([("arm_controller", JTC, "active")])      # count not read yet: stay quiet
    assert "NO_TRAJ_ACTION" not in codes(evaluate(unknown, 100.0, Config(grace=1)))
    inactive = ctrl_state([("arm_controller", JTC, "inactive")], traj_action_pubs={"arm_controller": 0})
    assert "NO_TRAJ_ACTION" not in codes(evaluate(inactive, 100.0, Config(grace=1)))   # CTRL_INACTIVE says it


def test_moveit_controller_names_must_exist_in_ros2_control():
    ctrls = [("arm_controller", JTC, "active"), ("joint_state_broadcaster", "joint_state_broadcaster/JointStateBroadcaster", "active")]
    bad = ctrl_state(ctrls, moveit_ctrl_names=["scaled_joint_trajectory_controller"], traj_action_pubs={"arm_controller": 1})
    out = [x for x in evaluate(bad, 100.0, Config(grace=1)) if x.code == "MOVEIT_CTRL_MISMATCH"]
    assert out and "scaled_joint_trajectory_controller" in out[0].msg
    good = ctrl_state(ctrls, moveit_ctrl_names=["arm_controller"], traj_action_pubs={"arm_controller": 1})
    assert "MOVEIT_CTRL_MISMATCH" not in codes(evaluate(good, 100.0, Config(grace=1)))
    assert "MOVEIT_CTRL_MISMATCH" in HINTS


def test_gripper_at_its_closed_limit_is_not_flagged_before_arm_joints_are_known():
    s = State(t0=0.0)
    s.limits = {"gripper_joint": dict(lower=0.0, upper=1.3), "shoulder_pan_joint": dict(lower=-3.0, upper=3.0)}
    feed_joints(s, 9.0, ["gripper_joint", "shoulder_pan_joint"], [0.0, 2.999], [0.0, 0.0], [0.0, 0.0])
    out = evaluate(s, 9.0, Config(grace=1))
    msgs = [x.msg for x in out if x.code == "NEAR_LIMIT"]
    assert any("shoulder_pan_joint" in m for m in msgs) and not any("gripper_joint" in m for m in msgs)
