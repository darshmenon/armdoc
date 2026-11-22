"""armdoc --learn / --baseline against the fake arm graph. Needs a sourced ROS 2 env."""
import json
import os
import subprocess
import sys
import time

import pytest

pytest.importorskip("rclpy")
ROOT = os.path.join(os.path.dirname(__file__), "..")
ARM = os.path.join(os.path.dirname(__file__), "fake_arm.py")
_domain = [170]


def run(hz, args, secs):
    _domain[0] += 1
    env = dict(os.environ, ROS_DOMAIN_ID=str(_domain[0]), ROS_LOCALHOST_ONLY="1",
               PYTHONPATH=ROOT + os.pathsep + os.environ.get("PYTHONPATH", ""))
    arm = subprocess.Popen([sys.executable, ARM, "--hz", str(hz)], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(1.5)
        p = subprocess.run([sys.executable, "-m", "armdoc.doctor", *args], env=env, cwd=ROOT, capture_output=True, text=True, timeout=secs + 40)
    finally:
        arm.terminate()
        arm.wait(timeout=10)
    return p.stdout + p.stderr


def test_learn_then_baseline_flags_a_slowed_arm(tmp_path):
    f = str(tmp_path / "b.json")
    out = run(50, ["--learn", "9", "--baseline-file", f], 9)
    learned = json.load(open(f))["streams"]
    assert "learned" in out and 40 < learned["joint_states"]["hz"] < 60, (learned, out)
    out = run(50, ["--once", "8", "--baseline", "--baseline-file", f], 8)
    assert "RATE_DROP" not in out and "RATE_SPIKE" not in out, out
    out = run(10, ["--once", "8", "--baseline", "--baseline-file", f], 8)
    assert "RATE_DROP" in out and "JOINT_STATES_STALE" not in out, out


def test_rosout_move_group_line_is_named_live():
    _domain[0] += 1
    env = dict(os.environ, ROS_DOMAIN_ID=str(_domain[0]), ROS_LOCALHOST_ONLY="1",
               PYTHONPATH=ROOT + os.pathsep + os.environ.get("PYTHONPATH", ""))
    arm = subprocess.Popen([sys.executable, ARM, "--log", "Action client not connected to action server: arm_controller/follow_joint_trajectory"],
                           env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(1.5)
        p = subprocess.run([sys.executable, "-m", "armdoc.doctor", "--once", "6"], env=env, cwd=ROOT, capture_output=True, text=True, timeout=40)
    finally:
        arm.terminate()
        arm.wait(timeout=10)
    assert "LOG_ACTION_NOT_CONNECTED" in p.stdout, p.stdout + p.stderr
