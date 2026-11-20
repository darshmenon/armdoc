"""armdoc_cpp learns the arm's joints from the controller manager, so a gripper finger resting at its closed limit is
never judged as an arm joint, even after the arm controller is deactivated. Needs ROS 2, the armdoc_cpp package built
and sourced, and the control msgs; skipped otherwise. Uses its own ROS domain."""
import os
import shutil
import subprocess
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))


def _have_cpp():
    if shutil.which("ros2") is None:
        return False
    r = subprocess.run(["ros2", "pkg", "prefix", "armdoc_cpp"], capture_output=True, text=True)
    return r.returncode == 0


pytestmark = pytest.mark.skipif(not _have_cpp(), reason="needs ROS 2 with armdoc_cpp built and sourced")
ENV = dict(os.environ, ROS_DOMAIN_ID="86", ROS_LOCALHOST_ONLY="1")


def run_with_fake(extra, secs, cpp=True):
    fake = subprocess.Popen([sys.executable, os.path.join(HERE, "fake_gripper_arm.py")] + extra, env=ENV,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(2)
        cmd = (["ros2", "run", "armdoc_cpp", "armdoc_fast", "--ros-args", "-p", f"once:={float(secs)}", "-p", "grace:=0.0"]
               if cpp else [sys.executable, "-m", "armdoc.doctor", "--once", str(secs), "--grace", "0"])
        env = dict(ENV, PYTHONPATH=os.pathsep.join([os.path.join(HERE, ".."), ENV.get("PYTHONPATH", "")]))
        p = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=secs + 30)
        return p.returncode, p.stdout
    finally:
        fake.terminate()
        fake.wait(timeout=10)


def test_gripper_finger_at_its_limit_is_not_an_arm_joint():
    rc, out = run_with_fake([], 8)
    assert rc == 0 and "arm looks healthy" in out, out
    assert "NEAR_LIMIT" not in out and "finger" not in out


def test_deactivating_the_arm_controller_keeps_the_arm_joints():
    # the arm controller is seen active first (polls every 2 s), then goes inactive and claims nothing
    rc, out = run_with_fake(["--deactivate-after", "8"], 14)
    assert "NEAR_LIMIT" not in out and "finger" not in out, out


def test_python_tool_agrees_when_the_arm_controller_is_deactivated():
    pytest.importorskip("rclpy")
    rc, out = run_with_fake(["--deactivate-after", "8"], 14, cpp=False)
    assert "CTRL_INACTIVE" in out and "NO_ACTIVE_ARM_CTRL" in out, out
    assert "NEAR_LIMIT" not in out and "finger" not in out, out


def test_json_output_matches_the_python_shape():
    import json
    sec = 14
    cmd = ["ros2", "run", "armdoc_cpp", "armdoc_fast", "--ros-args", "-p", f"once:={float(sec)}", "-p", "grace:=0.0",
           "-p", "json:=true"]
    fake = subprocess.Popen([sys.executable, os.path.join(HERE, "fake_gripper_arm.py"), "--deactivate-after", "8"],
                            env=ENV, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(2)
        p = subprocess.run(cmd, env=ENV, capture_output=True, text=True, timeout=sec + 30)
    finally:
        fake.terminate()
        fake.wait(timeout=10)
    found = json.loads(p.stdout.strip().splitlines()[-1])
    assert isinstance(found, list)
    assert all(set(x) == {"level", "code", "msg"} for x in found), found
    assert not any(x["code"] == "NEAR_LIMIT" for x in found), found
