"""The C++ core (armdoc_cpp) must give the same findings as armdoc/core.py on the same joint_states stream."""
import math
import os
import random
import shutil
import subprocess

import pytest

from armdoc import core

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "armdoc_cpp")
NAMES = [f"j{i}" for i in range(6)]
LIMITS = {n: dict(lower=-3.0, upper=3.0, velocity=2.0, effort=50.0, continuous=False) for n in NAMES}
LIMITS["j5"] = dict(lower=None, upper=None, velocity=3.0, effort=None, continuous=True)

pytestmark = pytest.mark.skipif(shutil.which("g++") is None, reason="needs g++")


@pytest.fixture(scope="module")
def cli(tmp_path_factory):
    out = str(tmp_path_factory.mktemp("cpp") / "armdoc_core_cli")
    subprocess.run(["g++", "-std=c++17", "-O2", "-I", os.path.join(ROOT, "include"),
                    os.path.join(ROOT, "src", "core_cli.cpp"), "-o", out], check=True)
    return out


def _fmt(v):
    return "-" if v is None else repr(float(v))


def scenario(seed, kind):
    """Returns (scenario text for the C++ cli, list of python findings per evaluation)."""
    rnd = random.Random(seed)
    s = core.State(t0=0.0)
    s.limits = {n: dict(v) for n, v in LIMITS.items()}
    lines = [f"L {n} {_fmt(v['lower'])} {_fmt(v['upper'])} {_fmt(v['velocity'])} {_fmt(v['effort'])} {int(v['continuous'])}"
             for n, v in LIMITS.items()]
    if kind == "subset":
        s.arm_joints = {"j0", "j1", "j2"}
        lines += [f"A {n}" for n in sorted(s.arm_joints)]
    c = core.Config()
    py, t = [], 0.0
    dt = 0.01 if kind != "slow" else 0.2
    q = [rnd.uniform(-1, 1) for _ in NAMES]
    for step in range(260):
        t += dt
        if kind == "stale" and step == 130:
            for _ in range(2):             # the stream goes silent for 3 s; evaluate while it is down
                t += 1.5
                py.append(core.evaluate(s, t, c)); lines.append(f"E {t!r}")
        pos = [x + 0.3 * math.sin(t * (i + 1)) for i, x in enumerate(q)]
        vel = [0.3 * (i + 1) * math.cos(t * (i + 1)) for i in range(6)]
        eff = [rnd.uniform(-5, 5) for _ in NAMES]
        if kind == "noisy":
            pos = [x + rnd.gauss(0, 0.01) for x in pos]
        if kind == "limits":
            pos[0] = 3.2 if step > 100 else 2.99
            pos[1] = -2.99
            vel[2] = 3.0
        if kind == "nan" and step > 150:
            pos[3] = float("nan")
        if kind == "effort":
            eff[1] = 49.0
        lag = None
        if kind == "lag":
            lag = 0.8 + rnd.uniform(0, 0.1)
        elif kind in ("healthy", "noisy"):
            lag = rnd.uniform(0.0, 0.02)
        core.feed_joints(s, t, NAMES, pos, vel, eff, lag)
        lines.append(f"S {t!r} {'-' if lag is None else repr(lag)} 6 " +
                     " ".join(f"{n} {a!r} {b!r} {e!r}" for n, a, b, e in zip(NAMES, pos, vel, eff)))
        if step % 20 == 0 or step in (259,):
            py.append(core.evaluate(s, t, c)); lines.append(f"E {t!r}")
    return "\n".join(lines) + "\n", py


def run_cli(cli, text, tmp_path):
    p = tmp_path / "s.txt"
    p.write_text(text)
    out = subprocess.run([cli, str(p)], check=True, capture_output=True, text=True).stdout
    groups, cur = [], []
    for ln in out.splitlines():
        if ln == "--":
            groups.append(sorted(cur)); cur = []
        else:
            cur.append(ln)
    return groups


@pytest.mark.parametrize("kind", ["healthy", "noisy", "limits", "nan", "effort", "lag", "stale", "subset", "slow"])
@pytest.mark.parametrize("seed", [1, 2])
def test_same_findings_as_python(cli, tmp_path, kind, seed):
    text, py = scenario(seed, kind)
    cpp = run_cli(cli, text, tmp_path)
    assert len(cpp) == len(py)
    for i, (pf, cf) in enumerate(zip(py, cpp)):
        pyl = sorted(f"{x.level} {x.code}" for x in pf)
        assert pyl == cf, f"{kind} seed {seed} evaluation {i}: python {pyl} vs c++ {cf}"


def test_scenarios_actually_exercise_the_checks(cli, tmp_path):
    """Guard against a vacuous pass: every code the core covers shows up somewhere in the scenarios."""
    seen = set()
    for kind in ("noisy", "limits", "nan", "effort", "lag", "stale", "healthy"):
        _, py = scenario(1, kind)
        seen |= {x.code for fl in py for x in fl}
    assert {"NOISY_JOINTS", "LIMIT_POS", "NEAR_LIMIT", "LIMIT_VEL", "JOINT_NAN", "EFFORT_SAT", "STREAM_LAG",
            "JOINT_STATES_STALE"} <= seen


def test_is_arm_controller_matches_python(cli, tmp_path):
    cases = [("arm_controller", "joint_trajectory_controller/JointTrajectoryController"),
             ("joint_state_broadcaster", "joint_state_broadcaster/JointStateBroadcaster"),
             ("gripper_controller", "position_controllers/GripperActionController"),
             ("panda_hand_controller", "position_controllers/GripperActionController"),
             ("forward_command_controller_effort", "forward_command_controller/ForwardCommandController"),
             ("my_position", "position_controllers/JointGroupPositionController"),
             ("imp", "ur_controllers/CartesianImpedanceController"),
             ("Hand_GRIPPER", "velocity_controllers/JointGroupVelocityController"),
             ("diff_drive", "diff_drive_controller/DiffDriveController"),
             ("adm", "admittance_controller/AdmittanceController")]
    p = tmp_path / "k.txt"
    p.write_text("".join(f"K {n} {t}\n" for n, t in cases))
    out = subprocess.run([cli, str(p)], check=True, capture_output=True, text=True).stdout.split()
    assert [int(x) for x in out] == [int(core.is_arm_controller(n, t)) for n, t in cases]
    assert any(int(x) for x in out) and not all(int(x) for x in out)
