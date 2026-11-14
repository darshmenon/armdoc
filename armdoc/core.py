"""ROS-free diagnosis logic for armdoc: turns a snapshot of an arm's state into findings."""
import math
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass, field

ERROR, WARN = "ERROR", "WARN"


@dataclass
class Finding:
    level: str
    code: str
    msg: str

    def __str__(self):
        return f"{self.level:5} [{self.code}] {self.msg}"


@dataclass
class Config:
    stale: float = 2.0            # s without joint_states / state topic before it counts as dead
    grace: float = 20.0           # s after start before missing things are errors
    near_limit: float = 0.01      # fraction of the joint range that counts as "at the limit"
    vel_margin: float = 1.05
    effort_frac: float = 0.95
    effort_time: float = 2.0
    track_err: float = 0.05       # rad of trajectory tracking error
    track_time: float = 3.0
    stall_time: float = 5.0       # goal active this long with no joint motion
    stall_dist: float = 0.005     # rad
    abort_count: int = 3
    abort_window: float = 30.0
    noise_rad: float = 0.002      # estimated per-sample position noise (rad) that counts as noisy
    noise_window: float = 1.0     # s of joint_states used for the noise estimate
    noise_min_samples: int = 10
    noise_max_dt: float = 0.05    # skip the noise check below 20 Hz: coarse sampling aliases real motion
    lag_max: float = 0.5          # s between a joint_states header stamp and its arrival
    lag_window: float = 2.0
    moveit_abort_count: int = 1   # aborted /move_action goals inside moveit_window that raise MOVEIT_PLAN_FAILED
    moveit_window: float = 30.0


@dataclass
class State:
    t0: float = 0.0
    joints: dict = field(default_factory=dict)       # name -> (pos, vel, effort)
    joint_t: float = None
    limits: dict = field(default_factory=dict)       # name -> dict(lower, upper, velocity, effort)
    controllers: list = None                          # [(name, type, state)] or None if never answered
    arm_joints: set = None                            # joints claimed by arm controllers; None = check all
    cm_missing: bool = False
    track: dict = field(default_factory=dict)         # controller -> (t, max abs error)
    track_bad_since: dict = field(default_factory=dict)
    moveit_nodes: bool = None                         # None = not asked to check
    goal_active_since: float = None
    pos_hist: deque = field(default_factory=lambda: deque(maxlen=4000))
    aborts: deque = field(default_factory=deque)
    lag_hist: deque = field(default_factory=lambda: deque(maxlen=2000))   # (receive time, header-stamp lag in s)
    moveit_aborts: deque = field(default_factory=deque)                    # receive times of aborted /move_action goals
    planning_scene: bool = None                       # None = unknown; False = move_group is up but no scene is served
    eff_sat_since: dict = field(default_factory=dict)


def parse_limits(urdf_xml):
    """Joint limits from a URDF string: {joint: {lower, upper, velocity, effort}} for non-fixed joints."""
    out = {}
    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError:
        return out
    for j in root.iter("joint"):
        lim = j.find("limit")
        if j.get("type") in ("fixed", "floating", "planar") or lim is None:
            continue
        g = lambda k: float(lim.get(k)) if lim.get(k) not in (None, "") else None
        out[j.get("name")] = dict(lower=g("lower"), upper=g("upper"), velocity=g("velocity"), effort=g("effort"),
                                  continuous=j.get("type") == "continuous")
    return out


def feed_joints(s, now, names, pos, vel, eff, lag=None):
    """Record one joint_states sample. `lag` is receive time minus header stamp in seconds, when known."""
    s.joint_t = now
    if lag is not None and math.isfinite(lag):
        s.lag_hist.append((now, lag))
    for i, n in enumerate(names):
        g = lambda a: a[i] if i < len(a) else float("nan")
        s.joints[n] = (g(pos), g(vel), g(eff))
    s.pos_hist.append((now, {n: s.joints[n][0] for n in names}))


def _moved(s, now, window):
    """Largest joint displacement over the last `window` seconds."""
    old = None
    for t, p in s.pos_hist:
        if t >= now - window:
            old = p
            break
    if old is None or not s.pos_hist:
        return None
    cur = s.pos_hist[-1][1]
    return max((abs(cur[n] - old[n]) for n in cur if n in old), default=0.0)


def estimate_noise(s, now, c, joints=None):
    """Per-joint position noise (rad, one sigma) from the fourth difference over the last `noise_window` seconds.

    For white noise of sigma e the fourth difference (1, -4, 6, -4, 1) has variance 70 e^2, while smooth motion (even a
    fast acceleration) cancels out of it, so this separates jitter from movement. Samples are treated as evenly spaced.
    Returns {} when there are too few samples or the stream is too slow (`noise_max_dt`).
    """
    win = [(t, p) for t, p in s.pos_hist if t >= now - c.noise_window]
    if len(win) < max(c.noise_min_samples, 6):
        return {}
    dts = sorted(win[i + 1][0] - win[i][0] for i in range(len(win) - 1))
    if not dts or dts[len(dts) // 2] > c.noise_max_dt:
        return {}
    out = {}
    for n in win[-1][1]:
        if joints is not None and n not in joints:
            continue
        xs = [p.get(n, math.nan) for _, p in win]
        acc, k = 0.0, 0
        for i in range(len(xs) - 4):
            a, b, d, e, g = xs[i:i + 5]
            if not all(math.isfinite(v) for v in (a, b, d, e, g)):
                continue
            h = a - 4 * b + 6 * d - 4 * e + g
            acc += h * h
            k += 1
        if k >= 5:
            out[n] = math.sqrt(acc / k / 70.0)
    return out


def is_arm_controller(name, typ):
    """Controllers that drive arm joints (not broadcasters, not grippers)."""
    if "broadcaster" in typ or "gripper" in name.lower() or "gripper" in typ.lower():
        return False
    return any(k in typ for k in ("trajectory", "effort", "position", "velocity", "impedance", "admittance"))


def update_arm_joints(previous, claimed):
    """Joints to check after a controller poll. An inactive controller claims no interfaces, so an empty `claimed`
    keeps the last known set instead of widening the checks to every joint (which would judge gripper fingers resting
    at their closed limit as arm joints). None until an arm controller has been seen active."""
    return set(claimed) if claimed else previous


def evaluate(s, now, c=Config()):
    f = []
    up = now - s.t0
    late = up > c.grace

    if s.joint_t is None:
        if late:
            f.append(Finding(ERROR, "JOINT_STATES_NONE", "no /joint_states received (joint_state_broadcaster not running, or the sim is paused)"))
    elif now - s.joint_t > c.stale:
        f.append(Finding(ERROR, "JOINT_STATES_STALE", f"/joint_states silent for {now - s.joint_t:.1f}s "
                         "(controller manager or joint_state_broadcaster is down)"))

    live = s.joint_t is not None and now - s.joint_t <= c.stale
    if live:
        for n, (p, v, e) in s.joints.items():
            if s.arm_joints is not None and n not in s.arm_joints:
                continue
            if not math.isfinite(p):
                f.append(Finding(ERROR, "JOINT_NAN", f"{n} position is NaN/inf (diverged controller or bad state)"))
                continue
            lim = s.limits.get(n)
            if not lim:
                continue
            lo, hi = lim.get("lower"), lim.get("upper")
            if lo is not None and hi is not None and not lim.get("continuous") and hi > lo:
                m = c.near_limit * (hi - lo)
                if p < lo or p > hi:
                    f.append(Finding(ERROR, "LIMIT_POS", f"{n} at {p:.3f} is outside its limits [{lo:.3f}, {hi:.3f}]"))
                elif p < lo + m or p > hi - m:
                    f.append(Finding(WARN, "NEAR_LIMIT", f"{n} at {p:.3f} is within {c.near_limit:.0%} of its "
                                     f"limit [{lo:.3f}, {hi:.3f}]; planners often fail here"))
            if lim.get("velocity") and math.isfinite(v) and abs(v) > lim["velocity"] * c.vel_margin:
                f.append(Finding(WARN, "LIMIT_VEL", f"{n} moving at {v:.2f} rad/s, limit {lim['velocity']:.2f}"))
            em = lim.get("effort")
            if em and math.isfinite(e) and abs(e) >= c.effort_frac * em:
                t0 = s.eff_sat_since.setdefault(n, now)
                if now - t0 >= c.effort_time:
                    f.append(Finding(WARN, "EFFORT_SAT", f"{n} pinned at {abs(e):.1f} of {em:.1f} N*m for "
                                     f"{now - t0:.0f}s (blocked by contact, or payload too heavy)"))
            else:
                s.eff_sat_since.pop(n, None)

    if live:
        noisy = {n: v for n, v in estimate_noise(s, now, c, s.arm_joints).items() if v > c.noise_rad}
        if noisy:
            n, v = max(noisy.items(), key=lambda kv: kv[1])
            more = f" (and {len(noisy) - 1} more)" if len(noisy) > 1 else ""
            f.append(Finding(WARN, "NOISY_JOINTS", f"{n} position jitter is about {v:.4f} rad{more}, above "
                             f"{c.noise_rad:g} rad (encoder or driver noise, or a loose connection)"))
        recent = sorted(l for t, l in s.lag_hist if t >= now - c.lag_window)
        if len(recent) >= 3 and recent[len(recent) // 2] > c.lag_max:
            f.append(Finding(WARN, "STREAM_LAG", f"/joint_states arrive {recent[len(recent) // 2]:.2f}s after their "
                             f"header stamp (limit {c.lag_max:g}s): controllers and planners see old data"))

    if s.cm_missing:
        if late:
            f.append(Finding(WARN, "NO_CONTROLLER_MANAGER", "/controller_manager/list_controllers is not answering"))
    elif s.controllers is not None:
        arm = [x for x in s.controllers if is_arm_controller(x[0], x[1])]
        for name, typ, st in s.controllers:
            if st != "active" and name in [a[0] for a in arm] and late:
                f.append(Finding(ERROR, "CTRL_INACTIVE", f"controller {name} ({typ}) is {st}, not active"))
        if arm and not any(x[2] == "active" for x in arm) and late:
            f.append(Finding(ERROR, "NO_ACTIVE_ARM_CTRL", "no arm controller is active, so nothing can move the arm"))

    if s.moveit_nodes is False and late:
        f.append(Finding(ERROR, "NO_MOVE_GROUP", "no move_group node on the graph; MoveIt requests will time out"))

    if s.moveit_nodes is not None:
        while s.moveit_aborts and now - s.moveit_aborts[0] > c.moveit_window:
            s.moveit_aborts.popleft()
        if len(s.moveit_aborts) >= c.moveit_abort_count:
            f.append(Finding(WARN, "MOVEIT_PLAN_FAILED", f"{len(s.moveit_aborts)} MoveIt request(s) aborted in the "
                             f"last {c.moveit_window:.0f}s (planning or execution failed)"))
    if s.moveit_nodes and s.planning_scene is False and late:
        f.append(Finding(WARN, "NO_PLANNING_SCENE", "move_group is running but no planning scene is being "
                         "published or served; planning requests will fail or see an empty world"))

    for ctrl, (t, err) in s.track.items():
        if now - t > c.stale:
            continue
        if err > c.track_err:
            t0 = s.track_bad_since.setdefault(ctrl, now)
            if now - t0 >= c.track_time:
                f.append(Finding(WARN, "TRACKING_ERROR", f"{ctrl} tracking error {err:.3f} rad for {now - t0:.0f}s "
                                 "(gains too low, a collision, or the commanded path is too fast)"))
        else:
            s.track_bad_since.pop(ctrl, None)

    if s.goal_active_since is not None and now - s.goal_active_since >= c.stall_time:
        mv = _moved(s, now, c.stall_time)
        if mv is not None and mv < c.stall_dist:
            f.append(Finding(ERROR, "TRAJ_STALLED", f"a trajectory goal has been active for "
                             f"{now - s.goal_active_since:.0f}s but the joints moved {mv:.4f} rad"))

    while s.aborts and now - s.aborts[0] > c.abort_window:
        s.aborts.popleft()
    if len(s.aborts) >= c.abort_count:
        f.append(Finding(WARN, "GOAL_ABORTS", f"{len(s.aborts)} trajectory goals aborted or cancelled in "
                         f"{c.abort_window:.0f}s (check limits, collisions and controller tolerances)"))
    return f


HINTS = {
    "JOINT_STATES_NONE": "start joint_state_broadcaster (ros2 control load_controller --set-state active joint_state_broadcaster)",
    "JOINT_STATES_STALE": "check the controller manager and that the simulator is not paused",
    "JOINT_NAN": "restart the controller; lower gains or the control period if it diverged",
    "NEAR_LIMIT": "move the arm away from the joint limit before planning",
    "LIMIT_POS": "the arm is outside its URDF limits; reset the joint or widen the limit",
    "LIMIT_VEL": "slow the trajectory (velocity scaling) or raise the URDF velocity limit",
    "EFFORT_SAT": "something is blocking the joint, or the payload is too heavy",
    "NO_CONTROLLER_MANAGER": "start ros2_control_node, or pass --ns for a namespaced arm",
    "CTRL_INACTIVE": "ros2 control set_controller_state <name> active",
    "NO_ACTIVE_ARM_CTRL": "activate an arm controller (ros2 control list_controllers shows the state)",
    "NO_MOVE_GROUP": "launch move_group, or drop --moveit",
    "TRACKING_ERROR": "raise controller gains, check for collisions, or slow the path",
    "TRAJ_STALLED": "the controller accepted the goal but the joints do not move; check hardware or sim and the goal tolerances",
    "NOISY_JOINTS": "check the encoder cable and driver; if the arm is fine, raise --noise-rad or filter the joint states",
    "STREAM_LAG": "the driver or network is delaying joint states; check the CPU load, the DDS settings and the clock (use_sim_time)",
    "MOVEIT_PLAN_FAILED": "ros2 topic echo /move_action/_action/status, then check the start state, limits and collisions in the planning scene",
    "NO_PLANNING_SCENE": "check that move_group loaded its planning scene monitor (publish_planning_scene, robot_description_semantic)",
    "GOAL_ABORTS": "check joint limits, collisions and goal_time / path tolerances in the controller config",
}


def verdict(findings):
    """READY when nothing blocks motion; the second item names blocking codes and warnings."""
    errors = [x.code for x in findings if x.level == ERROR]
    warns = [x.code for x in findings if x.level == WARN]
    return (not errors), errors, warns


def verdict_line(findings):
    ok, errors, warns = verdict(findings)
    if not ok:
        return f"NOT READY - {len(errors)} blocking: {', '.join(errors)}"
    return "READY" + (f" - {len(warns)} warning(s): {', '.join(warns)}" if warns else "")


def format_findings(findings, hints=False):
    out = []
    for x in findings:
        out.append(str(x))
        if hints and x.code in HINTS:
            out.append(f"        -> {HINTS[x.code]}")
    return "\n".join(out) if out else "OK    arm looks healthy"
