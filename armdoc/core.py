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


def feed_joints(s, now, names, pos, vel, eff):
    s.joint_t = now
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


def is_arm_controller(name, typ):
    """Controllers that drive arm joints (not broadcasters, not grippers)."""
    if "broadcaster" in typ or "gripper" in name.lower() or "gripper" in typ.lower():
        return False
    return any(k in typ for k in ("trajectory", "effort", "position", "velocity", "impedance", "admittance"))


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
