#!/usr/bin/env python3
"""armdoc: attach to a running ros2_control / MoveIt 2 arm and say why it is not moving. Read-only."""
import argparse
import json
import sys
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from armdoc.record import Recorder
from armdoc.core import (Config, State, evaluate, feed_joints, format_findings, is_arm_controller, parse_limits,
                         verdict_line)

EXECUTING, ABORTED, CANCELED = 2, 6, 5


class ArmDoc(Node):
    """Parameters (ros2 launch / --params-file) override the matching CLI flags: ns, moveit, stale, grace,
    track_err, stall_time, noise_rad, lag_max, period, hints, diagnostics."""

    def __init__(self, a):
        super().__init__("armdoc")
        d = self.declare_parameter
        p = lambda name, default: d(name, default).value
        self.ns = str(p("ns", a.ns)).rstrip("/")
        self.moveit = bool(p("moveit", a.moveit))
        self.period = float(p("period", a.period))
        self.hints = bool(p("hints", a.hints)) or a.ready is not None
        self.diagnostics = bool(p("diagnostics", a.diagnostics))
        grace = float(p("grace", a.grace))
        if a.once is not None:
            grace = min(grace, a.once * 0.6)
        self.cfg = Config(stale=float(p("stale", a.stale)), grace=grace, track_err=float(p("track_err", a.track_err)),
                          stall_time=float(p("stall_time", a.stall_time)), noise_rad=float(p("noise_rad", a.noise_rad)),
                          lag_max=float(p("lag_max", a.lag_max)))
        self.s = State(t0=self.now())
        self.seen_goals = {}
        self.baselined = set()
        self.subscribed = set()
        self.create_subscription(JointState, f"{self.ns}/joint_states", self.on_js, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, f"{self.ns}/robot_description", self.on_urdf, latched)
        self.cli = self.create_client(ListControllers, f"{self.ns}/controller_manager/list_controllers")
        self.pending = None
        if self.moveit:
            self.watch_status(f"{self.ns}/move_action/_action/status", False, moveit=True)
        self.create_timer(1.0, self.poll)

    def now(self):
        return time.monotonic()

    def on_js(self, m):
        lag = None
        st = m.header.stamp
        if st.sec or st.nanosec:
            d = self.get_clock().now().nanoseconds * 1e-9 - (st.sec + st.nanosec * 1e-9)
            if abs(d) < 1e4:       # a huge gap means the stamps use another clock (sim time vs wall): no verdict
                lag = d
        feed_joints(self.s, self.now(), m.name, m.position, m.velocity, m.effort, lag=lag)

    def on_urdf(self, m):
        self.s.limits = parse_limits(m.data)

    def watch_status(self, topic, motion, moveit=False):
        if topic in self.subscribed:
            return
        self.subscribed.add(topic)
        self.create_subscription(GoalStatusArray, topic, lambda m, k=topic: self.on_status(k, m, motion, moveit), 10)

    def goal_age(self, g):
        """Seconds since the goal was accepted, from its stamp; None when unusable (no stamp, or another clock)."""
        st = g.goal_info.stamp
        if not (st.sec or st.nanosec):
            return None
        age = self.get_clock().now().nanoseconds * 1e-9 - (st.sec + st.nanosec * 1e-9)
        return age if abs(age) < 1e4 else None

    def on_status(self, key, m, motion, moveit):
        now = self.now()
        first = key not in self.baselined
        self.baselined.add(key)
        active = False
        for g in m.status_list:
            gid = bytes(g.goal_info.goal_id.uuid)
            if g.status == EXECUTING:
                active = active or motion
            changed = self.seen_goals.get(gid) != g.status
            # A status list carries recent history. In the first list we saw, a goal that already finished only counts
            # when its stamp says it is brand new (a fast failure); later lists only count real transitions.
            fresh = not first
            if first and changed:
                age = self.goal_age(g)
                fresh = age is not None and -1.0 <= age <= 5.0
            if changed and fresh:
                if moveit and g.status == ABORTED:
                    self.s.moveit_aborts.append(now)
                elif not moveit and g.status in (ABORTED, CANCELED):
                    self.s.aborts.append(now)
            self.seen_goals[gid] = g.status
        if motion:
            if active and self.s.goal_active_since is None:
                self.s.goal_active_since = now
            elif not active:
                self.s.goal_active_since = None

    def on_state(self, name, m):
        err = max((abs(e) for e in m.error.positions), default=0.0)
        self.s.track[name] = (self.now(), err)

    def poll(self):
        now = self.now()
        if self.pending is not None and self.pending.done():
            try:
                res = self.pending.result()
                self.s.controllers = [(c.name, c.type, c.state) for c in res.controller]
                self.s.cm_missing = False
                claimed = {i.rsplit("/", 1)[0] for c in res.controller if is_arm_controller(c.name, c.type)
                           for i in c.claimed_interfaces}
                self.s.arm_joints = claimed or None
                for name, typ, _ in self.s.controllers:
                    if "joint_trajectory" in typ and name not in self.subscribed:
                        self.subscribed.add(name)
                        self.create_subscription(JointTrajectoryControllerState, f"{self.ns}/{name}/state",
                                                 lambda m, n=name: self.on_state(n, m), 10)
                        self.watch_status(f"{self.ns}/{name}/follow_joint_trajectory/_action/status", True)
            except Exception:
                self.s.cm_missing = True
            self.pending = None
        if self.pending is None:
            if self.cli.service_is_ready():
                self.pending = self.cli.call_async(ListControllers.Request())
            else:
                self.s.cm_missing = True
        if self.moveit:
            self.s.moveit_nodes = any(n == "move_group" for n, _ in self.get_node_names_and_namespaces())
            if self.s.moveit_nodes:
                served = {n for n, _ in self.get_service_names_and_types()}
                self.s.planning_scene = (bool(self.get_publishers_info_by_topic(f"{self.ns}/monitored_planning_scene"))
                                         or f"{self.ns}/get_planning_scene" in served)

    def enable_diagnostics(self):
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        self._dm = (DiagnosticArray, DiagnosticStatus, KeyValue)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

    def publish_diagnostics(self, findings):
        DiagnosticArray, DiagnosticStatus, KeyValue = self._dm
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for x in findings or [None]:
            st = DiagnosticStatus(name=f"armdoc: {x.code if x else 'arm'}", hardware_id=self.ns or "arm")
            if x is None:
                st.level, st.message = DiagnosticStatus.OK, "arm looks healthy"
            else:
                st.level = DiagnosticStatus.ERROR if x.level == "ERROR" else DiagnosticStatus.WARN
                st.message = x.msg
            arr.status.append(st)
        self.diag_pub.publish(arr)

    def report(self):
        return evaluate(self.s, self.now(), self.cfg)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == "report":
        from armdoc.report import main as report_main
        return report_main(argv[1:])
    ros_argv = list(argv)                                   # kept whole for rclpy (--ros-args, --params-file, remaps)
    argv = remove_ros_args(args=["armdoc"] + ros_argv)[1:]  # argparse only sees armdoc's own flags
    ap = argparse.ArgumentParser(prog="armdoc", description="Why isn't my arm moving? Read-only diagnosis of a "
                                 "ros2_control / MoveIt 2 arm.")
    ap.add_argument("--ns", default="", help="namespace of the arm (e.g. /ur3)")
    ap.add_argument("--moveit", action="store_true", help="also require a running move_group")
    ap.add_argument("--once", type=float, metavar="SECONDS", help="watch this long, print one report, exit 1 on ERROR")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--hints", action="store_true", help="print a suggested next step under each finding")
    ap.add_argument("--ready", nargs="?", type=float, const=10.0, default=None, metavar="SECS",
                    help="pre-flight: watch SECS (default 10), print hints and a READY / NOT READY verdict; "
                         "exit 1 if not ready")
    ap.add_argument("--diagnostics", action="store_true",
                    help="also publish findings to /diagnostics (the only thing armdoc ever publishes; off by default)")
    ap.add_argument("--record", metavar="FILE.jsonl", help="log timestamped joint positions and findings to a JSONL file; "
                    "turn it into a timeline with `armdoc report FILE.jsonl --html report.html`")
    ap.add_argument("--period", type=float, default=2.0)
    ap.add_argument("--grace", type=float, default=20.0)
    ap.add_argument("--stale", type=float, default=2.0)
    ap.add_argument("--track-err", type=float, default=0.05, help="tracking error (rad) that warns")
    ap.add_argument("--stall-time", type=float, default=5.0)
    ap.add_argument("--noise-rad", type=float, default=Config.noise_rad,
                    help="estimated position noise (rad) that raises NOISY_JOINTS")
    ap.add_argument("--lag-max", type=float, default=Config.lag_max,
                    help="seconds between a joint_states stamp and its arrival that raise STREAM_LAG "
                         "(needs use_sim_time:=true when the stamps are sim time)")
    a = ap.parse_args(argv)
    if a.ready is not None:
        a.once = a.ready
    rclpy.init(args=["armdoc"] + ros_argv)
    node = ArmDoc(a)
    if node.diagnostics:
        node.enable_diagnostics()
    start = last = time.monotonic()
    code = 0
    rec = Recorder(a.record, a.ns) if a.record else None
    last_eval = -1e9
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if rec:
                t = now - start
                rec.joints(t, {n: v[0] for n, v in node.s.joints.items()} if node.s.joint_t is not None
                           and now - node.s.joint_t <= node.cfg.stale else None)
                if now - last_eval >= 1.0:       # evaluate once a second for the timeline, whatever --period is
                    last_eval = now
                    if not rec.has_limits and node.s.limits:
                        rec.set_limits(node.s.limits)
                    rec.findings(t, node.report(), node.s.controllers)
            if a.once is not None:
                if now - start < a.once:
                    continue
            elif now - last < node.period:
                continue
            last = now
            f = node.report()
            if a.json:
                print(json.dumps([x.__dict__ for x in f]), flush=True)
            else:
                print(format_findings(f, node.hints), flush=True)
                if a.ready is not None:
                    print(verdict_line(f), flush=True)
            if node.diagnostics:
                node.publish_diagnostics(f)
            if a.once is not None:
                code = 1 if any(x.level == "ERROR" for x in f) else 0
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    if rec:
        rec.close()
    try:                                       # a second Ctrl-C (a terminal sends it to the whole launch group) is fine
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException, Exception):
        pass
    return code


if __name__ == "__main__":
    sys.exit(main())
