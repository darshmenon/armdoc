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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from armdoc.core import Config, State, evaluate, feed_joints, is_arm_controller, parse_limits

EXECUTING, ABORTED, CANCELED = 2, 6, 5


class ArmDoc(Node):
    def __init__(self, ns, moveit, cfg):
        super().__init__("armdoc")
        self.cfg, self.ns, self.moveit = cfg, ns.rstrip("/"), moveit
        self.s = State(t0=self.now())
        self.seen_goals = {}
        self.subscribed = set()
        self.create_subscription(JointState, f"{self.ns}/joint_states", self.on_js, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, f"{self.ns}/robot_description", self.on_urdf, latched)
        self.cli = self.create_client(ListControllers, f"{self.ns}/controller_manager/list_controllers")
        self.pending = None
        if moveit:
            self.watch_status(f"{self.ns}/move_action/_action/status", False)
        self.create_timer(1.0, self.poll)

    def now(self):
        return time.monotonic()

    def on_js(self, m):
        feed_joints(self.s, self.now(), m.name, m.position, m.velocity, m.effort)

    def on_urdf(self, m):
        self.s.limits = parse_limits(m.data)

    def watch_status(self, topic, motion):
        if topic in self.subscribed:
            return
        self.subscribed.add(topic)
        self.create_subscription(GoalStatusArray, topic, lambda m, k=topic: self.on_status(k, m, motion), 10)

    def on_status(self, key, m, motion):
        now = self.now()
        active = False
        for g in m.status_list:
            gid = bytes(g.goal_info.goal_id.uuid)
            if g.status == EXECUTING:
                active = active or motion
            if g.status in (ABORTED, CANCELED) and self.seen_goals.get(gid) != g.status:
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

    def report(self):
        return evaluate(self.s, self.now(), self.cfg)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="armdoc", description="Why isn't my arm moving? Read-only diagnosis of a "
                                 "ros2_control / MoveIt 2 arm.")
    ap.add_argument("--ns", default="", help="namespace of the arm (e.g. /ur3)")
    ap.add_argument("--moveit", action="store_true", help="also require a running move_group")
    ap.add_argument("--once", type=float, metavar="SECONDS", help="watch this long, print one report, exit 1 on ERROR")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--period", type=float, default=2.0)
    ap.add_argument("--grace", type=float, default=20.0)
    ap.add_argument("--stale", type=float, default=2.0)
    ap.add_argument("--track-err", type=float, default=0.05, help="tracking error (rad) that warns")
    ap.add_argument("--stall-time", type=float, default=5.0)
    a = ap.parse_args(argv)
    cfg = Config(stale=a.stale, grace=a.grace if not a.once else min(a.grace, a.once * 0.6),
                 track_err=a.track_err, stall_time=a.stall_time)
    rclpy.init()
    node = ArmDoc(a.ns, a.moveit, cfg)
    start = last = time.monotonic()
    code = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if a.once is not None:
                if now - start < a.once:
                    continue
            elif now - last < a.period:
                continue
            last = now
            f = node.report()
            if a.json:
                print(json.dumps([x.__dict__ for x in f]), flush=True)
            elif f:
                print("\n".join(str(x) for x in f), flush=True)
            else:
                print("OK    arm looks healthy", flush=True)
            if a.once is not None:
                code = 1 if any(x.level == "ERROR" for x in f) else 0
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
