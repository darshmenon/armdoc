#!/usr/bin/env python3
"""Fake ros2_control arm graph for testing armdoc without a simulator.

  fake_arm.py            healthy arm
  fake_arm.py --freeze   joint_states stop after 3 s
  fake_arm.py --inactive arm controller is reported inactive
  fake_arm.py --stall    a trajectory goal is "executing" but the joints never move
"""
import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

URDF = '<robot name="r"><link name="a"/><link name="b"/>' + "".join(
    f'<joint name="j{i}" type="revolute"><parent link="a"/><child link="b"/>'
    f'<limit lower="-3" upper="3" velocity="3" effort="50"/></joint>' for i in range(3)) + "</robot>"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--freeze", action="store_true")
    ap.add_argument("--inactive", action="store_true")
    ap.add_argument("--stall", action="store_true")
    a = ap.parse_args()
    rclpy.init()
    n = Node("fake_arm")
    t0 = time.monotonic()
    js = n.create_publisher(JointState, "/joint_states", 10)
    n.create_publisher(String, "/robot_description", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)).publish(String(data=URDF))
    st = n.create_publisher(GoalStatusArray, "/arm_controller/follow_joint_trajectory/_action/status", 10)

    def cm(req, res):
        c = ControllerState(name="arm_controller", type="joint_trajectory_controller/JointTrajectoryController",
                            state="inactive" if a.inactive else "active")
        res.controller = [c]
        return res
    n.create_service(ListControllers, "/controller_manager/list_controllers", cm)

    def tick():
        t = time.monotonic() - t0
        if a.freeze and t > 3:
            return
        m = JointState(name=["j0", "j1", "j2"])
        m.header.stamp = n.get_clock().now().to_msg()
        m.position = [0.0] * 3 if a.stall else [math.sin(t + i) for i in range(3)]
        m.velocity = [0.0] * 3
        m.effort = [0.0] * 3
        js.publish(m)
        if a.stall:
            g = GoalStatusArray()
            s = GoalStatus(status=GoalStatus.STATUS_EXECUTING)
            g.status_list = [s]
            st.publish(g)
    n.create_timer(0.02, tick)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass


if __name__ == "__main__":
    main()
