#!/usr/bin/env python3
"""Fake arm + gripper for armdoc: joints j0-j2 (arm) mid-range, `finger` resting at its closed limit (0.0, range 0..0.04),
a robot_description with limits, and a controller manager that reports claimed interfaces.

  fake_gripper_arm.py                       arm_controller active the whole time
  fake_gripper_arm.py --deactivate-after S  arm_controller goes inactive (claims nothing) after S seconds

Used to check that the arm joints are learned from the controller manager, so a gripper finger at its closed limit is
never judged as an arm joint (and that deactivating the arm controller does not widen the checks)."""
import argparse
import math
import time

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

URDF = """<robot name="r">
<joint name="j0" type="revolute"><limit lower="-3" upper="3" velocity="2" effort="50"/></joint>
<joint name="j1" type="revolute"><limit lower="-3" upper="3" velocity="2" effort="50"/></joint>
<joint name="j2" type="revolute"><limit lower="-3" upper="3" velocity="2" effort="50"/></joint>
<joint name="finger" type="prismatic"><limit lower="0.0" upper="0.04" velocity="0.2" effort="20"/></joint>
</robot>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--deactivate-after", type=float, default=None)
    a, ros = ap.parse_known_args()
    rclpy.init()
    n = Node("fake_gripper_arm")
    t0 = time.monotonic()
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    desc = n.create_publisher(String, "/robot_description", latched)
    desc.publish(String(data=URDF))
    js = n.create_publisher(JointState, "/joint_states", 10)

    def active():
        return a.deactivate_after is None or time.monotonic() - t0 < a.deactivate_after

    def lst(req, res):
        arm = ControllerState(name="arm_controller", type="joint_trajectory_controller/JointTrajectoryController",
                              state="active" if active() else "inactive",
                              claimed_interfaces=[f"j{i}/position" for i in range(3)] if active() else [])
        grip = ControllerState(name="gripper_controller", type="position_controllers/GripperActionController",
                               state="active", claimed_interfaces=["finger/position"])
        jsb = ControllerState(name="joint_state_broadcaster", type="joint_state_broadcaster/JointStateBroadcaster",
                              state="active", claimed_interfaces=[])
        res.controller = [arm, grip, jsb]
        return res

    n.create_service(ListControllers, "/controller_manager/list_controllers", lst)

    count = [0]

    def tick():
        count[0] += 1
        t = count[0] * 0.02            # positions follow the sample count, so a late timer tick is not seen as jitter
        m = JointState(name=["j0", "j1", "j2", "finger"])
        m.header.stamp = n.get_clock().now().to_msg()
        m.position = [0.5 * math.sin(t), 0.4 * math.cos(t), 0.3, 0.0]
        m.velocity = [0.0] * 4
        m.effort = [0.0] * 4
        js.publish(m)

    n.create_timer(0.02, tick)
    rclpy.spin(n)


if __name__ == "__main__":
    main()
