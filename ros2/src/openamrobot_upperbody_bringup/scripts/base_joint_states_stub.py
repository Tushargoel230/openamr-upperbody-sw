#!/usr/bin/env python3
"""Publish the mobile base's wheel joints at rest on /joint_states, for the MOCK stack only.

The base is not under ros2_control (ADR-003), so nothing in the mock bringup reports `left_wheel_joint` and `right_wheel_joint`, and
MoveIt, whose robot model includes them, warns every second that "the complete state of the robot is not yet known". The base does not
move during manipulation (AD-30), so reporting the wheels at zero is the true state of the thing being modelled, not an invention.
In Gazebo the real wheel states come from the simulator (merged by sim.launch.py), and this node is not started.

    ros2 run openamrobot_upperbody_bringup base_joint_states_stub.py [--ros-args -p rate:=10.0]
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

WHEELS = ("left_wheel_joint", "right_wheel_joint")


class BaseJointStatesStub(Node):
    def __init__(self):
        super().__init__("base_joint_states_stub")
        self.declare_parameter("rate", 10.0)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.publish)

    def publish(self):
        msg = JointState(name=list(WHEELS), position=[0.0] * len(WHEELS), velocity=[0.0] * len(WHEELS))
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BaseJointStatesStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
